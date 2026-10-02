// SPIKE (docs/architecture/flows-spike.md), not engine code: included into
// solver.rs's `lanes` by the BUILD's `flows_spike_srcs`, so it reaches the
// lanes' private kernels. `solve_across` taken apart into a pipeline's
// stages, its staged run replaced by the generic `flows::Colored::passes`:
//
// - `prepare_flow`: the bodies as states, the contacts colored (generically, by
//   `flows::Coloring::greedy`, Box2D v3's rule) and packed into lanes, and
//   the batches filled with each contact's masses and points (`head`,
//   `setup` and the run's first stage);
// - `passes_flow`: the substeps, every pass a `Stage` of `Colored::passes`;
// - `finish_flow`: impulses back into the contacts and points, states back into
//   the bodies (`solve_across`'s tail).
//
// Bit for bit `solve_across`, which is `solve` at any thread count:
// `flows_spike` and `flows_spike_test` assert it. The names above are this
// file's alone; it uses solver.rs's only through what `lanes` already has.
// The imports `lanes` makes (`super::*`, `Range`, `Mutex`, the atomics)
// are this file's too, since it's included there.

/// A pass of the substeps, as `Colored::passes` hands it to the kernels.
#[derive(Clone, Copy, Debug)]
pub enum Step {
    Gravity,
    Warm,
    /// The pushing pass; whether it's the substep's last.
    Push(bool),
    Move,
    /// A relaxing pass: whether it's the substep's first and its last.
    Relax(bool, bool),
    Bounce,
}

/// How the passes reach their kernels: what the spike measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Shape {
    /// A kernel over a batch of `N` edges, monomorphized into the primitive.
    Batch,
    /// The same kernel behind `&dyn Fn`: a call through a pointer per item.
    Dyn,
    /// A kernel per edge, handed its two states (`Colored::edges`): `N` is 1.
    Edge,
}

/// The step's contacts as the passes solve them: a flow's payload, its
/// allocations kept from step to step.
pub struct Prepared<const N: usize> {
    hd: Head,
    pub atoms: Vec<Atom>,
    gravity: Vec<Vec2>,
    /// Each spinning body's angle over the step, as bits: the `Move` stage
    /// writes them from any thread.
    angle: Vec<AtomicU32>,
    pub items: Vec<Batch<N>>,
    lanes: Vec<[Lane; N]>,
    pub layout: flows::Colored,
    pub coloring: flows::Coloring,
    place: Vec<Option<(u32, u32)>>,
    owner: Vec<u32>,
    kept: Vec<(usize, [(f32, f32); 2])>,
    solved: Vec<usize>,
    masks: Vec<u64>,
    moves: Vec<bool>,
    /// Whether the passes may be shared between threads (`shareable`).
    pub shared: bool,
}

impl<const N: usize> Default for Prepared<N> {
    fn default() -> Self {
        let soft = Softness { rate: 0.0, mass: 0.0, impulse: 0.0 };
        Prepared {
            hd: Head {
                s: Vec::new(),
                inertia: Vec::new(),
                groups: Vec::new(),
                n_groups: 0,
                count: Vec::new(),
                first: Vec::new(),
                overflow: 0,
                batches: 0,
                nowhere: 0,
                h: 0.0,
                inv_h: 0.0,
                share: 0.0,
                warm: 0.0,
                soft: (soft, soft),
            },
            atoms: Vec::new(),
            gravity: Vec::new(),
            angle: Vec::new(),
            items: Vec::new(),
            lanes: Vec::new(),
            layout: flows::Colored::default(),
            coloring: flows::Coloring::default(),
            place: Vec::new(),
            owner: Vec::new(),
            kept: Vec::new(),
            solved: Vec::new(),
            masks: Vec::new(),
            moves: Vec::new(),
            shared: false,
        }
    }
}

impl<const N: usize> flows::Recycle for Prepared<N> {
    fn recycle(&mut self) {
        // Every vector is cleared or overwritten by `prepare_flow`, so there's
        // nothing to do but keep them.
    }
}

/// Whether the step is solved in lanes, colored: what `solve_across` shares
/// between threads, and the only path this spike ports.
pub fn ported(params: &Params, spinning: &[Spinning], points: &[Points]) -> bool {
    !(points.is_empty() && spinning.is_empty())
        && matches!(params.wide, Wide::Colored(_))
        && params.separation == Separation::Turned
        && !params.block
}

/// `head`, `setup`'s placing and the run's first stage, into `p`: the
/// pipeline's `prepare`. Contacts that aren't solved are finished here, as
/// `solve_across` finishes them before its run.
pub fn prepare_flow<const N: usize>(
    p: &mut Prepared<N>,
    params: &Params,
    (bodies, spinning): (&[SolverBody], &[Spinning]),
    contacts: &mut [Constraint],
    points: &[Points],
    dt: f32,
    workers: &engine_ecs::Workers,
) {
    assert!(ported(params, spinning, points), "the spike ports the colored lanes only");
    let substeps = params.substeps;
    let h = dt / substeps as f32;
    let inv_h = 1.0 / h;
    let share = 1.0 / substeps as f32;
    let hd = &mut p.hd;
    (hd.h, hd.inv_h, hd.share) = (h, inv_h, share);
    hd.warm = if params.warm { share } else { 0.0 };
    hd.soft = (
        Softness::new(params.stiffness * inv_h, DAMPING_RATIO, h),
        Softness::new(params.static_stiffness * inv_h, DAMPING_RATIO, h),
    );
    hd.nowhere = bodies.len() as u32;
    hd.s.clear();
    hd.s.extend(bodies.iter().map(|b| State { v: b.v, ..State::default() }));
    hd.s.push(State::default());
    hd.inertia.clear();
    hd.inertia.resize(bodies.len(), 0.0);
    for sp in spinning {
        hd.s[sp.body as usize].w = sp.w;
        hd.inertia[sp.body as usize] = sp.inv_inertia;
    }
    p.moves.clear();
    p.moves.extend(bodies.iter().zip(hd.inertia.iter()).map(|(b, i)| b.inv_mass > 0.0 || *i > 0.0));

    // `group`'s coloring, as the generic primitive offers it: Box2D's rule
    // keeps an edge with an end that doesn't move out of color 0.
    p.coloring.greedy(contacts.len(), |i| (contacts[i].a, contacts[i].b), &p.moves, true, &mut p.masks);
    p.layout = p.coloring.pack(N, &mut p.place);
    let items = p.layout.items();
    p.lanes.clear();
    p.lanes.resize(items, [Lane::NONE; N]);
    p.owner.clear();
    p.owner.resize(points.len(), NONE);
    p.kept.clear();
    p.solved.clear();
    for (i, c) in contacts.iter_mut().enumerate() {
        if c.points > 0 {
            p.owner[c.points as usize - 1] = i as u32;
        }
        match p.place[i] {
            Some((item, l)) => p.lanes[item as usize][l as usize].contact = i as u32,
            None => {
                let (jn, jt) = (c.jn, c.jt);
                (c.jn, c.jt) = (0.0, 0.0);
                let (pts, at, n_points, speed) = start(params, c, (jn, jt), bodies, &p.hd, points, dt);
                c.speed = speed;
                if at != NONE {
                    p.solved.push(at as usize);
                }
                unsolved(params, c, (&pts, at, n_points), &mut p.kept);
            }
        }
    }
    p.gravity.clear();
    p.gravity.extend(bodies.iter().map(|b| b.gravity));
    p.atoms.clear();
    p.atoms.extend(p.hd.s.iter().enumerate().map(|(i, st)| {
        let mut st = *st;
        if let Some(g) = p.gravity.get(i) {
            st.v -= *g;
        }
        Atom::new(&st)
    }));
    p.angle.clear();
    p.angle.extend(spinning.iter().map(|_| AtomicU32::new(0.0f32.to_bits())));
    p.shared = shareable(bodies, spinning);

    // The batches, filled across the threads: `run_across`'s first stage.
    let filling = std::time::Instant::now();
    p.items.clear();
    p.items.resize(items, Batch::empty(p.hd.nowhere));
    let hd = &p.hd;
    let contacts = &*contacts;
    let mut pairs: Vec<(&mut Batch<N>, &mut [Lane; N])> = p.items.iter_mut().zip(p.lanes.iter_mut()).collect();
    flows::par_for_each_mut(workers, &mut pairs, 16, |_, (o, ln)| {
        for (l, lane) in ln.iter_mut().enumerate() {
            if lane.contact == NONE {
                continue;
            }
            let c = &contacts[lane.contact as usize];
            let (pts, at, n_points, speed) = start(params, c, (c.jn, c.jt), bodies, hd, points, dt);
            put(o, l, c, bodies, hd, (&pts, at, n_points));
            (lane.at, lane.count, lane.speed) = (at, n_points as u8, speed);
        }
    });
    FILLED.store(filling.elapsed().as_nanos() as u64, Ordering::Relaxed);
}

/// How long the last `prepare_flow` took to fill its batches, in ns: the
/// part of it that `run_across` makes its first stage.
pub static FILLED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// The substeps as `run` makes its passes, for `Colored::passes`.
fn program(params: &Params, states: usize, spins: usize) -> Vec<flows::Stage<Step>> {
    use flows::Stage::{Each, Items};
    let mut out = Vec::new();
    for _ in 0..params.substeps {
        out.push(Each(Step::Gravity, states));
        out.push(Items(Step::Warm));
        out.push(Items(Step::Push(params.relax == 0)));
        out.push(Each(Step::Move, states + spins));
        for r in 0..params.relax {
            out.push(Items(Step::Relax(r == 0, r + 1 == params.relax)));
        }
    }
    out.push(Items(Step::Bounce));
    out
}

/// A batch's pass, over any way of reaching the bodies.
#[inline(always)]
fn kernel<const N: usize>(params: &Params, inv_h: f32, k: Step, o: &mut Batch<N>, s: &mut (impl Bodies + ?Sized)) {
    match k {
        Step::Warm => warm_start(o, s),
        Step::Push(last) => pass::<N, true, COMPUTE>(o, s, inv_h, last),
        Step::Relax(true, last) => pass::<N, false, STORE>(o, s, inv_h, last),
        Step::Relax(false, last) => pass::<N, false, LOAD>(o, s, inv_h, last),
        Step::Bounce => {
            if o.restitution.0.iter().any(|e| *e != 0.0) {
                restitute(o, s, params.bounce);
            }
        }
        Step::Gravity | Step::Move => unreachable!("a stage over states"),
    }
}

/// One edge's two bodies, for a kernel handed them alone (`Shape::Edge`):
/// the batch's indices say which is which.
struct Ends<'a> {
    a: u32,
    at_a: &'a Atom,
    at_b: &'a Atom,
}

impl Ends<'_> {
    #[inline(always)]
    fn of(&self, i: usize) -> &Atom {
        if i as u32 == self.a { self.at_a } else { self.at_b }
    }
}

impl Bodies for Ends<'_> {
    #[inline(always)]
    fn load(&self, i: usize) -> State {
        self.of(i).state()
    }

    #[inline(always)]
    fn load_v(&self, i: usize) -> (Vec2, f32) {
        let a = self.of(i);
        (Vec2::new(a.get(0), a.get(1)), a.get(2))
    }

    #[inline(always)]
    fn store_v(&mut self, i: usize, v: Vec2, w: f32) {
        let a = self.of(i);
        a.put(0, v.x);
        a.put(1, v.y);
        a.put(2, w);
    }
}

/// The substeps and restitution: `run`, through `Colored::passes`.
pub fn passes_flow<const N: usize>(p: &mut Prepared<N>, params: &Params, spinning: &[Spinning], workers: &engine_ecs::Workers, shape: Shape) {
    let one = engine_ecs::Workers::default();
    // Not shareable: the passes on one thread, as `solve_across` falls back.
    let workers = if p.shared { workers } else { &one };
    let (h, inv_h, share) = (p.hd.h, p.hd.inv_h, p.hd.share);
    let nb = p.atoms.len();
    let program = program(params, nb, spinning.len());
    let (gravity, angle) = (&p.gravity, &p.angle);
    let each = |k: Step, r: Range<usize>, atoms: &[Atom]| match k {
        Step::Gravity => {
            let g = &gravity[r.start.min(gravity.len())..r.end.min(gravity.len())];
            for (a, g) in atoms[r.clone()].iter().zip(g) {
                let mut st = a.state();
                st.v += *g * share;
                a.put(0, st.v.x);
                a.put(1, st.v.y);
            }
        }
        Step::Move => {
            for a in atoms[r.start.min(nb)..r.end.min(nb)].iter() {
                let mut st = a.state();
                st.moved += st.v * h;
                a.put(3, st.moved.x);
                a.put(4, st.moved.y);
            }
            for j in r.start.max(nb) - nb..r.end.max(nb) - nb {
                let (sp, angle) = (&spinning[j], &angle[j]);
                let a = &atoms[sp.body as usize];
                let b = a.state();
                let turned_by = f32::from_bits(angle.load(Ordering::Relaxed)) + h * b.w;
                angle.store(turned_by.to_bits(), Ordering::Relaxed);
                let turned = match params.integrate {
                    Integrate::Rotation => b.turned.integrate(h * b.w),
                    Integrate::Angle => Rot::from_angle(turned_by),
                };
                a.put(5, turned.c);
                a.put(6, turned.s);
            }
        }
        _ => unreachable!("a stage over items"),
    };
    let atoms = &p.atoms[..];
    match shape {
        Shape::Batch => p.layout.passes(workers, &mut p.items, atoms, &program, |k, o, s| kernel(params, inv_h, k, o, &mut Shared(s)), each),
        Shape::Dyn => {
            // Hidden from the optimizer, so the call stays one through a
            // pointer, as a kernel compiled apart from the primitive's loop
            // would be.
            let k = |k, o: &mut Batch<N>, s: &[Atom]| kernel(params, inv_h, k, o, &mut Shared(s));
            let f: &(dyn Fn(Step, &mut Batch<N>, &[Atom]) + Sync) = std::hint::black_box(&k);
            p.layout.passes(workers, &mut p.items, atoms, &program, f, each)
        }
        Shape::Edge => {
            assert_eq!(N, 1, "a kernel per edge is a batch of one");
            p.layout.edges(
                workers,
                &mut p.items,
                atoms,
                &program,
                |o| (o.a[0], o.b[0]),
                |k, o, at_a, at_b| kernel(params, inv_h, k, o, &mut Ends { a: o.a[0], at_a, at_b }),
                each,
            )
        }
    }
}

/// What `clear` and `finish` write, across the threads by what each writes:
/// `solve_across`'s tail, and the contacts that weren't solved.
pub fn finish_flow<const N: usize>(
    p: &Prepared<N>,
    params: &Params,
    (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
    contacts: &mut [Constraint],
    points: &mut [Points],
    workers: &engine_ecs::Workers,
) {
    let tasks = workers.threads();
    let mut work = Vec::with_capacity(tasks);
    let (mut cs, mut ps, mut bs, mut ss) = (&mut *contacts, &mut *points, &mut *bodies, &mut *spinning);
    let (nc, np, nb, ns) = (cs.len(), ps.len(), p.atoms.len() - 1, ss.len());
    for k in 0..tasks {
        let cut = |n: usize| n * (k + 1) / tasks - n * k / tasks;
        let at = |n: usize| n * k / tasks;
        let (c, rest) = std::mem::take(&mut cs).split_at_mut(cut(nc));
        cs = rest;
        let (q, rest) = std::mem::take(&mut ps).split_at_mut(cut(np));
        ps = rest;
        let (b, rest) = std::mem::take(&mut bs).split_at_mut(cut(nb));
        bs = rest;
        let (s, rest) = std::mem::take(&mut ss).split_at_mut(cut(ns));
        ss = rest;
        work.push(((at(nc), at(np), at(nb), at(ns)), c, q, b, s));
    }
    let (substeps, sh) = (params.substeps, Shared(&p.atoms));
    let (items, lanes, place, owner, angle) = (&p.items, &p.lanes, &p.place, &p.owner, &p.angle);
    flows::par_for_each_mut(workers, &mut work, 1, |_, ((c0, p0, b0, s0), cs, ps, bs, ss)| {
        for (i, c) in (*c0..).zip(cs.iter_mut()) {
            if let Some((item, l)) = place[i] {
                let lane = &lanes[item as usize][l as usize];
                (c.jn, c.jt) = impulses(&items[item as usize], l as usize, lane);
                c.speed = lane.speed;
            }
        }
        for (at, to) in (*p0..).zip(ps.iter_mut()) {
            to.solved = false;
            for q in to.point.iter_mut() {
                (q.jn, q.jt) = (0.0, 0.0);
            }
            let Some(Some((item, l))) = place.get(owner[at] as usize) else { continue };
            let lane = &lanes[*item as usize][*l as usize];
            if lane.at as usize == at {
                to.solved = true;
                carried(params.carry, substeps, &items[*item as usize], *l as usize, lane.count as usize, to);
            }
        }
        for (i, b) in (*b0..).zip(bs.iter_mut()) {
            let st = sh.load(i);
            (b.v, b.moved) = (st.v, st.moved);
        }
        for (j, sp) in (*s0..).zip(ss.iter_mut()) {
            let st = sh.load(sp.body as usize);
            (sp.w, sp.turned, sp.angle) = (st.w, st.turned, f32::from_bits(angle[j].load(Ordering::Relaxed)));
        }
    });
    for &at in &p.solved {
        points[at].solved = true;
    }
    for (at, j) in &p.kept {
        for (q, j) in points[*at].point.iter_mut().zip(j) {
            (q.jn, q.jt) = *j;
        }
    }
}

/// The three stages in turn: `solve_across`, as a pipeline would run it.
pub fn solve_flow<const N: usize>(
    p: &mut Prepared<N>,
    params: &Params,
    (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
    contacts: &mut [Constraint],
    points: &mut [Points],
    dt: f32,
    workers: &engine_ecs::Workers,
    shape: Shape,
) {
    prepare_flow(p, params, (bodies, spinning), contacts, points, dt, workers);
    passes_flow(p, params, spinning, workers, shape);
    finish_flow(p, params, (bodies, spinning), contacts, points, workers);
}

/// `group`'s coloring of these contacts, for checking the generic one
/// against: each contact's group, and how many.
pub fn colors_as_built(params: &Params, bodies: &[SolverBody], spinning: &[Spinning], contacts: &[Constraint]) -> (Vec<u32>, usize) {
    let mut inertia = vec![0.0f32; bodies.len()];
    for sp in spinning {
        inertia[sp.body as usize] = sp.inv_inertia;
    }
    let moves: Vec<bool> = bodies.iter().zip(inertia.iter()).map(|(b, i)| b.inv_mass > 0.0 || *i > 0.0).collect();
    group(contacts, &moves, params.wide)
}

/// `solve_across`, kept to its passes' time: the hand-tuned staged run on
/// the same input, its setup and tail timed apart. `(setup, run, tail)` in
/// µs.
pub fn solve_across_timed<const N: usize>(
    params: &Params,
    (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
    contacts: &mut [Constraint],
    points: &mut [Points],
    dt: f32,
    gang: &dyn Gang,
) -> (f64, f64, f64) {
    let t = std::time::Instant::now();
    let hd = head::<N>(params, (bodies, spinning), contacts, dt);
    let (mut filled, mut overflowed) = (vec![0usize; hd.n_groups], 0);
    let mut lanes: Vec<[Lane; N]> = vec![[Lane::NONE; N]; hd.batches];
    let (mut kept, mut solved) = (Vec::new(), Vec::new());
    let mut slot = vec![(NONE, 0u32); contacts.len()];
    let mut owner = vec![NONE; points.len()];
    for (i, c) in contacts.iter_mut().enumerate() {
        if c.points > 0 {
            owner[c.points as usize - 1] = i as u32;
        }
        let k = hd.groups[i];
        if k == UNSOLVED {
            let (jn, jt) = (c.jn, c.jt);
            (c.jn, c.jt) = (0.0, 0.0);
            let (pts, at, n_points, speed) = start(params, c, (jn, jt), bodies, &hd, points, dt);
            c.speed = speed;
            if at != NONE {
                solved.push(at as usize);
            }
            unsolved(params, c, (&pts, at, n_points), &mut kept);
            continue;
        }
        let (batch, l) = place::<N>(k, &hd, (&mut filled, &mut overflowed));
        lanes[batch][l].contact = i as u32;
        slot[i] = (batch as u32, l as u32);
    }
    let gravity: Vec<Vec2> = bodies.iter().map(|b| b.gravity).collect();
    let mut angle = vec![0.0f32; spinning.len()];
    let setup = t.elapsed().as_secs_f64() * 1e6;
    let t = std::time::Instant::now();
    let shared = (&*bodies, &*spinning, &*contacts, &*points);
    let (atoms, outs) = run_across::<N>(params, shared, (&hd, &gravity, dt), (&mut lanes, &mut angle), gang);
    let run = t.elapsed().as_secs_f64() * 1e6;
    let t = std::time::Instant::now();
    let flat: Vec<&Batch<N>> = outs.iter().flatten().collect();
    let tasks = gang.threads();
    let mut work = Vec::with_capacity(tasks);
    let (mut cs, mut ps, mut bs, mut ss) = (&mut *contacts, &mut *points, &mut *bodies, &mut *spinning);
    for k in 0..tasks {
        let cut = |n: usize| n * (k + 1) / tasks - n * k / tasks;
        let at = |n: usize| n * k / tasks;
        let (c, rest) = std::mem::take(&mut cs).split_at_mut(cut(slot.len()));
        cs = rest;
        let (p, rest) = std::mem::take(&mut ps).split_at_mut(cut(owner.len()));
        ps = rest;
        let (b, rest) = std::mem::take(&mut bs).split_at_mut(cut(atoms.len() - 1));
        bs = rest;
        let (s, rest) = std::mem::take(&mut ss).split_at_mut(cut(angle.len()));
        ss = rest;
        let from = (at(slot.len()), at(owner.len()), at(atoms.len() - 1), at(angle.len()));
        work.push(Mutex::new((from, c, p, b, s)));
    }
    let (substeps, sh) = (params.substeps, Shared(&atoms));
    gang.run(tasks, &|k| {
        let mut part = work[k].try_lock().expect("a task's own");
        let ((c0, p0, b0, s0), cs, ps, bs, ss) = &mut *part;
        for (i, c) in (*c0..).zip(cs.iter_mut()) {
            let (batch, l) = slot[i];
            if batch != NONE {
                let lane = &lanes[batch as usize][l as usize];
                (c.jn, c.jt) = impulses(flat[batch as usize], l as usize, lane);
                c.speed = lane.speed;
            }
        }
        for (p, to) in (*p0..).zip(ps.iter_mut()) {
            to.solved = false;
            for q in to.point.iter_mut() {
                (q.jn, q.jt) = (0.0, 0.0);
            }
            let Some(&(batch, l)) = slot.get(owner[p] as usize) else { continue };
            if batch != NONE && lanes[batch as usize][l as usize].at as usize == p {
                to.solved = true;
                let lane = &lanes[batch as usize][l as usize];
                carried(params.carry, substeps, flat[batch as usize], l as usize, lane.count as usize, to);
            }
        }
        for (i, b) in (*b0..).zip(bs.iter_mut()) {
            let st = sh.load(i);
            (b.v, b.moved) = (st.v, st.moved);
        }
        for (j, sp) in (*s0..).zip(ss.iter_mut()) {
            let st = sh.load(sp.body as usize);
            (sp.w, sp.turned, sp.angle) = (st.w, st.turned, angle[j]);
        }
    });
    drop(work);
    for at in solved {
        points[at].solved = true;
    }
    for (at, j) in kept {
        for (q, j) in points[at].point.iter_mut().zip(j) {
            (q.jn, q.jt) = j;
        }
    }
    (setup, run, t.elapsed().as_secs_f64() * 1e6)
}
