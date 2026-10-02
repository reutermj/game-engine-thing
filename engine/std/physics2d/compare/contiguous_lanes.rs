// SPIKE, not engine code: included into `solver::lanes` by the genrule
// `contiguous_spike_srcs` (BUILD.bazel), and by nothing else, so that
// `contiguous_spike.rs` can run the lanes' own kernels (`warm_start`,
// `pass`, `restitute`) over bodies kept where the world would keep them,
// rather than over the dense `State`s `setup` copies them into. Everything
// here mirrors `setup`, `run` and `finish` operation for operation, so a
// solve through it is the lanes' solve bit for bit (the spike checks).
// See docs/architecture/contiguous-columns.md.

/// A body's state where an in-place solve finds it, by an index the step
/// doesn't make: the world's columns (`v`, `w`), and scratch beside them by
/// the same index for what only the step has (`moved`, `turned`).
pub trait Place {
    fn v(&self, i: usize) -> Vec2;
    fn set_v(&mut self, i: usize, v: Vec2);
    fn w(&self, i: usize) -> f32;
    fn set_w(&mut self, i: usize, w: f32);
    fn moved(&self, i: usize) -> Vec2;
    fn set_moved(&mut self, i: usize, m: Vec2);
    fn turned(&self, i: usize) -> Rot;
    fn set_turned(&mut self, i: usize, q: Rot);
}

/// What the contacts' prepare reads of a body by the same index: what the
/// copy's `SolverBody` and `head`'s inertias hold, here derived from the
/// world's columns or read from columns that keep them.
pub trait Source {
    /// Velocity, inverse mass masked by kind, and the step's gravity.
    fn body(&self, i: usize) -> SolverBody;
    /// Turn rate and inverse inertia: both 0 for a body that doesn't turn.
    fn ang(&self, i: usize) -> (f32, f32);
}

/// A `Place` as the kernels see bodies.
struct InPlace<'a, P: ?Sized>(&'a mut P);

impl<P: Place + ?Sized> Bodies for InPlace<'_, P> {
    #[inline(always)]
    fn load(&self, i: usize) -> State {
        State { v: self.0.v(i), w: self.0.w(i), moved: self.0.moved(i), turned: self.0.turned(i) }
    }

    #[inline(always)]
    fn load_v(&self, i: usize) -> (Vec2, f32) {
        (self.0.v(i), self.0.w(i))
    }

    #[inline(always)]
    fn store_v(&mut self, i: usize, v: Vec2, w: f32) {
        self.0.set_v(i, v);
        self.0.set_w(i, w);
    }
}

/// A step's contacts, prepared to be solved in place: `Solve` without its
/// states, which are wherever the `Place` keeps them.
pub struct InPlaceSolve<const N: usize> {
    out: Vec<Batch<N>>,
    lanes: Vec<[Lane; N]>,
    /// Each body's gravity for the step, by the place's index.
    gravity: Vec<Vec2>,
    /// Batches of the overflow (first), then of each group in turn.
    overflow: usize,
    counts: Vec<usize>,
    h: f32,
    inv_h: f32,
    share: f32,
}

/// `start`, reading bodies from `src`.
#[inline(always)]
fn start_in(
    params: &Params,
    c: &Constraint,
    (jn, jt): (f32, f32),
    src: &impl Source,
    hd: &Head,
    points: &[Points],
    dt: f32,
) -> ([Point; 2], u32, usize, f32) {
    let (a, b) = (c.a as usize, c.b as usize);
    let (ba, bb) = (src.body(a), src.body(b));
    let ((wa, ia), (wb, ib)) = (src.ang(a), src.ang(b));
    let spins = |w: f32, i: f32| i > 0.0 || w != 0.0;
    let turns = c.points > 0 && (spins(wa, ia) || spins(wb, ib));
    let mut pts = [Point::default(); 2];
    if turns {
        let at = c.points as usize - 1;
        let ang = |w: f32, i: f32| Ang { w, inv_inertia: i, turned: Rot::IDENTITY, angle: 0.0 };
        let bounce = (params.closing, dt, c.restitution);
        let (t, speed) = prepare((&ba, &ang(wa, ia)), (&bb, &ang(wb, ib)), c.normal, &points[at], at, hd.warm, bounce);
        (t.p, at as u32, t.count, speed)
    } else {
        let speed = super::closing(params.closing, (&ba, &bb), c, dt);
        let k = ba.inv_mass + bb.inv_mass;
        let mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        pts[0] =
            Point { base: -c.depth, normal_mass: mass, tangent_mass: mass, jn: jn * hd.share, jt: jt * hd.share, speed, ..Point::default() };
        (pts, NONE, 1, speed)
    }
}

/// `put`, reading bodies from `src`.
#[inline(always)]
fn put_in<const N: usize>(o: &mut Batch<N>, l: usize, c: &Constraint, src: &impl Source, hd: &Head, (pts, at, n_points): (&[Point; 2], u32, usize)) {
    let (a, b) = (c.a as usize, c.b as usize);
    let (ma, mb) = (src.body(a).inv_mass, src.body(b).inv_mass);
    let (moving, fixed) = hd.soft;
    let soft = if ma == 0.0 || mb == 0.0 { fixed } else { moving };
    o.a[l] = a as u32;
    o.b[l] = b as u32;
    (o.nx.0[l], o.ny.0[l]) = (c.normal.x, c.normal.y);
    (o.ma.0[l], o.mb.0[l]) = (ma, mb);
    (o.ia.0[l], o.ib.0[l]) = (src.ang(a).1, src.ang(b).1);
    (o.friction.0[l], o.restitution.0[l]) = (c.friction, c.restitution);
    (o.rate.0[l], o.soft_mass.0[l], o.soft_impulse.0[l]) = (soft.rate, soft.mass, soft.impulse);
    (o.two[l], o.linear[l]) = (n_points > 1, at == NONE);
    for (q, p) in o.p.iter_mut().zip(pts.iter()).take(n_points) {
        (q.rax.0[l], q.ray.0[l], q.rbx.0[l], q.rby.0[l]) = (p.ra.x, p.ra.y, p.rb.x, p.rb.y);
        (q.base.0[l], q.normal_mass.0[l], q.tangent_mass.0[l]) = (p.base, p.normal_mass, p.tangent_mass);
        (q.rna.0[l], q.rnb.0[l], q.rta.0[l], q.rtb.0[l]) = (p.rna, p.rnb, p.rta, p.rtb);
        (q.jn.0[l], q.jt.0[l], q.speed.0[l]) = (p.jn, p.jt, p.speed);
    }
}

/// `head` and `setup` for bodies in place: `len` indices (holes, the
/// stand-in for statics and `nowhere` included), the bodies at `live`.
/// Takes the step's gravity out of each live body's velocity, in place.
pub fn setup_in<const N: usize, B: Place + Source>(
    params: &Params,
    bodies: &mut B,
    (len, live, nowhere): (usize, &[Range<usize>], u32),
    contacts: &mut [Constraint],
    points: &mut [Points],
    dt: f32,
) -> InPlaceSolve<N> {
    let substeps = params.substeps;
    let h = dt / substeps as f32;
    let inv_h = 1.0 / h;
    let moving = Softness::new(params.stiffness * inv_h, DAMPING_RATIO, h);
    let fixed = Softness::new(params.static_stiffness * inv_h, DAMPING_RATIO, h);
    let share = 1.0 / substeps as f32;
    let warm = if params.warm { share } else { 0.0 };
    let src = &*bodies;
    let mut moves = vec![false; len];
    let mut gravity = vec![Vec2::ZERO; len];
    for r in live {
        for i in r.clone() {
            let b = src.body(i);
            moves[i] = b.inv_mass > 0.0 || src.ang(i).1 > 0.0;
            gravity[i] = b.gravity;
        }
    }
    let (groups, n_groups) = group(contacts, &moves, params.wide);
    let mut count = vec![0usize; n_groups];
    let mut overflow = 0;
    for k in groups.iter() {
        match *k {
            UNSOLVED => (),
            OVERFLOW => overflow += 1,
            k => count[k as usize] += 1,
        }
    }
    let mut first = vec![0usize; n_groups];
    let mut batches = overflow;
    for k in 0..n_groups {
        first[k] = batches;
        batches += count[k].div_ceil(N);
    }
    let hd = Head {
        s: Vec::new(),
        inertia: Vec::new(),
        groups,
        n_groups,
        count,
        first,
        overflow,
        batches,
        nowhere,
        h,
        inv_h,
        share,
        warm,
        soft: (moving, fixed),
    };
    let (mut filled, mut overflowed) = (vec![0usize; hd.n_groups], 0);
    let mut out: Vec<Batch<N>> = vec![Batch::empty(hd.nowhere); hd.batches];
    let mut lanes: Vec<[Lane; N]> = vec![[Lane::NONE; N]; hd.batches];
    let mut kept: Vec<(usize, [(f32, f32); 2])> = Vec::new();
    let mut solved: Vec<usize> = Vec::new();
    for (i, c) in contacts.iter_mut().enumerate() {
        let k = hd.groups[i];
        let (jn, jt) = (c.jn, c.jt);
        (c.jn, c.jt) = (0.0, 0.0);
        let (pts, at, n_points, speed) = start_in(params, c, (jn, jt), src, &hd, points, dt);
        c.speed = speed;
        if at != NONE {
            solved.push(at as usize);
        }
        if k == UNSOLVED {
            unsolved(params, c, (&pts, at, n_points), &mut kept);
            continue;
        }
        let (batch, l) = place::<N>(k, &hd, (&mut filled, &mut overflowed));
        lanes[batch][l] = Lane { contact: i as u32, at, count: n_points as u8, speed };
        put_in(&mut out[batch], l, c, src, &hd, (&pts, at, n_points));
    }
    clear(points, solved, kept);
    for r in live {
        for i in r.clone() {
            let v = Place::v(bodies, i) - gravity[i];
            bodies.set_v(i, v);
        }
    }
    let counts = hd.count.iter().map(|c| c.div_ceil(N)).collect();
    InPlaceSolve { out, lanes, gravity, overflow: hd.overflow, counts, h, inv_h, share }
}

/// `run`, over bodies in place: `spinning` are the indices of the bodies
/// that turn, `angle` theirs, in the same order.
pub fn run_in<const N: usize>(
    p: &mut InPlaceSolve<N>,
    params: &Params,
    place: &mut (impl Place + ?Sized),
    live: &[Range<usize>],
    (spinning, angle): (&[u32], &mut [f32]),
) {
    let InPlaceSolve { out, gravity, h, inv_h, share, .. } = p;
    let (h, inv_h, share, substeps) = (*h, *inv_h, *share, params.substeps);
    for _ in 0..substeps {
        for r in live {
            for i in r.clone() {
                place.set_v(i, place.v(i) + gravity[i] * share);
            }
        }
        {
            let mut s = InPlace(&mut *place);
            for o in out.iter_mut() {
                warm_start(o, &mut s);
            }
            let last = params.relax == 0;
            for o in out.iter_mut() {
                pass::<N, true, COMPUTE>(o, &mut s, inv_h, last);
            }
        }
        for r in live {
            for i in r.clone() {
                place.set_moved(i, place.moved(i) + place.v(i) * h);
            }
        }
        for (&sp, angle) in spinning.iter().zip(angle.iter_mut()) {
            let i = sp as usize;
            let w = place.w(i);
            *angle += h * w;
            let turned = match params.integrate {
                Integrate::Rotation => place.turned(i).integrate(h * w),
                Integrate::Angle => Rot::from_angle(*angle),
            };
            place.set_turned(i, turned);
        }
        let mut s = InPlace(&mut *place);
        for r in 0..params.relax {
            let last = r + 1 == params.relax;
            if r == 0 {
                for o in out.iter_mut() {
                    pass::<N, false, STORE>(o, &mut s, inv_h, last);
                }
            } else {
                for o in out.iter_mut() {
                    pass::<N, false, LOAD>(o, &mut s, inv_h, last);
                }
            }
        }
    }
    let mut s = InPlace(&mut *place);
    for o in out.iter_mut() {
        if o.restitution.0.iter().any(|e| *e != 0.0) {
            restitute(o, &mut s, params.bounce);
        }
    }
}

/// `finish`'s impulses into the contacts and points; the bodies are
/// already where they live.
pub fn finish_in<const N: usize>(p: &InPlaceSolve<N>, params: &Params, contacts: &mut [Constraint], points: &mut [Points]) {
    let substeps = params.substeps;
    for (o, lanes) in p.out.iter().zip(p.lanes.iter()) {
        for (l, lane) in lanes.iter().enumerate() {
            if lane.contact == NONE {
                continue;
            }
            let c = &mut contacts[lane.contact as usize];
            (c.jn, c.jt) = impulses(o, l, lane);
            if lane.at != NONE {
                carried(params.carry, substeps, o, l, lane.count as usize, &mut points[lane.at as usize]);
            }
        }
    }
}

/// The lanes' own `solve`, timed by part: setting up (states and batches),
/// the passes, and finishing (impulses and states back into the copy), µs.
pub fn solve_timed<const N: usize>(
    params: &Params,
    (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
    contacts: &mut [Constraint],
    points: &mut [Points],
    dt: f32,
) -> [f64; 3] {
    let t0 = std::time::Instant::now();
    let mut p = setup::<N>(params, (bodies, spinning), contacts, points, dt);
    let t1 = std::time::Instant::now();
    run(&mut p, params, spinning);
    let t2 = std::time::Instant::now();
    let Solve { s, out, lanes, angle, .. } = p;
    finish(params, out.iter().zip(lanes.iter()), (bodies, spinning), (&s[..], angle), contacts, points);
    let t3 = std::time::Instant::now();
    let us = |a: std::time::Instant, b: std::time::Instant| (b - a).as_secs_f64() * 1e6;
    [us(t0, t1), us(t1, t2), us(t2, t3)]
}

// ---- Across threads ----

/// The bodies the threads of a solve share, each value an `f32`'s bits in
/// a relaxed atomic: the copy's `Atom`s (as `solve_across` shares them), or
/// atomic views of the world's columns. What a stage does to a body, by
/// its index.
pub trait Atomics: Sync {
    type View<'a>: Bodies
    where
        Self: 'a;
    fn view(&self) -> Self::View<'_>;
    /// `v += dv`.
    fn add_v(&self, i: usize, dv: Vec2);
    /// `moved += v * h`.
    fn step(&self, i: usize, h: f32);
    fn w(&self, i: usize) -> f32;
    fn turned(&self, i: usize) -> Rot;
    fn set_turned(&self, i: usize, q: Rot);
}

/// States as `solve_across` shares them, one `Atom` (32 bytes) a body.
pub struct Atoms(pub Vec<Atom>);

impl Atoms {
    /// From a place, its first `n` indices.
    pub fn of(p: &(impl Place + ?Sized), n: usize) -> Atoms {
        Atoms((0..n).map(|i| Atom::new(&State { v: p.v(i), w: p.w(i), moved: p.moved(i), turned: p.turned(i) })).collect())
    }

    pub fn state(&self, i: usize) -> (Vec2, f32, Vec2, Rot) {
        let s = self.0[i].state();
        (s.v, s.w, s.moved, s.turned)
    }
}

impl Atomics for Atoms {
    type View<'a> = Shared<'a>;
    fn view(&self) -> Shared<'_> {
        Shared(&self.0)
    }
    #[inline(always)]
    fn add_v(&self, i: usize, dv: Vec2) {
        let a = &self.0[i];
        let v = Vec2::new(a.get(0), a.get(1)) + dv;
        a.put(0, v.x);
        a.put(1, v.y);
    }
    #[inline(always)]
    fn step(&self, i: usize, h: f32) {
        let a = &self.0[i];
        let m = Vec2::new(a.get(3), a.get(4)) + Vec2::new(a.get(0), a.get(1)) * h;
        a.put(3, m.x);
        a.put(4, m.y);
    }
    #[inline(always)]
    fn w(&self, i: usize) -> f32 {
        self.0[i].get(2)
    }
    #[inline(always)]
    fn turned(&self, i: usize) -> Rot {
        Rot { c: self.0[i].get(5), s: self.0[i].get(6) }
    }
    #[inline(always)]
    fn set_turned(&self, i: usize, q: Rot) {
        self.0[i].put(5, q.c);
        self.0[i].put(6, q.s);
    }
}

/// Columns as words: velocity's `x` and `y` at `vx` and `vy` of each row's
/// two words (where the component's layout puts them), spin one word a
/// row, and the step's motion and turn two words a row each, all by the
/// same index.
pub struct Words<'a> {
    pub v: &'a [AtomicU32],
    pub vx: usize,
    pub vy: usize,
    pub w: &'a [AtomicU32],
    pub moved: &'a [AtomicU32],
    pub turned: &'a [AtomicU32],
}

#[inline(always)]
fn word(a: &AtomicU32) -> f32 {
    f32::from_bits(a.load(Ordering::Relaxed))
}

#[inline(always)]
fn put_word(a: &AtomicU32, x: f32) {
    a.store(x.to_bits(), Ordering::Relaxed)
}

impl Words<'_> {
    #[inline(always)]
    fn vel(&self, i: usize) -> Vec2 {
        Vec2::new(word(&self.v[2 * i + self.vx]), word(&self.v[2 * i + self.vy]))
    }

    pub fn state(&self, i: usize) -> (Vec2, f32, Vec2, Rot) {
        let s = WordsView(self).load(i);
        (s.v, s.w, s.moved, s.turned)
    }
}

pub struct WordsView<'b, 'a>(&'b Words<'a>);

impl Bodies for WordsView<'_, '_> {
    #[inline(always)]
    fn load(&self, i: usize) -> State {
        let s = self.0;
        State {
            v: s.vel(i),
            w: word(&s.w[i]),
            moved: Vec2::new(word(&s.moved[2 * i]), word(&s.moved[2 * i + 1])),
            turned: Rot { c: word(&s.turned[2 * i]), s: word(&s.turned[2 * i + 1]) },
        }
    }
    #[inline(always)]
    fn load_v(&self, i: usize) -> (Vec2, f32) {
        (self.0.vel(i), word(&self.0.w[i]))
    }
    #[inline(always)]
    fn store_v(&mut self, i: usize, v: Vec2, w: f32) {
        let s = self.0;
        put_word(&s.v[2 * i + s.vx], v.x);
        put_word(&s.v[2 * i + s.vy], v.y);
        put_word(&s.w[i], w);
    }
}

impl<'a> Atomics for Words<'a> {
    type View<'b>
        = WordsView<'b, 'a>
    where
        Self: 'b;
    fn view(&self) -> WordsView<'_, 'a> {
        WordsView(self)
    }
    #[inline(always)]
    fn add_v(&self, i: usize, dv: Vec2) {
        let v = self.vel(i) + dv;
        put_word(&self.v[2 * i + self.vx], v.x);
        put_word(&self.v[2 * i + self.vy], v.y);
    }
    #[inline(always)]
    fn step(&self, i: usize, h: f32) {
        let m = Vec2::new(word(&self.moved[2 * i]), word(&self.moved[2 * i + 1])) + self.vel(i) * h;
        put_word(&self.moved[2 * i], m.x);
        put_word(&self.moved[2 * i + 1], m.y);
    }
    #[inline(always)]
    fn w(&self, i: usize) -> f32 {
        word(&self.w[i])
    }
    #[inline(always)]
    fn turned(&self, i: usize) -> Rot {
        Rot { c: word(&self.turned[2 * i]), s: word(&self.turned[2 * i + 1]) }
    }
    #[inline(always)]
    fn set_turned(&self, i: usize, q: Rot) {
        put_word(&self.turned[2 * i], q.c);
        put_word(&self.turned[2 * i + 1], q.s);
    }
}

/// A stage's unit of work in `run_across_in`: `Part`, with the batches
/// already filled (by `setup_in`, on the calling thread).
enum PartIn<'a, const N: usize> {
    Batches(&'a mut [Batch<N>]),
    /// Runs of live bodies, by their place in `live`.
    Bodies(Range<usize>),
    Spins(Range<usize>, &'a mut [f32]),
}

#[repr(align(128))]
struct BlockIn<'a, const N: usize> {
    mark: AtomicUsize,
    part: Mutex<PartIn<'a, N>>,
}

/// `run_across`'s stages after its first (filling the batches, which
/// `setup_in` did), over bodies `a` shares: the passes, the bodies' gravity
/// and motion, restitution. Its protocol line for line, so it is `run_in`
/// bit for bit at any thread count, as `run_across` is `run`.
pub fn run_across_in<const N: usize, A: Atomics>(
    p: &mut InPlaceSolve<N>,
    params: &Params,
    a: &A,
    live: &[Range<usize>],
    (spinning, angle): (&[u32], &mut [f32]),
    gang: &dyn Gang,
) {
    let threads = gang.threads();
    let (h, inv_h, share) = (p.h, p.inv_h, p.share);
    let gravity = &p.gravity;
    let mut blocks: Vec<BlockIn<N>> = Vec::new();
    let block = |part| BlockIn { mark: AtomicUsize::new(0), part: Mutex::new(part) };
    let mut rest: &mut [Batch<N>] = &mut p.out;
    let overflowed = if p.overflow > 0 {
        let (head, tail) = rest.split_at_mut(p.overflow);
        rest = tail;
        blocks.push(block(PartIn::Batches(head)));
        Some(blocks.len() - 1..blocks.len())
    } else {
        None
    };
    let mut colors = Vec::new();
    for &n in &p.counts {
        let (mut group, tail) = rest.split_at_mut(n);
        rest = tail;
        let from = blocks.len();
        for r in blocks_of(group.len(), 4, threads) {
            let (head, tail) = group.split_at_mut(r.len());
            group = tail;
            blocks.push(block(PartIn::Batches(head)));
        }
        if blocks.len() > from {
            colors.push(from..blocks.len());
        }
    }
    // Bodies in blocks of about 32, as `run_across`'s, in runs of `live`.
    let rows: usize = live.iter().map(|r| r.len()).sum();
    let per_run = (rows / live.len().max(1)).max(1);
    let from = blocks.len();
    blocks.extend(blocks_of(live.len(), 32usize.div_ceil(per_run), threads).map(|r| block(PartIn::Bodies(r))));
    let bodies_at = from..blocks.len();
    let mut turning: &mut [f32] = angle;
    for r in blocks_of(spinning.len(), 32, threads) {
        let (head, tail) = turning.split_at_mut(r.len());
        turning = tail;
        blocks.push(block(PartIn::Spins(r, head)));
    }
    let moving = bodies_at.start..blocks.len();

    let mut stages: Vec<(Work, Range<usize>)> = Vec::new();
    let pass_of = |stages: &mut Vec<(Work, Range<usize>)>, work: Work| {
        stages.extend(overflowed.iter().chain(colors.iter()).map(|b| (work, b.clone())));
    };
    for _ in 0..params.substeps {
        stages.push((Work::Gravity, bodies_at.clone()));
        pass_of(&mut stages, Work::Warm);
        pass_of(&mut stages, Work::Push(params.relax == 0));
        stages.push((Work::Move, moving.clone()));
        for r in 0..params.relax {
            pass_of(&mut stages, Work::Relax(r == 0, r + 1 == params.relax));
        }
    }
    pass_of(&mut stages, Work::Bounce);

    let exec = |work: Work, part: &mut PartIn<'_, N>, sh: &mut A::View<'_>| match (work, part) {
        (Work::Warm, PartIn::Batches(b)) => b.iter_mut().for_each(|o| warm_start(o, sh)),
        (Work::Push(last), PartIn::Batches(b)) => b.iter_mut().for_each(|o| pass::<N, true, COMPUTE>(o, sh, inv_h, last)),
        (Work::Relax(true, last), PartIn::Batches(b)) => b.iter_mut().for_each(|o| pass::<N, false, STORE>(o, sh, inv_h, last)),
        (Work::Relax(false, last), PartIn::Batches(b)) => b.iter_mut().for_each(|o| pass::<N, false, LOAD>(o, sh, inv_h, last)),
        (Work::Bounce, PartIn::Batches(b)) => {
            for o in b.iter_mut() {
                if o.restitution.0.iter().any(|e| *e != 0.0) {
                    restitute(o, sh, params.bounce);
                }
            }
        }
        (Work::Gravity, PartIn::Bodies(r)) => {
            for run in &live[r.clone()] {
                for i in run.clone() {
                    a.add_v(i, gravity[i] * share);
                }
            }
        }
        (Work::Move, PartIn::Bodies(r)) => {
            for run in &live[r.clone()] {
                for i in run.clone() {
                    a.step(i, h);
                }
            }
        }
        (Work::Move, PartIn::Spins(r, angle)) => {
            for (&sp, angle) in spinning[r.clone()].iter().zip(angle.iter_mut()) {
                let i = sp as usize;
                let w = a.w(i);
                *angle += h * w;
                let turned = match params.integrate {
                    Integrate::Rotation => a.turned(i).integrate(h * w),
                    Integrate::Angle => Rot::from_angle(*angle),
                };
                a.set_turned(i, turned);
            }
        }
        _ => unreachable!("a stage over another stage's blocks"),
    };

    let done: Vec<Count> = stages.iter().map(|_| Count::default()).collect();
    let failed = AtomicBool::new(false);
    let each = |w: usize| {
        let _failing = Failing(&failed);
        let mut sh = a.view();
        for (t, (work, on)) in stages.iter().enumerate() {
            let (n, count) = (on.len(), &done[t].0);
            if count.load(Ordering::Acquire) == n {
                continue;
            }
            let mut take = |k: usize| {
                let b = &blocks[on.start + k];
                if b.mark.fetch_max(t + 1, Ordering::AcqRel) > t {
                    return false;
                }
                exec(*work, &mut b.part.try_lock().expect("a block's taker alone has it"), &mut sh);
                true
            };
            let (start, mut ran) = (first_block(w, n, threads), 0);
            let mut k = start;
            while take(k) {
                ran += 1;
                k = if k + 1 == n { 0 } else { k + 1 };
            }
            let mut k = start;
            loop {
                k = if k == 0 { n - 1 } else { k - 1 };
                if !take(k) {
                    break;
                }
                ran += 1;
            }
            if ran > 0 {
                count.fetch_add(ran, Ordering::Release);
            }
            let mut spins = 0u32;
            while count.load(Ordering::Acquire) < n {
                if failed.load(Ordering::Relaxed) {
                    return;
                }
                std::hint::spin_loop();
                spins = spins.wrapping_add(1);
                if spins.is_multiple_of(1024) {
                    std::thread::yield_now();
                }
            }
        }
    };
    gang.run(threads, &each);
}
