//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): the 2D mod's
//! solve as its pipeline runs it (`prepare`, `passes`, `finish`:
//! pipeline.rs), on captured inputs, with its passes handed to a
//! dispatcher (`dispatch.rs`) instead of `Passes::run`. The kernels are
//! the mod's (`solver::staged`), unchanged.

use std::time::Instant;

use engine_ecs::Executor;
use engine_ecs::shape::{Colored, Coloring, Shareable, Stage, States};

use crate::dispatch::{self, Plant, Prepared, Program, Protocol, Trace};
use crate::scene::Scene;
use crate::solver::staged::{Atom, Shared, Staged, State, Step};
use crate::solver::{self, Constraint, PARAMS, Points, SolverBody, Spinning};
use crate::{Sim, arrays, ecs, variants};

const LANES: usize = 4;

#[derive(Clone)]
pub struct Input {
    pub bodies: Vec<SolverBody>,
    pub spinning: Vec<Spinning>,
    pub contacts: Vec<Constraint>,
    pub points: Vec<Points>,
}

/// pipeline.rs's: the lanes' states shared as relaxed atomics.
impl Shareable for State {
    type Shared = Atom;

    fn share(&self) -> Atom {
        State::share(self)
    }

    fn load(shared: &Atom) -> State {
        State::load(shared)
    }

    fn store(shared: &Atom, value: State) {
        State::store(shared, value)
    }
}

/// The solver's inputs at steps `at` of `scene`, turning: solver_bench's
/// `capture`.
pub fn capture(scene: &Scene, at: &[u32]) -> Vec<Input> {
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    let got: Rc<RefCell<Vec<Input>>> = Rc::default();
    let step = Rc::new(Cell::new(0u32));
    let (g, s, when) = (got.clone(), step.clone(), at.to_vec());
    let solve: variants::Boxed = Box::new(move |bodies, spinning, contacts, points, dt| {
        s.set(s.get() + 1);
        if when.contains(&s.get()) {
            let input =
                Input { bodies: bodies.to_vec(), spinning: spinning.to_vec(), contacts: contacts.to_vec(), points: points.to_vec() };
            g.borrow_mut().push(input);
        }
        solver::solve_points(bodies, spinning, contacts, points, dt);
    });
    let mut flat = ecs::Flat::new(scene, true, solve, "capture");
    flat.step(*at.iter().max().unwrap());
    drop(flat);
    Rc::try_unwrap(got).ok().unwrap().into_inner()
}

/// Every value a solve leaves, as bits: solver_bench's `bits`.
pub fn bits(i: &Input) -> Vec<u32> {
    let mut out = Vec::new();
    for b in &i.bodies {
        out.extend([b.v.x, b.v.y, b.moved.x, b.moved.y].map(f32::to_bits));
    }
    for s in &i.spinning {
        out.extend([s.w, s.turned.c, s.turned.s, s.angle].map(f32::to_bits));
    }
    for c in &i.contacts {
        out.extend([c.jn, c.jt, c.speed].map(f32::to_bits));
    }
    for p in &i.points {
        out.push(p.solved as u32);
        for q in &p.point {
            out.extend([q.jn, q.jt].map(f32::to_bits));
        }
    }
    out
}

/// Whether a step can be shared between threads bit for bit: a copy of
/// `lanes::shareable` (private to solver.rs), get-znt.39's condition. No
/// still body with a negative zero where batches write it back.
pub fn shareable(bodies: &[SolverBody], spinning: &[Spinning]) -> bool {
    let neg = |x: f32| x.to_bits() == (-0.0f32).to_bits();
    let still = |b: &SolverBody| b.inv_mass == 0.0;
    bodies.iter().all(|b| !still(b) || ![b.v.x, b.v.y, b.gravity.x, b.gravity.y].into_iter().any(neg))
        && spinning.iter().all(|sp| !still(&bodies[sp.body as usize]) || sp.inv_inertia > 0.0 || !neg(sp.w))
}

/// What the pipeline keeps between steps: the graph flow's payload.
#[derive(Default)]
pub struct Solve {
    staged: Staged<LANES>,
    moves: Vec<bool>,
    coloring: Coloring,
    masks: Vec<u64>,
    place: Vec<Option<(u32, u32)>>,
    layout: Colored,
    program: Vec<Stage<Step>>,
}

#[derive(Clone, Copy)]
pub enum How<'a> {
    /// The passes as `Passes::run` runs them on one thread, plain.
    One,
    Across(Protocol, &'a dyn Executor, Plant),
    /// As `Across`, but shared even where `shareable` says not to: what
    /// get-znt.39 guards against.
    Unguarded(Protocol, &'a dyn Executor),
}

impl Solve {
    /// `prepare`: the bodies as states, the contacts colored and in lanes.
    /// False where the step isn't solved in lanes (the pipeline's `finish`
    /// then solves it whole).
    pub fn prepare(&mut self, i: &mut Input) -> bool {
        let (params, dt) = (PARAMS, arrays::DT);
        if !Staged::<LANES>::takes(&params, &i.spinning, &i.points) {
            return false;
        }
        let moves = self.staged.begin(&params, (&i.bodies, &i.spinning), dt);
        self.moves.clear();
        self.moves.extend_from_slice(moves);
        let c = &i.contacts;
        self.coloring.greedy(c.len(), |k| (c[k].a, c[k].b), &self.moves, true, &mut self.masks);
        self.layout = self.coloring.pack(LANES, &mut self.place);
        self.staged.fill(&params, (&i.bodies, i.spinning.len()), (&mut i.contacts, &mut i.points), (&self.place, self.layout.items()), dt);
        self.program.clear();
        let program = &mut self.program;
        solver::staged::program(&params, self.staged.states(), i.spinning.len(), |k, each| {
            program.push(match each {
                None => Stage::Items(k),
                Some(n) => Stage::Each(k, n),
            })
        });
        true
    }

    /// `passes`, dispatched by `how`.
    pub fn passes(&mut self, i: &Input, how: How, trace: Option<&Trace>) {
        let params = PARAMS;
        let n = self.staged.states();
        let share = shareable(&i.bodies, &i.spinning);
        let (items, states, kernels) = self.staged.split(&params, &i.spinning);
        let (layout, program) = (&self.layout, &self.program);
        match how {
            How::One => dispatch::one_thread(
                layout,
                items,
                states,
                program,
                &|k, block, s| match s {
                    States::Plain(s) => kernels.block(k, block, s),
                    States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
                },
                &|k, r, s| match s {
                    States::Plain(s) => kernels.each(k, r, s, n),
                    States::Shared(s) => kernels.each(k, r, &mut Shared(s), n),
                },
            ),
            How::Across(..) | How::Unguarded(..) => {
                let (protocol, exec, plant, share) = match how {
                    How::Across(p, e, plant) => (p, e, plant, share),
                    How::Unguarded(p, e) => (p, e, Plant::None, true),
                    How::One => unreachable!(),
                };
                dispatch::passes(
                    (protocol, exec, share, plant),
                    layout,
                    items,
                    states,
                    program,
                    |k, block, s| match s {
                        States::Plain(s) => kernels.block(k, block, s),
                        States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
                    },
                    |k, r, s| match s {
                        States::Plain(s) => kernels.each(k, r, s, n),
                        States::Shared(s) => kernels.each(k, r, &mut Shared(s), n),
                    },
                    trace,
                )
            }
        }
    }

    /// `finish`: the impulses and states back into the contacts and
    /// bodies.
    pub fn finish(&self, i: &mut Input) {
        self.staged.finish(&PARAMS, (&mut i.bodies, &mut i.spinning), (&mut i.contacts, &mut i.points));
    }

    /// The whole solve, the pipeline's way; µs of prepare, passes and
    /// finish.
    pub fn solve(&mut self, i: &mut Input, how: How, trace: Option<&Trace>) -> [f64; 3] {
        let t0 = Instant::now();
        if !self.prepare(i) {
            solver::solve_with(&PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT);
            return [0.0, 0.0, t0.elapsed().as_secs_f64() * 1e6];
        }
        let t1 = Instant::now();
        self.passes(i, how, trace);
        let t2 = Instant::now();
        self.finish(i);
        let us = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1e6;
        [us(t0, t1), us(t1, t2), us(t2, Instant::now())]
    }

    /// Each stage's blocks as a dispatcher at `threads` lays them out.
    pub fn stage_blocks(&mut self, i: &Input, threads: usize) -> Vec<usize> {
        let (items, states, _) = self.staged.split(&PARAMS, &i.spinning);
        let p: Prepared<'_, _, State, Step> = Prepared::new(&self.layout, items, states, &self.program, threads);
        let run = |_: usize, _: usize| {};
        p.program(&run).stages.iter().map(|s| s.0).collect()
    }

    /// The overflow's items and each color's, as last prepared.
    pub fn layout(&self) -> &Colored {
        &self.layout
    }
}

/// Two steps' passes (already prepared) as one dispatch of two programs,
/// or one after the other: what filling one program's thin stages with
/// another's blocks buys. µs.
pub fn two(solves: &mut [Solve; 2], inputs: &[Input; 2], protocol: Protocol, exec: &dyn Executor, together: bool) -> f64 {
    let params = PARAMS;
    let t = Instant::now();
    if !together {
        for (s, i) in solves.iter_mut().zip(inputs) {
            s.passes(i, How::Across(protocol, exec, Plant::None), None);
        }
        return t.elapsed().as_secs_f64() * 1e6;
    }
    let [a, b] = solves;
    let (na, nb) = (a.staged.states(), b.staged.states());
    let (ia, sa, ka) = a.staged.split(&params, &inputs[0].spinning);
    let (ib, sb, kb) = b.staged.split(&params, &inputs[1].spinning);
    let threads = exec.threads();
    let pa: Prepared<'_, _, State, Step> = Prepared::new(&a.layout, ia, sa, &a.program, threads);
    let pb: Prepared<'_, _, State, Step> = Prepared::new(&b.layout, ib, sb, &b.program, threads);
    let ra = |t: usize, k: usize| {
        pa.exec(
            t,
            k,
            &|k, block, s| match s {
                States::Plain(s) => ka.block(k, block, s),
                States::Shared(s) => ka.block(k, block, &mut Shared(s)),
            },
            &|k, r, s| match s {
                States::Plain(s) => ka.each(k, r, s, na),
                States::Shared(s) => ka.each(k, r, &mut Shared(s), na),
            },
        )
    };
    let rb = |t: usize, k: usize| {
        pb.exec(
            t,
            k,
            &|k, block, s| match s {
                States::Plain(s) => kb.block(k, block, s),
                States::Shared(s) => kb.block(k, block, &mut Shared(s)),
            },
            &|k, r, s| match s {
                States::Plain(s) => kb.each(k, r, s, nb),
                States::Shared(s) => kb.each(k, r, &mut Shared(s), nb),
            },
        )
    };
    let programs: [Program; 2] = [pa.program(&ra), pb.program(&rb)];
    dispatch::run(protocol, exec, &programs, None, Plant::None);
    let us = t.elapsed().as_secs_f64() * 1e6;
    pa.finish(sa);
    pb.finish(sb);
    us
}

/// The scenes the spike measures, as step_bench names them, with the steps
/// captured.
pub fn scenes() -> Vec<(&'static str, Scene, Vec<u32>)> {
    let pile = Scene::Pile { n: 10000, width: 401.0, stagger: true };
    vec![
        ("pile 10 000, settled", pile, vec![401, 430, 460]),
        ("pyramid 5050", Scene::Pyramid { base: 100 }, vec![601, 630, 660]),
        // In step_bench's window (steps 2-31), where contacts have begun.
        ("pile 10 000, falling", pile, vec![20, 25, 30]),
    ]
}
