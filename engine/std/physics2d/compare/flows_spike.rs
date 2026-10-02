//! SPIKE, not engine code: the measurements behind
//! docs/architecture/flows-spike.md. Kept as a bench target so its numbers
//! can be taken again; nothing depends on it.
//!
//! The question: can physics's solve be a pipeline of flows (values systems
//! hand one another within a frame, `//engine/ecs:flows_spike`), its
//! parallel passes a generic primitive (`flows::Colored::passes`), bit for
//! bit the solve as built, and at what cost? The 2D mod runs a scene in the
//! engine to a step; then, on that world between frames:
//!
//! 1. the pipeline (`flows_physics.rs`) is checked bit for bit against the
//!    solve as built, at one thread and across the pool's;
//! 2. both are timed whole, by stage, and as one system calling the same
//!    stages (`fused`), with flows' allocations recycled and fresh;
//! 3. the solver alone, on the gathered input: `solve_across` as built
//!    (its staged run timed apart) against the generic primitive, a batch
//!    of 4 or 8 edges a kernel call, the same behind `&dyn Fn`, and a
//!    kernel per edge with its two states;
//! 4. what a flow's hand-off costs, on systems that do nothing.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:flows_spike
//!
//! `REPS` (21), the median of each; `THREADS` (8) for the pool's threads;
//! `ONLY=falling`, `settled` or `pyramid`.

#[allow(dead_code)]
#[path = "flows_spike_arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[allow(dead_code)]
mod flows_physics;
#[path = "flows_spike_narrow.rs"]
mod narrow;
#[allow(dead_code)]
#[path = "flows_spike_pool.rs"]
mod pool;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
// solver.rs with flows_lanes.rs included in its `lanes` (the BUILD's
// `flows_spike_srcs`).
#[allow(dead_code)]
#[path = "flows_spike_solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "flows_spike_split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Executor, Query, Workers, World};
use flows::{Make, Pass, See, Take, flow};
use physics2d::Tuning;
pub use sim::{Dyn, Sim};

use flows_physics::Shape;
use scene::Scene;
use solver::lanes::Prepared;
use solver::{Constraint, Points, SolverBody, Spinning, Wide};

const DT: f32 = 1.0 / 60.0;

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

fn us(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e6
}

/// Keeps the pool's threads busy for a while, so its cores are clocked up
/// before a run across them
/// (docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md).
fn warm(gang: &Workers) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(30) {
        gang.run(gang.threads(), |_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_micros(500) {
                std::hint::spin_loop();
            }
        });
    }
}

/// The mod's own stages over a window, µs a step.
fn in_situ(ecs: &mut ecs::Ecs, steps: u32) -> String {
    ecs.reset();
    ecs.step(steps);
    let stages = ecs.engine().send("physics2d", "stages").unwrap();
    let per = |k: &str| {
        let mut words = stages.split_whitespace();
        words.find(|w| *w == k).unwrap_or_else(|| panic!("no {k} in {stages}"));
        words.next().unwrap().parse::<f64>().unwrap()
    };
    format!(
        "in the mod over {steps} steps, one thread, µs a step: solve {:.0} (gather {:.1}, solver {:.0}, write-back {:.1}, its sides and sleeping included)",
        per("solve_gather") + per("solver") + per("write_back"),
        per("solve_gather"),
        per("solver"),
        per("write_back"),
    )
}

/// The solver's input as the gathers leave it.
#[derive(Clone)]
struct Input {
    bodies: Vec<SolverBody>,
    spinning: Vec<Spinning>,
    constraints: Vec<Constraint>,
    points: Vec<Points>,
    params: solver::Params,
}

fn capture(world: &World) -> Input {
    let got: Arc<Mutex<Option<Input>>> = Arc::default();
    let g = got.clone();
    let sys = move |_: &mut Cx,
                    (mut gravity, mut tuning): (Query<&physics2d::Gravity>, Query<&Tuning>),
                    mut moving: flows_physics::MovingQ,
                    mut turning: flows_physics::TurningQ,
                    mut contacts: flows_physics::ContactsQ| {
        let one = Workers::default();
        let (mut b, mut t, mut c) =
            (flows_physics::Bodies::default(), flows_physics::Turning::default(), flows_physics::Contacts::default());
        let grav = gravity.single(|_, g| physics2d::Vec2::new(g.x, g.y)).unwrap_or_default();
        flows_physics::gather_bodies(DT, grav, &mut moving, &one, &mut b);
        flows_physics::gather_turning(&mut turning, &b, &mut t);
        flows_physics::gather_contacts(&mut contacts, &b, &one, &mut c);
        let params = solver::Params::of(&tuning.single(|_, t| *t).unwrap_or(Tuning::DEFAULT));
        *g.lock().unwrap() = Some(Input { bodies: b.bodies, spinning: t.spinning, constraints: c.constraints, points: c.points, params });
    };
    Schedule { systems: vec![sys.system(world, "capture")] }.run_sequential(world);
    got.lock().unwrap().take().unwrap()
}

/// A solve's output, as bits.
fn bits(i: &Input) -> Vec<u32> {
    let mut out = Vec::new();
    for b in &i.bodies {
        out.extend([b.v.x, b.v.y, b.moved.x, b.moved.y].map(f32::to_bits));
    }
    for s in &i.spinning {
        out.extend([s.w, s.turned.c, s.turned.s, s.angle].map(f32::to_bits));
    }
    for c in &i.constraints {
        out.extend([c.jn, c.jt, c.speed].map(f32::to_bits));
    }
    for p in &i.points {
        for q in &p.point {
            out.extend([q.jn, q.jt].map(f32::to_bits));
        }
        out.push(p.solved as u32);
    }
    out
}

/// Ways of solving the captured input, each `(setup, passes, tail)` in µs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Way {
    /// `solver::solve_across` as built, whole.
    Built(usize),
    /// The same, its staged run timed apart (`solve_across_timed`).
    Staged(usize),
    /// The flow stages, the passes on `Colored::passes`.
    Generic(usize, Shape),
}

impl Way {
    fn name(self) -> String {
        match self {
            Way::Built(n) => format!("as built, {n} wide (`solve_across`)"),
            Way::Staged(n) => format!("as built, {n} wide, staged run apart"),
            Way::Generic(n, Shape::Batch) => format!("generic, a kernel a batch of {n}"),
            Way::Generic(n, Shape::Dyn) => format!("generic, a batch of {n} through `&dyn Fn`"),
            Way::Generic(n, Shape::Edge) => format!("generic, a kernel an edge ({n} wide), its two states"),
        }
    }
}

const WAYS: [Way; 10] = [
    Way::Built(4),
    Way::Staged(4),
    Way::Generic(4, Shape::Batch),
    Way::Generic(4, Shape::Dyn),
    Way::Built(8),
    Way::Generic(8, Shape::Batch),
    Way::Built(1),
    Way::Staged(1),
    Way::Generic(1, Shape::Batch),
    Way::Generic(1, Shape::Edge),
];

#[derive(Default)]
struct Kept {
    p1: Prepared<1>,
    p4: Prepared<4>,
    p8: Prepared<8>,
}

fn solve_one(way: Way, input: &mut Input, kept: &mut Kept, workers: &Workers) -> (f64, f64, f64) {
    let wide = |n: usize| solver::Params { wide: Wide::Colored(n), ..input.params };
    let Input { bodies, spinning, constraints, points, .. } = input;
    match way {
        Way::Built(n) => {
            let t = Instant::now();
            solver::solve_across(&wide(n), (&mut **bodies, &mut **spinning), constraints, points, DT, workers);
            (0.0, us(t), 0.0)
        }
        Way::Staged(n) => {
            let parts = (&mut **bodies, &mut **spinning);
            match n {
                4 => solver::lanes::solve_across_timed::<4>(&wide(4), parts, constraints, points, DT, workers),
                8 => solver::lanes::solve_across_timed::<8>(&wide(8), parts, constraints, points, DT, workers),
                _ => solver::lanes::solve_across_timed::<1>(&wide(1), parts, constraints, points, DT, workers),
            }
        }
        Way::Generic(n, shape) => {
            macro_rules! stages {
                ($p:expr) => {{
                    let params = wide(n);
                    let t = Instant::now();
                    solver::lanes::prepare_flow($p, &params, (&**bodies, &**spinning), constraints, points, DT, workers);
                    let setup = us(t);
                    let t = Instant::now();
                    solver::lanes::passes_flow($p, &params, spinning, workers, shape);
                    let run = us(t);
                    let t = Instant::now();
                    solver::lanes::finish_flow($p, &params, (&mut **bodies, &mut **spinning), constraints, points, workers);
                    (setup, run, us(t))
                }};
            }
            match n {
                4 => stages!(&mut kept.p4),
                8 => stages!(&mut kept.p8),
                _ => stages!(&mut kept.p1),
            }
        }
    }
}

/// The solver alone on `input`, every way, at one thread and on `gang`.
fn solvers(label: &str, input: &Input, reps: usize, gang: &Workers) {
    let one = Workers::default();
    let mut want = input.clone();
    solver::solve_across(&input.params, (&mut want.bodies, &mut want.spinning), &mut want.constraints, &mut want.points, DT, &one);
    let want = bits(&want);
    // The generic coloring is `group`'s, contact for contact.
    let (groups, n) = solver::lanes::colors_as_built(&input.params, &input.bodies, &input.spinning, &input.constraints);
    let mut kept = Kept::default();
    let mut x = input.clone();
    solver::lanes::prepare_flow(&mut kept.p4, &input.params, (&x.bodies, &x.spinning), &mut x.constraints, &x.points, DT, &one);
    assert_eq!(kept.p4.coloring.of, groups, "{label}: the generic coloring is the solver's");
    assert_eq!(kept.p4.coloring.count.len(), n);
    let mut times: HashMap<(Way, usize), Vec<(f64, f64, f64)>> = HashMap::new();
    for rep in 0..reps {
        let mut ways = WAYS.to_vec();
        ways.rotate_left(rep % WAYS.len());
        for way in ways {
            for (threads, w) in [(1, &one), (gang.threads(), gang)] {
                if threads > 1 {
                    warm(w);
                }
                let mut x = input.clone();
                let t = solve_one(way, &mut x, &mut kept, w);
                assert!(bits(&x) == want, "{label}: {} at {threads} threads isn't the solve as built", way.name());
                times.entry((way, threads)).or_default().push(t);
            }
        }
    }
    println!("\n#### The solver alone: {label}\n");
    println!(
        "µs, the median of {reps}; every way bit for bit `solve_across` at one thread. Setup: coloring, states, lanes and batches filled (as built, the batches are filled in its run's first stage); tail: impulses and states written back.\n"
    );
    println!("| way | 1 thread: setup | passes | tail | all | {0} threads: setup | passes | tail | all |", gang.threads());
    println!("|---|---|---|---|---|---|---|---|---|");
    for way in WAYS {
        print!("| {} |", way.name());
        for threads in [1, gang.threads()] {
            let ts = &times[&(way, threads)];
            let m = |f: fn(&(f64, f64, f64)) -> f64| median(ts.iter().map(f).collect());
            let all = m(|t| t.0 + t.1 + t.2);
            match way {
                Way::Built(_) => print!(" – | – | – | {all:.0} |"),
                _ => print!(" {:.0} | {:.0} | {:.0} | {all:.0} |", m(|t| t.0), m(|t| t.1), m(|t| t.2)),
            }
        }
        println!();
    }
}

/// Each way of running the whole solve on the world, timed by frame and by
/// stage; the world put back after each.
fn systems(label: &str, world: &World, reps: usize, gang: &Arc<dyn Executor>) {
    let (save, restore, _) = flows_physics::saver(world);
    save.run_sequential(world);
    let reference = flows_physics::reference(world);
    let pipeline = flows_physics::pipeline(world);
    let fused = flows_physics::fused(world);
    let names = ["as built, one system", "pipeline of flows", "pipeline, fresh allocations", "fused: the flow stages in one system"];
    let mut frames: HashMap<(usize, usize), Vec<f64>> = HashMap::new();
    let mut stages: HashMap<(usize, usize, &'static str), Vec<f64>> = HashMap::new();
    for (threads, exec) in [(1, None), (gang.threads(), Some(gang.clone()))] {
        world.set_executor(exec);
        // A frame of each first, so flows and kept buffers have their
        // allocations.
        for s in [&reference, &pipeline, &fused] {
            s.run_sequential(world);
            restore.run_sequential(world);
        }
        flows_physics::take_times();
        for rep in 0..reps {
            let mut order: Vec<usize> = (0..names.len()).collect();
            order.rotate_left(rep % names.len());
            for i in order {
                flows::set_recycling(world, i != 2);
                if threads > 1 {
                    warm(&Workers::new(Some(gang.clone())));
                }
                let s = match i {
                    0 => &reference,
                    1 | 2 => &pipeline,
                    _ => &fused,
                };
                let t = Instant::now();
                s.run_sequential(world);
                frames.entry((i, threads)).or_default().push(us(t));
                for (name, us) in flows_physics::take_times() {
                    stages.entry((i, threads, name)).or_default().push(us);
                }
                restore.run_sequential(world);
            }
        }
        flows::set_recycling(world, true);
    }
    world.set_executor(None);
    println!("\n#### The solve on the world, whole: {label}\n");
    println!("µs a frame (the harness running the systems in plan order, apply nodes included), the median of {reps}\n");
    println!("| way | 1 thread | {} threads |", gang.threads());
    println!("|---|---|---|");
    for (i, name) in names.iter().enumerate() {
        println!("| {name} | {:.0} | {:.0} |", median(frames[&(i, 1)].clone()), median(frames[&(i, gang.threads())].clone()));
    }
    println!("\nBy stage, µs, the median of {reps} (the systems' bodies; a frame less their sum is the hand-off and the harness)\n");
    println!("| way | stage | 1 thread | {} threads |", gang.threads());
    println!("|---|---|---|---|");
    for (i, name) in names.iter().enumerate() {
        let mut names: Vec<&'static str> = stages.keys().filter(|(w, t, _)| *w == i && *t == 1).map(|(_, _, n)| *n).collect();
        names.sort_by_key(|n| STAGE_ORDER.iter().position(|o| o == n));
        let mut sums = [0.0f64; 2];
        for stage in names {
            let (a, b) = (median(stages[&(i, 1, stage)].clone()), median(stages[&(i, gang.threads(), stage)].clone()));
            sums[0] += a;
            sums[1] += b;
            println!("| {name} | {stage} | {a:.1} | {b:.1} |");
        }
        let (fa, fb) = (median(frames[&(i, 1)].clone()), median(frames[&(i, gang.threads())].clone()));
        println!("| {name} | frame less stages | {:.1} | {:.1} |", fa - sums[0], fb - sums[1]);
    }
}

const STAGE_ORDER: [&str; 12] = [
    "fused",
    "gather",
    "gather_bodies",
    "gather_turning",
    "gather_contacts",
    "prepare",
    "solver",
    "passes",
    "finish",
    "write_back",
    "scatter_bodies",
    "scatter_contacts",
];

flow! {
    pub struct Token: "spike::Token" {
        pub n: Vec<u64>,
    }
}

/// What a flow's hand-off costs: `k` systems that do nothing, each taking
/// or passing a flow, against `k` that take nothing; and the plan check.
fn handoff(k: usize, reps: usize) {
    let world = World::new();
    flows::reset(&world);
    let bare = Schedule { systems: (0..k).map(|i| (|_: &mut Cx| {}).system(&world, &format!("bare{i}"))).collect() };
    let mut systems = vec![(|_: &mut Cx, mut t: Make<Token>| t.n.push(1)).system(&world, "make")];
    for i in 1..k - 1 {
        systems.push(if i % 2 == 0 {
            (|_: &mut Cx, t: See<Token>| assert!(!t.n.is_empty())).system(&world, &format!("see{i}"))
        } else {
            (|_: &mut Cx, mut t: Pass<Token>| t.n[0] += 1).system(&world, &format!("pass{i}"))
        });
    }
    systems.push((|_: &mut Cx, t: Take<Token>| assert!(!t.n.is_empty())).system(&world, "take"));
    let flowing = Schedule { systems };
    let t = Instant::now();
    for _ in 0..100 {
        flows::check(&world, &flowing).unwrap();
    }
    let check = us(t) / 100.0;
    let mut times = [Vec::new(), Vec::new()];
    for _ in 0..reps {
        for (i, s) in [&bare, &flowing].into_iter().enumerate() {
            let t = Instant::now();
            for _ in 0..100 {
                s.run_sequential(&world);
            }
            times[i].push(us(t) * 1e3 / 100.0 / k as f64);
        }
    }
    let (a, b) = (median(times[0].clone()), median(times[1].clone()));
    println!(
        "\n#### A flow's hand-off\n\n{k} systems that do nothing, a frame run by the harness: {a:.0} ns a system bare, {b:.0} ns a system making, passing, seeing or taking a flow (each but the seers with an apply node), so {:.0} ns a use; the plan check {check:.1} µs for the {k}.",
        b - a
    );
}

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(21);
    let only = std::env::var("ONLY").unwrap_or_default();
    let threads: usize = std::env::var("THREADS").ok().and_then(|t| t.parse().ok()).unwrap_or(8);
    let pool = Arc::new(pool::Pool::new(threads));
    let exec: Arc<dyn Executor> = pool.clone();
    let gang = Workers::new(Some(exec.clone()));
    let warmed = Instant::now();
    while warmed.elapsed() < Duration::from_millis(300) {
        warm(&gang);
    }
    handoff(16, reps);
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    let pile = Scene::Pile { n: 10000, width: 401.0, stagger: true };
    let pyramid = Scene::Pyramid { base: 100 };
    let cases = [
        ("pile 10 000, falling, step 31", pile, 1, 30),
        ("pile 10 000, settled, step 430", pile, 400, 30),
        ("pyramid 5050, step 630", pyramid, 600, 30),
    ];
    for (name, scene, before, window) in cases.iter().filter(|c| only.is_empty() || c.0.contains(only.as_str())) {
        let mut ecs = ecs::Ecs::new(&manifest, scene, false, true);
        ecs.step(*before);
        let situ = in_situ(&mut ecs, *window);
        println!("\n### {name}\n\n{situ}");
        let world = ecs.engine().world();
        flows::reset(world);
        let [n, ..] = flows_physics::check_against_reference(world, &[Shape::Batch, Shape::Dyn]);
        world.set_executor(Some(exec.clone()));
        flows_physics::check_against_reference(world, &[Shape::Batch, Shape::Dyn]);
        world.set_executor(None);
        println!("\nThe pipeline bit for bit the solve as built at 1 and {threads} threads: {n} awake bodies, every one checked.");
        systems(name, world, reps, &exec);
        let input = capture(world);
        println!(
            "\n{} bodies, {} turning, {} contacts ({} with points).",
            input.bodies.len(),
            input.spinning.len(),
            input.constraints.len(),
            input.points.len()
        );
        solvers(name, &input, reps, &gang);
    }
}
