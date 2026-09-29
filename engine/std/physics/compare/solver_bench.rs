//! The 2D solver alone, on its inputs as the comparison's scenes hand them
//! over, bodies turning: what each way of solving costs on the same work,
//! and how far its result is from the solve one contact at a time in pair
//! order (`Wide::Off`). The inputs are captured from the step on arrays
//! (`tests/arrays.rs`, bit for bit the mod's) at steps the comparison times,
//! so the times are the solver stage of its "turning" cases without the
//! rest of the step. What it measured: physics.md, "The solver's speed".
//!
//!     ./bazel run --config=bench //engine/std/physics/compare:solver_bench
//!
//! `ONLY=<scene text>` runs one scene; `REPS` (9) solves each input that
//! many times, reporting the median.
//!
//! `THREADS=1,2,4,8,16` times the default solved across that many threads
//! instead (`solver::solve_across`), each result checked bit for bit
//! against the solve on one: body, point and contact, every value. The
//! threads are kept between solves (`tests/pool.rs`, whose `SPIN_US` says
//! how long they spin before parking), or with `POOL=scoped` spawned for
//! each (`engine_ecs::Scoped`), or with `POOL=late` all on the calling
//! thread, one after another (what sharing costs with nothing shared:
//! `variants::Backwards`); each count's threads are kept busy for
//! `WARM_MS` (300) first, so the cores are clocked up
//! (docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md). Which
//! cores they run on is `taskset`'s: `taskset -c 0-7` is one CCD here
//! (docs/lore/cpus-16-to-31-are-the-same-cores-as-0-to-15.md). The scenes
//! then include rain 10 000, falling bodies arriving and leaving.

#[allow(dead_code)]
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
#[path = "../tests/pool.rs"]
mod pool;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
#[allow(dead_code)]
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};

pub use sim::{Dyn, Sim};

use scene::Scene;
use solver::{Constraint, PARAMS, Params, Points, SolverBody, Spinning, Wide};

#[derive(Clone)]
struct Input {
    bodies: Vec<SolverBody>,
    spinning: Vec<Spinning>,
    contacts: Vec<Constraint>,
    points: Vec<Points>,
}

/// The solver's inputs at steps `at` of `scene`, turning, solved as built.
fn capture(scene: &Scene, at: &[u32]) -> Vec<Input> {
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
    if let Scene::Rain { n, .. } = scene {
        for tick in 0..*at.iter().max().unwrap() {
            Sim::rain(&mut flat, &scene.rain(tick), *n as usize);
            Sim::step(&mut flat, 1);
        }
    } else {
        flat.step(*at.iter().max().unwrap());
    }
    drop(flat);
    Rc::try_unwrap(got).ok().unwrap().into_inner()
}

/// The ways of solving, by name; the first is what the others' results
/// are compared with.
fn solvers() -> Vec<(&'static str, Params)> {
    let at = |wide: Wide| Params { wide, ..PARAMS };
    vec![
        ("one at a time, pair order", at(Wide::Off)),
        ("by level, 1 wide", at(Wide::Levels(1))),
        ("by level, 4 wide", at(Wide::Levels(4))),
        ("by level, 8 wide", at(Wide::Levels(8))),
        ("colored, 1 wide", at(Wide::Colored(1))),
        ("colored, 4 wide (as built)", PARAMS),
        ("colored, 8 wide", at(Wide::Colored(8))),
        ("colored, 4 wide, 6 substeps", Params { substeps: 6, ..PARAMS }),
        ("colored, 4 wide, 1 relax", Params { relax: 1, ..PARAMS }),
        ("colored, 4 wide, Box2D's passes (4 substeps, 1 relax)", Params { substeps: 4, relax: 1, ..PARAMS }),
    ]
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// How far `out` is from `base`: bodies whose velocity differs at all, and
/// the most it differs by (linear, angular).
fn differs(base: &Input, out: &Input) -> (usize, f32, f32) {
    let mut n = 0;
    let (mut dv, mut dw) = (0.0f32, 0.0f32);
    for (a, b) in base.bodies.iter().zip(out.bodies.iter()) {
        if a.v.x.to_bits() != b.v.x.to_bits() || a.v.y.to_bits() != b.v.y.to_bits() {
            n += 1;
        }
        dv = dv.max((a.v - b.v).len());
    }
    for (a, b) in base.spinning.iter().zip(out.spinning.iter()) {
        if a.w.to_bits() != b.w.to_bits() {
            n += 1;
        }
        dw = dw.max((a.w - b.w).abs());
    }
    (n, dv, dw)
}

fn solve(params: &Params, i: &mut Input) {
    solver::solve_with(params, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT);
}

/// Every value a solve leaves, as bits: the bodies' velocities and moves,
/// the spinning's turns, every contact's and point's impulses.
fn bits(i: &Input) -> Vec<u32> {
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

/// `THREADS`: the default across threads against it on one (see the
/// module's docs).
fn across(inputs: &[Input], reps: usize, counts: &[usize]) {
    let pool = std::env::var("POOL").unwrap_or_default();
    let warm_ms: u64 = std::env::var("WARM_MS").ok().and_then(|r| r.parse().ok()).unwrap_or(300);
    let bases: Vec<Vec<u32>> = inputs
        .iter()
        .map(|i| {
            let mut o = i.clone();
            solve(&PARAMS, &mut o);
            bits(&o)
        })
        .collect();
    let one = time(inputs, reps, |i| solve(&PARAMS, i));
    println!("| threads | µs (median of {reps}, mean over inputs) | against one thread | bit for bit |");
    println!("|---|---|---|---|");
    println!("| one thread, `solve_with` | {one:.0} | 1.00× | – |");
    for &n in counts {
        let exec: Arc<dyn engine_ecs::Executor> = match pool.as_str() {
            "scoped" => Arc::new(engine_ecs::Scoped(n)),
            "late" => Arc::new(variants::Backwards(n)),
            _ => Arc::new(pool::Pool::new(n)),
        };
        let gang = engine_ecs::Workers::new(Some(exec));
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(warm_ms) {
            engine_ecs::Workers::run(&gang, n, |_| {
                let t = Instant::now();
                while t.elapsed() < Duration::from_millis(1) {
                    std::hint::spin_loop();
                }
            });
        }
        let dt = arrays::DT;
        let us = time(inputs, reps, |i| {
            solver::solve_across(&PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, dt, &gang)
        });
        let same = inputs.iter().zip(&bases).all(|(i, base)| {
            let mut o = i.clone();
            solver::solve_across(&PARAMS, (&mut o.bodies, &mut o.spinning), &mut o.contacts, &mut o.points, dt, &gang);
            bits(&o) == *base
        });
        println!("| {n} | {us:.0} | {:.2}× | {} |", one / us, if same { "yes" } else { "NO" });
    }
}

/// The median of `reps` runs of `f` on each input, averaged over them.
fn time(inputs: &[Input], reps: usize, f: impl Fn(&mut Input)) -> f64 {
    let mut us = 0.0;
    for input in inputs {
        let mut times = Vec::with_capacity(reps);
        for _ in 0..reps {
            let mut i = input.clone();
            let t = Instant::now();
            f(&mut i);
            times.push(t.elapsed().as_secs_f64() * 1e6);
        }
        us += median(times) / inputs.len() as f64;
    }
    us
}

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(9);
    let only = std::env::var("ONLY").unwrap_or_default();
    // The comparison's turning cases, at steps it times.
    let cases = [
        (Scene::Pile { n: 1000, width: 41.0, stagger: true }, vec![401, 430, 460]),
        (Scene::Pile { n: 10000, width: 401.0, stagger: true }, vec![401, 430, 460]),
        (Scene::Pyramid { base: 20 }, vec![601, 630, 660]),
        (Scene::Pyramid { base: 100 }, vec![601, 630, 660]),
    ];
    let counts: Option<Vec<usize>> =
        std::env::var("THREADS").ok().map(|t| t.split(',').map(|n| n.parse().expect("THREADS=1,2,..")).collect());
    let rain = (Scene::Rain { n: 10000, width: 801.0 }, vec![scene::RAIN_LIFE + 241, scene::RAIN_LIFE + 270, scene::RAIN_LIFE + 300]);
    let cases: Vec<_> = cases.into_iter().chain(counts.as_ref().map(|_| rain)).collect();
    let solvers = solvers();
    for (scene, at) in cases.iter().filter(|(s, _)| only.is_empty() || s.text() == only) {
        let inputs = capture(scene, at);
        let contacts = inputs.iter().map(|i| i.contacts.len()).sum::<usize>() / inputs.len();
        println!("\n### {}, turning, steps {at:?}: {contacts} contacts\n", scene.text());
        if let Some(counts) = &counts {
            across(&inputs, reps, counts);
            continue;
        }
        println!("| solver | µs (median of {reps}, mean over inputs) | ns a contact | bodies differing | most dv / dw |");
        println!("|---|---|---|---|---|");
        let bases: Vec<Input> = inputs
            .iter()
            .map(|i| {
                let mut o = i.clone();
                solve(&solvers[0].1, &mut o);
                o
            })
            .collect();
        for (name, params) in solvers.iter() {
            let mut us = 0.0;
            let mut worst = (0, 0.0f32, 0.0f32);
            for (input, base) in inputs.iter().zip(bases.iter()) {
                let mut times = Vec::with_capacity(reps);
                let mut out = input.clone();
                for _ in 0..reps {
                    let mut i = input.clone();
                    let t = Instant::now();
                    solve(params, &mut i);
                    times.push(t.elapsed().as_secs_f64() * 1e6);
                    out = i;
                }
                us += median(times) / inputs.len() as f64;
                let d = differs(base, &out);
                worst = (worst.0.max(d.0), worst.1.max(d.1), worst.2.max(d.2));
            }
            let per = us * 1e3 / contacts as f64;
            println!("| {name} | {us:.0} | {per:.1} | {} | {:.1e} / {:.1e} |", worst.0, worst.1, worst.2);
        }
    }
}
