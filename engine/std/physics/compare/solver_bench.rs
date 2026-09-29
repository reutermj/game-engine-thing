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

#[allow(dead_code)]
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[path = "../narrow.rs"]
mod narrow;
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
use std::time::Instant;

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
    flat.step(*at.iter().max().unwrap());
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
    let solvers = solvers();
    for (scene, at) in cases.iter().filter(|(s, _)| only.is_empty() || s.text() == only) {
        let inputs = capture(scene, at);
        let contacts = inputs.iter().map(|i| i.contacts.len()).sum::<usize>() / inputs.len();
        println!("\n### {}, turning, steps {at:?}: {contacts} contacts\n", scene.text());
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
