//! The 2D mod's solve and its whole step, in the engine, on the scenes the
//! flows work was measured on (docs/architecture/flows.md, "Physics's
//! adoption"): a pile of 10 000 falling and settled, and a 5050 pyramid,
//! every body turning, and the settled pile with nothing turning (the
//! solve one contact at a time). Each run builds its scene in a fresh
//! engine, steps it to the window, and times the window: the mod's own
//! timings for its solve (gather, solver, write-back, its sides and
//! sleeping included) and the frame's wall time. The median of the runs.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:step_bench
//!
//! `RUNS` (5), `ONLY=<case>`, and `THREADS=1,8` (1) for the host's threads,
//! kept between steps (`tests/pool.rs`). Until the scheduler runs declared
//! shapes across threads (get-znt.34), the solve runs on one thread
//! whatever the host has; only the broadphase and narrowphase use them.

#[allow(dead_code)]
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[allow(dead_code)]
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

use std::sync::Arc;

pub use sim::{Dyn, Sim};

use scene::Scene;

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

/// One run's window, µs a step: the mod's solve and its parts, and the
/// frame.
fn run(manifest: &engine_control::Manifest, scene: &Scene, turning: bool, (before, window): (u32, u32), threads: usize) -> [f64; 8] {
    let mut ecs = ecs::Ecs::new(manifest, scene, false, turning);
    if threads > 1 {
        ecs = ecs.on_threads(Arc::new(pool::Pool::new(threads)));
    }
    ecs.step(before);
    ecs.reset();
    ecs.step(window);
    let stages: std::collections::HashMap<String, f64> = ecs.stages().into_iter().collect();
    let per = |k: &str| stages[k] / window as f64;
    let (gather, solver, write_back) = (per("ours:solve_gather"), per("ours:solver"), per("ours:write_back"));
    // The solver's systems, µs a step as the mod reports them.
    let text = ecs.engine().send("physics2d", "stages").unwrap();
    let part = |k: &str| {
        let mut words = text.split_whitespace();
        words.find(|w| *w == k).and_then(|_| words.next()).and_then(|v| v.parse().ok()).unwrap_or(f64::NAN)
    };
    [gather + solver + write_back, gather, solver, write_back, per("engine step"), part("prepare"), part("passes"), part("finish")]
}

fn main() {
    let runs: usize = std::env::var("RUNS").ok().and_then(|r| r.parse().ok()).unwrap_or(5);
    let only = std::env::var("ONLY").unwrap_or_default();
    let threads: Vec<usize> =
        std::env::var("THREADS").unwrap_or("1".into()).split(',').map(|t| t.trim().parse().expect("THREADS=1,8")).collect();
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    let pile = Scene::Pile { n: 10000, width: 401.0, stagger: true };
    let pyramid = Scene::Pyramid { base: 100 };
    let cases = [
        ("pile 10 000, falling, steps 2-31", pile, true, (1, 30)),
        ("pile 10 000, settled, steps 401-430", pile, true, (400, 30)),
        ("pyramid 5050, steps 601-630", pyramid, true, (600, 30)),
        ("pile 10 000 not turning, settled, steps 401-430", pile, false, (400, 30)),
    ];
    println!("µs a step, the median of {runs} runs\n");
    println!("| case | threads | solve | gather | solver (prepare, passes, finish) | write-back | whole step |");
    println!("|---|---|---|---|---|---|---|");
    for (name, scene, turning, at) in cases.iter().filter(|c| only.is_empty() || c.0.contains(only.as_str())) {
        for &n in &threads {
            let all: Vec<[f64; 8]> = (0..runs).map(|_| run(&manifest, scene, *turning, *at, n)).collect();
            let m = |k: usize| median(all.iter().map(|r| r[k]).collect());
            println!(
                "| {name} | {n} | {:.0} | {:.0} | {:.0} ({:.0}, {:.0}, {:.0}) | {:.0} | {:.0} |",
                m(0),
                m(1),
                m(2),
                m(5),
                m(6),
                m(7),
                m(3),
                m(4)
            );
        }
    }
}
