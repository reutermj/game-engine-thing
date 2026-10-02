//! The 3D mod's solve and its whole step, in the engine (lockstep, one
//! thread), on pile3d's scenes, the comparison's (`scenes.rs`): piles of
//! boxes falling and settled, planks and spheres settled, a locked pile,
//! and a stack. Each run builds its scene in a fresh engine, steps it to
//! the window, and times the window: the mod's own timings for its solve
//! (gather, solver, write-back) and the frame's wall time. The median of
//! the runs, as 2D's `step_bench` (physics2d/compare) measures the 2D mod.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics3d:step_bench
//!
//! `RUNS` (5), `ONLY=<case>`. `stages` is the breakdown of one run by
//! stage over a pile's whole life.

use std::path::PathBuf;
use std::time::Instant;

use engine_loader::engine::Engine;

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

struct Case {
    name: &'static str,
    /// What `pile3d build` takes: the scene and its n.
    build: &'static str,
    locked: bool,
    /// Steps before the window, and the window's.
    at: (u32, u32),
}

/// One run's window, µs a step: the solve and its parts, and the frame.
fn run(manifest: &engine_control::Manifest, case: &Case, k: usize) -> [f64; 5] {
    let dir = std::env::temp_dir().join(format!("physics3d-step-bench-{}-{k}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
    e.load_batch(&manifest.mods).expect("loading the pile");
    if case.locked {
        e.send("pile3d", "lock").unwrap();
    }
    e.send("pile3d", &format!("build {}", case.build)).unwrap();
    let (before, window) = case.at;
    if before > 0 {
        e.send("lockstep", &format!("step {before}")).unwrap();
    }
    e.send("physics3d", "reset_timings").unwrap();
    let t = Instant::now();
    e.send("lockstep", &format!("step {window}")).unwrap();
    let frame = t.elapsed().as_secs_f64() * 1e6 / window as f64;
    let stages = e.send("physics3d", "stages").unwrap();
    let (gather, solver, write_back) = (field(&stages, "solve_gather"), field(&stages, "solver"), field(&stages, "write_back"));
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
    [gather + solver + write_back, gather, solver, write_back, frame]
}

fn main() {
    let runs: usize = std::env::var("RUNS").ok().and_then(|r| r.parse().ok()).unwrap_or(5);
    let only = std::env::var("ONLY").unwrap_or_default();
    let manifest = engine_control::read_manifest(&std::env::var("PILE3D").unwrap()).unwrap();
    let cases = [
        Case { name: "boxes 10 000, falling, steps 61-90", build: "boxes 10000", locked: false, at: (60, 30) },
        Case { name: "boxes 10 000, settled, steps 901-930", build: "boxes 10000", locked: false, at: (900, 30) },
        Case { name: "planks 1000, settled, steps 601-630", build: "planks 1000", locked: false, at: (600, 30) },
        Case { name: "spheres 10 000, settled, steps 901-930", build: "spheres 10000", locked: false, at: (900, 30) },
        Case { name: "boxes 10 000 locked, settled, steps 901-930", build: "boxes 10000", locked: true, at: (900, 30) },
        Case { name: "stack 20, steps 301-330", build: "stack 20", locked: false, at: (300, 30) },
    ];
    println!("µs a step, the median of {runs} runs\n");
    println!("| case | solve | gather | solver | write-back | whole step |");
    println!("|---|---|---|---|---|---|");
    let mut k = 0;
    for case in cases.iter().filter(|c| only.is_empty() || c.name.contains(only.as_str())) {
        let all: Vec<[f64; 5]> = (0..runs)
            .map(|_| {
                k += 1;
                run(&manifest, case, k)
            })
            .collect();
        let m = |i: usize| median(all.iter().map(|r| r[i]).collect());
        println!("| {} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |", case.name, m(0), m(1), m(2), m(3), m(4));
    }
}
