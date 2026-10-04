//! The 3D mod's solve and its whole step, in the engine (lockstep, on the
//! scheduler's threads: the `threads` mod's pool, `ENGINE_THREADS` of
//! them, pinned to one CCD), on pile3d's scenes, the comparison's (`scenes.rs`): piles of
//! boxes falling and settled, planks and spheres settled, a locked pile,
//! and a stack. Each run builds its scene in a fresh engine, steps it to
//! the window, and times the window: the mod's own timings for its solve
//! (gather, solver, write-back) and the frame's wall time. The median of
//! the runs, as 2D's `step_bench` (physics2d/compare) measures the 2D mod.
//!
//!     ENGINE_THREADS=8 ./bazel run --config=bench //engine/std/physics3d:step_bench
//!
//! No `taskset`: the pool pins its own threads (docs/architecture/threads.md).
//!
//! `RUNS` (5), `ONLY=<case>`, `TUNE=<variant>` (pile3d's `tune`, so
//! `TUNE=lanes=0` times the solve one contact at a time on the same
//! build). `stages` is the breakdown of one run by
//! stage over a pile's whole life.
//!
//! After the table, each case's whole step broken down, µs a step: every
//! stage the mod times (`stages`), then every node of the frame as the
//! scheduler timed it (`sequential`'s `time on`: each system and its apply
//! node, where the re-sort and the structural changes land), and what the
//! frame spends outside its nodes (the bootstrap, the scheduler, the plan).

use std::path::PathBuf;
use std::time::Instant;

use engine_loader::engine::Engine;

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

/// Each `key value` pair in `text`, words or lines: a mod's timings.
fn timings(text: &str) -> Vec<(String, f64)> {
    let words: Vec<&str> = text.split_whitespace().collect();
    words.windows(2).filter(|w| w[0].parse::<f64>().is_err()).filter_map(|w| Some((w[0].to_string(), w[1].parse().ok()?))).collect()
}

/// What `stages` says that isn't a time: the last step's layout.
const COUNTS: [&str; 11] =
    ["pairs", "contacts", "points", "kept", "matched", "recycled", "groups", "overflow", "batches", "widest", "narrowest"];

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

/// One run's window, µs a step: the solve and its parts, and the frame;
/// then how the window's last step laid its contacts out in lanes
/// (physics3d's `stages`): contacts, groups (levels or colors), the
/// overflow's batches, all batches, and the most and fewest batches a
/// group has; zeros where it solved them whole.
fn run(manifest: &engine_control::Manifest, case: &Case, k: usize) -> ([f64; 14], Vec<(String, f64)>) {
    let dir = std::env::temp_dir().join(format!("physics3d-step-bench-{}-{k}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
    e.load_batch(&manifest.mods).expect("loading the pile");
    if let Ok(t) = std::env::var("TUNE") {
        e.send("pile3d", &format!("tune {t}")).unwrap();
    }
    if case.locked {
        e.send("pile3d", "lock").unwrap();
    }
    e.send("pile3d", &format!("build {}", case.build)).unwrap();
    let (before, window) = case.at;
    if before > 0 {
        e.send("lockstep", &format!("step {before}")).unwrap();
    }
    e.send("physics3d", "reset_timings").unwrap();
    e.send("sequential", "time reset").unwrap();
    e.send("sequential", "time on").unwrap();
    let t = Instant::now();
    e.send("lockstep", &format!("step {window}")).unwrap();
    let frame = t.elapsed().as_secs_f64() * 1e6 / window as f64;
    e.send("sequential", "time off").unwrap();
    let nodes = timings(&e.send("sequential", "times").unwrap());
    let stages = e.send("physics3d", "stages").unwrap();
    let mut rows: Vec<(String, f64)> =
        timings(&stages).into_iter().filter(|(k, _)| !COUNTS.contains(&k.as_str())).map(|(k, v)| (format!("stage {k}"), v)).collect();
    let in_nodes: f64 = nodes.iter().map(|(_, v)| v).sum();
    rows.extend(nodes);
    rows.extend([
        ("nodes, summed".to_string(), in_nodes),
        ("outside the nodes".to_string(), frame - in_nodes),
        ("whole step".to_string(), frame),
    ]);
    let (gather, solver, write_back) = (field(&stages, "solve_gather"), field(&stages, "solver"), field(&stages, "write_back"));
    let layout = ["contacts", "groups", "overflow", "batches", "widest", "narrowest"].map(|k| field(&stages, k));
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
    let [contacts, groups, overflow, batches, widest, narrowest] = layout;
    let (prepare, passes, finish) = (field(&stages, "prepare"), field(&stages, "passes"), field(&stages, "finish"));
    let times = [
        gather + solver + write_back,
        gather,
        solver,
        write_back,
        frame,
        contacts,
        groups,
        overflow,
        batches,
        widest,
        narrowest,
        prepare,
        passes,
        finish,
    ];
    (times, rows)
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
    println!(
        "| case | solve | gather | solver | write-back | whole step | contacts | groups | overflow | batches | widest | narrowest | prepare | passes | finish |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let mut k = 0;
    let mut broken = Vec::new();
    for case in cases.iter().filter(|c| only.is_empty() || c.name.contains(only.as_str())) {
        let (all, rows): (Vec<[f64; 14]>, Vec<Vec<(String, f64)>>) = (0..runs)
            .map(|_| {
                k += 1;
                run(&manifest, case, k)
            })
            .unzip();
        let m = |i: usize| median(all.iter().map(|r| r[i]).collect());
        let cells: Vec<String> = (0..14).map(|i| format!("{:.0}", m(i))).collect();
        println!("| {} | {} |", case.name, cells.join(" | "));
        broken.push((case.name, rows));
    }
    for (name, rows) in broken {
        println!("\n{name}: the step broken down, µs a step (the median of {runs} runs)\n");
        println!("| stage | µs |");
        println!("|---|---|");
        for (stage, _) in &rows[0] {
            let v = median(rows.iter().filter_map(|r| r.iter().find(|(x, _)| x == stage).map(|(_, v)| *v)).collect());
            println!("| {stage} | {v:.1} |");
        }
    }
}
