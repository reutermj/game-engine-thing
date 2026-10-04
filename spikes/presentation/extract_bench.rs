//! SPIKE (get-3hd.1): the extract and the hand-off, in the engine
//! (lockstep, the sequential scheduler's node timings): `spike_scene`
//! moves a share of its drawables each frame, `spike_draw` extracts the
//! draw list (full, incremental or delta), and `spike_sink` reads it from
//! another mod and copies it, as a presenter's upload would. Beside them,
//! the copy alone: the same bytes copied in this process.
//!
//!     ./bazel run --config=bench //spikes/presentation:extract_bench
//!
//! `FRAMES` (60) timed after 10, `RUNS` (3): the median run. `PATTERN=block`:
//! the rows that move each frame are one block, not spread over the rows.

use std::path::PathBuf;
use std::time::Instant;

use engine_loader::engine::Engine;

fn node(times: &str, name: &str) -> f64 {
    times
        .lines()
        .find_map(|l| l.strip_prefix(name).and_then(|rest| rest.trim().parse().ok()))
        .unwrap_or_else(|| panic!("no {name} in {times}"))
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

/// µs to copy `n` 32-byte items into a kept buffer, as the sink does.
fn copy_alone(n: usize) -> f64 {
    let from = vec![[0u32; 8]; n];
    let mut to: Vec<[u32; 8]> = Vec::with_capacity(n);
    let mut times = Vec::new();
    for _ in 0..50 {
        let t = Instant::now();
        to.clear();
        to.extend_from_slice(&from);
        std::hint::black_box(&to);
        times.push(t.elapsed().as_secs_f64() * 1e6);
    }
    median(times)
}

fn main() {
    let frames: u32 = std::env::var("FRAMES").ok().and_then(|f| f.parse().ok()).unwrap_or(60);
    let runs: usize = std::env::var("RUNS").ok().and_then(|f| f.parse().ok()).unwrap_or(3);
    let pattern = std::env::var("PATTERN").unwrap_or("spread".into());
    let manifest = engine_control::read_manifest(&std::env::var("EXTRACT_GAME").unwrap()).unwrap();
    println!(
        "µs a frame, the median of {runs} runs of {frames} frames, movers {pattern}: the scene's move, the extract, the sink's read\n"
    );
    println!("| drawables | moving | mode | move | extract | sink read | copy alone |");
    println!("|---|---|---|---|---|---|---|");
    for n in [1_000usize, 10_000, 100_000] {
        let copy = copy_alone(n);
        for churn in [1, 10, 100] {
            for mode in ["full", "incremental", "delta"] {
                let mut rows = Vec::new();
                for k in 0..runs {
                    let dir = std::env::temp_dir().join(format!("extract-bench-{}-{n}-{churn}-{mode}-{k}", std::process::id()));
                    let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
                    e.load_batch(&manifest.mods).expect("loading the scene");
                    e.send("spike_scene", &format!("spawn {n}")).unwrap();
                    e.send("spike_scene", &format!("churn {churn}")).unwrap();
                    e.send("spike_scene", &format!("pattern {pattern}")).unwrap();
                    e.send("spike_draw", &format!("mode {mode}")).unwrap();
                    e.send("lockstep", "step 10").unwrap();
                    e.send("sequential", "time reset").unwrap();
                    e.send("sequential", "time on").unwrap();
                    e.send("lockstep", &format!("step {frames}")).unwrap();
                    e.send("sequential", "time off").unwrap();
                    let times = e.send("sequential", "times").unwrap();
                    let stats = e.send("spike_sink", "stats").unwrap();
                    assert!(stats.contains(&format!("items {n} ")), "the sink saw {stats}");
                    if std::env::var("VERBOSE").is_ok() {
                        eprintln!("{n} {churn}% {mode}: {}", e.send("spike_draw", "stats").unwrap());
                    }
                    rows.push([
                        node(&times, "spike_scene::animate"),
                        node(&times, "spike_draw::extract"),
                        node(&times, "spike_sink::read"),
                    ]);
                    drop(e);
                    let _ = std::fs::remove_dir_all(dir);
                }
                let col = |i: usize| median(rows.iter().map(|r| r[i]).collect());
                println!("| {n} | {churn}% | {mode} | {:.1} | {:.1} | {:.1} | {copy:.1} |", col(0), col(1), col(2));
            }
        }
    }
}
