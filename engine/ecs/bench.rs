//! How much of the fire scenario runs at once, per variant: frame times on
//! one thread and on several, and one frame's timeline.
//!
//!   ./bazel run -c opt //engine/ecs:bench

use std::time::{Duration, Instant};

use ecs_game::{Burning, Options, SparseBurning, TableBurning};
use engine_ecs::World;
use engine_ecs::harness::{Schedule, Span, overlaps};

const WALKERS: usize = 20_000;
const LABELS: usize = 20_000;
/// Busy work per entity walked; `WORK=0` measures the framework alone.
fn work() -> u32 {
    std::env::var("WORK").ok().and_then(|w| w.parse().ok()).unwrap_or(200)
}
const FRAMES: usize = 20;
const THREADS: usize = 8;

fn setup<B: Burning>(anywhere: bool) -> (World, Schedule) {
    let w = ecs_game::world::<B>();
    ecs_game::populate(&w, WALKERS, LABELS);
    ecs_game::set_work(work());
    let s = ecs_game::schedule::<B>(&w, Options { anywhere });
    (w, s)
}

fn timeline(spans: &[Span]) -> String {
    let end = spans.iter().map(|s| s.end).max().unwrap_or_default().as_secs_f64().max(1e-9);
    let width = 64.0;
    let mut out = String::new();
    for s in spans {
        let (a, b) = (s.start.as_secs_f64() / end * width, s.end.as_secs_f64() / end * width);
        let (a, b) = (a as usize, (b.ceil() as usize).max(a as usize + 1));
        out += &format!(
            "  {:<16} t{} |{}{}{}|\n",
            s.node,
            s.thread,
            " ".repeat(a),
            "#".repeat(b - a),
            " ".repeat(64usize.saturating_sub(b))
        );
    }
    out
}

fn measure<B: Burning>(label: &str, anywhere: bool) {
    let (w, s) = setup::<B>(anywhere);
    let t = Instant::now();
    let mut seq_spans = Vec::new();
    for _ in 0..FRAMES {
        seq_spans = s.run_sequential(&w);
    }
    let seq = t.elapsed() / FRAMES as u32;

    let (w, s) = setup::<B>(anywhere);
    let t = Instant::now();
    let mut last = Vec::new();
    for _ in 0..FRAMES {
        last = s.run_parallel(&w, THREADS);
    }
    let par = t.elapsed() / FRAMES as u32;

    // The longest node bounds the frame from below, whatever the threads.
    let critical = last.iter().map(|s| s.end - s.start).max().unwrap_or(Duration::ZERO);
    println!("{label}");
    println!(
        "  sequential {:>7.2} ms   parallel ({THREADS} threads) {:>7.2} ms   speedup {:.2}x   longest node {:.2} ms",
        seq.as_secs_f64() * 1e3,
        par.as_secs_f64() * 1e3,
        seq.as_secs_f64() / par.as_secs_f64(),
        critical.as_secs_f64() * 1e3
    );
    println!("  last sequential frame, per node (ms):");
    for sp in &seq_spans {
        println!("    {:<16} {:>7.3}", sp.node, (sp.end - sp.start).as_secs_f64() * 1e3);
    }
    println!("  last parallel frame:");
    print!("{}", timeline(&last));
    let pairs: Vec<String> = overlaps(&last).into_iter().map(|(a, b)| format!("{a} || {b}")).collect();
    println!("  ran at once: {}\n", pairs.join(", "));
}

fn main() {
    println!("{WALKERS} walkers, {LABELS} labels, {} units of work per entity walked, {FRAMES} frames\n", work());
    measure::<SparseBurning>("Burning sparse", false);
    measure::<TableBurning>("Burning in tables, added through walkers' rows", false);
    measure::<TableBurning>("Burning in tables, added through a query matching everything", true);
}
