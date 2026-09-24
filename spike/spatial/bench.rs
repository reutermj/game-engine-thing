//! Per scene and layout, per frame: broadphase, upkeep and query times, rows
//! moved, candidate pairs, and what queries visit.
//!
//!   ./bazel run -c opt //spike/spatial:bench

use spatial::layouts;
use spatial::sim::{self, Scene, Sim, Timing};

fn us(d: std::time::Duration, n: u32) -> f64 {
    d.as_secs_f64() * 1e6 / n.max(1) as f64
}

fn run(label: &str, scene: Scene, frames: u32) {
    println!("{label}: {} rows, {frames} frames", scene.rows.len());
    println!(
        "  {:<11} {:>6} {:>10} {:>10} {:>10} {:>10} {:>8} {:>10} {:>11}",
        "layout", "pages", "pairs us", "upkeep us", "respawn us", "queries us", "moved", "pages/q", "rows/q"
    );
    for mut l in layouts(&scene) {
        let mut s = Sim::new(&scene);
        let mut t = Timing::default();
        for _ in 0..frames {
            s.step(l.as_mut(), &mut t);
        }
        let q = (t.frames as usize * 64).max(1) as f64;
        println!(
            "  {:<11} {:>6} {:>10.1} {:>10.1} {:>10.2} {:>10.1} {:>8.1} {:>10.1} {:>11.1}",
            l.name(),
            l.pages(),
            us(t.pairs, t.frames),
            us(t.upkeep, t.frames),
            us(t.small_upkeep, t.small_writes),
            us(t.queries, t.frames),
            t.moved as f64 / t.frames as f64,
            t.query_visit.pages as f64 / q,
            t.query_visit.rows as f64 / q,
        );
    }
    println!();
}

fn main() {
    run("pile, falling", sim::pile(1000), 60);
    run("pile, falling and settling", sim::pile(1000), 400);
    run("platformer", sim::platformer(), 400);
    run("drift", sim::drift(10_000), 200);
}
