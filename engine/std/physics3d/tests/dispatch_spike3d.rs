//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): physics3d's
//! colored solve with its passes run by each dispatcher of physics2d's
//! `compare/dispatch.rs`, against one thread, every result checked bit
//! for bit against the one-thread solve; and where the threads' time goes
//! in each stage's tail (the thin last colors).
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics3d:dispatch_spike3d
//!
//! `THREADS` (1,2,4,8), `PROTOCOLS` (all), `REPS` (9), `ONLY=<scene>`,
//! `PARTS` (solve,trace), `PIN=1`, `WARM_MS` (300), as 2D's spike.

#![allow(dead_code)]

#[path = "../../physics2d/compare/dispatch.rs"]
mod dispatch;
#[path = "dispatch_3d.rs"]
mod dispatch_3d;
#[path = "../gjk.rs"]
mod gjk;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../../physics2d/tests/pool.rs"]
mod pool;
#[path = "../solver.rs"]
mod solver;

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Order, Quat, Reduce, Shape, Tuning, Vec3};

use std::sync::Arc;
use std::time::{Duration, Instant};

use dispatch::{Plant, Protocol, Trace};
use dispatch_3d::{How, Solve, bits};
use engine_ecs::Executor;

fn env<T: std::str::FromStr>(k: &str, default: T) -> T {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn warm(exec: &dyn Executor) {
    let warm = Duration::from_millis(env("WARM_MS", 300));
    let start = Instant::now();
    while start.elapsed() < warm {
        exec.run(exec.threads(), &|_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_millis(1) {
                std::hint::spin_loop();
            }
        });
    }
}

fn main() {
    let reps: usize = env("REPS", 9);
    let only = std::env::var("ONLY").unwrap_or_default();
    let counts: Vec<usize> = std::env::var("THREADS").unwrap_or("1,2,4,8".into()).split(',').map(|n| n.parse().unwrap()).collect();
    let protocols: Vec<Protocol> = match std::env::var("PROTOCOLS") {
        Ok(p) => p.split(',').map(Protocol::parse).collect(),
        Err(_) => Protocol::ALL.to_vec(),
    };
    let parts = std::env::var("PARTS").unwrap_or("solve,trace".into());
    let part = |p: &str| parts.split(',').any(|x| x == p);
    let execs: Vec<(usize, Arc<dyn Executor>)> = counts
        .iter()
        .map(|&n| {
            let exec: Arc<dyn Executor> = Arc::new(pool::Pool::new(n));
            if std::env::var("PIN").is_ok_and(|p| p == "1") {
                dispatch::pin(&*exec, &(0..n).collect::<Vec<_>>());
            }
            (n, exec)
        })
        .collect();
    println!("pinned {}, warm {} ms, {reps} reps", std::env::var("PIN").unwrap_or("0".into()), env("WARM_MS", 300));
    let scenes = [
        ("boxes 10 000, settled", "boxes 10000", vec![900, 915, 930]),
        ("planks 1000, settled", "planks 1000", vec![600, 615, 630]),
        ("boxes 10 000, falling", "boxes 10000", vec![61, 75, 90]),
    ];
    for (name, what, at) in scenes.iter().filter(|s| only.is_empty() || s.0.contains(only.as_str())) {
        let inputs = dispatch_3d::capture(what, at);
        let mut s = Solve::default();
        let bases: Vec<String> = inputs
            .iter()
            .map(|i| {
                let mut o = i.clone();
                s.solve(&mut o, How::One, None);
                bits(&o)
            })
            .collect();
        let whole = inputs.iter().zip(&bases).all(|(i, b)| {
            let mut o = i.clone();
            solver::solve(&mut o.bodies, &mut o.contacts, dispatch_3d::DT, &solver::Tuning::default());
            bits(&o) == *b
        });
        let mut probe = inputs[0].clone();
        let lanes = s.prepare(&mut probe);
        let blocks8 = if lanes { s.stage_blocks(&probe, 8) } else { Vec::new() };
        println!(
            "\n## {name}, steps {at:?}: {:?} contacts, in lanes {lanes}; the one-thread passes are `solver::solve` bit for bit: {}; \
             first step's [groups, overflow, batches, widest, narrowest]: {:?}; {} stages at 8 threads ({} of fewer than 8 blocks), {} blocks\n",
            inputs.iter().map(|i| i.contacts.len()).collect::<Vec<_>>(),
            if whole { "yes" } else { "NO" },
            s.layout(),
            blocks8.len(),
            blocks8.iter().filter(|&&b| b < 8).count(),
            blocks8.iter().sum::<usize>()
        );
        if part("solve") {
            warm(&engine_ecs::Scoped(1));
            let mut one = [0.0; 3];
            for input in &inputs {
                let runs: Vec<[f64; 3]> = (0..reps)
                    .map(|_| {
                        let mut i = input.clone();
                        s.solve(&mut i, How::One, None)
                    })
                    .collect();
                for (j, o) in one.iter_mut().enumerate() {
                    *o += median(runs.iter().map(|r| r[j]).collect()) / inputs.len() as f64;
                }
            }
            println!(
                "One thread, plain: {:.0} µs (prepare {:.0}, passes {:.0}, finish {:.0})\n",
                one.iter().sum::<f64>(),
                one[0],
                one[1],
                one[2]
            );
            print!("| threads |");
            for p in &protocols {
                print!(" {}: solve (passes) |", p.name());
            }
            println!();
            println!("|---|{}", "---|".repeat(protocols.len()));
            for (n, exec) in &execs {
                warm(&**exec);
                let mut all = vec![vec![Vec::new(); inputs.len()]; protocols.len()];
                for _ in 0..reps {
                    for (k, input) in inputs.iter().enumerate() {
                        for (m, out) in all.iter_mut().enumerate() {
                            let mut i = input.clone();
                            out[k].push(s.solve(&mut i, How::Across(protocols[m], &**exec, Plant::None), None));
                        }
                    }
                }
                print!("| {n} |");
                for (m, per) in all.iter().enumerate() {
                    let same = inputs.iter().zip(&bases).all(|(i, b)| {
                        let mut o = i.clone();
                        s.solve(&mut o, How::Across(protocols[m], &**exec, Plant::None), None);
                        bits(&o) == *b
                    });
                    let mut t = [0.0; 3];
                    for runs in per {
                        for (j, o) in t.iter_mut().enumerate() {
                            *o += median(runs.iter().map(|r| r[j]).collect()) / inputs.len() as f64;
                        }
                    }
                    print!(
                        " {:.0} ({:.0}, {:.2}×){} |",
                        t.iter().sum::<f64>(),
                        t[1],
                        one[1] / t[1],
                        if same { "" } else { " NOT BIT FOR BIT" }
                    );
                }
                println!();
            }
            println!();
        }
        if part("trace") && lanes {
            println!("| threads | protocol | passes wall | busy | idle | stage tails (thin) | handoffs (median, µs) | other |");
            println!("|---|---|---|---|---|---|---|---|");
            for (n, exec) in execs.iter().filter(|(n, _)| *n > 1) {
                warm(&**exec);
                for &p in &protocols {
                    let mut sum = dispatch::Idle::default();
                    for input in &inputs {
                        let mut i = input.clone();
                        s.prepare(&mut i);
                        let blocks = s.stage_blocks(&i, *n);
                        s.passes(&i, How::Across(p, &**exec, Plant::None), None);
                        let mut i = input.clone();
                        s.prepare(&mut i);
                        let trace = Trace::new();
                        s.passes(&i, How::Across(p, &**exec, Plant::None), Some(&trace));
                        let d = dispatch::idle(&trace, &blocks, *n);
                        let k = inputs.len() as f64;
                        sum.wall += d.wall / k;
                        sum.busy += d.busy / k;
                        sum.tail += d.tail / k;
                        sum.thin += d.thin / k;
                        sum.handoff += d.handoff / k;
                        sum.handoff_median += d.handoff_median / k;
                    }
                    let cap = sum.wall * *n as f64;
                    let idle = cap - sum.busy;
                    println!(
                        "| {n} | {} | {:.0} | {:.0} | {:.0} ({:.0}%) | {:.0} ({:.0}) | {:.0} ({:.2}) | {:.0} |",
                        p.name(),
                        sum.wall,
                        sum.busy,
                        idle,
                        100.0 * idle / cap,
                        sum.tail,
                        sum.thin,
                        sum.handoff,
                        sum.handoff_median,
                        idle - sum.tail - sum.handoff
                    );
                }
            }
            println!();
        }
    }
}
