//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): can a general
//! task scheduler hand a `Passes` program's stages out across threads as
//! fast as `lanes::run_across`? The 2D mod's solve on captured inputs, its
//! passes run by each dispatcher of `dispatch.rs`, against
//! `solver::solve_across` and against one thread; every result checked bit
//! for bit against the one-thread solve.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:dispatch_spike
//!
//! `THREADS` (1,2,4,8), `PROTOCOLS` (all), `REPS` (9), `ONLY=<scene>`,
//! `PARTS` (solve,synthetic,trace,two: which tables), `POOL=kept|scoped`
//! (tests/pool.rs's kept threads, or spawned for each run), `PIN=1` (each
//! kept thread on its own CPU of 0-7), `WARM_MS` (300: the threads kept
//! busy first, so the cores are clocked up), `COLD_MS` (0: a sleep before
//! each timed run, as a frame's idle time would leave them).

#[allow(dead_code)]
#[path = "../tests/arrays.rs"]
mod arrays;
mod dispatch;
mod dispatch_2d;
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

use std::sync::Arc;
use std::time::{Duration, Instant};

pub use sim::{Dyn, Sim};

use dispatch::{Plant, Protocol, Trace};
use dispatch_2d::{How, Input, Solve, bits};
use engine_ecs::Executor;

fn env<T: std::str::FromStr>(k: &str, default: T) -> T {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

struct Bench {
    reps: usize,
    cold: Duration,
}

impl Bench {
    /// The median of `reps` runs of `f` on a fresh copy of each input, mean
    /// over the inputs; each element of what `f` returns apart.
    fn time<const N: usize>(&self, inputs: &[Input], mut f: impl FnMut(&mut Input) -> [f64; N]) -> [f64; N] {
        let mut out = [0.0; N];
        for input in inputs {
            let mut all: Vec<[f64; N]> = Vec::with_capacity(self.reps);
            for _ in 0..self.reps {
                let mut i = input.clone();
                if !self.cold.is_zero() {
                    std::thread::sleep(self.cold);
                }
                all.push(f(&mut i));
            }
            for (k, o) in out.iter_mut().enumerate() {
                *o += median(all.iter().map(|r| r[k]).collect()) / inputs.len() as f64;
            }
        }
        out
    }
}

fn executor(n: usize) -> Arc<dyn Executor> {
    let exec: Arc<dyn Executor> = match std::env::var("POOL").unwrap_or_default().as_str() {
        "scoped" => Arc::new(engine_ecs::Scoped(n)),
        _ => Arc::new(pool::Pool::new(n)),
    };
    if std::env::var("PIN").is_ok_and(|p| p == "1") {
        dispatch::pin(&*exec, &(0..n).collect::<Vec<_>>());
    }
    exec
}

/// `exec`'s threads kept busy for `WARM_MS`, so the cores are clocked up
/// (docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md): just
/// before each count's measurements, as solver_bench warms.
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
    let bench = Bench { reps: env("REPS", 9), cold: Duration::from_millis(env("COLD_MS", 0)) };
    let only = std::env::var("ONLY").unwrap_or_default();
    let counts: Vec<usize> = std::env::var("THREADS").unwrap_or("1,2,4,8".into()).split(',').map(|n| n.parse().unwrap()).collect();
    let protocols: Vec<Protocol> = match std::env::var("PROTOCOLS") {
        Ok(p) => p.split(',').map(Protocol::parse).collect(),
        Err(_) => Protocol::ALL.to_vec(),
    };
    let parts = std::env::var("PARTS").unwrap_or("solve,synthetic,trace,two".into());
    let part = |p: &str| parts.split(',').any(|x| x == p);
    println!(
        "pool {}, pinned {}, warm {} ms, cold {} ms, {} reps",
        std::env::var("POOL").unwrap_or("kept".into()),
        std::env::var("PIN").unwrap_or("0".into()),
        env("WARM_MS", 300),
        env("COLD_MS", 0),
        bench.reps
    );
    let execs: Vec<(usize, Arc<dyn Executor>)> = counts.iter().map(|&n| (n, executor(n))).collect();

    if part("synthetic") {
        synthetic(&execs, &protocols);
    }

    for (name, scene, at) in dispatch_2d::scenes().iter().filter(|s| only.is_empty() || s.0.contains(only.as_str())) {
        let inputs = dispatch_2d::capture(scene, at);
        let contacts: Vec<usize> = inputs.iter().map(|i| i.contacts.len()).collect();
        let mut s = Solve::default();
        let bases: Vec<Vec<u32>> = inputs
            .iter()
            .map(|i| {
                let mut o = i.clone();
                s.solve(&mut o, How::One, None);
                bits(&o)
            })
            .collect();
        let whole = inputs.iter().zip(&bases).all(|(i, b)| {
            let mut o = i.clone();
            solver::solve_with(&solver::PARAMS, (&mut o.bodies, &mut o.spinning), &mut o.contacts, &mut o.points, arrays::DT);
            bits(&o) == *b
        });
        let guarded = inputs.iter().filter(|i| !dispatch_2d::shareable(&i.bodies, &i.spinning)).count();
        let mut probe = inputs[0].clone();
        s.prepare(&mut probe);
        let blocks8 = s.stage_blocks(&probe, 8);
        let stages = blocks8.len();
        let thin = blocks8.iter().filter(|&&b| b < 8).count();
        println!(
            "\n## {name}, steps {at:?}: {contacts:?} contacts; the pipeline's one-thread solve is `solve_with` bit for bit: {}; \
             steps not shareable (one thread, get-znt.39): {guarded}; first step's batches: overflow {}, colors {:?}; \
             {stages} stages at 8 threads ({thin} of fewer than 8 blocks), {} blocks\n",
            if whole { "yes" } else { "NO" },
            s.layout().overflow,
            s.layout().colors,
            blocks8.iter().sum::<usize>()
        );
        if part("solve") {
            solve_table(&bench, &inputs, &bases, &execs, &protocols);
        }
        if part("trace") {
            trace_table(&inputs, &execs, &protocols);
        }
        if part("two") && inputs.len() >= 2 {
            two_table(&bench, &inputs, &execs, &protocols);
        }
        if part("guard") {
            guard(&inputs[0], &execs);
        }
    }
}

/// get-znt.39: the first input with its still body's velocity a negative
/// zero, solved across the most threads unguarded (shared though not
/// shareable) a few times: how often it differs from one thread.
fn guard(input: &Input, execs: &[(usize, Arc<dyn Executor>)]) {
    let mut i = input.clone();
    i.bodies.last_mut().unwrap().v = physics2d::Vec2::new(-0.0, -0.0);
    let mut s = Solve::default();
    let mut o = i.clone();
    s.solve(&mut o, How::One, None);
    let want = bits(&o);
    let (n, exec) = execs.last().unwrap();
    for p in [Protocol::Counter, Protocol::Graph] {
        let differ = (0..20)
            .filter(|_| {
                let mut o = i.clone();
                s.solve(&mut o, How::Unguarded(p, &**exec), None);
                bits(&o) != want
            })
            .count();
        println!("Unguarded, a still body's -0.0, {} at {n} threads: {differ} of 20 differ from one thread.", p.name());
    }
    println!();
}

/// The whole solve against `solve_across`, and the passes alone, µs.
fn solve_table(bench: &Bench, inputs: &[Input], bases: &[Vec<u32>], execs: &[(usize, Arc<dyn Executor>)], protocols: &[Protocol]) {
    let mut s = Solve::default();
    warm(&engine_ecs::Scoped(1));
    let one = bench.time(inputs, |i| s.solve(i, How::One, None));
    let [whole] = bench.time(inputs, |i| {
        let t = Instant::now();
        solver::solve_with(&solver::PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT);
        [t.elapsed().as_secs_f64() * 1e6]
    });
    println!(
        "One thread: the pipeline's solve {:.0} µs (prepare {:.0}, passes {:.0}, finish {:.0}); `solve_with` {whole:.0}.\n",
        one.iter().sum::<f64>(),
        one[0],
        one[1],
        one[2]
    );
    print!("| threads | `solve_across` |");
    for p in protocols {
        print!(" {}: solve (passes) |", p.name());
    }
    println!();
    println!("|---|---|{}", "---|".repeat(protocols.len()));
    for (n, exec) in execs {
        warm(&**exec);
        let gang = engine_ecs::Workers::new(Some(exec.clone()));
        // Method 0 is `solve_across`, then each protocol; interleaved rep by
        // rep, so a drift in the clocks reaches them all alike.
        let mut solve = |m: usize, i: &mut Input| -> [f64; 3] {
            if m == 0 {
                let t = Instant::now();
                solver::solve_across(&solver::PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT, &gang);
                return [0.0, t.elapsed().as_secs_f64() * 1e6, 0.0];
            }
            s.solve(i, How::Across(protocols[m - 1], &**exec, Plant::None), None)
        };
        let methods = protocols.len() + 1;
        let mut all = vec![vec![Vec::new(); inputs.len()]; methods];
        for _ in 0..bench.reps {
            for (k, input) in inputs.iter().enumerate() {
                for (m, out) in all.iter_mut().enumerate() {
                    let mut i = input.clone();
                    if !bench.cold.is_zero() {
                        std::thread::sleep(bench.cold);
                    }
                    out[k].push(solve(m, &mut i));
                }
            }
        }
        let same: Vec<bool> = (0..methods)
            .map(|m| {
                inputs.iter().zip(bases).all(|(i, b)| {
                    let mut o = i.clone();
                    solve(m, &mut o);
                    bits(&o) == *b
                })
            })
            .collect();
        let t: Vec<[f64; 3]> = all
            .iter()
            .map(|per| {
                let mut out = [0.0; 3];
                for runs in per {
                    for (j, o) in out.iter_mut().enumerate() {
                        *o += median(runs.iter().map(|r| r[j]).collect()) / inputs.len() as f64;
                    }
                }
                out
            })
            .collect();
        let flag = |m: usize| if same[m] { "" } else { " NOT BIT FOR BIT" };
        print!("| {n} | {:.0}{} |", t[0][1], flag(0));
        for m in 1..methods {
            if std::env::var("DETAIL").is_ok() {
                print!(" {:.0} ({:.0}, {:.0}, {:.0}){} |", t[m].iter().sum::<f64>(), t[m][0], t[m][1], t[m][2], flag(m));
            } else {
                print!(" {:.0} ({:.0}){} |", t[m].iter().sum::<f64>(), t[m][1], flag(m));
            }
        }
        println!();
    }
    println!();
}

/// Where the threads' time goes in the passes, from one traced run of each
/// input, mean over them: µs of thread time a solve.
fn trace_table(inputs: &[Input], execs: &[(usize, Arc<dyn Executor>)], protocols: &[Protocol]) {
    println!("| threads | protocol | passes wall | busy | idle | stage tails (thin) | handoffs (median, µs) | other |");
    println!("|---|---|---|---|---|---|---|---|");
    let mut s = Solve::default();
    for (n, exec) in execs.iter().filter(|(n, _)| *n > 1) {
        warm(&**exec);
        for &p in protocols {
            let mut sum = dispatch::Idle::default();
            for input in inputs {
                let mut i = input.clone();
                s.prepare(&mut i);
                let blocks = s.stage_blocks(&i, *n);
                // Run once untraced first, so the traced run's caches are as
                // a timed run's.
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

/// Two steps' passes in one dispatch against one after the other, µs.
fn two_table(bench: &Bench, inputs: &[Input], execs: &[(usize, Arc<dyn Executor>)], protocols: &[Protocol]) {
    println!("Two steps' passes (inputs 1 and 2), µs: one after the other / as one dispatch of two programs\n");
    print!("| threads |");
    for p in protocols {
        print!(" {} |", p.name());
    }
    println!();
    println!("|---|{}", "---|".repeat(protocols.len()));
    let pair = [inputs[0].clone(), inputs[1].clone()];
    for (n, exec) in execs.iter().filter(|(n, _)| *n > 1) {
        warm(&**exec);
        print!("| {n} |");
        for &p in protocols {
            let mut r = [0.0; 2];
            for (k, together) in [false, true].into_iter().enumerate() {
                let mut all = Vec::new();
                for _ in 0..bench.reps {
                    let mut solves = [Solve::default(), Solve::default()];
                    let mut ins = pair.clone();
                    for (s, i) in solves.iter_mut().zip(ins.iter_mut()) {
                        s.prepare(i);
                    }
                    all.push(dispatch_2d::two(&mut solves, &ins, p, &**exec, together));
                }
                r[k] = median(all);
            }
            print!(" {:.0} / {:.0} |", r[0], r[1]);
        }
        println!();
    }
    println!();
}

/// The dispatchers alone: 150 stages of 4 blocks a thread, each block no
/// work or 1 µs of spinning; µs a stage over the ideal.
fn synthetic(execs: &[(usize, Arc<dyn Executor>)], protocols: &[Protocol]) {
    const STAGES: usize = 150;
    println!("\n## The dispatchers alone: {STAGES} stages of 4 blocks a thread, µs a stage over the ideal (the median of 21)\n");
    print!("| threads | work a block |");
    for p in protocols {
        print!(" {} |", p.name());
    }
    println!();
    println!("|---|---|{}", "---|".repeat(protocols.len()));
    for (n, exec) in execs {
        warm(&**exec);
        for work in [0u64, 1000] {
            print!("| {n} | {} µs |", work / 1000);
            for &p in protocols {
                let blocks = 4 * n;
                let ideal = STAGES as f64 * (blocks.div_ceil(*n) as f64) * work as f64 / 1e3;
                let us = median((0..21).map(|_| dispatch::synthetic(p, &**exec, (STAGES, blocks, work))).collect());
                print!(" {:.2} |", (us - ideal) / STAGES as f64);
            }
            println!();
        }
    }
}
