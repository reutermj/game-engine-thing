//! ./bazel run --config=bench //bench/physics3d:bench -- [scenes] [sizes] [backends] [flags]
//!
//! Each positional argument is a comma-separated list or "all" (the default):
//! scenes are spheres, boxes, planks and rain; sizes are body counts (default
//! 1000,10000); backends are those in physics3d_bench::BACKENDS. Flags:
//! --iters8 sets every engine's iteration knob to 8, --rotate lets bodies
//! turn (locked by default), --sleep lets bodies
//! sleep, --runs N repeats each run and reports the median phase times
//! (default 3 up to 2000 bodies, 1 above), --tune=... runs a variant of ours
//! (physics3d::Tuning::parse). Prints markdown tables.
//!
//! --behave runs behaviour scenes instead (physics.md, "Quality beyond
//! settling"), whose size is the scene's parameter: ramp_hold, ramp_slide
//! and ramp_roll (any size), bounce (restitution in hundredths), ratio (how
//! many times as heavy), a table of what each engine did:
//!
//!     bench -- ramp_hold,ramp_slide,ramp_roll 1 all --rotate --behave
//!     bench -- bounce 25,50,75,100 all --rotate --behave
//!     bench -- ratio 10,100,1000 all --rotate --behave

use physics3d_bench::measure::{self, Run};
use physics3d_bench::scenes::{self, Kind, Scene};
use physics3d_bench::{BACKENDS, Config, Iters, make_backend};

fn list<'a>(arg: Option<&'a String>, all: &[&'a str]) -> Vec<&'a str> {
    match arg.map(String::as_str) {
        None | Some("all") => all.to_vec(),
        Some(s) => s.split(',').collect(),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (flags, positional): (Vec<&String>, Vec<&String>) = args.iter().partition(|a| a.starts_with("--"));
    let mut iters = Iters::Default;
    let mut sleep = false;
    let mut rotate = false;
    let mut runs = None;
    let mut tune = "";
    let mut behave = false;
    for f in flags {
        match f.as_str() {
            "--iters8" => iters = Iters::Eight,
            "--sleep" => sleep = true,
            "--rotate" => rotate = true,
            "--behave" => behave = true,
            f if f.starts_with("--runs=") => runs = f["--runs=".len()..].parse().ok(),
            f if f.starts_with("--tune=") => tune = f["--tune=".len()..].to_string().leak(),
            f => panic!("unknown flag {f}"),
        }
    }
    let scenes = list(positional.first().copied(), &["spheres", "boxes", "planks", "rain"]);
    let sizes: Vec<usize> = list(positional.get(1).copied(), &["1000", "10000"]).iter().map(|s| s.parse().expect("size")).collect();
    let backends = list(positional.get(2).copied(), BACKENDS);

    println!("iterations: {iters:?}, sleep: {sleep}, rotate: {rotate}, dt 1/60, single-threaded\n");
    for s in &scenes {
        let kind = Kind::parse(s).unwrap_or_else(|| panic!("unknown scene {s}"));
        for &n in &sizes {
            let scene = scenes::build(kind, n);
            let runs = runs.unwrap_or(if n <= 2000 { 3 } else { 1 });
            let config = Config { iters, sleep, max_bodies: (n + scene.statics.len() + 16) as u32, rotate, tune };
            if behave {
                behaviour(&scene, &config, &backends);
                continue;
            }
            let mut results = Vec::new();
            for b in &backends {
                let mut all: Vec<Run> = (0..runs)
                    .map(|_| {
                        let mut backend = make_backend(b, &config).unwrap_or_else(|| panic!("unknown backend {b}"));
                        measure::run(&scene, backend.as_mut())
                    })
                    .collect();
                let medians: Vec<f64> = scene
                    .phases
                    .iter()
                    .map(|(_, r)| median(all.iter().map(|run| run.phase_ms(r)).collect()))
                    .chain(std::iter::once(median(all.iter().map(|run| run.phase_ms(&(1..scene.steps))).collect())))
                    .collect();
                results.push((all.swap_remove(0), medians));
            }
            report(&scene, runs, &results);
        }
    }
}

/// Each engine on one behaviour scene, a row each.
fn behaviour(scene: &Scene, config: &Config, backends: &[&str]) {
    println!("## {} {}, {} steps\n", scene.kind.name(), scene.n, scene.steps);
    let runs: Vec<_> = backends
        .iter()
        .map(|b| physics3d_bench::behave::behave(scene, make_backend(b, config).unwrap_or_else(|| panic!("unknown backend {b}")).as_mut()))
        .collect();
    let names: Vec<&str> = runs[0].values.iter().map(|(k, _)| *k).collect();
    println!("| engine | {} |", names.join(" | "));
    println!("|---|{}", "---|".repeat(names.len()));
    for r in &runs {
        let cells: Vec<String> = r.values.iter().map(|(_, v)| format!("{v:.6}")).collect();
        println!("| {} | {} |", r.backend, cells.join(" | "));
    }
    println!();
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn report(scene: &Scene, runs: usize, results: &[(Run, Vec<f64>)]) {
    println!("## {}, {} bodies, {} steps (median of {})\n", scene.kind.name(), scene.n, scene.steps, runs);
    let heads: Vec<String> = scene
        .phases
        .iter()
        .map(|(name, r)| format!("{name} {}-{} ms", r.start, r.end))
        .chain(std::iter::once("whole run ms".to_string()))
        .collect();
    println!("| backend | solver | {} |", heads.join(" | "));
    println!("|---|---|{}", "---:|".repeat(heads.len()));
    for (run, ms) in results {
        let cells: Vec<String> = ms.iter().map(|m| format!("{m:.3}")).collect();
        println!("| {} | {} | {} |", run.backend, run.solver, cells.join(" | "));
    }
    println!();
    println!(
        "| backend | native pairs/body | geometric pairs/body | partners/body | contacts 0..8+ | not columns | pen max | pen mean | settled at step | escaped | moving | max speed | KE | KE a body | mean y | pen max / mean during | tilt ° | top moved |"
    );
    println!("|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|");
    for (run, _) in results {
        let q = &run.quality;
        let hist: Vec<String> = q.histogram.iter().map(|c| c.to_string()).collect();
        println!(
            "| {} | {:.2} | {:.2} | {:.2} | {} | {:.2} of {} | {:.4} | {:.4} | {} | {} | {} | {:.3} | {:.3} | {:.1e} | {:.2} | {:.4} / {:.4} | {:.1} | {:.3} |",
            run.backend,
            run.native_touching as f64 / q.bodies.max(1) as f64,
            q.pairs as f64 / q.bodies.max(1) as f64,
            q.contacts_per_body,
            hist.join("/"),
            q.not_columns,
            q.supported,
            q.pen_max,
            q.pen_mean,
            run.settled_at.map_or("never".into(), |s| s.to_string()),
            q.escaped,
            q.moving,
            q.max_speed,
            q.kinetic_energy,
            q.kinetic_energy / q.bodies.max(1) as f64,
            q.mean_height,
            run.pen_max_during,
            run.pen_mean_during,
            q.tilt,
            run.top_moved,
        );
    }
    println!();
    for (run, _) in results {
        if !run.stages.is_empty() {
            let s: Vec<String> = run.stages.iter().map(|(name, us)| format!("{name} {us:.0}")).collect();
            println!("- {} stages, mean µs per step: {}", run.backend, s.join(", "));
            for (name, range) in &scene.phases {
                let s: Vec<String> = run.phase_stages(range).iter().map(|(name, us)| format!("{name} {us:.0}")).collect();
                println!("  - {name}: {}", s.join(", "));
            }
        }
    }
    println!();
}
