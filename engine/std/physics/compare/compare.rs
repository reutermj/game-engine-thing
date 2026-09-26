//! //engine/std/physics against Box2D v3 and Rapier 2D, like for like, on
//! one thread: `./bazel run -c opt //engine/std/physics/compare`.
//!
//! Every engine builds the same scenes from `scene.rs` (the same bodies,
//! sizes, masses, friction, restitution and mixing, gravity and step, and
//! rotation locked in the other two, since ours has none), runs to the
//! step to measure, and is timed over the same steps: the whole step by the
//! wall clock, and its stages by the engine's own counters. Then how well
//! it settled, measured the same way for all of them (`quality.rs`).
//!
//! Ours runs twice: the physics mod in the engine, and the same step on
//! plain arrays (`tests/arrays.rs`), which must end bit for bit where the
//! mod does on the scenes without rain.
//!
//! In the environment: `REPS` (runs per case, 3), `ONLY` (cases whose
//! names contain it), `ENGINES` (engines whose labels contain one of its
//! comma-separated words), `SLEEP=1` (each engine's default sleeping
//! instead of none, the arrays left out), `LONG=1` (the piles also 4000
//! steps in, at rest), `VARIANTS` (more settings, comma-separated:
//! `box2d:<substeps>`, `rapier:<iterations>`, and `arrays:<solver>`, our
//! step with another solver from `variants.rs`), `SETTLE=<steps>` (how
//! soon each comes to rest instead of the timings; `TRACE=1` names what
//! moves again). What it found: docs/architecture/physics.md, "Against
//! other engines" and "Settling".

#[allow(dead_code)] // `Arrays::snapshot`, which only `:tax` uses.
#[path = "../tests/arrays.rs"]
mod arrays;
mod box2d;
mod ecs;
#[path = "../narrow.rs"]
mod narrow;
mod quality;
mod rapier;
mod scene;
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)] // The constants and tests only the experiments use.
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
mod variants;

use std::collections::BTreeMap;
use std::time::Instant;

use quality::Quality;
use scene::{RAIN_LIFE, Scene};

/// A dynamic body as every engine reports it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dyn {
    pub circle: bool,
    pub hx: f32,
    pub hy: f32,
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    /// Radians: 0 always, or a body turned that should have been locked.
    pub angle: f32,
}

/// One engine running one scene.
pub trait Sim {
    fn label(&self) -> String;
    fn step(&mut self, n: u32);
    /// Adds a rain step's arrivals and removes the oldest past `alive`.
    fn rain(&mut self, arrivals: &[scene::Spec], alive: usize);
    /// The dynamic bodies, in the order they came.
    fn bodies(&self) -> Vec<Dyn>;
    fn reset(&mut self);
    /// µs spent since `reset`: `broadphase`, `narrowphase` and `solver`
    /// first, as each engine's stages best map to them, then its own
    /// stages under its prefix, and `engine step`, the step as timed.
    fn stages(&self) -> Vec<(String, f64)>;
    /// Contacts the engine is solving that touch.
    fn contacts(&self) -> usize;
    /// Its own counts, for the notes.
    fn native(&self) -> String;
}

struct Case {
    name: String,
    scene: Scene,
    /// Steps before the ones timed, and how many are timed.
    warmup: u32,
    steps: u32,
}

/// One run of one engine on one case.
struct Run {
    label: String,
    /// µs per step: the wall clock around the steps, and the churn (rain's
    /// arrivals and removals) apart.
    frame: f64,
    churn: f64,
    stages: Vec<(String, f64)>,
    contacts: usize,
    native: String,
    quality: Quality,
    /// Mean and greatest distance a body moved over the timed steps, per
    /// second.
    drift: (f64, f32),
    /// The pyramid's: how far its boxes are from where they began.
    from_start: (f64, f32),
    bodies: Vec<Dyn>,
}

fn run(case: &Case, sim: &mut dyn Sim) -> Run {
    let alive = match case.scene {
        Scene::Rain { n, .. } => n as usize,
        _ => 0,
    };
    let rain = alive > 0;
    let mut tick = 0;
    let mut advance = |sim: &mut dyn Sim, n: u32, churn: &mut f64| {
        if !rain {
            sim.step(n);
            return;
        }
        for _ in 0..n {
            let start = Instant::now();
            sim.rain(&case.scene.rain(tick), alive);
            *churn += start.elapsed().as_secs_f64() * 1e6;
            tick += 1;
            sim.step(1);
        }
    };
    let mut churn = 0.0;
    advance(sim, case.warmup, &mut churn);
    let before = sim.bodies();
    sim.reset();
    churn = 0.0;
    let start = Instant::now();
    advance(sim, case.steps, &mut churn);
    let wall = start.elapsed().as_secs_f64() * 1e6;
    let bodies = sim.bodies();
    let seconds = case.steps as f64 * scene::DT as f64;
    let drift = if rain {
        (f64::NAN, f32::NAN)
    } else {
        let (mean, max) = quality::moved(&before, &bodies);
        (mean / seconds, max / seconds as f32)
    };
    let from_start = if let Scene::Pyramid { .. } = case.scene {
        let start: Vec<Dyn> = case
            .scene
            .build()
            .iter()
            .filter(|s| s.dynamic)
            .map(|s| Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: s.x, y: s.y, vx: 0.0, vy: 0.0, angle: 0.0 })
            .collect();
        quality::moved(&start, &bodies)
    } else {
        (f64::NAN, f32::NAN)
    };
    let steps = case.steps as f64;
    Run {
        label: sim.label(),
        frame: (wall - churn) / steps,
        churn: churn / steps,
        stages: sim.stages().into_iter().map(|(k, us)| (k, us / steps)).collect(),
        contacts: sim.contacts(),
        native: sim.native(),
        quality: quality::measure(&case.scene, &bodies),
        drift,
        from_start,
        bodies,
    }
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

type Make<'a> = Box<dyn Fn(&Scene) -> Box<dyn Sim> + 'a>;

fn main() {
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    let env = |k: &str| std::env::var(k).ok().filter(|v| !v.is_empty());
    let reps: usize = env("REPS").map_or(3, |r| r.parse().expect("REPS"));
    let sleep = env("SLEEP").is_some();

    // By name, which `ENGINES` picks from before any is built.
    let mut engines: Vec<(String, Make)> = vec![("ours (ECS)".into(), Box::new(|s| Box::new(ecs::Ecs::new(&manifest, s, sleep))))];
    if !sleep {
        engines.push(("ours (arrays)".into(), Box::new(|s| Box::new(ecs::Flat::new(s, Box::new(solver::solve), "ours (arrays)")))));
    }
    engines.push(("Box2D".into(), Box::new(move |s| Box::new(box2d::Box2d::new(s, 4, sleep)))));
    engines.push(("Rapier".into(), Box::new(move |s| Box::new(rapier::Rapier::new(s, 4, sleep)))));
    for v in env("VARIANTS").iter().flat_map(|v| v.split(",")) {
        if let Some(spec) = v.strip_prefix("arrays:") {
            let label = format!("ours (arrays) {spec}");
            let spec = spec.to_string();
            engines.push((label.clone(), Box::new(move |s| Box::new(ecs::Flat::new(s, variants::parse(&spec), &label)))));
            continue;
        }
        let (which, n) = v.split_once(":").expect("VARIANTS: box2d:<substeps>, rapier:<iterations> or arrays:<solver>");
        let n: usize = n.parse().expect("a number");
        match which {
            "box2d" => engines.push((format!("Box2D {n}"), Box::new(move |s| Box::new(box2d::Box2d::new(s, n as i32, sleep))))),
            "rapier" => engines.push((format!("Rapier {n}"), Box::new(move |s| Box::new(rapier::Rapier::new(s, n, sleep))))),
            other => panic!("no engine {other}"),
        }
    }
    if let Some(wanted) = env("ENGINES") {
        let wanted: Vec<String> = wanted.split(",").map(str::to_lowercase).collect();
        engines.retain(|(name, _)| wanted.iter().any(|w| name.to_lowercase().contains(w.as_str())));
    }

    let mut cases = Vec::new();
    // The pile as :tax runs it, whose columns stand in the other engines:
    // for how each settles, not for time.
    cases.push(Case { name: "columns 1000".into(), scene: Scene::Pile { n: 1000, width: 41.0, stagger: false }, warmup: 400, steps: 60 });
    for (n, width) in [(1000, 41.0), (10000, 401.0)] {
        let scene = Scene::Pile { n, width, stagger: true };
        cases.push(Case { name: format!("pile {n}, falling"), scene, warmup: 1, steps: 60 });
        cases.push(Case { name: format!("pile {n}, settled"), scene, warmup: 400, steps: 60 });
        if env("LONG").is_some() {
            cases.push(Case { name: format!("pile {n}, at rest"), scene, warmup: 4000, steps: 60 });
        }
    }
    for base in [20, 100] {
        let boxes = base * (base + 1) / 2;
        cases.push(Case { name: format!("pyramid {boxes}"), scene: Scene::Pyramid { base }, warmup: 600, steps: 60 });
    }
    // Whether the big one still stands after a minute.
    cases.push(Case { name: "pyramid 5050, a minute on".into(), scene: Scene::Pyramid { base: 100 }, warmup: 3600, steps: 60 });
    for (n, width) in [(1000, 81.0), (10000, 801.0)] {
        // Timed once it is full and the first drops have been removed a
        // while: the churn of a steady state.
        cases.push(Case { name: format!("rain {n}"), scene: Scene::Rain { n, width }, warmup: RAIN_LIFE + 240, steps: 60 });
    }
    if let Some(only) = env("ONLY") {
        cases.retain(|c| c.name.contains(&only));
    }
    if let Some(max) = env("SETTLE") {
        settle(&cases, &engines, max.parse().expect("SETTLE: steps"));
        return;
    }

    let sleeping = if sleep { "at each engine default" } else { "off everywhere" };
    println!("µs per step, -c opt, one thread, {reps} runs a case: median [min–max] of the runs. Sleeping {sleeping}.\n");
    for case in &cases {
        let mut runs: BTreeMap<usize, Vec<Run>> = BTreeMap::new();
        for rep in 0..reps {
            for (k, (_, make)) in engines.iter().enumerate() {
                let mut sim = make(&case.scene);
                let r = run(case, sim.as_mut());
                eprintln!("{} {rep}: {} {:.0} µs", case.name, r.label, r.frame);
                runs.entry(k).or_default().push(r);
            }
        }
        report(case, &runs);
    }
}

/// Below this every body counts as at rest: the sleep threshold of ours
/// and of Box2D.
const REST: f32 = 0.05;

/// How soon each engine comes to rest on each case's scene (rain left
/// out): stepped `max` steps, looked at every 10.
fn settle(cases: &[Case], engines: &[(String, Make)], max: u32) {
    let mut seen = Vec::new();
    for case in cases {
        if matches!(case.scene, Scene::Rain { .. }) || seen.contains(&case.scene) {
            continue;
        }
        seen.push(case.scene);
        println!("### settling: {}, {max} steps\n", case.scene.text());
        println!(
            "| engine | first at rest / at rest from step | fastest at 100 / 200 / 400 | energy at 400 | deepest at 400 (over 0.01) | deepest / mean at end (over 0.01) | energy at end | top moved | µs a step |"
        );
        println!("|---|---|---|---|---|---|---|---|---|");
        let start: Vec<Dyn> = case
            .scene
            .build()
            .iter()
            .filter(|s| s.dynamic)
            .map(|s| Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: s.x, y: s.y, vx: 0.0, vy: 0.0, angle: 0.0 })
            .collect();
        for (_, make) in engines {
            let mut sim = make(&case.scene);
            let (mut rest_from, mut first_rest, mut fastest) = (None, None, Vec::new());
            let mut q400 = None;
            let mut wall = 0.0;
            let mut step = 0;
            while step < max {
                let t = Instant::now();
                sim.step(10);
                wall += t.elapsed().as_secs_f64();
                step += 10;
                let bodies = sim.bodies();
                let top = bodies.iter().map(|b| (b.vx * b.vx + b.vy * b.vy).sqrt()).fold(0.0, f32::max);
                if top < REST {
                    rest_from.get_or_insert(step);
                    first_rest.get_or_insert(step);
                } else {
                    if rest_from.is_some() && std::env::var_os("TRACE").is_some() {
                        let (i, b) =
                            bodies.iter().enumerate().max_by(|a, b| a.1.vx.hypot(a.1.vy).total_cmp(&b.1.vx.hypot(b.1.vy))).unwrap();
                        eprintln!("{}: moving again at {step}: body {i} at {:.2}, {:.2} at {top:.3}", sim.label(), b.x, b.y);
                    }
                    rest_from = None;
                }
                if [100, 200, 400].contains(&step) {
                    fastest.push(top);
                }
                if step == 400 {
                    q400 = Some(quality::measure(&case.scene, &bodies));
                }
            }
            let bodies = sim.bodies();
            let q = quality::measure(&case.scene, &bodies);
            let q400 = q400.unwrap_or_default();
            let top = match case.scene {
                Scene::Pyramid { .. } => format!("{:.3}", bodies.last().unwrap().y - start.last().unwrap().y),
                _ => "–".into(),
            };
            let at = |s: Option<u32>| s.map_or("never".to_string(), |s| s.to_string());
            let rest = format!("{} / {}", at(first_rest), at(rest_from));
            let fast: Vec<String> = fastest.iter().map(|f| format!("{f:.3}")).collect();
            print!("| {} | {rest} | {} | {:.1e} | {:.4} ({}) ", sim.label(), fast.join(" / "), q400.energy, q400.max_depth, q400.deep);
            println!(
                "| {:.4} / {:.4} ({}) | {:.1e} | {top} | {:.0} |",
                q.max_depth,
                q.mean_depth,
                q.deep,
                q.energy,
                wall * 1e6 / step as f64
            );
            eprintln!("settle {}: {} rest from {rest}", case.scene.text(), sim.label());
        }
        println!();
    }
}

fn stage(r: &Run, k: &str) -> f64 {
    r.stages.iter().find(|(s, _)| s == k).map_or(f64::NAN, |(_, us)| *us)
}

fn report(case: &Case, runs: &BTreeMap<usize, Vec<Run>>) {
    println!("### {} ({} steps in, {} timed)\n", case.name, case.warmup, case.steps);
    println!("| engine | step | broadphase | narrowphase | solver | rest | churn | step and churn | contacts |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for rs in runs.values() {
        let mut frames: Vec<f64> = rs.iter().map(|r| r.frame).collect();
        let lo = frames.iter().copied().fold(f64::MAX, f64::min);
        let hi = frames.iter().copied().fold(0.0, f64::max);
        let f = median(&mut frames);
        let m = |k: &str| median(&mut rs.iter().map(|r| stage(r, k)).collect::<Vec<_>>());
        let (b, n, s) = (m("broadphase"), m("narrowphase"), m("solver"));
        let churn = median(&mut rs.iter().map(|r| r.churn).collect::<Vec<_>>());
        let total = median(&mut rs.iter().map(|r| r.frame + r.churn).collect::<Vec<_>>());
        let churn = if churn > 0.0 { format!("{churn:.0}") } else { "–".into() };
        let rest = f - b - n - s;
        print!("| {} | {f:.0} [{lo:.0}–{hi:.0}] | {b:.0} | {n:.0} | {s:.0} | {rest:.0} ", rs[0].label);
        println!("| {churn} | {total:.0} | {} |", rs[0].contacts);
    }
    println!();
    print!("| engine | bodies | contacts a body | islands | deepest | mean overlap | over 0.01 | mean speed | fastest | energy a body ");
    println!("| drift a second, mean / most | from start, mean / most | escaped |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for rs in runs.values() {
        let (r, q) = (&rs[0], &rs[0].quality);
        let pair = |(mean, max): (f64, f32)| if mean.is_nan() { "–".to_string() } else { format!("{mean:.4} / {max:.3}") };
        print!("| {} | {} | {:.2} | {} ", r.label, q.bodies, q.contacts_per_body, q.islands);
        print!("| {:.4} | {:.4} | {} | {:.4} | {:.3} | {:.2e} ", q.max_depth, q.mean_depth, q.deep, q.mean_speed, q.max_speed, q.energy);
        println!("| {} | {} | {} |", pair(r.drift), pair(r.from_start), q.escaped);
    }
    println!();
    for rs in runs.values() {
        let own: Vec<String> = rs[0]
            .stages
            .iter()
            .filter(|(k, _)| k.contains(":"))
            .map(|(k, _)| {
                let us = median(&mut rs.iter().map(|r| stage(r, k)).collect::<Vec<_>>());
                format!("{} {us:.0}", k.split_once(":").unwrap().1)
            })
            .collect();
        println!("- {}: {}; {}", rs[0].label, own.join(", "), rs[0].native);
    }
    // The arrays are the mod step, so they must agree with it bit for bit
    // where nothing arrives. Rain reuses entities, whose pair order then
    // is not the arrays.
    let ecs = runs.values().find(|rs| rs[0].label == "ours (ECS)");
    let flat = runs.values().find(|rs| rs[0].label == "ours (arrays)");
    if let (Some(e), Some(f)) = (ecs, flat)
        && !matches!(case.scene, Scene::Rain { .. })
    {
        let (e, f) = (&e[0].bodies, &f[0].bodies);
        let same = e.len() == f.len() && e.iter().zip(f).all(|(a, b)| a.x.to_bits() == b.x.to_bits() && a.y.to_bits() == b.y.to_bits());
        println!("- ECS and arrays {}", if same { "agree bit for bit" } else { "DIFFER" });
    }
    println!();
}
