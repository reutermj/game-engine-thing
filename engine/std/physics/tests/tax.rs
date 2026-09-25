//! What physics in the ECS costs over the same step on plain arrays:
//! `./bazel run -c opt //engine/std/physics:tax`.
//!
//! A pile runs in the engine (the physics mod, on the lockstep bootstrap)
//! until the frame to measure. Its state is then copied out (bodies, and
//! contacts with their impulses) into arrays, and both run the same frames:
//! the mod in the world, and the same step here, with the same narrowphase
//! and solver, contacts in the same order, and bodies as indices instead of
//! entities. They must end bit for bit the same, or the comparison is of two
//! different computations. Then each stage is timed on both.

#[path = "arrays.rs"]
mod arrays;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use arrays::{Arrays, Stages};
use engine_ecs::Entity;
use engine_loader::engine::Engine;
use physics::Position;

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

fn main() {
    let manifest = engine_control::read_manifest(&std::env::var("PILE").unwrap()).unwrap();
    const FRAMES: u32 = 60;
    println!("µs per step, {FRAMES} steps, -c opt, one thread; ECS / arrays\n");
    println!("| bodies | scene | contacts | frame | gravity | gather | broadphase | narrowphase | merge | solve: gather | solver | write back | outside systems |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (n, width) in [(1000u32, 40.0f32), (10000, 400.0)] {
        // Settled is still creeping (every body 1e-4 to 1e-2 a step); at rest
        // is still bit for bit, which the 10 000 are by about step 3000.
        for (scene, warmup) in [("falling", 1u32), ("settled", 400), ("at rest", 4000)] {
            let dir = std::env::temp_dir().join(format!("physics-tax-{}-{n}-{}", std::process::id(), scene.replace(' ', "-")));
            let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
            e.load_batch(&manifest.mods).expect("loading the pile");
            e.send("pile", &format!("widen {width}")).unwrap();
            e.send("pile", &format!("drop {n}")).unwrap();
            e.send("lockstep", &format!("step {warmup}")).unwrap();

            let mut arrays = Arrays::snapshot(e.world());
            let mut t = Stages::default();
            let start = Instant::now();
            for _ in 0..FRAMES {
                arrays.step(&mut t, solver::solve);
            }
            let array_frame = (start.elapsed().as_secs_f64() * 1e6 - t.fresh_sweep) / FRAMES as f64;

            e.send("physics", "reset_timings").unwrap();
            let start = Instant::now();
            e.send("lockstep", &format!("step {FRAMES}")).unwrap();
            let ecs_frame = start.elapsed().as_secs_f64() * 1e6 / FRAMES as f64;
            let (stats, stages) = (e.send("physics", "stats").unwrap(), e.send("physics", "stages").unwrap());

            // The same computation, or the numbers mean nothing.
            let ecs: HashMap<Entity, Position> = e.world().values::<Position>().unwrap().into_iter().collect();
            let differ = arrays
                .entity
                .iter()
                .zip(&arrays.pos)
                .filter(|(e, p)| ecs[e].x.to_bits() != p.x.to_bits() || ecs[e].y.to_bits() != p.y.to_bits())
                .count();
            assert_eq!(differ, 0, "{n} {scene}: {differ} bodies ended elsewhere than the arrays put them");
            assert_eq!(field(&stats, "contacts") as usize, arrays.contacts.len(), "{n} {scene}: contacts");

            let f = FRAMES as f64;
            let per_step = stats.split("us/step").nth(1).expect("timings in stats");
            let systems = field(per_step, "gravity") + field(per_step, "contacts") + field(per_step, "solve");
            let pair = |ecs: f64, arr: f64| format!("{ecs:.0} / {arr:.0}");
            println!("  (the arrays' sweep, sorting afresh each step: {:.0} µs)", t.fresh_sweep / f);
            println!(
                "| {n} | {scene} | {} | {} | {} | {} / – | {} | {} | {} | {} | {} | {} | {:.0} |",
                arrays.contacts.len(),
                pair(ecs_frame, array_frame),
                pair(field(per_step, "gravity"), t.gravity / f),
                field(&stages, "gather").round(),
                pair(field(&stages, "broadphase"), t.broadphase / f),
                pair(field(&stages, "narrowphase"), t.narrowphase / f),
                pair(field(&stages, "merge"), t.merge / f),
                pair(field(&stages, "solve_gather"), t.solve_gather / f),
                pair(field(&stages, "solver"), t.solver / f),
                pair(field(&stages, "write_back"), t.write_back / f),
                ecs_frame - systems,
            );
            drop(e);
            let _ = std::fs::remove_dir_all(dir);
        }
    }
    sleeping(&manifest, FRAMES);
}

/// Sleeping, which changes the simulation, so the ECS alone: the pile with
/// `Sleep` on, from when all of it is asleep, against the same pile awake at
/// the same step.
fn sleeping(manifest: &engine_control::Manifest, frames: u32) {
    const SPEED: f32 = 0.05;
    const TIME: f32 = 0.5;
    println!("\nSleeping (speed {SPEED}, {TIME} s), ECS only: µs per step, asleep / awake at the same step\n");
    println!("| bodies | asleep at step | frame | gravity | gather | broadphase | narrowphase | merge | solve: gather | solver | write back | outside systems | deepest overlap |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (n, width) in [(1000u32, 40.0f32), (10000, 400.0)] {
        let run = |sleep: bool, until: Option<u32>| {
            let dir = std::env::temp_dir().join(format!("physics-tax-{}-{n}-sleep-{sleep}", std::process::id()));
            let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
            e.load_batch(&manifest.mods).expect("loading the pile");
            e.send("pile", &format!("widen {width}")).unwrap();
            e.send("pile", &format!("drop {n}")).unwrap();
            if sleep {
                e.send("pile", &format!("sleep {SPEED} {TIME}")).unwrap();
            }
            let mut steps = 0;
            match until {
                Some(s) => {
                    e.send("lockstep", &format!("step {s}")).unwrap();
                    steps = s;
                }
                None => {
                    while field(&e.send("physics", "sleeping").unwrap(), "asleep") < n as f64 && steps < 6000 {
                        e.send("lockstep", "step 10").unwrap();
                        steps += 10;
                    }
                }
            }
            e.send("physics", "reset_timings").unwrap();
            let start = Instant::now();
            e.send("lockstep", &format!("step {frames}")).unwrap();
            let frame = start.elapsed().as_secs_f64() * 1e6 / frames as f64;
            let (stats, stages) = (e.send("physics", "stats").unwrap(), e.send("physics", "stages").unwrap());
            let per_step = stats.split("us/step").nth(1).expect("timings in stats").to_string();
            let systems = field(&per_step, "gravity") + field(&per_step, "contacts") + field(&per_step, "solve");
            let pile = e.send("pile", "stats").unwrap();
            drop(e);
            let _ = std::fs::remove_dir_all(dir);
            (steps, frame, stages, frame - systems, field(&pile, "deepest"))
        };
        let (at, frame, stages, outside, deepest) = run(true, None);
        let (_, frame_awake, stages_awake, outside_awake, deepest_awake) = run(false, Some(at));
        let pair = |k: &str| format!("{:.0} / {:.0}", field(&stages, k), field(&stages_awake, k));
        println!(
            "| {n} | {at} | {frame:.0} / {frame_awake:.0} | {} | {} | {} | {} | {} | {} | {} | {} | {outside:.0} / {outside_awake:.0} | {deepest:.3} / {deepest_awake:.3} |",
            pair("gravity"),
            pair("gather"),
            pair("broadphase"),
            pair("narrowphase"),
            pair("merge"),
            pair("solve_gather"),
            pair("solver"),
            pair("write_back"),
        );
    }
}
