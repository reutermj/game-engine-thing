//! Where a 3D pile's step goes, by stage, in the engine (the physics3d mod
//! on pile3d's scenes, lockstep, one thread):
//! `./bazel run --config=bench //engine/std/physics3d:stages -- [n] [spheres|boxes|planks|rain] [locked]`.
//! The comparison with other engines is //engine/std/physics3d/compare; this is the
//! breakdown behind it.

use std::path::PathBuf;
use std::time::Instant;

use engine_loader::engine::Engine;

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let n: usize = args.get(1).and_then(|a| a.parse().ok()).unwrap_or(10_000);
    let kind = args.get(2).map_or("boxes", String::as_str);
    let manifest = engine_control::read_manifest(&std::env::var("PILE3D").unwrap()).unwrap();
    let dir = std::env::temp_dir().join(format!("physics3d-stages-{}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
    e.load_batch(&manifest.mods).expect("loading the pile");
    if args.iter().any(|a| a == "locked") {
        e.send("pile3d", "lock").unwrap();
    }
    e.send("pile3d", &format!("build {kind} {n}")).unwrap();
    println!("{n} {kind}: µs per step by stage (the mod in the engine, one thread)");
    println!(
        "| steps | frame | gravity | gather | broadphase | narrowphase | merge | solve: gather | solver | write back | outside systems | pairs | contacts |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let mut done = 0;
    for (from, to) in [(0, 60), (60, 300), (300, 400), (400, 900), (900, 1000)] {
        if from > done {
            e.send("lockstep", &format!("step {}", from - done)).unwrap();
        }
        e.send("physics3d", "reset_timings").unwrap();
        let t = Instant::now();
        e.send("lockstep", &format!("step {}", to - from)).unwrap();
        let frame = t.elapsed().as_secs_f64() * 1e6 / (to - from) as f64;
        done = to;
        let (stats, stages) = (e.send("physics3d", "stats").unwrap(), e.send("physics3d", "stages").unwrap());
        let per_step = stats.split("us/step").nth(1).expect("timings in stats");
        let systems = field(per_step, "gravity") + field(per_step, "contacts") + field(per_step, "solve");
        let cells: Vec<String> = ["gravity", "gather", "broadphase", "narrowphase", "merge", "solve_gather", "solver", "write_back"]
            .iter()
            .map(|k| format!("{:.0}", field(&stages, k)))
            .collect();
        println!(
            "| {from}-{to} | {frame:.0} | {} | {:.0} | {} | {} |",
            cells.join(" | "),
            frame - systems,
            field(&stages, "pairs"),
            field(&stages, "contacts")
        );
    }
    println!("\n{}", e.send("pile3d", "stats").unwrap());
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
}
