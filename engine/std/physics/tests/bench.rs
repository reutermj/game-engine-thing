//! Time per frame for piles of bodies, and where it goes, on the lockstep
//! bootstrap: `./bazel run -c opt //engine/std/physics:bench`.

use std::path::PathBuf;
use std::time::Instant;

use engine_loader::engine::Engine;

fn main() {
    let manifest = engine_control::read_manifest(&std::env::var("PILE").unwrap()).unwrap();
    for n in [250, 500, 1000] {
        let dir = std::env::temp_dir().join(format!("physics-bench-{}-{n}", std::process::id()));
        let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
        e.load_batch(&manifest.mods).expect("loading the pile");
        e.send("pile", &format!("drop {n}")).unwrap();
        // Falling and settling, then settled.
        for (label, frames) in [("falling", 60), ("settled", 240)] {
            e.send("physics", "reset_timings").unwrap();
            let start = Instant::now();
            e.send("lockstep", &format!("step {frames}")).unwrap();
            let ms = start.elapsed().as_secs_f64() * 1e3 / frames as f64;
            let stats = e.send("physics", "stats").unwrap();
            let per = stats.split("us/step").nth(1).unwrap_or("");
            println!("{n:>5} bodies, {label:<8} {ms:>7.3} ms/frame   us/step{per}");
        }
        println!("        {}", e.send("pile", "stats").unwrap());
        drop(e);
        let _ = std::fs::remove_dir_all(dir);
    }
}
