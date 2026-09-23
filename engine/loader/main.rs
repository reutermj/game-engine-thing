//! The engine binary: a mod loader and nothing else.
//!
//! It loads the mods named in a manifest, then hands the thread to the
//! bootstrap mod, which owns the frame loop: it steps the other mods, and
//! once a frame pumps the loader, which serves the control socket (how
//! `bazel run //mods/<name>` loads or hot-reloads a mod) and swaps builds.
//! The loader has no loop of its own. The bootstrap is resident, so it is
//! never swapped out from under itself.

use std::process::ExitCode;
use std::time::Duration;

use engine_api::{Pumped, Status};
use engine_control::Manifest;
use engine_loader::control_server::ControlServer;
use engine_loader::engine::Engine;

fn main() -> ExitCode {
    // `engine_game` targets set ENGINE_MANIFEST; running the bare engine starts
    // it empty, waiting for mods over the control socket.
    let manifest = match std::env::var("ENGINE_MANIFEST") {
        Ok(rlocation) => match engine_control::read_manifest(&rlocation) {
            Ok(manifest) => manifest,
            Err(e) => {
                eprintln!("[engine] {e}");
                return ExitCode::FAILURE;
            }
        },
        Err(_) => Manifest { reload: None, bootstrap: None, mods: Vec::new() },
    };

    let has_bootstrap = manifest.bootstrap.is_some();
    let engine = Engine::new(manifest.bootstrap, engine_control::runtime_dir().join("libs"));
    let server = match ControlServer::bind(&engine_control::socket_path(), engine.requests()) {
        Ok(server) => server,
        Err(e) => {
            eprintln!("[engine] {e}");
            return ExitCode::FAILURE;
        }
    };
    println!("[engine] control socket at {}", server.path().display());

    engine.set_reload_hint(manifest.reload);
    // One batch, so the game starts with every mod or not at all.
    if !manifest.mods.is_empty() {
        match engine.load_batch(&manifest.mods) {
            Ok(msg) => println!("[engine] {msg}"),
            Err(e) => {
                eprintln!("[engine] error: {e}");
                return ExitCode::FAILURE;
            }
        }
    }

    let code = if has_bootstrap {
        match engine.run_bootstrap() {
            Ok(Status::QUIT | Status::OK) => ExitCode::SUCCESS,
            Ok(_) => ExitCode::FAILURE,
            Err(e) => {
                eprintln!("[engine] error: {e}");
                ExitCode::FAILURE
            }
        }
    } else {
        // Nothing runs frames, so only requests need serving.
        while engine.pump(Duration::from_secs(1)) != Pumped::Quit {}
        ExitCode::SUCCESS
    };

    drop(server);
    engine.shutdown();
    println!("[engine] shut down");
    code
}
