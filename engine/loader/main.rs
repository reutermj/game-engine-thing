//! The engine binary: a mod loader and nothing else.
//!
//! It loads the mods named in a manifest, then repeatedly steps the bootstrap
//! mod, which owns the frame loop and steps everything else. Between steps it
//! serves the control socket, which is how `bazel run //mods/<name>` loads or
//! hot-reloads a mod.
//!
//! The loop lives here rather than inside the bootstrap mod so that the
//! bootstrap mod can be reloaded too: code can't be swapped while it's on the stack.

use std::process::ExitCode;
use std::time::Duration;

use engine_api::Status;
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

    let server = match ControlServer::bind(&engine_control::socket_path()) {
        Ok(server) => server,
        Err(e) => {
            eprintln!("[engine] {e}");
            return ExitCode::FAILURE;
        }
    };
    println!("[engine] control socket at {}", server.path().display());

    let engine = Engine::new(manifest.bootstrap, engine_control::runtime_dir().join("libs"));
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

    loop {
        if server.poll(&engine) {
            break;
        }
        match engine.step_bootstrap() {
            Some(Status::QUIT) => break,
            Some(_) => {}
            // Nothing to drive frames yet; idle until a request arrives.
            None => std::thread::sleep(Duration::from_millis(16)),
        }
    }

    engine.shutdown();
    println!("[engine] shut down");
    ExitCode::SUCCESS
}
