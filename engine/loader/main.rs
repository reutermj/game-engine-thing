//! The engine binary: a mod loader and nothing else.
//!
//! It loads the mods named in a manifest, then repeatedly steps the bootstrap
//! mod, which owns the frame loop and steps everything else. Between steps it
//! serves the control socket, which is how `bazel run //mods/<name>` loads or
//! hot-reloads a mod.
//!
//! The loop lives here rather than inside the bootstrap mod so that the
//! bootstrap mod can be reloaded too: code can't be swapped while it's on the stack.

mod control_server;
mod engine;
mod world;

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use engine_api::Status;
use runfiles::Runfiles;

use crate::control_server::ControlServer;
use crate::engine::Engine;

/// A mod listed in the manifest written by the `engine_game` rule.
struct Entry {
    name: String,
    path: PathBuf,
}

struct Manifest {
    bootstrap: Option<String>,
    mods: Vec<Entry>,
}

/// Each line is `bootstrap <name> <rlocationpath>` or `mod <name> <rlocationpath>`.
/// The bootstrap mod is loaded first, then the rest in order.
fn read_manifest(rlocation: &str) -> Result<Manifest, String> {
    let runfiles = Runfiles::create().map_err(|e| format!("finding runfiles: {e}"))?;
    let resolve = |rlocation: &str| -> Result<PathBuf, String> {
        let path = runfiles
            .rlocation(rlocation)
            .ok_or_else(|| format!("{rlocation} is not in runfiles"))?;
        // The engine keeps the path for reporting; make it independent of the cwd.
        path.canonicalize()
            .map_err(|e| format!("{}: {e}", path.display()))
    };

    let manifest_path = resolve(rlocation)?;
    let text = std::fs::read_to_string(&manifest_path)
        .map_err(|e| format!("reading {}: {e}", manifest_path.display()))?;
    let mut manifest = Manifest { bootstrap: None, mods: Vec::new() };
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        let mut parts = line.splitn(3, ' ');
        let (Some(kind), Some(name), Some(rlocation)) = (parts.next(), parts.next(), parts.next())
        else {
            return Err(format!("malformed manifest line {line:?}"));
        };
        let entry = Entry { name: name.into(), path: resolve(rlocation)? };
        match kind {
            "bootstrap" => {
                manifest.bootstrap = Some(entry.name.clone());
                manifest.mods.insert(0, entry);
            }
            "mod" => manifest.mods.push(entry),
            _ => return Err(format!("unknown manifest entry kind {kind:?}")),
        }
    }
    Ok(manifest)
}

fn main() -> ExitCode {
    // `engine_game` targets set ENGINE_MANIFEST; running the bare engine starts
    // it empty, waiting for mods over the control socket.
    let manifest = match std::env::var("ENGINE_MANIFEST") {
        Ok(rlocation) => match read_manifest(&rlocation) {
            Ok(manifest) => manifest,
            Err(e) => {
                eprintln!("[engine] {e}");
                return ExitCode::FAILURE;
            }
        },
        Err(_) => Manifest { bootstrap: None, mods: Vec::new() },
    };

    let server = match ControlServer::bind(&engine_control::socket_path()) {
        Ok(server) => server,
        Err(e) => {
            eprintln!("[engine] {e}");
            return ExitCode::FAILURE;
        }
    };
    println!("[engine] control socket at {}", server.path().display());

    let engine = Engine::new(
        manifest.bootstrap,
        engine_control::runtime_dir().join("libs"),
    );
    for entry in &manifest.mods {
        match engine.load(&entry.name, &entry.path) {
            Ok(msg) => println!("[engine] {msg}"),
            Err(e) => eprintln!("[engine] error: {e}"),
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
