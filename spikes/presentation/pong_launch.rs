//! SPIKE (get-3hd.1): starts `:pong_window` (pong on the windowed
//! lockstep bootstrap, pacing on) on its own socket and runtime directory,
//! so it never meets another engine's, and prints the commands an agent
//! plays it with (AGENT.md has the rest). Then it becomes the engine.
//!
//! `ENGINE_SOCKET` and `ENGINE_RUNTIME_DIR` are kept if set.

use std::os::unix::process::CommandExt;
use std::process::{Command, ExitCode};

const RUNTIME_DIR: &str = "/run/user/1000/pong-window";
const SOCKET: &str = "/run/user/1000/pong-window/control.sock";

fn main() -> ExitCode {
    // This binary's runfiles hold the engine, the manifest and every mod;
    // the engine is pointed at them, since its own hold no game.
    let runfiles = match std::env::var_os("RUNFILES_DIR") {
        Some(dir) => std::path::PathBuf::from(dir),
        None => match std::env::current_exe() {
            Ok(exe) => exe.with_extension("runfiles"),
            Err(e) => {
                eprintln!("pong_launch: finding runfiles: {e}");
                return ExitCode::FAILURE;
            }
        },
    };
    let (Ok(engine), Ok(manifest)) = (std::env::var("PONG_ENGINE"), std::env::var("PONG_MANIFEST")) else {
        eprintln!("pong_launch: run it with `./bazel run //spikes/presentation:pong_window_launch`");
        return ExitCode::FAILURE;
    };
    let socket = std::env::var("ENGINE_SOCKET").unwrap_or_else(|_| SOCKET.into());
    let runtime = std::env::var("ENGINE_RUNTIME_DIR").unwrap_or_else(|_| RUNTIME_DIR.into());
    let modctl = std::env::var("BUILD_WORKSPACE_DIRECTORY")
        .map_or("bazel-bin/engine/modctl/modctl".into(), |w| format!("{w}/bazel-bin/engine/modctl/modctl"));
    eprintln!(
        "pong with a spectator window, lockstep, paced at 60 fps. An agent plays it with:\n\
         \n  export ENGINE_SOCKET={socket}\
         \n  {modctl} send pong_text state             # observe (or: show)\
         \n  {modctl} send pong_text up                # act: up | down | stay\
         \n  {modctl} send lockstep step 30            # advance 30 frames (half a second, played out)\
         \n  {modctl} send lockstep pace off           # pacing: on | off | <speed>\
         \n  {modctl} quit                             # or close the window, or Escape\
         \n\nsee spikes/presentation/AGENT.md\n"
    );
    let e = Command::new(runfiles.join(engine))
        .env("RUNFILES_DIR", &runfiles)
        .env("ENGINE_MANIFEST", manifest)
        .env("ENGINE_SOCKET", &socket)
        .env("ENGINE_RUNTIME_DIR", &runtime)
        // Pong's canvas (`pong_view.rs`, 28 px a court cell), so the window
        // is the game's resolution, fixed: tiling window managers float it.
        // WM_CLASS is "pong", "game-engine-thing".
        .env("SPIKE_WINDOW", "1184x680")
        .env("SPIKE_WINDOW_NAME", "pong")
        .exec();
    eprintln!("pong_launch: starting the engine: {e}");
    ExitCode::FAILURE
}
