//! SPIKE (get-3hd.1): starts a windowed lockstep pong (pacing on) on its
//! own socket and runtime directory, so it never meets another engine's,
//! and prints the commands an agent plays it with. Then it becomes the
//! engine. Two targets use it:
//!
//! - `:pong_window_launch`, `:pong_window`: one agent against `pong_ai`
//!   (AGENT.md), on `/run/user/1000/pong-window`.
//! - `:pong_versus_launch`, `:pong_versus` (`PONG_VERSUS=1`): two agents,
//!   in turns of `PONG_TURN_FRAMES` frames (AGENT_VERSUS.md), on
//!   `/run/user/1000/pong-versus`.
//!
//! `ENGINE_SOCKET` and `ENGINE_RUNTIME_DIR` are kept if set.

use std::os::unix::process::CommandExt;
use std::process::{Command, ExitCode};

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
    let versus = std::env::var("PONG_VERSUS").is_ok_and(|v| v == "1");
    let dir = if versus { "/run/user/1000/pong-versus" } else { "/run/user/1000/pong-window" };
    let socket = std::env::var("ENGINE_SOCKET").unwrap_or_else(|_| format!("{dir}/control.sock"));
    let runtime = std::env::var("ENGINE_RUNTIME_DIR").unwrap_or_else(|_| dir.into());
    let turn_frames = std::env::var("PONG_TURN_FRAMES").unwrap_or_else(|_| "6".into());
    let m = std::env::var("BUILD_WORKSPACE_DIRECTORY")
        .map_or("bazel-bin/engine/modctl/modctl".into(), |w| format!("{w}/bazel-bin/engine/modctl/modctl"));
    if versus {
        eprintln!(
            "pong for two agents, in turns of {turn_frames} frames, paced at 60 fps. Each plays with:\n\
             \n  export ENGINE_SOCKET={socket}\
             \n  {m} send pong_versus state            # observe, both sides named (or: show)\
             \n  {m} send lockstep turn left up        # submit: turn <left|right> <up|down|stay>\
             \n  {m} send lockstep turn                # whose move it is\
             \n  {m} quit                              # or close the window, or Escape\
             \n\nsee spikes/presentation/AGENT_VERSUS.md\n"
        );
    } else {
        eprintln!(
            "pong with a spectator window, lockstep, paced at 60 fps. An agent plays it with:\n\
             \n  export ENGINE_SOCKET={socket}\
             \n  {m} send pong_text state             # observe (or: show)\
             \n  {m} send pong_text up                # act: up | down | stay\
             \n  {m} send lockstep step 30            # advance 30 frames (half a second, played out)\
             \n  {m} send lockstep pace off           # pacing: on | off | <speed>\
             \n  {m} quit                             # or close the window, or Escape\
             \n\nsee spikes/presentation/AGENT.md\n"
        );
    }
    let mut command = Command::new(runfiles.join(engine));
    command
        .env("RUNFILES_DIR", &runfiles)
        .env("ENGINE_MANIFEST", manifest)
        .env("ENGINE_SOCKET", &socket)
        .env("ENGINE_RUNTIME_DIR", &runtime)
        // Pong's canvas (`pong_view.rs`, 28 px a court cell), so the window
        // is the game's resolution, fixed: tiling window managers float it.
        // WM_CLASS is "pong", "game-engine-thing".
        .env("SPIKE_WINDOW", "1184x680")
        .env("SPIKE_WINDOW_NAME", "pong");
    if versus {
        // Turns on in the bootstrap (`lockstep_window.rs`).
        command.env("SPIKE_TURNS", &turn_frames);
    } else {
        command.env_remove("SPIKE_TURNS");
    }
    let e = command.exec();
    eprintln!("pong_launch: starting the engine: {e}");
    ExitCode::FAILURE
}
