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
//!
//! Every session is recorded (REVIEW.md): a directory under
//! `/run/user/1000/pong-sessions` (or `PONG_SESSIONS`) named for the time
//! and the game, holding `session.jsonl`. This writes its first line, what
//! was played (the game, the commit, the turn length); the bootstrap
//! appends the rest (`SPIKE_RECORD`). `PONG_RECORD=0` turns it off.

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
    let record = if std::env::var("PONG_RECORD").is_ok_and(|v| v == "0") {
        None
    } else {
        let workspace = std::env::var("BUILD_WORKSPACE_DIRECTORY").ok();
        match start_session(versus, &turn_frames, workspace.as_deref()) {
            Ok(path) => Some(path),
            Err(e) => {
                eprintln!("pong_launch: not recording: {e}");
                None
            }
        }
    };
    if let Some(path) = &record {
        eprintln!("recording to {path}\n  replay: ./bazel run //spikes/presentation:pong_replay -- {path}\n");
    }
    let mut command = Command::new(runfiles.join(engine));
    match &record {
        Some(path) => command.env("SPIKE_RECORD", path),
        None => command.env_remove("SPIKE_RECORD"),
    };
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

/// Makes the session's directory and writes the log's first line; returns
/// the log's path.
fn start_session(versus: bool, turn_frames: &str, workspace: Option<&str>) -> Result<String, String> {
    let base = std::env::var("PONG_SESSIONS").unwrap_or_else(|_| "/run/user/1000/pong-sessions".into());
    let game = if versus { "pong_versus" } else { "pong_window" };
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_err(|e| e.to_string())?.as_secs();
    let dir = format!("{base}/{}-{game}", utc(now));
    std::fs::create_dir_all(&dir).map_err(|e| format!("making {dir}: {e}"))?;
    let git = |args: &[&str]| -> Option<String> {
        let out = Command::new("git").arg("-C").arg(workspace?).args(args).output().ok()?;
        out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
    };
    let commit = git(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".into());
    // Uncommitted changes mean the commit alone doesn't say what was played.
    let dirty = git(&["status", "--porcelain", "--untracked-files=no"]).is_some_and(|s| !s.is_empty());
    let turns: u32 = if versus { turn_frames.parse().unwrap_or(6) } else { 0 };
    let path = format!("{dir}/session.jsonl");
    let meta = format!(
        "{{\"t\":\"meta\",\"game\":\"{game}\",\"target\":\"//spikes/presentation:{game}\",\"commit\":\"{commit}\",\
         \"dirty\":{dirty},\"turn_frames\":{turns},\"started\":{now},\"format\":1}}\n"
    );
    std::fs::write(&path, meta).map_err(|e| format!("writing {path}: {e}"))?;
    Ok(path)
}

/// `YYYYMMDD-HHMMSS`, UTC, from Unix seconds (Howard Hinnant's
/// civil-from-days), so session directories sort by time with no date
/// crate.
fn utc(secs: u64) -> String {
    let (days, rem) = ((secs / 86_400) as i64, secs % 86_400);
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}{m:02}{d:02}-{:02}{:02}{:02}", rem / 3600, rem / 60 % 60, rem % 60)
}
