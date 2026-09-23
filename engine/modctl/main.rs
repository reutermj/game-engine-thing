//! Sends a request to a running engine.
//!
//! With no arguments it loads the mod named by `ENGINE_MOD_NAME`, from the
//! library at runfiles path `ENGINE_MOD_RLOCATION`. `engine_mod` targets set
//! both, which is what makes `bazel run //mods/<name>` a hot reload.
//!
//! Otherwise the arguments are a request: `list`, `unload <name>`,
//! `load <name> <path>` or `quit`.

use std::path::Path;
use std::process::ExitCode;

use engine_control::Request;
use runfiles::Runfiles;

fn request_from_env() -> Result<Request, String> {
    let name = std::env::var("ENGINE_MOD_NAME")
        .map_err(|_| "usage: modctl list | load <name> <path> | unload <name> | quit")?;
    let rlocation = std::env::var("ENGINE_MOD_RLOCATION")
        .map_err(|_| "ENGINE_MOD_RLOCATION is not set")?;
    let runfiles = Runfiles::create().map_err(|e| format!("finding runfiles: {e}"))?;
    let path = runfiles
        .rlocation(&rlocation)
        .ok_or_else(|| format!("{rlocation} is not in runfiles"))?;
    // Resolve through the runfiles symlink so the engine gets an absolute path to
    // the real build output.
    let path = path
        .canonicalize()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(Request::Load { name, path })
}

fn request_from_args(args: &[String]) -> Result<Request, String> {
    let request = Request::parse(&args.join(" "))?;
    // Relative paths are relative to where the user ran `bazel run`, not the runfiles cwd.
    if let Request::Load { name, path } = request {
        let base = std::env::var_os("BUILD_WORKING_DIRECTORY").unwrap_or_else(|| ".".into());
        let path = Path::new(&base)
            .join(path)
            .canonicalize()
            .map_err(|e| e.to_string())?;
        return Ok(Request::Load { name, path });
    }
    Ok(request)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let request = if args.is_empty() {
        request_from_env()
    } else {
        request_from_args(&args)
    };
    let request = match request {
        Ok(request) => request,
        Err(e) => {
            eprintln!("modctl: {e}");
            return ExitCode::FAILURE;
        }
    };

    let reply = match engine_control::send(&request) {
        Ok(reply) => reply,
        Err(e) => {
            eprintln!(
                "modctl: can't reach the engine at {}: {e}\n\
                 start one with `bazel run //game` (or `bazel run //engine/loader:engine`)",
                engine_control::socket_path().display()
            );
            return ExitCode::FAILURE;
        }
    };
    let (status, msg) = reply.split_once(' ').unwrap_or((reply.trim_end(), ""));
    print!("{msg}");
    if status == "ok" { ExitCode::SUCCESS } else { ExitCode::FAILURE }
}
