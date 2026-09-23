//! The control protocol spoken over the engine's Unix domain socket, and the
//! game manifest both the engine and `modctl` read.
//!
//! One request per connection: the client writes the request and shuts down
//! its side, then reads the reply until EOF. The reply's first word is `ok` or
//! `err`; anything after is free text.
//!
//! Requests:
//!   load <name> <path>   load a mod, or reload it if one with that name is running
//!   batch                load or reload several mods at once, all or nothing:
//!   <name> <path>        one line per mod, after the `batch` line
//!   unload <name>        close and unload a mod
//!   send <name> <text>   deliver text to a mod; its reply is the reply
//!   list                 print loaded mods
//!   quit                 shut the engine down

use std::io;
use std::path::PathBuf;

use runfiles::Runfiles;

#[derive(Debug, PartialEq, Eq)]
pub enum Request {
    Load { name: String, path: PathBuf },
    Batch { mods: Vec<(String, PathBuf)> },
    Unload { name: String },
    Send { name: String, message: String },
    List,
    Quit,
}

impl Request {
    pub fn parse(text: &str) -> Result<Request, String> {
        let mut lines = text.lines().map(|l| l.trim_end_matches('\r'));
        let line = lines.next().unwrap_or("");
        let (cmd, rest) = line.split_once(' ').unwrap_or((line, ""));
        if cmd == "batch" {
            let mods = lines
                .filter(|l| !l.is_empty())
                .map(|l| {
                    // The path is the rest of the line so it may contain spaces.
                    let (name, path) = l.split_once(' ').ok_or("usage: <name> <path> after batch")?;
                    validate_name(name)?;
                    Ok((name.to_string(), PathBuf::from(path)))
                })
                .collect::<Result<Vec<_>, String>>()?;
            return Ok(Request::Batch { mods });
        }
        if lines.any(|l| !l.is_empty()) {
            return Err(format!("{cmd:?} takes a single line"));
        }
        match cmd {
            "load" => {
                // The path is the rest of the line so it may contain spaces.
                let (name, path) = rest
                    .split_once(' ')
                    .ok_or("usage: load <name> <path>")?;
                validate_name(name)?;
                Ok(Request::Load { name: name.into(), path: path.into() })
            }
            "unload" => {
                validate_name(rest)?;
                Ok(Request::Unload { name: rest.into() })
            }
            "send" => {
                let (name, message) = rest.split_once(' ').unwrap_or((rest, ""));
                validate_name(name)?;
                Ok(Request::Send { name: name.into(), message: message.into() })
            }
            "list" => Ok(Request::List),
            "quit" => Ok(Request::Quit),
            _ => Err(format!("unknown command {cmd:?}")),
        }
    }

    pub fn encode(&self) -> String {
        match self {
            Request::Load { name, path } => format!("load {name} {}\n", path.display()),
            Request::Batch { mods } => {
                let lines: String =
                    mods.iter().map(|(name, path)| format!("{name} {}\n", path.display())).collect();
                format!("batch\n{lines}")
            }
            Request::Unload { name } => format!("unload {name}\n"),
            Request::Send { name, message } => format!("send {name} {message}\n"),
            Request::List => "list\n".into(),
            Request::Quit => "quit\n".into(),
        }
    }
}

pub fn validate_name(name: &str) -> Result<(), String> {
    if name.is_empty() || name.contains(char::is_whitespace) {
        return Err(format!("invalid mod name {name:?}"));
    }
    Ok(())
}

/// `$ENGINE_SOCKET`, else `control.sock` in [`runtime_dir`].
pub fn socket_path() -> PathBuf {
    if let Some(path) = std::env::var_os("ENGINE_SOCKET") {
        return path.into();
    }
    runtime_dir().join("control.sock")
}

/// Scratch directory for the socket and copies of loaded libraries:
/// `$ENGINE_RUNTIME_DIR`, else a per-user directory under `$XDG_RUNTIME_DIR` or
/// `/tmp`. Tests set the override so they stay inside their sandbox and can
/// run in parallel.
pub fn runtime_dir() -> PathBuf {
    if let Some(dir) = std::env::var_os("ENGINE_RUNTIME_DIR") {
        return dir.into();
    }
    match std::env::var_os("XDG_RUNTIME_DIR") {
        Some(dir) => PathBuf::from(dir).join("game-engine-thing"),
        None => std::env::temp_dir().join(format!("game-engine-thing-{}", uid())),
    }
}

fn uid() -> u32 {
    use std::os::unix::fs::MetadataExt;
    std::fs::metadata("/proc/self").map(|m| m.uid()).unwrap_or(0)
}

/// Sends one request and returns the reply text, or an error if the engine isn't reachable.
pub fn send(request: &Request) -> io::Result<String> {
    use std::io::{Read, Write};
    let mut stream = std::os::unix::net::UnixStream::connect(socket_path())?;
    stream.write_all(request.encode().as_bytes())?;
    stream.shutdown(std::net::Shutdown::Write)?;
    let mut reply = String::new();
    stream.read_to_string(&mut reply)?;
    Ok(reply)
}

/// A game's manifest, written by `engine_game`: the mods to load, in
/// dependency order. Each line is `reload <label>`, `bootstrap <name>
/// <rlocationpath>` or `mod <name> <rlocationpath>`.
pub struct Manifest {
    /// The game's reload target, for error messages that tell the user to run it.
    pub reload: Option<String>,
    pub bootstrap: Option<String>,
    /// Every mod including the bootstrap mod, with its library resolved to an
    /// absolute path.
    pub mods: Vec<(String, PathBuf)>,
}

pub fn read_manifest(rlocation: &str) -> Result<Manifest, String> {
    let runfiles = Runfiles::create().map_err(|e| format!("finding runfiles: {e}"))?;
    let resolve = |rlocation: &str| -> Result<PathBuf, String> {
        let path = runfiles
            .rlocation(rlocation)
            .ok_or_else(|| format!("{rlocation} is not in runfiles"))?;
        // Absolute, since the engine may be sent it by a process with another cwd.
        path.canonicalize().map_err(|e| format!("{}: {e}", path.display()))
    };

    let manifest_path = resolve(rlocation)?;
    let text = std::fs::read_to_string(&manifest_path)
        .map_err(|e| format!("reading {}: {e}", manifest_path.display()))?;
    let mut manifest = Manifest { reload: None, bootstrap: None, mods: Vec::new() };
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        let mut parts = line.splitn(3, ' ');
        match (parts.next(), parts.next(), parts.next()) {
            (Some("reload"), Some(label), None) => manifest.reload = Some(label.into()),
            (Some(kind @ ("bootstrap" | "mod")), Some(name), Some(rlocation)) => {
                if kind == "bootstrap" {
                    manifest.bootstrap = Some(name.into());
                }
                manifest.mods.push((name.into(), resolve(rlocation)?));
            }
            _ => return Err(format!("malformed manifest line {line:?}")),
        }
    }
    Ok(manifest)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_request_survives_a_round_trip() {
        let requests = [
            Request::Load { name: "counter".into(), path: "/tmp/a b/libcounter.so".into() },
            Request::Batch {
                mods: vec![("a".into(), "/x/liba.so".into()), ("b".into(), "/y z/libb.so".into())],
            },
            Request::Batch { mods: vec![] },
            Request::Unload { name: "counter".into() },
            Request::Send { name: "pong_text".into(), message: "up".into() },
            Request::Send { name: "lockstep".into(), message: "step 10".into() },
            Request::List,
            Request::Quit,
        ];
        for request in requests {
            let text = request.encode();
            assert!(text.ends_with('\n'), "{text:?}");
            assert_eq!(Request::parse(&text), Ok(request));
        }
    }

    #[test]
    fn a_load_path_is_the_rest_of_the_line() {
        assert_eq!(
            Request::parse("load counter /with  two  spaces/lib.so\r\n"),
            Ok(Request::Load { name: "counter".into(), path: "/with  two  spaces/lib.so".into() })
        );
    }

    #[test]
    fn malformed_requests_are_rejected() {
        for line in [
            "",
            "load",
            "load counter",
            "unload",
            "unload two words",
            "reload counter",
            "LIST",
            "list\nquit",
            "batch\nno-path",
            "batch\n /p",
        ] {
            assert!(Request::parse(line).is_err(), "{line:?} parsed");
        }
    }
}
