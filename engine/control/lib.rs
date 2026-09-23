//! The control protocol spoken over the engine's Unix domain socket.
//!
//! One request per connection: the client writes a single line and reads the reply
//! until EOF. The reply's first word is `ok` or `err`; anything after is free text.
//!
//! Requests:
//!   load <name> <path>   load a mod, or reload it if one with that name is running
//!   unload <name>        close and unload a mod
//!   list                 print loaded mods
//!   quit                 shut the engine down

use std::io;
use std::path::PathBuf;

#[derive(Debug, PartialEq, Eq)]
pub enum Request {
    Load { name: String, path: PathBuf },
    Unload { name: String },
    List,
    Quit,
}

impl Request {
    pub fn parse(line: &str) -> Result<Request, String> {
        let line = line.trim_end_matches(['\r', '\n']);
        let (cmd, rest) = line.split_once(' ').unwrap_or((line, ""));
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
            "list" => Ok(Request::List),
            "quit" => Ok(Request::Quit),
            _ => Err(format!("unknown command {cmd:?}")),
        }
    }

    pub fn to_line(&self) -> String {
        match self {
            Request::Load { name, path } => format!("load {name} {}\n", path.display()),
            Request::Unload { name } => format!("unload {name}\n"),
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

/// `$ENGINE_SOCKET`, else a per-user path under `$XDG_RUNTIME_DIR` or `/tmp`.
pub fn socket_path() -> PathBuf {
    if let Some(path) = std::env::var_os("ENGINE_SOCKET") {
        return path.into();
    }
    runtime_dir().join("control.sock")
}

/// Per-user scratch directory for the socket and copies of loaded libraries.
pub fn runtime_dir() -> PathBuf {
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
    stream.write_all(request.to_line().as_bytes())?;
    stream.shutdown(std::net::Shutdown::Write)?;
    let mut reply = String::new();
    stream.read_to_string(&mut reply)?;
    Ok(reply)
}
