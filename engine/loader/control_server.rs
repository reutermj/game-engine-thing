//! Serves the control socket. Polled from the main loop between frames, so
//! requests are handled at a point where no mod code is running.

use std::io::{self, Read, Write};
use std::os::unix::fs::DirBuilderExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::time::Duration;

use engine_control::Request;

use crate::engine::Engine;

pub struct ControlServer {
    listener: UnixListener,
    path: PathBuf,
}

impl ControlServer {
    pub fn bind(path: &Path) -> Result<ControlServer, String> {
        if let Some(dir) = path.parent() {
            std::fs::DirBuilder::new()
                .recursive(true)
                .mode(0o700)
                .create(dir)
                .map_err(|e| format!("creating {}: {e}", dir.display()))?;
        }
        if path.exists() {
            if UnixStream::connect(path).is_ok() {
                return Err(format!(
                    "another engine is already listening on {}",
                    path.display()
                ));
            }
            // Left behind by an engine that didn't exit cleanly.
            let _ = std::fs::remove_file(path);
        }
        let listener =
            UnixListener::bind(path).map_err(|e| format!("binding {}: {e}", path.display()))?;
        listener.set_nonblocking(true).map_err(|e| e.to_string())?;
        Ok(ControlServer { listener, path: path.into() })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Handles every pending request. Returns true if one asked the engine to quit.
    pub fn poll(&self, engine: &Engine) -> bool {
        let mut quit = false;
        loop {
            match self.listener.accept() {
                Ok((stream, _)) => {
                    if let Err(e) = serve(stream, engine, &mut quit) {
                        eprintln!("[engine] control connection failed: {e}");
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => return quit,
                Err(e) => {
                    eprintln!("[engine] accepting control connection: {e}");
                    return quit;
                }
            }
        }
    }
}

impl Drop for ControlServer {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

fn serve(stream: UnixStream, engine: &Engine, quit: &mut bool) -> io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(Duration::from_secs(1)))?;
    // Clients shut down their side after writing, so EOF ends the request.
    let mut text = String::new();
    (&stream).read_to_string(&mut text)?;

    let result = Request::parse(&text).and_then(|request| {
        match &request {
            Request::Batch { mods } => println!("[engine] batch of {} mod(s)", mods.len()),
            other => println!("[engine] {}", other.encode().trim_end()),
        }
        match request {
            Request::Load { name, path } => engine.load(&name, &path),
            Request::Batch { mods } => engine.load_batch(&mods),
            Request::Unload { name } => engine.unload(&name),
            Request::List => Ok(engine.list()),
            Request::Quit => {
                *quit = true;
                Ok("quitting".into())
            }
        }
    });
    let reply = match result {
        Ok(msg) => {
            println!("[engine] {}", msg.lines().next().unwrap_or(""));
            format!("ok {msg}\n")
        }
        Err(msg) => {
            eprintln!("[engine] error: {msg}");
            format!("err {msg}\n")
        }
    };
    (&stream).write_all(reply.as_bytes())
}
