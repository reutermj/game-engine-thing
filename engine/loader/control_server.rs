//! The control socket's transport. A thread accepts each connection, reads
//! its request, and queues it for the engine (`Engine::requests`), which
//! serves it the next time the bootstrap pumps the loader: the point where
//! builds can be swapped safely. The reply goes back on the same connection.
//!
//! The thread runs the loader's code, never a mod's, so no reload can unmap
//! what it runs.

use std::io::{self, Read, Write};
use std::os::unix::fs::DirBuilderExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::Duration;

use crate::engine::Pending;

pub struct ControlServer {
    path: PathBuf,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ControlServer {
    pub fn bind(path: &Path, requests: Sender<Pending>) -> Result<ControlServer, String> {
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
        let stop = Arc::new(AtomicBool::new(false));
        let thread = std::thread::Builder::new()
            .name("control".into())
            .spawn({
                let stop = stop.clone();
                move || accept(listener, requests, &stop)
            })
            .map_err(|e| format!("starting the control thread: {e}"))?;
        Ok(ControlServer { path: path.into(), stop, thread: Some(thread) })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for ControlServer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        // Wakes the thread from `accept`, so it sees `stop` and returns.
        let _ = UnixStream::connect(&self.path);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        let _ = std::fs::remove_file(&self.path);
    }
}

fn accept(listener: UnixListener, requests: Sender<Pending>, stop: &AtomicBool) {
    for stream in listener.incoming() {
        if stop.load(Ordering::SeqCst) {
            return;
        }
        let stream = match stream {
            Ok(stream) => stream,
            Err(e) => {
                eprintln!("[engine] accepting control connection: {e}");
                continue;
            }
        };
        let text = match read_request(&stream) {
            Ok(text) => text,
            Err(e) => {
                eprintln!("[engine] control connection failed: {e}");
                continue;
            }
        };
        let reply = Box::new(move |reply: String| {
            if let Err(e) = (&stream).write_all(reply.as_bytes()) {
                eprintln!("[engine] control connection failed: {e}");
            }
        });
        if requests.send(Pending { text, reply }).is_err() {
            return; // The engine is gone.
        }
    }
}

fn read_request(stream: &UnixStream) -> io::Result<String> {
    stream.set_read_timeout(Some(Duration::from_secs(1)))?;
    // Clients shut down their side after writing, so EOF ends the request.
    let mut text = String::new();
    (&*stream).read_to_string(&mut text)?;
    Ok(text)
}
