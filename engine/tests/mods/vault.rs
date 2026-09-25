//! Builds of `vault`, a resident mod whose transient part owns a thread. The
//! thread runs this library's code, which is only safe because a resident
//! mod's build stays mapped until the engine shuts down, when the transient
//! part is dropped and the thread joined.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::thread::JoinHandle;

use engine_api::{Cx, Mod, export_mod};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

static TRANSIENTS_MADE: AtomicU32 = AtomicU32::new(0);

engine_api::mod_state! {
    #[derive(Default)]
    struct Vault {}
}

#[derive(Default)]
pub struct Ticker {
    ticks: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Ticker {
    /// Starts the thread, named `tick-<mod name>` so a test can find it.
    fn start(&mut self, name: &str) {
        TRANSIENTS_MADE.fetch_add(1, Ordering::SeqCst);
        let (ticks, stop) = (self.ticks.clone(), self.stop.clone());
        let thread = std::thread::Builder::new().name(format!("tick-{name}")).spawn(move || {
            while !stop.load(Ordering::SeqCst) {
                ticks.fetch_add(1, Ordering::SeqCst);
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        });
        self.thread = thread.ok();
    }
}

impl Drop for Ticker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Mod for Vault {
    type Transient = Ticker;

    fn load(&mut self, ticker: &mut Ticker, cx: &mut Cx) {
        ticker.start(cx.name());
    }
}

impl vault::Vault for Vault {
    fn build(&mut self, _: &mut Ticker, _cx: &mut Cx) -> String {
        BUILD.into()
    }

    fn transients_made(&mut self, _: &mut Ticker, _cx: &mut Cx) -> u32 {
        TRANSIENTS_MADE.load(Ordering::SeqCst)
    }

    fn ticks(&mut self, ticker: &mut Ticker, _cx: &mut Cx) -> u64 {
        ticker.ticks.load(Ordering::SeqCst)
    }
}

export_mod!(Vault, provides = [vault::Vault]);
