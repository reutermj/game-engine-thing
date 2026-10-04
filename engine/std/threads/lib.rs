//! The scheduler's threads, as a resident mod: at load it makes a pool
//! (pool.rs) on one CCD and installs it as the world's executor, which is
//! how declared shapes (`Passes`, `ParMap`, `Reduce`) reach threads; at close it
//! takes it out and joins the threads, while this library is mapped. See
//! docs/architecture/threads.md.
//!
//! Resident, because its threads run this library's code (rayon's) between
//! frames, parked or spinning: code on a thread's stack can't be swapped
//! (hot-reload.md, "Resident mods"). Mods' code runs on them only inside a
//! dispatch, which returns before the system that made it does.
//!
//! Its settings, read at load (the game's environment) and changed between
//! frames by message, each change a new pool:
//!
//! - `ENGINE_THREADS=<n>`, `threads <n>`: threads, the calling one's
//!   included (default: one a core of one CCD);
//! - `ENGINE_PIN=0`, `pin off`: leave them where the OS puts them (tests,
//!   where many engines share the machine);
//! - `ENGINE_WARM_US=<µs>`, `warm <µs>`: how long a worker spins after a
//!   dispatch before it parks;
//! - `status`: the pool, and how many dispatches it has run.

use std::sync::Arc;
use std::time::Duration;

use engine_api::{Cx, Mod, export_mod};
use engine_threads::{Executor, OneCcd, Pool, Settings};

engine_api::mod_state! {
    #[derive(Default)]
    struct Threads {}
}

/// The pool, and what it was made with.
#[derive(Default)]
pub struct Host {
    settings: Settings,
    pool: Option<Arc<Pool>>,
}

impl Host {
    /// A pool made as `settings` say, installed in the world in place of
    /// this mod's last one; with one thread, no pool at all.
    fn start(&mut self, cx: &mut Cx) {
        self.stop(cx);
        let pool = Arc::new(Pool::new(&OneCcd, &self.settings));
        cx.log(describe(&pool));
        if pool.threads() > 1 {
            let executor: Arc<dyn Executor> = pool.clone();
            cx.world().install_executor(executor);
            self.pool = Some(pool);
        }
    }

    /// Takes this mod's pool out of the world, if it's still there (a test
    /// may have put its own in), and joins its threads.
    fn stop(&mut self, cx: &mut Cx) {
        let Some(pool) = self.pool.take() else { return };
        let ours: Arc<dyn Executor> = pool;
        cx.world().take_executor_if(&ours);
    }
}

fn describe(pool: &Pool) -> String {
    let placed = match pool.cpus() {
        [] => "placed by the OS".to_string(),
        cpus if pool.pins_caller() => format!("pinned to CPUs {cpus:?}, the frame's thread to the first"),
        cpus => format!("pinned to CPUs {cpus:?}, the first left for the frame's thread"),
    };
    format!("{} threads, {placed}, warm {} µs, {} dispatches", pool.threads(), pool.warm().as_micros(), pool.dispatches())
}

impl Mod for Threads {
    type Transient = Host;

    fn load(&mut self, host: &mut Host, cx: &mut Cx) {
        host.settings = Settings::from_env();
        host.start(cx);
    }

    fn close(&mut self, host: &mut Host, cx: &mut Cx) {
        host.stop(cx);
    }

    fn message(&mut self, host: &mut Host, cx: &mut Cx, message: &str) -> Result<String, String> {
        let words: Vec<&str> = message.split_whitespace().collect();
        let number = |w: &str| w.parse::<u64>().map_err(|_| format!("not a number: {w:?}"));
        let s = &mut host.settings;
        match words[..] {
            ["status"] => return Ok(host.pool.as_deref().map_or("1 thread: no pool".into(), describe)),
            ["threads", n] => s.threads = Some(number(n)?.max(1) as usize),
            ["warm", us] => s.warm = Duration::from_micros(number(us)?),
            ["pin", "on"] => s.pin = true,
            ["pin", "off"] => s.pin = false,
            _ => return Err("usage: threads <n> | warm <µs> | pin on|off | status".into()),
        }
        host.start(cx);
        Ok(host.pool.as_deref().map_or("1 thread: no pool".into(), describe))
    }
}

export_mod!(Threads);
