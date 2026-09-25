//! A persistent thread pool: an `engine_ecs::Executor` whose threads wait
//! between runs, for the benchmarks and the tests of host threads running
//! a mod's tasks. Not in `engine_ecs`: see `Pool`.

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use engine_ecs::Executor;

/// Threads kept between runs, waiting for the next: what a resident
/// scheduler would own. A worker spins for a while (`spin`) after a run, since a
/// step's stages come microseconds apart, then parks. The one unsafe
/// thing is handing workers the run's closure, which borrows the caller's
/// stack, as a `'static` reference: sound because the run is closed, and
/// every worker out of it, before `run` returns. It's the part a persistent
/// pool can't do in safe Rust, and why it isn't in `engine_ecs`, whose
/// unsafe code must not depend on concurrency (storage.md).
pub struct Pool {
    shared: Arc<Shared>,
    workers: Vec<std::thread::JoinHandle<()>>,
}

struct Shared {
    /// Bumped by each run, and on stopping: what idle workers watch.
    generation: AtomicU64,
    job: Mutex<Option<(&'static (dyn Fn(usize) + Sync), usize)>>,
    next: AtomicUsize,
    /// Whether the current run takes workers; a worker that isn't `busy`
    /// before it closes never touches its closure.
    open: AtomicBool,
    busy: AtomicUsize,
    parked: Mutex<usize>,
    wake: Condvar,
    stop: AtomicBool,
    panicked: AtomicBool,
    /// Threads, the caller's included, and whether task `k` is always
    /// thread `k % threads`'s (`STICKY` set) rather than whichever takes it
    /// first: the same pages to the same core every step, so a stage finds
    /// them in its own cache, at the price of waiting for a late thread.
    threads: usize,
    sticky: bool,
    completed: AtomicUsize,
}

/// How long an idle worker spins before it parks: `SPIN_US`, 50 µs by
/// default. Spinning keeps a step's next stage from paying a wake-up, and
/// costs the thread that isn't waiting (a spinning SMT sibling shares its
/// core, and every spinning core its power).
fn spin() -> Duration {
    static SPIN: std::sync::OnceLock<Duration> = std::sync::OnceLock::new();
    *SPIN.get_or_init(|| Duration::from_micros(std::env::var("SPIN_US").ok().and_then(|s| s.parse().ok()).unwrap_or(50)))
}

impl Pool {
    pub fn new(threads: usize) -> Pool {
        let shared = Arc::new(Shared {
            generation: AtomicU64::new(0),
            job: Mutex::new(None),
            next: AtomicUsize::new(0),
            open: AtomicBool::new(false),
            busy: AtomicUsize::new(0),
            parked: Mutex::new(0),
            wake: Condvar::new(),
            stop: AtomicBool::new(false),
            panicked: AtomicBool::new(false),
            threads: threads.max(1),
            sticky: std::env::var_os("STICKY").is_some(),
            completed: AtomicUsize::new(0),
        });
        let workers = (1..threads.max(1))
            .map(|k| {
                let s = shared.clone();
                std::thread::Builder::new().name(format!("pool-{k}")).spawn(move || worker(&s, k)).expect("a worker thread")
            })
            .collect();
        Pool { shared, workers }
    }
}

fn tasks(s: &Shared, f: &(dyn Fn(usize) + Sync), n: usize, me: usize) {
    let run = |k: usize| {
        if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(k))).is_err() {
            s.panicked.store(true, Ordering::Relaxed);
        }
        s.completed.fetch_add(1, Ordering::Release);
    };
    if s.sticky {
        (me..n).step_by(s.threads).for_each(run);
        return;
    }
    loop {
        let k = s.next.fetch_add(1, Ordering::Relaxed);
        if k >= n {
            break;
        }
        run(k);
    }
}

fn worker(s: &Shared, me: usize) {
    let mut seen = 0;
    loop {
        let idle = Instant::now();
        let mut spins = 0u32;
        loop {
            if s.stop.load(Ordering::Acquire) {
                return;
            }
            let g = s.generation.load(Ordering::Acquire);
            if g != seen {
                seen = g;
                break;
            }
            spins += 1;
            if spins % 64 == 0 && idle.elapsed() > spin() {
                let mut parked = s.parked.lock().unwrap();
                *parked += 1;
                // Checked under the lock `run` takes to wake: no lost wakeup.
                while s.generation.load(Ordering::Acquire) == seen && !s.stop.load(Ordering::Acquire) {
                    parked = s.wake.wait(parked).unwrap();
                }
                *parked -= 1;
                spins = 0;
                continue;
            }
            std::hint::spin_loop();
        }
        s.busy.fetch_add(1, Ordering::SeqCst);
        if s.open.load(Ordering::SeqCst) {
            let job = *s.job.lock().unwrap();
            if let Some((f, n)) = job {
                tasks(s, f, n, me);
            }
        }
        s.busy.fetch_sub(1, Ordering::SeqCst);
    }
}

impl Executor for Pool {
    fn threads(&self) -> usize {
        self.workers.len() + 1
    }

    fn run(&self, n: usize, f: &(dyn Fn(usize) + Sync)) {
        let s = &*self.shared;
        // SAFETY: the reference is reachable only through `job` while the
        // run is open, and this doesn't return until the run is closed and
        // no worker is inside it (`busy` is 0), after which none will read
        // `job` for this run (a worker checks `open` after marking itself
        // busy; both sides SeqCst).
        let f: &'static (dyn Fn(usize) + Sync) = unsafe { std::mem::transmute(f) };
        *s.job.lock().unwrap() = Some((f, n));
        s.next.store(0, Ordering::SeqCst);
        s.completed.store(0, Ordering::SeqCst);
        s.open.store(true, Ordering::SeqCst);
        s.generation.fetch_add(1, Ordering::SeqCst);
        if *s.parked.lock().unwrap() > 0 {
            s.wake.notify_all();
        }
        tasks(s, f, n, 0);
        // Sticky, another thread's tasks are its alone: waited for before
        // the run closes, or they'd never run.
        while s.sticky && s.completed.load(Ordering::Acquire) < n {
            std::hint::spin_loop();
        }
        s.open.store(false, Ordering::SeqCst);
        while s.busy.load(Ordering::SeqCst) != 0 {
            std::hint::spin_loop();
        }
        *s.job.lock().unwrap() = None;
        if s.panicked.swap(false, Ordering::Relaxed) {
            panic!("a task panicked");
        }
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        self.shared.generation.fetch_add(1, Ordering::SeqCst);
        drop(self.shared.parked.lock().unwrap());
        self.shared.wake.notify_all();
        for w in self.workers.drain(..) {
            w.join().expect("a worker");
        }
    }
}
