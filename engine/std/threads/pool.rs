//! The scheduler's threads: an `engine_ecs::Executor` on an explicit rayon
//! `ThreadPool`, its threads kept for the session, placed on one CCD and
//! pinned there with core_affinity. The `threads` mod (lib.rs) makes one
//! and installs it as the world's executor; benchmarks and tests make
//! their own. See docs/architecture/threads.md.
//!
//! Rayon is the thread host and nothing more: a dispatch starts each
//! worker once (a scope, a spawn a worker, as Rapier starts its staged
//! solver), and what runs inside it hands out its own work. Rayon's work
//! stealing never hands out a shape's blocks: a block taken by a thief runs
//! on a cold cache (dispatch-spike.md, `steal`). Never rayon's global
//! pool: every mod library has its own copy of rayon's statics, so a global
//! pool would be one a library, and gone with it.

use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use engine_ecs::Executor;

/// Where a pool's threads go: the CPUs they're pinned to, one a thread,
/// the calling thread's first. A game that wants other places builds its
/// own thread mod over this library with its own (threads.md, "Placement").
pub trait Placement {
    fn cpus(&self) -> Vec<usize>;
}

/// The cores of one CCD, one CPU a core: those sharing the last-level
/// cache with the lowest CPU this process may run on, without their SMT
/// siblings. On the Ryzen 9 7950X that's CPUs 0 to 7. Threads spread over
/// two CCDs take 1.9 times as long on a colored solve as on one, and a
/// sibling adds about 5% (dispatch-spike.md, "What the pool must
/// provide"; docs/lore/cpus-16-to-31-are-the-same-cores-as-0-to-15.md).
pub struct OneCcd;

impl Placement for OneCcd {
    fn cpus(&self) -> Vec<usize> {
        one_ccd(Path::new("/sys/devices/system/cpu"), allowed().as_deref())
    }
}

/// The CPUs this process may run on, as the first thread to ask found them,
/// or `None` where that thread may run on one CPU alone. `get_core_ids`
/// reads the calling thread's own mask, and a thread that has dispatched is
/// pinned to one CPU (`Settings::pin_caller`): asked from it, a second pool
/// would be placed on that CPU alone, every worker with it (measured: a
/// second pool made on the frame's thread ran a step's passes 25 times
/// slower; docs/lore/a-pinned-thread-sees-one-cpu-and-its-new-threads-inherit-it.md).
/// The first answer is kept, but every load of the `threads` mod is a fresh
/// copy of this static, so a mask of one CPU is taken for a pinned thread's
/// and the topology alone places the pool.
fn allowed() -> Option<Vec<usize>> {
    static ALLOWED: std::sync::OnceLock<Option<Vec<usize>>> = std::sync::OnceLock::new();
    ALLOWED
        .get_or_init(|| core_affinity::get_core_ids().map(|ids| ids.into_iter().map(|c| c.id).collect::<Vec<_>>()).filter(|a| a.len() > 1))
        .clone()
}

/// The CPUs given, in order.
pub struct Cpus(pub Vec<usize>);

impl Placement for Cpus {
    fn cpus(&self) -> Vec<usize> {
        self.0.clone()
    }
}

/// `OneCcd` from the CPU topology under `sys` (`/sys/devices/system/cpu`),
/// among `allowed` (all, if `None`). Empty if the topology can't be read.
pub fn one_ccd(sys: &Path, allowed: Option<&[usize]>) -> Vec<usize> {
    let read = |cpu: usize, file: &str| std::fs::read_to_string(sys.join(format!("cpu{cpu}")).join(file)).ok();
    let mut cpus: Vec<usize> = std::fs::read_dir(sys)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| e.file_name().to_str()?.strip_prefix("cpu")?.parse().ok())
        .filter(|cpu| allowed.is_none_or(|a| a.contains(cpu)))
        .collect();
    cpus.sort_unstable();
    // A CPU's last-level cache: the shared list of its highest level.
    let llc = |cpu: usize| {
        (0..8)
            .filter_map(|i| {
                let level: u32 = read(cpu, &format!("cache/index{i}/level"))?.trim().parse().ok()?;
                Some((level, read(cpu, &format!("cache/index{i}/shared_cpu_list"))?))
            })
            .max_by_key(|(level, _)| *level)
            .map(|(_, list)| cpu_list(&list))
    };
    let Some(first) = cpus.first() else { return Vec::new() };
    let Some(ccd) = llc(*first) else { return Vec::new() };
    cpus.into_iter()
        .filter(|cpu| ccd.contains(cpu))
        // One a core: the first of its SMT siblings.
        .filter(|&cpu| read(cpu, "topology/thread_siblings_list").map(|s| cpu_list(&s)).and_then(|s| s.into_iter().min()) == Some(cpu))
        .collect()
}

/// A kernel CPU list, `0-7,16-23`.
pub fn cpu_list(text: &str) -> Vec<usize> {
    let mut out = Vec::new();
    for part in text.trim().split(',').filter(|p| !p.is_empty()) {
        let (lo, hi) = part.split_once('-').unwrap_or((part, part));
        if let (Ok(lo), Ok(hi)) = (lo.parse::<usize>(), hi.parse::<usize>()) {
            out.extend(lo..=hi);
        }
    }
    out
}

/// How a pool is made.
#[derive(Clone, Debug)]
pub struct Settings {
    /// Threads, the calling thread's included; `None` for one a CPU of the
    /// placement.
    pub threads: Option<usize>,
    /// Pin the threads to the placement's CPUs; else the OS places them.
    pub pin: bool,
    /// Pinning, pin a thread that dispatches too, to the placement's first
    /// CPU, the one no worker has, at its first dispatch: the frame's
    /// thread, left to the OS, makes a step at 8 threads about twice as long
    /// (threads.md, "Placement"). Not for a pool many threads dispatch on
    /// at once (`shared`), which would pile them on one core.
    pub pin_caller: bool,
    /// How long a worker spins after a dispatch, waiting for the next,
    /// before rayon parks it (threads.md, "Warmth").
    pub warm: Duration,
}

/// Measured on the step (threads.md, "Warmth").
pub const WARM: Duration = Duration::from_micros(1000);

impl Default for Settings {
    fn default() -> Self {
        Settings { threads: None, pin: true, pin_caller: true, warm: WARM }
    }
}

impl Settings {
    /// The defaults, as the environment changes them: `ENGINE_THREADS=<n>`,
    /// `ENGINE_PIN=0`, `ENGINE_WARM_US=<µs>`. What the `threads` mod reads at
    /// load, and `shared` here.
    pub fn from_env() -> Settings {
        let env = |name: &str| std::env::var(name).ok()?.trim().parse::<u64>().ok();
        let d = Settings::default();
        Settings {
            threads: env("ENGINE_THREADS").map(|n| n.max(1) as usize).or(d.threads),
            pin: env("ENGINE_PIN").map_or(d.pin, |p| p != 0),
            pin_caller: d.pin_caller,
            warm: env("ENGINE_WARM_US").map_or(d.warm, Duration::from_micros),
        }
    }
}

/// One pool for the whole process, as the environment sets it (`OneCcd`,
/// `Settings::from_env`); `None` at one thread. For a test or benchmark that
/// makes engine after engine: each load of the `threads` mod spawns its
/// threads from a fresh copy of std, which takes a pthread key it never
/// gives back and stays mapped, and a process has 1024 keys
/// (docs/lore/a-mod-that-spawns-threads-takes-a-tls-key-each-load.md). Such
/// a harness loads its game without the thread host and installs this.
pub fn shared() -> Option<Arc<Pool>> {
    static SHARED: std::sync::OnceLock<Option<Arc<Pool>>> = std::sync::OnceLock::new();
    SHARED
        .get_or_init(|| {
            let pool = Pool::new(&OneCcd, &Settings { pin_caller: false, ..Settings::from_env() });
            (pool.threads() > 1).then(|| Arc::new(pool))
        })
        .clone()
}

/// The thread host's mod name, as `engine_game` names the default
/// (`//engine/std/threads`): what a harness leaves out of a manifest to
/// install `shared` in its place.
pub const MOD_NAME: &str = "threads";

/// What a warm worker watches: a dispatch bumps `generation`.
#[derive(Default)]
struct Idle {
    generation: AtomicU64,
    stop: AtomicBool,
}

impl Idle {
    /// Spins until the next dispatch, `warm` at most: a worker's job between
    /// dispatches, so the next one's start finds it running.
    fn spin(&self, seen: u64, warm: Duration) {
        let start = Instant::now();
        let mut turns = 0u32;
        while self.generation.load(Ordering::Acquire) == seen && !self.stop.load(Ordering::Relaxed) {
            std::hint::spin_loop();
            turns = turns.wrapping_add(1);
            if turns.is_multiple_of(64) && start.elapsed() >= warm {
                return;
            }
        }
    }
}

/// Threads kept for the session, which `run` starts once a dispatch: the
/// calling thread is one of them (task 0), and none is a main thread.
pub struct Pool {
    pool: Option<rayon::ThreadPool>,
    workers: Vec<JoinHandle<()>>,
    threads: usize,
    /// Where each thread is pinned, the calling thread's place first;
    /// empty if none is.
    cpus: Vec<usize>,
    /// Whether a thread that dispatches is pinned to `cpus[0]`.
    pin_caller: bool,
    warm: Duration,
    idle: Arc<Idle>,
    dispatches: AtomicU64,
}

impl Pool {
    pub fn new(placement: &dyn Placement, settings: &Settings) -> Pool {
        let placed = placement.cpus();
        let threads = settings.threads.unwrap_or(placed.len()).max(1);
        let cpus: Vec<usize> = if settings.pin { placed.into_iter().take(threads).collect() } else { Vec::new() };
        let idle = Arc::new(Idle::default());
        let mut workers = Vec::new();
        let pool = (threads > 1).then(|| {
            let pins = cpus.clone();
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads - 1)
                .thread_name(|i| format!("engine-pool-{}", i + 1))
                // Worker `i` is thread `i + 1`: the calling thread is 0.
                .start_handler(move |i| {
                    if let Some(&cpu) = pins.get(i + 1) {
                        core_affinity::set_for_current(core_affinity::CoreId { id: cpu });
                    }
                })
                // Spawned here, so the pool can join them: a thread still in
                // rayon's code when its library is unmapped would crash.
                .spawn_handler(|thread| {
                    let mut b = std::thread::Builder::new();
                    if let Some(name) = thread.name() {
                        b = b.name(name.to_owned());
                    }
                    workers.push(b.spawn(|| thread.run())?);
                    Ok(())
                })
                .build()
                .expect("the pool's threads")
        });
        let pin_caller = settings.pin_caller && !cpus.is_empty();
        Pool { pool, workers, threads, cpus, pin_caller, warm: settings.warm, idle, dispatches: AtomicU64::new(0) }
    }

    /// Where the threads are pinned, the calling thread's place first;
    /// empty if they aren't.
    pub fn cpus(&self) -> &[usize] {
        &self.cpus
    }

    /// Whether a thread that dispatches is pinned to the first of `cpus`.
    pub fn pins_caller(&self) -> bool {
        self.pin_caller
    }

    /// Pins the calling thread to `cpus[0]`, once: a thread-local remembers
    /// where it was put (a `Cell` with no destructor, so it registers none,
    /// which would keep this library mapped).
    fn pin_caller(&self) {
        thread_local! {
            static PINNED: std::cell::Cell<usize> = const { std::cell::Cell::new(usize::MAX) };
        }
        let cpu = self.cpus[0];
        if PINNED.get() != cpu && core_affinity::set_for_current(core_affinity::CoreId { id: cpu }) {
            PINNED.set(cpu);
        }
    }

    pub fn warm(&self) -> Duration {
        self.warm
    }

    /// Runs that started the pool's workers, since it was made.
    pub fn dispatches(&self) -> u64 {
        self.dispatches.load(Ordering::Relaxed)
    }
}

impl Executor for Pool {
    fn threads(&self) -> usize {
        self.threads
    }

    /// Task `j` of the first `threads` on thread `j` (the calling thread
    /// runs task 0), the rest to whichever thread is free: each thread
    /// starts with its own and takes more from a counter, so rayon only
    /// starts the workers. Returns once every task has, as the scope does.
    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        let n = tasks.min(self.threads);
        let Some(pool) = self.pool.as_ref().filter(|_| n > 1) else {
            (0..tasks).for_each(f);
            return;
        };
        if self.pin_caller {
            self.pin_caller();
        }
        let seen = self.idle.generation.fetch_add(1, Ordering::AcqRel) + 1;
        self.dispatches.fetch_add(1, Ordering::Relaxed);
        let next = AtomicUsize::new(n);
        let part = |j: usize| {
            f(j);
            loop {
                let k = next.fetch_add(1, Ordering::Relaxed);
                if k >= tasks {
                    break;
                }
                f(k);
            }
        };
        pool.in_place_scope(|s| {
            for j in 1..n {
                let part = &part;
                s.spawn(move |_| part(j));
            }
            part(0);
        });
        if !self.warm.is_zero() {
            let (idle, warm) = (self.idle.clone(), self.warm);
            pool.spawn_broadcast(move |_| idle.spin(seen, warm));
        }
    }
}

impl Drop for Pool {
    /// Stops the workers and waits for them to leave, while this library
    /// is still mapped.
    fn drop(&mut self) {
        self.idle.stop.store(true, Ordering::Relaxed);
        self.idle.generation.fetch_add(1, Ordering::AcqRel);
        drop(self.pool.take());
        for w in self.workers.drain(..) {
            // A worker's panic was rayon's to report; there is nothing to
            // add to it here.
            let _ = w.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_lists_parse_as_the_kernel_writes_them() {
        assert_eq!(cpu_list("0-3,8,10-11\n"), [0, 1, 2, 3, 8, 10, 11]);
        assert_eq!(cpu_list("5"), [5]);
        assert_eq!(cpu_list(""), Vec::<usize>::new());
    }

    /// A machine of two CCDs of four cores, each core's SMT sibling eight
    /// CPUs on: the first CCD's cores, siblings left out, or the first CCD
    /// the process may run on.
    #[test]
    fn one_ccd_is_the_first_last_level_caches_cores() {
        let root = std::env::temp_dir().join(format!("one_ccd_{}", std::process::id()));
        for cpu in 0..16 {
            let dir = root.join(format!("cpu{cpu}"));
            std::fs::create_dir_all(dir.join("topology")).unwrap();
            let core = cpu % 8;
            std::fs::write(dir.join("topology/thread_siblings_list"), format!("{core},{}\n", core + 8)).unwrap();
            for (i, level, shared) in
                [(0, 1, format!("{core},{}", core + 8)), (3, 3, if core < 4 { "0-3,8-11" } else { "4-7,12-15" }.to_string())]
            {
                let cache = dir.join(format!("cache/index{i}"));
                std::fs::create_dir_all(&cache).unwrap();
                std::fs::write(cache.join("level"), format!("{level}\n")).unwrap();
                std::fs::write(cache.join("shared_cpu_list"), format!("{shared}\n")).unwrap();
            }
        }
        std::fs::create_dir_all(root.join("cpufreq")).unwrap();
        assert_eq!(one_ccd(&root, None), [0, 1, 2, 3]);
        assert_eq!(one_ccd(&root, Some(&[1, 2, 3, 5, 9])), [1, 2, 3]);
        assert_eq!(one_ccd(&root, Some(&[5, 6, 13])), [5, 6]);
        assert_eq!(one_ccd(&root.join("nothing"), None), Vec::<usize>::new());
        std::fs::remove_dir_all(&root).unwrap();
    }

    fn pools() -> Vec<Pool> {
        [1, 2, 4, 8]
            .into_iter()
            .flat_map(|n| {
                let unpinned = Settings { threads: Some(n), pin: false, ..Settings::default() };
                [Pool::new(&OneCcd, &unpinned), Pool::new(&OneCcd, &Settings { warm: Duration::ZERO, ..unpinned.clone() })]
            })
            .collect()
    }

    /// Every task once, and `run` returns only once every task has, the
    /// slowest last. Which thread runs which task is rayon's: a worker that
    /// finishes its task may take a sleeping one's, which the protocols
    /// run inside a dispatch allow for (a late thread holds up nobody).
    #[test]
    fn a_run_runs_every_task_once_and_returns_after_the_last() {
        for pool in pools() {
            for tasks in [0, 1, 3, pool.threads(), 4 * pool.threads() + 1] {
                let ran: Vec<AtomicUsize> = (0..tasks).map(|_| AtomicUsize::new(0)).collect();
                pool.run(tasks, &|k| {
                    std::thread::sleep(Duration::from_micros(200 * (k % 3) as u64));
                    ran[k].fetch_add(1, Ordering::AcqRel);
                });
                assert!(ran.iter().all(|r| r.load(Ordering::Acquire) == 1), "{tasks} tasks on {}", pool.threads());
            }
        }
    }

    /// `Passes` on the pool is `Passes` on one thread, bit for bit: a ring
    /// of springs relaxed edge by edge (Gauss-Seidel, so the result is the
    /// colors' order's), at 2 to 8 threads, warm and cold (the workers
    /// parked between runs, so they come late to every one).
    #[test]
    fn passes_on_the_pool_are_the_passes_on_one_thread() {
        use engine_ecs::harness::{Cx, IntoSystem, Schedule};
        use engine_ecs::{Coloring, Passes, Stage, States, World};
        use std::sync::Mutex;

        let n = 2000u32;
        let mut edges: Vec<(u32, u32)> = (0..n).map(|i| (i, (i + 1) % n)).collect();
        edges.extend((0..n).step_by(3).map(|i| (i, (i * 7 + 5) % n)).filter(|(a, b)| a != b));
        let moves: Vec<bool> = (0..n).map(|i| i % 5 != 0).collect();
        let mut coloring = Coloring::default();
        coloring.greedy(edges.len(), |i| edges[i], &moves, true, &mut Vec::new());
        let mut place = Vec::new();
        let layout = coloring.pack(1, &mut place);
        let mut items = vec![(0, 0); layout.items()];
        for (i, p) in place.iter().enumerate() {
            if let Some((item, _)) = p {
                items[*item as usize] = edges[i];
            }
        }
        let start: Vec<f32> = (0..n).map(|i| (i as f32 * 0.37).sin()).collect();
        let solve = |pool: Option<Pool>| {
            let w = World::new();
            w.set_executor(pool.map(|p| Arc::new(p) as Arc<dyn Executor>));
            let out = Arc::new(Mutex::new(Vec::new()));
            let (layout, items, start, moves, got) = (layout.clone(), Mutex::new(items.clone()), start.clone(), moves.clone(), out.clone());
            let system = move |_: &mut Cx, passes: Passes| {
                let mut states = start.clone();
                let relax = |s: &mut States<'_, f32>, (a, b): (u32, u32), k: f32| {
                    let (a, b) = (a as usize, b as usize);
                    let d = (s.get(b) - s.get(a)) * k;
                    if moves[a] {
                        s.set(a, s.get(a) + d);
                    }
                    if moves[b] {
                        s.set(b, s.get(b) - d);
                    }
                };
                let program: Vec<Stage<f32>> =
                    (0..20).flat_map(|i| [Stage::Items(0.3 + i as f32 * 0.01), Stage::Each(0.999, n as usize)]).collect();
                for _ in 0..5 {
                    passes.run(
                        &layout,
                        &mut items.lock().unwrap(),
                        &mut states,
                        &program,
                        |k, block, mut s| block.iter().for_each(|&e| relax(&mut s, e, k)),
                        |k, r, mut s| r.for_each(|i| s.set(i, s.get(i) * k)),
                    );
                    std::thread::sleep(Duration::from_millis(2));
                }
                *got.lock().unwrap() = states.iter().map(|x| x.to_bits()).collect::<Vec<u32>>();
            };
            Schedule { systems: vec![system.system(&w, "relax")] }.run_sequential(&w);
            std::mem::take(&mut *out.lock().unwrap())
        };
        let one = solve(None);
        assert_eq!(one.len(), n as usize);
        for threads in [1, 2, 4, 8] {
            for warm in [Duration::ZERO, WARM] {
                let pool = Pool::new(&OneCcd, &Settings { threads: Some(threads), pin: false, pin_caller: false, warm });
                assert!(solve(Some(pool)) == one, "{threads} threads, warm {warm:?}");
            }
        }
    }

    /// This process's pool threads, by name, with the CPUs each may run
    /// on, once `n` of them have named themselves (a thread takes its name
    /// once it's running).
    fn pool_threads(n: usize) -> Vec<(String, Vec<usize>)> {
        let read = || {
            let mut out: Vec<(String, Vec<usize>)> = std::fs::read_dir("/proc/self/task")
                .unwrap()
                .flatten()
                .filter_map(|t| {
                    let name = std::fs::read_to_string(t.path().join("comm")).ok()?.trim().to_string();
                    let status = std::fs::read_to_string(t.path().join("status")).ok()?;
                    let cpus = status.lines().find_map(|l| l.strip_prefix("Cpus_allowed_list:"))?;
                    name.starts_with("engine-pool").then(|| (name, cpu_list(cpus)))
                })
                .collect();
            out.sort();
            out
        };
        let start = Instant::now();
        let mut out = read();
        while out.len() != n && start.elapsed() < Duration::from_secs(2) {
            std::thread::sleep(Duration::from_millis(1));
            out = read();
        }
        out
    }

    /// Workers pinned where the placement says (worker `i` is thread
    /// `i + 1`), the calling thread left where it is.
    #[test]
    fn workers_are_pinned_to_the_placements_cpus() {
        let Some(allowed) = core_affinity::get_core_ids() else { return };
        let cpus: Vec<usize> = allowed.iter().map(|c| c.id).take(3).collect();
        if cpus.len() < 3 {
            return;
        }
        let pool = Pool::new(&Cpus(cpus.clone()), &Settings::default());
        assert_eq!(pool.cpus(), cpus);
        let want = vec![("engine-pool-1".to_string(), vec![cpus[1]]), ("engine-pool-2".to_string(), vec![cpus[2]])];
        let start = Instant::now();
        // Pinned once each starts.
        while pool_threads(2) != want && start.elapsed() < Duration::from_secs(2) {
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(pool_threads(2), want);
    }

    /// A pool made on a thread an earlier pool pinned (the frame's thread,
    /// `pin_caller`) is placed as the first was, on the whole CCD, not on
    /// that thread's one CPU: placement reads the process's CPUs as first
    /// found.
    #[test]
    fn a_pool_made_from_a_pinned_thread_is_placed_as_the_first() {
        let first = Pool::new(&OneCcd, &Settings { threads: Some(2), ..Settings::default() });
        let placed = OneCcd.cpus();
        if placed.len() < 2 {
            return;
        }
        first.run(2, &|_| {});
        assert!(first.pins_caller());
        drop(first);
        let second = Pool::new(&OneCcd, &Settings::default());
        assert_eq!(second.cpus(), placed, "placed on the CCD, not on the pinned thread's CPU");
    }

    /// A pool's threads end when it's dropped: the `threads` mod drops it
    /// on close, while its code is mapped, and the drop joins them. (A
    /// joined thread's entry in `/proc` can outlast the join for a moment,
    /// so this waits for it.)
    #[test]
    fn dropping_a_pool_ends_its_threads() {
        assert_eq!(pool_threads(0).len(), 0, "no pool of another test's left");
        let pool = Pool::new(&OneCcd, &Settings { threads: Some(4), pin: false, ..Settings::default() });
        pool.run(4, &|_| {});
        assert_eq!(pool_threads(3).len(), 3);
        drop(pool);
        assert_eq!(pool_threads(0).len(), 0);
    }
}
