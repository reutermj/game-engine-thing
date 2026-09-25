//! How far the solver parallelizes:
//! `./bazel run -c opt //engine/std/physics:parallel_solver`
//! (`PIN=1` pins thread `i` to CPU `i`, `PIN=ccd` fills one CCD first;
//! `QUICK=1` skips the settling runs; `ONLY=<text>` runs only the scenes
//! whose names contain it; `DIAG=1` shows where colored solves wait).
//!
//! The solver's input is taken as the step on arrays gathers it
//! (`arrays.rs`, the step `:tax` checks against the mod bit for bit) from
//! piles run in the engine (`:tax`'s, which settle as columns, and real
//! ones of 1000 to 40 000) and a scene of separate stacks built here. Over
//! it, the same sequential impulses four ways:
//!
//! - **serial**: `solver::solve`, contacts in pair order, the baseline;
//! - **colored**: contacts colored so that no two in a color share a body
//!   that moves (Box2D v3's graph coloring), colors solved in turn and each
//!   color's contacts spread over the threads. That solves contacts in
//!   another order than pair order, so it's a different computation from
//!   serial, but one fixed by the contacts alone: the same colors, the same
//!   order in each, whatever the thread count, so it must end bit for bit
//!   the same on every count, which is checked;
//! - **wide**: colored, each color in batches of 8 solved with AVX2, the
//!   same operations, so checked bit for bit against colored;
//! - **islands**: groups of bodies joined by contacts, each solved serially
//!   in pair order by one thread (or a thread's run of islands together).
//!   Islands share no moving body, so this is serial's computation
//!   exactly, checked bit for bit against it.
//!
//! Then what each costs to build a step, and whether colored settles the
//! pile as well as serial, over whole steps on arrays. What it found:
//! docs/architecture/physics.md, "Parallel solving".

#[path = "arrays.rs"]
mod arrays;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

use std::cell::UnsafeCell;
use std::hint::black_box;
use std::ops::Range;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use arrays::{Arrays, DT, Stages};
use engine_loader::engine::Engine;
use physics::{Body, Collider, DYNAMIC, Vec2};
use solver::{BETA, BOUNCE_THRESHOLD, Constraint, ITERATIONS, SLOP, SolverBody};

const THREADS: [usize; 7] = [1, 2, 4, 8, 12, 16, 32];
const REPS: usize = 41;

// ---- A pool of spinning workers ----

/// Padded so that two threads' flags never share a cache line: a line
/// bouncing between two cores, across CCDs on this machine, costs more than
/// the flags' whole work.
#[repr(align(128))]
#[derive(Default)]
struct Padded<T>(T);

/// A job's closure, as a raw pointer so a worker can hold it between runs.
/// Only dereferenced while `Pool::run` is blocked on the job, which keeps
/// the closure alive.
type Job = *const (dyn Fn(usize) + Sync);

struct Shared {
    /// Per worker: bumped to hand it a job.
    epoch: Vec<Padded<AtomicU64>>,
    /// Per worker: set while it's parked, so a run knows to unpark it.
    parked: Vec<Padded<AtomicBool>>,
    job: UnsafeCell<Option<Job>>,
    remaining: Padded<AtomicUsize>,
    stop: AtomicBool,
}

// SAFETY: `job` is written by the caller of `run` before the release store
// of each worker's epoch, and read by a worker only after its acquire load
// sees the new epoch, and not again until the next.
unsafe impl Sync for Shared {}
unsafe impl Send for Shared {}

/// Workers spin this long for the next job before parking: a solve runs
/// jobs back to back, and waking a parked thread costs tens of µs, more
/// than a color's work. Short enough that a serial run started after a
/// parallel one isn't sharing the chip with 31 spinning cores for long.
const SPIN: Duration = Duration::from_micros(300);

struct Pool {
    shared: Arc<Shared>,
    threads: Vec<JoinHandle<()>>,
    epoch: AtomicU64,
}

impl Pool {
    /// `n` workers, the caller being worker 0, each on `pin`'s CPU if any.
    fn new(n: usize, pin: Option<fn(usize) -> usize>) -> Pool {
        let shared = Arc::new(Shared {
            epoch: (0..n).map(|_| Padded::default()).collect(),
            parked: (0..n).map(|_| Padded::default()).collect(),
            job: UnsafeCell::new(None),
            remaining: Padded::default(),
            stop: AtomicBool::new(false),
        });
        if let Some(cpu) = pin {
            pin_to(cpu(0));
        }
        let threads = (1..n)
            .map(|w| {
                let s = shared.clone();
                std::thread::spawn(move || {
                    if let Some(cpu) = pin {
                        pin_to(cpu(w));
                    }
                    let mut seen = 0;
                    loop {
                        seen = wait_for(&s, w, seen);
                        if s.stop.load(Ordering::Acquire) {
                            return;
                        }
                        // SAFETY: see `Shared`.
                        let job = unsafe { (*s.job.get()).unwrap() };
                        unsafe { (*job)(w) };
                        s.remaining.0.fetch_sub(1, Ordering::Release);
                    }
                })
            })
            .collect();
        Pool { shared, threads, epoch: AtomicU64::new(0) }
    }

    /// Runs `job(w)` on workers `0..threads`, returning when all have.
    fn run(&self, threads: usize, job: &(dyn Fn(usize) + Sync)) {
        let s = &*self.shared;
        assert!(threads <= s.epoch.len());
        let e = self.epoch.fetch_add(1, Ordering::Relaxed) + 1;
        // SAFETY: no worker reads `job` until it sees the epoch below, and
        // every worker of the last run has finished with it (`remaining`).
        unsafe { *s.job.get() = Some(std::mem::transmute::<&(dyn Fn(usize) + Sync), Job>(job)) };
        s.remaining.0.store(threads - 1, Ordering::Relaxed);
        for w in 1..threads {
            s.epoch[w].0.store(e, Ordering::SeqCst);
            if s.parked[w].0.swap(false, Ordering::SeqCst) {
                self.threads[w - 1].thread().unpark();
            }
        }
        job(0);
        while s.remaining.0.load(Ordering::Acquire) != 0 {
            std::hint::spin_loop();
        }
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        for w in 1..self.shared.epoch.len() {
            self.shared.epoch[w].0.fetch_add(1, Ordering::SeqCst);
            self.threads[w - 1].thread().unpark();
        }
        for t in self.threads.drain(..) {
            t.join().unwrap();
        }
    }
}

/// Spins, then parks, until worker `w`'s epoch moves past `seen`.
fn wait_for(s: &Shared, w: usize, seen: u64) -> u64 {
    let start = Instant::now();
    let mut spins = 0u32;
    loop {
        let e = s.epoch[w].0.load(Ordering::Acquire);
        if e != seen {
            return e;
        }
        std::hint::spin_loop();
        spins += 1;
        if spins % 256 == 0 && start.elapsed() > SPIN {
            // Park, unless the epoch moved while saying so: `run` stores
            // the epoch before looking at `parked`, both SeqCst, so one of
            // the two sees the other.
            s.parked[w].0.store(true, Ordering::SeqCst);
            while s.epoch[w].0.load(Ordering::SeqCst) == seen {
                std::thread::park();
            }
            s.parked[w].0.store(false, Ordering::SeqCst);
        }
    }
}

unsafe extern "C" {
    fn sched_setaffinity(pid: i32, size: usize, mask: *const u64) -> i32;
}

/// This machine numbers the 16 cores 0-15 (0-7 on one CCD, 8-15 on the
/// other) and their SMT siblings 16-31, so thread `i` on CPU `i` keeps 8
/// threads on one L3 and 16 on one thread per core.
fn by_core(w: usize) -> usize {
    w
}

/// Fills one CCD, SMT siblings too, before the other: 16 threads on one L3.
fn by_ccd(w: usize) -> usize {
    [0, 16, 8, 24][w / 8] + w % 8
}

fn pin_to(cpu: usize) {
    let mut mask = [0u64; 16];
    mask[cpu / 64] |= 1 << (cpu % 64);
    // SAFETY: a mask of 1024 bits, for this thread (pid 0).
    let r = unsafe { sched_setaffinity(0, std::mem::size_of_val(&mask), mask.as_ptr()) };
    assert_eq!(r, 0, "pinning to CPU {cpu}");
}

/// A spinning barrier: every stage of a colored solve ends in one, a few
/// hundred a solve, so a futex's wake-up would cost more than the stages.
struct Barrier {
    n: usize,
    count: Padded<AtomicUsize>,
    generation: Padded<AtomicUsize>,
}

impl Barrier {
    fn new(n: usize) -> Barrier {
        Barrier { n, count: Padded::default(), generation: Padded::default() }
    }

    fn wait(&self) {
        if self.n == 1 {
            return;
        }
        let g = self.generation.0.load(Ordering::Acquire);
        if self.count.0.fetch_add(1, Ordering::AcqRel) == self.n - 1 {
            // Reset before releasing: nobody arrives for the next round
            // until they've seen the new generation.
            self.count.0.store(0, Ordering::Relaxed);
            self.generation.0.store(g + 1, Ordering::Release);
        } else {
            while self.generation.0.load(Ordering::Acquire) == g {
                std::hint::spin_loop();
            }
        }
    }
}

/// Keeps `threads` workers busy for `ms`, so their cores are clocked up
/// when a timed run starts. This machine's governor (schedutil) clocks a
/// core by its recent load: one that was idle stays at 3 GHz, not the 5+
/// it boosts to, for about 50 ms of load, which halved a solve on 2 threads
/// until it had (docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md).
fn warm_up(pool: &Pool, threads: usize, ms: u64) {
    pool.run(threads, &|_| {
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(ms) {
            std::hint::spin_loop();
        }
    });
}

// ---- The solver's steps, one contact at a time ----

/// A pointer shared by the threads of a solve. Which thread touches which
/// element is the coloring's (or the island's) business: see `Colors`.
#[derive(Clone, Copy)]
struct Ptr<T>(*mut T);
unsafe impl<T> Send for Ptr<T> {}
unsafe impl<T> Sync for Ptr<T> {}

impl<T> Ptr<T> {
    fn of(s: &mut [T]) -> Ptr<T> {
        Ptr(s.as_mut_ptr())
    }

    /// SAFETY: `i` in bounds, and no other thread writing it meanwhile.
    unsafe fn at(self, i: usize) -> *mut T {
        unsafe { self.0.add(i) }
    }
}

// These are `solver::solve`'s loop bodies, operation for operation, so a
// solve made of them in pair order is `solver::solve` bit for bit (checked
// in `scene`). One difference: they don't write a body that doesn't move,
// where `solve` adds zero to it, since other threads read it meanwhile.
// Adding `impulse * 0.0` changes no finite value, though it can turn -0.0
// into 0.0.

#[inline(always)]
unsafe fn target(b: Ptr<SolverBody>, c: &mut Constraint, dt: f32) -> f32 {
    let (a, bb) = unsafe { (*b.at(c.a as usize), *b.at(c.b as usize)) };
    let vn = (bb.v - a.v).dot(c.normal);
    c.speed = -vn;
    if c.depth < 0.0 {
        c.depth / dt
    } else if -vn > BOUNCE_THRESHOLD {
        -c.restitution * vn
    } else {
        0.0
    }
}

#[inline(always)]
unsafe fn push_v(b: Ptr<SolverBody>, c: &Constraint, impulse: Vec2) {
    let (pa, pb) = unsafe { (b.at(c.a as usize), b.at(c.b as usize)) };
    unsafe {
        let (ia, ib) = ((*pa).inv_mass, (*pb).inv_mass);
        if ia != 0.0 {
            (*pa).v -= impulse * ia;
        }
        if ib != 0.0 {
            (*pb).v += impulse * ib;
        }
    }
}

#[inline(always)]
unsafe fn warm(b: Ptr<SolverBody>, c: &Constraint) {
    let impulse = c.normal * c.jn + c.normal.perp() * c.jt;
    unsafe { push_v(b, c, impulse) };
}

#[inline(always)]
unsafe fn relax(b: Ptr<SolverBody>, c: &mut Constraint, target: f32) {
    let (pa, pb) = unsafe { (b.at(c.a as usize), b.at(c.b as usize)) };
    unsafe {
        let k = (*pa).inv_mass + (*pb).inv_mass;
        if k == 0.0 {
            return;
        }
        let rel = (*pb).v - (*pa).v;
        let jn = (c.jn + (target - rel.dot(c.normal)) / k).max(0.0);
        let dn = jn - c.jn;
        c.jn = jn;
        push_v(b, c, c.normal * dn);

        let t = c.normal.perp();
        let rel = (*pb).v - (*pa).v;
        let limit = c.friction * c.jn;
        let jt = (c.jt - rel.dot(t) / k).clamp(-limit, limit);
        let d = jt - c.jt;
        c.jt = jt;
        push_v(b, c, t * d);
    }
}

#[inline(always)]
unsafe fn relax_pseudo(b: Ptr<SolverBody>, c: &Constraint, pj: &mut f32, dt: f32) {
    let (pa, pb) = unsafe { (b.at(c.a as usize), b.at(c.b as usize)) };
    unsafe {
        let (ia, ib) = ((*pa).inv_mass, (*pb).inv_mass);
        let k = ia + ib;
        if k == 0.0 || c.depth <= SLOP {
            return;
        }
        let bias = BETA * (c.depth - SLOP) / dt;
        let rel = (*pb).pseudo - (*pa).pseudo;
        let new = (*pj + (bias - rel.dot(c.normal)) / k).max(0.0);
        let d = new - *pj;
        *pj = new;
        let impulse = c.normal * d;
        if ia != 0.0 {
            (*pa).pseudo -= impulse * ia;
        }
        if ib != 0.0 {
            (*pb).pseudo += impulse * ib;
        }
    }
}

// ---- Coloring ----

/// Box2D v3's counts (`B2_GRAPH_COLOR_COUNT`, `B2_DYNAMIC_COLOR_COUNT`):
/// contacts between two moving bodies take the first 20 colors, contacts
/// with one that doesn't move colors 1 to 23, and a contact with no color
/// free goes to the overflow, solved by one thread after the colors.
const COLORS: usize = 24;
const DYNAMIC_COLORS: usize = 20;
const OVERFLOW: u8 = COLORS as u8;
/// Neither end moves: nothing to solve, only its closing speed to report.
const INERT: u8 = COLORS as u8 + 1;

#[derive(Clone, Copy, PartialEq)]
enum Rule {
    /// Box2D's: a contact with one moving end never takes color 0.
    Box2d,
    /// Every contact takes the lowest color free.
    Greedy,
}

/// Each contact's color, in pair order: the lowest free at both of its
/// moving ends, taking contacts in pair order, so it's a function of the
/// step's contacts alone, not of when each began or of the threads.
fn color(bodies: &[SolverBody], contacts: &[Constraint], rule: Rule) -> Vec<u8> {
    let mut used = vec![0u32; bodies.len()];
    contacts
        .iter()
        .map(|c| {
            let (a, b) = (c.a as usize, c.b as usize);
            let (ma, mb) = (bodies[a].inv_mass != 0.0, bodies[b].inv_mass != 0.0);
            if !ma && !mb {
                return INERT;
            }
            let taken = if ma { used[a] } else { 0 } | if mb { used[b] } else { 0 };
            let allowed: u32 = match (ma && mb, rule) {
                (true, _) => (1 << DYNAMIC_COLORS) - 1,
                (false, Rule::Box2d) => ((1 << COLORS) - 1) & !1,
                (false, Rule::Greedy) => (1 << COLORS) - 1,
            };
            let free = allowed & !taken;
            if free == 0 {
                return OVERFLOW;
            }
            let k = free.trailing_zeros();
            if ma {
                used[a] |= 1 << k;
            }
            if mb {
                used[b] |= 1 << k;
            }
            k as u8
        })
        .collect()
}

/// The contacts in color order, as the parallel solve takes them.
struct Colors {
    /// Each contact's place in color order, by pair order. Color order is
    /// the colors, then the overflow, then inert contacts; pair order
    /// within each.
    place: Vec<u32>,
    /// Ranges of `order`: one per color, then the overflow (the last).
    groups: Vec<Range<usize>>,
    /// Per group, the positions in color order of the contacts the split
    /// impulse solves (sunk past the slop): a subset, so its own lists.
    sunk: Vec<Vec<u32>>,
}

impl Colors {
    fn of(colors: &[u8], contacts: &[Constraint]) -> Colors {
        let mut count = [0usize; COLORS + 2];
        for &c in colors {
            count[c as usize] += 1;
        }
        let mut start = [0usize; COLORS + 3];
        for k in 0..COLORS + 2 {
            start[k + 1] = start[k] + count[k];
        }
        let mut at = start;
        let mut place = vec![0u32; colors.len()];
        for (i, &c) in colors.iter().enumerate() {
            place[i] = at[c as usize] as u32;
            at[c as usize] += 1;
        }
        let groups: Vec<Range<usize>> = (0..=COLORS).map(|k| start[k]..start[k + 1]).collect();
        // In pair order, so each list is in color order too.
        let mut sunk = vec![Vec::new(); groups.len()];
        for ((c, &color), &k) in contacts.iter().zip(colors).zip(&place) {
            if c.depth > SLOP && (color as usize) < groups.len() {
                sunk[color as usize].push(k);
            }
        }
        Colors { place, groups, sunk }
    }

    /// One color holding every contact in pair order: the colored solve
    /// run as `solver::solve` runs, to check its steps are the same.
    fn single(n: usize, contacts: &[Constraint]) -> Colors {
        let sunk = vec![(0..n as u32).filter(|&k| contacts[k as usize].depth > SLOP).collect(), Vec::new()];
        Colors { place: (0..n as u32).collect(), groups: vec![0..n, n..n], sunk }
    }

    /// One pass over the contacts in pair order, each written at its
    /// color's place, as a gather from the world would write them.
    fn permute(&self, contacts: &[Constraint]) -> Vec<Constraint> {
        let mut out = vec![Constraint::default(); contacts.len()];
        for (c, &k) in contacts.iter().zip(&self.place) {
            out[k as usize] = *c;
        }
        out
    }

    /// Back in pair order, as writing back to the world would read them.
    fn unpermute(&self, colored: &[Constraint], contacts: &mut [Constraint]) {
        for (c, &k) in contacts.iter_mut().zip(&self.place) {
            *c = colored[k as usize];
        }
    }

    fn overflow(&self) -> usize {
        self.groups.len() - 1
    }
}

/// A thread's share of a color is at least this many contacts, so a small
/// color goes to fewer threads while the rest wait at the barrier: handing
/// a core a few contacts costs more in cache lines than solving them.
const MIN_CHUNK: usize = 32;

fn chunk(r: &Range<usize>, w: usize, threads: usize) -> Range<usize> {
    let size = r.len().div_ceil(threads).max(MIN_CHUNK);
    let start = (r.start + w * size).min(r.end);
    start..(start + size).min(r.end)
}

/// Sequential impulses over `colored` (the contacts in `colors`' order),
/// each color's contacts spread over `threads`, a barrier between colors.
fn solve_colored(pool: &Pool, threads: usize, bodies: &mut [SolverBody], colored: &mut [Constraint], colors: &Colors, dt: f32) {
    solve_colored_timed(pool, threads, bodies, colored, colors, dt, None);
}

/// With `waits`, each thread adds the ns it spent at barriers to its own.
fn solve_colored_timed(
    pool: &Pool,
    threads: usize,
    bodies: &mut [SolverBody],
    colored: &mut [Constraint],
    colors: &Colors,
    dt: f32,
    waits: Option<&[Padded<AtomicU64>]>,
) {
    let n = colored.len();
    let mut targets = vec![0.0f32; n];
    let mut pj = vec![0.0f32; n];
    let (b, cs, tg, pj) = (Ptr::of(bodies), Ptr::of(colored), Ptr::of(&mut targets), Ptr::of(&mut pj));
    let barrier = Barrier::new(threads);
    let overflow = colors.overflow();
    let groups: Vec<usize> = (0..colors.groups.len()).filter(|&g| !colors.groups[g].is_empty()).collect();
    let sunk: Vec<usize> = (0..colors.sunk.len()).filter(|&g| !colors.sunk[g].is_empty()).collect();
    // A thread's share of group `g` of `len`: all of the overflow for
    // thread 0, none for the rest.
    let mine = |g: usize, len: usize, w: usize| {
        if g != overflow {
            chunk(&(0..len), w, threads)
        } else if w == 0 {
            0..len
        } else {
            0..0
        }
    };
    let wait = |w: usize| match waits {
        None => barrier.wait(),
        Some(waits) => {
            let start = Instant::now();
            barrier.wait();
            waits[w].0.fetch_add(start.elapsed().as_nanos() as u64, Ordering::Relaxed);
        }
    };
    // SAFETY, throughout: between two barriers, a thread writes only the
    // contacts of its chunk of one color and their moving bodies, which no
    // other contact of the color has (`color`); the overflow is one
    // thread's. Bodies that don't move are only read.
    let job = |w: usize| unsafe {
        for i in chunk(&(0..n), w, threads) {
            *tg.at(i) = target(b, &mut *cs.at(i), dt);
        }
        wait(w);
        for &g in &groups {
            let r = &colors.groups[g];
            for i in mine(g, r.len(), w) {
                warm(b, &*cs.at(r.start + i));
            }
            wait(w);
        }
        for _ in 0..ITERATIONS {
            for &g in &groups {
                let r = &colors.groups[g];
                for i in mine(g, r.len(), w) {
                    let i = r.start + i;
                    relax(b, &mut *cs.at(i), *tg.at(i));
                }
                wait(w);
            }
        }
        for _ in 0..ITERATIONS {
            for &g in &sunk {
                let list = &colors.sunk[g];
                for &i in &list[mine(g, list.len(), w)] {
                    relax_pseudo(b, &*cs.at(i as usize), &mut *pj.at(i as usize), dt);
                }
                wait(w);
            }
        }
    };
    if threads == 1 {
        job(0);
    } else {
        pool.run(threads, &job);
    }
}

// ---- Wide: a color's contacts in batches of lanes ----

/// Lanes a batch: 8, AVX2's `f32`s, when the kernels are built for it.
const W: usize = 8;

/// `W` contacts of one color, field by field, as Box2D v3 keeps a color's
/// contacts for SIMD. Inverse masses are copied in, since a batch would
/// otherwise gather them from bodies on every pass for values that never
/// change. A lane past the color's end points both ends at the stand-in
/// for statics, which doesn't move, so it computes nothing it keeps.
#[derive(Clone, Copy, Default)]
struct VBatch {
    a: [u32; W],
    b: [u32; W],
    ia: [f32; W],
    ib: [f32; W],
    nx: [f32; W],
    ny: [f32; W],
    friction: [f32; W],
    target: [f32; W],
    jn: [f32; W],
    jt: [f32; W],
}

/// The split impulse's batches: only contacts sunk past the slop.
#[derive(Clone, Copy, Default)]
struct PBatch {
    a: [u32; W],
    b: [u32; W],
    ia: [f32; W],
    ib: [f32; W],
    nx: [f32; W],
    ny: [f32; W],
    bias: [f32; W],
    pj: [f32; W],
}

struct Wide {
    v: Vec<VBatch>,
    /// Batch ranges of `v` by color.
    v_groups: Vec<Range<usize>>,
    p: Vec<PBatch>,
    p_groups: Vec<Range<usize>>,
    /// Where each contact in color order landed: (batch, lane).
    lane: Vec<(u32, u8)>,
}

impl Wide {
    /// Transposes `colored` into batches, working out each contact's
    /// target as the scalar solve does first (from `bodies` before any
    /// impulse), and its speed with it.
    fn of(bodies: &[SolverBody], colored: &mut [Constraint], colors: &Colors, dt: f32) -> Wide {
        assert!(colors.groups[colors.overflow()].is_empty(), "the wide solve has no overflow: a scene that needs one needs it written");
        let still = (bodies.len() - 1) as u32;
        assert_eq!(bodies[still as usize].inv_mass, 0.0, "the last body stands for statics");
        let pad = |x: &mut VBatch, from: usize| {
            for l in from..W {
                (x.a[l], x.b[l]) = (still, still);
            }
        };
        let (mut v, mut v_groups, mut lane) = (Vec::new(), Vec::new(), vec![(u32::MAX, 0u8); colored.len()]);
        let bp = Ptr(bodies.as_ptr() as *mut SolverBody);
        for g in &colors.groups[..colors.overflow()] {
            let start = v.len();
            for (k, i) in g.clone().enumerate() {
                if k % W == 0 {
                    v.push(VBatch::default());
                }
                let (x, l) = (v.last_mut().unwrap(), k % W);
                let c = &mut colored[i];
                // SAFETY: only read, on this thread.
                let t = unsafe { target(bp, c, dt) };
                (x.a[l], x.b[l], x.nx[l], x.ny[l]) = (c.a, c.b, c.normal.x, c.normal.y);
                (x.ia[l], x.ib[l]) = (bodies[c.a as usize].inv_mass, bodies[c.b as usize].inv_mass);
                (x.friction[l], x.target[l], x.jn[l], x.jt[l]) = (c.friction, t, c.jn, c.jt);
                lane[i] = ((v.len() - 1) as u32, l as u8);
            }
            if g.len() % W != 0 {
                pad(v.last_mut().unwrap(), g.len() % W);
            }
            v_groups.push(start..v.len());
        }
        // Inert contacts (after the overflow): only their speed.
        for c in &mut colored[colors.groups[colors.overflow()].end..] {
            // SAFETY: as above.
            unsafe { target(bp, c, dt) };
        }
        let (mut p, mut p_groups) = (Vec::new(), Vec::new());
        for list in &colors.sunk[..colors.overflow()] {
            let start = p.len();
            for (k, &i) in list.iter().enumerate() {
                if k % W == 0 {
                    p.push(PBatch { a: [still; W], b: [still; W], ..PBatch::default() });
                }
                let (x, l) = (p.last_mut().unwrap(), k % W);
                let c = &colored[i as usize];
                (x.a[l], x.b[l], x.nx[l], x.ny[l]) = (c.a, c.b, c.normal.x, c.normal.y);
                (x.ia[l], x.ib[l]) = (bodies[c.a as usize].inv_mass, bodies[c.b as usize].inv_mass);
                // What the scalar solve works out on every pass, once.
                x.bias[l] = BETA * (c.depth - SLOP) / dt;
            }
            p_groups.push(start..p.len());
        }
        Wide { v, v_groups, p, p_groups, lane }
    }

    /// The impulses back into the contacts, for the step to keep.
    fn finish(&self, colored: &mut [Constraint]) {
        for (c, &(k, l)) in colored.iter_mut().zip(&self.lane) {
            if k != u32::MAX {
                let x = &self.v[k as usize];
                (c.jn, c.jt) = (x.jn[l as usize], x.jt[l as usize]);
            }
        }
    }
}

// The scalar steps (`warm`, `relax`, `relax_pseudo`) lane by lane, with
// the same operations in the same order, so that a wide solve is the
// colored solve bit for bit (checked). A branch in the scalar step is a
// select here: no lane skips, a lane that would have keeps what it had.
// Built for AVX2 (`main` checks the CPU has it), so the lane loops become
// 8-wide instructions, where the build's baseline target, SSE2, would split
// each in two. Not FMA: a fused multiply-add rounds once where the scalar
// steps round twice, so it would be another computation.

/// `f32::clamp` without its assert that `lo <= hi`, which is a branch per
/// lane and stops the loop vectorizing. It holds: `limit` is a friction
/// times an impulse clamped at 0, and the scalar step's clamp asserts it.
#[inline(always)]
fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    let x = if x < lo { lo } else { x };
    if x > hi { hi } else { x }
}

#[inline(always)]
unsafe fn gather(b: Ptr<SolverBody>, ends: &[u32; W], pseudo: bool) -> ([f32; W], [f32; W]) {
    let (mut x, mut y) = ([0.0; W], [0.0; W]);
    for l in 0..W {
        let s = unsafe { *b.at(ends[l] as usize) };
        let v = if pseudo { s.pseudo } else { s.v };
        (x[l], y[l]) = (v.x, v.y);
    }
    (x, y)
}

#[inline(always)]
unsafe fn scatter(b: Ptr<SolverBody>, ends: &[u32; W], inv: &[f32; W], x: &[f32; W], y: &[f32; W], pseudo: bool) {
    for l in 0..W {
        if inv[l] != 0.0 {
            let s = unsafe { &mut *b.at(ends[l] as usize) };
            let v = if pseudo { &mut s.pseudo } else { &mut s.v };
            *v = Vec2::new(x[l], y[l]);
        }
    }
}

#[target_feature(enable = "avx2")]
unsafe fn warm_wide(b: Ptr<SolverBody>, x: &VBatch) {
    let ((mut ax, mut ay), (mut bx, mut by)) = unsafe { (gather(b, &x.a, false), gather(b, &x.b, false)) };
    for l in 0..W {
        let ix = x.nx[l] * x.jn[l] + -x.ny[l] * x.jt[l];
        let iy = x.ny[l] * x.jn[l] + x.nx[l] * x.jt[l];
        (ax[l], ay[l]) = (ax[l] - ix * x.ia[l], ay[l] - iy * x.ia[l]);
        (bx[l], by[l]) = (bx[l] + ix * x.ib[l], by[l] + iy * x.ib[l]);
    }
    unsafe {
        scatter(b, &x.a, &x.ia, &ax, &ay, false);
        scatter(b, &x.b, &x.ib, &bx, &by, false);
    }
}

#[target_feature(enable = "avx2")]
unsafe fn relax_wide(b: Ptr<SolverBody>, x: &mut VBatch) {
    let ((mut ax, mut ay), (mut bx, mut by)) = unsafe { (gather(b, &x.a, false), gather(b, &x.b, false)) };
    for l in 0..W {
        let (ia, ib, nx, ny) = (x.ia[l], x.ib[l], x.nx[l], x.ny[l]);
        let k = ia + ib;
        let live = k != 0.0;
        let (rx, ry) = (bx[l] - ax[l], by[l] - ay[l]);
        let jn = (x.jn[l] + (x.target[l] - (rx * nx + ry * ny)) / k).max(0.0);
        let dn = jn - x.jn[l];
        let jn = if live { jn } else { x.jn[l] };
        let (px, py) = (nx * dn, ny * dn);
        // A body that doesn't move keeps its velocity: the scalar step
        // doesn't write it.
        let (a1x, a1y) = if ia != 0.0 && live { (ax[l] - px * ia, ay[l] - py * ia) } else { (ax[l], ay[l]) };
        let (b1x, b1y) = if ib != 0.0 && live { (bx[l] + px * ib, by[l] + py * ib) } else { (bx[l], by[l]) };

        let (tx, ty) = (-ny, nx);
        let (rx, ry) = (b1x - a1x, b1y - a1y);
        let limit = x.friction[l] * jn;
        let jt = clamp(x.jt[l] - (rx * tx + ry * ty) / k, -limit, limit);
        let d = jt - x.jt[l];
        let jt = if live { jt } else { x.jt[l] };
        let (qx, qy) = (tx * d, ty * d);
        (ax[l], ay[l]) = if ia != 0.0 && live { (a1x - qx * ia, a1y - qy * ia) } else { (a1x, a1y) };
        (bx[l], by[l]) = if ib != 0.0 && live { (b1x + qx * ib, b1y + qy * ib) } else { (b1x, b1y) };
        (x.jn[l], x.jt[l]) = (jn, jt);
    }
    unsafe {
        scatter(b, &x.a, &x.ia, &ax, &ay, false);
        scatter(b, &x.b, &x.ib, &bx, &by, false);
    }
}

#[target_feature(enable = "avx2")]
unsafe fn relax_pseudo_wide(b: Ptr<SolverBody>, x: &mut PBatch) {
    let ((mut ax, mut ay), (mut bx, mut by)) = unsafe { (gather(b, &x.a, true), gather(b, &x.b, true)) };
    for l in 0..W {
        let (ia, ib, nx, ny) = (x.ia[l], x.ib[l], x.nx[l], x.ny[l]);
        let k = ia + ib;
        // Padding lanes have k = 0: the only ones, since the list holds
        // only contacts that move something.
        let live = k != 0.0;
        let (rx, ry) = (bx[l] - ax[l], by[l] - ay[l]);
        let new = (x.pj[l] + (x.bias[l] - (rx * nx + ry * ny)) / k).max(0.0);
        let d = new - x.pj[l];
        let (px, py) = (nx * d, ny * d);
        if live {
            x.pj[l] = new;
        }
        if ia != 0.0 && live {
            (ax[l], ay[l]) = (ax[l] - px * ia, ay[l] - py * ia);
        }
        if ib != 0.0 && live {
            (bx[l], by[l]) = (bx[l] + px * ib, by[l] + py * ib);
        }
    }
    unsafe {
        scatter(b, &x.a, &x.ia, &ax, &ay, true);
        scatter(b, &x.b, &x.ib, &bx, &by, true);
    }
}

/// Batches of a color smaller than this aren't split further (as
/// `MIN_CHUNK`, in batches).
const MIN_BATCHES: usize = MIN_CHUNK / W;

fn chunk_of(r: &Range<usize>, w: usize, threads: usize, min: usize) -> Range<usize> {
    let size = r.len().div_ceil(threads).max(min);
    let start = (r.start + w * size).min(r.end);
    start..(start + size).min(r.end)
}

/// The colored solve over `wide`'s batches: its colors in turn, each one's
/// batches spread over `threads`.
fn solve_wide(pool: &Pool, threads: usize, bodies: &mut [SolverBody], wide: &mut Wide) {
    let (b, v, p) = (Ptr::of(bodies), Ptr::of(&mut wide.v), Ptr::of(&mut wide.p));
    let barrier = Barrier::new(threads);
    let (v_groups, p_groups) = (&wide.v_groups, &wide.p_groups);
    // SAFETY: as `solve_colored`'s, batch for contact.
    let job = |w: usize| unsafe {
        for g in v_groups.iter().filter(|g| !g.is_empty()) {
            for k in chunk_of(g, w, threads, MIN_BATCHES) {
                warm_wide(b, &*v.at(k));
            }
            barrier.wait();
        }
        for _ in 0..ITERATIONS {
            for g in v_groups.iter().filter(|g| !g.is_empty()) {
                for k in chunk_of(g, w, threads, MIN_BATCHES) {
                    relax_wide(b, &mut *v.at(k));
                }
                barrier.wait();
            }
        }
        for _ in 0..ITERATIONS {
            for g in p_groups.iter().filter(|g| !g.is_empty()) {
                for k in chunk_of(g, w, threads, MIN_BATCHES) {
                    relax_pseudo_wide(b, &mut *p.at(k));
                }
                barrier.wait();
            }
        }
    };
    if threads == 1 {
        job(0);
    } else {
        pool.run(threads, &job);
    }
}

// ---- Islands ----

struct Islands {
    /// Pair-order indices of the contacts that move something, grouped by
    /// island, pair order within each.
    contacts: Vec<u32>,
    /// Ranges of `contacts`, largest island first (ties by first contact),
    /// the order threads take them in.
    islands: Vec<Range<usize>>,
    /// Contacts neither end of which moves: only their speed to report.
    inert: Vec<u32>,
    /// Bodies per island, for the report, in the same order.
    bodies: Vec<usize>,
}

fn find(parent: &mut [u32], mut i: u32) -> u32 {
    while parent[i as usize] != i {
        parent[i as usize] = parent[parent[i as usize] as usize];
        i = parent[i as usize];
    }
    i
}

impl Islands {
    /// Union-find over contacts between two moving bodies: a body that
    /// doesn't move joins nothing, as in sleeping's islands.
    fn of(bodies: &[SolverBody], contacts: &[Constraint]) -> Islands {
        let moves = |i: u32| bodies[i as usize].inv_mass != 0.0;
        let mut parent: Vec<u32> = (0..bodies.len() as u32).collect();
        for c in contacts {
            if moves(c.a) && moves(c.b) {
                let (ra, rb) = (find(&mut parent, c.a), find(&mut parent, c.b));
                if ra != rb {
                    // The smaller root wins, so islands don't depend on
                    // the order contacts were joined in.
                    let (lo, hi) = (ra.min(rb), ra.max(rb));
                    parent[hi as usize] = lo;
                }
            }
        }
        let mut id = vec![u32::MAX; bodies.len()];
        let mut count: Vec<usize> = Vec::new();
        let mut inert = Vec::new();
        let mut island_of = Vec::with_capacity(contacts.len());
        for (k, c) in contacts.iter().enumerate() {
            let end = if moves(c.a) { c.a } else if moves(c.b) { c.b } else {
                inert.push(k as u32);
                island_of.push(u32::MAX);
                continue;
            };
            let r = find(&mut parent, end) as usize;
            if id[r] == u32::MAX {
                id[r] = count.len() as u32;
                count.push(0);
            }
            count[id[r] as usize] += 1;
            island_of.push(id[r]);
        }
        let mut body_count = vec![0usize; count.len()];
        for i in 0..bodies.len() as u32 {
            if moves(i) {
                let r = find(&mut parent, i) as usize;
                if id[r] != u32::MAX {
                    body_count[id[r] as usize] += 1;
                }
            }
        }
        let mut start = vec![0usize; count.len() + 1];
        for k in 0..count.len() {
            start[k + 1] = start[k] + count[k];
        }
        let mut at = start.clone();
        let mut grouped = vec![0u32; island_of.len() - inert.len()];
        for (k, &i) in island_of.iter().enumerate() {
            if i != u32::MAX {
                grouped[at[i as usize]] = k as u32;
                at[i as usize] += 1;
            }
        }
        let mut by_size: Vec<usize> = (0..count.len()).collect();
        by_size.sort_by_key(|&i| std::cmp::Reverse(count[i]));
        Islands {
            contacts: grouped,
            islands: by_size.iter().map(|&i| start[i]..start[i + 1]).collect(),
            bodies: by_size.iter().map(|&i| body_count[i]).collect(),
            inert,
        }
    }
}

/// Each island solved by one thread as `solver::solve` would, threads
/// taking the next island as they finish, largest first. Which thread takes
/// which doesn't change what's computed: islands share no moving body.
fn solve_islands(pool: &Pool, threads: usize, bodies: &mut [SolverBody], contacts: &mut [Constraint], islands: &Islands, dt: f32) {
    let n = contacts.len();
    let mut targets = vec![0.0f32; n];
    let mut pj = vec![0.0f32; n];
    let (b, cs, tg, pj) = (Ptr::of(bodies), Ptr::of(contacts), Ptr::of(&mut targets), Ptr::of(&mut pj));
    let next = AtomicUsize::new(0);
    // SAFETY: an island's contacts and moving bodies are only its thread's.
    let job = |w: usize| unsafe {
        if w == 0 {
            for &i in &islands.inert {
                target(b, &mut *cs.at(i as usize), dt);
            }
        }
        loop {
            let k = next.fetch_add(1, Ordering::Relaxed);
            let Some(r) = islands.islands.get(k) else { break };
            let list = &islands.contacts[r.clone()];
            for &i in list {
                *tg.at(i as usize) = target(b, &mut *cs.at(i as usize), dt);
            }
            for &i in list {
                warm(b, &*cs.at(i as usize));
            }
            for _ in 0..ITERATIONS {
                for &i in list {
                    relax(b, &mut *cs.at(i as usize), *tg.at(i as usize));
                }
            }
            for _ in 0..ITERATIONS {
                for &i in list {
                    relax_pseudo(b, &*cs.at(i as usize), &mut *pj.at(i as usize), dt);
                }
            }
        }
    };
    if threads == 1 {
        job(0);
    } else {
        pool.run(threads, &job);
    }
}

/// Islands dealt to threads in runs of consecutive islands (by first
/// contact) of about equal contacts, each thread's solved together in pair
/// order: still `solver::solve`'s computation, since islands share no
/// moving body, but with contacts of different islands interleaved as pair
/// order has them. One island at a time, each contact depends on the last
/// through the body they share, and the loop waits on it.
struct Batches {
    /// Per thread, its contacts' pair-order indices, in pair order.
    contacts: Vec<Vec<u32>>,
    inert: Vec<u32>,
}

impl Batches {
    fn of(bodies: &[SolverBody], contacts: &[Constraint], threads: usize) -> Batches {
        let moves = |i: u32| bodies[i as usize].inv_mass != 0.0;
        let mut parent: Vec<u32> = (0..bodies.len() as u32).collect();
        for c in contacts {
            if moves(c.a) && moves(c.b) {
                let (ra, rb) = (find(&mut parent, c.a), find(&mut parent, c.b));
                if ra != rb {
                    parent[ra.max(rb) as usize] = ra.min(rb);
                }
            }
        }
        // Islands numbered by first contact, and their sizes.
        let mut id = vec![u32::MAX; bodies.len()];
        let mut size: Vec<usize> = Vec::new();
        let island: Vec<u32> = contacts
            .iter()
            .map(|c| {
                let end = if moves(c.a) { c.a } else if moves(c.b) { c.b } else { return u32::MAX };
                let r = find(&mut parent, end) as usize;
                if id[r] == u32::MAX {
                    id[r] = size.len() as u32;
                    size.push(0);
                }
                size[id[r] as usize] += 1;
                id[r]
            })
            .collect();
        let total: usize = size.iter().sum();
        let (mut thread_of, mut t, mut filled) = (Vec::with_capacity(size.len()), 0usize, 0usize);
        for &n in &size {
            if filled >= (t + 1) * total / threads && t + 1 < threads {
                t += 1;
            }
            thread_of.push(t as u32);
            filled += n;
        }
        let mut out = Batches { contacts: vec![Vec::new(); threads], inert: Vec::new() };
        for (k, &i) in island.iter().enumerate() {
            if i == u32::MAX {
                out.inert.push(k as u32);
            } else {
                out.contacts[thread_of[i as usize] as usize].push(k as u32);
            }
        }
        out
    }
}

fn solve_batches(pool: &Pool, threads: usize, bodies: &mut [SolverBody], contacts: &mut [Constraint], batches: &Batches, dt: f32) {
    let n = contacts.len();
    let mut targets = vec![0.0f32; n];
    let mut pj = vec![0.0f32; n];
    let (b, cs, tg, pj) = (Ptr::of(bodies), Ptr::of(contacts), Ptr::of(&mut targets), Ptr::of(&mut pj));
    // SAFETY: a thread's contacts and their moving bodies are only its own.
    let job = |w: usize| unsafe {
        if w == 0 {
            for &i in &batches.inert {
                target(b, &mut *cs.at(i as usize), dt);
            }
        }
        let list = &batches.contacts[w];
        for &i in list {
            *tg.at(i as usize) = target(b, &mut *cs.at(i as usize), dt);
        }
        for &i in list {
            warm(b, &*cs.at(i as usize));
        }
        for _ in 0..ITERATIONS {
            for &i in list {
                relax(b, &mut *cs.at(i as usize), *tg.at(i as usize));
            }
        }
        for _ in 0..ITERATIONS {
            for &i in list {
                relax_pseudo(b, &*cs.at(i as usize), &mut *pj.at(i as usize), dt);
            }
        }
    };
    if threads == 1 {
        job(0);
    } else {
        pool.run(threads, &job);
    }
}

// ---- Scenes ----

/// The solver's input as the step on arrays gathers it.
#[derive(Clone)]
struct Input {
    bodies: Vec<SolverBody>,
    contacts: Vec<Constraint>,
}

/// Takes one more step, keeping what its solver was given.
fn capture(a: &mut Arrays) -> Input {
    let mut input = None;
    a.step(&mut Stages::default(), |b, c, dt| {
        input = Some(Input { bodies: b.to_vec(), contacts: c.to_vec() });
        solver::solve(b, c, dt);
    });
    input.unwrap()
}

fn pile(manifest: &engine_control::Manifest, n: u32, width: f32, steps: u32) -> Arrays {
    let dir = std::env::temp_dir().join(format!("physics-parallel-{}-{n}-{steps}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
    e.load_batch(&manifest.mods).expect("loading the pile");
    e.send("pile", &format!("widen {width}")).unwrap();
    e.send("pile", &format!("drop {n}")).unwrap();
    e.send("lockstep", &format!("step {steps}")).unwrap();
    let mut a = Arrays::snapshot(e.world());
    a.time_fresh_sweep = false;
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
    a
}

/// `stacks` columns of `high` unit boxes on one floor, 2 apart: each column
/// an island, since the floor doesn't move and so joins nothing.
fn stacks(stacks: usize, high: usize) -> Arrays {
    const H: f32 = 0.45;
    let (mut pos, mut collider, mut body, mut moving) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let width = stacks as f32 * 2.0 + 2.0;
    pos.push(Vec2::new(width / 2.0, 1.0));
    collider.push(Collider::rect(width / 2.0, 0.5));
    body.push(Body::fixed());
    for s in 0..stacks {
        for k in 0..high {
            // A little apart, so each lands; offset by the index, so the
            // columns aren't all alike.
            let jitter = ((s * 31 + k * 7) % 10) as f32 * 0.01 - 0.05;
            moving.push(pos.len() as u32);
            pos.push(Vec2::new(2.0 + s as f32 * 2.0 + jitter, 0.5 - H - 0.02 - k as f32 * (2.0 * H + 0.02)));
            collider.push(Collider::rect(H, H));
            body.push(Body { friction: 0.6, restitution: 0.0, ..Body::default() });
        }
    }
    assert!(body.iter().skip(1).all(|b| b.kind == DYNAMIC));
    Arrays::of(pos, collider, body, moving, Vec2::new(0.0, 20.0))
}

// ---- Checking and timing ----

/// The solver's outputs: velocity and pseudo velocity by body, then
/// impulses and speed by contact in pair order, as bits.
type Outcome = (Vec<[u32; 4]>, Vec<[u32; 3]>);

fn outcome(bodies: &[SolverBody], contacts: &[Constraint]) -> Outcome {
    (
        bodies.iter().map(|b| [b.v.x.to_bits(), b.v.y.to_bits(), b.pseudo.x.to_bits(), b.pseudo.y.to_bits()]).collect(),
        contacts.iter().map(|c| [c.jn.to_bits(), c.jt.to_bits(), c.speed.to_bits()]).collect(),
    )
}

/// Median, and the 10th and 90th percentiles, of `xs` µs.
fn spread(mut xs: Vec<f64>) -> (f64, f64, f64) {
    xs.sort_by(f64::total_cmp);
    let at = |q: f64| xs[((xs.len() - 1) as f64 * q).round() as usize];
    (at(0.5), at(0.1), at(0.9))
}

fn us(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1e6
}

/// A run to time, on how many threads: fresh inputs, then the run, in µs.
type Variant<'a> = (usize, Box<dyn FnMut() -> f64 + 'a>);

/// Runs each variant `REPS` times, after keeping its threads busy long
/// enough for the governor to clock their cores up (`warm_up`). Not round
/// robin, as the other benches run: switching thread counts leaves cores
/// idle between a variant's runs, and each would start cold.
fn timings(pool: &Pool, variants: &mut [Variant]) -> Vec<(f64, f64, f64)> {
    let mut times = vec![Vec::new(); variants.len()];
    for (k, (threads, v)) in variants.iter_mut().enumerate() {
        warm_up(pool, *threads, 300);
        for _ in 0..REPS {
            times[k].push(v());
        }
    }
    if std::env::var("RAW").is_ok_and(|v| v == "1") {
        for t in &times {
            println!("  raw: {}", t.iter().map(|x| format!("{x:.0}")).collect::<Vec<_>>().join(" "));
        }
    }
    times.into_iter().map(spread).collect()
}

fn check_coloring(input: &Input, colors: &[u8]) {
    let mut seen = vec![0u32; input.bodies.len()];
    for (c, &k) in input.contacts.iter().zip(colors) {
        if k >= OVERFLOW {
            continue;
        }
        for end in [c.a, c.b] {
            if input.bodies[end as usize].inv_mass != 0.0 {
                assert_eq!(seen[end as usize] & (1 << k), 0, "two contacts of color {k} share body {end}");
                seen[end as usize] |= 1 << k;
            }
        }
    }
}

fn histogram(colors: &[u8], of: impl Fn(usize) -> bool) -> Vec<usize> {
    let mut h = vec![0usize; COLORS + 2];
    for (i, &c) in colors.iter().enumerate() {
        if of(i) {
            h[c as usize] += 1;
        }
    }
    h
}

fn show_colors(input: &Input, rule: Rule) {
    let colors = color(&input.bodies, &input.contacts, rule);
    check_coloring(input, &colors);
    let all = histogram(&colors, |_| true);
    let sunk = histogram(&colors, |i| input.contacts[i].depth > SLOP);
    let used = (0..COLORS).filter(|&k| all[k] > 0).count();
    let row = |h: &[usize]| (0..COLORS).filter(|&k| all[k] > 0).map(|k| h[k].to_string()).collect::<Vec<_>>().join(" ");
    println!(
        "  {} rule: {used} colors; contacts per color: {}; overflow {}, inert {}",
        if rule == Rule::Box2d { "Box2D's" } else { "greedy" },
        row(&all),
        all[OVERFLOW as usize],
        all[INERT as usize],
    );
    println!("    of which sunk past the slop (the split impulse's): {}", row(&sunk));
}

/// Every thread count must give the same bits: the answer is the colors'.
fn scene(name: &str, input: &Input, pool: &Pool, threads: &[usize]) {
    let n_moving = input.bodies.iter().filter(|b| b.inv_mass != 0.0).count();
    let sunk = input.contacts.iter().filter(|c| c.depth > SLOP).count();
    println!("\n### {name}: {n_moving} moving bodies, {} contacts, {sunk} sunk past the slop\n", input.contacts.len());
    show_colors(input, Rule::Box2d);
    show_colors(input, Rule::Greedy);
    let islands = Islands::of(&input.bodies, &input.contacts);
    let largest = islands.islands.first().map_or(0, |r| r.len());
    println!(
        "  islands: {}, the largest {} bodies and {largest} contacts ({:.0}% of the contacts), the next {:?}",
        islands.islands.len(),
        islands.bodies.first().copied().unwrap_or(0),
        100.0 * largest as f64 / islands.contacts.len().max(1) as f64,
        islands.islands.iter().skip(1).take(5).map(|r| r.len()).collect::<Vec<_>>(),
    );

    // Correctness first: the steps, in pair order, are `solver::solve`'s.
    let reference = {
        let mut i = input.clone();
        solver::solve(&mut i.bodies, &mut i.contacts, DT);
        outcome(&i.bodies, &i.contacts)
    };
    {
        let single = Colors::single(input.contacts.len(), &input.contacts);
        let mut i = input.clone();
        let mut colored = single.permute(&i.contacts);
        solve_colored(pool, 1, &mut i.bodies, &mut colored, &single, DT);
        single.unpermute(&colored, &mut i.contacts);
        assert!(outcome(&i.bodies, &i.contacts) == reference, "{name}: the colored solve's steps in pair order aren't solver::solve's");
    }
    let colors = Colors::of(&color(&input.bodies, &input.contacts, Rule::Box2d), &input.contacts);
    let colored_run = |t: usize| {
        let mut i = input.clone();
        let mut colored = colors.permute(&i.contacts);
        solve_colored(pool, t, &mut i.bodies, &mut colored, &colors, DT);
        colors.unpermute(&colored, &mut i.contacts);
        outcome(&i.bodies, &i.contacts)
    };
    let colored_1 = colored_run(1);
    let wide_run = |t: usize| {
        let mut i = input.clone();
        let mut colored = colors.permute(&i.contacts);
        let mut wide = Wide::of(&i.bodies, &mut colored, &colors, DT);
        solve_wide(pool, t, &mut i.bodies, &mut wide);
        wide.finish(&mut colored);
        colors.unpermute(&colored, &mut i.contacts);
        outcome(&i.bodies, &i.contacts)
    };
    let differ = colored_1.0.iter().zip(&reference.0).filter(|(a, b)| a != b).count();
    println!("  colored vs serial: {differ} of {} bodies' velocities differ (a different order of solving)", reference.0.len());
    for &t in threads {
        for _ in 0..3 {
            assert!(colored_run(t) == colored_1, "{name}: colored on {t} threads differs from colored on 1");
            assert!(wide_run(t) == colored_1, "{name}: wide on {t} threads differs from colored on 1");
            let mut i = input.clone();
            solve_islands(pool, t, &mut i.bodies, &mut i.contacts, &islands, DT);
            assert!(outcome(&i.bodies, &i.contacts) == reference, "{name}: islands on {t} threads differ from serial");
            let mut i = input.clone();
            solve_batches(pool, t, &mut i.bodies, &mut i.contacts, &Batches::of(&input.bodies, &input.contacts, t), DT);
            assert!(outcome(&i.bodies, &i.contacts) == reference, "{name}: island batches on {t} threads differ from serial");
        }
    }
    println!("  bit for bit: colored and wide the same on every thread count; islands and island batches the same as serial on every count (3 runs each)");

    // What building them costs a step.
    let mut build: Vec<Variant> = vec![
        (1, Box::new(|| {
            let start = Instant::now();
            black_box(color(&input.bodies, &input.contacts, Rule::Box2d));
            us(start)
        })),
        (1, Box::new(|| {
            let c = color(&input.bodies, &input.contacts, Rule::Box2d);
            let start = Instant::now();
            let colors = Colors::of(&c, &input.contacts);
            black_box(colors.permute(&input.contacts));
            us(start)
        })),
        (1, Box::new(|| {
            let mut contacts = input.contacts.clone();
            let colored = colors.permute(&contacts);
            let start = Instant::now();
            colors.unpermute(&colored, &mut contacts);
            black_box(&contacts);
            us(start)
        })),
        (1, Box::new(|| {
            let start = Instant::now();
            black_box(Islands::of(&input.bodies, &input.contacts));
            us(start)
        })),
        (1, Box::new(|| {
            let start = Instant::now();
            black_box(Batches::of(&input.bodies, &input.contacts, 16));
            us(start)
        })),
        (1, Box::new(|| {
            let mut colored = colors.permute(&input.contacts);
            let start = Instant::now();
            let wide = Wide::of(&input.bodies, &mut colored, &colors, DT);
            wide.finish(&mut colored);
            black_box(&colored);
            us(start)
        })),
    ];
    let b = timings(pool, &mut build);
    println!(
        "  building, µs (median, 10th-90th): coloring {:.1} ({:.1}-{:.1}), ordering and copying into color order {:.1} ({:.1}-{:.1}), copying back {:.1} ({:.1}-{:.1}), islands {:.1} ({:.1}-{:.1}), island batches for 16 threads {:.1} ({:.1}-{:.1}), \
         wide batches from color order and back, with the targets {:.1} ({:.1}-{:.1})\n",
        b[0].0, b[0].1, b[0].2, b[1].0, b[1].1, b[1].2, b[2].0, b[2].1, b[2].2, b[3].0, b[3].1, b[3].2, b[4].0, b[4].1, b[4].2,
        b[5].0, b[5].1, b[5].2
    );

    // The solves. The pool is woken just before each timed run, as a step
    // would keep it awake through the physics.
    let mut variants: Vec<Variant> = Vec::new();
    let mut names = Vec::new();
    variants.push((1, Box::new(|| {
        let mut i = input.clone();
        warm_up(pool, 1, 2);
        let start = Instant::now();
        solver::solve(&mut i.bodies, &mut i.contacts, DT);
        us(start)
    })));
    names.push(("serial".to_string(), 1));
    for &t in threads {
        let colors = &colors;
        variants.push((t, Box::new(move || {
            let mut i = input.clone();
            let mut colored = colors.permute(&i.contacts);
            warm_up(pool, t, 2);
            let start = Instant::now();
            solve_colored(pool, t, &mut i.bodies, &mut colored, colors, DT);
            us(start)
        })));
        names.push(("colored".to_string(), t));
    }
    for &t in threads {
        let colors = &colors;
        variants.push((t, Box::new(move || {
            let mut i = input.clone();
            let mut colored = colors.permute(&i.contacts);
            let mut wide = Wide::of(&i.bodies, &mut colored, colors, DT);
            warm_up(pool, t, 2);
            let start = Instant::now();
            solve_wide(pool, t, &mut i.bodies, &mut wide);
            us(start)
        })));
        names.push(("colored, wide".to_string(), t));
    }
    for &t in threads {
        let islands = &islands;
        variants.push((t, Box::new(move || {
            let mut i = input.clone();
            warm_up(pool, t, 2);
            let start = Instant::now();
            solve_islands(pool, t, &mut i.bodies, &mut i.contacts, islands, DT);
            us(start)
        })));
        names.push(("islands".to_string(), t));
    }
    for &t in threads {
        let batches = Batches::of(&input.bodies, &input.contacts, t);
        variants.push((t, Box::new(move || {
            let mut i = input.clone();
            warm_up(pool, t, 2);
            let start = Instant::now();
            solve_batches(pool, t, &mut i.bodies, &mut i.contacts, &batches, DT);
            us(start)
        })));
        names.push(("island batches".to_string(), t));
    }
    let r = timings(pool, &mut variants);
    let base = r[0].0;
    if std::env::var("DIAG").is_ok_and(|v| v == "1") {
        diagnose(input, pool, &colors, threads);
    }
    println!("| solver | threads | µs (median) | 10th-90th | speedup |");
    println!("|---|---|---|---|---|");
    for ((name, t), (m, lo, hi)) in names.iter().zip(&r) {
        println!("| {name} | {t} | {m:.0} | {lo:.0}-{hi:.0} | {:.2}× |", base / m);
    }
}

/// Where a colored solve's time goes on each thread count: the mean over
/// threads of time at barriers, and its spread, against the whole solve.
fn diagnose(input: &Input, pool: &Pool, colors: &Colors, threads: &[usize]) {
    for &t in threads {
        let mut solves = Vec::new();
        let mut waited = vec![Vec::new(); t];
        warm_up(pool, t, 300);
        for _ in 0..REPS {
            let waits: Vec<Padded<AtomicU64>> = (0..t).map(|_| Padded::default()).collect();
            let mut i = input.clone();
            let mut colored = colors.permute(&i.contacts);
            warm_up(pool, t, 2);
            let start = Instant::now();
            solve_colored_timed(pool, t, &mut i.bodies, &mut colored, colors, DT, Some(&waits));
            solves.push(us(start));
            for (w, x) in waits.iter().enumerate() {
                waited[w].push(x.0.load(Ordering::Relaxed) as f64 / 1e3);
            }
        }
        let solve = spread(solves).0;
        let per: Vec<f64> = waited.into_iter().map(|w| spread(w).0).collect();
        let (lo, hi) = (per.iter().copied().fold(f64::MAX, f64::min), per.iter().copied().fold(0.0, f64::max));
        let working: Vec<String> = per.iter().map(|w| format!("{:.0}", solve - w)).collect();
        println!("  diag {t} threads: solve {solve:.0} µs; at barriers, per thread {lo:.0}-{hi:.0} µs; working, by thread: {}", working.join(" "));
    }
}

/// The cost of the pool's hand-off and of one barrier, alone.
fn overheads(pool: &Pool, threads: &[usize]) {
    println!("\n### Synchronization alone\n");
    println!("| threads | a job handed out and joined, µs | a barrier, µs | spawning and joining scoped threads instead, µs |");
    println!("|---|---|---|---|");
    for &t in threads.iter().filter(|&&t| t > 1) {
        const BARRIERS: usize = 1000;
        let mut v: Vec<Variant> = vec![
            (t, Box::new(|| {
                pool.run(t, &|_| {});
                let start = Instant::now();
                pool.run(t, &|_| {});
                us(start)
            })),
            (t, Box::new(|| {
                let barrier = Barrier::new(t);
                pool.run(t, &|_| {});
                let start = Instant::now();
                pool.run(t, &|_| {
                    for _ in 0..BARRIERS {
                        barrier.wait();
                    }
                });
                us(start) / BARRIERS as f64
            })),
            (1, Box::new(|| {
                let start = Instant::now();
                std::thread::scope(|s| {
                    for _ in 1..t {
                        s.spawn(|| black_box(0));
                    }
                });
                us(start)
            })),
        ];
        let r = timings(pool, &mut v);
        println!(
            "| {t} | {:.1} ({:.1}-{:.1}) | {:.2} ({:.2}-{:.2}) | {:.0} ({:.0}-{:.0}) |",
            r[0].0, r[0].1, r[0].2, r[1].0, r[1].1, r[1].2, r[2].0, r[2].1, r[2].2
        );
    }
}

// ---- Settling ----

#[derive(Default)]
struct Settled {
    resting: usize,
    deepest: f32,
    mean: f32,
    fastest: f32,
    moved: usize,
}

fn settled(a: &Arrays, before: &[Vec2]) -> Settled {
    let moving: Vec<usize> = a.moving.iter().map(|&i| i as usize).filter(|&i| a.body[i].kind == DYNAMIC).collect();
    let speeds: Vec<f32> = moving.iter().map(|&i| a.vel[i].len()).collect();
    Settled {
        resting: speeds.iter().filter(|&&s| s < 0.1).count(),
        deepest: a.contacts.iter().map(|c| c.depth).fold(0.0, f32::max),
        mean: speeds.iter().sum::<f32>() / speeds.len() as f32,
        fastest: speeds.iter().copied().fold(0.0, f32::max),
        moved: moving.iter().filter(|&&i| a.pos[i].x.to_bits() != before[i].x.to_bits() || a.pos[i].y.to_bits() != before[i].y.to_bits()).count(),
    }
}

type Solve<'a> = Box<dyn FnMut(&mut [SolverBody], &mut [Constraint], f32) + 'a>;

/// The colored solve as a step's solver: colored afresh each step.
fn colored_solver(pool: &Pool, threads: usize) -> Solve<'_> {
    Box::new(move |b, c, dt| {
        let colors = Colors::of(&color(b, c, Rule::Box2d), c);
        let mut cs = colors.permute(c);
        solve_colored(pool, threads, b, &mut cs, &colors, dt);
        colors.unpermute(&cs, c);
    })
}

fn islands_solver(pool: &Pool, threads: usize) -> Solve<'_> {
    Box::new(move |b, c, dt| {
        let islands = Islands::of(b, c);
        solve_islands(pool, threads, b, c, &islands, dt);
    })
}

/// The same pile from its first step to `steps`, solved serially and
/// colored: how each settles, and, colored, the same bits on 1 and 16
/// threads at every step.
fn settling(name: &str, from: &Arrays, pool: &Pool, steps: u32) {
    let marks = [100u32, 400, 1000, 2000, 3000, steps];
    let runs: Vec<(&str, Solve, Solve)> = vec![
        ("serial (islands on 16 threads alongside)", Box::new(|b, c, dt| solver::solve(b, c, dt)), islands_solver(pool, 16)),
        ("colored (1 thread, 16 alongside)", colored_solver(pool, 1), colored_solver(pool, 16)),
    ];
    println!("\n### Settling: {name} from step 1, on arrays\n");
    println!("| solver | step | resting (< 0.1) | deepest overlap | mean speed | fastest | bodies moved this step | step µs |");
    println!("|---|---|---|---|---|---|---|---|");
    for (solver, mut solve, mut twin) in runs {
        let (mut a, mut b) = (clone_arrays(from), clone_arrays(from));
        let mut rest_at = None;
        let mut t = Stages::default();
        let mut took = 0.0;
        for step in 1..=steps {
            let before = a.pos.clone();
            let start = Instant::now();
            a.step(&mut t, &mut solve);
            took += us(start);
            b.step(&mut Stages::default(), &mut twin);
            let same = a.pos.iter().zip(&b.pos).all(|(p, q)| p.x.to_bits() == q.x.to_bits() && p.y.to_bits() == q.y.to_bits())
                && a.vel.iter().zip(&b.vel).all(|(p, q)| p.x.to_bits() == q.x.to_bits() && p.y.to_bits() == q.y.to_bits());
            assert!(same, "{name}, {solver}: the twin runs differ at step {step}");
            let s = settled(&a, &before);
            if s.moved == 0 && rest_at.is_none() {
                rest_at = Some(step);
            }
            if marks.contains(&step) {
                let per_step = took / (step - marks.iter().copied().filter(|&m| m < step).max().unwrap_or(0)) as f64;
                println!(
                    "| {solver} | {step} | {} | {:.4} | {:.4} | {:.3} | {} | {per_step:.0} |",
                    s.resting, s.deepest, s.mean, s.fastest, s.moved
                );
                took = 0.0;
            }
        }
        println!("| {solver} | at rest bit for bit from step | {} | | | | | |", rest_at.map_or("never".to_string(), |s| s.to_string()));
    }
}

fn clone_arrays(a: &Arrays) -> Arrays {
    Arrays {
        entity: a.entity.clone(),
        pos: a.pos.clone(),
        vel: a.vel.clone(),
        collider: a.collider.clone(),
        body: a.body.clone(),
        moving: a.moving.clone(),
        gravity: a.gravity,
        contacts: a.contacts.clone(),
        by_x: a.by_x.clone(),
        time_fresh_sweep: false,
    }
}

// ---- Colors from step to step ----

/// If colors were kept in storage (an ordered key, or a table per color), a
/// contact whose color changes would move. How many do, recoloring from
/// scratch each step, and how many a coloring that keeps each persisting
/// contact's color would have to make, and at what cost in colors.
fn churn(from: &Arrays, steps: u32) {
    let mut a = clone_arrays(from);
    let mut prev: Vec<((u32, u32), u8)> = Vec::new();
    let mut kept: Vec<((u32, u32), u8)> = Vec::new();
    let (mut changed, mut persisted, mut sticky_colors, mut fresh_colors) = (0usize, 0usize, 0usize, 0usize);
    for _ in 0..steps {
        let input = capture(&mut a);
        // By entity, not by the solver's index: statics all share one.
        let colors = color(&input.bodies, &input.contacts, Rule::Box2d);
        let now: Vec<((u32, u32), u8)> = a.contacts.iter().map(|c| (c.a, c.b)).zip(colors.iter().copied()).collect();
        // Recolored from scratch: how many persisting contacts changed.
        let mut k = 0;
        for &(p, c) in &now {
            while k < prev.len() && prev[k].0 < p {
                k += 1;
            }
            if k < prev.len() && prev[k].0 == p {
                persisted += 1;
                changed += (prev[k].1 != c) as usize;
            }
        }
        // Kept: persisting contacts keep last step's color, new ones take
        // the lowest free, in pair order after all the kept ones.
        let mut used = vec![0u32; input.bodies.len()];
        let mut sticky = vec![u8::MAX; now.len()];
        let mut k = 0;
        for (i, &(p, _)) in now.iter().enumerate() {
            while k < kept.len() && kept[k].0 < p {
                k += 1;
            }
            if k < kept.len() && kept[k].0 == p && kept[k].1 < OVERFLOW {
                let c = kept[k].1;
                let (ca, cb) = (&input.contacts[i].a, &input.contacts[i].b);
                let clash = [ca, cb].iter().any(|&&e| input.bodies[e as usize].inv_mass != 0.0 && used[e as usize] & (1 << c) != 0);
                if !clash {
                    for e in [*ca, *cb] {
                        if input.bodies[e as usize].inv_mass != 0.0 {
                            used[e as usize] |= 1 << c;
                        }
                    }
                    sticky[i] = c;
                }
            }
        }
        for (i, c) in input.contacts.iter().enumerate() {
            if sticky[i] != u8::MAX {
                continue;
            }
            let (ma, mb) = (input.bodies[c.a as usize].inv_mass != 0.0, input.bodies[c.b as usize].inv_mass != 0.0);
            if !ma && !mb {
                sticky[i] = INERT;
                continue;
            }
            let taken = if ma { used[c.a as usize] } else { 0 } | if mb { used[c.b as usize] } else { 0 };
            let allowed: u32 = if ma && mb { (1 << DYNAMIC_COLORS) - 1 } else { ((1 << COLORS) - 1) & !1 };
            let free = allowed & !taken;
            sticky[i] = if free == 0 { OVERFLOW } else { free.trailing_zeros() as u8 };
            if free != 0 {
                for (m, e) in [(ma, c.a), (mb, c.b)] {
                    if m {
                        used[e as usize] |= 1 << sticky[i];
                    }
                }
            }
        }
        let count = |cs: &[u8]| (0..COLORS as u8).filter(|k| cs.contains(k)).count();
        sticky_colors = sticky_colors.max(count(&sticky));
        fresh_colors = fresh_colors.max(count(&colors));
        kept = now.iter().map(|&(p, _)| p).zip(sticky.iter().copied()).collect();
        prev = now;
    }
    println!(
        "\n  over {steps} steps: recolored from scratch, {changed} of {persisted} persisting contacts ({:.2}%) changed color; \
         at most {fresh_colors} colors fresh, {sticky_colors} keeping colors",
        100.0 * changed as f64 / persisted.max(1) as f64
    );
}

fn main() {
    let manifest = engine_control::read_manifest(&std::env::var("PILE").unwrap()).unwrap();
    let pin_env = std::env::var("PIN").unwrap_or_default();
    let pin: Option<fn(usize) -> usize> = match pin_env.as_str() {
        "1" => Some(by_core),
        "ccd" => Some(by_ccd),
        _ => None,
    };
    let quick = std::env::var("QUICK").is_ok_and(|v| v == "1");
    let max = *THREADS.iter().max().unwrap();
    println!(
        "Sequential impulses in parallel, -c opt, {} threads available, {}; µs per solve, {REPS} runs each after 300 ms keeping its threads busy (the median, and 10th to 90th percentiles)",
        std::thread::available_parallelism().map_or(0, |n| n.get()),
        match pin_env.as_str() {
            "1" => "thread i pinned to CPU i (cores first, then their SMT siblings)",
            "ccd" => "pinned to fill one CCD, SMT siblings included, before the other",
            _ => "unpinned",
        },
    );
    assert!(std::is_x86_feature_detected!("avx2"), "the wide kernels are built for AVX2");
    let pool = Pool::new(max, pin);

    overheads(&pool, &THREADS);

    let only = |name: &str| std::env::var("ONLY").map_or(true, |only| name.contains(&only));

    // The pile `:tax` and the docs measure: 331 a row, an odd number, so
    // each column alternates circles and boxes, and without rotation a
    // circle centered on a box stays put: 331 columns that never touch.
    let name = "10 000 pile in a box 400 wide (331 columns), step 400";
    if only(name) {
        let mut a = pile(&manifest, 10000, 400.0, 400);
        scene(name, &capture(&mut a), &pool, &THREADS);
        churn(&a, 60);
    }
    let name = "10 000 pile in a box 400 wide (331 columns), falling (step 30)";
    if only(name) {
        scene(name, &capture(&mut pile(&manifest, 10000, 400.0, 30)), &pool, &THREADS);
    }
    let name = "1000 pile in a box 40 wide (31 columns), step 400";
    if only(name) {
        scene(name, &capture(&mut pile(&manifest, 1000, 40.0, 400)), &pool, &THREADS);
    }
    // An even number a row, so a column is all circles or all boxes, and
    // the circles' columns fall over: a pile.
    for (n, width) in [(10000u32, 401.0f32), (1000, 41.0), (10000, 101.0), (40000, 802.0)] {
        let name = format!("{n} in a box {width} wide (a pile), step 400");
        if only(&name) {
            let mut a = pile(&manifest, n, width, 400);
            scene(&name, &capture(&mut a), &pool, &THREADS);
            if n == 10000 && width == 401.0 {
                churn(&a, 60);
            }
        }
    }
    let name = "1000 stacks of 10 boxes, step 300";
    if only(name) {
        let mut columns = stacks(1000, 10);
        for _ in 0..300 {
            columns.step(&mut Stages::default(), solver::solve);
        }
        scene(name, &capture(&mut columns), &pool, &THREADS);
    }

    if !quick {
        settling("10 000 in a box 401 wide (a pile)", &pile(&manifest, 10000, 401.0, 1), &pool, 4000);
        settling("10 000 in a box 400 wide (columns)", &pile(&manifest, 10000, 400.0, 1), &pool, 4000);
    }
}
