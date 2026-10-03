//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): ways a
//! scheduler could hand a `Passes` program's stages out across threads,
//! each running the program exactly as `Passes::run` defines it
//! (engine_ecs::shape), with no change to the kernels. Throwaway: the
//! numbers are in the doc.
//!
//! - [`Protocol::Counter`]: `lanes::run_across`'s protocol (physics2d's
//!   solver.rs), generalised to any program: every thread walks every
//!   stage in order, takes blocks by raising their marks (`fetch_max`),
//!   counts them done, spins until the stage's count is full.
//! - [`Protocol::Graph`]: a task graph. Each stage has a count of the
//!   stages it waits for; the thread that finishes a stage's last block
//!   lowers its successors' counts and publishes those that reach zero on
//!   a ready list. Threads take blocks of any ready stage by `fetch_add`.
//!   Several programs are one graph, so a thread idle in one program's
//!   thin stage takes another's blocks.
//! - [`Protocol::Affine`]: the graph, its blocks taken as `Counter` takes
//!   them, from the thread's own share forward and back, so a thread finds
//!   the same blocks stage after stage.
//! - [`Protocol::Steal`]: a work-stealing pool's shape (rayon's, Jolt's,
//!   with locks where they have lock-free deques): a ready stage is one
//!   range task on the releasing thread's deque, split in halves as it's
//!   taken; idle threads steal the oldest half of another's.
//! - [`Protocol::Park`]: the graph with idle threads parked on a condvar
//!   and woken at each publish, as a pool that doesn't spin would be.
//!
//! The calling thread is one of the workers in all of them: the executor
//! runs task 0 on it.

use std::collections::VecDeque;
use std::ops::Range;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::Instant;

use engine_ecs::Executor;
use engine_ecs::shape::{Colored, Shareable, Stage, States};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    Counter,
    Graph,
    Affine,
    Steal,
    Park,
}

impl Protocol {
    pub const ALL: [Protocol; 5] = [Protocol::Counter, Protocol::Graph, Protocol::Affine, Protocol::Steal, Protocol::Park];

    pub fn name(self) -> &'static str {
        match self {
            Protocol::Counter => "counter",
            Protocol::Graph => "graph",
            Protocol::Affine => "affine",
            Protocol::Steal => "steal",
            Protocol::Park => "park",
        }
    }

    pub fn parse(s: &str) -> Protocol {
        *Protocol::ALL.iter().find(|p| p.name() == s).unwrap_or_else(|| panic!("{s}: counter, graph, affine, steal or park"))
    }
}

/// A bug planted in every protocol, for the test that the bit-for-bit
/// check sees one: a block run twice (the first relaxing stage's first),
/// or a stage let start before the last one's blocks are done (released
/// at one block short).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Plant {
    None,
    Twice,
    Early,
}

/// A program as the dispatchers see it: each stage's blocks and its first
/// mark (stages over the same blocks, a color's batches, share marks, as
/// `run_across`'s blocks are shared by the passes over them); and how to
/// run block `b` of stage `t`.
pub struct Program<'a> {
    pub stages: Vec<(usize, usize)>,
    pub marks: usize,
    pub run: &'a (dyn Fn(usize, usize) + Sync),
}

/// One per stage or mark, on its own lines.
#[repr(align(128))]
#[derive(Default)]
struct Cell(AtomicUsize);

fn cells(n: usize) -> Vec<Cell> {
    (0..n).map(|_| Cell::default()).collect()
}

/// The programs as one list of stages: each stage's program and index in
/// it, blocks, first mark (offset per program), and the stage after it.
struct Flat<'p, 'a> {
    programs: &'p [Program<'a>],
    at: Vec<(usize, usize)>,
    blocks: Vec<usize>,
    mark: Vec<usize>,
    next: Vec<Option<usize>>,
    first: Vec<usize>,
    marks: usize,
}

impl<'p, 'a> Flat<'p, 'a> {
    fn new(programs: &'p [Program<'a>]) -> Self {
        let mut f = Flat { programs, at: vec![], blocks: vec![], mark: vec![], next: vec![], first: vec![], marks: 0 };
        for (p, prog) in programs.iter().enumerate() {
            if prog.stages.is_empty() {
                continue;
            }
            f.first.push(f.at.len());
            for (t, &(n, m)) in prog.stages.iter().enumerate() {
                assert!(n > 0, "a stage of no blocks");
                let g = f.at.len();
                f.at.push((p, t));
                f.blocks.push(n);
                f.mark.push(f.marks + m);
                f.next.push((t + 1 < prog.stages.len()).then_some(g + 1));
            }
            f.marks += prog.marks;
        }
        f
    }

    fn len(&self) -> usize {
        self.at.len()
    }

    #[inline]
    fn call(&self, g: usize, b: usize) {
        let (p, t) = self.at[g];
        (self.programs[p].run)(t, b)
    }
}

/// Where each thread's blocks went in time, for the idle analysis.
pub struct Trace {
    origin: Instant,
    pub lanes: Mutex<Vec<Vec<Span>>>,
}

#[derive(Clone, Copy, Debug)]
pub struct Span {
    pub stage: usize,
    pub t0: u64,
    pub t1: u64,
}

impl Trace {
    pub fn new() -> Trace {
        Trace { origin: Instant::now(), lanes: Mutex::new(Vec::new()) }
    }
}

/// A worker's spans, handed to the trace when it leaves.
struct Lane<'t> {
    trace: Option<&'t Trace>,
    spans: Vec<Span>,
}

impl Lane<'_> {
    #[inline]
    fn run(&mut self, g: usize, f: impl FnOnce()) {
        match self.trace {
            None => f(),
            Some(t) => {
                let t0 = t.origin.elapsed().as_nanos() as u64;
                f();
                let t1 = t.origin.elapsed().as_nanos() as u64;
                self.spans.push(Span { stage: g, t0, t1 });
            }
        }
    }
}

impl Drop for Lane<'_> {
    fn drop(&mut self) {
        if let Some(t) = self.trace {
            t.lanes.lock().unwrap().push(std::mem::take(&mut self.spans));
        }
    }
}

/// Set when a worker panics, so the others stop waiting for blocks it
/// won't finish (`run_across`'s `Failing`).
struct Failing<'a>(&'a AtomicBool);

impl Drop for Failing<'_> {
    fn drop(&mut self) {
        if std::thread::panicking() {
            self.0.store(true, Ordering::Relaxed);
        }
    }
}

/// Spinning, a yield every 1024 turns: `run_across`'s wait.
#[inline]
fn spin(spins: &mut u32) {
    std::hint::spin_loop();
    *spins = spins.wrapping_add(1);
    if spins.is_multiple_of(1024) {
        std::thread::yield_now();
    }
}

/// Where thread `w` of `threads` starts in a stage of `n` blocks:
/// `run_across`'s `first_block` (Box2D's `GetWorkerStartIndex`).
fn first_block(w: usize, n: usize, threads: usize) -> usize {
    if n <= threads {
        return w % n;
    }
    let (per, rest) = (n / threads, n % threads);
    per * w + rest.min(w)
}

/// `0..n` in blocks of at least `least`, at most four a thread:
/// `run_across`'s `blocks_of` (Box2D's sizes).
pub fn blocks_of(n: usize, least: usize, threads: usize) -> impl Iterator<Item = Range<usize>> {
    let count = if n > least * 4 * threads { 4 * threads } else { n.div_ceil(least) };
    (0..count).map(move |k| n * k / count..n * (k + 1) / count)
}

/// Runs `programs` across `exec`'s threads by `protocol`: each program's
/// stages in order, each stage's blocks once, after the stage before.
pub fn run(protocol: Protocol, exec: &dyn Executor, programs: &[Program<'_>], trace: Option<&Trace>, plant: Plant) {
    let f = Flat::new(programs);
    if f.len() == 0 {
        return;
    }
    match protocol {
        Protocol::Counter => counter(exec, &f, trace, plant),
        _ => graph(protocol, exec, &f, trace, plant),
    }
}

/// The stage `Plant::Twice` runs a block of twice: the second stage of
/// the first program, whatever it is.
const TWICE_AT: usize = 1;

fn counter(exec: &dyn Executor, f: &Flat, trace: Option<&Trace>, plant: Plant) {
    let threads = exec.threads();
    let marks = cells(f.marks);
    let done = cells(f.len());
    let failed = AtomicBool::new(false);
    exec.run(threads, &|w| {
        let _failing = Failing(&failed);
        let mut lane = Lane { trace, spans: Vec::new() };
        for g in 0..f.len() {
            let (n, count) = (f.blocks[g], &done[g].0);
            let need = if plant == Plant::Early && n > 1 { n - 1 } else { n };
            if count.load(Ordering::Acquire) >= need {
                continue;
            }
            let mut take = |k: usize| {
                if marks[f.mark[g] + k].0.fetch_max(g + 1, Ordering::AcqRel) > g {
                    return false;
                }
                lane.run(g, || f.call(g, k));
                if plant == Plant::Twice && g == TWICE_AT && k == 0 {
                    f.call(g, k);
                }
                true
            };
            let (start, mut ran) = (first_block(w, n, threads), 0);
            let mut k = start;
            while take(k) {
                ran += 1;
                k = if k + 1 == n { 0 } else { k + 1 };
            }
            let mut k = start;
            loop {
                k = if k == 0 { n - 1 } else { k - 1 };
                if !take(k) {
                    break;
                }
                ran += 1;
            }
            if ran > 0 {
                count.fetch_add(ran, Ordering::Release);
            }
            let mut spins = 0u32;
            while count.load(Ordering::Acquire) < need {
                if failed.load(Ordering::Relaxed) {
                    return;
                }
                spin(&mut spins);
            }
        }
    });
}

/// The graph's shared state.
struct Graph {
    /// Stages each stage still waits for.
    pending: Vec<Cell>,
    /// The next block to hand out (`Graph`, `Park`).
    next: Vec<Cell>,
    done: Vec<Cell>,
    marks: Vec<Cell>,
    /// Stages published, in order, and how many.
    ready: Vec<AtomicUsize>,
    published: Cell,
    finished: Cell,
    failed: AtomicBool,
    /// `Park`'s: a count of publishes, the lock and the condvar.
    wake: (Mutex<()>, Condvar),
    /// `Steal`'s deques, a worker's each.
    deques: Vec<Mutex<VecDeque<(usize, usize, usize)>>>,
}

const UNPUBLISHED: usize = usize::MAX;

impl Graph {
    fn publish(&self, protocol: Protocol, g: usize, n: usize, w: usize) {
        if protocol == Protocol::Steal {
            self.deques[w % self.deques.len()].lock().unwrap().push_back((g, 0, n));
        }
        let i = self.published.0.fetch_add(1, Ordering::AcqRel);
        self.ready[i].store(g, Ordering::Release);
        if protocol == Protocol::Park {
            drop(self.wake.0.lock().unwrap());
            self.wake.1.notify_all();
        }
    }

    /// `ran` more blocks of stage `g` done (a thread's, counted once it
    /// finds no more, as `run_across` counts them): the last publishes
    /// what waited.
    fn complete(&self, protocol: Protocol, f: &Flat, (g, ran): (usize, usize), w: usize, plant: Plant) {
        if ran == 0 {
            return;
        }
        let n = f.blocks[g];
        let release = if plant == Plant::Early { n.saturating_sub(1).max(1) } else { n };
        let now = self.done[g].0.fetch_add(ran, Ordering::AcqRel) + ran;
        if now - ran < release && now >= release {
            let ready = f.next[g].filter(|&s| self.pending[s].0.fetch_sub(1, Ordering::AcqRel) == 1);
            if let Some(s) = ready {
                self.publish(protocol, s, f.blocks[s], w);
            }
        }
        if now == n {
            self.finished.0.fetch_add(1, Ordering::AcqRel);
            if protocol == Protocol::Park {
                drop(self.wake.0.lock().unwrap());
                self.wake.1.notify_all();
            }
        }
    }
}

fn graph(protocol: Protocol, exec: &dyn Executor, f: &Flat, trace: Option<&Trace>, plant: Plant) {
    let threads = exec.threads();
    let total = f.len();
    let gr = Graph {
        pending: (0..total).map(|_| Cell(AtomicUsize::new(1))).collect(),
        next: cells(total),
        done: cells(total),
        marks: cells(f.marks),
        ready: (0..total).map(|_| AtomicUsize::new(UNPUBLISHED)).collect(),
        published: Cell::default(),
        finished: Cell::default(),
        failed: AtomicBool::new(false),
        wake: (Mutex::new(()), Condvar::new()),
        deques: (0..threads).map(|_| Mutex::new(VecDeque::new())).collect(),
    };
    for &g in &f.first {
        gr.pending[g].0.store(0, Ordering::Relaxed);
        gr.publish(protocol, g, f.blocks[g], 0);
    }
    exec.run(threads, &|w| {
        let _failing = Failing(&gr.failed);
        let mut lane = Lane { trace, spans: Vec::new() };
        let run = |g: usize, k: usize, lane: &mut Lane| {
            lane.run(g, || f.call(g, k));
            if plant == Plant::Twice && g == TWICE_AT && k == 0 {
                f.call(g, k);
            }
        };
        // The first published stage this thread hasn't found empty.
        let mut from = 0;
        let mut spins = 0u32;
        loop {
            if gr.finished.0.load(Ordering::Acquire) == total || gr.failed.load(Ordering::Relaxed) {
                return;
            }
            // What a parked thread waits to see change: read before looking,
            // so a publish while it looks wakes it.
            let seen = (gr.published.0.load(Ordering::Acquire), gr.finished.0.load(Ordering::Acquire));
            let mut worked = false;
            if protocol == Protocol::Steal {
                let mine = gr.deques[w].lock().unwrap().pop_back();
                let task = mine.or_else(|| (1..threads).find_map(|v| gr.deques[(w + v) % threads].lock().unwrap().pop_front()));
                if let Some((g, lo, mut hi)) = task {
                    // Halves to the deque, the rest kept, until one block's
                    // left: rayon's split of a range.
                    while hi - lo > 1 {
                        let mid = lo + (hi - lo) / 2;
                        gr.deques[w].lock().unwrap().push_back((g, mid, hi));
                        hi = mid;
                    }
                    run(g, lo, &mut lane);
                    gr.complete(protocol, f, (g, 1), w, plant);
                    worked = true;
                }
            } else {
                let len = gr.published.0.load(Ordering::Acquire);
                let mut i = from;
                while i < len && !worked {
                    let g = gr.ready[i].load(Ordering::Acquire);
                    if g == UNPUBLISHED {
                        break;
                    }
                    let n = f.blocks[g];
                    let mut ran = 0;
                    if protocol == Protocol::Affine {
                        let take = |k: usize, lane: &mut Lane| {
                            if gr.marks[f.mark[g] + k].0.fetch_max(g + 1, Ordering::AcqRel) > g {
                                return false;
                            }
                            run(g, k, lane);
                            true
                        };
                        let start = first_block(w, n, threads);
                        let mut k = start;
                        while take(k, &mut lane) {
                            ran += 1;
                            k = if k + 1 == n { 0 } else { k + 1 };
                        }
                        let mut k = start;
                        loop {
                            k = if k == 0 { n - 1 } else { k - 1 };
                            if !take(k, &mut lane) {
                                break;
                            }
                            ran += 1;
                        }
                    } else {
                        while gr.next[g].0.load(Ordering::Relaxed) < n {
                            let k = gr.next[g].0.fetch_add(1, Ordering::AcqRel);
                            if k >= n {
                                break;
                            }
                            run(g, k, &mut lane);
                            ran += 1;
                        }
                    }
                    gr.complete(protocol, f, (g, ran), w, plant);
                    worked = ran > 0;
                    // Nothing more of `g` for this thread.
                    if i == from {
                        from += 1;
                    }
                    i += 1;
                }
            }
            if worked {
                spins = 0;
                continue;
            }
            if protocol == Protocol::Park {
                let mut guard = gr.wake.0.lock().unwrap();
                // Checked under the lock the publisher takes before it
                // notifies: no lost wake-up.
                while (gr.published.0.load(Ordering::Acquire), gr.finished.0.load(Ordering::Acquire)) == seen
                    && seen.1 < total
                    && !gr.failed.load(Ordering::Relaxed)
                {
                    guard = gr.wake.1.wait(guard).unwrap();
                }
            } else {
                spin(&mut spins);
            }
        }
    });
}

/// How a `Passes` program's stage maps to the dispatchers': the overflow's
/// block and each color's blocks a stage each (an `Items` stage), or a
/// stage of ranges (an `Each`).
#[derive(Clone, Copy)]
enum Kind<K> {
    /// The kernel's argument, and the stage's first block.
    Items(K, usize),
    /// The kernel's argument, `n`, and the stage's blocks.
    Each(K, usize, usize),
}

/// A `Passes` program laid out for the dispatchers, its states shared:
/// what `Passes::run` would make before its first stage.
pub struct Prepared<'a, I, T: Shareable, K> {
    shared: Vec<T::Shared>,
    blocks: Vec<Mutex<&'a mut [I]>>,
    kinds: Vec<Kind<K>>,
    stages: Vec<(usize, usize)>,
    marks: usize,
}

impl<'a, I: Send, T: Shareable, K: Copy + Sync> Prepared<'a, I, T, K> {
    /// Blocks as `run_across` sizes them for `threads`: the overflow one,
    /// a color's batches at least 4 a block, states at least 32.
    pub fn new(layout: &Colored, items: &'a mut [I], states: &[T], program: &[Stage<K>], threads: usize) -> Self {
        assert_eq!(items.len(), layout.items(), "items as the coloring laid them out");
        let mut blocks = Vec::new();
        // Each group's (first block, count): the overflow's, then each
        // non-empty color's.
        let mut groups = Vec::new();
        let mut rest = items;
        for (i, n) in std::iter::once(layout.overflow).chain(layout.colors.iter().copied()).enumerate() {
            let (mut group, tail) = std::mem::take(&mut rest).split_at_mut(n);
            rest = tail;
            if n == 0 {
                continue;
            }
            let first = blocks.len();
            if i == 0 {
                blocks.push(Mutex::new(group));
            } else {
                for r in blocks_of(n, 4, threads) {
                    let (head, tail) = group.split_at_mut(r.len());
                    group = tail;
                    blocks.push(Mutex::new(head));
                }
            }
            groups.push((first, blocks.len() - first));
        }
        let mut marks = blocks.len();
        let mut each_marks: Vec<(usize, usize)> = Vec::new();
        let (mut kinds, mut stages) = (Vec::new(), Vec::new());
        for stage in program {
            match *stage {
                Stage::Items(k) => {
                    for &(first, count) in &groups {
                        kinds.push(Kind::Items(k, first));
                        stages.push((count, first));
                    }
                }
                Stage::Each(k, n) if n > 0 => {
                    let count = blocks_of(n, 32, threads).count();
                    let at = match each_marks.iter().find(|(m, _)| *m == n) {
                        Some(&(_, at)) => at,
                        None => {
                            each_marks.push((n, marks));
                            marks += count;
                            marks - count
                        }
                    };
                    kinds.push(Kind::Each(k, n, count));
                    stages.push((count, at));
                }
                Stage::Each(..) => {}
            }
        }
        Prepared { shared: states.iter().map(T::share).collect(), blocks, kinds, stages, marks }
    }

    /// Runs block `b` of stage `t`.
    #[inline]
    pub fn exec(&self, t: usize, b: usize, block: &impl Fn(K, &mut [I], States<'_, T>), each: &impl Fn(K, Range<usize>, States<'_, T>)) {
        match self.kinds[t] {
            Kind::Items(k, first) => {
                let mut items = self.blocks[first + b].try_lock().expect("a block's taker alone has it");
                block(k, &mut items, States::Shared(&self.shared));
            }
            Kind::Each(k, n, count) => each(k, n * b / count..n * (b + 1) / count, States::Shared(&self.shared)),
        }
    }

    pub fn program<'r>(&self, run: &'r (dyn Fn(usize, usize) + Sync)) -> Program<'r> {
        Program { stages: self.stages.clone(), marks: self.marks, run }
    }

    /// The states written back, as `Passes::run` writes them after its
    /// last stage.
    pub fn finish(self, states: &mut [T]) {
        for (s, x) in states.iter_mut().zip(&self.shared) {
            *s = T::load(x);
        }
    }
}

/// `Passes::run` across threads by `protocol`. Where `shareable` is false
/// (get-znt.39), on one thread, plain.
#[allow(clippy::too_many_arguments)]
pub fn passes<I: Send, T: Shareable, K: Copy + Sync>(
    (protocol, exec, shareable, plant): (Protocol, &dyn Executor, bool, Plant),
    layout: &Colored,
    items: &mut [I],
    states: &mut [T],
    program: &[Stage<K>],
    block: impl Fn(K, &mut [I], States<'_, T>) + Sync,
    each: impl Fn(K, Range<usize>, States<'_, T>) + Sync,
    trace: Option<&Trace>,
) {
    if !shareable {
        return one_thread(layout, items, states, program, &block, &each);
    }
    let p = Prepared::new(layout, items, states, program, exec.threads());
    let run = |t: usize, b: usize| p.exec(t, b, &block, &each);
    self::run(protocol, exec, &[p.program(&run)], trace, plant);
    p.finish(states);
}

/// `Passes::run`'s stages on one thread, plain: a copy of engine_ecs's
/// `stages` (private there), the result every dispatcher is held to.
pub fn one_thread<I, T: Shareable, K: Copy>(
    layout: &Colored,
    items: &mut [I],
    states: &mut [T],
    program: &[Stage<K>],
    block: &impl Fn(K, &mut [I], States<'_, T>),
    each: &impl Fn(K, Range<usize>, States<'_, T>),
) {
    for stage in program {
        match *stage {
            Stage::Items(k) => {
                let mut rest = &mut items[..];
                for n in std::iter::once(layout.overflow).chain(layout.colors.iter().copied()) {
                    let (color, tail) = std::mem::take(&mut rest).split_at_mut(n);
                    rest = tail;
                    if !color.is_empty() {
                        block(k, color, States::Plain(states));
                    }
                }
            }
            Stage::Each(k, n) if n > 0 => each(k, 0..n, States::Plain(states)),
            Stage::Each(..) => {}
        }
    }
}

/// Where the threads' time went in a traced run of one program: µs of
/// thread time. `tail` is each stage's, from a thread's last block in it
/// (or the stage's start, for a thread with none) to the stage's last
/// block's end; `thin` the part of it in stages of fewer blocks than
/// threads; `handoff` from a stage's end to the next one's first block,
/// every thread; `busy` in blocks.
#[derive(Clone, Copy, Debug, Default)]
pub struct Idle {
    pub wall: f64,
    pub busy: f64,
    pub tail: f64,
    pub thin: f64,
    pub handoff: f64,
    /// The median handoff, µs: a stage's latency from its last block to
    /// the next stage's first.
    pub handoff_median: f64,
}

pub fn idle(trace: &Trace, blocks: &[usize], threads: usize) -> Idle {
    let lanes = trace.lanes.lock().unwrap();
    let stages = blocks.len();
    let (mut start, mut end) = (vec![u64::MAX; stages], vec![0u64; stages]);
    let mut busy = 0u64;
    // Each lane's last end in each stage.
    let mut last: Vec<Vec<Option<u64>>> = Vec::new();
    for lane in lanes.iter() {
        let mut mine = vec![None; stages];
        for s in lane {
            start[s.stage] = start[s.stage].min(s.t0);
            end[s.stage] = end[s.stage].max(s.t1);
            busy += s.t1 - s.t0;
            mine[s.stage] = Some(mine[s.stage].map_or(s.t1, |x: u64| x.max(s.t1)));
        }
        last.push(mine);
    }
    let (mut tail, mut thin) = (0u64, 0u64);
    for g in 0..stages {
        if start[g] == u64::MAX {
            continue;
        }
        let mut t = 0;
        for k in 0..threads {
            t += match last.get(k).and_then(|l| l[g]) {
                Some(x) => end[g] - x,
                None => end[g] - start[g],
            };
        }
        tail += t;
        if blocks[g] < threads {
            thin += t;
        }
    }
    let mut gaps: Vec<u64> = (1..stages).map(|g| start[g].saturating_sub(end[g - 1])).collect();
    let handoff: u64 = gaps.iter().sum::<u64>() * threads as u64;
    gaps.sort_unstable();
    let wall = end.iter().max().unwrap_or(&0) - start.iter().min().unwrap_or(&0);
    let us = |x: u64| x as f64 / 1e3;
    Idle {
        wall: us(wall),
        busy: us(busy),
        tail: us(tail),
        thin: us(thin),
        handoff: us(handoff),
        handoff_median: gaps.get(gaps.len() / 2).map_or(0.0, |x| us(*x)),
    }
}

/// A program of `stages` stages of `blocks` blocks each, every block
/// `work_ns` of spinning: the dispatchers' own cost, with no kernel in it.
/// µs a run.
pub fn synthetic(protocol: Protocol, exec: &dyn Executor, (stages, blocks, work_ns): (usize, usize, u64)) -> f64 {
    let run = |_: usize, _: usize| {
        if work_ns > 0 {
            let t = Instant::now();
            while (t.elapsed().as_nanos() as u64) < work_ns {
                std::hint::spin_loop();
            }
        }
    };
    let program = Program { stages: (0..stages).map(|_| (blocks, 0)).collect(), marks: blocks, run: &run };
    let t = Instant::now();
    self::run(protocol, exec, &[program], None, Plant::None);
    t.elapsed().as_secs_f64() * 1e6
}

// SPIKE, unsafe: `sched_setaffinity` on the calling thread, which libc
// declares and std doesn't wrap. A real pool would take this from a crate
// (core_affinity) or keep this one FFI call: get-znt.20's decision.
unsafe extern "C" {
    fn sched_setaffinity(pid: i32, size: usize, mask: *const u64) -> i32;
}

fn pin_me(cpu: usize) -> bool {
    let mut mask = [0u64; 16];
    mask[cpu / 64] |= 1 << (cpu % 64);
    // SAFETY: the mask is a live array of the size passed, and pid 0 is the
    // calling thread.
    unsafe { sched_setaffinity(0, std::mem::size_of_val(&mask), mask.as_ptr()) == 0 }
}

/// Pins each of `exec`'s threads to its own CPU of `cpus`, the calling
/// thread's included, in the order they first take a task: runs of tasks
/// that wait a little, so every thread takes one, until all have.
pub fn pin(exec: &dyn Executor, cpus: &[usize]) {
    // Which call last pinned this thread: the calling thread is every
    // pool's, and is pinned again with each.
    thread_local! {
        static PINNED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    }
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    let call = CALLS.fetch_add(1, Ordering::Relaxed) + 1;
    let threads = exec.threads();
    assert!(cpus.len() >= threads, "a CPU a thread");
    let next = AtomicUsize::new(0);
    for _ in 0..1000 {
        exec.run(threads * 4, &|_| {
            if PINNED.get() != call {
                let k = next.fetch_add(1, Ordering::AcqRel);
                if k < threads {
                    assert!(pin_me(cpus[k]), "sched_setaffinity");
                    PINNED.set(call);
                }
            }
            let t = Instant::now();
            while t.elapsed().as_micros() < 200 {
                std::hint::spin_loop();
            }
        });
        if next.load(Ordering::Acquire) >= threads {
            return;
        }
    }
    panic!("not every thread took a task to pin it");
}
