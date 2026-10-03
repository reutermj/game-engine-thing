//! The task graph a shape's work runs on across the world's executor
//! (docs/architecture/threads.md, "Dispatch"): stages of blocks, each stage
//! waiting for a count of others, in one run of the executor.
//!
//! The dispatch spike's `affine` protocol (dispatch-spike.md), rewritten for
//! the engine: a stage is published by whichever thread completes the last
//! stage it waited for; within a stage a thread takes blocks from its own
//! share first, forward then back, by raising each block's mark
//! (`fetch_max`, Box2D v3's `syncIndex`), and counts what it ran once; idle
//! threads spin. Every thread of the run is a worker, the calling one
//! included, and none is the main one: a thread that comes late skips what
//! is done, and one that never comes holds up nobody, which is all an
//! executor promises (`Executor::run`).
//!
//! All of it is safe Rust: atomics, and the caller's own locks on what a
//! block hands its kernel.

use std::any::Any;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, PoisonError};

use crate::par::Executor;

/// Stages and the order they run in. A stage is `blocks` blocks, claimed
/// through the marks from `mark`; stages over the same data share marks, so
/// a thread finds its blocks in its cache stage after stage. A stage that
/// shares marks with another must come after it in the plan.
#[derive(Debug, Default)]
pub(crate) struct Plan {
    stages: Vec<StagePlan>,
    marks: usize,
    /// The last stage of the chain being added, which the next one waits
    /// for.
    last: Option<usize>,
}

#[derive(Debug)]
struct StagePlan {
    blocks: usize,
    mark: usize,
    /// Stages that wait for this one. One today, a chain; several
    /// programs' chains side by side, and a stage several wait for, are
    /// what system parallelism will add (get-znt.5).
    next: Option<usize>,
    /// How many stages this one waits for.
    waits: usize,
}

impl Plan {
    /// Starts a chain: the next stage waits for nothing, and each after it
    /// for the one before. A program is a chain.
    pub fn chain(&mut self) {
        self.last = None;
    }

    /// `n` marks for blocks of stages to come; the first.
    pub fn marks(&mut self, n: usize) -> usize {
        self.marks += n;
        self.marks - n
    }

    /// A stage of `blocks` blocks, claimed through the marks from `mark`,
    /// after the chain's last; its index.
    pub fn stage(&mut self, blocks: usize, mark: usize) -> usize {
        assert!(blocks > 0, "a stage of no blocks");
        assert!(mark + blocks <= self.marks, "marks for every block");
        let g = self.stages.len();
        self.stages.push(StagePlan { blocks, mark, next: None, waits: usize::from(self.last.is_some()) });
        if let Some(prev) = self.last {
            self.stages[prev].next = Some(g);
        }
        self.last = Some(g);
        g
    }
}

/// One per counter, on its own lines: threads raising neighbouring marks
/// would otherwise take turns on one line.
#[repr(align(128))]
#[derive(Default)]
struct Cell(AtomicUsize);

fn cells(n: usize) -> Vec<Cell> {
    (0..n).map(|_| Cell::default()).collect()
}

const UNPUBLISHED: usize = usize::MAX;

/// A run's shared state.
struct Graph<'p> {
    plan: &'p Plan,
    /// Stages each stage still waits for.
    waits: Vec<Cell>,
    /// Blocks of each stage done.
    done: Vec<Cell>,
    /// One past the last stage that took each block.
    marks: Vec<Cell>,
    /// Stages in the order they were published, and how many.
    ready: Vec<AtomicUsize>,
    published: Cell,
    finished: Cell,
    failed: AtomicBool,
    /// The first panic of a kernel, raised again on the calling thread.
    panic: Mutex<Option<Box<dyn Any + Send>>>,
}

impl Graph<'_> {
    fn publish(&self, g: usize) {
        // The slot is claimed before it's written, so a reader that sees the
        // count may still see `UNPUBLISHED` there, and waits.
        let i = self.published.0.fetch_add(1, Ordering::AcqRel);
        self.ready[i].store(g, Ordering::Release);
    }

    /// `ran` more blocks of stage `g` done, a thread's, counted once it
    /// finds no more: the thread that completes it publishes what waited.
    fn complete(&self, g: usize, ran: usize) {
        let stage = &self.plan.stages[g];
        let now = self.done[g].0.fetch_add(ran, Ordering::AcqRel) + ran;
        debug_assert!(now <= stage.blocks, "a block counted twice");
        if now < stage.blocks {
            return;
        }
        if let Some(s) = stage.next
            && self.waits[s].0.fetch_sub(1, Ordering::AcqRel) == 1
        {
            self.publish(s);
        }
        self.finished.0.fetch_add(1, Ordering::AcqRel);
    }

    /// Takes stage `g`'s blocks for worker `w`: its own share's first,
    /// forward until a block is taken, then back from its first. With every
    /// thread there each takes the same blocks stage after stage; a block
    /// left by a thread that isn't is taken by a neighbour's run, which
    /// stops only at a block someone has.
    fn take(&self, g: usize, w: usize, threads: usize, run: &(dyn Fn(usize, usize) + Sync)) -> usize {
        let stage = &self.plan.stages[g];
        let n = stage.blocks;
        let mine = |k: usize| self.marks[stage.mark + k].0.fetch_max(g + 1, Ordering::AcqRel) <= g;
        let start = first_block(w, n, threads);
        let mut ran = 0;
        let mut k = start;
        while mine(k) {
            run(g, k);
            ran += 1;
            k = if k + 1 == n { 0 } else { k + 1 };
        }
        let mut k = start;
        loop {
            k = if k == 0 { n - 1 } else { k - 1 };
            if !mine(k) {
                break;
            }
            run(g, k);
            ran += 1;
        }
        ran
    }

    /// Worker `w`'s loop: every published stage it hasn't looked at, until
    /// every stage is done.
    fn work(&self, w: usize, threads: usize, run: &(dyn Fn(usize, usize) + Sync)) {
        let total = self.plan.stages.len();
        // Stages before `from` in `ready` this thread has taken what it
        // could of; their other blocks are another thread's run.
        let mut from = 0;
        let mut spins = 0u32;
        loop {
            if self.finished.0.load(Ordering::Acquire) == total || self.failed.load(Ordering::Relaxed) {
                return;
            }
            let published = self.published.0.load(Ordering::Acquire);
            let mut worked = false;
            while from < published {
                let g = self.ready[from].load(Ordering::Acquire);
                if g == UNPUBLISHED {
                    break;
                }
                from += 1;
                let ran = self.take(g, w, threads, run);
                if ran > 0 {
                    self.complete(g, ran);
                    worked = true;
                }
            }
            if worked {
                spins = 0;
            } else {
                spin(&mut spins);
            }
        }
    }
}

/// Spinning, with a yield every 1024 turns: a stage is microseconds, and a
/// wake-up tens of them (dispatch-spike.md, `park`); the yield gives a
/// thread holding a block its core back on a busy machine.
#[inline]
fn spin(spins: &mut u32) {
    std::hint::spin_loop();
    *spins = spins.wrapping_add(1);
    if spins.is_multiple_of(1024) {
        std::thread::yield_now();
    }
}

/// Where worker `w` of `threads` starts in a stage of `n` blocks: its
/// share's first (Box2D's `GetWorkerStartIndex`). With more threads than
/// blocks every thread may take any block (it starts at `w % n`), since
/// none is sure to come
/// (docs/lore/a-stage-loop-without-a-main-thread-must-let-any-thread-take-any-block.md).
pub(crate) fn first_block(w: usize, n: usize, threads: usize) -> usize {
    if n <= threads {
        return w % n;
    }
    let (per, rest) = (n / threads, n % threads);
    per * w + rest.min(w)
}

/// How many blocks `0..n` is cut into, each at least `least`, at most four
/// a thread: Box2D v3's block sizes (`b2SolverStage`), so a thread that
/// starts late has its share taken, and a block is a few microseconds.
pub(crate) fn blocks_of(n: usize, least: usize, threads: usize) -> usize {
    if n > least * 4 * threads { 4 * threads } else { n.div_ceil(least) }
}

/// Block `k` of `0..n` cut into `count`.
pub(crate) fn block_range(n: usize, k: usize, count: usize) -> std::ops::Range<usize> {
    n * k / count..n * (k + 1) / count
}

/// Runs `plan` across `executor`'s threads, `run(stage, block)` for each
/// block of each stage once, every stage after the stages it waits for;
/// returns once every block has run and every thread of the run has left
/// it. A panic in `run` is caught on the thread it happened on, stops the
/// others, and is raised again here: it never unwinds through the
/// executor's code, which is another library's, with another copy of std,
/// whose `catch_unwind` would abort on it (threads.md, "Panics").
pub(crate) fn dispatch(executor: &dyn Executor, plan: &Plan, run: &(dyn Fn(usize, usize) + Sync)) {
    let total = plan.stages.len();
    if total == 0 {
        return;
    }
    let threads = executor.threads().max(1);
    let graph = Graph {
        plan,
        waits: plan.stages.iter().map(|s| Cell(AtomicUsize::new(s.waits))).collect(),
        done: cells(total),
        marks: cells(plan.marks),
        ready: (0..total).map(|_| AtomicUsize::new(UNPUBLISHED)).collect(),
        published: Cell::default(),
        finished: Cell::default(),
        failed: AtomicBool::new(false),
        panic: Mutex::new(None),
    };
    for (g, s) in plan.stages.iter().enumerate() {
        if s.waits == 0 {
            graph.publish(g);
        }
    }
    executor.run(threads, &|w| {
        if let Err(p) = catch_unwind(AssertUnwindSafe(|| graph.work(w, threads, run))) {
            graph.failed.store(true, Ordering::Relaxed);
            graph.panic.lock().unwrap_or_else(PoisonError::into_inner).get_or_insert(p);
        }
    });
    if let Some(p) = graph.panic.into_inner().unwrap_or_else(PoisonError::into_inner) {
        resume_unwind(p);
    }
    debug_assert_eq!(graph.finished.0.load(Ordering::Acquire), total, "every stage done");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::par::Scoped;

    /// Tasks one after another on the calling thread, the last first: a
    /// worker that does everything alone, and others that come once it's
    /// done.
    struct Backwards(usize);

    impl Executor for Backwards {
        fn threads(&self) -> usize {
            self.0
        }

        fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
            (0..tasks).rev().for_each(f);
        }
    }

    /// Threads that start one at a time, each a little after the last.
    struct Late(usize);

    impl Executor for Late {
        fn threads(&self) -> usize {
            self.0
        }

        fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
            std::thread::scope(|s| {
                for k in 1..tasks {
                    s.spawn(move || {
                        std::thread::sleep(std::time::Duration::from_micros(200 * k as u64));
                        f(k)
                    });
                }
                f(0);
            });
        }
    }

    fn executors() -> Vec<Box<dyn Executor>> {
        let mut out: Vec<Box<dyn Executor>> = Vec::new();
        for n in [1, 2, 3, 4, 8] {
            out.push(Box::new(Scoped(n)));
            out.push(Box::new(Backwards(n)));
            out.push(Box::new(Late(n)));
        }
        out
    }

    /// Two chains of stages over shared marks; every block once, each stage
    /// after its chain's last, whoever runs it.
    #[test]
    fn every_block_runs_once_after_the_stage_before() {
        for exec in executors() {
            let mut plan = Plan::default();
            let mut sizes = Vec::new();
            for (stages, blocks) in [(30, 7), (12, 1), (5, 40)] {
                plan.chain();
                let mark = plan.marks(blocks);
                for _ in 0..stages {
                    plan.stage(blocks, mark);
                    sizes.push(blocks);
                }
            }
            let ran: Vec<Vec<AtomicUsize>> = sizes.iter().map(|&n| (0..n).map(|_| AtomicUsize::new(0)).collect()).collect();
            // The chain each stage is in, and the stage before it in it.
            let before = |g: usize| (g > 0 && g != 30 && g != 42).then(|| g - 1);
            let early = AtomicBool::new(false);
            dispatch(&*exec, &plan, &|g, k| {
                if let Some(b) = before(g)
                    && ran[b].iter().any(|r| r.load(Ordering::Acquire) == 0)
                {
                    early.store(true, Ordering::Relaxed);
                }
                // A block takes a while, so a stage let start before the
                // last one's last block is done overlaps it, and shows.
                let t = std::time::Instant::now();
                while t.elapsed() < std::time::Duration::from_micros(5) {
                    std::hint::spin_loop();
                }
                ran[g][k].fetch_add(1, Ordering::AcqRel);
            });
            assert!(!early.load(Ordering::Relaxed), "a stage started before the one it waits for was done ({} threads)", exec.threads());
            for (g, stage) in ran.iter().enumerate() {
                for (k, r) in stage.iter().enumerate() {
                    assert_eq!(r.load(Ordering::Relaxed), 1, "stage {g} block {k} ({} threads)", exec.threads());
                }
            }
        }
    }

    #[test]
    fn a_panic_in_a_block_reaches_the_caller_and_stops_the_rest() {
        for exec in executors() {
            let mut plan = Plan::default();
            plan.chain();
            let mark = plan.marks(16);
            for _ in 0..50 {
                plan.stage(16, mark);
            }
            let ran = AtomicUsize::new(0);
            let caught = catch_unwind(AssertUnwindSafe(|| {
                dispatch(&*exec, &plan, &|g, k| {
                    ran.fetch_add(1, Ordering::Relaxed);
                    assert!(!(g == 3 && k == 5), "block {k} of stage {g}");
                })
            }));
            let message = caught.expect_err("the panic").downcast::<String>().expect("its message");
            assert_eq!(*message, "block 5 of stage 3");
            assert!(ran.load(Ordering::Relaxed) < 50 * 16, "the stages after it don't run");
        }
    }

    #[test]
    fn blocks_are_four_a_thread_at_most_and_cover_the_range() {
        for (n, least, threads, count) in [(0, 4, 8, 0), (3, 4, 8, 1), (17, 4, 2, 5), (1000, 4, 8, 32), (1000, 32, 1, 4), (100, 32, 8, 4)] {
            assert_eq!(blocks_of(n, least, threads), count, "{n} over {threads}");
            let ranges: Vec<_> = (0..count).map(|k| block_range(n, k, count)).collect();
            assert!(ranges.windows(2).all(|w| w[0].end == w[1].start));
            assert_eq!(ranges.first().map_or(0, |r| r.start), 0);
            assert_eq!(ranges.last().map_or(0, |r| r.end), n);
        }
    }
}
