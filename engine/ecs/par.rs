//! Data parallelism inside a system: work split into tasks that threads
//! run, with results that don't depend on how many threads there are or
//! when each finished. The threads belong to whoever made the `Executor`
//! (the host: a resident scheduler, or a benchmark), never to a mod, so no
//! mod code is on any worker's stack once the system returns
//! (docs/architecture/hot-reload.md, "code on the stack can't be swapped").
//!
//! A system reaches the executor through its `Workers` parameter, which
//! declares nothing: tasks only divide what the system's own parameters
//! already hold. Without an executor installed in the world, everything
//! runs on the system's thread, in the same order.

use std::ops::Range;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use crate::query::{Declare, FrameCx, Param, ParamDecl};

/// Runs tasks on threads. `run` returns once every task has, and a task's
/// panic is re-raised by `run`.
pub trait Executor: Send + Sync {
    /// How many threads run tasks, the caller's included.
    fn threads(&self) -> usize;
    /// Runs `f(0)` to `f(tasks - 1)`, each once, in any order, on any
    /// thread.
    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync));
}

/// Threads spawned for each `run` and joined before it returns
/// (`std::thread::scope`): no unsafe code and nothing alive between runs,
/// at the price of spawning every time.
pub struct Scoped(pub usize);

impl Executor for Scoped {
    fn threads(&self) -> usize {
        self.0.max(1)
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        let next = AtomicUsize::new(0);
        let work = || {
            loop {
                let k = next.fetch_add(1, Ordering::Relaxed);
                if k >= tasks {
                    break;
                }
                f(k);
            }
        };
        std::thread::scope(|s| {
            let spawned: Vec<_> = (1..self.threads().min(tasks)).map(|_| s.spawn(work)).collect();
            work();
            // Joined here, not by the scope, so a task's own panic is what
            // the caller sees, not "a scoped thread panicked".
            for thread in spawned {
                if let Err(panic) = thread.join() {
                    std::panic::resume_unwind(panic);
                }
            }
        });
    }
}

/// A system's way to its world's executor, if the world has one: see the
/// module's docs. Declares nothing, as `Dt` doesn't.
#[derive(Clone, Default)]
pub struct Workers(Option<Arc<dyn Executor>>);

impl Workers {
    pub fn new(executor: Option<Arc<dyn Executor>>) -> Workers {
        Workers(executor)
    }

    pub fn threads(&self) -> usize {
        self.0.as_ref().map_or(1, |e| e.threads())
    }

    /// Runs `f(0..tasks)`, on the caller's thread in order when there's
    /// one task or one thread, so work too small to split pays no hand-off.
    pub fn run(&self, tasks: usize, f: impl Fn(usize) + Sync) {
        match &self.0 {
            Some(e) if tasks > 1 && e.threads() > 1 => e.run(tasks, &f),
            _ => (0..tasks).for_each(f),
        }
    }

    /// How many chunks to split `units` of work into, each at least `min`:
    /// a few per thread, so a thread that starts late (or a chunk that's
    /// slow) doesn't hold up the rest.
    pub fn chunks(&self, units: usize, min: usize) -> usize {
        let threads = self.threads();
        if threads == 1 {
            return 1;
        }
        (units / min.max(1)).clamp(1, threads * CHUNKS_PER_THREAD)
    }

    /// `f` over contiguous ranges of `0..n`, of at least `min` each, in
    /// parallel; the results in order of their ranges.
    pub fn map_ranges<R: Send>(&self, n: usize, min: usize, f: impl Fn(Range<usize>) -> R + Sync) -> Vec<R> {
        let ranges = even(n, self.chunks(n, min));
        self.map_each(ranges, |_, r| f(r))
    }

    /// `f` over each item, in parallel, with the item's own `&mut`; the
    /// results in the items' order.
    pub fn map_each<T: Send, R: Send>(&self, items: Vec<T>, f: impl Fn(usize, T) -> R + Sync) -> Vec<R> {
        let cells: Vec<Mutex<(Option<T>, Option<R>)>> = items.into_iter().map(|t| Mutex::new((Some(t), None))).collect();
        self.run(cells.len(), |k| {
            // Each cell is one task's: never contended.
            let item = cells[k].lock().expect("a task's own cell").0.take().expect("run once");
            let out = f(k, item);
            cells[k].lock().expect("a task's own cell").1 = Some(out);
        });
        cells.into_iter().map(|c| c.into_inner().expect("a finished task").1.expect("every task ran")).collect()
    }
}

/// Four chunks a thread: at 10 000 bodies, a chunk of a physics walk is
/// then about 150 rows, microseconds, against a hand-off of about one.
const CHUNKS_PER_THREAD: usize = 4;

/// `0..n` in `chunks` contiguous ranges, as even as can be.
pub fn even(n: usize, chunks: usize) -> Vec<Range<usize>> {
    let chunks = chunks.clamp(1, n.max(1));
    (0..chunks).map(|k| n * k / chunks..n * (k + 1) / chunks).collect()
}

/// Items with weights in at most `chunks` contiguous ranges of about equal
/// weight, none empty: pages of a query by their rows. No items, no ranges.
pub fn balanced(weights: &[usize], chunks: usize) -> Vec<Range<usize>> {
    if weights.is_empty() {
        return Vec::new();
    }
    let total: usize = weights.iter().sum();
    let chunks = chunks.clamp(1, weights.len());
    let mut out = Vec::with_capacity(chunks);
    let (mut start, mut sum) = (0, 0);
    // The last item is always the last range's, so it's never empty.
    for (i, &w) in weights[..weights.len() - 1].iter().enumerate() {
        sum += w;
        // The chunk ends where its share of the total does.
        if sum * chunks >= total * (out.len() + 1) && out.len() + 1 < chunks {
            out.push(start..i + 1);
            start = i + 1;
        }
    }
    out.push(start..weights.len());
    out
}

/// `slice` cut into consecutive pieces of `lens`: outputs carved per chunk,
/// so each task writes its own and nothing is copied together after.
pub fn carve<T>(mut slice: &mut [T], lens: impl IntoIterator<Item = usize>) -> Vec<&mut [T]> {
    let mut out = Vec::new();
    for n in lens {
        let (head, tail) = std::mem::take(&mut slice).split_at_mut(n);
        out.push(head);
        slice = tail;
    }
    out
}

impl Param for Workers {
    type Item<'w> = Workers;

    fn declare(_: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Dt
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> Workers {
        Workers(cx.world.executor())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn balanced_ranges_cover_everything_once_in_order() {
        for weights in [vec![], vec![5], vec![1; 2], vec![1; 17], vec![16, 1, 1, 1, 16, 16, 3, 0, 0, 9], vec![0, 0, 0]] {
            for chunks in 1..8 {
                let r = balanced(&weights, chunks);
                if weights.is_empty() {
                    assert!(r.is_empty());
                    continue;
                }
                assert_eq!(r.first().map(|r| r.start), Some(0));
                assert_eq!(r.last().map(|r| r.end), Some(weights.len()));
                assert!(r.windows(2).all(|w| w[0].end == w[1].start), "{r:?}");
                assert!(r.iter().all(|r| !r.is_empty()), "{weights:?} in {chunks}: {r:?}");
                assert!(r.len() <= chunks);
            }
        }
    }

    #[test]
    fn scoped_runs_every_task_once() {
        for threads in [1, 2, 7] {
            let w = Workers::new(Some(Arc::new(Scoped(threads))));
            let out = w.map_ranges(1000, 10, |r| r.clone().sum::<usize>());
            assert_eq!(out.iter().sum::<usize>(), (0..1000).sum::<usize>());
            let each = w.map_each((0..50).collect(), |k, t: usize| (k, t * 2));
            assert_eq!(each, (0..50).map(|t| (t, t * 2)).collect::<Vec<_>>());
        }
    }

    #[test]
    #[should_panic(expected = "task 3")]
    fn a_tasks_panic_reaches_the_caller() {
        Workers::new(Some(Arc::new(Scoped(4)))).run(8, |k| assert!(k != 3, "task {k}"));
    }
}
