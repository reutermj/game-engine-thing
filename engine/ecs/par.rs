//! The threads parallel work runs on, and the engine's own splits over
//! them. The threads belong to whoever made the `Executor` (the host: the
//! resident `threads` mod, or a benchmark), never to a mod, so no mod code
//! is on any worker's stack once the system returns
//! (docs/architecture/hot-reload.md, "code on the stack can't be swapped").
//!
//! A system reaches them only through a declared shape (`ParMap`,
//! `Reduce`, `Passes`: shape.rs), whose dispatch is `dispatch.rs`'s. The
//! ECS's own work inside a declared node (a `Live`'s broadphase, a
//! re-sort's re-bounding) splits through `Split`, crate-private, on the
//! same dispatch. Without an executor installed in the world, everything
//! runs on the calling thread, in the same order.

use std::ops::Range;
use std::sync::{Arc, Mutex};

use crate::dispatch::{Plan, dispatch};
use crate::world::World;

/// Runs tasks on threads. `run` returns once every task has, and a task's
/// panic is re-raised by `run`. The engine's own callers (`dispatch`) never
/// let a task panic into it: they catch it on the task's thread and raise
/// it again on theirs, since an executor is normally another library's
/// code, with another copy of std, whose `catch_unwind` aborts on a panic
/// from ours (docs/architecture/threads.md, "Panics").
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
        let next = std::sync::atomic::AtomicUsize::new(0);
        let work = || {
            loop {
                let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
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

/// The world's executor where it has more than one thread, for the ECS's
/// own splits and the shapes': what a node's work is cut into chunks for,
/// and runs them on.
#[derive(Clone, Default)]
pub(crate) struct Split(Option<Arc<dyn Executor>>);

impl Split {
    pub(crate) fn of(world: &World) -> Split {
        Split(world.executor().filter(|e| e.threads() > 1))
    }

    pub(crate) fn executor(&self) -> Option<&dyn Executor> {
        self.0.as_deref()
    }

    pub(crate) fn threads(&self) -> usize {
        self.0.as_ref().map_or(1, |e| e.threads())
    }

    /// How many chunks to split `units` of work into, each at least `min`:
    /// a few per thread, so a thread that starts late (or a chunk that's
    /// slow) doesn't hold up the rest.
    pub(crate) fn chunks(&self, units: usize, min: usize) -> usize {
        let threads = self.threads();
        if threads == 1 {
            return 1;
        }
        (units / min.max(1)).clamp(1, threads * CHUNKS_PER_THREAD)
    }

    /// `f` over each item, with the item's own `&mut`, the results in the
    /// items' order: an item a block of one stage (`dispatch`), each thread
    /// taking its own share first. On the caller's thread, in order, when
    /// there's one item or one thread, so work too small to split pays no
    /// hand-off.
    pub(crate) fn map_each<T: Send, R: Send>(&self, items: Vec<T>, f: impl Fn(usize, T) -> R + Sync) -> Vec<R> {
        let exec = match &self.0 {
            Some(e) if items.len() > 1 => &**e,
            _ => return items.into_iter().enumerate().map(|(k, t)| f(k, t)).collect(),
        };
        let cells: Vec<Mutex<(Option<T>, Option<R>)>> = items.into_iter().map(|t| Mutex::new((Some(t), None))).collect();
        one_stage(exec, cells.len(), &|k| {
            // A cell is its block's: a lock found taken is a dispatch bug.
            let mut cell = cells[k].try_lock().expect("a block's taker alone has it");
            let item = cell.0.take().expect("run once");
            cell.1 = Some(f(k, item));
        });
        cells.into_iter().map(|c| c.into_inner().expect("a finished task").1.expect("every task ran")).collect()
    }
}

/// `f(0)` to `f(blocks - 1)` across `exec`'s threads, each once: a plan of
/// one stage, which is how the shapes' and the ECS's one-step splits run.
pub(crate) fn one_stage(exec: &dyn Executor, blocks: usize, f: &(dyn Fn(usize) + Sync)) {
    let mut plan = Plan::default();
    plan.chain();
    let mark = plan.marks(blocks);
    plan.stage(blocks, mark);
    dispatch(exec, &plan, &|_, b| f(b));
}

/// Four chunks a thread: at 10 000 bodies, a chunk of a physics walk is
/// then about 150 rows, microseconds, against a hand-off of about one.
const CHUNKS_PER_THREAD: usize = 4;

/// `0..n` in `chunks` contiguous ranges, as even as can be.
pub(crate) fn even(n: usize, chunks: usize) -> Vec<Range<usize>> {
    let chunks = chunks.clamp(1, n.max(1));
    (0..chunks).map(|k| n * k / chunks..n * (k + 1) / chunks).collect()
}

/// Items with weights in at most `chunks` contiguous ranges of about equal
/// weight, none empty: pages of a query by their rows. No items, no ranges.
pub(crate) fn balanced(weights: &[usize], chunks: usize) -> Vec<Range<usize>> {
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
pub(crate) fn carve<T>(mut slice: &mut [T], lens: impl IntoIterator<Item = usize>) -> Vec<&mut [T]> {
    let mut out = Vec::new();
    for n in lens {
        let (head, tail) = std::mem::take(&mut slice).split_at_mut(n);
        out.push(head);
        slice = tail;
    }
    out
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

    fn split(threads: usize) -> Split {
        Split(Some(Arc::new(Scoped(threads)) as Arc<dyn Executor>).filter(|e| e.threads() > 1))
    }

    #[test]
    fn a_split_maps_every_item_once_in_order() {
        for threads in [1, 2, 7] {
            let s = split(threads);
            let ranges = even(1000, s.chunks(1000, 10));
            assert_eq!(ranges.len(), if threads == 1 { 1 } else { threads * CHUNKS_PER_THREAD });
            let sums = s.map_each(ranges, |_, r| r.sum::<usize>());
            assert_eq!(sums.iter().sum::<usize>(), (0..1000).sum::<usize>());
            let each = s.map_each((0..50).collect(), |k, t: usize| (k, t * 2));
            assert_eq!(each, (0..50).map(|t| (t, t * 2)).collect::<Vec<_>>());
        }
    }

    #[test]
    #[should_panic(expected = "task 3")]
    fn a_tasks_panic_reaches_the_caller() {
        split(4).map_each((0..8).collect(), |k, _: usize| assert!(k != 3, "task {k}"));
    }
}
