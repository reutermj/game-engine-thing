//! Every run a test binary makes, made once. A harness's quality tests,
//! behaviour tests and baseline (`record.rs`) ask for the same scenes, and
//! whichever asks first runs one while the others wait for it, so the
//! baseline costs the suite nothing it didn't already run. The runs are
//! deterministic, so a run shared is the run each would have made.

use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex, OnceLock};

type Slots<T> = Mutex<HashMap<String, Arc<OnceLock<Arc<T>>>>>;

/// Runs by key: what each harness's `runs.rs` keeps its scenes' results
/// in, a static each.
pub struct Runs<T>(LazyLock<Slots<T>>);

impl<T> Runs<T> {
    pub const fn new() -> Runs<T> {
        Runs(LazyLock::new(Default::default))
    }

    /// `run`'s result under `key`: run by the first to ask, waited for by
    /// the rest. The map's lock is held only to find the slot, so runs of
    /// different keys go on side by side.
    pub fn get(&self, key: String, run: impl FnOnce() -> T) -> Arc<T> {
        let slot = self.0.lock().unwrap().entry(key).or_default().clone();
        slot.get_or_init(|| Arc::new(run())).clone()
    }
}

impl<T> Default for Runs<T> {
    fn default() -> Runs<T> {
        Runs::new()
    }
}

/// Each of `items` through `f`, in threads of their own, the results in
/// their order: a test's time is its slowest scene's. At most `wide` at
/// once, which each harness sets by what a run costs to have open (its
/// `runs::par` says).
pub fn par<A: Sync, T: Send>(items: &[A], wide: usize, f: impl Fn(&A) -> T + Sync) -> Vec<T> {
    let f = &f;
    items
        .chunks(wide)
        .flat_map(|chunk| {
            std::thread::scope(|s| {
                let threads: Vec<_> = chunk.iter().map(|a| s.spawn(move || f(a))).collect();
                threads.into_iter().map(|t| t.join().expect("a run panicked")).collect::<Vec<T>>()
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn a_key_runs_once_however_many_ask_at_once() {
        static RUNS: Runs<usize> = Runs::new();
        let calls = AtomicUsize::new(0);
        let got = par(&[0; 16], 4, |_| {
            *RUNS.get("k".into(), || {
                calls.fetch_add(1, Ordering::SeqCst);
                std::thread::sleep(std::time::Duration::from_millis(20));
                7
            })
        });
        assert_eq!(got, [7; 16]);
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        assert_eq!(*RUNS.get("other".into(), || 8), 8, "another key is another run");
    }

    #[test]
    fn par_keeps_the_order_of_its_items() {
        let items: Vec<usize> = (0..10).collect();
        assert_eq!(par(&items, 3, |&i| i * i), items.iter().map(|i| i * i).collect::<Vec<_>>());
    }
}
