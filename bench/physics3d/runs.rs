//! Every run of ours a test binary makes, made once: the quality tests,
//! the behaviour tests and the baseline (`record.rs`) ask for the same
//! scenes, and whichever asks first runs one while the others wait, so the
//! baseline costs the suite nothing it didn't already run (2D's
//! `runs.rs`, the same).

use std::collections::HashMap;
use std::sync::{Arc, LazyLock, Mutex, OnceLock};

use crate::behave::{self, Behaviour};
use crate::measure::{self, Run};
use crate::scenes::{self, Kind};
use crate::{Config, Iters, make_backend};

type Slots<T> = LazyLock<Mutex<HashMap<String, Arc<OnceLock<Arc<T>>>>>>;

static RUNS: Slots<Run> = LazyLock::new(Default::default);
static BEHAVED: Slots<Behaviour> = LazyLock::new(Default::default);

fn once<T>(slots: &Slots<T>, key: String, run: impl FnOnce() -> T) -> Arc<T> {
    let slot = slots.lock().unwrap().entry(key).or_default().clone();
    slot.get_or_init(|| Arc::new(run())).clone()
}

/// Each of `items` through `f`, in threads of their own, the results in
/// their order.
pub fn par<A: Sync, T: Send>(items: &[A], f: impl Fn(&A) -> T + Sync) -> Vec<T> {
    std::thread::scope(|s| {
        let threads: Vec<_> = items.iter().map(|a| s.spawn(|| f(a))).collect();
        threads.into_iter().map(|t| t.join().expect("a run panicked")).collect()
    })
}

/// The variant of ours `TUNE` names in the environment
/// (`physics3d::Tuning::parse`), "" for the defaults: how the bounds were
/// checked to fail on the variants they should.
pub fn tune() -> &'static str {
    static TUNE: LazyLock<&'static str> = LazyLock::new(|| std::env::var("TUNE").unwrap_or_default().leak());
    &TUNE
}

fn config(max_bodies: usize, rotate: bool) -> Config {
    Config { iters: Iters::Default, sleep: false, max_bodies: max_bodies as u32, rotate, tune: tune() }
}

/// Ours on `kind` at `n`, bodies turning or locked.
pub fn ours(kind: Kind, n: usize, rotate: bool) -> Arc<Run> {
    once(&RUNS, format!("{} {n} {rotate} {}", kind.name(), tune()), || {
        let scene = scenes::build(kind, n);
        measure::run(&scene, make_backend("ours", &config(n + 16, rotate)).unwrap().as_mut())
    })
}

/// Ours on a behaviour scene, bodies turning.
pub fn behaved(kind: Kind, n: usize) -> Arc<Behaviour> {
    once(&BEHAVED, format!("{} {n} {}", kind.name(), tune()), || {
        behave::behave(&scenes::build(kind, n), make_backend("ours", &config(16, true)).unwrap().as_mut())
    })
}
