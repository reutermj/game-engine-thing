//! Every run of ours a test binary makes, made once: the quality tests,
//! the behaviour tests and the baseline (`record.rs`) ask for the same
//! scenes, and whichever asks first runs one while the others wait, so the
//! baseline costs the suite nothing it didn't already run (2D's
//! `runs.rs`, the same).

use std::sync::{Arc, LazyLock};

use physics_testkit::runs::Runs;

use crate::behave::{self, Behaviour};
use crate::measure::{self, Run};
use crate::scenes::{self, Kind};
use crate::{Config, Iters, Threads, make_backend};

static RUNS: Runs<Run> = Runs::new();
static BEHAVED: Runs<Behaviour> = Runs::new();

/// Each of `items` through `f`, in threads of their own, the results in
/// their order: at most 64 at once, since each of ours is an engine with
/// its mods loaded, and a long bounce grid's thousands at once run out of
/// address space to map them in.
pub fn par<A: Sync, T: Send>(items: &[A], f: impl Fn(&A) -> T + Sync) -> Vec<T> {
    physics_testkit::runs::par(items, 64, f)
}

/// The variant of ours `TUNE` names in the environment
/// (`physics3d::Tuning::parse`), "" for the defaults: how the bounds were
/// checked to fail on the variants they should.
pub fn tune() -> &'static str {
    static TUNE: LazyLock<&'static str> = LazyLock::new(|| std::env::var("TUNE").unwrap_or_default().leak());
    &TUNE
}

fn config(max_bodies: usize, rotate: bool) -> Config {
    Config {
        iters: Iters::Default,
        sleep: false,
        max_bodies: max_bodies as u32,
        rotate,
        tune: tune(),
        gravity: scenes::EARTH,
        substeps: 0,
        threads: Threads::Shared,
    }
}

/// Ours on `kind` at `n`, bodies turning or locked.
pub fn ours(kind: Kind, n: usize, rotate: bool) -> Arc<Run> {
    RUNS.get(format!("{} {n} {rotate} {}", kind.name(), tune()), || {
        let scene = scenes::build(kind, n);
        measure::run(&scene, make_backend("ours", &config(n + 16, rotate)).unwrap().as_mut())
    })
}

/// Ours on a bounce (`Kind::Hit`).
pub fn hit(h: &scenes::Hit) -> Arc<Behaviour> {
    behaved(Kind::Hit, h.pack())
}

/// Ours on a behaviour scene, bodies turning.
pub fn behaved(kind: Kind, n: usize) -> Arc<Behaviour> {
    BEHAVED.get(format!("{} {n} {}", kind.name(), tune()), || {
        let scene = scenes::build(kind, n);
        behave::behave(&scene, make_backend("ours", &Config::of(&scene, true, tune())).unwrap().as_mut())
    })
}
