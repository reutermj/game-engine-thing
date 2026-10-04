//! The `threads` mod in a real `Engine`: it installs its pool as the
//! world's executor, swaps it for each new one, and takes out only its own,
//! so an executor someone else installed after it (a test's, a harness's)
//! outlives the pool. Run with `ENGINE_THREADS=3` and `ENGINE_PIN=0`
//! (BUILD.bazel): three threads, so the pool is installed, and placed by
//! the OS, since tests share the machine.

use std::path::PathBuf;
use std::sync::Arc;

use engine_ecs::{Executor, Scoped};
use engine_loader::engine::Engine;
use runfiles::Runfiles;

fn engine(test: &str) -> Box<Engine> {
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").expect("TEST_TMPDIR")).join(test);
    let e = Engine::new(None, dir);
    let rlocation = std::env::var("THREADS").expect("$THREADS is not set");
    let lib = Runfiles::create().expect("runfiles").rlocation(&rlocation).expect("the threads mod in runfiles");
    e.load("threads", &lib).expect("loading the threads mod");
    e
}

fn send(e: &Engine, message: &str) -> String {
    e.send("threads", message).unwrap_or_else(|err| panic!("threads {message}: {err}"))
}

#[test]
fn the_pool_is_installed_at_load_swapped_for_each_new_one_and_taken_out_at_close() {
    let e = engine("own_pool");
    assert_eq!(e.world().executor_threads(), Some(3), "the pool from the environment, installed at load");
    assert!(send(&e, "threads 2").starts_with("2 threads"));
    assert_eq!(e.world().executor_threads(), Some(2), "the new pool in place of the old");
    assert_eq!(send(&e, "threads 1"), "1 thread: no pool");
    assert_eq!(e.world().executor_threads(), None, "one thread: the old pool out and none in");
    send(&e, "threads 4");
    e.shutdown();
    assert_eq!(e.world().executor_threads(), None, "closed: its pool out of the world");
}

#[test]
fn an_executor_installed_after_the_pool_outlives_it() {
    let e = engine("theirs_after");
    let theirs: Arc<dyn Executor> = Arc::new(Scoped(5));
    e.world().set_executor(Some(theirs.clone()));
    // `threads 1` stops the pool and starts none: what's left is what stop
    // left.
    assert_eq!(send(&e, "threads 1"), "1 thread: no pool");
    assert_eq!(e.world().executor_threads(), Some(5), "stop left the executor that isn't its own");

    send(&e, "threads 2");
    assert_eq!(e.world().executor_threads(), Some(2));
    e.world().set_executor(Some(theirs.clone()));
    e.shutdown();
    assert!(e.world().take_executor_if(&theirs), "close left the executor that isn't its own");
    assert_eq!(e.world().executor_threads(), None);
}
