//! The fuzzers' corpus, replayed through the same drivers: natively in the
//! default suite, so an input that once found a bug keeps checking it, and
//! under Miri (`//engine/ecs/fuzz:miri_replay_*`), which checks what the
//! fuzzers can't see without a sanitizer: aliasing, uninitialized reads,
//! use after free.

use std::path::Path;

/// Relative to the runfiles root, where the tests run: the corpus is their
/// data.
const CORPUS: &str = "engine/ecs/fuzz/corpus";

/// This shard's index and the shard count, from Bazel's `shard_count`:
/// under Miri, which interprets one input after another on one thread, the
/// corpus would take hours unsharded. libtest doesn't shard, so this does,
/// by input.
fn shard() -> (usize, usize) {
    let var = |name| std::env::var(name).ok().and_then(|v| v.parse().ok());
    if let Ok(status) = std::env::var("TEST_SHARD_STATUS_FILE") {
        // Tells Bazel this test shards, so it doesn't run every input in
        // every shard.
        std::fs::write(status, "").unwrap();
    }
    (var("TEST_SHARD_INDEX").unwrap_or(0), var("TEST_TOTAL_SHARDS").unwrap_or(1))
}

fn replay(target: &str, run: impl Fn(&[u8])) {
    let dir = Path::new(CORPUS).join(target);
    let mut inputs: Vec<_> =
        std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("{}: {e}", dir.display())).map(|entry| entry.unwrap().path()).collect();
    inputs.sort();
    assert!(!inputs.is_empty(), "a corpus for {target}");
    let (index, total) = shard();
    for input in inputs.iter().skip(index).step_by(total) {
        let data = std::fs::read(input).unwrap();
        eprintln!("{}", input.display());
        run(&data);
    }
}

#[test]
fn world_corpus() {
    ecs_ops::quiet_expected_panics();
    replay("world", |data| {
        ecs_ops::world(&mut ecs_ops::Bytes::new(data), usize::MAX, 0);
    });
}

#[test]
fn columns_corpus() {
    ecs_ops::quiet_expected_panics();
    replay("columns", |data| ecs_ops::columns(&mut ecs_ops::Bytes::new(data), usize::MAX));
}
