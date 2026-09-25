# Runbook: run Miri and fuzz the unsafe core

- **Trigger:** a change to the ECS's unsafe code (`engine/ecs/erased.rs`,
  `schema.rs`, `component!`'s glue in `component.rs`, the bounds and key
  glue in `spatial.rs` and `ordered.rs`), to what calls it (`world.rs`'s
  moves, migrations and re-sorts), or to the drivers
  (`engine/ecs/tests/ops.rs`). Also after bumping the Miri nightly or the
  stable toolchain.
- **Why it isn't in `./bazel test //...`:** Miri interprets, a thousand
  times slower than native, and fuzzing runs until stopped. Both are
  tagged `manual`. What the default suite does run: the drivers seeded
  (`//engine/ecs:model_test`) and the checked-in corpus replayed natively
  (`//engine/ecs/fuzz:replay_test`), a few seconds together.

What each finds is in
[storage.md, "Testing the core"](../architecture/storage.md#testing-the-core).

## Miri

```sh
./bazel test //engine/ecs:miri          # the crate's tests and the integration tests
./bazel test //engine/ecs/fuzz:miri     # the fuzz corpus, replayed
```

Each is a pair of targets per test crate, `_sb` (Stacked Borrows) and `_tb`
(Tree Borrows), run in parallel. Measured on 32 cores (2026-09-25): the
first took 22 minutes of wall clock, its longest targets being the Tree
Borrows runs of `model_test`, `spatial_test` and `page_test` (20 to 22
minutes each; Stacked Borrows takes half that); the second, 41 minutes, in
16 shards per model. The first run also builds the Miri sysroot (about a
minute). A failure's
`test.log` has Miri's report: the kind of UB, and a backtrace to the line.

Miri's toolchain is its own nightly (`toolchains.experimental_miri` in
`MODULE.bazel`), registered for Miri's toolchain types only; see
[the lore](../lore/rules-rs-runs-miri-hermetically-and-only-its-source-says-so.md).
To bump it, change the date and run both commands; `MODULE.bazel.lock`
records the new checksums, so commit it with the change. A date without a
`miri-preview` component fails the fetch; try a day either side.

**One test at a time**: the runner takes no `--test_arg`, so filters can't
be passed through Bazel. Build the target, then run its args file by hand,
with libtest's arguments after the `--` it ends with:

```sh
./bazel build //engine/ecs:miri_model_test_sb
cd bazel-bin/engine/ecs/miri_model_test_sb.runfiles/_main
mapfile -t args < ../../miri_model_test_sb.miri_runner_args
"${args[@]}" random_column_operations_match_the_model
```

**A test too slow to interpret** is sized down with `sized(native, miri)`
in the test file, or skipped with `#[cfg_attr(miri, ignore = "why")]` when
its point needs the full size. Say which in the reason.

## Fuzzing

Two targets, one per driver: `world` (structural changes and migrations
against a map) and `columns` (`ErasedColumn`'s operations against vectors).
Run one from the repo root, into a scratch copy of the corpus, so what it
finds can be minimized before it's checked in:

```sh
mkdir -p /tmp/fuzz/world && cp engine/ecs/fuzz/corpus/world/* /tmp/fuzz/world/
./bazel run //engine/ecs/fuzz:world -- -fork=8 -max_total_time=1800 \
    -artifact_prefix=/tmp/fuzz/ /tmp/fuzz/world
```

`-fork=N` runs N workers; add `-ignore_crashes=1` to keep going past a
crash. Paths must be absolute: `bazel run` starts the binary in its
runfiles tree. libFuzzer's `WARNING: Failed to find function
"__sanitizer_..."` lines are expected (there is no ASan). A crash leaves
`/tmp/fuzz/crash-<sha1>`, and the log has the panic: the driver's step and
operation, and what differed from the model.

**When it finds something:**

1. Reproduce: `./bazel run //engine/ecs/fuzz:world -- /tmp/fuzz/crash-<sha1>`.
2. Minimize: add `-minimize_crash=1 -runs=100000` to that, for a shorter
   input that fails the same way.
3. Fix it, and add a test at the lowest tier that can fail on it (usually
   a unit test in `erased.rs`).
4. Put the minimized input in `engine/ecs/fuzz/corpus/<target>/`, named
   for what it found, so the replays keep checking it.

**Refreshing the corpus** after a campaign: merged into an empty directory
(merging into the checked-in one only adds to it), down to the inputs that
add coverage, counting edges only (`-use_counters=0`), so the replay under
Miri stays short:

```sh
mkdir /tmp/fuzz/min
./bazel run //engine/ecs/fuzz:world -- -merge=1 -use_counters=0 /tmp/fuzz/min \
    "$PWD/engine/ecs/fuzz/corpus/world" /tmp/fuzz/world
rm engine/ecs/fuzz/corpus/world/* && cp /tmp/fuzz/min/* engine/ecs/fuzz/corpus/world/
./bazel test //engine/ecs/fuzz:replay_test //engine/ecs/fuzz:miri
```

Keep crash regressions (named inputs) when replacing the rest. The Miri
replay is sharded by input (`shard_count` in `engine/ecs/fuzz/BUILD.bazel`;
the test shards itself, since libtest doesn't): raise it if the corpus
grows, or merge it harder: the time is in the world inputs.

## A seeded soak

The drivers seeded, for long, natively, without libFuzzer: a quick check
after changing a driver, and a different distribution of operations from
the fuzzer's.

```sh
./bazel test //engine/ecs:model_test --test_arg=--ignored --test_arg=soak \
    --test_env=SEED=7 --test_env=STEPS=200000 --test_output=errors
```
