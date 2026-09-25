# Runbook: fuzz hot reload

- **Trigger:** a change to the reload sequence or what it relies on: the
  loader (`engine/loader/engine.rs`), installing layouts and migrating
  values (`engine/ecs/world.rs`, `schema.rs`), event queues (`events.rs`),
  ordered or spatial re-sorts, keepalives, the ABI (`engine/api`); or to
  the fuzzer's test mods and model (`engine/tests/fuzz/`). Also before
  trusting a new rule in hot-reload.md or mod-deps.md: add it to the model
  first.
- **Why it isn't in `./bazel test //...`:** fuzzing runs until stopped. What
  the default suite runs: `//engine/tests/fuzz:replay_test`, the checked-in
  corpus and scripts replayed, and a dozen seeded sessions that must between
  them reach every kind of reply the model knows (about 20 s).

- **Related:** the replay test runs in poison mode, so a stale pointer
  into an unloaded build faults at once; the loader under AddressSanitizer
  is [runbook 004](004-run-the-loader-under-sanitizers.md).

What the driver covers, and the model's rules, are in
[hot-reload.md, "How reload is tested"](../architecture/hot-reload.md#how-reload-is-tested).

## Fuzzing

```sh
mkdir -p /tmp/fuzz/reload && cp engine/tests/fuzz/corpus/* /tmp/fuzz/reload/
./bazel run //engine/tests/fuzz:reload -- -fork=6 -max_total_time=1800 \
    -close_fd_mask=1 -artifact_prefix=/tmp/fuzz/ /tmp/fuzz/reload
```

A worker runs about 16 inputs a second (each is a session on a fresh
engine, a few milliseconds a load), so give it workers: `-fork=N`, and
`-ignore_crashes=1` to keep going past a crash. `-close_fd_mask=1` closes
stdout, which the engine logs every load to; failures are on stderr. As
for the ECS fuzzers, paths must be absolute, and the `__sanitizer_`
warnings are expected. `//engine/tests/fuzz:reload` is the libFuzzer
binary (the loader and ECS instrumented) launched with the test mods
built normally: instrumented mods would be unmapped with libFuzzer's
counters in them.

**When it finds something**, the log has the check that failed and the
session up to it, one operation a line, in the script format:

1. Reproduce: `./bazel run //engine/tests/fuzz:reload -- /tmp/fuzz/crash-<sha1>`.
   A crash (a segfault) leaves no report; rerun with `RELOAD_FUZZ_TRACE=1`
   in the environment, which prints each operation as it starts.
2. Save the session as a script (the lines after "the session", or the
   trace, without the replies) and reduce it by hand, rerunning it alone:

   ```sh
   RELOAD_FUZZ_SCRIPT=/tmp/fuzz/crash.txt ./bazel run //engine/tests/fuzz:replay_test -- scripts_replay --nocapture
   ```

   Loads, batches and reloads of one mod usually matter; messages and
   frames between them often don't.
3. Decide whether the engine or the model is wrong: the model follows the
   docs, so a model that disagrees with a documented rule is the bug.
4. Fix it; put the reduced script in `engine/tests/fuzz/scripts/`, named for
   what it found, with a comment saying so, and a lower-tier test where one
   can see it.

**Refreshing the corpus**, as for the ECS fuzzers (runbook 002): merge into
an empty directory, edges only, and replace the checked-in inputs.

```sh
mkdir /tmp/fuzz/min
./bazel run //engine/tests/fuzz:reload -- -merge=1 -use_counters=0 -close_fd_mask=1 \
    /tmp/fuzz/min "$PWD/engine/tests/fuzz/corpus" /tmp/fuzz/reload
rm engine/tests/fuzz/corpus/* && cp /tmp/fuzz/min/* engine/tests/fuzz/corpus/
./bazel test //engine/tests/fuzz:replay_test
```

## Checking the fuzzer still finds bugs

`engine/tests/fuzz/planted/` has bugs planted in the loader and the ECS,
one patch each, and how long the fuzzer took to find each when it was
written (6 workers, 2026-09-25): from a one-input seed corpus, and from the
checked-in corpus after a 40-minute campaign.

| patch | the bug | from the seed | from the corpus |
|---|---|---|---|
| `1-migration-saturates` | integer narrowing saturates instead of wrapping | 96 s | 20 s |
| `2-unload-keeps-its-build` | an unloaded mod's build stays mapped | 1 s | 7 s |
| `3-new-key-glue-keys-only-what-changed` | a new ordered key's glue doesn't rekey the rows | 861 s | 146 s |
| `4-reload-rereads-events` | a reload resets event cursors: events seen twice | 753 s | 615 s |
| `5-state-migrates-across-a-version-bump` | a state version bump migrates instead of resetting | 2 s | 8 s |
| `6-world-holds-no-build` | the world keeps no build mapped for its values | 1 s | 8 s |

And `//engine/tests/fuzz:replay_test`, in the default suite, fails on each
of the six (checked 2026-09-25): the corpus replay or the seeded sessions
reach every one.

After changing the driver's choices, check some still fall: `git apply`
one, run the fuzzer from a copy of the corpus, and `git apply -R` it. A
patch that no longer applies needs redoing by hand; what it breaks is in
its name. The driver's weights were set by bug 4, which the fuzzer missed
for 30 minutes while it picked builds uniformly (bugs 1 to 3 were timed
from the seed before that change, 4 to 6 after).

## A seeded soak

```sh
./bazel test //engine/tests/fuzz:replay_test --test_arg=--ignored --test_arg=soak \
    --test_env=SEED=7 --test_env=STEPS=100000 --test_env=SESSION=500 --test_timeout=3600 --test_output=errors
```

About 220 operations a second, in sessions of `SESSION` operations. The
same driver as the fuzzer, with other choices: runs of hundreds of
operations on one engine, where the fuzzer's inputs are short.
