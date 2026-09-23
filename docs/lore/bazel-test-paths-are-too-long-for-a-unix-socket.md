# Bazel test paths are too long for a Unix socket

A Unix socket's path is limited to 108 bytes (`sun_path`), and a sandboxed
test's `$TEST_TMPDIR` alone uses most of that:

```
/home/mark/.cache/bazel/_bazel_mark/<32 hex>/sandbox/linux-sandbox/238/execroot/_main/_tmp/<32 hex>/
```

An engine told to put its socket there fails at startup with
`path must be shorter than SUN_LEN`. The first sign in the e2e tests was
every engine test failing with nothing but `modctl`'s "can't reach the
engine": the harness only showed `modctl`'s output, so the engine's error was
invisible. It now reports the engine's output whenever the engine exits early.

## Resolution, and the two tempting fixes that don't work

`engine/tests/e2e_test.rs` puts each test's runtime directory directly under
`/tmp`, named after a hash of `$TEST_TMPDIR`.

- **The pid is not unique.** Each sandbox has its own pid namespace, so every
  concurrent run of the test binary had the same small pid (measured: 12).
  Directories named `/tmp/engine-e2e-<pid>-<test>` collided under
  `--runs_per_test`, and one run's engine found another's socket.
- **`/tmp` is not private to a run.** The collision shows it is shared between
  concurrent test runs, even though nothing appeared in the host's `/tmp`.

`$TEST_TMPDIR` is unique per run, so a hash of it is a short, unique name.
