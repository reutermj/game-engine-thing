# Runbook: run the loader under the sanitizers, and read a poison fault

- **Trigger:** a change to how the loader opens, swaps or drops builds
  (`engine/loader/engine.rs`, `poison.rs`), to the host threads or the
  control server, to what the world keeps of a build (keepalives, drop
  functions), or a crash in a reload test that stops the log mid-line.
- **Why it isn't in `./bazel test //...`:** it needs a second build of
  everything under the tests, and runs them about three times slower (48 s
  for the five, against 17 s for the whole default suite, measured
  2026-09-25). Tagged `manual`. What the default suite does run is the
  poison mode, which costs nothing measurable.
- **Related:** the reload fuzzer ([runbook 003](003-fuzz-hot-reload.md))
  finds wrong reloads, which a sanitizer can't; how the pieces fit is in
  [hot-reload.md, "How reload is tested"](../architecture/hot-reload.md#how-reload-is-tested).

## Poison mode (always on in the loader's tests)

Every test that loads mods into a real engine runs with
`ENGINE_POISON_UNLOADED=1` (`POISON_ENV` in `engine/defs.bzl`): when a
build's last keepalive drops, the loader closes it and maps its span
`PROT_NONE`, so a stale pointer into it (a drop function, a vtable, a
system) faults at its first use. A fault there prints, before the process
dies:

```
[poison] fault at 0x7f22e1dc1730, inside an unloaded build of <path>/libbag_v1_mod.so; llvm-symbolizer --obj=<that> 0x60730 names it
  called from:
  <path>/reload_test 0x3c3608
  backtrace:
  ...
```

Name both ends with the llvm module's symbolizer (the `called from` line is
the return address, one byte past the call):

```sh
SYM=$(./bazel info output_base)/external/llvm++llvm_toolchain_minimal+llvm-toolchain-minimal-linux-amd64/bin/llvm-symbolizer
$SYM --obj=<path>/libbag_v1_mod.so 0x60730      # what was called
$SYM --obj=<path>/reload_test 0x3c3607          # who called it
```

The lines are stderr, so a test that doesn't fail still hides them: add
`--test_arg=--nocapture` to see the rest (`stayed mapped`, `couldn't
guard`). Other values of the variable: `strict` also protects builds
`dlclose` keeps mapped (a TLS destructor registered in them; it faults at
that thread's exit, which is the point: it shows what pins the build);
`keep` never closes a build (what the sanitized tests use).

## AddressSanitizer and LeakSanitizer

```sh
./bazel test //engine/tests:asan    # every sanitized test, about 1 min
./bazel test //engine/tests:reload_test_asan --test_filter=pumping   # one test
```

Each wraps a test in `SANITIZED_TESTS` (`engine/sanitize.bzl`): the test and
every crate under it, the mods and the engine binary included, built again
with `-Zsanitizer=address` on the stable rustc (`RUSTC_BOOTSTRAP=1`), on the
precompiled std. A report fails the test: ASan's at the first error,
LeakSanitizer's at exit (exit code 23). An e2e test fails too, since it
asserts the engine's exit status.

ThreadSanitizer isn't here: it needs std built from source, and so a second
toolchain, which was deferred (bead get-lm3).

**Reading a report:** frames carry file and line
(`-Cdebuginfo=line-tables-only`). A frame in a mod that says
`<unknown module>` is in a build unloaded before the report; the
sanitized tests keep builds mapped (`ENGINE_POISON_UNLOADED=keep`) so it
shouldn't. **A leak that exists only because a build was unmapped** (a
static in the build holding heap memory) is invisible with the builds
kept; to look for those, run with them kept and statics not counted as
roots:

```sh
./bazel test //engine/tests:reload_test_asan --test_env=LSAN_OPTIONS=use_globals=0
```

and read the reports whose stack is in a mod (the rest are the host's own
statics, which are fine).

**Adding a test:** `sanitized_tests(tests = [":x_test"])` in its package,
and its label in `SANITIZED_TESTS`, which the suite is made from. A test that
counts the builds still mapped can't pass with them kept: `skip` it
(physics does).

What each found, and the build's sharp edges, are in the lore:
[ASan](../lore/the-stable-rustc-sanitizes-with-rustc-bootstrap.md),
[the stdout leak](../lore/a-mods-own-std-leaks-its-stdout-buffer-when-unloaded.md).
