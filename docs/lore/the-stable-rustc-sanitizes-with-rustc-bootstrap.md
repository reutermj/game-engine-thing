# The stable rustc sanitizes, with RUSTC_BOOTSTRAP

`-Zsanitizer` is a nightly flag, but nothing it needs is nightly-only. The
stable `rust-std` for `x86_64-unknown-linux-gnu` ships the runtimes
(`lib/rustlib/x86_64-unknown-linux-gnu/lib/librustc-stable_rt.{asan,lsan,msan,tsan}.a`,
checked in the 1.98.1 component), and `RUSTC_BOOTSTRAP=1` unlocks the flag
on the stable compiler. rules_rust has a setting for exactly that
(`@rules_rust//rust/settings:extra_rustc_env`, whose docstring names
`RUSTC_BOOTSTRAP=1`). So `//engine:sanitize.bzl` needs no second toolchain
for AddressSanitizer: a transition adds `-Zsanitizer=address` to
`extra_rustc_flags` and the env var to `extra_rustc_env`, and every crate
under the test builds again in that configuration.

Staying on the one stable rustc matters here beyond convenience: the
loader refuses a mod built by another rustc than itself (the one-compiler
rule), so a nightly engine could only load nightly mods.

## Sharp edges

- **A dlopened mod can't find the runtime.** rustc links a sanitizer's
  runtime into executables only, and a `cdylib` built with `-Zsanitizer`
  calls into it, so every mod failed to load with `undefined symbol:
  __asan_option_detect_stack_use_after_return` (it is linked `-z now`).
  `-Clink-arg=-Wl,--export-dynamic` on everything puts the executable's
  symbols, the runtime's among them, where `dlopen` resolves them; on a
  mod it changes nothing, since rustc's version script decides a cdylib's
  exports.
- **A leak in an unloaded build prints `<unknown module>`** for every frame
  in it: LeakSanitizer reports at exit, when the build is long gone. The
  sanitized tests set `ENGINE_POISON_UNLOADED=keep` (never close a build,
  and keep its staged file) so the frames resolve. But keeping a build
  keeps its statics, and a leak that is only a leak *because* the build was
  unmapped (a heap pointer held by one of its statics) is then reachable
  and unreported; see
  [the stdout leak](a-mods-own-std-leaks-its-stdout-buffer-when-unloaded.md)
  for how to find those.
- rustc's ABI check (`mixing -Zsanitizer will cause an ABI mismatch`)
  fires for `thread` against the precompiled std, not for `address`
  (measured 1.98.1): ASan builds on the prebuilt std as is. TSan needs
  std rebuilt from source, and so a second toolchain; that recipe was
  deferred (bead get-lm3).
- **`bazel test --test_env` doesn't override a test rule's `env`**; the
  rule's value wins. A mode the sanitized tests force (the poison mode) is
  changed in `//engine:sanitize.bzl`, not on the command line.

Measured 2026-09-25, 32 cores, other agents sharing the machine: building
the five sanitized tests under ASan took 15 s after the default build
(1 687 actions), and running them 48 s against 17 s for the whole default
suite; `physics_test` is most of it (47 s against 15 s).
