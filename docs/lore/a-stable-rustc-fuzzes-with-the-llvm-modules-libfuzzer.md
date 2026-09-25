# A stable rustc fuzzes with the llvm module's libFuzzer

cargo-fuzz needs nightly only for its sanitizers. Coverage-guided fuzzing
itself is two things, and neither needs nightly or cargo:

- **SanitizerCoverage instrumentation**, through rustc's stable codegen
  flags: `-Cpasses=sancov-module` and `-Cllvm-args=-sanitizer-coverage-*`
  (level 4, inline 8-bit counters, PC table, trace-compares), as
  cargo-fuzz passes them.
- **libFuzzer**, which the hermetic `llvm` module builds from source:
  `@llvm//runtimes/compiler-rt:clang_rt.fuzzer.static` is a static library
  with libFuzzer's `main`. It isn't a `cc_library`, so it goes in through a
  `cc_import` with `alwayslink = True` (nothing in the Rust code refers to
  it). The Rust binary is `#![no_main]` and exports
  `LLVMFuzzerTestOneInput`; rustc links it with the C++ runtime as for any
  C++ dependency, with no further flags.

The flags must reach every crate in the binary, not just the harness, or
libFuzzer sees no coverage of the code under test. A `with_cfg.bzl`
transition that extends `@rules_rust//rust/settings:extra_rustc_flags`
does it, and builds those crates a second time in the binary's own
configuration, so the default build never sees the flags. `aquery` on the
binary shows `sancov-module` on `engine_ecs` and `ecs_ops` (checked
2026-09-25).

## Sharp edges

- `with_cfg.bzl` 0.12.0 wrapping `rust_binary` fails at load time with
  `'NoneType' value has no field or method 'items'`: it reads
  `exec_properties` as a dict, and gets `None`. Passing
  `exec_properties = {}` works around it.
- libFuzzer starts with `WARNING: Failed to find function
  "__sanitizer_print_stack_trace"` and two more: those are ASan's, and
  there is no ASan. Harmless. A panic aborts, which libFuzzer reports as a
  crash with the input, which is all a failed check needs.
- Without AddressSanitizer (`-Zsanitizer` is nightly-only), memory errors
  natively only show if they crash or corrupt what the model compares.
  The drivers make the rest visible: heap values count their drops
  (`Canary` in `engine/ecs/tests/ops.rs`), and the corpus replays under
  Miri, which sees every one.
