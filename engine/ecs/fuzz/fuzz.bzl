"""`fuzz_binary`: a rust_binary built for libFuzzer.

Every Rust crate in the binary (the ECS, the drivers, the harness) is
compiled with SanitizerCoverage, the edge counters and comparison hooks
libFuzzer's `-fsanitize=fuzzer` would add to C++, through rustc's stable
`-Cpasses` and `-Cllvm-args`, so the stable toolchain builds it. A
transition sets them, so the crates are built once more in the binary's
own configuration and the default build never sees the flags.

No AddressSanitizer: rustc's `-Zsanitizer` is nightly-only, and the only
nightly toolchain in the build is Miri's. The drivers' canaries catch
double drops and leaks natively, and the corpus is replayed under Miri for
the rest (docs/runbooks/002-run-miri-and-fuzz-the-core.md).
"""

load("@rules_rs//rs:rust_binary.bzl", "rust_binary")
load("@with_cfg.bzl", "with_cfg")

# As cargo-fuzz passes them.
_SANCOV = [
    "-Cpasses=sancov-module",
    "-Cllvm-args=-sanitizer-coverage-level=4",
    "-Cllvm-args=-sanitizer-coverage-inline-8bit-counters",
    "-Cllvm-args=-sanitizer-coverage-pc-table",
    "-Cllvm-args=-sanitizer-coverage-trace-compares",
    "--cfg=fuzzing",
    # Optimized, as a fuzzer should be, but with the checks a test build
    # has: an overflow or a debug assertion is a finding too.
    "-Cdebug-assertions=on",
    "-Coverflow-checks=on",
]

fuzz_binary, _fuzz_binary_internal = with_cfg(rust_binary).set(
    "compilation_mode",
    "opt",
).extend(
    Label("@rules_rust//rust/settings:extra_rustc_flags"),
    _SANCOV,
).build()
