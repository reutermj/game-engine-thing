# rules_rs runs Miri hermetically, and only its source says so

Miri needs a nightly toolchain and a sysroot built from the standard
library's source with Miri's flags, which is what `cargo miri setup`
normally does outside any build system. `rules_rs` 0.0.111 does all of it
as Bazel actions, but nothing outside its source mentions it: not the
README, not the release notes. Read in the fetched source,
`$(./bazel info output_base)/external/rules_rs+/rs/experimental/miri/`,
and its test, `test/miri/BUILD.bazel`.

- `toolchains.experimental_miri(version = "nightly/<date>")` on the same
  extension as the stable toolchain downloads that nightly's `rustc`,
  `rust-std`, `rustc-src` and `miri-preview` (checksums recorded in
  `MODULE.bazel.lock`), and declares toolchains of **its own types**
  (`@rules_rs//rs/experimental/miri:toolchain_type`), in a repo registered
  with `register_toolchains("@miri_toolchains//:all")`. So the stable
  toolchain stays the one every other target builds with: nothing but a
  `miri_test` resolves a Miri toolchain.
- The sysroot is built from the nightly's library source by an aspect
  (`miri_sysroot_compile_aspect`), once, cached like any action: about a
  minute the first time.
- `miri_test` compiles the test crate's dependencies through another aspect
  on `deps` (as Miri-as-rustc, `MIRI_BE_RUSTC=target`), then runs the test
  crate under the interpreter. A crate's own unit tests are a `miri_test`
  with the crate's `srcs` and `crate_root`, since it takes no `crate`.

## Sharp edges

- **The runner takes only its args file.** `--test_arg` fails with
  `usage: miri_test_runner @args-file`; libtest filters and Miri flags go
  in the rule's `args` and `miri_flags`, fixed at build time. Tests too
  slow to interpret are marked `#[cfg_attr(miri, ignore = "...")]` next to
  the test, where a rename can't silently make a filter match nothing.
- **Miri hides the host's environment** unless isolation is off
  (`-Zmiri-disable-isolation`) or a variable is forwarded
  (`-Zmiri-env-forward=NAME`). libtest's unstable options (`--report-time`)
  need `RUSTC_BOOTSTRAP=1` visible to the test, so forwarded. Reading files
  (the fuzz corpus replay) needs isolation off.
- **Interpreting is about a thousand times slower**, and the storage's
  tests spawn hundreds of rows through re-sorting tables: a test that only
  populated 700 spatial rows took six minutes under Miri (measured
  2026-09-25). The tests size their inputs with `sized(native, miri)`.
- Miri finishes a test binary at the first UB it finds, so a second bug is
  only seen once the first is fixed (or its test ignored).
- **Miri runs one thread at a time**, so a test binary is one core however
  many tests it has. The fuzz corpus (35 KB of inputs) takes 41 minutes
  even in 16 shards per aliasing model (Tree Borrows about twice Stacked
  Borrows' time, measured 2026-09-25); it's sharded with `shard_count`, which a
  `miri_test` accepts like any test, and the test itself reading
  `TEST_SHARD_INDEX` (visible with isolation off) and touching
  `TEST_SHARD_STATUS_FILE`.
