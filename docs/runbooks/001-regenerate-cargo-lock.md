# Runbook: regenerate Cargo.lock

- **Trigger:** a crates.io dependency was added, removed or bumped in a
  member `Cargo.toml` (today only `engine/loader/Cargo.toml`), a new member
  was added to the root `Cargo.toml`, or the Rust toolchain version in
  `MODULE.bazel` changed.

## Gap

Third-party crates reach Bazel through `rules_rs`'s `crate.from_cargo` in
`MODULE.bazel`, which generates the `@crates` repo from the root
`Cargo.lock`. `rules_rs` reads that lockfile but never writes it. Its module
extension runs `cargo metadata --no-deps --locked` and parses the lock
(`rs/extensions.bzl`, `_generate_hub_and_spokes`, read in the fetched source
under `$(./bazel info output_base)/external/rules_rs+/`). There is no
repin target or environment variable, so the lockfile has to come from a
real `cargo`.

Forgetting it doesn't fail where you'd expect. Editing only a member
`Cargo.toml` doesn't re-run the extension at all, and the build carries on.
The failure appears once a `BUILD` file references the new crate:

```
ERROR: .../engine/loader/BUILD.bazel:3:12: no such target
'@@rules_rs++crate+crates//:bitflags': target 'bitflags' not declared in package ''
```

## Resolution

With a local `cargo` (only for this; no Bazel build uses it):

```sh
cargo update --workspace   # add the new entries, leave every other version as is
# or: cargo generate-lockfile   to re-resolve everything to the latest versions
```

Cargo picks versions compatible with the member's `rust-version`, or with
the *host's* `rustc` when that is unset, and the host's is not the toolchain
Bazel builds with. Check the `Locking ... to latest Rust <version>
compatible version` line names the version in `MODULE.bazel`. A new member
`Cargo.toml` needs `rust-version` set for this, and bumping the toolchain in
`MODULE.bazel` means bumping every `rust-version` with it.

Then add `@crates//:<crate>` to the target's `deps` and verify through Bazel:

```sh
./bazel build //...
```

Commit `Cargo.lock` and `MODULE.bazel.lock` together with the `Cargo.toml`
change.
