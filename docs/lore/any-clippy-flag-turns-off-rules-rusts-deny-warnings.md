# Any clippy flag turns off rules_rust's deny-warnings

`rust_clippy_aspect` fails a target on any clippy warning by default, and
stops doing so the moment a clippy flag is set, anywhere. In
`rust/private/clippy.bzl` (the patched rules_rust under
`$(./bazel info output_base)/external/rules_rs++rules_rust+rules_rust/`):

```python
if clippy_flags or lint_files:
    args.rustc_flags.add_all(clippy_flags)
else:
    args.rustc_flags.add("-Dwarnings")
```

`clippy_flags` is the target's `lint_config` flags plus every
`--@rules_rust//rust/settings:clippy_flag`. So giving mod crates a
`lint_config` that denies two lints, or allowing one lint repo-wide on the
command line, quietly demotes every other lint to a warning. The action then
succeeds, Bazel caches it, and the warning is never shown again: a
lint gate that passes everything.

Measured 2026-09-25 with `--ignore_all_rc_files` and a `manual_contains`
planted in `engine/loader/main.rs`: the aspect alone fails the build; the
same build with `--@rules_rust//rust/settings:clippy_flag=-Aclippy::type_complexity`
added prints the lint as a warning and completes successfully.

## Resolution

`.bazelrc` passes `clippy_flag=-Dwarnings` itself, first, on `common`, so the
deny doesn't depend on whether any other flag is set.
