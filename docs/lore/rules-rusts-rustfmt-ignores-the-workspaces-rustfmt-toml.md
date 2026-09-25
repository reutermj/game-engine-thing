# rules_rust's rustfmt ignores the workspace's rustfmt.toml

`./bazel run @rules_rust//:rustfmt` and the `rustfmt_aspect` don't read a
`rustfmt.toml` at the workspace root. They read the file the label flag
`@rules_rust//rust/settings:rustfmt.toml` names, which defaults to one inside
rules_rust (`settings.bzl`, `rustfmt_toml`). With our `rustfmt.toml`
(`max_width = 140`) present and the flag unset, a reformat broke lines at
rustfmt's default of 100 and changed 102 files (+8942 −2224). With
`--@rules_rust//rust/settings:rustfmt.toml=//:rustfmt.toml` in `.bazelrc`
(and the file exported from the root package) the same reformat changed 64
files. Measured 2026-09-25.

Nothing warns: the formatter runs, succeeds, and formats to the wrong
config, and the check then enforces that wrong config too. If formatting
ever looks off, check the flag before the config.
