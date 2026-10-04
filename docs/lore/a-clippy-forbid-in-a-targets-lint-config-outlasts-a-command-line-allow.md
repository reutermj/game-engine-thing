# A clippy forbid in a target's lint_config outlasts a command-line allow

A lint a target's `lint_config` sets with `-F` (forbid) can't be turned off
by an allow passed later on the command line, where one set with `-D`
(deny) can. Measured 2026-10-04 with clippy-driver 1.98.1, on
`clippy::disallowed_methods`: `-F clippy::disallowed_methods -Dwarnings
-A clippy::disallowed_methods` still errors on a listed call; with `-D`
in place of `-F`, the trailing `-A` silences it.

The order matters because of where rules_rust puts the flags. In
`rust/private/clippy.bzl` (the copy `rules_rs` ships, under
`$(./bazel info output_base)/external/`), the lint config's flags go
before every `--@rules_rust//rust/settings:clippy_flag`, so `.bazelrc`'s
repo-wide flags come last and win over a `-D`, but never over a `-F`.

That is how the thread lint is laid out: `//:clippy.toml` lists
`std::thread::spawn`, `std::thread::scope`, `Builder::spawn` and
`Builder::spawn_scoped` under `disallowed_methods`; `//engine:mod_lints`
forbids the lint for reloadable mods' code; `.bazelrc` allows it
everywhere else; and resident mods and `engine_ecs` take
`//engine:mod_lints_threads_allowed`, which forbids printing but not
threads. A deny in `mod_lints` would have been quietly undone by
`.bazelrc`'s allow.

**A second trap, found at the same time:** `./bazel test` on an
`engine_mod` target doesn't lint its library, since the clippy aspect
doesn't reach the library through the mod's wrapper rule. The pattern
`//mods/...` does, as `//...` does. To check a mod's lints, test a
pattern, not the mod's own label.

See also [any-clippy-flag-turns-off-rules-rusts-deny-warnings](any-clippy-flag-turns-off-rules-rusts-deny-warnings.md),
the other way the same flag order bites.
