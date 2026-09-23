# CLAUDE.md

Project guide for AI agents (Claude Code and others) working in this repo.
See [README.md](README.md) for the human-facing overview.

## What this project is

A game engine prototype in which **everything is a mod**. The engine binary
is a mod loader and nothing else; the frame loop, and eventually rendering,
input and the game itself, are dynamic libraries it loads and hot-reloads,
cr.h-style. The loader owns each mod's state, so a reload swaps code under
live data.

Three invariants, each of which a plausible change would break:

- **The loader stays bare.** A feature that "just needs a little help from
  the loader" is a mod, or a host service in `engine/api` that any mod can
  use. The loader's own loop exists only so the bootstrap mod can be
  reloaded: code on the stack can't be swapped.
- **Bazel is the build.** Everything is built by Bazel with hermetic
  toolchains (`rules_rs` for Rust, `llvm` for C/C++ and linking). There is
  no Cargo build; the `Cargo.toml` files only declare crates.io deps for
  `rules_rs` (see
  [runbooks/001](docs/runbooks/001-regenerate-cargo-lock.md)).
- **`bazel run` is the reload trigger.** `bazel run //mods/<name>` builds the
  mod and tells the running engine to load or reload it. A workflow that
  needs a second tool to reload something is a gap in the rules, not the
  way to do it.

Full design in [docs/architecture/](docs/architecture/). Read it before
changing the ABI, the reload sequence or the Bazel rules.

## Where things live

- `engine/api/` — the ABI between loader and mods. Its own crate because
  it is the only code linked into both sides: everything in it is
  `#[repr(C)]`, and changing a type's shape means bumping `API_VERSION`.
  Also holds the safe `Mod` trait and `export_mod!`, so a mod never touches
  the raw ABI.
- `engine/loader/` — the engine binary.
  - `engine.rs` — the mod list, state memory and host callbacks. The one
    file where mod code is called, so it owns the rule that mods run only
    while the list is shared-borrowed.
  - `control_server.rs` — serves the control socket. Separate because it is
    polled between frames, and that timing is what makes a reload safe.
  - `main.rs` — manifest reading and the trampoline loop.
- `engine/control/` — the control protocol and socket path. Its own crate
  because both the engine and `modctl` speak it; neither should import the
  other.
- `engine/modctl/` — the client. Every `engine_mod` target is a symlink to
  this binary with the mod's name and library baked in through
  `RunEnvironmentInfo`, which is what makes `bazel run //mods/x` a reload.
- `engine/defs.bzl` — `engine_mod` and `engine_game`. If a mod needs a new
  build setting (a link flag, a runtime linkage), it goes here, so every mod
  gets it.
- `mods/` — `bootstrap` (owns the frame loop), `counter` (hot-reload demo),
  `hello` (loaded live, not in the manifest).
- `game/` — the `engine_game` target listing the mods loaded at startup.
- `bazel` — runs a pinned, checksummed bazelisk so a fresh checkout needs no
  host Bazel. Always invoke Bazel as `./bazel`.
- `docs/architecture/` — design docs, one per area. Living documents.
- `docs/lore/` — non-obvious discoveries that cost real effort. See below.
- `docs/runbooks/` — recurring repo-maintenance procedures.

## Working conventions

- **Build and run through `./bazel`**, never `cargo build`/`cargo test`:
  Cargo uses a different toolchain and resolution path, so it can pass
  while the Bazel build is red. `cargo` is only for regenerating
  `Cargo.lock`.
- **Investigate fetched Bazel repos locally, not via their READMEs.** Once a
  build has fetched a ruleset into
  `$(./bazel info output_base)/external/<repo>/`, read its `.bzl` source for
  attributes and behavior. READMEs lag and show the happy path;
  `cc_runtime_linkage`, which the mods depend on, appears only in the
  patched `rules_rust` source (see
  [lore](docs/lore/rust-shared-libraries-link-the-cxx-runtime-dynamically.md)).
- **Verify a reload end to end, not just the build.** A mod change is done
  when `./bazel run //game` is running, `./bazel run //mods/<name>` reloads
  it, and the log shows the state carried over (or reset, if its layout
  changed). There are no automated tests yet; until there are, say which of
  these you ran and which you didn't.
- **Comments explain why, not what.** A "what" comment is a second copy of
  the code: it can only be redundant or wrong, and it turns wrong the
  moment the line it describes changes. Spend the comment on what the code
  can't say: the alternative that was rejected, a constraint from outside
  the file, the direction a wrong guess fails in.
  - **"What" earns its place in three spots:** a module-level `//!` or
    docstring orienting someone who lands in the file cold; a one-line
    gloss on a non-obvious helper *followed by* the why; and a mirror of an
    external schema. What-then-why is the pattern; what alone isn't.
  - **A comment is missing** not when a function lacks a doc comment (most
    don't need one) but when a reader would have to reconstruct a choice,
    or would plausibly *fix* it into a bug. Silent fallbacks, deliberate
    omissions and magic values are where that goes wrong: the copy before
    `dlopen`, the `.max(1)` on a zero-sized state layout.
  - **One home per rationale.** A comment says why *this code* is shaped
    this way; `docs/architecture/` says why the *design* is. Where they
    overlap, point at the doc instead of restating it.
  - **History goes in a footnote, not in the prose.** An abandoned approach
    is worth recording, since it stops the next person reinventing it, but
    it is not what a reader came for. Describe what the code does now, then
    section history off: a `[^slug]` footnote in a doc, a trailing
    *(History: ...)* in a comment. Date it and say what was removed and why.
  - **A comment stating a checkable claim is a test that hasn't been
    written yet.** "Linked with `-z now`, so a missing symbol fails here" is
    an assertion about behavior. Pin it when a test tier exists; until then,
    re-check it when the thing it names changes. Prose naming an identifier
    is a reference nothing checks, so re-grep after a rename.

## When you learn something non-obvious

If you hit a surprising `dlopen` behavior, a Bazel or toolchain quirk, or
find out why an approach doesn't work, add an entry to
[docs/lore/](docs/lore/) before the session ends. Measure the claim first
where you can: a lore entry teaches with the authority of experience, so a
wrong one does more harm than a missing one.
