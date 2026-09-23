# CLAUDE.md

Project guide for AI agents (Claude Code and others) working in this repo.
See [README.md](README.md) for the human-facing overview.

## What this project is

A game engine prototype in which **everything is a mod**. The engine binary
is a mod loader and nothing else; the frame loop, and eventually rendering,
input and the game itself, are dynamic libraries it loads and hot-reloads,
cr.h-style. The loader owns each mod's state, and the ECS world where game
data lives, so a reload swaps code under live data.

Three invariants, each of which a plausible change would break:

- **The loader stays bare.** A feature that "just needs a little help from
  the loader" is a mod, or a host service in `engine/api` that any mod can
  use. It doesn't even have a loop: the resident bootstrap mod runs the
  session and hands the loader control once a frame (`pump_loader`). Code on
  the stack can't be swapped, so the loader only swaps builds when every mod
  running is resident.
- **Bazel is the build.** Everything is built by Bazel with hermetic
  toolchains (`rules_rs` for Rust, `llvm` for C/C++ and linking). There is
  no Cargo build; the `Cargo.toml` files only declare crates.io deps for
  `rules_rs` (see
  [runbooks/001](docs/runbooks/001-regenerate-cargo-lock.md)).
- **`bazel run` is the reload trigger.** `bazel run //mods/<name>` builds the
  mod and tells the running engine to load or reload it; `bazel run
  //game:reload` does the same for every mod an interface change reached. A
  workflow that needs a second tool to reload something is a gap in the
  rules, not the way to do it.

Full design in [docs/architecture/](docs/architecture/). Read it before
changing the ABI, the reload sequence or the Bazel rules.

## Where things live

- `engine/api/` — the ABI between loader and mods. Its own crate because
  it is the only code linked into both sides: everything in it is
  `#[repr(C)]`, and changing a type's shape means bumping `API_VERSION`.
  Also holds the safe `Mod` trait, `mod_state!` and `export_mod!`, so a mod
  never touches the raw ABI. A mod's state is carried across reloads and so
  follows the component rule (only `FieldType` fields); anything else it
  keeps goes in its `Transient`, which each build makes for itself. See
  [docs/architecture/hot-reload.md](docs/architecture/hot-reload.md#who-owns-state).
  - `ecs.rs` — the world's ABI (`WorldApi`) and the typed `World`/`Component`
    API over it. Separate from `lib.rs` because it is a second contract: how
    mods share data, not how a mod is loaded.
  - `scheduler.rs` — the `Scheduler` service the engine declares, and the
    frame primitives a scheduler mod is built from.
  - `system.rs` — systems: `Mod::systems` declarations, `Query` and
    `EventReader` parameters (which are the access declaration), commands
    and events. See
    [docs/architecture/scheduling.md](docs/architecture/scheduling.md).
  - `service.rs` — calls between mods: `service!`, which generates a
    provider trait and caller functions, and the host calls that resolve a
    call to the provider's current build. See
    [docs/architecture/mod-deps.md](docs/architecture/mod-deps.md#calls-between-mods).
- `engine/loader/` — the engine: a library (`lib.rs`) with everything but
  the manifest and the loop, so tests can drive an `Engine` directly, and
  the binary (`main.rs`) around it.
  - `engine.rs` — the mod list, state memory, host callbacks and request
    handling. The one file where mod code is called, so it owns the rules
    that mods run only while the list is shared-borrowed and that requests
    are served only at a safe point (`safe_point`).
  - `schedule.rs` — turns every build's declarations into the order a
    frame runs systems in. Pure, so it's unit-tested alone, and so a load
    can be checked against it before it commits.
  - `world.rs` — the ECS storage behind `WorldApi`, and the commands and
    events a frame defers. Untyped by design: it
    holds only bytes and layouts, never code from a mod, so no reload can
    leave it pointing into an unmapped library. See
    [docs/architecture/ecs.md](docs/architecture/ecs.md).
  - `control_server.rs` — the control socket's thread, which reads requests
    and queues them for the engine. Separate because it's the one other
    thread in the loader, and must never run mod code: requests wait for the
    bootstrap's pump, and that timing is what makes a reload safe.
- `engine/control/` — the control protocol and socket path. Its own crate
  because both the engine and `modctl` speak it; neither should import the
  other.
- `engine/modctl/` — the client. Every `engine_mod` target is a symlink to
  this binary with the mod's name and library baked in through
  `RunEnvironmentInfo`, which is what makes `bazel run //mods/x` a reload.
- `engine/defs.bzl` — `engine_mod` (with its `interface`, `mod_deps` and
  `resident`) and `engine_game` (with its reload target). If a mod needs a new build
  setting (a link flag, a runtime linkage), it goes here, so every mod gets
  it.
- `engine/tools/mod_links.rs` — the build action that digests a mod's
  interface, so the engine can check at load time what each build was
  compiled against. See
  [docs/architecture/mod-deps.md](docs/architecture/mod-deps.md).
- `engine/std/` — the mods the engine ships, which `engine_game` uses by
  default: `realtime` (the frame loop in real time) and `lockstep` (frames
  only when sent `step N`), both resident bootstraps; `clock` (the `Clock`
  both publish); and `sequential`, the default scheduler.
- `mods/` — demo mods: `counter` (per-mod state across reloads), `hello` (loaded live, not in the manifest), and the ECS
  demo: `transform` declares `Position` and runs nothing, `physics` declares
  `Velocity` and moves things, `spawner` creates entities, `reporter` prints
  positions. Their `mod_deps` are the dependency example.
- `engine/tests/` — integration and e2e tests, and the test mods they load.
  The test mods are separate from `mods/` so editing a demo never changes
  what a test proves. See the testing conventions below.
- `game/` — the `engine_game` target listing the mods loaded at startup,
  and its `reload` target.
- `platformer/` — the second game: `core` (the player and the rules, mod
  `platformer`), `walkers` (enemies), `level` (the map, `map.txt`, rebuilt
  live when it changes), `text`, and `platformer_test`, which replays routes
  an agent played.
- `pong/` — the first real game: `core` (the rules, mod `pong`), `ai`,
  `text` (commands and drawing over messages), the game target on the
  lockstep bootstrap, and `pong_test`, which plays it through messages.
- `bazel` — runs a pinned, checksummed bazelisk so a fresh checkout needs no
  host Bazel. Always invoke Bazel as `./bazel`.
- `docs/architecture/` — design docs, one per area. Living documents.
- `docs/lore/` — non-obvious discoveries that cost real effort. See below.
- `docs/runbooks/` — recurring repo-maintenance procedures.
- `docs/retrospectives/` — what a piece of work showed about the design,
  kept as evidence. Read the latest before redesigning a part it covers.

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
- **Tests come in three tiers, and each proves something the others
  can't.** Run them all with `./bazel test //...`. Fix a bug at the lowest
  tier that can fail on it.
  - *Unit tests* (`//engine/api:api_test`, `//engine/loader:loader_test`,
    `//engine/control:control_test`) cover pure logic: the world store,
    migration, the `component!` schema, `__dispatch`, the protocol. They
    are the cheap place to pin edge cases and negatives.
  - *Integration tests* (`//engine/tests:reload_test`) load real mod
    libraries into a real `Engine` and step exact frame counts: no process,
    socket or sleeping. The only tier that exercises staging, `dlopen` and
    the state handoff.
  - *End-to-end tests* (`//engine/tests:e2e_test`) run the engine binary
    and drive it with `modctl`, the only tier that covers the manifest,
    runfiles and the socket. Keep them few; they are the slowest and the
    only ones that can be timing-sensitive.
  - `bazel run` itself as the reload trigger is untested, since Bazel
    doesn't run inside `bazel test`. After changing the rules or `modctl`,
    run `./bazel run //game` and `./bazel run //mods/<name>` by hand and say
    that you did.
- **A reload test needs two different builds.** Load `counter_v1`, then
  `counter_v2`, and assert on which one's code ran. Reloading the same build
  passes even when reloading does nothing, because `dlopen` hands back the
  image it already has.
- **Green has to be earned.** For each new test, make the edit that should
  break it and watch it fail; for a change to tested code, do the same for
  the tests that claim to cover it. Each tier was checked this way when it
  was written, and every surviving break exposed a test that couldn't see
  what it claimed to (a panicking mod that panicked on every step looked the
  same disabled or not). Check a new e2e test with `--runs_per_test=50`
  before trusting it.
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
    an assertion about behavior, and the integration tests are where it
    gets pinned. Prose naming an identifier
    is a reference nothing checks, so re-grep after a rename.

## When you learn something non-obvious

If you hit a surprising `dlopen` behavior, a Bazel or toolchain quirk, or
find out why an approach doesn't work, add an entry to
[docs/lore/](docs/lore/) before the session ends. Measure the claim first
where you can: a lore entry teaches with the authority of experience, so a
wrong one does more harm than a missing one.

<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:1105d646 -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/core-concepts/sync-concepts.md for details and anti-patterns.

## Agent Context Profiles

The managed Beads block is task-tracking guidance, not permission to override repository, user, or orchestrator instructions.

- **Conservative (default)**: Use `bd` for task tracking. Do not run git commits, git pushes, or Dolt remote sync unless explicitly asked. At handoff, report changed files, validation, and suggested next commands.
- **Minimal**: Keep tool instruction files as pointers to `bd prime`; use the same conservative git policy unless active instructions say otherwise.
- **Team-maintainer**: Only when the repository explicitly opts in, agents may close beads, run quality gates, commit, and push as part of session close. A current "do not commit" or "do not push" instruction still wins.

## Session Completion

This protocol applies when ending a Beads implementation workflow. It is subordinate to explicit user, repository, and orchestrator instructions.

1. **File issues for remaining work** - Create beads for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **Handle git/sync by active profile**:
   ```bash
   # Conservative/minimal/default: report status and proposed commands; wait for approval.
   git status

   # Team-maintainer opt-in only, unless current instructions forbid it:
   git pull --rebase
   git push
   git status
   ```
5. **Hand off** - Summarize changes, validation, issue status, and any blocked sync/commit/push step

**Critical rules:**
- Explicit user or orchestrator instructions override this Beads block.
- Do not commit or push without clear authority from the active profile or the current user request.
- If a required sync or push is blocked, stop and report the exact command and error.
<!-- END BEADS INTEGRATION -->
