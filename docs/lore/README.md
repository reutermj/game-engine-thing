# Lore

Non-trivial discoveries: things that took real effort to figure out and
aren't obvious from reading the code or the architecture docs. Tribal
knowledge that would otherwise live in someone's head, or be rediscovered
painfully by the next person (human or agent) who hits the same thing.

## What belongs here

- A `dlopen`, linker or loader behavior that was surprising or
  under-documented.
- A Bazel rule or toolchain quirk that cost time to track down.
- Why a previously tried approach was abandoned, and what specifically went
  wrong with it.
- Any "if you don't know this, you will waste an afternoon" fact.

## What doesn't belong here

- Current design and the reasons for it: that's
  [docs/architecture/](../architecture/).
- A repeatable maintenance procedure: that's a [runbook](../runbooks/).
- Anything easily re-derived by reading the current code.

## Format

One file per discovery, named for the finding as a sentence
(`dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md`), so the
directory listing reads as a list of facts. Keep entries short: what you
hit, why it's surprising, and what the resolution was. Say how the claim was
established (measured, read in source, or inferred), and put the
measurement in the entry, since a lesson that is later reversed goes on
teaching the old rule. When a code change overturns an entry, fix or delete
the entry in the same commit; when it only renames what the entry names,
add a dated *(History: ...)* note where the old name would mislead.

**A dated entry names targets and files as they were that day.** The
measurement was made on that code, so the names stay. The changes that
most often leave an entry's names behind:

- `//engine/std/physics` became `//engine/std/physics2d`, its `compare`
  subpackage with it, and `physics_test` became `physics2d_test` (fd2104c,
  2026-09-29).
- `//bench/physics3d` became `//engine/std/physics3d/compare` (6058d91,
  2026-09-29).
- `solver::solve_across`, `lanes::run_across` and the kept pool
  `tests/pool.rs` were removed (ed0b68d, 2026-10-03): `Passes` runs across
  the `threads` mod's pool through `engine/ecs/dispatch.rs`
  ([threads.md](../architecture/threads.md)).

## Where to look

The listing is the index; these are its rough areas, with an entry to
start from in each.

- **Loading and unloading builds** (`dlopen`, unmapping, what pins a
  library, pointers into an unloaded build):
  [dlopen-returns-the-loaded-image-for-a-file-it-has-seen](dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md),
  [dlclose-unmaps-a-mod-nothing-holds](dlclose-unmaps-a-mod-nothing-holds.md),
  [a-thread-local-a-mod-touches-keeps-its-build-mapped](a-thread-local-a-mod-touches-keeps-its-build-mapped.md).
- **Bazel and the toolchain** (rules_rs, rules_rust, clippy, rustfmt,
  sanitizers, fuzzing, C/C++ flags):
  [rust-shared-libraries-link-the-cxx-runtime-dynamically](rust-shared-libraries-link-the-cxx-runtime-dynamically.md),
  [any-clippy-flag-turns-off-rules-rusts-deny-warnings](any-clippy-flag-turns-off-rules-rusts-deny-warnings.md),
  [the-stable-rustc-sanitizes-with-rustc-bootstrap](the-stable-rustc-sanitizes-with-rustc-bootstrap.md).
- **Codegen and the ECS's costs** (inlining, vectorizing, a row's cost):
  [a-query-row-cost-its-dispatch-not-its-data](a-query-row-cost-its-dispatch-not-its-data.md),
  [array-lanes-vectorize-only-where-a-store-to-them-seeds-llvm](array-lanes-vectorize-only-where-a-store-to-them-seeds-llvm.md).
- **Threads and the CPU** (CCDs, clocks, pinning, claiming work):
  [the-scheduler-spreads-a-pool-over-both-ccds-and-a-colored-solve-halves](the-scheduler-spreads-a-pool-over-both-ccds-and-a-colored-solve-halves.md),
  [idle-cores-run-a-parallel-solve-at-half-speed](idle-cores-run-a-parallel-solve-at-half-speed.md).
- **Physics behaviour** (soft contacts, piles that won't settle, how noisy
  a measure is), the largest area:
  [a-soft-contact-sinks-by-its-load-and-only-substeps-make-it-stiffer](a-soft-contact-sinks-by-its-load-and-only-substeps-make-it-stiffer.md),
  [a-piles-step-to-rest-moves-by-hundreds-with-rounding-alone](a-piles-step-to-rest-moves-by-hundreds-with-rounding-alone.md).
- **The reference engines** (Box2D, Rapier, Jolt, Box3D) and what they do
  by default: [jolt-damps-every-body-by-default](jolt-damps-every-body-by-default.md),
  [rapier-has-no-restitution-threshold](rapier-has-no-restitution-threshold.md).
