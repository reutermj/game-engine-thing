# Design and consistency review (2026-10-04)

A review of the codebase and docs at `2f86a49`, after a week that built
flows, parallel shapes, the task-graph dispatcher and the resident `threads`
pool, removed `Workers`, extracted `physics_common` and gave 3D its lanes and
colored solve. It is a review, not a refactor: no code was changed. Trivial
doc-only fixes went in one commit (listed at the end); everything else is a
finding here, and the non-trivial ones are beads.

Line numbers are as of `2f86a49`. Excluded, because another agent was
changing them at the same time: `World::executor`'s visibility (get-znt.47)
and the 3D restitution test.

**Counts:** 15 wrong, 14 inconsistent, 15 untidy and 14 design
observations. Many entries group several lines: about 180 individual
findings, all checked by grep or by reading the code.

## The ten that matter most

1. **The spike rule isn't holding for physics2d's experiment binaries.**
   They still build their own unsafe thread pool and a frozen solver.
   (U1, get-emj.112)
2. **Shapes are declared, but nothing reads the declaration.** "Declared and
   run by the scheduler" is true in name only. (W1, get-znt.50)
3. **`engine_api` re-exports `Scoped`, `near_pairs` and all of `engine_ecs`.**
   A mod can fan out, or keep a hidden pair cache, with nothing declared.
   (W2, get-znt.51)
4. **`Crossing` covers `&'static T`.** A service can return a pointer into
   its own image that outlives a reload. (W3, get-y5t.9)
5. **The comparisons time our engine on the shared pool while saying every
   engine runs on one thread.** (W4, get-emj.113)
6. **physics.md is a 5555-line log.** Its thread sections still call the pool
   undecided. (W5 and D1, get-emj.114)
7. **The parity table has fallen behind three of the last four physics
   changes**, despite the rule that each change updates it. (W6, get-emj.115)
8. **The unsafe core is wider than CLAUDE.md says.** Spatial and ordered glue
   are unsafe. (I1, get-m3q)
9. **The parallelism docs describe the week before.** scheduling.md,
   overview.md, parallel-relations.md and parts of flows.md still put
   `Workers` or "one thread" in the present. (W7, get-cy2)
10. **Chunking is done three or four ways** across `par.rs`, `dispatch.rs`
    and both physics mods. (I2, get-znt.52 and get-emj.110)

## Wrong: docs or code that would mislead

**W1. The scheduler never sees a shape.**
- Shapes are said to tell the scheduler that a node fans out, and stage 3
  is said to size its work by them:
  - `docs/architecture/flows.md:420-426`
  - `scheduling.md:76-78`
  - `engine/ecs/query.rs:817-818`
  - `engine/ecs/shape.rs:2-3`
- In the code, no non-test code matches a `ShapeKind`:
  - `graph.rs:195` skips it.
  - `FramePlan`/`PlannedNode` (`engine/api/scheduler.rs:39-51`) carry only an
    id and a name.
  - A shape runs because the system calls `passes.run`, which dispatches
    onto the world's executor from inside the node.
- *Fix:* decide whether get-znt.5 makes the declaration matter, or say that
  the ECS runs shapes. **get-znt.50**

**W2. Mods can fan out, or keep hidden caches, outside any declaration.**
- `engine/api/lib.rs:34-41` has `pub use engine_ecs`, plus `Scoped` and
  `near_pairs`. `Scoped(n).run(..)` spawns threads with nothing declared.
- That contradicts `threads.md:40-41` and `par.rs:7-8`, and invites the
  TLS-key and never-unmapped problems in `docs/lore/a-mod-that-spawns-*`.
- `engine_ecs::live::LivePairs` (`live.rs:604`) is `pub` and `Default`: the
  very hidden cache live.md argues against.
- *Fix:* narrow the re-exports, and make `LivePairs`, `near_pairs` and
  `harness` crate- or test-only. **get-znt.51**

**W3. `Crossing` holds for every `&T` (`engine/ecs/component.rs:192-193`),
including `&'static T`.**
- A service returning `&'static str` or `&'static dyn Fn` into the
  provider's library passes the check (`engine/api/service.rs:198-201,
  247-254`), and the caller can keep it past a provider reload.
- `mod-deps.md:162-166` repeats "a borrow can't outlive the call".
- *Fix:* a separate bound for return position, plus a reload test.
  **get-y5t.9**

**W4. The comparison harnesses run ours on the shared pool.**
- `physics2d/compare/ecs.rs:72` and `physics3d/compare/ours.rs:51` install
  `engine_threads::shared()`. That is one CCD's worth of threads unless
  `ENGINE_THREADS=1` (`engine/std/threads/pool.rs:164-172`).
- Their comments say "one thread each" or "all single-threaded":
  - `physics2d/compare/BUILD.bazel:7`
  - `physics3d/compare/BUILD.bazel:2`
  - `physics3d/compare/lib.rs:2`
  - `physics3d/compare/ours.rs:2`
- Quality is unaffected, since results are the same bits at any thread
  count. Timing tables may not compare like with like.
- Runbook 005:45 still prescribes `taskset`, which the pool's own pinning
  makes wrong. `physics2d/compare/step_bench.rs:10` and `:20` contradict
  each other.
- *Fix:* decide the intent, then set one thread or label the tables.
  **get-emj.113**

**W5. physics.md's thread sections predate the `threads` mod.**
- Several places still call the pool, its placement and rayon "get-znt.5's
  decision": `docs/architecture/physics.md:781-805`, `:807-820`,
  `:1314-1340` and `:4498-4504` (the last inside a "Status: built"
  section). threads.md decided all three.
- The benches are said to use `tests/pool.rs` (`:688`, `:801`, `:4664`),
  which was deleted at `ed0b68d`, with knobs `STICKY`, `SPIN_US` and
  `POOL=late` (`:767`, `:4619`) that exist nowhere.
- Other stale passages:
  - `:773-779`: "three systems" (there are eleven).
  - `:2432-2435`: "nothing shared but storage" (`physics_common` now exists).
  - `:310-344`: a spatial-query example using `Circle::new` and
    `overlapping_point`, neither of which exists.
  - `:5287-5289`: a walker index "still to do", which no longer applies.
  - `:3425-3440`: says "four things wrong"; there are three.
- *Fix:* point these passages to threads.md and correct the rest.
  **get-emj.114**

**W6. Parity table rows don't match the code**
(`docs/architecture/physics-sharing.md`).
- `:54` and `:66` say 3D's broadphase and solve run on "one thread"; both
  now run across the pool.
- `:66` names `solve_across` as 2D's reference; it was removed by
  get-emj.93.
- `:67` omits that 2D's gathers, scatters and gravity are `ParMap` walks,
  while 3D's are serial.
- `:77` says nothing reloads 2D's turning, threaded path, but
  `physics2d/tests/physics2d_test.rs:1323` does.
- The test counts in `:78` have drifted.
- `:251-253`, `:293` and `:410` describe a `sed`-patching colors spike that
  is gone.
- `:419-420` treats 3D lanes and threads as future work; both are built.
- CREDITS.md is stale in the same way: `:74-75` (60 against 75 Hz),
  `:162-164` (levels against the colored default), and `:273`, `:313` and
  `:350` (the references are called "locked", but the shims rotate).
- *Fix:* update the rows, and consider a check. **get-emj.115**

**W7. The parallelism docs describe the week before.**
- `scheduling.md:13-15` says "everything here runs on one thread".
  `:260-261` has the scheduler owning the workers, which `threads.md:76-86`
  rejects.
- `overview.md:74-78` calls threading undecided.
- `parallel-relations.md:3` says "proposed, nothing built", and `:137-151`,
  `:449` and `:467` present `Workers` and `lanes::run_across` as current.
- `flows.md:11-13` describes physics3d as "six systems over three flows… no
  shape"; it is now eight over four, with `Passes`. `:713-715` says "3D
  never split its step", and `:772-774` puts world-storage shapes out of
  scope, though `par_for_each*` take a `ParMap`.
- *Fix:* rewrite against threads.md (judgment calls, not done here).
  **get-cy2**

**W8. storage.md is out of date beyond the borrowing question.** That
question is get-00a, already open.
- The walkthrough (`:123-153`) uses APIs that don't exist: `Single`,
  `Query::iter`, and systems without the `Transient` argument.
- `:73`, `:171` and `:254` point to `cx.commands()` and exclusive systems,
  which don't exist.
- `:645` lists "events as publish nodes" as not done; it is done.
- `:3-6` and `:215` call data parallelism future work.
- `:497` lists page size per table as an open question; it already is per
  table.
- `:503` says migration happens "on first access"; it happens at commit.
- *Fix:* refresh the doc. **get-k6b**

**W9. hot-reload.md describes the ABI and reload sequence as they were.**
- `:19-26` says everything crossing the boundary is `#[repr(C)]` and "the
  ABI is C". That contradicts CLAUDE.md and `mod-deps.md:149-151`: only the
  load-time tables are `repr(C)`.
- `:170-176` leaves the install step (migration, flow bins dropped) out of
  the sequence (`engine/loader/engine.rs:302-314`).
- `:44` lists "registered resources", which don't exist.
- `mod-deps.md:94-95` says migration happens "on first access"; it happens
  at commit.
- *Fix:* restate these against the code. **get-bxd**

**W10. Lore entries the code has overturned.**
- `docs/lore/a-mods-own-std-leaks-its-stdout-buffer-when-unloaded.md:36`
  says "Not fixed". It was fixed: `between.rs:18-25` and
  `//engine:mod_lints`.
- `a-stage-loop-without-a-main-thread-must-let-any-thread-take-any-block.md:4-21`
  names `solve_across`, `lanes::run_across` and a removed test.
- `memory-a-task-allocates-is-its-threads.md:36` names `near_pairs_with`,
  which doesn't exist.
- `a-cc-library-under-c-opt-is-o2-not-a-cmake-release-o3.md:22` names
  `bench/physics3d/copts.bzl`, which has moved.
- The lore README requires an entry to be fixed in the commit that
  overturns it. **get-4k7**

**W11. ECS docs with stale examples and status lines.**
- `ecs.md:18-21`: examples that are gone.
- `spatial-storage.md:484-489`: "In 3D: a spike, translation-only".
- `relationships.md:220-223`: get-emj.19 presented as open; it is fixed.
- `working-sets.md:369-375`: says physics hasn't adopted flows; it has.
- `working-sets.md:31-33` and `:377-383`: propose a shared `Slots` that
  already lives in `physics_common`.
- `live.md:3-7` and `:388`: get-pmk cited as both done and open.
- **get-87r**

**W12. `physics3d/solver.rs:1046` says 2D's `shareable` "is the same rule".
It isn't.**
- 2D rejects only `-0.0` and keeps the lanes serial (`physics2d/solver.rs:1974`,
  `pipeline.rs:542`).
- 3D also rejects non-finite values and falls back to `in_order`
  (`physics3d/solver.rs:1049`, `:1393`).
- **get-emj.116**

**W13. Code comments naming things that no longer exist or work
differently.**
- `engine/api/lib.rs:453-455`: `Bootstrap::run` says `cx.step_mods()`; it
  is `run_frame`.
- `engine/loader/engine.rs:796-800`: `run_node`'s doc sits on `fn steps`.
- `engine.rs:397-398`: says "without running any mod code", but
  `Mod::systems` runs.
- `engine/tests/probe.rs:2`: names `WorldStorage`.
- `engine/control/lib.rs:8-15`: the module doc omits `schedule`.
- `engine/modctl/main.rs:9-19`: the usage text omits `send`.
- `platformer/core/components.rs:21-23`: says walkers meet the player by
  spatial query; it is an `Overlap` sensor.
- `game/BUILD.bazel:3-5`: says physics reads the clock.
- `physics_common/levels.rs:4-5`: calls levels 3D's default order; colors
  are.
- `physics2d/compare/behaviour_test.rs:361`: says get-emj.57 "jitters for
  ever"; it now rests late.
- **get-bxd, get-ijd, get-emj.116**

**W14. CLAUDE.md gaps.**
- The test tiers omit `//engine/tools:mod_links_test`,
  `//engine/tests:poison_test` and the games' reload replays.
- The `defs.bzl` entry omits `twin`, plus engine_game's `scheduler` and
  `threads`.
- The `physics_common` entry omits `levels` and its dependency on
  `//engine/ecs`.
- The beads block calls `.beads/issues.jsonl` a passive export, but no
  checkout has one.
- **get-fi1**

**W15. Stale statements fixed directly** (see "Fixed directly" below):
- The status lines for physics-sharing.md and flows-spike.md.
- storage.md's "refusal" (the code panics) and `schedule_test`.
- hot-reload.md's step number.
- mod-deps.md's error text.
- Runbook 001's members.
- flows.md's `Passes` kernel signature.
- The rotation cap: "a quarter turn" where the code says π/4.
- physics3d's "parallelism left out".
- README's `engine/std` list and the `libfoo.so` name.
- overview.md's mod list.
- ecs.md's `FieldType` list.

## Inconsistent: the same thing done or said two ways

**I1. What the unsafe core is.**
- CLAUDE.md names `erased.rs` and `schema.rs`. `storage.md:319-323` says
  "that one module", and `:399-400` says "the only crate with unsafe code".
- In fact there is also unsafe glue in:
  - `spatial.rs:117-155`, `:253-268` and `:933`;
  - `ordered.rs:27-45` and `:175`;
  - `events.rs:27`;
  - `engine/api`'s FFI.
- *Fix:* move the glue beside the core, or name it in CLAUDE.md and in the
  long-check triggers. **get-m3q**

**I2. Chunking is done three or four ways.**
- `engine/ecs/par.rs:91-135`: `Split::chunks`, `even`, `CHUNKS_PER_THREAD`.
- `dispatch.rs:225-230`: `blocks_of` with a literal `4`, and `block_range`.
- `ParMap` cuts with `blocks_of`, while `Query::par_for_each*` on the same
  `ParMap` cut with `Split::chunks`.
- The physics mods' identical `chunks` are at `physics2d/lib.rs:136` and
  `physics3d/lib.rs:314`.
- `Split::map_each` and `shape::map_across` do the same job.
- **get-znt.52**, then get-emj.110.

**I3. 2D and 3D name the same concepts differently**, and handle
no-moving-body contacts differently with no parity row.
- `Wide` against `Order`+`Lanes`; `Params::of` against `Tuning::of`;
  `staged` against `Staged`; `Warm::None` against `Warm::Cold`; carry codes
  in a different order; `Physics` against `Physics3d`.
- Contacts with no moving body: 2D solves them last as `UNSOLVED`
  (`physics2d/solver.rs:1247`); 3D folds them into group 0
  (`physics3d/solver.rs:1073-1081`).
- **get-emj.116**

**I4. Where a service may reach the world.**
- `storage.md:226-233` says a service has no world access.
- `mod-deps.md:168-176` says it may between frames, and
  `engine/tests/mods/marking_scheduler.rs:29` does.
- get-znt.16 (open) covers the system-to-service case; this needs the same
  decision. **get-bxd**

**I5. The cost of shared over plain on one thread is quoted four ways:**
- `flows-spike.md:423`: 6 to 14%;
- `flows.md:519-521`: 6, 7 and 24%;
- `dispatch-spike.md:286` and `:520`: 8 to 18%;
- `threads.md:250`: 8 to 18%.

*Fix:* give it one home. **get-cy2**

**I6. Same rationale in two homes.**
- The `thread_local!` question is open in `scheduling.md:286-291` and
  answered in threads.md "Hot reload".
- The pipeline diagram has three drifting copies: `physics.md:225-238` and
  `:2319-2331`, `flows.md:601-611`, and the `pipeline.rs` headers.
- `physics.md:4853-4863` and `:5139-5146` copy `threads.md`'s numbers
  digit for digit.
- **get-cy2, get-emj.114**

**I7. Two parallel executors.**
- `harness::run_parallel` (`engine/ecs/harness.rs:285-330`) spawns scoped
  threads per frame and shares nothing with `Executor` and `dispatch`.
- Yet it is the only proof that parallel frames equal sequential ones.
- **get-znt.53**

**I8. Code that splits stages the docs measured as no gain.**
- 2D's gravity (`physics2d/lib.rs:258`) and the colliders' gathers
  (`:420-441`) run `par_for_each`.
- `physics.md:721-732` argues against splitting them, `threads.md:493` and
  `:542` measured no gain, and no comment says which way was chosen.
- Tracked by get-emj.106; no new bead.

**I9. The Rapier licence travels two ways, and transitive crates have no
notices.**
- 2D uses a checked-in copy; 3D fetches it by sha256.
- The pins differ: `=0.36.0` against `0.36`.
- Rapier's transitive crates (parry, nalgebra, …) have no licence texts in
  the runfiles, against CREDITS.md:17-21.
- Runbook 005 doesn't cover bumping Jolt, Box3D or rapier3d.
- **get-emj.118**

**I10. Retrospectives say they are "not updated afterward"**
(`docs/retrospectives/README.md`).
- But pong (`:64-77`), the platformer (`:72-78`) and physics have in-place
  addenda.
- One of them names `cx.commands()`, which doesn't exist (platformer
  `:72-73`).
- *Fix:* allow same-day addenda in the README, or stop adding them
  (trivial; not done, as it is a policy call).

**I11. Games' unload and tuning conventions differ.**
- `mods/spawner` and `platformer/level` clean up in `close`. `pong/core` and
  `platformer/core` don't.
- The platformer's tuning constants live in its interface
  (`platformer/core/components.rs`), the opposite of physics's `Tuning`
  components. The platformer retrospective's recommendation was dropped.
- **get-ijd**

**I12. The manifest env var is spelled two ways:** `PONG_MANIFEST` against
`MANIFEST` (`pong/BUILD.bazel:25`, `platformer/BUILD.bazel:24`).
**get-ijd**

**I13. Where the equivalence tests and step benches live differs by
dimension.**
- 2D's bit-for-bit tests are in the slow `compare:quality_test`; 3D's are
  in `physics3d:exact_test`.
- 2D's `step_bench` is in `compare/`; 3D's is in `tests/`, with different
  thread knobs (`THREADS` against `ENGINE_THREADS`).
- *Fix:* align them when next touched. **get-emj.113** (thread knobs)

**I14. The status of done work is spread across docs that disagree.**
- `threads.md`'s status omitted get-znt.45 (fixed).
- `flows-spike.md` had no "built as" pointer (fixed).
- `physics-sharing.md:364-373` still offers Phase 0 "for the user to add to
  CLAUDE.md", though it is there and get-emj.84 is closed. **get-emj.115**

## Untidy: leftovers and comments

**U1. The spike rule.** These binaries answer questions that are already
decided, aren't marked `# SPIKE`, and still build. Each should be retired
with a footnote naming its last commit, or declared a maintained bench.
- `//engine/std/physics2d:parallel_solver` (`tests/parallel_solver.rs`, 1849
  lines) has its own pool, with `unsafe impl Sync`, a transmuted borrowed
  closure and `sched_setaffinity` FFI (`:80-280`). It duplicates `engine_threads`.
- `:solver_layout` (1348 lines, raw-pointer layouts).
- `:narrow_bench`.
- `physics.md:1737-1740` deferred porting them "for when the soft step is
  parallelized". It now is, and the rule says not to port.
- **get-emj.112**

**U2. More benches that keep decided comparisons building.**
- ECS: `//engine/ecs:spatial_turn_bench`, the z=0 arm of `spatial3d_bench`,
  and `//engine/ecs:bench` ("the spike's, ported", `storage.md:516`).
- `(SPIKE, get-znt.18.)` tags remain in landed code: `world.rs:406`, and
  `query.rs:892`, `:1003` and `:1712`.
- **get-znt.49, get-87r**

**U3. About 40 `#[allow(dead_code)]` in physics2d have no reason.**
- For example `physics2d/solver.rs:81`, `94`, `141`, `173`, `270`, `279`,
  `426`, `434`, `443`; `step_bench.rs:31-49`; `solver_bench.rs:19-36`.
- All are caused by `#[path]` sharing (D5).
- `SolverBody::displacement(&self, _dt)` (`solver.rs:236`) takes `dt` only
  to match the retired `split_impulse.rs:43`.
- **get-emj.117**

**U4. Removed identifiers in live comments with no History mark.**
- `engine/ecs/shape.rs:445` and `physics2d/solver.rs:2307` name
  `run_across`.
- `live.md:84` and `:349` name `pairs_with(&workers)`.
- **get-87r, get-cy2**

**U5. Dead ABI field.** `ModInfo::state_drop` (`engine/api/lib.rs:132`,
`:423`) is filled in but never read by the loader. **get-y5t.11**

**U6. A silent failure path.** `engine/loader/engine.rs:310-313` only
`eprintln!`s an `install_all` error, while every other `load_batch` failure
returns `Err`. **get-y5t.11**

**U7. Small duplications in the loader.**
- `close_state` against direct `call(Op::CLOSE)`: `engine.rs:284`, `:293`
  and `:1013`.
- Two unrelated `check_plan`s: `engine.rs:1078` and `flows::check_plan`.
- **get-y5t.11**

**U8. A checkable claim with no test.**
- "`unload` and `close` can't make calls (`Unavailable`)"
  (`mod-deps.md:193-194`).
- Nested or concurrent dispatch (a kernel dispatching again) is neither
  refused nor tested.
- **get-y5t.12, get-znt.53**

**U9. The engine test library is coupled to physics2d.**
- `engine/tests/replay.rs:28-31` and `:259` hard-code physics2d's
  components.
- `replay.rs:53-56` and `pong/BUILD.bazel:43-44` justify early stops by
  "debug build" reload costs, but every build is `-c opt`.
- **get-5qa**

**U10. Lore names targets as they were.**
- About 20 dated entries name `//bench/physics3d`,
  `//engine/std/physics:tax`, `/compare` or `physics_test`.
- The `tests/pool.rs` measurements in
  `the-scheduler-spreads-a-pool-over-both-ccds-*` and
  `relaxed-atomic-floats-*` lack a History line.
- **get-4k7**

**U11. Smaller game leftovers.**
- `platformer/level/lib.rs:141` duplicates core's private `start()`.
- Level's and spawner's `close` scan every `ChildOf` instead of using
  `in_keys`.
- `pong/text/lib.rs:97-98` declares an empty `systems()`.
- **get-ijd**

**U12. Stale wording in physics-testing.md and the compare tools.**
- physics-testing.md: `:38` and `:154` say "four layers" (the table has
  nine); `:643` gives a fastbuild budget.
- `physics3d/compare/main.rs:4-8` documents `--runs N`, but only `--runs=N`
  parses.
- **get-emj.118**

**U13. Leftovers in physics.md.**
- An unreferenced 80-line `[^sleep-copy]` footnote (`:5389-5467`).
- Two footnotes mid-doc (`:1757-1772`).
- Fastbuild timings at `:3128` and `:3263`.
- `./bazel run -c opt` at `:1345`, `:2163` and `:5171`.
- The colored SIMD solve isn't marked done at `:3066-3072`.
- **get-emj.114**

**U14. Small comment errors.**
- `par.rs:130-131`: "about 150 rows" a chunk assumes 16 threads; the
  default is 8.
- `component.rs:270-273`: the comment sits above the wrong impls.
- `ops.rs:2-7`: "one driver", then "two drivers".
- `query_bench.rs:1-2`: says `-c opt`; it should be `--config=bench`.
- **get-znt.52, get-87r**

**U15. `engine/ecs/par.rs` is `pub mod`** (`lib.rs:24`), though everything
but `Executor` and `Scoped` is crate-private. *Fix:* `mod par;` plus the
existing re-exports. **get-znt.51**

## Design observations

Described, not decided.

**D1. physics.md has outgrown a living doc.**
- 5555 lines, ordered by date, with about 15 inner Status lines, "built
  since" asides and history in prose.
- A 3D design scattered through it ("3D, translation only (spike)" still
  calls physics3d plain harness systems).
- Rationale owned by threads.md and flows.md, copied in.
- A possible shape: a short current physics.md, physics3d's own doc, and a
  measurements and history doc, with thread material left to threads.md.
- **get-emj.114**

**D2. Does the scheduler own parallel work, or the ECS?**
- get-znt.28's rule says the scheduler runs declared shapes.
- Today the ECS dispatches them from inside the node, and the `sequential`
  scheduler never sees them (W1).
- get-znt.5 is where this gets decided. Until then the docs overstate it.
- **get-znt.50**

**D3. The loader is less bare than the invariant.**
- Fixed-rate accumulation and `MAX_STEPS` (`engine/loader/engine.rs:61-92`,
  `:729-742`).
- A built-in sequential frame duplicating `//engine/std/sequential`
  (`:801-817`).
- A `while engine.pump(..)` loop in `main.rs:64-66`, beside "the loader has
  no loop".
- `Engine::step_all` bypasses any loaded scheduler, so loader tests never
  run one.
- **get-f1a**

**D4. The public surface of `engine_ecs`, through `engine_api`, is the
whole crate.**
- `World`, `harness`, `LivePairs` and `Scoped` are all reachable from a mod
  (W2).
- The design says declarations are the only door. A deliberate split
  between a mod-facing prelude and the loader's API would make that
  checkable.
- **get-znt.51**

**D5. physics2d shares sources by `#[path]` into about ten binaries.**
- That causes the `dead_code` allowances (U3) and API shaped by retired
  copies.
- physics3d mostly avoids it.
- A `physics2d_core` library, or fewer path-sharing binaries, would fix it,
  but the constraint recorded at `physics.md:5240-5245` must be re-checked
  first.
- **get-emj.117**

**D6. Interface-safe common types.**
- 3D's interface redefines `Closing` (`physics3d/components.rs:357-372`) and
  maps it onto `physics_common::Closing` (`solver.rs:468-473`), because
  interfaces can't depend on `physics_common`.
- Warm-start matching (get-emj.91) has the same constraint.
- Worth deciding whether small value enums belong in an interface-safe
  crate. No bead: the cost is small today.

**D7. Unused shapes kept on speculation.**
- `ParMap::map_into` and `Reduce` have no caller outside tests
  (`threads.md:292-297` says so).
- Not spikes, but the same cost the spike rule guards against.
- Keep or drop when get-znt.5 lands.

**D8. Nested dispatch** (U8). The dispatch's spin protocol inside rayon's
`in_place_scope` has never been exercised re-entrantly. **get-znt.53**

**D9. The parity rule isn't self-enforcing.**
- Three of the last four physics changes missed the table (W6).
- A check (for example, a test that the table names every `Tuning` field
  of both mods), or a review step, would hold it.
- **get-emj.115**

**D10. No retrospective covers the parallelism week.**
- The latest is 2026-09-27; the week since covered about 90 commits,
  including the whole thread and flow design.
- CLAUDE.md asks to read the latest before redesigning, and get-znt.5 is
  next.
- **get-yzc**

**D11. The test library and games encode physics2d.**
- `engine/tests/replay.rs` belongs to the engine tier but knows one std mod
  (U9).
- When a renderer or a second physics user arrives, the coupling will
  matter. **get-5qa**

**D12. Tuning as data or as interface constants** (I11): physics chose
data, and the games chose constants. A convention in mod-deps.md would
settle it. **get-ijd**

**D13. Service calls are resolved by name alone**
(`engine/loader/engine.rs:1253-1285`).
- The interface-check guarantee in `service.rs:33-35` holds only if the
  caller lists the provider in its `mod_deps`.
- **get-y5t.10**

**D14. Lore has no index.**
- The README promises one ("so the index reads as a list of facts"), and 71
  entries are about a third physics measurements.
- A short categorised index, or rewording "index" to "the directory
  listing", would do. Trivial; left to get-4k7.

## Beads

**Filed by this review:**

| bead | about | findings |
|---|---|---|
| get-emj.112 | retire physics2d's answered experiment binaries | U1 |
| get-znt.49 | ECS benches that keep decided comparisons building | U2 |
| get-znt.50 | shapes declared but unread | W1, D2 |
| get-znt.51 | `engine_api`'s surface: `Scoped`, `near_pairs`, `LivePairs` | W2, D4, U15 |
| get-y5t.9 | `Crossing` for `&'static T` | W3 |
| get-y5t.10 | services resolved without a dependency check | D13 |
| get-znt.52 | one way to cut work into blocks | I2, U14 |
| get-emj.113 | comparisons time ours on the pool | W4, I13 |
| get-emj.114 | physics.md restructure and stale thread sections | W5, D1, I6, U13 |
| get-k6b | storage.md refresh beyond get-00a | W8 |
| get-m3q | the unsafe core is wider than CLAUDE.md says | I1 |
| get-emj.115 | parity table and CREDITS drift | W6, I14, D9 |
| get-emj.116 | 2D/3D shareable rule and naming | W12, I3 |
| get-f1a | the loader's bareness | D3 |
| get-cy2 | parallelism docs reconciliation | W7, I5, I6, U4 |
| get-4k7 | lore overturned or renamed | W10, U10, D14 |
| get-bxd | hot-reload.md, mod-deps.md and loader comments | W9, W13, I4 |
| get-y5t.11 | loader leftovers: `state_drop`, `install_all` | U5, U6, U7 |
| get-y5t.12 | test that `unload` and `close` can't call | U8 |
| get-znt.53 | nested dispatch; harness's second executor | I7, U8, D8 |
| get-ijd | games: cleanup, tuning, small leftovers | I11, I12, U11, D12 |
| get-5qa | replay library coupled to physics2d | U9, D11 |
| get-emj.117 | `#[path]`-shared sources and `allow(dead_code)` | U3, D5 |
| get-emj.118 | runbook 005, licences, physics-testing wording | I9, U12 |
| get-yzc | a retrospective for the parallelism week | D10 |
| get-87r | ECS docs, examples and SPIKE tags | W11, U2, U4, U14 |
| get-fi1 | CLAUDE.md gaps | W14 |

**Already tracked, not re-filed:** get-00a (storage.md borrowing),
get-znt.16 (service from a system), get-emj.110 (physics `chunks`),
get-emj.106 (splits with no gain), get-emj.83 (2D threaded reload replay).

**Open beads that look done or overtaken** (listed only; none closed):

- **get-znt.6**, "Step 3: data parallelism (`par_for_each`)":
  `Query::par_for_each*` taking a `ParMap` are built (`query.rs:1447-1499`).
- **get-znt.22**, `Colored<R>` over a relation: `parallel-relations.md:27-30`
  calls the world-storage primitive likely superseded by `Colored` over
  flows.
- **get-znt.27**, "Flows spike: recycled allocations make prepare slower":
  the spike code is gone. Re-check on the built flows bins, or close.
- **get-emj.83**, a 2D threaded reload replay: partly done, since
  `physics2d_test.rs:1323` reloads under the pool. Only the every-frame
  replay remains.
- **get-y5t**, the epic: resident mods are built (`reload_test.rs:547`).
  Only the "registered resources" half remains (get-y5t.3, .4, .8).
- **get-pmk**: its several-relations-per-key part is built (`live.md:3-7`).
  What remains is "other kinds of relation".

The bead database was being updated while this review ran (several of the
candidates above were closed today), so re-check before acting.

## Fixed directly

In one commit (`8be70ab`), doc-only, each checked against the code:

- `docs/architecture/README.md`: physics-sharing.md "(proposed)" changed to
  "(accepted; phases 1 and 2 built)"; flows-spike.md now points to
  flows.md.
- `docs/architecture/storage.md`: a contended guard "panics", not "is
  reported as a refusal"; `schedule_test` changed to `graph_test`'s
  `readiness` tests.
- `docs/architecture/hot-reload.md`: `-z now` acts in step 3, not step 2.
- `docs/architecture/mod-deps.md`: the error text says `physics2d's`, as
  `engine.rs:519` prints it.
- `docs/runbooks/001-regenerate-cargo-lock.md`: "today only
  `engine/loader/Cargo.toml`" now points to the root's `members` (four
  crates).
- `docs/architecture/flows.md`: the `Passes::run` kernels take `first`
  (`shape.rs:382`).
- `docs/architecture/threads.md`: get-znt.45 added to the status line.
- `docs/architecture/physics-sharing.md`: 2D column header `physics2d`; 3D
  rotation cap π/4 (`MAX_ROTATION`); the harnesses share
  `physics_testkit`.
- `docs/CREDITS.md`: Box3D's rotation cap is π/4, not a quarter turn.
- `docs/architecture/physics-testing.md`: 3D's `behaviour_test` exists.
- `docs/architecture/physics.md`: 3D no longer leaves out parallelism.
- `docs/architecture/ecs.md`: `FieldType` includes tuples, `Instant` and
  `Duration`.
- `docs/architecture/overview.md`: "every other mod" now names physics and
  the games.
- `README.md`: `engine/std` lists `threads` and physics, and says lockstep
  is opt-in; modctl `send`; `lib<name>_mod.so`; the retrospectives
  description.

Nothing in the build reads these files (no BUILD or `.bzl` file references
a `.md`), so no test run was needed.
