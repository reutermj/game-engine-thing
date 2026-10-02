# Flows: spike results

**Status: spike results** (2026-10-02, get-znt.24). Nothing in the engine
changed: no `engine_ecs` code, no ABI, no scheduler. The spike is three
bench or test targets, all built on `engine_ecs`'s public API and its
harness:

- `//engine/ecs:flows_spike` (`engine/ecs/tests/flows.rs`): the mechanism
  and its parallel shapes, with unit tests (`:flows_test`);
- `//engine/std/physics2d/compare:flows_spike` (`flows_physics.rs`,
  `flows_lanes.rs`, `flows_spike.rs`): physics's solve ported onto it, bit
  for bit the solve as built, and timed. `:flows_spike_test` checks the bit
  for bit claims on small scenes in the default suite;
- `//engine/ecs:flows_hierarchy_spike`: transform propagation as the
  second user.[^spike-code]

This is a report, not the design doc. It says what was built, what it
measured, and what a design doc would have to settle. The design doc is
[flows.md](flows.md) (get-znt.25), which settles each question listed
under "Recommendation".

**In short:**

- **Flows held up.** Physics's solve runs as eight systems passing four
  flows, bit for bit the solve as built at one thread and at eight. At 8
  threads it costs what the solve as built costs: −4% to +1% on the three
  scenes. At one thread it's 4 to 7% slower, all of it the shared atomics
  the generic primitive always uses (below).
- **The mechanism is cheap.** A flow's use costs about 85 ns a system,
  apply node included. The pipeline of eight systems runs within noise of
  the same stages called in one system.
- **Edges need nothing new in the graph.** A flow's edge is an event
  queue: `See` reads it, everything else writes it. `graph.rs` already
  orders that exactly as a flow needs. What's missing is small: a
  declaration that says which use it is, and a home for the values in the
  world.
- **A generic colored primitive nearly matches the hand-tuned solver** if
  its kernel takes a block of batches. On the same atomics it is level at
  one thread (−1.0% to +1.6%). At 8 threads it is 2 to 8% slower than
  `lanes::solve_across`'s staged run (25 µs on the settled pile), nearly
  all of it the batch fill moved out of the run. A kernel per edge costs
  1.6 to 2.1 times as much, and all of that is the SIMD lanes lost: the
  call itself is free.
- **The second user isn't helped.** Hierarchy propagation as flows costs
  what any copy costs. That's 1.5 to 4.7 times the idiom a game writes
  today. Flows are for systems that already copy.
- **Recommendation:** proceed to a design doc, scoped to what the spike
  showed works: flows as a declared parameter kind, their values in the
  world, a plan check at load, and `Colored::passes` with a block kernel
  as the provided parallel shape. Open: whether the scheduler infers
  order from flows, which is the user's decision. How the parallel shapes
  are declared has since been decided (get-znt.28, 2026-10-02): parallel
  work is declared and run by the scheduler, never by a system.

## The question

The user's idea (2026-10-02): "It's systems, but instead of systems always
operating on data in the ECS world, they act more like map/reduce in
functional programming. They take data from some previous system, apply
the operation, and pass it to some other system. Systems at the beginning
and end act on the world in this way."

The physics solve is that already, inside one system: it gathers the
world into dense arrays, iterates on them, and scatters the results back.
Three spikes found that the copy is the solver's need, not a storage
workaround:

- working-sets.md;
- parallel-relations.md, (c);
- the contiguous-columns spike, unmerged: even contiguous world columns
  make an in-place solve 3 to 6% slower at one thread, and 17 to 40% at
  8.

So the question is whether the copy's stages can be systems the ECS sees.
They would hand each other values by declared edges. Parallel work would
go only through shapes the ECS provides, which answers what phase 2 of
parallel-relations.md was rejected for: "systems shouldn't take the pool
to run parallel work on private copies the ECS can't see."

## How others pass values between systems

Read 2026-10-02. Flecs 4.0.4 was read in source, from a Bazel output base
where it was fetched. Bevy was read as single files fetched from GitHub
`main` at `90942be` (0.20.0-dev). Unity Physics 1.5.0 was read from the
`needle-mirror/com.unity.physics` mirror. Timely Dataflow was read at
`9efd010`. Unity's Jobs and Entities were read only in their manuals,
since their C# source isn't public. TBB's flow graph was read in its
specification. Rx, iterator fusion, Halide and Taskflow weren't read.
Credits: docs/CREDITS.md.

| | edges | borrow or move | lifetime | who owns allocations | parallel shapes | one producer, many readers |
|---|---|---|---|---|---|---|
| **Bevy `pipe`** (`In`, `InRef`, `InMut`) | wired by hand: `a.pipe(b)`, types matched at compile time | `In<T>` moves; `InRef`/`InMut` borrow only from a caller of `run`, not across a pipe | one call: the two systems are one scheduled node, their accesses joined (`combinator.rs`, `PipeSystem::run_unsafe`, `initialize`) | the producer's return value, dropped by the consumer | none: A then B, serially, as one node | **no**: "system pipes cannot branch" |
| **Bevy `Local<T>`** | none: one system's own | `&mut` | across runs | the system's state, so allocations are reused | – | no |
| **Bevy messages** | by type, ordered by hand (`.before`/`.after`) | readers borrow, `drain` moves | double-buffered, about two updates | a resource; `Vec::clear` keeps capacity (inferred) | readers of one type run together | yes, a cursor per reader |
| **Flecs** pipelines, singletons, `ctx` | phases and entity order; merges inferred from terms read and written (`pipeline.c`, `flecs_pipeline_check_term`) | singletons in place; `ctx` a `void*` | singleton persistent; `ecs_run`'s `param` one call | the world (singletons), the user (`ctx`) | a multithreaded system's tables sliced across workers; commands queued per stage | through a singleton |
| **Unity Jobs** (`NativeContainer` and `JobHandle`) | wired by hand: a handle passed into `Schedule`, `CombineDependencies` | shared, many readers or one writer, checked by the safety system | by allocator: `Temp` a frame, `TempJob` four, `Persistent` | the creator `Dispose`s, or `[DeallocateOnJobCompletion]` | `IJobParallelFor` batches, with stealing | yes |
| **Unity Physics** systems | phase groups and a singleton; `state.Dependency` chained by hand | the singleton holds a pointer to one `Simulation`, each stage system edits it in turn | persistent, reset each step; per-step streams disposed by scheduled jobs | the system, kept and grown only (`PhysicsWorldData.cs`, `Simulation.cs`) | jobs; the solve in phases of a body each (`DispatchPairSequencer`, up to 64) | through the singleton |
| **Timely Dataflow** | wired by hand: operators chained on streams | moved; `Push::push(&mut Option<T>)` lets the receiver swap a buffer back | per batch, with frontiers | moved along, buffers handed back by swapping | workers and exchange | cloned for all but the last reader (`tee.rs`) |
| **TBB flow graph** | wired by hand (`make_edge`) | the body gets `const Input&`, output copied to successors | per message | copies | `concurrency` per node | broadcast |
| **this spike** | by type, declared as parameters; the graph orders them and a check refuses bad plans | `See` borrows; `Pass` edits in place; `Take` owns | one frame | the world keeps the last frame's emptied value for the next `Make` | `par_map`, `par_for_each_mut`, `reduce` in a fixed order, `Colored::passes` | yes: any number of `See` |

What it says:

- **Nobody has typed, frame-scoped values whose uses are scheduled
  edges.** Bevy's `pipe` is the nearest: a typed value moved from one
  system to the next. But the scheduler sees one node, and a pipe can't
  branch. Everyone else passes values through persistent world state
  (a resource, a singleton, a component holding a pointer), with order
  from phases, component access or job handles, not from the value.
- **Unity Physics is the closest to this spike's physics pipeline.** Its
  broadphase, narrowphase, Jacobians and solve are separate systems,
  handing one `Simulation` on through a singleton pointer and job handles
  wired by hand. Unity's docs say outright that the dependency system
  "doesn't track the dependencies that a job might have on data passed
  through a NativeArray": that is the gap a declared flow closes.
- **Kept allocations are everyone's**: Bevy's `Local`, Unity Physics's
  persistent buffers grown only, Timely's swap. Here they're the
  world's, by flow type.
- **Explicit wiring is the norm outside ECSs** (Timely, TBB, Unity's
  handles). Edges by type need no wiring, and fit an ECS where mods don't
  know each other. They also fit this repo, whose graph already orders
  by what a parameter declares.

## The mechanism as built

```rust
flow! {
    /// The awake bodies, dense, by an index the step makes.
    pub struct Bodies: "physics2d::flow::Bodies" {
        pub entities: Vec<Entity>,
        pub bodies: Vec<SolverBody>,
        pub kinds: Vec<(bool, u8)>,
        pub slots: Slots,
    }
}

fn gather_bodies(_: &mut Cx, dt: Dt, moving: Query<(&Body, &Velocity, &Position), Without<Asleep>>, mut out: Make<Bodies>) { .. }
fn gather_contacts(_: &mut Cx, contacts: Query<..>, bodies: See<Bodies>, mut out: Make<Contacts>) { .. }
fn scatter_bodies(_: &mut Cx, moving: Query<(&Body, &mut Velocity, &mut Position), ..>, bodies: Take<Bodies>, ..) { .. }
```

- **`flow!`** declares a struct, a marker event of the same name (its
  edge), and `Recycle`, which empties each field and keeps its
  allocations (`Vec::clear`; an `Option` back to `None`).
- **Four uses:**
  - `Make<T>` derefs to this frame's `T`, which is last frame's emptied
    if there is one;
  - `See<T>` borrows it, and any number of `See`s may run at once;
  - `Pass<T>` edits it in place and hands it on;
  - `Take<T>` owns it; dropped, it goes back emptied for the next `Make`,
    and `into_inner` keeps it instead.

  The sketch's "produce by returning it" became `Make`, because harness
  systems return nothing. A real `IntoSystem` could treat a returned
  flow as a `Make` with no other change.
- **Edges are event queues** (`declare` in `flows.rs`). `See` declares the
  flow's queue as a read; the others declare it as a write. `graph.rs`'s
  rule for events then gives exactly what a flow needs:
  - two `See`s run together;
  - anything else waits for every earlier use;
  - a reader also waits for a writer's apply node.

  Checked against the harness: `the_graph_orders_a_flows_uses_and_lets_its_readers_run_together`
  in `flows_test.rs`, where readers don't wait for each other and an
  editor waits for both.
- **Which use a parameter is** rides in its declaration as a count of `Dt`
  leaves, which touch nothing. That's a stand-in, read only by the plan
  check (`flows::accesses`). The real thing would be a
  `ParamDecl::Flow { slot, access }`.
- **The values live beside the world**, in a store found by the world's
  address and leaked for the process. The real thing would keep them in
  the world, as event queues are.
- **The plan check** (`flows::check`) walks the plan before it runs and
  refuses:
  - a flow made twice;
  - a flow seen, passed or taken before it's made;
  - a flow seen, passed or taken after it's taken.

  It notes a flow nothing reads. At run time the same rules are asserted
  at fetch, so a plan that skipped the check fails at the first bad use,
  naming the flow. A value from an earlier frame is never this frame's:
  a slot remembers the frame it was made in.
- **The parallel shapes** are the only way the spike's systems use the
  pool:
  - `par_map` and `par_for_each_mut` over a flow's items;
  - `reduce`, mapped a fixed-size chunk at a time and folded in the
    items' order, so a float sum is the same at any thread count;
  - `Coloring::greedy` and `Colored::passes` (below).

## Ownership: what happens when

Each behaviour was picked for a reason, and each is a test in
`flows_test.rs`:

| case | behaviour | why |
|---|---|---|
| two systems `Take` one flow | refused by the plan check ("Take by t2 after t1 took it") | the second would get nothing: a plan error, found at load like a cycle, not a frame that fails |
| nothing takes it | allowed, noted; the next frame's `Make` recycles it | a debug view or an optional reader `See`s it, and nobody should have to take it to keep the frame valid |
| a `See` after the `Take` | refused by the check | it would read nothing. A flow made again after a `Take` is a new value, and seeing that is fine |
| a `See` before the `Make` | refused by the check; asserted at fetch | the flow doesn't carry across frames. Unlike an event, a reader before the producer can't get last frame's: it's gone, or recycled |
| made twice in one frame | refused | two producers of one value is ambiguous. A stage that changes it is a `Pass` |
| one system `See`s and `Make`s one flow | refused by the existing `check_conflicts` (one system reads and writes one event queue) | nonsense either way |
| `Take` and `Make` of one type in one system | refused the same way | **so a hook that edits a flow is a `Pass`**, not "take it and return it" as sketched |

**Recycling is by type, and that shaped the pipeline.** A stage that
takes `Bodies` and makes a `Solved` from its vectors moves the allocation
into `Solved`'s bin. `Bodies`' bin then starts empty every frame, and its
`Make` allocates afresh. So the stages that change a flow use `Pass`.
Either recycling follows a value across types, or stages edit in place.
The spike did the second.

## Physics as a pipeline

```text
gather_bodies    world -> Make<Bodies>
gather_turning   See<Bodies>, world -> Make<Turning>
gather_contacts  See<Bodies>, world -> Make<Contacts>
prepare          See<Bodies>, See<Turning>, Pass<Contacts> -> Make<Graph>
solve            See<Turning>, Pass<Graph>
finish           Take<Graph> -> Pass<Bodies>, Pass<Turning>, Pass<Contacts>
scatter_bodies   Take<Bodies>, Take<Turning> -> world
scatter_contacts Take<Contacts> -> world
```

- **Sources and sinks** are lib.rs's `solve`, split into functions. The
  sources take read-only queries. A source over the write-typed queries
  the solve as built uses got an apply node that re-sorted the bodies'
  spatial tables for nothing, which an early run measured.
- **`prepare`** is `head`, `setup`'s placing and `run_across`'s first
  stage:
  - the bodies as states;
  - the contacts colored by `flows::Coloring::greedy` (Box2D's rule,
    asserted equal to `lanes::group` contact for contact);
  - each contact packed into its lane, and the batches filled with masses
    and points (`par_for_each_mut`).
- **`solve`** is the substeps as a `Colored::passes` program:
  - `Each(Gravity)`;
  - `Items(Warm)`, `Items(Push)`;
  - `Each(Move)`;
  - `Items(Relax)` twice;
  - then `Items(Bounce)`.
- **`finish`** writes the impulses and states back into the flows.
- **The debug view.** A `See` of three flows between `finish` and the
  sinks is what the comparison reads (`pipeline_seen`). It is also the
  kind of system another mod could add knowing nothing else.

**Bit for bit.** Every run of the bench checks it on each scene, and so
does `flows_spike_test` on a pyramid and a pile at several steps. The
checks are:

- each body's velocity, how far it moved, its spin, turn and angle;
- each contact's impulses and closing speed, and each point's impulses;
- every value the scatters write to the world (velocities, positions,
  rotations, spins, impulses, what was solved and pressed, the points'
  impulses), between the solve as built and the pipeline;

at 1 and 8 threads (3 in the test), with every kernel shape. The test
adds two contacts neither end of which moves, a path the scenes don't
take. Sleeping, sides (`Touching`) and the `Contact` events are left out
of both, since none changes a solved value. Mutation-checked: a dropped
warm start, an unwritten angle, a skipped carry, a skipped scatter, the
coloring without Box2D's rule, swapped ends, no turning in `Move`, a
wrong `dyn` kernel and the block kernel without its warm start each fail
it. One edit survived: not zeroing an unsolved contact's impulses before
`unsolved`, which overwrites them anyway, so the mutation changes
nothing.

## Measurements

`taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:flows_spike`,
2026-10-02, Ryzen 9 7950X, one CCD, the machine otherwise mostly idle
(load 1 to 2 before the runs). Each number is the median over runs of
each run's median of 21. The whole-frame tables are over six runs, three
before the block kernel was added, since none of their ways changed. The
solver tables are over three. The pool is the benchmarks' kept threads
(`tests/pool.rs`), warmed before each threaded rep. Harness systems run
on the engine's world between frames, as working_set_spike does. The
world is put back after each way, so every way sees the same world.

### The solve, whole

µs a frame (the harness running the systems in plan order, apply nodes
included):

| way | falling, 1 thread | falling, 8 threads | settled, 1 | settled, 8 | pyramid, 1 | pyramid, 8 |
|---|---|---|---|---|---|---|
| as built, one system | 1162 | 578 | 5385 | 1672 | 3408 | 952 |
| pipeline of flows | 1220 | 554 | 5745 | 1655 | 3532 | 958 |
| pipeline, fresh allocations | 1261 | 599 | 5771 | 1714 | 3556 | 979 |
| fused: the same stages in one system, buffers kept | 1228 | 552 | 5767 | 1654 | 3543 | 939 |

The scenes: "falling" is the pile of 10 000 at step 31, "settled" the
same at step 430, "pyramid" 5050 at step 630. The mod's own solve over
the window before each, one thread, sleeping and sides included:
570, 5100 and 3315 µs.

- **At 8 threads the pipeline is the solve as built:** −4.2%, −1.0% and
  +0.6%.
- **At one thread it's 3.6 to 6.7% slower.** That is the generic
  primitive sharing states as relaxed atomics even on one thread. The
  solve as built has a plain-array path there (`lanes::solve`). The
  solver alone shows the same gap between its own two paths (below), and
  lore measured it before
  ([relaxed atomic floats](../lore/relaxed-atomic-floats-still-vectorize-at-a-movd-a-load.md)).
- **The flows between systems cost nothing measurable.** The pipeline
  against the same stages fused into one system: −0.7% to −0.3% at one
  thread, and +0.1% to +2.0% at 8.

By stage, µs, the median over six runs, the pipeline:

| stage | falling, 1 | falling, 8 | settled, 1 | settled, 8 | pyramid, 1 | pyramid, 8 |
|---|---|---|---|---|---|---|
| gather_bodies | 43.8 | 40.5 | 50.3 | 54.0 | 22.3 | 30.9 |
| gather_turning | 64.2 | 67.2 | 86.6 | 102.6 | 31.8 | 34.1 |
| gather_contacts | 36.4 | 29.1 | 262.1 | 180.4 | 196.5 | 105.7 |
| prepare | 110.6 | 89.9 | 775.1 | 407.0 | 480.1 | 241.1 |
| passes | 724.2 | 194.4 | 4000.9 | 685.2 | 2544.8 | 421.7 |
| finish | 40.0 | 8.7 | 188.1 | 29.0 | 95.4 | 18.4 |
| scatter_bodies | 75.8 | 66.8 | 121.6 | 102.6 | 46.2 | 44.4 |
| scatter_contacts | 9.9 | 7.7 | 83.2 | 22.6 | 47.4 | 18.1 |
| the frame less these | 114.2 | 47.0 | 171.4 | 65.2 | 67.1 | 38.3 |
| *as built:* gather / solver / write-back | 183 / 784 / 86 | 183 / 285 / 75 | 508 / 4519 / 197 | 451 / 1062 / 118 | 297 / 2959 / 91 | 204 / 659 / 60 |

"The frame less these" is the harness, query fetches, apply nodes (the
spatial re-sort after the scatters included) and the flows' hand-offs.
For the fused system it is 105, 35, 149 to 153, 45, 54 and 25 µs. That
is 9 to 22 µs less than the pipeline's at one thread, and 12 to 20 at
eight, for seven more systems.

### The flow mechanism itself

- **A use costs about 85 ns.** Sixteen systems that do nothing, a frame
  run by the harness: 71 ns a system bare, 156 a system making, passing,
  seeing or taking a flow (every use but `See` with an apply node). Three
  runs: 83 to 86 ns.
- **The plan check costs 0.3 µs** for those 16 systems. It runs at load,
  not each frame.
- **Ordering costs nothing extra:** the edges are the graph's own event
  rule.
- **Recycling saves 2 to 7% of the frame at 8 threads** (554 against 599
  falling, 1655 against 1714 settled, 958 against 979 on the pyramid).
  At one thread it saves 1 to 3%. It is the gathers that gain:
  `gather_contacts` 262 against 349 µs settled at one thread, 180
  against 258 at 8. **But the settled pile's `prepare` is 5 to 7%
  slower with recycled allocations** (775 against 728, 407 against 379).
  Its vectors are cleared and refilled to the same length either way.
  Not explained; get-znt.27.

### The generic primitive against the hand-tuned solver

The solver alone on the gathered input. µs, the median of three runs of
21. Setup is coloring, states, lanes and, generic only, the batches
filled, in brackets. As built fills the batches in its staged run's
first stage, so they're in its passes. Tail is the impulses and states
written back. Settled pile:

| way | 1 thread: setup (fill) | passes | tail | all | 8 threads: setup (fill) | passes | tail | all |
|---|---|---|---|---|---|---|---|---|
| as built, 4 wide (`solve_across`) | – | – | – | 4569 | – | – | – | 1019 |
| as built, 4 wide, staged run apart | 140 | 4564 | 198 | 4902 | 144 | 800 | 34 | 978 |
| generic, a kernel a batch of 4 | 782 (614) | 4260 | 194 | 5240 | 288 (138) | 682 | 29 | 1000 |
| generic, a kernel a block of batches of 4 | 659 (509) | 4053 | 191 | 4905 | 284 (137) | 689 | 29 | 1003 |
| generic, a batch of 4 through `&dyn Fn` | 664 (512) | 4211 | 191 | 5066 | 284 (137) | 722 | 29 | 1038 |
| as built, 8 wide, staged run apart | 140 | 4560 | 193 | 4893 | 144 | 792 | 33 | 970 |
| generic, a kernel a batch of 8 | 860 (696) | 4641 | 190 | 5694 | 280 (130) | 743 | 29 | 1053 |
| generic, a kernel a block of batches of 8 | 660 (508) | 4051 | 190 | 4901 | 279 (131) | 683 | 28 | 991 |
| as built, 1 wide, staged run apart | 138 | 10 204 | 198 | 10 540 | 143 | 1577 | 40 | 1760 |
| generic, a kernel a batch of 1 | 764 (597) | 9650 | 184 | 10 610 | 288 (138) | 1466 | 28 | 1782 |
| generic, a kernel an edge, its two states | 652 (498) | 9594 | 182 | 10 428 | 285 (138) | 1495 | 28 | 1810 |

The one-thread "batch of 4" row is noisy: the six-run tables put it at
4910 to 4964.

The pyramid, and the falling pile:

| way | pyramid, 1 thread | pyramid, 8 threads | falling, 1 thread | falling, 8 threads |
|---|---|---|---|---|
| as built, 4 wide (`solve_across`) | 2954 | 636 | 788 | 281 |
| as built, 4 wide, staged | 3138 | 624 | 900 | 268 |
| generic, a block of batches of 4 | 3150 | 651 | 905 | 282 |
| generic, a batch of 4 through `&dyn Fn` | 3244 | 668 | 927 | 285 |
| as built, 8 wide, staged | 3076 | 623 | 878 | 265 |
| generic, a batch of 8 | 3476 | 690 | 938 | 292 |
| generic, a block of batches of 8 | 3045 | 645 | 892 | 285 |
| as built, 1 wide, staged | 6494 | 1061 | 1675 | 440 |
| generic, a kernel an edge | 6454 | 1098 | 1664 | 446 |

What it shows:

- **With a block kernel, the generic primitive is the hand-tuned run.**
  Against `run_across` on the same atomics, at one thread it is level:
  −1.0% to +1.6%, 4 and 8 wide on the three scenes. At 8 threads it
  is +2.6%, +4.3% and +5.2% at 4 wide, and +2.2%, +3.5% and +7.5% at 8
  wide. The 4 wide settled figure, +2.6%, is 25 µs.
- **So "nearly", not "matches".** Within 0 to 3.5% at 8 threads is what
  the passes alone show: they are faster than `run_across`'s, since its
  include the fill.
- **Where the 8-thread gap is: the fill, moved out of the run.** As
  built, the batches are filled in the staged run's first stage, by the
  threads that then solve them. The pipeline fills them in `prepare`,
  its own `par_for_each_mut`, one more hand-off to the pool, with chunks
  that don't line up with the passes' blocks: 137 µs against about 115.
  The serial part (coloring, states, lanes) is the same, 141 to 150 µs
  against 144. A `prepare` that fills in the passes' first stage would
  close it. Then `prepare` and `solve` are one system, and the pipeline
  has a coarser stage, which is the design doc's choice.
- **A kernel per edge costs its SIMD lanes, not its call.** Handed its
  two states, a kernel per edge is the hand-tuned solve at one lane to
  within 3.5%. The batch of one costs the same. Both are 1.6 to 1.8
  times the four-lane solve at 8 threads, and 1.8 to 2.1 times at one. The closure is
  inlined, since the primitive is generic over it.
- **A call through `&dyn Fn` costs 1 to 4%.** A kernel compiled apart from
  the primitive, as a mod's would be if the primitive weren't generic,
  pays that.
- **Deciding the pass per item costs at 8 lanes.** With the pass matched
  inside a kernel called per batch, 8 wide was 7 to 16% slower than the
  hand-tuned run, though 4 wide wasn't. Handing the kernel a block of
  batches, matched once and looped in each arm as `run_across` does,
  recovered it.
  ([lore](../lore/a-kernel-per-edge-costs-its-simd-lanes-not-its-call.md)).
  So the primitive's kernel should take a slice of items.
- **One thread on atomics costs 6 to 14%** against the solve as built's
  plain path (settled 4902 against 4569, pyramid 3138 against 2954,
  falling 900 against 788).
  The generic primitive could offer a plain path too, if its kernels
  were written over a state view, as `lanes::Bodies` is: get-znt.26.

## The second user: hierarchy propagation

**Why hierarchy.** It is the one real candidate working-sets.md found:
every scene graph has it. It has a measured baseline: hierarchy_spike.rs,
the idiom a game writes today at 29 µs. And it's a different shape from
physics: one cheap pass in levels of a tree, not many passes over a
colored graph. The games in the repo pass nothing between systems at a
scale worth measuring: a handful of lookups a frame (working-sets.md, "The
second user").

```text
gather     world -> Make<Nodes>          (rows, a depth order, each parent's place)
propagate  See<Nodes> -> Make<Globals>   (a level at a time, each level by par_for_each_mut)
extent     See<Globals> -> Make<Extent>  (the forest's box, by `reduce`)
scatter    See<Nodes>, Take<Globals> -> world
```

`taskset -c 0-7 ./bazel run --config=bench //engine/ecs:flows_hierarchy_spike`,
2026-10-02, µs a frame, every local written each frame. The median over
three runs of each run's median of 41. Every frame, every way is checked
bit for bit against a recursive reference, and the extent against one
folded in order:

| forest | way | gather | propagate | extent | scatter | frame |
|---|---|---|---|---|---|---|
| 10 500 nodes, 2 deep | hand, by level (the idiom) | – | 28.6 | – | – | **29.5** |
| | hand, sorted each frame | – | 144.7 | – | – | 145.9 |
| | flows, sorted by depth, 1 thread | 107.4 | 14.3 | 4.5 | 8.6 | 138.1 |
| | flows, sorted by depth, 8 threads | 106.8 | 15.3 | 2.2 | 8.5 | 137.4 |
| | flows, in walk order | 38.8 | 20.0 | 4.4 | 8.4 | 74.3 |
| 10 080 nodes, 5 deep | hand, by level | – | 28.1 | – | – | **29.0** |
| | hand, sorted each frame | – | 127.2 | – | – | 128.3 |
| | flows, sorted by depth, 1 thread | 97.0 | 14.1 | 4.3 | 8.1 | 126.6 |
| | flows, sorted by depth, 8 threads | 95.9 | 12.5 | 2.1 | 8.1 | 122.1 |
| | flows, in walk order | 38.0 | 19.8 | 4.2 | 7.9 | 73.1 |
| 10 080, 5 deep, indices scrambled | hand, by level | – | 59.1 | – | – | **60.1** |
| | hand, sorted each frame | – | 135.6 | – | – | 136.8 |
| | flows, sorted by depth, 1 thread | 105.6 | 14.0 | 4.3 | 8.4 | 135.5 |
| | flows, sorted by depth, 8 threads | 104.5 | 16.6 | 2.1 | 8.5 | 135.8 |
| | flows, in walk order | 38.3 | 34.9 | 4.2 | 8.2 | 88.7 |

"In walk order" is the idiom's order on the copy: rows as walked, each
parent found through the index, as many passes as it takes.

What flows made easier:

- **A second reader was free.** `extent` sees the globals, and nothing
  else in the pipeline knows it's there. As a fixed-order `reduce` it is
  the same box at any thread count. The plan check notes that nothing
  reads its output, which is allowed: it's there for whoever wants it.
- **The stages are separable and swappable.** Two `gather`/`propagate`
  pairs (by depth, in walk order) sit behind the same flows and the same
  `scatter`.
- **The pipeline's frame overhead is about 3 µs** for four systems and
  three flows: the frame less the systems' bodies.

What it made harder, or didn't help:

- **The copy is the cost, and flows are a copy.** Flows sorted by depth
  cost what the sorted copy costs (122 to 138 against 128 to 146). Kept
  allocations take 1 to 8 µs off. In walk order they cost 73 to 89
  against the idiom's 29 to 60. The 38 µs gather and 8 µs scatter alone
  are more than the idiom's whole frame on fresh indices.
- **Threads don't pay at this size.** A level's propagation is about
  1.5 ns a node, so one pool run per level costs what it saves:
  12.5 to 16.6 against 14.0 to 14.3 µs.
- **No provided shape fits "a level reads the levels before".** The spike
  splits the slice by hand (`split_at_mut` at the level's start), which is
  safe but is the system's own code, not a shape the ECS provides.

So the second user says flows aren't physics-shaped in their mechanism
(edges, ownership, fan-out, reductions all worked unchanged). But their
cost model is the copy's: they serve systems whose work is many times the
copy, as the solve's 20 passes are. For hierarchy, working-sets.md's
answer stands: a depth order kept by storage (get-qdi).

## What the spike showed

### Edges

- **The existing graph is enough.** A flow's edge is an event queue,
  read by `See` and written by the rest. That gives `See ∥ See`, and
  everything else in plan order, with no new overlap rule.
- **The overhead of borrowing events is small but real.** Every writer
  gets an apply node that publishes nothing: about 85 ns a use, with the
  harness's bookkeeping. A `ParamDecl::Flow` would skip the apply node
  and carry the access kind the spike smuggles in as `Dt` leaves.
- **Order is checked, not inferred.** The plan comes from phases and
  `.after`/`.before`, as today, and a plan whose flows are out of turn is
  refused. Inferring order from flows would be a topological sort of
  each `Make` before its uses. That is a change to how
  `engine/loader/schedule.rs` builds plans, and it is **the user's
  decision**. Mods that don't know each other would then get a working
  order without naming each other's systems. Without it, a consumer must
  name its producer's phase or system, as it would for a component.

### Ownership

The table above. Two findings to carry into the design:

1. **A hook is a `Pass`.** A system can't take and make one flow (the
   event rule refuses reading and writing one queue in one system).
2. **Recycling is by type,** so stages that change a value keep its type
   (`Pass`), or the allocation is lost to another type's bin. The design
   should say which, or let a `Take` hand its allocation to a `Make` of
   another type.

### Hot reload and mods that don't know each other

Reasoned, not built:

- **Nothing a flow holds crosses a frame**, and reloads happen between
  frames, so no value is carried by a reload.
- **But the recycling bins do outlive a frame.** A bin holds an emptied
  value whose drop code is the build that made it, as an event queue's
  values are. A real store needs the same keepalive events have
  (`EventQueue::_keepalive`): drop a flow's bin when a build that
  declares it reloads, or keep that build mapped until the bin goes. The
  cost is one frame's fresh allocations after a reload.
- **A flow's type isn't a component's.** `Vec<SolverBody>` has no
  `FieldType` schema, so it can't be migrated or fingerprinted field by
  field. A flow's layout is part of its declaring mod's interface. A
  consumer compiled against an older interface is caught by the
  interface digest at load (mod-deps.md), not by a schema. That is
  stricter than components need, and right for a value that lives one
  frame.
- **Identify flows by name, not `TypeId`.** The spike's store matches by
  name and downcasts by `TypeId`. Across builds a `TypeId` can differ
  for the same name, and the downcast should fail loudly rather than
  alias. The real store should check the interface digest instead.
- **Mods that don't know each other:**
  - a debug view or an optional reader only `See`s, and needs nothing
    from the producer;
  - a second `Take` from another mod is refused at load, as a cycle is;
  - a `Pass` from another mod (a hook such as one-way platforms) is
    ordered by plan order like any two writers of a component, so it
    needs a phase or an `.after`.

### Parallel shapes

- **`Colored::passes` is a closed shape.** Colors of items, a program of
  stages over the items or over ranges of states, one run of the pool,
  any thread taking any block. It carries physics's whole staged solve
  with no physics in it, within 2 to 7.5% of the hand-tuned run at 8
  threads, and level at one.
- **The kernel shape matters:** a block of items, the pass decided outside
  the loop. A kernel per edge loses the SIMD lanes, 1.6 to 2.1 times. A
  kernel behind a pointer loses 1 to 4%.
- **It still takes `Workers`, which declares nothing.** The spike's
  systems reach the pool only through the provided shapes. But as built,
  the scheduler sees no more than it did with phase 2's staged run: what
  differs is the shape, closed and generic, not visibility. For the
  scheduler to see the parallel work, the shapes would have to be
  declared, for example a `Passes<Graph>` parameter whose footprint
  claims the pool, or the pool claimed by the flow's edge. That is the
  phase 2 rejection's real question, and **the user's decision**. Decided
  2026-10-02 (get-znt.28): parallel work is declared and run by the
  scheduler, never by a system. A shape is a declaration the scheduler
  turns into tasks on its own threads (get-znt.29), not a call a system
  makes through `Workers`.
- **No unsafe code.** States are shared as relaxed atomics (`lanes::Atom`),
  blocks behind a lock each, as `run_across` does. The benchmarks' pool
  (`tests/pool.rs`) is the only unsafe code the spike runs, as every
  bench does.

## Recommendation

**Proceed to a design doc** (get-znt.25), scoped to what held up. A design doc should
settle these:

1. **The declaration:** `ParamDecl::Flow { slot, access }`, with no apply
   node, and `Make` as a returned value where that reads better. It
   changes `engine_ecs` and so `API_VERSION`.
2. **Where values live:** in the world, by name, with bins kept for each
   flow and dropped, or kept mapped, across a reload of a declaring
   build. Whether recycling can follow a value across types.
3. **Ordering:** checked at load (as built here) or inferred from edges by
   the plan builder. The user's decision.
4. **Fixed-rate phases:** a flow lives one run of the node list, so one
   step of a fixed-rate group. Say so, and refuse an edge across a group
   boundary.
5. **The parallel shapes:**
   - `Colored::passes` with a block kernel;
   - a one-thread path without atomics (get-znt.26), or the 6 to 14% at
     one thread accepted;
   - the shapes as declarations the scheduler runs, as decided
     (get-znt.28): how a system declares a program of stages, and how
     the scheduler hands out its tasks (get-znt.29, get-znt.30).
   - Whether to provide "levels of a tree" too, which the second user
     wanted.
6. **Physics's adoption, if any:** the port here is bit for bit and costs
   nothing at 8 threads. Adopted, it would want `prepare`'s fill moved
   into the passes' first stage, and colors kept as world state
   (parallel-relations.md, phase 1) as a flow's input.
7. **What flows are not for:** one cheap pass over the world, where the
   copy costs more than the work (the hierarchy).

## Open questions

- **Open question:** why the settled pile's `prepare` is 5 to 7% slower
  on recycled allocations than on fresh ones (get-znt.27).
- **Open question:** whether a generic primitive's one-thread path can be
  plain memory without making kernels generic over a state view.
- **Open question:** whether flows should be visible to snapshots and
  replays. They're frame-scoped, so they needn't be, but a replay that
  stops between two stages would see one.

[^spike-code]: *(History, 2026-10-02.)* The spike's code was removed once its findings were written here: spikes are built to answer a question and then thrown away. Every spike target and command named in this doc builds and runs at commit `c72e8b2` (`git checkout c72e8b2`), the last commit with every spike building.
