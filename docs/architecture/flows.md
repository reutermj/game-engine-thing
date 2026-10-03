# Flows

**Status: stage 1 built** (2026-10-02; designed in get-znt.25, built in
get-znt.32): the mechanism and the shapes in `engine_ecs`
(`engine/ecs/flows.rs`, `engine/ecs/shape.rs`), the shapes run on one
thread, and the plan check in the loader's `schedule.rs`. **Stage 2
built** (2026-10-02, get-znt.33): physics2d's solve is a pipeline of nine
systems over five flows, its passes on `Passes`, bit for bit the solve it
replaced ([below](#physicss-adoption-stage-2)); since get-znt.26 its
kernels get plain memory on one thread ([On one thread](#on-one-thread)).
physics3d's solve followed (2026-10-02, get-znt.35): six systems over
three flows, its solve whole in one system and no shape, bit for bit
([below](#physics3d)). **Stage 3 built** (2026-10-03, get-znt.34): the
scheduler's threads, a resident mod's pool, and `Passes` run across them
as a task graph, both physics solves among them, bit for bit
([threads.md](threads.md)).
The spike behind every choice here, with its measurements, is
[flows-spike.md](flows-spike.md); this doc cites its numbers rather than
restating them.

**In short:**

- **A flow is a typed value that lives one frame** (one step, in a
  fixed-rate group) and passes from system to system: a source makes it
  from the world, stages see, edit or take it, and a sink writes what it
  carries back. Systems take it as parameters, `Make<T>`, `See<T>`,
  `Pass<T>` and `Take<T>`, so every use is a declaration.
- **It's for systems that already copy.** Physics's solve, as eight
  systems passing four flows, was bit for bit the solve as built and cost
  the same at 8 threads. Transform propagation as flows cost 1.5 to 4.7
  times the idiom it would replace. A flow is a copy, and it pays only
  where the work is many times the copy.
- **Order comes from the plan, as now.** Flows add a check at load that
  refuses a plan whose flows are out of turn, and its errors name the
  fix: "`See<Graph>` in `debug::view` runs before `Make<Graph>` in
  `physics2d::prepare`; add `.after("physics2d::prepare")` to
  `debug::view`".
- **Edges are the graph's own.** A flow's uses order like an event
  queue's: `See`s together, anything else one at a time, with no apply
  node.
- **Values live in the world, by name**, out of sight of everything but
  the systems that declare them, and empty between frames. Each flow keeps
  one value for its allocations, a recycling bin, dropped whenever a build
  that uses the flow is installed.
- **Parallel work is a declared shape**: `ParMap`, `Reduce` and `Passes`
  are parameters, and the kernels a system hands them are the only code
  that may run across threads. Stage 1 fixed their results on the
  system's own thread; since stage 3 `Passes` reproduces them across the
  scheduler's threads, at any count.

## What a flow is, and isn't

The user's idea (2026-10-02): systems that act like map/reduce stages,
taking data from a previous system and passing it to the next, with the
systems at either end acting on the world. Physics's solve is already
that inside one system: it gathers the world into dense arrays, iterates
on them, and scatters the results back. Three spikes found that the copy
is the solver's need, not a storage workaround
([working-sets.md](working-sets.md), [parallel-relations.md](parallel-relations.md)
(c), [contiguous-columns.md](contiguous-columns.md)). Flows make the
copy's stages systems the ECS can see.

A flow is:

- **frame-scoped.** Nothing it holds survives the frame, or the step of
  its fixed-rate group. Persistent state belongs in components, as it
  does today.
- **typed and named.** `"physics2d::flow::Graph"` is one value, wherever
  it's declared from.
- **declared at each use**, so the graph orders its uses and the plan
  check can refuse a bad order before anything runs.

A flow isn't:

- **a cheaper way to read the world.** It is a copy, and costs what any
  copy costs. The hierarchy spike's gather alone (38 µs in walk order) was
  more than the idiom's whole frame (29 µs) on fresh indices. Flows serve
  systems whose work is many times the copy, as the solve's 20 passes
  are. For hierarchy the answer stays a depth order kept by storage
  (working-sets.md, get-qdi).
- **a channel between frames.** That is an event, or a component.
- **a resource.** There is no world-wide singleton; a flow has a producer
  every frame, or it has nothing.

## Declaring a flow

```rust
engine_api::flow! {
    /// The awake bodies, dense, by an index the step makes.
    pub struct Bodies: "physics2d::flow::Bodies" {
        pub entities: Vec<Entity>,
        pub bodies: Vec<SolverBody>,
        pub kinds: Vec<(bool, u8)>,
    }
}
```

`flow!` declares the struct and implements `Flow` for it: its name, and
`recycle`, which empties each field and keeps its allocations. A field
can be any type that implements `Recycle`: `Vec` (cleared), `Option`
(back to `None`), the scalars (back to their default), `String`, or a
type of the mod's own. Unlike a component, a flow's fields needn't be
`FieldType`s: a flow is never migrated, stored between frames or seen by
another build (below).

A flow that other mods use goes in its mod's interface, as a component
does, and is named after the mod by convention.

## The parameters

| use | what the system gets | how many a frame | runs |
|---|---|---|---|
| `Make<T>` | `&mut T`, empty: last frame's value emptied if there is one, else `T::default()`. What it holds when the system returns is the frame's | one | before every other use |
| `See<T>` | `&T` | any | after the `Make`, before any `Take`; together with other `See`s |
| `Pass<T>` | `&mut T`, to edit in place and hand on | any | after the `Make`, before any `Take`, one at a time |
| `Take<T>` | owns it. Dropped, it goes back to the bin; `into_inner` keeps it | at most one after each `Make` | after the `Make` |

The rules, each from a case the spike tested (flows-spike.md,
"Ownership"):

- **One maker.** Two producers of one value is ambiguous. A stage that
  changes a flow takes `Pass<T>`.
- **Nothing has to take it.** A flow nobody takes is moved to the bin at
  the end of the frame. An optional reader, such as a debug view, only
  `See`s, and nobody has to take the flow to keep the frame valid.
- **Nothing uses it after it's taken**, except a second `Make`, which
  makes a new value.
- **A system uses a flow once.** `See` and `Make` of one flow in one
  system is nonsense either way, and a hook that edits a flow is a `Pass`,
  not a `Take` and a `Make`. Refused when the system is added, as two
  conflicting queries are.

**`Make` is a parameter, not a return value.** A returned flow reads
better for a pure function, and Bevy's `pipe` does it that way. But:

- the value starts from last frame's allocations, so the system needs it
  as `&mut T` before it fills it. A returned value would get them through
  a parameter anyway, or lose them. Recycling saved 2 to 7% of the solve's
  frame at 8 threads.
- a system that makes two flows (physics's `prepare` passes `Contacts`
  and makes `Graph`) would return a tuple, while its other uses stay
  parameters: two ways to declare one thing.
- the loader's `SystemFn` returns a status, and every system returns
  `()`. Return values would change `IntoSystem` and the ABI for one
  parameter kind.

**Recycling is by type.** A stage that takes `Bodies` and makes a
`Solved` from its vectors moves the allocation into `Solved`'s bin, and
`Bodies`' `Make` allocates afresh every frame. So stages that change a
flow edit it in place (`Pass`), as the spike's pipeline did. Recycling
doesn't follow a value across types, and `Take::into_inner` keeps the
value for good, so its allocation leaves the store.

## Edges in the graph

A flow's uses are edges the frame's graph already knows how to order. The
spike declared each flow as an event queue: `See` reads it, the other
uses write it, and `graph.rs`'s rule for events gave exactly what a flow
needs (flows-spike.md, "Edges"):

- two `See`s run together;
- anything else waits for every earlier use, in plan order.

The real declaration is `ParamDecl::Flow { slot, name, access, ty }`.
`graph.rs` treats `See` as a read of the slot and every other use as a
write, as it treats an event queue, in a namespace of its own. Unlike an
event writer, a flow's use changes nothing in the world, so it has **no
apply node**: a reader waits for the system before it, not for an apply
after that. The spike's uses cost about 85 ns each, apply node included;
the real ones skip the apply node.[^stand-in]

## The store

A world keeps its flows as it keeps its event queues: a slot for each
name, interned when a build declares a system that uses it.

- **The value**, while a frame runs: made by `Make`, borrowed by `See`
  and `Pass`, moved out by `Take`. Each use takes the slot's lock with
  `try_*`, so two uses the graph should have ordered are a panic, never a
  race.
- **The bin**: one value, kept for its allocations for the next `Make`.
  `Take` puts its value back when dropped, and the end of the frame moves
  a value nothing took into the bin. `Make` empties whatever it starts
  from.
- **Between frames, a flow is empty.** Its bin holds allocations, not a
  value anyone can read. So flows are invisible to `WorldMut`, to hooks
  and message handlers, and to every inspection of the world
  (`World::values`, `summary`), and a replay that stops between frames
  never stops between two stages.

### Reload, and whose code a bin holds

A bin is a boxed value whose drop code, and whose type, is the code of
the build that made it. That's a problem for two reasons:

1. **Its code may go.** A bin made by `maker` v1 and dropped after v1 is
   unmapped calls into nothing. Event queues have the same problem, and
   solve it by keeping the build mapped (`EventQueue::_keepalive`).
2. **Its layout may change.** `maker` v2 can declare `Bodies` with
   another field. A downcast by `TypeId` doesn't tell the two apart:
   `TypeId` hashes the crate's name and the type's path, not its fields,
   so two builds of one crate can agree on it (reasoned, not measured).
   A v2 `Make` handed v1's bin would then read the wrong layout with no
   `unsafe` in sight.

So:

- **A flow's bin is dropped whenever a build that uses it is installed**
  (loaded or reloaded): as the load commits, before the old build is
  released, with its code still mapped. The cost is one frame of fresh
  allocations after a reload, about what the "fresh allocations" row of
  flows-spike.md's first table costs once.
- **The slot keeps every user's build mapped**, by build name, so a bin
  outliving an unloaded maker still drops with valid code. A newer build
  of the same mod replaces its predecessor's keepalive.
- **Poison mode checks it** (hot-reload.md, "Poison mode and the
  sanitizers"): a bin kept across a reload would be downcast through an
  unmapped vtable at the next `Make`, and fault naming the build.

### Names, and the interface digest

Flows are found by name, as events are. Their layout is guarded by the
declaring mod's **interface digest** (mod-deps.md), not by a field schema
or by `TypeId`:

- a flow other mods use is in its mod's interface, which they compile
  against and name in `mod_deps`;
- changing it changes the digest, and the loader refuses a batch that
  would leave a dependent built against the old one ("reload them together
  with `./bazel run //game:reload`");
- so every build running at once was compiled against one layout of each
  shared flow, and the bin dropped at install covers the moment between.

That is stricter than components need, and right for a value that lives
one frame: there's nothing to migrate.

The plan check also compares each use's `TypeId`. Builds of one crate
agree on it, so it can't guard a layout; what it catches is two mods
declaring one name separately, which would otherwise fail at the first
`Make` after both load.

## The plan check

**Plan order stays the only source of order.** The plan comes from
phases, `.after`/`.before` and declaration order, as scheduling.md says.
Flows add a check of the plan, run wherever the plan is built: when a load
or unload is checked before it commits, and when the plan is rebuilt. A
plan that fails it is refused like a cycle, and the running builds are
untouched.

Walking the plan in order, a fixed-rate group's phases once (every step
repeats them), it refuses, with errors that name the fix:

| refused | the error |
|---|---|
| a use before the `Make`, same phase | `` `See<test::Numbers>` in `b::view` runs before `Make<test::Numbers>` in `a::make`; add `.after("a::make")` to `b::view` `` |
| a use before the `Make`, earlier phase | `` `See<…>` in `b::view` (phase update) runs before `Make<…>` in `a::make` (phase late); move `b::view` to phase late, with `.after("a::make")` `` |
| a use with no `Make` loaded | `` `See<…>` in `b::view`: nothing loaded makes test::Numbers `` |
| a second `Make` | `` `Make<…>` in `b::again` and `Make<…>` in `a::make`: a flow has one maker; a stage that changes it takes `Pass<…>` `` |
| a use after the `Take` | `` `See<…>` in `c::late` runs after `Take<…>` in `b::sink` took it; add `.before("b::sink")` to `c::late` `` (or move it, as above) |
| a second `Take` | `` `Take<…>` in `c::t2` runs after `Take<…>` in `b::t1` took it: a flow has one taker; make one of them a `Pass` that runs before the other `` |
| uses in two groups | below |
| two types under one name | `` test::Numbers is declared as two types: `a::Numbers` by `a::make` and `b::Numbers` by `b::view`; a flow shared between mods is declared once, in its mod's interface `` |

A flow made again after it's taken is a new value, and is allowed. A flow
nothing reads is allowed too: it's there for whoever wants it.

**Why not infer the order from flows.** Inference would be a topological
sort with each `Make` before its uses, and mods that don't know each other
would get a working order without naming each other's systems. It isn't
done, for now, because:

- **every user so far is one mod.** Physics's eight systems are
  physics2d's, and so would be a hierarchy pipeline. Within one mod,
  declaration order already is the pipeline's order; the check only
  confirms it.
- **a consumer in another mod already names its producer** for anything
  else it reads: a component written in `simulate` is read in `late`, or
  after `.after("physics2d::step")`. A flow asks the same, and the error
  says exactly what to write.
- **two sources of order are two things to explain.** With inference, a
  plan's order would depend on which flows a system takes, and adding a
  `See` to a debug view could reorder a game's systems.

If it comes, inference starts within one mod, where it can't reorder
anyone else's systems.

**At run time** the same rules are asserted at fetch, so a plan that
skipped the check (a harness test, a bug) fails at the first bad use,
naming the flow: `See`, `Pass` or `Take` with no value present, or a
value of another type. A value never outlives its frame, so last frame's
can't be mistaken for this frame's. Within a frame, the run-time check
can't tell one step of a group from the next; the plan check is what
guarantees a later step's uses follow that step's `Make`.

## Fixed-rate groups

A fixed-rate group's phases run once a step, as many steps a frame as
time has accumulated (scheduling.md, "Fixed rates"), and the phases
outside every group run once a frame. **A flow lives one step of its
group**, or one frame outside any group:

- each step's `Make` starts from the value the step before left, if
  nothing took it, emptied. Nothing of one step's value reaches the next;
- **uses in two groups are refused**, and so are uses both in a group
  and outside one:

  `` `See<…>` in `render::draw` (once a frame) and `Make<…>` in `physics2d::prepare` (simulate at 60 Hz): a flow lives one step of its group, so its uses can't cross groups; use it in one group, or carry the value across in a component or an event ``

  A frame may run a group's steps none, one or several times, so a reader
  outside the group would see none, the last step's, or a value from a
  frame with no steps. None of those is a flow's meaning.

## Parallel shapes

**Decided by the user** (2026-10-02, get-znt.28): parallel work is
declared and run by the scheduler, never by a system. Systems declare the
shape of their parallel work; they never hold a thread pool. This
answers the rejection of parallel-relations.md's phase 2, systems taking
the pool to run parallel work the ECS can't see.

### A shape is a parameter

```rust
fn solve(&mut self, _: &mut (), _: &mut Cx, turning: See<Turning>, mut graph: Pass<Graph>, passes: Passes) {
    let Graph { layout, items, states, .. } = &mut *graph;
    passes.run(layout, items, states, &PROGRAM, |pass, block, states| solve_block(pass, block, states, &turning), |pass, range, states| integrate(pass, range, states));
}
```

(physics2d's own is `passes` in `engine/std/physics2d/pipeline.rs`.)

Three parameters, each declaring `ParamDecl::Shape` with its kind, and
nothing else: no footprint, no apply node.

- **`ParMap`**: `map_into(items, min, out, f)` sets `out` to `f(i, item)`
  of each item, in the items' order, keeping `out`'s allocation;
  `for_each_mut(items, min, f)` calls `f(i, &mut item)`. `min` is the
  fewest items a task is worth.
- **`Reduce`**: `reduce(items, chunk, map, fold)` maps fixed chunks of
  `chunk` items and folds the results left to right, in the items' order.
  The chunks are the input's, not the threads', so a float sum is the
  same at any thread count.
- **`Passes`**: `run(layout, items, states, program, block, each)` runs a
  program of stages over items in colors and the states they share:

  ```rust
  pub enum Stage<K> {
      /// Every item, a color at a time, the overflow first.
      Items(K),
      /// Each of `n` states, by range.
      Each(K, usize),
  }
  ```

  `block(k, &mut [I], States<T>)` gets a block of consecutive items of
  one color, and `each(k, Range<usize>, States<T>)` a range of states.
  `layout` is a `Colored`, made by `Coloring::greedy` and `pack` (Box2D
  v3's rule and layout, from the spike). The states are a `&mut [T]`,
  which kernels get as `States`: the slice itself on one thread, and on
  several its shared form (`T: Shareable`, relaxed atomics in physics),
  which kernels write only where their items' edges are ("On one
  thread", below).

Why each choice, from the spike:

- **The kernel takes a block, the stage decided outside the loop.** A
  kernel per edge cost 1.6 to 2.1 times the four-lane solve, all of it the
  SIMD lanes lost; matching the pass inside a kernel called per batch cost
  7 to 16% at 8 lanes. With a block kernel the generic primitive was level
  with the hand-tuned run at one thread (−1.0% to +1.6%).
- **Shapes are generic over their kernels**, so the mod's compiler inlines
  them. A kernel behind `&dyn Fn` cost 1 to 4%. The scheduler's side of
  stage 3 sees only a block at a time: one dynamic call a block, not an
  item.
- **The shapes are closed.** Colors of items, stages over items or
  ranges, a map, a fixed-order reduction. Physics's whole staged solve
  fits in them with no physics in the shape.

### Declared, and run by the scheduler

The system calls its shape with the frame's data and its kernels; what
runs them is the scheduler's. That is the line get-znt.28 draws:

- **the declaration** is the parameter: the plan knows, before a frame
  runs, which nodes fan out and how (a map, a reduction, a colored run).
  Stage 3 sizes its work by it, and keeps such a node from waiting behind
  threads another system's tasks hold;
- **the execution** is the scheduler's. `Passes::run` turns the program
  into (stage, block) tasks on the scheduler's threads, each stage's
  blocks after the last stage's, as a task graph (built, stage 3:
  [threads.md](threads.md#dispatch); any thread takes any block, a late
  thread skips stages already done). The system's thread is one of the
  workers, and every kernel has returned when `run` does, so no mod code
  is on a worker's stack once the node ends (get-znt.29's rule for hot
  reload);
- **the system holds no pool.** Its parameters give it no way to start a
  thread, and `Workers` goes once its users have moved (get-znt.31).

*Considered and not proposed:* a shape as a node of its own, the kernel
registered when the system is declared and called by the scheduler after
the system returns, with the flow as its data. It would take the call out
of the system's body entirely. But a program's data is the frame's (the
coloring changes every step, and kernels close over the step's settings
and `See`s), the plan would gain a node kind beside systems and applies,
and the kernels would become plain functions with no captures. That's a
change to how the scheduler runs a frame, and nothing the spike measured
needs it.

### Determinism

Each shape's result is the same on any number of threads, by
construction, and stage 1 fixes what that result is:

- `ParMap` calls `f` once an item; its output is in the items' order.
- `Reduce` folds `map(chunk)` results left to right in chunk order. Stage
  3 may map chunks in any order but must fold in this one.
- `Passes` runs stages in program order, and within an `Items` stage the
  overflow's block first and then each color's. The kernel must treat a
  block's items independently: how a color is cut into blocks is the
  scheduler's, and only colors are promised. A color's items share no
  moving state, so blocks in any order give the same states.

### On one thread

**Built** (2026-10-02, get-znt.26): `Passes::run` takes the states as
plain memory, `&mut [T]`, and hands each kernel call a view of them:

```rust
pub enum States<'a, T: Shareable> {
    Plain(&'a mut [T]),        // on one thread: the slice itself
    Shared(&'a [T::Shared]),   // on several: made before the first stage, read back after the last
}

pub trait Shareable: Copy + Send + Sync {
    type Shared: Sync;         // relaxed atomics of the fields' bits, normally
    fn share(&self) -> Self::Shared;
    fn load(shared: &Self::Shared) -> Self;
    fn store(shared: &Self::Shared, value: Self);
}
```

A kernel matches the view once a call and runs its generic body on
either, which is how physics's lanes were already written
(`lanes::Bodies`, over `[State]` and the shared `Atom`s):

```rust
|k, block, s| match s {
    States::Plain(s) => kernels.block(k, block, s),
    States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
}
```

`States` also has `get` and `set`, a match an access, for a kernel that
would rather not be generic and doesn't mind the cost.
`World::set_shapes_shared` hands kernels the shared view on one thread
too, so a mod's shared path is tested before any thread runs it
(`quality_test`'s `the_mod_is_the_arrays_with_its_states_shared`), and
measured.

**The open question, answered: kernels needn't be generic, but fast ones
are.** Measured on physics's passes (`step_bench`, one thread, the median
of 5 runs, µs a step of the `passes` system):

| kernels get the states | pile 10 000 falling | settled | pyramid 5050 |
|---|---|---|---|
| plain, matched once a call (built) | 297 | 3850 | 2423 |
| plain, matched once an access (`get`/`set`) | 354 | 4569 | 2911 |
| shared, matched once a call (part 1's path) | 368 | 4106 | 2576 |
| shared, matched once an access | 439 | 5389 | 3309 |

- **A match an access costs 19 to 20%** of the passes plain, and more
  than the atomics: LLVM doesn't hoist the match out of the kernels'
  loops. The atomics cost 6 to 7% of the passes settled and on the
  pyramid, 24% falling, about what the spike measured. So a mod whose kernels are hot writes them generic over a
  view of its own, as physics does, and matches once a call.
- **Considered and not built:** kernels as a trait with generic methods
  (`fn block<V: View<T>>`), the engine choosing the view. The same code
  in the mod, a trait in place of two closures, and an engine-defined
  view with only whole-`T` loads, where physics's lanes load a body's
  velocity alone (`load_v`, which spares the shared path four atomic
  loads a body). And a plain path behind `UnsafeCell`, refused: new
  unsafe code whose soundness would rest on the colors.
- **The pipeline on one thread is no slower than the solve it replaced**:
  alternated runs of the pre-port solve and the pipeline on one build
  (`step_bench`, two rounds of 5), the solve system(s) µs a step, before /
  after: falling 564, 560 / 559, 555; settled 5240, 5199 / 5183, 5100;
  pyramid 3356, 3360 / 3245, 3242; not turning 2739, 2758 / 2757, 2742.
  The whole step is level (settled 6793, 6766 / 6844, 6740).

The `angle`s physics's `Move` stage keeps for turning bodies stay relaxed
atomics on both paths: written once a substep a turning body, through the
shared reference every kernel has, too few to be worth a plain path.

## Stage 1: on one thread

Stage 1 builds the mechanism in `engine_ecs` and runs every shape on the
system's own thread:

- `ParMap` and `Reduce` loop in order, `Reduce` still chunked as asked.
- `Passes` runs each stage in order: an `Items` stage calls `block` once
  for the overflow and once for each color, in order; an `Each` stage
  calls `each` once, over every state.

That is the result stage 3 must reproduce, and does: since stage 3
(2026-10-03, [threads.md](threads.md)) `Passes` runs across the world's
executor where it has more than one thread, and this path where it
doesn't, or where the system says `serial` (get-znt.39). `ParMap` and
`Reduce` stay on one thread until they have users (threads.md, "ParMap
and Reduce").

`API_VERSION` goes up: `ParamDecl` gains `Flow` and `Shape`, the world a
flow store, and `Declarations` the flows a build uses.

## Testing

Each tier proves what the others can't (CLAUDE.md):

- **Unit tests** (`engine_ecs`, and the loader's `schedule.rs`):
  - every plan error, with its text, and what's allowed (made again after
    a `Take`, made and never read);
  - the graph's edges: readers together, a writer after them, no apply
    node;
  - run time: a use with no value, a value of another type, a value from
    an earlier frame, and a system using one flow twice;
  - recycling: from a `Take`, from a value nothing took, a later step's
    `Make` from an earlier step's value, a bin dropped at install;
  - the shapes' sequential semantics: order, the fixed-order reduction,
    stages and colors in order, and the coloring's invariants.
- **Integration** (`reload_test`): purpose-made mods, a maker whose
  interface declares a flow and a reader depending on it, in two builds
  whose flow layouts differ. Reloaded as a batch, the new builds' code
  runs, the first `Make` starts from nothing, and recycling resumes. Poison
  mode is on in every loader test, so a bin kept across the reload faults
  at the next `Make`. Unloading both and dropping the engine checks that a
  bin outliving its maker drops with its code mapped.
- **End to end**: nothing new. The manifest, runfiles and socket don't
  change.
- **Stage 3** adds what only threads can show: `Passes` bit for bit at 1
  to 8 threads, at once, late and one by one, on test executors
  (`flow_test`) and on the real pool (`//engine/std/threads:pool_test`);
  the mods held to one thread across it (threads.md, "Determinism").

## Physics's adoption (stage 2)

**Built** (2026-10-02, get-znt.33). physics2d's solve, one system until
then, is a pipeline (`engine/std/physics2d/pipeline.rs`; physics.md, "The
step"):

```text
solve            world -> Make<Settings>
gather_bodies    See<Settings>, world -> Make<Bodies>
gather_turning   See<Bodies>, world -> Make<Turning>
gather_contacts  See<Bodies>, world -> Make<Contacts>
prepare          See<Settings>, See<Bodies>, See<Turning>, Pass<Contacts> -> Make<Graph>
passes           See<Settings>, See<Turning>, Pass<Graph>, Passes
finish           See<Settings>, Take<Graph> -> Pass<Bodies>, Pass<Turning>, Pass<Contacts>
scatter_contacts See<Settings>, Pass<Contacts> -> world
scatter_bodies   See<Settings>, Take<Bodies>, Take<Turning>, Take<Contacts> -> world
```

It is the spike's, with what the spike left out put back (sleeping,
`Touching`, `Contact`, the solve one contact at a time where nothing
turns), and three changes, each for a contract the one system kept:

- **`solve` comes first and reads the settings** (`Tuning`, `Gravity`,
  `Sleep`, into `Settings`). Pre-solve hooks order themselves
  `.before("physics2d::solve")`, and plan order ties on load order, so a
  first stage of another name would have run before the hook; `solve`
  keeps the hook before anything is gathered, and the platformer's
  walkers, the tests' hooks and physics.md's advice stay as they were.
  The solver's pass is `passes`.
- **The contacts are written first, then the bodies with sleeping.**
  Sleeping needs both (the bodies' speeds, the contacts' links), and the
  bodies' write and sleeping's stay in one system, as they were, so the
  tick sleeping counts from (`slept`) is after every write of the step's
  and the spatial re-sort and the moves to the sleeping tables come at one
  apply node.
- **A step where nothing turns** is solved one contact at a time in pair
  order, whose result depends on that order, so no shape fits it:
  `finish` solves it whole (`solver::solve_with`) and `passes` does
  nothing.

The flows are physics2d's own, not its interface's: they carry the
solver's layout, and a mod that saw them would be rebuilt for every change
to it. A debug view gets a flow of plain values in the interface, when one
is wanted.

**Bit for bit.** Every value of both baselines printed as it was (227
default, 224 long, `baseline -- --all`, none moved), the long suites
pass, pong's and the platformer's replays are unchanged (their schedule
tests list the new systems), and the tests that hold the mod to the arrays
bit for bit pass (`quality_test`'s `the_mod_is_the_arrays_bit_for_bit`
and `the_mod_solves_at_the_substeps_its_world_sets`, `behaviour_test`'s
`the_mod_is_the_arrays_on_the_behaviour_scenes`): those are what catch a
change to the pipeline alone. A gravity share one ulp off in the pipeline's
kernel failed all three; the baseline itself moved four values, all
inside their bands, so a band is no bit-for-bit check. A one-ulp change in
a kernel the arrays share moved 34 baseline values, all inside their
bands, and failed the lanes' equivalence tests.

**What it costs**, `step_bench` (`--config=bench`, `taskset -c 0-7`, the
median of 7 runs, each a fresh engine stepped to the window and timed over
30 steps), µs a step, the solve system(s) / the whole step:

| case | 1 thread, before | after | 8 threads, before | after |
|---|---|---|---|---|
| pile 10 000 turning, falling | 561 / 1052 | 570 / 1068 | 367 / 725 | 591 / 962 |
| pile 10 000 turning, settled | 5194 / 6724 | 5312 / 6944 | 1415 / 2107 | 5327 / 6298 |
| pyramid 5050 turning | 3368 / 4478 | 3391 / 4538 | 882 / 1339 | 3394 / 4010 |
| pile 10 000 not turning, settled | 2712 / 3288 | 2676 / 3252 | 2648 / 3083 | 2698 / 3127 |

- **One thread: 0.7 to 2.3% slower** where bodies turn, the solver's
  relaxed atomics; level where nothing does. Since get-znt.26 the passes
  get plain memory on one thread, and the pipeline is level with the
  solve it replaced or faster (below, "On one thread").
- **Eight threads: the solve's threads were gone until stage 3**, as
  planned (they are back, on the scheduler's pool: threads.md,
  "Measured"). With no shape run across threads the solve takes its
  one-thread time (5327 against 1415 settled), and the step loses what
  the solve gained. The broadphase and
  narrowphase still split across `Workers` (get-znt.31). (History:
  `solver_bench`'s `THREADS` and the comparison's `rot/threads=<n>` timed
  `solve_across` on arrays until stage 3 removed it, 2026-10-03.)
- **The batch fill stayed in `prepare`.** Moving it into the passes' first
  stage closed most of the spike's 8-thread gap, and changes nothing on
  one thread, where everything runs now; it is measurable only with stage
  3, so it is stage 3's call.
- **Bodies in entity order** (get-emj.88) is left for later: the
  renumbering it needs costs what it saves on a falling pile, and folding
  it in would have made the port's measurements two changes'.
- **Kept colors** (parallel-relations.md, phase 1; get-emj.74) become an
  input of `prepare`, decided with stage 3, so one re-baseline covers
  both.

### physics3d

**Built** (2026-10-02, get-znt.35). physics3d's solve, one system until
then, is the same pipeline less what 3D lacks
(`engine/std/physics3d/pipeline.rs`; physics.md, "A mod"):

```text
solve            world -> Make<Settings>
gather_bodies    See<Settings>, world -> Make<Bodies>
gather_contacts  See<Bodies>, world -> Make<Contacts>
solver           See<Settings>, Pass<Bodies>, Pass<Contacts>
scatter_contacts Take<Contacts> -> world
scatter_bodies   Take<Bodies> -> world
```

- **No shape** (until get-emj.90, below). 3D's solve is the sweep one
  contact at a time in pair order (`solver::solve`; in lanes by level
  since get-emj.52, which is that sweep bit for bit), whose result is
  that order's, so `solver` runs it whole. Coloring it would move
  results: a physics change, re-baselined (get-emj.90), then threads
  (get-emj.75).
- **No sleeping, sides or events** in 3D, so `scatter_bodies` writes
  bodies alone, and the contacts are taken by `scatter_contacts`.
- **No `Workers`** to remove: 3D never had them.

**Bit for bit.** Every value of both baselines printed as it was (100
default, 94 long, `baseline -- --all`, diffed whole), and the reload
replay's 161 frames, every component value printed, were the same
before and after but for the change ticks, which count systems. A
one-ulp change in the gather (the gravity given back) or in the solver's
kernel (a substep's share) failed one test, `quality_test`'s
`piles_of_turning_planks_rest_as_soon_as_rapier_and_box3d_do`, a bound
from the references, not an exactness check; each moved 35 of the 100
baseline values, all inside their bands, and all 161 replay frames.
3D has no arrays to hold the mod to, as 2D's
`the_mod_is_the_arrays_bit_for_bit` does; since get-emj.89 (2026-10-02)
`//engine/std/physics3d:exact_test` holds it to a pinned fingerprint
instead, the mod's every frame and the solver alone, and fails on both
of these changes, the first only in the mod's lines and the second in
the solver's too (physics-testing.md, "The exact fingerprint"). Coloring
it is get-emj.90.

**What it costs**, `//engine/std/physics3d:step_bench` (`--config=bench`,
`taskset -c 0-7`, the median of 7 runs, each a fresh engine stepped to
the window and timed over 30 steps), µs a step, the solve system(s) / the
whole step:

| case | before | after |
|---|---|---|
| boxes 10 000, falling | 26242 / 32403 | 25556 / 31855 |
| boxes 10 000, settled | 26719 / 29966 | 25019 / 27882 |
| planks 1000, settled | 2805 / 3234 | 2748 / 3172 |
| spheres 10 000, settled | 20754 / 23282 | 20241 / 22749 |
| boxes 10 000 locked, settled | 24823 / 29456 | 24020 / 28354 |
| stack 20 | 26 / 34 | 26 / 35 |

Level: the gathers and write-backs are within 10% of a few hundred µs
either way, and the 2 to 6% the solve gained is the solver's own time,
whose code didn't change: the runs were one after the other, not
alternated, so that is the machine's drift, not the port's.

**On `Passes`** (2026-10-03, get-emj.90). `solver` is now `prepare`,
`passes` and `finish`, as 2D's, the passes a program on `Passes`
(physics.md, "A mod"): colored (`Tuning`'s `order=colored`, the default
since the same day, decided ahead of threads so the default suite
validates it) Box2D's colors, 11 on a pile of 10 000, a physics change,
re-baselined (physics.md, "Colouring the 3D solve"); by level
(`order=levels`) the sweep as before bit for bit (ported with both
baselines and the fingerprint byte-identical), the levels being
`Passes`' colors (246 of them on the same pile, so a stage is 246
blocks). The states are `solver::lanes::State`, a body's
velocities, move and turn in 64 bytes, shared as sixteen relaxed atomics
(`Atom`); the kernels are generic over the view, matched once a call, as
2D's. Held to its plain path by `exact_test`'s
`the_mod_is_its_fingerprint_with_its_states_shared`, by level and
colored. Not split into the stage loop's own `Step`: 3D's stages include
the refresh under `Inertia::Substep` and the impulse sums, which 2D's
`staged::Step` doesn't name, so each mod keeps its own.

## Out of scope

- **Inferring order from flows** (above): possible later, within one mod.
- **A "levels of a tree" shape.** The hierarchy spike wanted one, but it
  isn't a flows user, and no other user has asked. It comes when one
  does.
- **Recycling across types**, and a `Take` that hands its allocation to
  another flow's `Make`.
- **Flows across frames or groups**, and flows in snapshots or replays.
- **Shapes over world storage.** Shapes run over what a system holds,
  normally a flow. A colored iteration over a relation's rows
  (parallel-relations.md, phase 3) stays deferred.
- **A shape that names its flow**, so the scheduler could pipeline one
  system's blocks into the next's (get-znt.23). Shapes take slices.

## Open questions

- **Open question:** a maker whose mod has failed (and so is skipped)
  leaves its readers with no value, and each reader fails at fetch: one
  failed mod fails every mod downstream. Skipping a system whose flows
  weren't made would contain it.
- **Open question:** why the settled pile's `prepare` was 5 to 7% slower
  on recycled allocations than on fresh ones in the spike (get-znt.27).

[^stand-in]: *(History, 2026-10-02.)* The spike built flows on the ECS's
    public API without changing it: each flow declared a marker event, its
    parameters declared that event's queue, and the access kind rode in the
    declaration as a count of `Dt` leaves, read only by the plan check. Every
    use but `See` got an apply node that published nothing, and the values
    lived beside the world in a store found by the world's address.
    `ParamDecl::Flow` and the world's store replace all three.
