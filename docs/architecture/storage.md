# Storage and concurrent structural change

**Status: built, on one thread.** The `engine/ecs` crate is the engine's
world, run by the sequential scheduler. The parallel executor exists in
`engine_ecs::harness`, for tests and the benchmark; running it in the
loader is system parallelism (step 2, get-znt.5). It replaced one sparse
set per component, and structural changes and events applied at phase
boundaries (see [scheduling.md](scheduling.md)).[^landing]

## Goals

Decided 2026-09-23:

- **No stop-the-world points.** Structural changes (spawn, despawn, insert,
  remove) and events are applied while other systems keep running, as soon
  as nothing they touch is in use. Systems declare everything they read and
  write, which is what makes this possible.
- **Archetype storage in pages**, with **sparse-set storage as a
  per-component alternative**, chosen by the component, as flecs does.
- **Safe Rust, with the invariants in the types.** A small unsafe core is
  acceptable, and is tested aggressively. Every unsafe line added is testing
  burden taken on, so the core stays as small as it can be.
- **The same frame as the sequential scheduler.** Whatever runs in parallel,
  the outcome matches running the plan in order, so lockstep games replay
  exactly and a parallel bug shows up as a diff.

## Phase boundaries become dependency edges

Applying a structural change, or publishing an event, is a node in the
frame's dependency graph, like a system:

- A system that queues a change gets an **apply node** after it. Its
  footprint is what the change touches: the tables (or sparse sets) involved
  and the entities' locations.
- A later system in the plan whose footprint overlaps an apply node's runs
  after it. Everything else carries on, in this phase or the next.
- Publishing a system's events is a node after it; readers later in the
  plan depend on it. Readers earlier in the plan see the events next frame,
  as now.

Every edge comes from the plan, so the result is the sequential frame's:
each system sees exactly the changes and events of the systems before it in
the plan that overlap it. Phases stay as ordering constraints; they stop
being barriers.

### Structural changes go through query rows

An apply node's footprint has to be known before the system runs. So a
structural change is made through the **row** a query hands out with each
entity, and the query's type lists the changes its rows may make:

```rust
Query<Data, Filter, Changes>
//             Changes: Adds<..>, Removes<..>, Despawns, or a tuple of them
```

A row only comes from its query, so **the query's tables bound the change,
by construction**: nothing to declare separately, and no insert outside the
bound can be written. `query.get(e)` gives the row of an entity the system
didn't iterate to (one kept in state, or named by an event), if the query
matches it; a query matching everything (`Query<(), (), Adds<T>>`) is how a
system inserts anywhere, and its cost shows in its signature. Spawning,
which isn't about an existing row, is a `Spawner<(A, B, ..)>` parameter
naming the table it fills.

Every change is optional per row: declaring `(Removes<Frozen>, Adds<Wet>)`
permits either, both or neither. A row's changes are applied in the order
it made them, one log per system: remove then insert moves the entity to
its final table; insert then remove of one component leaves it removed.

Decided 2026-09-23, replacing a first design of separate typed parameters
(`Inserts<T, Target>`) that restated the query's bound as a second
declaration.[^inserts] Free-form `cx.commands()` is left to exclusive
systems, and to code outside a frame (hooks, message handlers).

### What a change is visible to

Decided 2026-09-23, the sequential frame's rules:

- **Not to the system that made it**, even later in its own loop. It lands
  in the apply node, after the system returns.
- **To every system after it in the plan, the same frame**, in this phase or
  a later one, in any mod: each such system's dependencies include the apply
  node. Systems before it in the plan see it next frame.
- **Filters count as touching the component.** `Without<Burning>` reads
  `Burning`'s membership, so it orders after the apply like any reader. Only
  systems that can't observe the change skip the wait.
- **An insert on an entity that has the component replaces the value**; one
  on an entity that has died is dropped.

"After in the plan" includes the tie-break (load order), which a hot load
can change. A system that relies on another's changes in the same frame
should say `.after(..)`. Flagging reliance on the tie-break is a job for a
game-side linter, not the engine.

## Walkthrough: walkers that catch fire

What a game writes, and what the engine does with it. Walkers that step
into lava start burning; a second system burns them down and puts them out.

### The components

```rust
// A table component: stored in archetype tables, in pages.
engine_api::component! {
    #[derive(Debug, Default, Copy)]
    pub struct Health: "game::Health" { pub hp: f32 }
}

// Comes and goes often, so it's sparse: adding or removing it doesn't move
// the entity to another table.
engine_api::component! {
    #[derive(Debug, Default, Copy)]
    pub struct Burning: "game::Burning", storage = sparse {
        pub dps: f32,
        pub left: f32,
    }
}
```

### The systems

```rust
impl Hazards {
    /// Sets walkers standing in lava on fire.
    fn ignite(
        &mut self, cx: &mut Cx,
        lava: Query<&Lava>,
        walkers: Query<(&Position, &Health), Without<Burning>, Adds<Burning>>,
    ) {
        let pools: Vec<Rect> = lava.iter().map(|(_, l)| l.area).collect();
        for (row, (pos, _)) in walkers.iter() {
            if pools.iter().any(|area| area.contains(pos)) {
                row.insert(Burning { dps: 5.0, left: 3.0 });
            }
        }
    }

    /// Burns them down, and puts them out when the fire runs out.
    fn burn(
        &mut self, cx: &mut Cx,
        clock: Single<&Clock>,
        fires: Query<(&mut Health, &mut Burning), (), Removes<Burning>>,
    ) {
        for (row, (health, fire)) in fires.iter_mut() {
            health.hp -= fire.dps * clock.dt;
            fire.left -= clock.dt;
            if fire.left <= 0.0 {
                row.remove::<Burning>();
            }
        }
    }
}

impl Mod for Hazards {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("ignite", Self::ignite);
        s.add("burn", Self::burn).after("hazards::ignite");
    }
}
```

In the mod's code:

- **Changes go through the row**, inside the query loop. The row writes to
  the system's own log, not the world, so it needs no lock, and the query
  is iterated without `cx`, which stays free for logging and spawning.
- **The query's type says what its rows can change.** `row.insert(Health {
  .. })` on a walkers row is refused: `Health` isn't in its `Adds`. A
  change no query declares needs an exclusive system.
- **The declaration is the signature.** `ignite` reads `Lava`, `Position`
  and `Health`, and adds `Burning` to walkers; `burn` reads `Clock`, writes
  `Health` and `Burning`, and removes `Burning`. `systems()` only places
  them: phase, `after`, `before`.

### The frame

With `hazards::ignite`, `hazards::burn` and `render::draw_health` (reads
`Health`) in `update`, and `physics::integrate` (writes `Position`) in
`simulate`, the frame's graph is:

```
ignite ──► apply(insert Burning) ──► burn ──► apply(remove Burning)
   │                                   │
   │                                   └──► draw_health   (reads Health, which burn writes)
   └──► physics::integrate                  (writes Position, which ignite reads)
```

- **The apply node touches only `Burning`'s sparse set.** No entity changes
  table, so no `Position` or `Health` page is involved, and nothing using
  only other components waits for it: `physics::integrate` doesn't.
- **`burn` waits for it**, being later in the plan and touching `Burning`,
  so it sees this frame's fires. Had `burn` come first in the plan, it would
  see them next frame.
- **No phase boundary is involved.** A `late` system touching only `Sprite`
  runs as soon as its own dependencies are done, even while `burn` is.

### If `Burning` were a table component

Then an insert moves the entity from the `(Position, Health)` table to
`(Position, Health, Burning)`. The apply node's footprint is the pages
holding those rows in both tables, and the entities' locations, which
overlaps `physics::integrate`'s writes to `Position` in those tables: one of
them waits for the other, in plan order, and the frame is less parallel.
That is why a component that's added and removed often should be sparse,
and why the choice is the component's.

### Iterating

`walkers.iter()` walks every page of every table matching `(Position,
Health)`, skipping entities in `Burning`'s sparse set. Each page hands the
mod plain `&[Position]` and `&[Health]` slices, in its own code, instead of a
host call per entity. Data parallelism (step 3) adds a page-level form that
hands whole pages to workers.

[^inserts]: *(History, 2026-09-23.)* The first design declared structural
    changes as separate parameters, `Inserts<T>`, `Removes<T>`,
    `Despawns<With<..>>`, and bounded a table insert with a second type
    parameter, `Inserts<T, With<..>>`, checked against every insert at
    runtime; an unbounded one held up every later table system until it ran.
    Moving changes onto the rows of the query that yielded them made the
    bound implicit and exact, and the check unnecessary.

### Services don't touch the world

Decided 2026-09-23: a service method is a function of its provider's state,
transient part and arguments, with no world access. A call's footprint is
then just the provider's state, which the scheduler orders like two systems
of one mod, and no call can need a guard its caller holds. Changing the
world is done by whoever holds the query: the caller itself, or a system
of the provider's that an event reaches. The platformer's `Rules` become
the events `Hurt` and `Bounce`.

**Later (get-znt.13):** service methods that declare parameters like a
system, joining the caller's footprint, for a call that must read the world
and answer. Deferred until a real case needs it.

### Sending events is declared too

A system sends events through an `EventWriter<E>` parameter, since a reader
later in the plan has to know, before the sender runs, whether to wait for
it. `cx.send_event` is for code outside a frame (hooks, message handlers);
those events are published when the next frame starts. An event is visible
to the systems after its sender in the plan, the same frame, like a
structural change, and to those before it the next frame.

### The world between frames

Hooks and message handlers use `cx.world()`, a direct view of the whole
world (spawn, insert, remove, despawn, get, for_each) that exists only
between frames, where the loader holds everything; in a frame, and so in a
service called from a system, it's refused. Exclusive systems, which would
get the same view inside a frame, are deferred.

## Landing

Decided 2026-09-23: the spike becomes the engine's ECS, as a crate shared by
the loader and every mod (`engine/ecs`). Mods use the world's Rust types
directly, over the Rust ABI, which the one-compiler rule already allows; a
change to the crate bumps `API_VERSION`. Components are identified across
builds by name and schema fingerprint, as before, not by `TypeId`. Systems
run on the sequential scheduler first; the parallel executor, testing the
unsafe core (Miri, fuzzing), and get-znt.11's performance work follow the
MVP.

**What landing showed** (2026-09-23): every mod, both games and the
tests moved to the row API. Pong's and the platformer's recorded routes
replay unchanged, the platformer's `Rules` service included once it became
events. The benchmark matches the spike's (sparse 0.41 ms, tables 0.81 ms
sequential at `WORK=0`). Two bugs only real mods could show:

- **Dropping the engine crashed.** The world declared the components,
  which keep each build's library mapped, before the tables holding their
  values, so the libraries went first (see
  [lore](../lore/drop-values-before-the-library-with-their-drop-code.md)).
- **A panicking system poisoned its guards**, and the next frame's
  `try_lock` read the poison as contention, failing whichever mod touched
  those components next. A poisoned guard is now taken like a free one: the
  failed system's writes stand, as a panicking system's always did.

## Archetype tables in pages

- A **table** holds every entity with exactly one set of table-stored
  components, a column per component. Its rows live in fixed-size **pages**.
- A **page** is the unit of borrowing: a task claims the pages it reads and
  writes, not whole components. An insert that appends to a table's last
  page conflicts only with whoever uses that page. Pages are also the unit
  of data parallelism (step 3), so `par_for_each` hands out pages.
- **Sparse-set storage** remains for components that come and go often
  (tags, short-lived state), where moving an entity between tables on every
  add and remove would cost more than it saves. A component chooses its
  storage in its declaration; queries join table and sparse components.
- The **archetype registry** only grows, so readers can use a prefix of it
  without synchronizing; a query updates its matched tables when the
  registry has grown.
- **Entity locations** (entity to table, page, row) are updated by every
  structural change. Each location is stored atomically, so disjoint
  changes update theirs concurrently.

**Later:** pages shared between versions of the world, copy-on-write
(`Arc<Page>` with `make_mut`), so readers never block writers. It would
also give pipelining (step 4) without an extract step. Treated as an
optimization on top of this design, not part of it.

## Where the unsafe is, and isn't

**No unsafe code whose correctness depends on concurrency.** That is the
line that keeps the testing burden bounded: unsafe code that is correct on
one thread can be tested exhaustively on one thread; unsafe code that
depends on interleavings needs model checking and luck.

- **Concurrency is safe Rust.** Each page has a guard (a `RwLock`, or an
  atomic borrow flag), taken once when a task starts, never per access. The
  graph means a guard is never contended; a failed `try_lock` is a scheduler
  bug, reported as a refusal. A task holds ordinary guards until it ends, so
  there are no scopes, and no barriers.
- **The unsafe core is type erasure.** The loader stores components it
  knows only as layouts and drop code from a mod's build, so a column of
  erased values, the page memory behind it, and turning a column into
  `&[T]` / `&mut [T]` once a component's layout is checked, are unsafe. That
  one module is all of it; every view above it is safe.
- **Mods iterate pages directly**, as `&mut [T]`, in their own code (over the
  Rust ABI, which the one-compiler rule already allows), instead of calling
  `extern "C"` per entity. That also removes the biggest cost get-8in expects
  to find.

### Testing the core

- **Differential tests** against a trivially correct model (a map of entity
  to component values): random sequences of spawns, despawns, inserts,
  removes, layout migrations and queries, applied to both, compared after
  each step.
- **Miri** on the core's own tests, to catch undefined behavior the
  differential tests can't see (aliasing, uninitialized reads, misalignment).
- **Fuzzing** the same operation sequences, coverage-guided.
- **The equivalence test** for the scheduler: pong and the platformer
  replayed under the parallel and sequential schedulers, frame by frame.

**Open question:** toolchain. Miri, and the sanitizers, need a nightly
compiler; the build is pinned to stable. Running them in their own hermetic
nightly toolchain, for the core crate only, is the likely answer, still to
be tried under Bazel (deferred until after the MVP).

## Other open questions

- **Page size**, and whether it's per table.
- **Spawned entity ids** under parallel execution: ids reserved by
  concurrent systems depend on timing unless reservations are given out
  per system in plan order.
- **Service calls** from systems: the provider's footprint joins the
  caller's. Probably declared by the provider per service (see get-znt.5).
- **Layout migration** after a reload becomes a task over the affected
  tables, instead of happening on first access.

## Spike results

2026-09-23, `spike/ecs` (get-znt.10): tables in pages, sparse sets, typed
structural buffers, apply nodes, and a thread pool running the plan with
no barriers, on the walkthrough's scenario plus spawning, reaping and an
unrelated `ui` system. `./bazel run -c opt //engine/ecs:bench` (the
spike's, ported) prints the numbers and timelines below.

**What held up:**

- **The unsafe stays in one file.** `erased.rs` (276 lines, type-erased
  columns) is the only `unsafe`; tables, pages, sparse sets, guards,
  buffers and the scheduler are safe Rust. Concurrency safety comes from
  per-column and per-set `RwLock` guards taken once per node, and atomics
  for entity locations.
- **Parallel frames equal sequential ones.** Five 8-thread runs of 30 frames
  per variant match the sequential run exactly (compared without entity
  ids). Breaking any overlap rule fails a test, and a scheduler bug shows up
  as a contended guard, which fails the frame, never as silent corruption.
- **Structural changes run alongside unrelated work.** With `Burning` sparse,
  or table-stored with a target filter, `ui` starts at the top of the frame
  and runs through ignite, every apply node and physics. Sparse inserts
  don't hold up systems on other components (asserted in
  `schedule_test`'s readiness tests, which drive the scheduler node by node
  and don't depend on timing).
- **The scheduler reaches the critical path.** 8 threads: 34.7 ms sequential,
  22.5 ms parallel (sparse), against a chain of ignite (13 ms) then physics
  (12 ms) on the same tables that no system-level scheduler can shorten.
  Those times are the benchmark's busy work (`WORK=200` per entity); see
  below for the framework's own cost.

**Sharp edges found:**

- **An unbounded table insert or remove holds up everything after it in the
  plan until its system has run**, since before then its source could be any
  table: `ui` couldn't start until `burn` had run, and the frame took 25.1 ms
  instead of 22.4. *(Resolved by the row API, below: the changing query
  bounds it.)*
- **Despawning orders against every reader of any sparse set**, because
  iterating a sparse set checks who's alive. It couples despawns to systems
  on unrelated components. Worth a design pass: a per-set view of liveness,
  or deaths that become visible to sparse iteration only at the apply.
- **Systems on the same tables form a chain** that system-level parallelism
  can't break: here the frame is bound by it. That is where data parallelism
  over pages (step 3) pays; at system level, page borrowing is equivalent to
  table-level, as expected.
- **A panicking node hung the frame**: the other workers waited for it
  forever. Now a panic fails the frame, and the executor re-raises it.
- **Buffers are applied by declaration, not by call order.** A system that
  spawns an entity and inserts onto it, with its `Inserts` declared before
  its `Spawns`, has the insert applied first, to an entity not yet placed,
  and dropped. *(Resolved by the row API: one ordered log per system.)*
- **Spawned entity ids depend on timing** under parallel execution, as
  expected; outcomes don't.
- **Readiness is rechecked from scratch** on every scheduling decision
  (every node against every earlier one, per table). Fine for ten nodes;
  hundreds of systems will need the static edges computed once per frame,
  with only apply bounds updated.

**Second iteration: the row API.** Systems are plain functions whose
parameters are the declaration (`Query<Data, Filter, Changes>`,
`Spawner<B>`), each parameter holding its own guards; changes go through
rows into one ordered log per system. The same scenario, with `ignite`
adding `Burning` through walkers' rows or through a query matching
everything:

| `WORK=200`, 8 threads | sequential | parallel |
|---|---|---|
| `Burning` sparse | 36.7 ms | 24.5 ms |
| table, through walkers' rows | 37.0 ms | 23.4 ms |
| table, through a query matching everything | 35.6 ms | 25.8 ms |

The coscheduling is as before: `ui` runs through the whole frame when the
change is bounded by the walkers' query, and waits for `ignite` when it goes
through a query matching everything. Tests (16) cover the readiness claims,
changes through rows (call order, adds and removes on one row, refused
undeclared changes, a system not seeing its own changes, `get` for entities
from elsewhere, conflicting queries refused at registration) and
sequential equivalence; four mutations of the new code are all caught.

New sharp edges:

- **A query iterates with `for_each(|row, items| ..)`, not `for .. in`.** A
  loop's items must all be alive at once, and a query writing both a table
  component and a sparse one (`(&mut Health, &mut Burning)`) can't hand out
  such items in safe Rust: two `&mut` into one page, or one sparse set, look
  aliased to the borrow checker. A closure gets each row's items one at a
  time, which is safe. An iterator is possible for table-only queries, and
  for the rest with unsafe code, which is the testing burden we're
  avoiding.
- **A row exists only for an entity that's placed.** A spawned entity gets
  no row until its apply, so the only way to give it components is its
  bundle. That removes the spawn-then-insert ordering problem rather than
  solving it; a spawn with a conditional extra component needs two bundles.
- **Checking `Adds` at compile time isn't done**: `row.insert::<T>` compiles
  for any component and is refused when it runs. A type-level membership
  check (index-inferred, as frunk does it) is the next step to try.
- **The framework's own cost first doubled** (`WORK=0`: 0.49 ms to 1.00 ms
  sequential), all of it implementation, none of it the API: items were
  fetched per row instead of per page, `burn` probed `Burning` for all
  20,000 walkers instead of walking `Burning`'s set, sparse sets were hash
  maps, the exact-footprint replay was quadratic in the rows it moved, and
  table moves took guards in and out of a map per column. With those fixed:

  | `WORK=0`, sequential | first API | row API |
  |---|---|---|
  | `Burning` sparse | 0.49 ms | 0.42 ms |
  | `Burning` in tables | 0.59 ms | 0.75 ms |

  `for_each` takes each term's slice once per page; a query walks its
  smallest source (a sparse set, when that's fewer rows than its tables);
  sparse sets index by entity in an array. The table case's remaining cost
  is mostly the moves (`apply(ignite)`, 0.26 ms), each of which still
  builds its destination's component list and looks it up under the table
  registry's mutex; caching the destination per table and component (the
  archetype graph's edges, as flecs and Bevy do) is the next fix
  (get-znt.11).

**The framework's own cost** (`WORK=0`, 20,000 walkers and 20,000 labels,
10 systems): 0.49 ms a frame sequential, 0.78 ms on 8 threads. Table
queries run at about 1.6 ns an entity (`physics`, 0.032 ms). The rest,
optimizations for later (get-znt.11), none of which changes the design:

- the parallel executor loses to sequential on light frames: it spawns its
  threads every frame (the first node starts ~50 µs in), sends every node
  through one mutex and condvar, rechecks readiness from scratch, and hands
  even microsecond nodes to other threads. A persistent pool, edges
  computed once per frame, and running small nodes inline;
- the sparse `Without` filter costs six times a table query (`ignite`,
  0.19 ms), from a `HashMap` lookup per entity: index sparse sets by entity
  in an array;
- table moves (`apply(ignite)`, 4,000 rows in 0.23 ms) clone the table's
  component list and look up guards in a `HashMap` per row.

**Not done yet:** Miri on `erased.rs`, which needs a nightly toolchain;
events as publish nodes (the same mechanism as apply nodes); exclusive
systems and mod state conflicts.

[^landing]: *(History, 2026-09-23.)* Designed here as a draft, prototyped
    as `spike/ecs` (see [Spike results](#spike-results)), then landed as
    `engine/ecs`. The spike was removed once every test of it had moved
    over.
