# Storage and concurrent structural change

**Status: design draft, not built.** It replaces how the world stores
components (`engine/loader/world.rs`, today one sparse set per component) and
how structural changes and events reach it (today applied at phase
boundaries; see [scheduling.md](scheduling.md)). System parallelism (step 2,
get-znt.5) waits on it.

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

### Structural intent is declared, in types

An apply node's footprint has to be known before the system runs, so a
system declares the structural changes it may make, as parameters:

```rust
fn collect(&mut self, _: &mut (), cx: &mut Cx, coins: Query<&Coin>, gone: Despawns<With<Coin>>) { ... }
fn spawn_player(&mut self, _: &mut (), cx: &mut Cx, new: Spawns<(Player, Input)>) { ... }
fn wake(&mut self, _: &mut (), cx: &mut Cx, add: Inserts<Awake>) { ... }
```

Decided 2026-09-23: typed structural parameters replace commands in
non-exclusive systems. Free-form `cx.commands()` is left to exclusive
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
        &mut self, _: &mut (), cx: &mut Cx,
        lava: Query<&Lava>,
        walkers: Query<(&Position, &Health), Without<Burning>>,
        burn: Inserts<Burning>,
    ) {
        let pools: Vec<Rect> = lava.iter(cx).map(|(_, l)| l.area).collect();
        for (e, (pos, _)) in walkers.iter(cx) {
            if pools.iter().any(|area| area.contains(pos)) {
                burn.insert(e, Burning { dps: 5.0, left: 3.0 });
            }
        }
    }

    /// Burns them down, and puts them out when the fire runs out.
    fn burn(
        &mut self, _: &mut (), cx: &mut Cx,
        clocks: Query<&Clock>,
        burning: Query<(&mut Health, &mut Burning)>,
        out: Removes<Burning>,
    ) {
        let Some(dt) = clocks.iter(cx).next().map(|(_, c)| c.dt) else { return };
        for (e, (health, fire)) in burning.iter(cx) {
            health.hp -= fire.dps * dt;
            fire.left -= dt;
            if fire.left <= 0.0 {
                out.remove(e);
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

- **`burn.insert` works inside the query loop.** `Inserts<Burning>` is a
  handle to this system's own buffer, not to the world, so it needs no `cx`
  (which the loop is borrowing) and no lock (only this system writes it).
- **The types say what can change.** `burn.insert(e, Health { .. })` doesn't
  compile, and neither does writing through `Query<&Health>`. A change the
  parameters don't declare needs an exclusive system.
- **The declaration is the signature.** `ignite` reads `Lava`, `Position`
  and `Health` and inserts `Burning`; `burn` reads `Clock`, writes `Health`
  and `Burning`, and removes `Burning`.

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

`walkers.iter(cx)` walks every page of every table matching `(Position,
Health)`, skipping entities in `Burning`'s sparse set. Each page hands the
mod plain `&[Position]` and `&[Health]` slices, in its own code, instead of a
host call per entity. Data parallelism (step 3) adds a page-level form that
hands whole pages to workers.

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
nightly toolchain, for the core crate only, is the likely answer; the spike
should check it works under Bazel.

## Other open questions

- **Page size**, and whether it's per table.
- **Spawned entity ids** under parallel execution: ids reserved by
  concurrent systems depend on timing unless reservations are given out
  per system in plan order.
- **Service calls** from systems: the provider's footprint joins the
  caller's. Probably declared by the provider per service (see get-znt.5).
- **Layout migration** after a reload becomes a task over the affected
  tables, instead of happening on first access.

## Next

A spike, outside the engine: a standalone crate with tables in pages,
sparse sets, the graph with apply and publish nodes, and a small thread
pool, with the differential test and a benchmark against today's sparse
sets. It decides whether the fine-grained graph pays for itself before the
engine is moved onto it.
