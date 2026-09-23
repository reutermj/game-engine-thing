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
