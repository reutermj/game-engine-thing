# Retrospective: physics in the ECS (2026-09-25)

The work after the [physics retrospective](2026-09-23-physics.md), from
c7be223 to b02d51e: taking the pillar's aim, physics expressible in the
ECS, at its word, then measuring whether that costs a physics engine its
performance.

What was built:
- **Spatial storage.** Space became a property of storage: tables holding
  a spatial key keep Z-order pages with boxes, and the broadphase and
  region queries walk them. See [spatial-storage.md](../architecture/spatial-storage.md).
- **Change ticks.** `Mut<T>` stamps a tick when written through, so
  re-sorts touch only what changed.
- **Fixed-rate phases,** a rate per phase.
- **Ordered tables** (`order = key`). Contacts and overlaps are entities
  stored in pair order, `ChildOf` is kept in parent order, and pre-solve
  hooks are ordinary systems. See [relationships.md](../architecture/relationships.md).
- **Cheaper structural changes** for every game.
- **Performance work** found by a bit-identical benchmark against the same
  step on arrays (`:tax`):
  - page lanes and a branch-free broadphase;
  - upkeep in proportion to change;
  - query walks over page slices;
  - pages split on Z-order blocks;
  - sleeping as tables of its own.
- **Measurements of parallelism,** the solver and the rest of the step.
- **Physics's verdict:** not a design blocker single-threaded. See
  [physics.md](../architecture/physics.md#what-the-ecs-costs).

| 10 000 bodies, µs per step, ECS / arrays | first measured | at the end |
|---|---|---|
| the columns scene, settled | 4738 / 1269 | 1359 / 1299 |
| the columns scene, falling | 1900 / 568 | 731 / 574 |
| a real pile, settled | – | 3059 / 3066 |
| a real pile, at rest | – | 2459 / 3022 |
| the whole pile asleep | – | 22 / – |

## What held up

- **Putting the problem into storage kept paying off.** The first physics
  retrospective's worst flaws were data beside the world:
  - an index physics rebuilt every step;
  - a contact cache in mod state;
  - entity lists in the games' mod state.

  Each became a property of storage (space, order, sleep), and each
  change removed code as well as time:
  - the index and the walkers' first-frame workaround went with spatial
    storage;
  - contacts in the world let the walkers read overlaps instead of
    re-testing geometry;
  - sleeping as a table, not a per-row flag, took a sleeping pile from
    580 to 22 µs.
- **A bit-identical baseline made every number mean something.** `:tax`
  runs the same step on plain arrays from a snapshot of the world and
  asserts both end the same bit for bit. So every gap it showed was the
  world's cost, never different work. Over two days it survived dozens of
  changes by eight agents, and it caught a scene that wasn't what anyone
  thought it was (below).
- **The design wasn't the bottleneck; missing measurements were.** Every
  suspected blocker turned out to be incidental once measured:
  - entities mapped to array indices by sorting and binary search (1.7 ms
    at 10 000);
  - lookups of a component most bodies lack;
  - SipHash in `Structural`;
  - footprints cloning component lists per spawn;
  - a query dispatching on each term's kind per row (5.4 ns a row, now
    1–2);
  - `f32::floor` compiling to a libm call;
  - bounds glue not inlined across crates;
  - pages split at the median key rather than on Z-order blocks, which
    doubled rows changing pages.

  None needed the design changed.
- **Mutation checks kept finding tests that couldn't see.** A few of many:
  - a player always on side `a` of every contact, so side handling went
    unchecked;
  - every collider part at `dy` 0;
  - one spawner per log;
  - removals always followed by a spawn that marked the table dirty
    anyway;
  - a table ordered by another key.

  Each was found because a break went uncaught, not by review.
- **The loader stayed bare.** Nothing in two days of storage and physics
  work needed the loader's help. Contacts, levels and sleep survive a
  reload as entities, like everything else.

## What fought back

### The benchmark scene was degenerate for two days

The pile every table was measured on settles, at 1000 and 10 000 bodies,
into separate columns that never touch: one contact per body, 331 islands
at 10 000. An odd count per row makes each column alternate circles and
boxes, and without rotation a circle on a box stays put. A real pile (a
box one unit wider) has 1.5 contacts per body and costs the solver twice
as much.

It surfaced only when the parallel-solver agent counted islands. The
single-threaded verdict survived the correction: settled, the ECS is even
with arrays on a real pile. But it survived because the ECS's broadphase
was 750 µs ahead there, while a cost the columns had hidden, the ordered
contacts table rebuilt whole every step, came to about 360 µs. The
settling tests (`physics_test`) use the same degenerate pile, so "a pile
comes to rest" has never been tested on a real pile.

### One physical order per table

A table can be in spatial order or one key's order, not both, like a
database's clustered index. The tension came up repeatedly:
- the demo spawner's bodies are `ChildOf` a box and have a position, so a
  range of them is a scan;
- a sweep-and-prune wants x order where storage keeps Z-order (measured:
  a global x sweep loses on square scenes);
- a solver in place would want dense, stable indices where rows move
  every step.

The answers that worked moved things between tables, which is the other
lever an archetype ECS has: sleeping bodies and resting contacts are
tables of their own. The answer that didn't was a second order kept
beside the first.

### What a system finds is as of when it found it

Walkers read overlaps a step late, from before the player respawned, and
killed it twice. Physics's facts are true at `find_contacts`. The fix was
scheduling, reading them between `find_contacts` and `solve`, and it
generalizes: order a reader after its finder. The platformer's schedule
test pins it.

### Built the simplest version, and a real scene found its limit

Ordered tables re-sort by rebuilding every column whenever the order
breaks. On the columns scene that was nearly free. On a real pile, where
some contact always begins or ends, it is the largest ECS-specific cost,
and it gets slower after other cores have written the pages. Splicing
(get-emj.31) was always the better design; it wasn't needed until the
scene was real.

### Parallelism through the ECS didn't scale yet

The solver parallelizes about 6× by graph coloring, on one CCD, and the
ECS needs nothing new for it. The rest of the step got slower on threads
through the ECS (0.8×) while the same split on arrays got 2.6× faster.
The leading causes:
- the ordered-table rebuild above;
- data moving between cores whenever a stage runs split and the next
  runs serial;
- a pool that was never pinned or warmed (get-emj.32).

Around them:
- A mod can't own threads.
- `engine_ecs` rules out unsafe code whose soundness depends on
  concurrency, so the pool belongs to the host.
- A `thread_local!` with a destructor that mod code touches keeps a build
  mapped.

None of it is shown inherent. None of it is shown to scale either.

### Results nobody explained

These are recorded rather than dropped:
- 24-row pages were erratic, worse than both 16 and 32 rows.
- The merged broadphase is about 10 µs slower than either branch alone.
- A per-contact test that was always false cost 30 µs at 10 000.
- Keeping the last parent while propagating a hierarchy cut lookups 8× and
  saved no time.
- A second store per write cost a nanosecond a row.

There is no profiler on the machine. `perf` would likely have settled some
of these.

## How the work went

- **Measure first, then decide** was the pattern that worked:
  - the relationships spike before choosing contacts' storage;
  - `:tax` before calling anything a blocker;
  - profiling inside a stage before changing it;
  - each cause of the solver's copy measured in isolation before deciding
    against solving in place.

  The early calls that didn't measure were the ones reversed.
- **The first recommendation for contacts sidestepped the ECS.** A
  physics-owned table measured best, and I recommended it before asking
  what it gave up: rows invisible to queries, one footprint for every
  hook, a second data model. The user asked "is that sidestepping ECS?",
  and closing the table's lead from inside the ECS became the work. The
  lesson is to put the spirit of the pillar next to the numbers, not
  after them.
- **Parallel agents in worktrees covered ground fast.** Eight
  investigations ran in three rounds, each owning one front and measuring
  against the same bench. Merging went well when the agent that wrote the
  code resolved its own conflicts, in order, onto the others' merged work.
  The friction:
  - worktrees under `.claude` broke `./bazel test //...`, hence the
    `.bazelignore` entry;
  - agents overwrote each other's scripts in the shared scratchpad;
  - each bumped `API_VERSION` on its own for unreleased work;
  - several rewrote the same doc sections;
  - some had no `bd`;
  - a shared machine meant medians, load detection, and ±5–10% noise
    throughout.
- **Knowing when to stop was the user's call.** After the verdict, the
  remaining optimizations went to the backlog, and the one open design
  risk (parallelism) was measured rather than built.

## What it suggests

What became of the last retrospective's suggestions:

1. **Contact response:** pre-solve hooks exist, since a system between
   `find_contacts` and `solve` may change a contact's `Response`. Several
   colliders per body is decided as compound data and not built
   (get-emj.8).
2. **A fixed step in real time:** done, as a fixed rate per phase.
3. **`Transform` as its own interface:** still open, and no longer a
   design question. A core-types mod owns it (get-emj.10).
4. **What the step is:** decided. A pipeline whose intermediate data
   (contacts, overlaps) is in the world, with a solver that copies bodies
   into its own layout each step, as Box2D, Bullet and Rapier do. The copy
   costs 18 µs (get-emj.11 is answered).
5. **Edge events and level geometry:** still open (get-emj.12).
6. **The index at load:** gone with the index; spatial storage is in
   order before the first frame.

New:

1. **Validate a benchmark scene before trusting it:** count contacts per
   body and islands, and look at it settled. Then give the settling tests
   a real pile.
2. **Ordered tables that splice** (get-emj.31): the one optimization a
   real scene showed is needed.
3. **Settle parallelism with the scheduler's pool** (get-znt.5,
   get-emj.32): pinned to one CCD and kept warm, before concluding
   anything about the rest of the step on threads.
4. **The correctness backlog the work turned up:**
   - restitution lost on speculative contacts (get-emj.19);
   - sleeping's known gaps, before it's on by default;
   - Miri and fuzzing for an unsafe core that grew (get-znt.14).
