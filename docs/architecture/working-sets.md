# Working sets

**Status: proposed** (2026-10-02, get-8m9). Nothing here is built. This
doc asks whether the solver's per-step copy of bodies is a workaround for a
storage feature the ECS lacks: a dense **working set** that storage keeps,
of chosen entities and columns, with an index by entity that the world
maintains. Two spikes measure it, both bench targets rather than engine
code:

- `//engine/std/physics2d/compare:working_set_spike` (`working_set_spike.rs`):
  the copy as built, against seven other ways, on the engine's own world.
- `//engine/ecs:hierarchy_spike` (`tests/hierarchy_spike.rs`): the second
  user, transform propagation down `ChildOf`.

**The recommendation, in short:**

- **Storage shouldn't own the copy.** Every way measured still copies,
  since the solver's layout is physics's own. What a kept working set adds
  is two things:
  - an index that is kept rather than rebuilt, worth 6 µs a step at 10 000
    bodies;
  - **an order**: bodies in entity order. That makes the solver 3% faster
    on the settled pile of 10 000 at one thread, and 9.5% at 8 threads. A
    per-step renumbering in the mod gets the same order.
- **Most of what the copy can shed is physics's own business** (phase 1,
  no ECS change, bit for bit). On the settled pile, of a solve system of
  about 5000 µs at one thread:
  - page walks with buffers kept between steps: 73 µs;
  - bodies renumbered in entity order: 140 µs off the solver, for about 67
    of renumbering.
- **The ECS gets one small shared piece** (phase 2): the entity-to-index
  map, written three times today (`Slots`), as one safe type in
  `engine_ecs`.
- **Optional query terms are worth something, but not much** (phase 3):
  9 to 41 µs off the solve's copy beyond page walks, and the collider
  gather's four queries become one. They are a change to the query
  language, so the decision is the user's.
- **A kept set waits** (phase 4). If physics or a second user comes to
  need one, the shape is already in storage: a sparse component is a dense
  array with an index by entity and swap-remove. What's missing is a dense
  view of it.
- **The second user, hierarchy propagation, wants an order more than a
  copy.** A kept working set beats the idiom a game writes today only
  where indices are scrambled (44 against 66 µs) or few transforms change
  (20 against 30). It loses where everything animates (44 against 30), and
  wherever links change (100 to 111 against 30), until its order is
  patched rather than rebuilt. What the idiom's speed actually depends on
  is `ChildOf`'s order happening to be topological, so the feature that
  would serve it is a depth order (Flecs's `cascade`), not a copy.

## The problem

The 2D solve (`solve` in `engine/std/physics2d/lib.rs`) does this every
step:

1. Walks the awake bodies, appending a `SolverBody` for each (velocity,
   inverse mass masked by kind, the step's gravity) and its entity.
2. Builds `Slots` from those entities: a vector by entity index giving each
   body's place.
3. Walks the turning bodies a second time, looking each up in `Slots`.
4. Walks the contacts in pair order, mapping each end through `Slots`.
5. Solves on the copy.
6. Writes back: velocities, positions where they moved, rotations and
   spins where they changed, impulses.

The copy is measured and justified. Solving in world pages was 9 to 16%
slower ([physics.md, "What the ECS
costs"](physics.md#what-the-ecs-costs); parallel-relations.md, "(c)
Solving in world storage"). Those measurements answered whether the
solver can work in pages; they didn't ask whether storage should own the
copy. Whether storage kept in one block a column would let it solve in
place instead is [contiguous-columns.md](contiguous-columns.md). The user's suspicion (2026-10-02): "it feels like it's a workaround
for a missing feature in the storage and maybe other things would also
want that missing feature."[^hypothesis]

The candidate feature is a declared system parameter:

- a set of entities and some columns, possibly with derived ones;
- dense arrays of them, with an entity-to-dense-index map that the world
  keeps incrementally;
- the map kept by entity, so that the spatial re-sort doesn't disturb it;
- the columns the system declares it writes, written back by the world;
- visible to the scheduler, to snapshots and to reloads, because it's
  declared.

## The evidence, tested

Each piece of evidence for a missing feature was measured, on the
turning default (2D, one thread, `--config=bench`, `taskset -c 0-7`):

| claim | what the spike found |
|---|---|
| `Slots` is written three times and built twice a step | **True.** Copies in physics2d, physics3d and `query_bench`; lib.rs builds one in eight places, two of them over every body (`find_contacts`, `solve`). Building one costs **6 µs** at 10 000 bodies and 3 at 5050, about 0.1% of the step. |
| colliders take four queries because queries have no optional terms | **True**, and it's an ergonomic cost more than a time cost. The same shape in the solve (bodies, then turning bodies: two walks) costs 9 to 41 µs more than one walk would, when both are page walks (below). |
| the inverse mass is a derived value the world can't hold | **Half true.** `Body::inv_mass` is in the world. What the gather derives each step is the inverse mass masked by kind (0 unless dynamic), the step's gravity (`g · gravity_scale · dt`), the kinds, and the inertia (`inv_mass · Collider::inertia_per_mass`). Deriving them is a few flops a body. Keeping them saved 44 µs, but only against a gather that already writes out of order (below). |
| about 150 µs of copying in and out, hand-built twice over | **Out of date, and mostly not bodies.** The 150 µs was the columns scene before rotation. On today's turning default the settled pile's solve gathers for 413 to 431 µs and writes back for 253 to 259 (in the mod, the median of 30 steps, three runs). Of that, bodies are about 54 + 101 µs in and 120 out. The rest is contacts, transposed into `Constraint`s and `Points` and written back, which no body working set touches. |

So the copy is real, and duplicated. But the index, the part storage
would most obviously own, is the cheapest piece of it.

## How others do it

Read in their fetched source, 2026-10-02:

- Box2D v3.1.1, Jolt v5.6.0 and Rapier 0.36 (2D and 3D, whose `src/`
  trees are identical), from the comparison packages' Bazel repositories;
- Flecs 4.0.4, from a Bazel output base where it had been fetched.

Bevy and Avian aren't fetched anywhere on this machine and weren't read
again. Their row repeats parallel-relations.md's reading of 2026-09-29.

| project | the dense set's index | kept? | the solver works on | derived values | written back |
|---|---|---|---|---|---|
| **Box2D** | `b2Body::localIndex` into the awake solver set (`body.h`) | **yes**: appended at creation, swap-removed with the moved body's index fixed up (`b2DestroyBody`, `body.c:410-440`), moved between sets on sleep and wake (`b2TrySleepIsland`, `b2WakeSolverSet`) | **the storage itself**: `stepContext->states = awakeSet->bodyStates.data` (`solver.c:1234`). A contact's body indices are stamped again every step in `b2CollideTask` (`world.c:398-405`) | stored: `invMass`, `invInertia`, `center` in `b2BodySim`, updated when mass or transform change (`b2UpdateBodyMassData`) | within its own storage, every step (`b2FinalizeBodiesTask`: pose deltas into the sim, boxes, sleep, events) |
| **Rapier** 0.36 | `RigidBodyIds::active_set_id` into the awake island's body list | **yes**: swap-remove and fix-up (`island_manager/manager.rs:92-103`), and an **epoch** bumped on any renumbering so that caches holding indices can tell they're stale (`manager.rs:31-34`) | **a copy, rebuilt every step**: `SolverBodies` cleared and refilled (`staged_island_solver/helpers.rs:56-74`, `worker.rs:41-104`, `SolverBodies::copy_from`) | stored on the body (`effective_inv_mass`, `effective_world_inv_inertia`) and recomputed every step after integration (`substep.rs:155`) | every step (`worker.rs:944-1038`) |
| **Jolt** | `MotionProperties::mIndexInActiveBodies` into an id list | **yes**: swap-remove and fix-up (`BodyManager.cpp:438-482`) | **in place**, through `Body*` (`AxisConstraintPart.h:59-66`). The active list holds ids, not data | local inverse mass and inertia stored; the world inertia computed per use and baked into each step's constraint parts | nothing to write back |
| **Flecs** | none by entity: a query cache of matched *tables*, kept by observers on table creation and deletion (`cache.c:1004-1063`) | per table | in place, table by table | none kept by the core | – |
| **Avian** (not read again) | `SolverBodies`, a resource of dense bodies after `b2BodyState` | – | its copy | – | per step |

What it says:

- **Everyone that keeps an index by entity is its own storage, or still
  copies.** Box2D and Jolt keep the index because their arrays *are* the
  bodies. Rapier keeps it and still rebuilds the dense copy every step: its
  kept index buys a stable order and caches keyed by index, not a cheaper
  copy. That is the closest shape to the hypothesis, and it is what the
  spike's "kept index" is.
- **The swap-remove index is the same everywhere**, and `engine_ecs`
  already has it. `SparseSet` (`world.rs`) is packed values and entities,
  with a vector by entity index, swap-removed with the moved entry fixed
  up: Box2D's awake set without the solver. EnTT's sparse set is the same
  (docs/CREDITS.md).
- **Derived mass values are stored by all three engines** and refreshed on
  change (Box2D) or every step (Rapier). In an ECS that means change ticks
  on every input: `Body`, `Gravity`, `Tuning` and `dt`.
- **No ECS keeps a dense per-system copy.** Flecs keeps per-table caches
  and *orders* (`order_by` sorts a table's own storage; `cascade` groups
  tables by depth, `cache.c:446-461`). Optional terms are decided once per
  table (`set_fields` cached per match, `cache.c:549`).

## Options

The candidates, each with what it costs and what it would take. The
measurements are from the spike, below; "settled" is the settled pile of
10 000, at one thread unless marked.

| | option | what it costs or saves | what it would take | verdict |
|---|---|---|---|---|
| a | **the copy as built**: the index rebuilt from the walk each step, bodies in walk (spatial) order | the baseline | – | – |
| b | **one shared index type** in `engine_ecs` for the three `Slots` | nothing, either way | a small safe type | **phase 2** |
| c | **an index rebuilt each step by the world**: a parameter that does (a)'s walk and map for the system | (a)'s cost, packaged | an API. The gathered layout (`SolverBody`, `Spinning`) is physics's, so its code stays in the mod: live.md's "where does the derivation's code live" again | no: (b) removes the duplication, and nothing else changes |
| d | **an index kept by storage, by entity**, slots in arrival order with swap-remove (Box2D, Rapier, Jolt) | index 6 → 0 µs. The gather writes each row to its slot, out of walk order: +7 falling, +52 settled, +11 pyramid. Write-back looks each row up: +15, +10, +7. **Solver in entity order: −142 settled (3%), −98 at 8 threads (9.5%)**, −19 / −46 on the pyramid, 0 falling. Spawn and despawn: within −2 and +6 ns of 180 to 290 (the index's own insert 3 to 5 ns, swap-remove 7 to 9). The re-sort: nothing, by construction | `SparseSet` already has this shape. New: a dense view of a sparse component's packed values with its index, and dead entries purged before it's handed out | **deferred** (phase 4): (e) gets its order for less code |
| e | **renumbered in entity order each step**, by the mod | the solver's gain as (d). Renumbering cost the spike +57 to +67 µs at 10 000 (a scan of the by-entity vector and a move of each gathered value), +29 on the pyramid | physics alone. A cheaper renumbering (physics.md measured 25 to 29 µs by first touch, 2026-09-24) would keep the falling pile from losing | **phase 1** |
| f | **derived values kept** with (d), refreshed when `Body` was written | the bodies' gather 49 µs settled, against (d)'s 93 and (a)'s 41 | change ticks on every input. The spike watched `Body` only; `Gravity`, `Tuning` and `dt` would need it too | with (d), deferred |
| g | **the slot as a column** in the bodies' own rows | write-back as (a)'s; gather −10 against (d) settled | a column in every body's row, kept by every structural change, moved by every re-sort | **no** |
| h | **the index found from entity locations** (table, page, row) | 7.1 ns a pair against 1.2 (physics.md, "Tried, and slower") | – | **no** |
| i | **optional query terms**, one walk over bodies and turning bodies | one page walk against two: −9 falling, **−41 settled**, −20 pyramid. As a per-row walk (`for_each`) its gather was *slower* than the two as built (+17 to +18) | query language and footprints (`graph.rs`), the user's decision | **phase 3** |
| j | **page walks** for the solve's gather and write-back (as built, `for_each`), buffers kept as (k) | **−31 falling, −73 settled, −15 pyramid**; −21, −30, −6 of it against (k) alone | physics alone | **phase 1** |
| k | **buffers kept** between steps (fresh vectors as built) | −10, −43, −9 | physics alone (its `Transient`) | **phase 1**, with (j) |
| l | **solving in world pages** | +9 to +16% (parallel-relations.md, "(c)") | – | **no** |

## Spike results: physics

`taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:working_set_spike`,
2026-10-02, Ryzen 9 7950X, one CCD, the machine otherwise idle (load under
1).

The spike runs the scene mod in the engine, turning, to a step. Then, on
that world between frames, harness systems (`engine_ecs::harness`, the
engine's own queries over its own storage) gather the solver's input each
way, solve it, and write it back. Every way's solve is asserted bit for bit
equal to the copy as built's, body by body by entity. After each way's
write-back the world is put back untimed, so every way sees the same world.
The copy here runs on one thread (the mod splits its walks over workers
when it has them). Its contact gather reads colder caches than the mod's,
where the merge has just written the contacts, so contacts take about 90
µs more here than in the mod; that cost is the same in every way.

µs, the median over three runs of each run's median of 21; gather and
write-back are the bodies', turning bodies' and contacts' (contacts are
gathered, not written back, the same in every way):

| way | pile falling, step 31: copy / solver | pile settled, step 430: copy / solver | pyramid 5050, step 630: copy / solver |
|---|---|---|---|
| (a) as built | 250 / 773 | 621 / 4457 | 367 / 2902 |
| (k) buffers kept | 241 / 775 | 578 / 4413 | 359 / 2906 |
| (j) two page walks, buffers kept | 219 / 775 | 548 / 4410 | 352 / 2902 |
| (i) one page walk (optional terms), buffers kept | 210 / 777 | 507 / 4409 | 332 / 2897 |
| (e) sorted by entity each step | 308 / 774 | 638 / 4282 | 382 / 2884 |
| (d) kept index, by entity | 257 / 776 | 624 / 4268 | 369 / 2881 |
| (g) slot column | 242 / 774 | 603 / 4288 | 361 / 2886 |
| (f) kept index, derived kept | 241 / 775 | 583 / 4282 | 365 / 2880 |

The copy's parts for (a), settled: index 6, bodies 41, turning bodies 100,
contacts 354, write-back 118.

The solve across 8 kept threads (`THREADS=8`, the pool warmed before each
solve; one run of 21):

| way | settled: copy / solver | pyramid: copy / solver | falling: solver |
|---|---|---|---|
| (a) as built | 683 / 1034 | 408 / 641 | 287 |
| (j) two page walks | 635 / 1028 | 366 / 640 | 282 |
| (i) one page walk | 600 / 1034 | 368 / 635 | 281 |
| (e) sorted by entity | 728 / 934 | 408 / 590 | 284 |
| (d) kept index | 718 / 937 | 420 / 592 | 282 |
| (f) derived kept | 671 / 929 | 426 / 594 | 282 |

What it shows:

- **The order is the lever, not the copy.** Every entity-ordered way
  solves the settled pile in 4268 to 4288 µs, against 4409 to 4457 for
  every way in walk order: about 140 µs, 3%. At 8 threads it's 929 to 937
  against 1028 to 1034, 9.5%. On the pyramid it's 0.7% and 7%; falling,
  nothing. Contacts are in pair order, which is entity order, so bodies in
  entity order are touched nearly in sequence; in spatial order they're
  scattered.
  physics.md measured this before, on the solver of 2026-09-24: 750 against
  791 µs, with renumbering costing as much as it saved. The solver has
  since grown (rotation, substeps, colors), so the gain grew with it and
  now pays for the renumbering on a settled pile.
- **A kept index gets the order, then pays for it in the gather.** Rows are
  walked in spatial order and written to slots in entity order, so the
  writes scatter: the bodies' gather is 41 µs in walk order, 93 by slot.
  On the settled pile, (d) and (e) come out the same; falling, (e) loses
  its renumbering's 57 µs and (d) doesn't.
- **Page walks and kept buffers are free gains** that need nothing from
  the ECS. Physics.md found that "where rows are only read, a page walk is
  no faster than `for_each`" (2026-09-24, before rotation); for these
  walks of four and six terms it is faster.
- **Optional terms save the second walk's lookups** (the turning bodies'
  100 µs becomes part of one walk), but only as a page walk. Through
  `for_each`, six terms a row cost more than two walks of three and four.
- **Derived values kept** save what (d)'s scattered writes cost (49 against
  93 µs for the bodies), not more: deriving is a few flops; the writes are
  the cost.

**Index upkeep**, on a fresh world of 10 000 bodies, each frame despawning
and spawning some spread over the pages, ns per spawn or despawn, the
median of 41 frames, three runs:

| a frame | the world alone | with the kept index's insert and swap-remove |
|---|---|---|
| 300 despawned, 300 spawned | 278 to 289 | 284 to 291 |
| 3000 and 3000 | 180 to 188 | 181 to 187 |

The index alone, on a million entities: 3 to 5 ns an insert, 7 to 9 ns a
swap-remove with its fix-up. A spatial re-sort moves rows between pages
and never touches an index by entity, so it costs that index nothing,
which is also why `Live` keys its records by entity (live.md, "How it
works").

## The second user

**The candidates** (2026-10-02, the repo as it stands):

| candidate | in the repo | what it would want | likely to be real |
|---|---|---|---|
| joints, ragdolls, springs between bodies | none | the solver's own bodies, solved with contacts (Box2D colors joints and contacts in one graph) | yes, but **the same user**: joints join physics's working set, they don't make a second one |
| cloth, particles, position-based ropes | none | physics's shape exactly: many passes over a relation, dense by index | only if a game needs them; none does |
| skeletons | none | a hierarchy (below), animated every frame | as a hierarchy |
| `ChildOf` transform propagation | `ChildOf` exists, kept in parent order ([relationships.md](relationships.md)); the platformer's levels and the demo spawner's box use it, with nothing propagated; a `Transform` is waiting for a core-types mod (get-emj.10) | each node's global from its parent's | **yes**: every scene graph has it |
| navigation graphs | none | a graph search, not a pass over columns | no |
| pong, platformer, mods | a handful of `Query::with` lookups a frame (a ball's paddle, a walker's player) | nothing at this scale | no |
| physics's own other sets | `find_contacts`' colliders (statics included, a second `Slots`); sleeping's islands (`fall_asleep`, small `Slots`) | another set over other entities, or small ones | inside physics, the same user again |

**Transform propagation** was chosen and built as `//engine/ecs:hierarchy_spike`,
in the smallest honest form. A node has a `Local` (an offset and a turn)
and a `Global`. A root's global is its local; anyone else's is its
parent's global composed with its own local. Four ways run on the same
world, and every frame each one is checked bit for bit against a recursive
reference:

- **hand, by level**: what a game writes with this ECS today. A query can't
  read a parent's `Global` while it writes its children's in the same
  table, so globals go into a vector by entity index, roots first. Then
  the children are walked once per level, each taking its parent's from
  the vector once that's there.
- **hand, sorted each frame**: physics's shape. Gather, index by entity
  (`Slots`), find each parent through it, sort by depth, propagate in one
  pass, write back.
- **working set, kept**: the index, the depth order and each node's parent
  slot kept across frames, rebuilt only when a node arrives or leaves or
  a link is written. A frame gathers the locals into their slots,
  propagates and writes back.
- **kept, changed locals**: the same, gathering only the locals written
  since its last frame (change ticks).

`taskset -c 0-7 ./bazel run --config=bench //engine/ecs:hierarchy_spike`,
2026-10-02, µs a frame, the median over three runs of each run's median of
41:

| frame | forest | by level | sorted each frame | kept | kept, changed locals |
|---|---|---|---|---|---|
| every local written (animated) | 10 500 nodes, 2 deep | **30** | 158 | 44 | 81 |
| | 10 080 nodes, 5 deep | **30** | 120 | 44 | 78 |
| | 5 deep, indices scrambled | 66 | 126 | **44** | 79 |
| roots' locals written (carried) | 2 deep | 31 | 132 | 27 | **21** |
| | 5 deep | 30 | 120 | 25 | **19** |
| | 5 deep, scrambled | 65 | 126 | 26 | **20** |
| roots', and two leaves swap parents | 2 deep | **30** | 131 | 111 | 111 |
| | 5 deep | **30** | 120 | 100 | 100 |
| | 5 deep, scrambled | **66** | 126 | 106 | 106 |

**Indices scrambled**: the forest is spawned into indices freed in a
scattered order, as a world that has run a while hands them out, so a
child's index is as likely below its parent's as above.

What it showed:

- **It wants an entity-indexed map and an order.** Each parent must be
  found by entity, cheaply, and parents must come before children. The
  `Slots` shape serves the first in every way that wins: by level's vector
  by entity index is one.
- **The game's idiom is fast by accident.** `ChildOf` is kept in parent
  order. With fresh indices a parent's index is below its children's, so
  parent order is a topological order and one walk does every level: 30
  µs, five deep as two deep. Scramble the indices and it takes several
  walks: 66.
- **A kept working set wins only where a frame's work is small.** Where
  only roots move it's 25 to 27 µs, or 19 to 21 gathering changed locals
  only. With scrambled indices it's 44 against 66. It **loses** where
  everything animates on fresh indices (44 against 30), and wherever a
  link is written. There its rebuild, the spike's fallback, costs 100 to
  111 µs a frame, until its order is patched in place rather than rebuilt.
  Gathering only changed locals costs twice the plain gather when they all
  changed (62 against 26), since each row is then found through change
  ticks and the index.
- **Sorting each frame, physics's shape, is the worst here** (120 to 158):
  the work is one cheap pass, and the copy and sort dominate it.
- **The shape that serves it is an order.** A depth order kept by storage
  would make the idiom's one walk correct whatever the indices: about 30
  µs, as the fresh-index rows measure. Flecs's `cascade` does this
  (`group_by` on depth, `cache.c:446-461`). A kept working set would be a
  second order beside `ChildOf`'s, which the 2026-09-25 retrospective
  found to be the answer that didn't work ("One physical order per
  table").

So the second user wants the index type of phase 2, and an order that
belongs with ordered tables. It doesn't want storage to keep a dense copy
for it.

## Recommendation

**Keep the copy in the systems that make it, give them the shared pieces,
and let orders, not copies, be what storage grows.**

### Phase 1: the copy in physics, cheaper (physics only; get-emj.88)

All bit for bit, since none changes a value the solver sees, which the
spike asserts for each:

- the solve's body gather and write-back as page walks, into buffers kept
  between steps in the mod's `Transient`: −31 falling, −73 settled, −15
  pyramid at one thread;
- bodies renumbered in entity order each step, so contacts touch them
  nearly in sequence: the solver −140 settled at one thread and −98 at 8,
  the pyramid −19 and −46, falling nothing. The spike's renumbering costs
  57 to 67 µs at 10 000, so falling loses about that until it's cheaper:
  physics.md's first-touch renumbering measured 25 to 29 (2026-09-24).

**Projected** from the parts, each measured alone ((j)'s copy, plus the
renumbering's cost over (k), plus (e)'s solver): on the settled pile about
180 µs off the solve system's 5080 at one thread (3.5%) and 80 off 1717 at
8 threads (4.7%); falling, about 25 µs worse until the renumbering is
cheaper. The same
in 3D, measured there. No ECS change, no unsafe code, no baseline moves.

### Phase 2: one entity index in `engine_ecs` (safe; get-cp4)

A public type for the dense map from entity to index, with `Slots`'
contract: by entity index, generation checked, `O(1)`. It replaces the
two physics mods' copies and `query_bench`'s, and serves hierarchy's
by-level idiom and anything else that maps entities to a list. It's a
utility, not storage: no footprint, nothing kept.

### Phase 3: optional terms (the user's decision; get-9ck)

`Option<&T>` in a query, decided per table as Flecs decides its optional
fields, handed to page walks as `Option<&[T]>`. Measured on the solve: 9
to 41 µs beyond page walks. The collider gather's four queries become one.
It changes the query language and the footprint rules (an optional term
reads or writes `T` in the tables that have it), so it is the user's call,
and mostly for ergonomics.

### Phase 4: a kept set, if a user comes to need one (deferred; get-9m5)

The shape exists. A sparse component is packed values with an index by
entity and swap-remove, in the world, declared, and carried by a reload or
a snapshot like any component. What a kept working set would add to it:

- **a dense view**: a sparse-only query handing a system its packed
  entities and values as slices, with the index. It's safe code over the
  typed view `SparseSet` already uses;
- **dead entries purged** before the view is handed out (a despawn leaves
  one, invisible, until `purge_dead`);
- **derived values kept fresh** by change ticks on every input. For
  physics: `Body`, `Gravity`, `Tuning` and `dt`.

Against phase 1, physics would gain the renumbering's cost back (about 55
µs settled and 66 falling, the difference between (f) and (e)), less if
the renumbering gets cheaper. That doesn't pay for a new storage feature
today. It is worth building when a system's index is expensive to rebuild
against its work, or when something must persist between steps: a
hierarchy whose order is patched rather than rebuilt, or parallel
relations' kept colors (parallel-relations.md) if their per-body masks
come to be stored.

### Hierarchy: an order (get-qdi, after get-emj.10)

When a `Transform` exists, propagate it with the idiom (a by-entity vector
and one walk), and make the walk's order right by storage. That means
`ChildOf`'s tables in depth order: a depth kept in the link, or a key of
(depth, parent), patched when a link is written. Measure it then against
the 30 µs above.

## What stays out

- **The solver's layout.** `SolverBody`, `Spinning`, `Constraint` and
  `Points` are physics's transpose. Storage can't own the gather without
  owning that code, which is live.md's open question of where a
  derivation's code lives.
- **The contacts' copy**, about 260 µs in and 130 out on the settled pile
  in the mod: a transpose into constraints, not a working set of
  entities. It belongs with get-emj.76 (the stages around the solver).
- **Writes from many threads into a storage-owned dense array.** A kept
  set shared by a parallel solve needs what parallel-relations.md's phase
  3 needs: atomic views of a column, a small unsafe cast that is **the
  user's decision**, with Miri over it; or physics's own copy as relaxed
  atomics, as today. Nothing above needs unsafe code.
- **Reload and snapshots.** A per-step copy carries nothing between steps,
  so none of the retrospective's objections to state beside the world
  apply to it. A kept set (phase 4) would be a component, carried as one.
- **3D.** physics3d's solve has the same shape (`Slots` twice, bodies in
  walk order) and wasn't spiked: phase 1 applies there, measured there.

## Open questions

- **Open question:** why entity order helps the solve across threads three
  times as much as on one (9.5% against 3%). The guess is fewer cache
  lines shared between the threads' blocks; it isn't measured.
- **Open question:** the cheapest renumbering. By first touch in the
  contact walk, by a scan of the entity index (the spike's), or by keeping
  last step's order and fixing up what moved.
- **Open question:** whether a depth order for `ChildOf` can be kept as
  cheaply as parent order is, through reparenting, or needs a depth stored
  in the link.
- **Open question:** whether phase 3's optional terms should apply to
  `for_each` at all, given that per-row walks of many terms measured slower
  than two narrower walks.

[^hypothesis]: *(History, 2026-10-02.)* The question came framed as a
    likely missing feature, with four pieces of evidence: `Slots` three
    times, colliders gathered by four queries, a derived inverse mass, and
    150 µs of copying. The spike was built to test that framing rather
    than assume it. What it removed from the hypothesis is the copy
    itself: every engine read still copies or is its own storage, and every
    way measured here still copies. What it kept are the index type and an
    order.
