# Spatial storage

**Status: built in `engine_ecs`** (`spatial.rs`, get-emj.15) after a spike
(`spike/spatial`, get-emj.14), with physics on it, and change detection
so re-sorts touch only what changed ([In the engine](#in-the-engine)).

Space as a property of storage, rather than an index beside it: a table
whose rows have a position keeps them in spatial order, so a page is a
neighborhood, and "what's near here" is answered by which pages a query
walks. It replaces physics's `SpatialIndex` (a copy of positions and
shapes, rebuilt every step, invisible to the scheduler) and its separate
broadphase grid. Motivated by the
[physics retrospective](../retrospectives/2026-09-23-physics.md) and the
pillar's aim that physics be expressible in the ECS; most engines keep
spatial structures outside the ECS, and this deliberately doesn't.

## The shape

- **Declared on the key.** `component! { pub struct Position:
  "physics::Position", order = spatial { .. } }`, with `SpatialKey`
  implemented: the box around a row from its key and, if the row has it,
  its extent component (`Collider`). Carried in `ComponentDesc` like
  storage, so every build that declares the component agrees, and
  changing it takes a restart. The bounds function is the declaring
  build's glue, like its drop function, and hot-reloads with it.
- **Spatial tables.** A table holding a key component (a position) and
  an extent (a collider) keeps its rows sorted along a Z-order (Morton)
  curve of the position's cell. Each page is a run of nearby rows and
  records the box around them.
- **Big bodies apart.** Rows larger than a threshold (walls, floors) are
  kept out of the order, in their own small list: in order, a floor's box
  stretches its page's box over the level.
- **Upkeep after writers.** A system that writes a spatial table's key or
  extent gets an apply node that rekeys what it wrote and re-sorts. Rows
  that cross a page move there; the rest are a reorder within a page. A
  writer doesn't see its own rows move, and everything after it does: the
  visibility rule for every other structural change.
- **Region queries are queries.** `q.in_region(r, |row, items| ..)` walks
  the pages whose boxes meet `r`, with the query's own footprint, so a
  spatial read is ordered after whatever moved things, like any read.
- **The broadphase is page pairs**: pages whose boxes meet, then rows
  within them. Static pages never re-sort, so their boxes don't change.

## Spike results

2026-09-24, `spike/spatial`: one table of bodies in three layouts, run
through the same simulation (the physics mod's real narrowphase and
solver, contacts in pair order), which must come out bit for bit the same;
four mutations of the layouts are caught by that agreement. The layouts:
today's (insertion order, a grid index rebuilt every step, and the
broadphase's own grid), pages per grid cell (2a), and Z-order pages (2b),
each with and without big bodies apart, and Z-order at 64, 16 and 8 rows
a page. `./bazel run -c opt //spike/spatial:bench`.

Per frame, µs, `-c opt`, one thread (Z-order with big bodies apart, 8 rows
a page, against today's):

| scene | broadphase | upkeep | queries (64) | respawn upkeep |
|---|---|---|---|---|
| pile, 1000, falling | 39 (86) | 25 (60) | 23 (45) | |
| pile, 1000, 400 frames | 48 (100) | 20 (59) | 20 (45) | |
| platformer | 0.4 (8.3) | 0.1 (4.0) | 3.0 (14.5) | 0.44 (4.9) |
| drift, 10 000 | 1012 (1089) | 653 (727) | 81 (46) | |

**What it decides:**

1. **Z-order over a grid.** Grid pages matched Z-order's broadphase and
   queries at 1000 bodies, but a cell per page means 286 pages for 1000
   rows (6377 for 10 000), mostly near-empty, and its upkeep (a map of
   cells) was double today's at scale. Z-order pages are full by
   construction.
2. **Automatic upkeep after every writer.** A one-row write (a respawn)
   re-sorts in under half a microsecond, against 4.9 µs for rebuilding
   today's index; after physics writes every body, upkeep is a third of
   today's rebuild. Nothing needs to wait for "the next re-sort".
3. **Big bodies must be kept apart.** Without it the grid's broadphase is
   6× slower and its queries 4×, since the largest body sets how far a
   cell's rows reach; Z-order suffers less but still. A list scanned by
   every query and page is fine for a level's few walls; a level of many
   big bodies would want a coarser second order.
4. **Spatial tables want small pages** (8 to 16 rows), where the engine's
   tables use 256: page size has to be per table, the open question in
   [storage.md](storage.md#other-open-questions).

**Sharp edges found:**

- **Scale needs a hierarchy over pages.** With 1250 pages, queries scan
  every page's box and the broadphase tests page pairs all against all;
  small pages then lose to large ones on queries (81 µs against 46). Pages
  are in Z-order, so boxes over runs of consecutive pages are a tree for
  nearly nothing; the spike doesn't have one.
- **Motion moves rows.** Falling, 18% of bodies change page each frame
  (settled, 5%); drifting at 10 000, 18%. Each is a row's components
  copied, which the upkeep times include (a row carries ~100 bytes of
  payload); in erased columns it's a move per column.
- **Queries test more rows than a grid's** (40 to 70 against about 20):
  a page's box is looser than a cell. Fine at these sizes; the hierarchy
  and smaller pages both tighten it.

**What the spike doesn't cover:** erased columns and real pages (it keeps
one vector of rows), parallel readers and writers, and a declaration
syntax. The scope question, a feature for physics's `Position` and
`Collider` or for any component pair, is a design choice the numbers
don't settle.

## In the engine

2026-09-24, `engine/ecs/spatial.rs`. A table holding a spatial key keeps,
beside its rows, each page's kind (ordered by a range of Z-order keys, big,
or staging), range, box and rows' boxes, and boxes over runs of 16 pages.
Pages hold 16 rows. `Structural` keeps the order: pushing a row into a
spatial table, writing a key or extent (a query writing one logs a
`Reorder`, so it has an apply node, and the footprint covers the table),
or handing one out mutably marks the table, and it's re-sorted when the
`Structural` drops. A re-sort bounds every row through the glue, moves
the rows whose keys left their page's range (splitting full pages, big
rows to big pages), then merges neighboring pages that fit in one.
`Query::in_region` walks runs, pages, then rows; `Query::near_pairs` pairs
runs, then pages, then rows, across the query's spatial tables.

Tested against brute force (`//engine/ecs:spatial_test`: regions and
pairs after random moves, spawns, table changes and a move of every row
at once; visibility; the apply node's ordering; parallel frames equal to
sequential ones); eight mutations of the order are caught, four of them
only after the tests grew to see them (big bodies the test data didn't
have, order and capacity the checker didn't check, and full pages of
misplaced rows only a mass move makes).

Physics runs on it: `Position` is the key, `SpatialIndex` and
`publish_index` are gone, `Spatial` is a region query plus exact shapes,
and the walkers' first-frame workaround is gone, since a level spawned in
`load` is in order before the first frame. The games' routes and replays
pass unchanged.

**What held up:** everything but speed. No index, no stale shapes, no
empty first frame; spatial reads ordered after whatever moved things by
the ordinary footprint rules; statics' pages never re-sorted; a respawn
visible to the next system.

**What didn't: the broadphase.** At 1000 bodies, `near_pairs` takes 110 to
180 µs a frame, against 39 to 48 in the spike and about as much as the
grid it replaced. With the re-sort after `solve` (about 30 µs) and the
index gone (about 50), a settled pile's frame went from 0.33 to 0.42 ms.
Splitting halved pages until merging was added (1000 rows had sat in 319
pages, 3 rows each); with merging, 86 pages of about 12 rows, and no
faster. On a dense layout (`//engine/ecs:spatial_bench`: 1000 touching
bodies, 3800 pairs) it's 258 µs, and 3.5 ms at 10 000: each page is tested
against every page its box meets, and each meeting pair of pages is about
144 row tests. The spike's scenes were sparser than a settled pile, which
flattered page pairs.

**Open question:** the broadphase algorithm over the order. Page pairs are
the simplest thing; a sweep along one axis within each meeting pair of
pages, or per-row neighbor walks through the Z-order, cut the row tests
that dominate. The storage itself (upkeep about 30 µs, region queries
faster than the index's) isn't the problem.

## How general it is

2026-09-24. `engine_ecs` knows nothing of physics: any component can be a
spatial key with any extent, and physics is one user. What it assumes:

- **A writer still visits its tables.** A system writing a key logs a
  re-sort of every spatial table its query matches, and the re-sort scans
  every row's tick; only rows written through are re-bounded and moved
  (`Mut<T>`, below).
- **One order per table**, like a database's clustered index: rows have
  one physical order, so an entity with two positions clusters by one.
  Inherent to storing space as structure. An ordered key
  ([relationships.md](relationships.md#ordered-tables)) in a spatial table
  gives way: ranges of it there are scanned.
- **One extent per key**: no compound shapes, which ties into several
  colliders per body (the physics retrospective's first flaw).
- **2D, boxes, two size classes** (ordered and big, at a threshold the
  key sets), and global page and run sizes (16 and 16).
- **`near_pairs` returns every pair**, statics' included; physics
  filters them after.

## The broadphase, reworked

2026-09-24 (get-emj.16). `near_pairs` sweeps along x within each meeting
pair of pages (each page's rows sorted by left edge once per call, a
merged sweep between two pages) instead of testing every row against
every row, reuses its buffers, and radix-sorts the pairs by entity index
(live entities never share one) instead of comparing entity pairs. On
the dense layout (`//engine/ecs:spatial_bench`): 254 to 98 µs at 1000
bodies, 3.5 to 1.7 ms at 10 000; the sort had been over a third of it.
The physics pile's settled frame: 0.42 to 0.35 ms, against 0.33 before
spatial storage, whose index rebuild is gone. Still open: pairs between
two pages that haven't changed are the same as last time, and could be
kept.

**Note, 2026-09-24 (broadphase against sweep and prune).** Each page's row
boxes are now kept by coordinate (`Lanes`, the order's only copy of them),
so a box is tested against a whole page at once as a bit mask, without a
branch per row. `near_pairs` sweeps pages along x, and in each meeting
pair tests only rows that reach the other page; pairs are sorted by a
counting pass over the lesser index. The re-sort walks only pages with a
row that may have left (an O(1) range test at re-bounding), and computes
Morton cells without `floor` (a libm call on this target; see lore).
`./bazel run -c opt //engine/std/physics:tax`, µs a step, ECS / arrays:
broadphase at 10 000 settled 1000 to 235 / 280 (arrays' sweep sorting
afresh: 425), falling 726 to 212 / 202; at 1000, 61 to 19 / 25 and 58 to
19 / 20. The re-sort outside the systems: 352 to 160 settled, 429 to 276
falling. The 10 000 settled frame: 2911 to 1930, against 1270. On the
dense layout (`spatial_bench`): 1723 to 616 µs at 10 000, 97 to 32 at 1000;
the 1% writer 164 to 37 µs.
What was tried and didn't pay: pages of 8 rows (broadphase twice as slow)
or 32 (broadphase the same, re-sort cheaper; regions not measured);
scalar tests for small clipped sets; a global sweep along x, whose sweep
alone is 126 µs on the pile (400 wide, 36 tall) but 2088 µs on the square dense layout against
487 for pages, since its work grows with the scene's height. Temporal
coherence by ticks can't help this scene: every body moves bitwise every
step, settled or not, while the pair set doesn't change at all from step to
step, so only slack (fat) boxes could reuse pairs.

## Change detection

2026-09-24 (get-emj.17). Every value in an erased column has a tick, the
world's counter when it was last written, kept with the value through
every row operation. A `&mut T` term hands out `Mut<T>` (as Bevy does),
which reads as `&T` and stamps the row's tick only when written through;
writes between frames stamp too. A spatial table remembers the tick it
was last sorted at, and a re-sort re-bounds only rows new to it or whose
key or extent was written since. A system visiting 10 000 rows mutably
and moving 1% costs 160 µs a frame, against 268 without, re-bounding 100
rows instead of all. The cost: a binding written through needs `mut`,
and every write stamps a tick (the physics pile, where everything moves,
went from 0.35 to 0.36 ms). Six mutations of it are caught. `Changed<T>`
filters are the natural next use, and aren't built.
