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
`Structural` drops. A re-sort bounds the rows written since the last
through the glue, a page at a time, moves the rows whose keys left their
page's range (splitting full pages at a boundary of a block of the
order, big rows to big pages), then merges neighboring pages that fit in
three quarters of one ([Upkeep, reworked](#upkeep-reworked),
[Pages as blocks](#pages-as-blocks-of-the-order)).
`Query::in_region` walks runs, pages, then rows; `Query::near_pairs` sweeps
pages along x, then tests rows a page at a time, across the query's
spatial tables ([Against sweep and prune](#against-sweep-and-prune)).

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
- **A query's `near_pairs` returns every pair**, statics' included;
  `near_pairs(active, passive, grow)` leaves out pairs of two passive
  rows ([Two sides](#two-sides)), which is how physics skips statics
  against statics and sleeping bodies against both.

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

## Two sides

2026-09-24 (get-emj.27). `engine_ecs::near_pairs(active, passive, grow)`
takes two sides, each a query or a tuple of them, and returns every pair
at least one end of which is active: pairs of two passive rows (things at
rest, which meet as they did last step) aren't looked for. Physics's
active side is its awake colliders that can move, its passive side
statics and sleeping bodies, each in tables of their own
([physics.md](physics.md#sleeping)); a query's own `near_pairs` is one
active side and nothing passive.

Active pages are swept against each other as before. Passive pages are
found from the other end: each run of a passive table (16 consecutive
ordered pages, with a box) and each big page looks up, by binary search in
the sweep's order, the active pages that may reach it (from its left edge
less the widest active page), and only pages of a run some active page
meets are tested. So a passive table no active page is near costs its
runs, not its rows. A pair of pages tests the rows of whichever side has
fewer reaching the other against all of the other's at once: a wall
reaching a page is one test, not one per row of the page, which took
walls on the passive side from 2% slower than on the active side (in a
pile 440 rows wide) to about 1%, and none in a square one. Pairs sort by comparison when there are few for the indices
they span (under a quarter): a bucket per index costs passes over every
index, and 800 pairs among 10 000 indices went from 25 µs to 18.

`spatial_bench`, 10 000 touching rows and three walls, µs, three runs:
one query, 658; walls passive, 650; everything passive but 200 rows on
top, 18 to 19. At 1000: 40, 40, 1.4. A table matched twice (by both sides,
or two queries of one) is allowed, at the price of making pairs unique
after, a pass over all of them.

Tested against brute force (`spatial_test`,
`pairs_between_sides_agree_with_brute_force`: sides split by a table
component, by a sparse one, by both, a table on both sides, reused
indices' generations); eight mutations of it are caught.

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
went from 0.35 to 0.36 ms). Six mutations of it are caught.

**Walking what changed** (2026-09-24, get-emj.27):
`Query::for_each_written(since, f)` is every row any of whose terms was
written after tick `since`, a `Query::now()` taken before (the world's
tick while that query held its guards, so every write to what it matched
is on one side of it). Each page's column keeps a tick of its own, at
least the latest of its rows', so a page none of whose terms was written
since is skipped with a look: physics looks for a game's write to any of
five components of 10 000 sleeping bodies in 8 µs, where reading every
velocity (one of them) took 15. The
page's tick is stamped when writes are handed out (a page view, a slice,
a row's `Mut`), whether or not one is made, and raised by a row moving in:
conservative, so it only ever costs a look at a page for nothing. Stamping
it per write instead cost 1.2 ns a row in a loop writing every row
([lore](../lore/a-second-store-per-write-cost-a-nanosecond-a-row.md)).
Table components only, as page walks are. A `Changed<T>` filter, which
would need a tick to compare against in the query's declaration, isn't
built.

## Upkeep, reworked

2026-09-24. With change detection, a settled pile still re-sorted every
body every step (the solver writes them all, and a settled pile creeps:
every body 1e-4 to 1e-2 a step, none still bit for bit until about step
3000), at 352 µs a step for 10 000 bodies. Measured by stage, before: 198
µs re-bounding and re-keying rows, 146 µs looking for rows to move, which
found none, 17 µs re-boxing every page. What changed, each measured:

- **Rows stay unless their page's range excludes them.** Each row was
  looked up in the order (a binary search over the pages) to see where it
  belonged; now it's checked against its own page's range first, and only
  pages with a row out of place are walked at all. Rows not re-bounded
  are where the last sort left them, and no range changes between sorts,
  so only re-bounded rows can be out of place.
- **The bounds glue takes a page**, the rows to bound and their boxes, not
  a row: the call can't be inlined, and one per row, with the caller's
  state saved around it, was a third of re-bounding. (`BoundsFn` changed,
  so `API_VERSION` did.)
- **A row keeps its key while it keeps its cell.** Each row's cell (both
  axes, packed) is kept beside its key; interleaving bits for the key was
  most of re-keying a row, and a creeping row almost never changes cell.
  The cell is truncated after an offset rather than floored: `floor` is a
  function call on the default target
  ([lore](../lore/f32-floor-is-a-function-call-on-the-default-target.md)).
- **Pages re-box only when a row moved in, out or was re-bounded**, the
  last while its boxes are still in cache; runs only when a page's box may
  have changed.
- **Merging waits for pages to fit in three quarters of one**, so a merged
  page has room before it splits again: falling, 10 000 bodies split 26
  pages a step and merged 184 rows back; now 7 and 48.
- **Nothing to move or merge, nothing done** but the scan of ticks: a pile
  at rest (and physics writes a position only when it changes) costs the
  scan alone.

`:tax`'s "outside the systems", almost all the re-sort, µs per step:

| | 1000, falling | 1000, settled | 10 000, falling | 10 000, settled | 10 000, at rest |
|---|---|---|---|---|---|
| before | 40 | 31 | 421 | 345 | 344 |
| after | 24 | 12 | 191 | 107 | 26 |

`//engine/ecs:spatial_bench` (every row of 10 000 moved a hair: 303 to 70
µs; a hundred rows of 10 000 written: 168 to 26). Settled, re-bounding is
now most of it, about 7 ns a row warm and 10 in the physics step: the tick
checks, the glue, the cell and the range check, for rows that all changed.
The one way under that is fewer rows written: at rest, or
[sleeping](physics.md#sleeping).

Two bugs in the order were found on the way, both in how a range of one
key is kept: a page of exactly one key shared with the next page could be
merged into a page of another range, leaving its rows at that page's
upper bound, outside it; and a split around a key half a page shares
narrowed the page's range without moving the rows the new page's range
took. Walking only some pages made them fail
`everything_moving_at_once_keeps_the_order`. `crowded_cells_keep_the_order`
(600 rows over 25 cells) covers the split, and the re-sort before this
rework never finishes on it (the test times out): crowded cells weren't
safe before either. `SpatialPages::check` now checks every row's key
against its box, and that page boxes are exact unless marked stale.
Thirteen mutations of the rework are caught by `spatial_test` and the
unit tests of `cell`.

## Against sweep and prune

2026-09-24. `:tax` compares the broadphase with sweep and prune on
arrays, which keeps its x order between steps: `near_pairs` was 1000 µs a
step at 10 000 settled against the arrays' 276. Profiled: 150 µs building
and sorting each page's rows every call, 790 in page pairs (5400 of them,
146 ns each, mostly mispredicted branches on box tests, which in a dense
pile are about even odds), 77 sorting pairs. What changed, each measured:

- **Rows' boxes by coordinate** (`Lanes`, the order's only copy of each
  row's box, kept by the re-sort and by rows leaving): a box is tested
  against a whole page at once, as a mask of the rows it meets, with no
  branch per row. `in_region` uses the same masks.
- **Pages swept along x**, and in each meeting pair only the rows that
  reach the other page's box tested against it.
- **Pairs as keys of the two indices, sorted by counting the lesser**,
  then insertion within each bucket.

µs a step, ECS / arrays, three runs (with the upkeep rework above):

| | 1000 falling | 1000 settled | 1000 at rest | 10 000 falling | 10 000 settled | 10 000 at rest |
|---|---|---|---|---|---|---|
| broadphase, before | 58 / 20 | 61 / 25 | | 726 / 202 | 1000 / 276 | |
| broadphase, after | 19–20 / 19 | 19–20 / 24 | 19–20 / 24 | 218–221 / 200–205 | 227–244 / 274–281 | 213–217 / 276–280 |
| frame, after | 109–111 / 53 | 172–175 / 122 | 163–167 / 120 | 1118–1134 / 561–570 | 1849–1907 / 1264–1290 | 1734–1760 / 1259–1281 |

The arrays' sweep sorting afresh each step is 425 µs at 10 000. On the
dense layout (`spatial_bench`), `near_pairs` went from 1723 to about 600 µs
at 10 000 and from 97 to 32 at 1000. What's left is the masks themselves,
about 28 000 a step at 10 000, a few ns each on the baseline target (four
lanes wide).

What didn't pay: pages of 8 rows (broadphase twice as slow) or 32 (about
the same; region queries not measured); scalar tests for small clipped
sets (branches again); dropping the second page's clip. A global sweep
along x, as the arrays do, sweeps the pile (400 wide, 36 tall) in 126 µs,
but the square dense layout in 2088 against 487 for pages: its work grows
with the scene's extent across the sweep, so it isn't a primitive for
storage to keep. Temporal coherence by ticks can't help a settled pile:
every body moves every step, while the pair set stays the same from step to
step, so only slack (fat) boxes and a cache of pairs could reuse them.

## Pages as blocks of the order

2026-09-24 (get-emj.26). Falling was where the ECS was furthest behind the
arrays in `:tax` (1.6×): bodies really move, so rows change pages every
step. Profiled at 10 000 falling, the re-sort was about 150 µs a step:
re-bounding every row (10 µs a thousand), 660 to 870 rows moved (about 65
ns each: four columns, about 60 bytes and four ticks, the lanes, and two
entity locations), splits and merges, and re-boxing. What changed, each
measured:

- **A full page splits at a block boundary**, the key between its rows'
  quartiles with the most trailing zeros, not at the median: in Z-order
  that's the edge of the largest square (or half of one) there, so pages
  tend to be whole blocks. A range split at the median straddles blocks,
  and its box covers both pieces and the gap between. Blocks have less
  edge per row, so rows cross fewer page boundaries too: half the moves
  (457 and 354 a step, in the two halves of `:tax`'s 60, against 866 and
  662), and a broadphase twice as fast. Found by making cells bigger (2 to
  8 units): at 4, a cell held about a page of the pile, pages were cells,
  and the broadphase and moves came out the same as they do now. Splitting
  at blocks makes the cell size matter little (0.5 to 4 measured the
  same), so `CELL` stays about the size of the smallest things.
  `pages_are_blocks_of_the_order` sees the difference (mean page width
  plus height 6.2 against 9.5 at the median).
- **Rows to move are bits by page**, set as rows are re-bounded and
  carried through moves, so a page with one row to move doesn't have
  every row asked where it belongs.
- **Each row's key and cells are in its page's lanes**, beside its box,
  rather than in vectors by page; a page's box is the lanes halved round
  by round rather than a fold, whose chain of dependent compares doesn't
  vectorize (18 µs to 8 for 1000 pages); a moved value is copied as a
  fixed size (a copy of a size known only at runtime is a call to
  `memcpy`, two per column per move); the rows to re-bound are a mask.
- **`Position::bounds` is inlined** into the glue: it's in physics's
  interface crate, and the glue called it per row
  ([lore](../lore/a-trait-impl-the-glue-calls-is-not-inlined-across-crates.md)).

Twelve mutations of these are caught: nine by `spatial_test` (a moved
row's bit left behind, no row marked, either half of the range check, a
lane skipped in the box, a key left behind by `swap_remove`, extents' or
new rows' writes unseen, the median split), three fixed-size copies of the
wrong size by `values_of_every_size_move_whole`.

`:tax`, µs per step, ECS / arrays, medians of five runs, before and after:

| | 1000 falling | 1000 settled | 1000 at rest | 10 000 falling | 10 000 settled | 10 000 at rest |
|---|---|---|---|---|---|---|
| frame, before | 87 / 53 | 137 / 122 | 130 / 121 | 909 / 561 | 1452 / 1276 | 1371 / 1275 |
| frame, after | 74 / 54 | 133 / 125 | 126 / 123 | 730 / 572 | 1350 / 1298 | 1288 / 1289 |
| broadphase, before | 19 | 19 | 20 | 220 | 226 | 216 |
| broadphase, after | 12 | 14 | 14 | 116 | 150 | 150 |
| outside the systems, before | 24 | 13 | 6 | 196 | 100 | 25 |
| outside the systems, after | 18 | 11 | 6 | 127 | 78 | 24 |

The arrays' broadphase is 204, 281 and 282 at 10 000. `spatial_bench`
(which gained a falling scene): the falling re-sort 224 µs to 150, the
creeping one 78 to 67, and on the dense layout `near_pairs` 631 µs to 343
at 10 000 and 32 to 26 at 1000, 64 regions 52 µs to 36.

**What didn't pay:**

- **Pages of 32 rows.** Before blocks, 30 µs better falling (a third fewer
  moves, half the pages to walk) and 16 worse at rest (the broadphase);
  with blocks, worse or even everywhere. 24 rows was erratic.
- **Testing only the lanes a page uses** in `Lanes::meeting`, in chunks up
  to its rows: the broadphase went from 219 µs to 310, the loop no longer
  a fixed one the compiler unrolls.
- **Prefetching the next pages' columns** in the re-bounding loop: 7% of
  it before `bounds` was inlined, within the noise after.
- **Re-keying every re-bounded row**, to drop the branch on whether its
  cell changed: creeping 22 µs slower, falling the same.
- **A wider interval to split in** (eighths): the same.
- **Slack, not built:** a row allowed to stay while its key is in a
  neighbouring page's range. 57% of the rows that left their range, before
  blocks, were in a neighbour's; after blocks halved the moves, that's
  worth perhaps 10 µs, and a split narrows its neighbours' ranges, so it
  would need a pass to move rows the split left outside.

**What's left**, at 10 000 falling (730 / 572): re-bounding, about 72 µs,
since every body is written every step (7 ns a row); moving about 400 rows,
20; applying 331 new contacts and 331 `Contact` events a step, about 35
(each a boxed closure in the log: batching a writer's consecutive changes
is the next step there); and gathering, the solver's gathering and writing
back, 117 over the arrays, unchanged ([physics.md](physics.md#what-the-ecs-costs)).
The broadphase (89 under the arrays') and narrowphase (23 under) pay for
most of it.
