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
  "physics2d::Position", order = spatial { .. } }`, with `SpatialKey`
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
a page.[^spike]

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
- **Up to two extents per key** ([Bounds from several
  components](#bounds-from-several-components)): no compound shapes,
  which ties into several colliders per body (the physics retrospective's
  first flaw).
- **2D or 3D ([In 3D](#in-3d)), boxes, two size classes** (ordered and big, at a threshold the
  key sets), and global page and run sizes (16 and 16).
- **A query's `near_pairs` returns every pair**, statics' included;
  `near_pairs(active, passive, grow)` leaves out pairs of two passive
  rows ([Two sides](#two-sides)), which is how physics skips statics
  against statics and sleeping bodies against both.
- **Kept pairs are a relation's** ([Keeping pairs](#keeping-pairs)): any
  number of relations on one key, each its own set; a side can't filter by
  a sparse component, which registration refuses.

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
runs, not its rows. A passive page is tested against an active one from
whichever side has fewer rows reaching the other, each against all of the
other's at once: a wall reaching a page is one test, not one per row of
the page, which took walls on the passive side from 2% slower than on the
active side (in a pile 440 rows wide) to about 1%. Between two active
pages it doesn't choose: there it cost the dense layout 7% (351 µs to
375). Pairs sort by comparison when there are few for the indices they
span (under a quarter): a bucket per index costs passes over every index,
and 800 pairs among 10 000 indices went from 25 µs to 18.

`spatial_bench`, 10 000 touching rows and three walls, µs, two runs, with
pages as blocks of the order: one query, 387; walls passive, 387 to 390;
everything passive but 200 rows on top, 13. At 1000: 36, 36, 1.2. In
physics's step the two-sided broadphase is about 10 µs over the one-sided
one at 10 000 (physics.md, "What the ECS costs"). A table matched twice
(by both sides, or two queries of one) is allowed, at the price of making
pairs unique after, a pass over all of them.

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

**Arriving and leaving** (2026-09-25, for sleeping's wakes: physics.md,
"Sleeping"). A spawned value is written, and so is a value inserted (as
Bevy's `Changed` includes `Added`), all at one tick per `Structural`: a
walk for what's written sees rows new to a query, which a static spawned
into sleeping bodies needs. Values a row carries to another table keep
their ticks, so a row that arrives by a removal isn't seen written. A row
that's gone leaves nothing to look at, so each table keeps the tick a row
last left it (despawned, or moved to another table) and last arrived in
it: `Query::left_since(since)` and `arrived_since(since)` are a look per
matched table, whether any did, not which, the which being a walk (of the
table for what left, of what's written for what arrived). They replace
counting rows, which one gone and another come in the same step fools.
`Query::written(e)` is one row's latest tick over the query's terms.

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

## In 3D

**Status: built** (in `engine_ecs`, on main). The storage is generic over
the dimensions. The 3D physics mod (`//engine/std/physics3d`, spheres and
boxes that turn, still experimental) runs on it, and
`//engine/std/physics3d/compare` compares it with Rapier 3D, Jolt and Box3D
([physics.md](physics.md#rotation-in-3d)). It began as a spike (2026-09-25,
branch `spike/physics3d`), for what 3D physics asks of the storage core
before more is built on 2D alone, under a translation-only step of
spheres and axis-aligned boxes
([the physics log](../retrospectives/2026-10-04-physics-log.md#3d-translation-only-spike)); this section's
measurements are that spike's, with turned boxes added the next day.

**What changed in `engine_ecs`.** Everything that knows the axes takes a
const `D` (2 or 3), defaulting to 2 so 2D code names `Bounds` and
`SpatialKey` alone and didn't change: `Bounds<D>`, `Lanes<D>` (a lane
per axis for the rows' mins and maxes), `SpatialPages<D>`, the re-sort and
re-bounding, `in_region` and the broadphase's sweep and masks. A key says
`impl SpatialKey<3>`; its glue is `BoundsGlue::Space`, and the world makes
the key's tables' orders a `SpatialOrder::Space`. The world picks the
dimensions once per operation (a re-sort, a region query, a
`near_pairs` call), never per row, so each is its own monomorphized loop.
A const generic can't pick an enum variant by itself, and arrays sized by a
trait's associated constant need an unstable feature, so the pick is a pair
of impls, one per dimension (`Axes<D>: Dims<D>`, `GlueOf<T, D>: MakeGlue`).
The 3D key is three 21-bit cells interleaved (a million cells either side
of zero, as in 2D); splits at blocks of the order, merging, change
detection, the two-sided broadphase and upkeep are unchanged: none of
them looks at an axis. A key's dimensions are fixed like its storage
(changing them takes a restart); a broadphase over tables of two
dimensions is refused. `BoundsFn` and `SpatialDesc` changed, so
`API_VERSION` did (25). The unsafe glue (`__bounds`) is the same code
made generic over `D`, no new unsafe.

**What 2D pays for being generic: nothing measured.** `:tax`, the same
binary before and after, alternated three times each, medians: every
scene's frame within 2% and no stage slower (10 000 in columns settled, 1342
against 1321 µs; the real pile settled 2891 against 2885), and still bit for
bit the arrays'. The alternative, one 3D storage with 2D at z = 0, costs
2D 13 to 16% in `near_pairs` on the dense layout (`spatial3d_bench`: 360
against 409 µs at 10 000, 27 against 31 at 1000; regions the same) and a
fifth more memory in lanes (704 bytes a page against 576).

`./bazel run --config=bench //engine/ecs:spatial3d_bench`, µs, one thread, touching
bodies on a lattice (0.9 apart, half extent 0.45), two runs agreeing within
3%:

| | 2D, 1000 | 3D, 1000 | 2D, 10 000 | 3D, 10 000 |
|---|---|---|---|---|
| pairs a row | 3.8 | 10.5 | 3.9 | 11.8 |
| `near_pairs`, 16 rows a page | 27 | 74 | 361 | 1590 |
| `near_pairs`, 32 rows a page | 34 | 83 | 319 | 1330 |
| re-sort, creeping (16 / 32) | 7 / 6 | 9 / 8 | 67 / 63 | 84 / 76 |
| re-sort, falling (16 / 32) | 15 / 12 | 16 / 14 | 131 / 120 | 193 / 156 |
| page extent, 16 rows | 3.9 x 2.4 | 2.3 x 2.1 x 1.8 | | |

- **Pairs cost more, and there are more of them.** Per pair found, 3D's
  `near_pairs` is about 1.5 times 2D's (13.5 against 9 ns at 10 000): a
  mask test is six compares, not four, and a page of 16 in 3D is 2.5 bodies
  a side, so most of its rows are on its surface and it meets about 26
  neighbours, where a 2D page meets about 8.
- **3D wants bigger pages.** At 32 rows (the most a `u32` mask holds) 3D's
  `near_pairs` is 16% faster and its falling re-sort 19%; 2D's broadphase is
  11% faster at 10 000 and 25% slower at 1000. Page size per table (or per
  dimension), the open question in [storage.md](storage.md#other-open-questions),
  now has a second reason; 64 rows would need `u64` masks. (Page size is
  per table, but every spatial table's is `SPATIAL_PAGE_ROWS`, 16.)
- **Upkeep grows less than the broadphase**: creeping, the re-sort is a
  quarter more in 3D (a key and cell per row, a lane more to re-box);
  falling, half again, since rows cross more page faces.
- **What the spike doesn't cover:** a hierarchy over pages (runs are 16
  pages whatever the dimension), mixed sizes in 3D, and big bodies in 3D
  beyond a floor and walls.
- **Turned boxes** (2026-09-26): a turned box's bounds depend on its
  rotation as well as its position and shape. The 3D step's `Position`
  has `(Collider, Rotation)` as its extents, as 2D's does (below), so a
  turn re-bounds the row through storage; a sphere around each collider
  instead cost 4-6 times the pairs, and ten times the broadphase once
  walls got it too. What 3D still wants from the storage (fat bounds or
  kept pairs, wider tuples):
  [the physics log](../retrospectives/2026-10-04-physics-log.md#what-3d-asks-of-the-storage-design).[^reach]

## Bounds from several components

**Status: built** (2026-09-26, with rotation in physics:
[physics.md](physics.md#rotation)). A turned box's bounds depend on where
it is, which way it faces, and its shape, where a `SpatialKey` had one key
and one extent. Now a key's `Extent` is an `Extents`: one component, as
before, or a pair, each of which a row may lack (`bounds` gets
`(Option<&A>, Option<&B>)`). Physics's `Position` has `(Collider,
Rotation)`, and a body that doesn't turn has no `Rotation`, so it is
bounded as it was. Writing either extent re-bounds the row, as writing the
key does; the glue takes a column per extent (`BoundsFn`, `API_VERSION`
26). The unsafe reads are the same kind as before, one more per row with a
second extent, beside the existing ones in `spatial.rs`. Nothing in the
order, the re-sort's moves, the lanes or the broadphase changed: they see
boxes, however they were made. It is as generic over dimensions as the rest
(3D's is `(Collider, Rotation)`, a quaternion).

The ways rotation could reach storage, measured on the same rows
(`./bazel run --config=bench //engine/ecs:spatial_turn_bench`: 10 000 boxes 0.8
across a unit apart, so axis-aligned they are out of reach and turned they
meet; µs a frame, the writer / the re-sort after it / `near_pairs`, and the
pairs found; median of 3):

| way | nothing turns, creeping | all turn, creeping | all turn, only turning | all turn, falling | 1 in 10 turns, creeping |
|---|---|---|---|---|---|
| no rotation at all | 15 / 62 / 111 (4962) | – | – | – | – |
| **extents a pair (collider, rotation)** | **14 / 64 / 111 (4962)** | **38 / 76 / 301 (31783)** | **36 / 76 / 339** | **49 / 212 / 627** | **19 / 69 / 186 (8613)** |
| a pose key (x, y, cos, sin) | 22 / 71 / 112 | 32 / 71 / 297 | 32 / 71 / 335 | 42 / 199 / 626 | 24 / 71 / 151 (8613) |
| a pose key with an angle | 20 / 81 / 109 | 21 / 109 / 303 | 22 / 114 / 337 | 31 / 276 / 625 | 22 / 87 / 152 |
| conservative (the circle around the box) | 15 / 67 / 263 (34414) | 37 / 70 / 268 (34414) | 36 / 18 / 274 | 50 / 207 / 651 (63437) | 20 / 73 / 400 (34414) |

- **Where nothing turns, the pair costs nothing** (14 / 64 / 111 against 15
  / 62 / 111). A pose key makes every row carry a rotation, 50% more to
  write and 15% more to re-sort, whether it turns or not; conservative
  bounds make every box a circle, 7 times the pairs and 2.4 times the
  broadphase.
- **Where everything turns, a pose key is 5–15% cheaper** to write and
  re-sort (one column, not two), with the same bounds, so the same
  broadphase.
- **An angle costs the re-sort a sine and a cosine a row**: 45% slower
  than storing the rotation as a cosine and sine, as Box2D's `b2Rot` and
  Rapier's unit complex do.
- **Conservative bounds win only where bodies turn in place**: turning
  never re-bounds (18 µs against 76). Bodies that turn also move, and it
  pays in pairs everywhere else.
- **A pair of extents splits the rows that lack one into tables of their
  own**, and where those interleave in space (one body in ten turning) the
  broadphase sweeps two sets of pages over the same ground: 186 µs against
  151 for one table (lore:
  [an extent some rows lack splits their tables](../lore/an-extent-some-rows-lack-splits-their-tables-and-the-broadphase-pays.md)).
  A game's bodies mostly all turn or all don't; a mix (a platformer's
  crates among its tiles) is small.

So the pair: it costs a world where nothing turns nothing, which is every
game today, and pays a pose key's price only where bodies turn. The same
reasoning gave 3D its `(Collider, Rotation)` pair: every 3D body turns or
is static, so there the pair splits no tables.

## Keeping pairs

**Status: built** (2026-09-27, get-emj.36, get-pmk): live proximity
relations, `engine_ecs::Live<R>` for a declared `R: Proximity` (`live.rs`),
which physics and physics3d find their pairs through. This
section is its measurements and choices; the design (the problem, its
use, storage, testing, lineage and how it could grow) is
[live.md](live.md). The
[retrospective](../retrospectives/2026-09-26-physics-against-other-engines.md)
found `near_pairs` finding every pair afresh: that won when everything
fell and lost when little moved (a resting 2D pile 400 µs a step against
Box2D's nothing; 3D 3 to 10 times Rapier's). Now a pile at rest costs a
look per page, a settled pile a test per kept pair, and falling what it
did.

**What it does.** `Live<R>` is a system parameter: the pairs relation `R`
keeps (its key, its sides, its grow and margin declared by `R`), and
`live.pairs()` answers what `near_pairs(active, passive, R::GROW)` would,
bit for bit.

- **Fat boxes.** Each row keeps a fat box, its box (grown by `grow`) grown
  by `margin` when last found, as long as its box stays inside it and it
  stays inside the box grown by twice the margin. The second bound, which
  Box2D's and Rapier's fat boxes don't have, is what lets new pairs be
  looked for among the order's boxes (below): a fat box is never more than
  `2 * margin` past its row's box, however the box shrank.
- **Candidates.** Pairs whose fat boxes meet and one of which is active,
  sorted by entity index, the order `near_pairs` answers in. Each call
  tests every candidate's boxes with the operations `near_pairs` uses, so
  the answer is the same bits.
- **What changed.** A call walks only the pages whose lanes changed since
  the last (`SpatialPages::changed`, a tick by page the re-sort stamps, and
  a row leaving stamps, since a despawn marks no table for a re-sort). A
  row whose box left its fat box, or that arrived (spawned, or moved into
  a covered table, or from one side to the other), is looked for again: its
  new candidates are among pairs whose boxes, grown by `grow + 2 * margin`
  (and a few ulps of the coordinates for rounding), meet, found by
  `near_pairs`' own sweep and passive walk with only those rows on the
  swept side. A row that left every covered table is seen gone from the
  page it was on.
- **Few changed.** When nothing moved and fewer rows changed than a 64th of
  the candidates, only their candidates are retested, found by searching
  the candidates from either end (the lesser end's are a run of them; the
  greater's a run of an index made when first needed), and the pairs are
  made again only if one came to meet or stopped.
- **Too much moving.** More than a tenth of the active rows moving
  (`MOST`), the call is `near_pairs` afresh, and the next few calls are too
  without walking (one, then two, up to sixteen), so falling doesn't pay a
  walk to find out every step.
- **Nothing changed**: the last answer, borrowed.

**Where it lives.** The world's, one set per relation, like the order it's
found in: plain data, so a reload keeps it; taken by a system as a
parameter, for writing (`ParamDecl::Live`) and reading the key in the
sides' tables, so two systems that take one relation are ordered like two
writers of a component, a system reading the key isn't held up, relations
on one key are apart, and a system can't take one relation twice
(`live_relations_are_a_footprint_the_scheduler_sees`). Its entries are by
entity index, not by page: the re-sort moves rows between pages every step
(5% of a settled pile, 18% falling), and pairs by entity don't notice.

**Tested** (`//engine/ecs:live_test`) against `near_pairs` and brute force,
in 2D and 3D, every frame of a script: at rest, creeping, a few rows
nudged back and forth (the few-changed path, which is seen to make and end
pairs), everything wobbled once, 2% flying, extents written, everything
falling and shaken (afresh, then waiting), rows spawned, despawned (their
indices reused, and not), moved between tables, between the sides and out
of the spatial tables, and despawns with nothing else re-sorting. After
every call `Live::check` compares everything kept with the tables by
brute force: each row's side, box and fat box, nothing kept of rows gone,
the candidates exactly the pairs whose fat boxes meet, and whether each
meets. How each frame was answered is asserted (at rest the last answer,
nudged the few-changed path, falling afresh, then waiting). 24 mutations
of the kept state, the new footprint rules in `graph.rs` and `query.rs`, and
the stamps in `spatial.rs` are caught, most by the check. One isn't: the
slack for rounding in the search, which takes coordinates of about 1e5 to
matter. The stamp on a pushed row was taken out rather than tested: the
re-sort that always follows a push stamps its page.

**What the others do** (read in their fetched source, 2026-09-27):

- **Box2D v3.1.1** grows each shape's box by `B2_SPECULATIVE_DISTANCE`
  (0.02 m), keeps a fat box `B2_AABB_MARGIN` (0.05 m) past it for moving
  shapes, and re-inserts a shape in its dynamic tree only when its box
  leaves the fat box (`b2UpdateShapeAABBs`, `b2FinalizeBodies`); those go
  in a move buffer, and only they query the trees for new pairs
  (`b2UpdateBroadPhasePairs`, a pair set against duplicates). A pair is a
  contact until its fat boxes stop meeting, tested for every contact in
  `b2Collide`, which its profile counts as the narrowphase: its broadphase
  is 0 µs at rest because that test is elsewhere.
- **Box3D v0.1.0** is the same, with a shape's margin an eighth of its
  size up to 0.05 (`B3_AABB_MARGIN_FRACTION`, `B3_MAX_AABB_MARGIN`).
- **Rapier 0.36** keeps a BVH of loosened boxes (a skin of 0.04 or an
  eighth of the shape, capped), its pairs in a map with an adjacency list
  per collider, and re-examines only pairs beside colliders whose leaves
  changed (`broad_phase_bvh/update.rs`: "a pair can only stop overlapping
  if one side changed"). It hands the narrowphase pair events, added and
  removed.
- **Jolt 5.6** finds its active bodies' pairs afresh every step
  (`BroadPhaseQuadTree::FindCollidingPairs`), as `near_pairs` did, and
  keeps contact manifolds between steps instead (its body pair cache).

What we take (docs/CREDITS.md): fat boxes and pairs kept until their fat
boxes part, looked for again only for what left its fat box (Box2D, Box3D);
looking only at the pairs of what changed (Rapier's adjacency, here a
search from either end). What we don't: a tree (the spatial pages are the
index, and `near_pairs`' sweep finds the new pairs), and handing out fat
pairs for the narrowphase to test (the answer has to be `near_pairs`',
exactly, and our narrowphase costs more per pair than a box test).

**Decisions, each measured** (µs a step, one thread, `-c opt`, 10 000
bodies unless said; 2D the real pile of `//engine/std/physics2d/compare`,
3D `//engine/std/physics3d/compare`'s boxes turning; `live_bench` the lattice without
physics):

1. **The margin.** A settled pile creeps less than any margin tried, so
   the smallest was cheapest: fewer candidates to test, and none moving.
   Box2D's 0.05 costs 6% more in 2D and 23% more in 3D, where candidates
   grow with a margin's volume. The case for a larger one is motion
   between the two, which the benches' settling windows barely have
   (rain moves too much for any: afresh at every margin, as before).

   | margin | 2D settled | 2D turning, settled | 2D candidates | 3D settling | 3D settled | 3D candidates | 3D falling |
   |---|---|---|---|---|---|---|---|
   | **0.02** | **131** | **200** | **19 314** | **314** | **314** | **31 022** | **876** |
   | 0.05 | 139 | 213 | 21 859 | 362 | 387 | 41 245 | 879 |
   | 0.1 | 143 | 260 | 23 893 | 435 | 453 | 65 162 | 965 |
   | 0.2 | 162 | 303 | 29 527 | 473 | 493 | 99 080 | 1088 |

   (physics's broadphase stage: the kept pairs, then turning them into
   the gathered colliders' slots; about 0 rows moving a step settled at
   every margin, 1.3 in 3D's settling window at 0.02.)
2. **Where the fat boxes live.** In the kept record by entity (built),
   beside each row's grown box and side: the walk reads and writes one
   record per changed row. Kept in the spatial pages as lanes, the re-sort
   could test containment while it has each box in registers, but the
   exact test still needs every candidate's box by entity, the pages
   would carry 256 bytes more (384 in 3D) and every storage user would
   pay for re-fattening, falling included; not built. Computed rather
   than kept (a box snapped outward to a grid of the margin) needs no
   memory, but a creeping row crosses a grid line with each step's
   motion over the margin, where a kept fat box lets it drift a whole
   margin first; not built either.
3. **Where the pairs live.** The world's, per relation (per key until
   the relations were declared, the same day), taken as a parameter
   (built). In the mod's own state (how it was first built) it ran the same, but
   the scheduler couldn't see it, two systems couldn't share it, and a
   reload lost it. As entities (the contact table, or a table of pairs),
   each begun or ended candidate is a structural change (about 100 ns
   each: 331 new contacts and their events cost 35 µs,
   [Pages as blocks](#pages-as-blocks-of-the-order)), thousands a step
   while a pile lands, and each candidate's boxes are two lookups
   through entity locations into lanes, four to six cache lines a box
   where a record is one. As caches in the spatial pages, readers of the
   tables would have to write the order, which serializes every region
   query, and a pair across two tables (a body and a static, a body and a
   sleeper) has no one page to live in.
4. **What says what changed.** A tick by page (built), which any number
   of readers can compare against their own last call. A move buffer as
   Box2D's needs the fat boxes in storage (2) and one reader. Rebuilding
   the pairs of pages whose rows changed is `near_pairs` afresh on a
   settled pile, whose rows on changed pages are 45% of them (2D) to all
   (3D) every step: 406 and 1195 µs.
5. **The API.** A parameter whose `pairs()` returns the pairs borrowed
   (built), for a relation declared as a type (live.md, "What using it
   looks like"). Keeping `near_pairs` and caching inside it would copy
   the answer out every call, 9 µs in 2D and 28 in 3D at 10 000 on the
   lattice (`live_bench`), where the pairs at rest cost 0.8; and a cache
   inside a read-only call is one the scheduler can't see. Pairs begun
   and ended, as Rapier's events, suit a narrowphase that keeps its own
   pairs; ours tests every pair every step, so it would keep the whole
   list anyway.
6. **When to go afresh.** Looking for a moving row's pairs costs about
   0.4 µs in 2D and 1 in 3D (`live_bench`); past about a tenth of the rows
   moving, `near_pairs` afresh is cheaper. Kept against afresh, 10 000:

   | rows moving a call | 1% | 5% | 20% |
   |---|---|---|---|
   | 2D | 157 / 432 | 368 / 499 | 949 / 478 |
   | 3D | 596 / 1927 | 1057 / 1886 | 1949 / 1501 |

   Walking every step to find out cost falling 70% more (`:tax`, 121 µs to
   203 at 10 000), so a call that goes afresh waits before walking again,
   twice as long each time up to 16 calls (64 measured the same).
7. **Few changed.** A pile at rest in which one page's rows still
   changed retested all 22 000 candidates: 60 µs of the step's
   broadphase. Retesting only the changed rows' candidates took it to 2.

**Before and after** (2026-09-27; before is edfea31, after this work; the
same binaries alternated, no build running beside them).
`//engine/std/physics2d/compare`, the real pile, µs a step, the step and its
broadphase (physics's stage: the pairs, then their colliders' slots; in
brackets the pairs alone), median of 3; Box2D's and Rapier's from the same
run:

| | before | after | Box2D | Rapier |
|---|---|---|---|---|
| 1000, falling | 170: 20 (19) | 171: 21 (20) | 204: 77 | 221: 60 |
| 1000, settled | 322: 24 (22) | 311: 11 (9) | 295: 0 | 319: 4 |
| 1000, at rest | 317: 24 (22) | 297: 3 (0) | 297: 0 | 319: 4 |
| 10 000, falling | 1655: 197 (189) | 1663: 206 (197) | 2550: 1121 | 2375: 658 |
| 10 000, settled | 3621: 406 (383) | 3331: 132 (107) | 3354: 0 | 3531: 64 |
| 10 000, at rest | 3544: 395 (368) | 3147: 26 (2) | 3367: 0 | 3518: 61 |

At 10 000 settled the step is now level with Box2D's, and at rest 7% under
it; what's left of the stage at rest is turning 17 000 pairs into slots, the
physics mod's own. `:tax` (ECS / arrays, bit for bit as before):

| | 10 000 settled | 10 000 at rest | 10 000 settled, turning | 10 000 at rest, turning | 10 000 falling |
|---|---|---|---|---|---|
| broadphase, before | 153 / 421 | 152 / 420 | 512 / 1710 | 514 / 1711 | 120-126 / 352-393 |
| broadphase, after | 15 / 428 | 15 / 433 | 196 / 1726 | 163 / 1694 | 128-134 / 349-391 |
| frame, before | 1558 / 1666 | 1559 / 1664 | 10530 / 12325 | 10562 / 12557 | 871-883 / 837-862 |
| frame, after | 1437 / 1692 | 1450 / 1706 | 10408 / 13091 | 10164 / 11277 | 870-889 / 831-863 |

In 3D (`//engine/std/physics3d/compare`, boxes turning, broadphase µs by phase: falling
1-61, settling 300-400, settled; 10 000 one run, 1000 the median of 3):

| | ours before | ours after | Rapier | Box3D |
|---|---|---|---|---|
| 1000 falling | 54-58 | 59 | 58 | 149 |
| 1000 settling | 43-50 | 16 | 0 | 0 |
| 1000 settled | 44-51 | 21 | 0 | 0 |
| 10 000 falling | 876-904 | 918 | 915 | 2591 |
| 10 000 settling | 1163-1204 | 321 | 154 | 102 |
| 10 000 settled | 1163-1195 | 298 | 0 | 0 |

(before: the two runs of edfea31.) The 3D step at 10 000 settled is 30.6
ms against 31.5-32.3, most of it the solver; Rapier's 13.4 and Box3D's
11.2.

**What's left.**

- **Settled isn't free.** Every candidate is tested each step a pile
  creeps: about 20 000 in 2D and 31 000 in 3D at 10 000, each two records
  read at random by entity, out of cache after the rest of a step (lore:
  [a broadphase bench alone flatters lookups by entity](../lore/a-broadphase-bench-alone-flatters-lookups-by-entity-three-times.md)).
  Box2D and Rapier show 0 there because their narrowphase does the same
  test on every fat pair (Box2D's `b2Collide`), and ours needs exact pairs
  for its cheaper narrowphase. The walk and the tests split by range would
  go across threads as `near_pairs` does; not done.
- **Falling pays a little.** In `:tax` at 10 000 falling (three
  alternated runs of each, both piles) the broadphase is 128-134 µs against
  120-126 before, and the frame the same (870-889 against 871-883). It
  isn't the walks that find out: waiting up to 64 calls between them
  instead of 16 measured the same (125-133). Not found; the fixed cost of
  a call afresh (three passes over the sides' tables, the sides sorted,
  the answer moved in) is the suspect.
- **One set a key** *(left when measured; solved the same day)*. A second
  broadphase over the same key's tables with other sides or grow would
  have started the kept pairs over each call. Relations are declared now,
  any number on one key, each its own set ([How general it
  is](#how-general-it-is);
  [live.md](live.md#1-more-than-one-kept-set-per-key-done)).

[^spike]: 2026-09-25. `spike/spatial` was removed once `engine/ecs/spatial.rs`,
    its tests and `//engine/ecs:spatial_bench` had superseded it; it is in
    git history up to c7be223.

[^reach]: 2026-09-26: until two extents landed, the 3D step kept the box
    around each body as turned in a derived `Reach` component, the key's
    one extent, rewritten by the solver when a box's box changed: a copy
    of what the collider and rotation already say, and a write the solver
    had to remember. Removed for the pair, at the same pairs and within a
    few percent of the step (physics.md, "Rotation in 3D", choice 5).
