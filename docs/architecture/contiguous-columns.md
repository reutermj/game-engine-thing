# Contiguous columns

**Status: deferred** (2026-10-02, get-5av; the decision, get-4yr, is
deferred by the user until real data supports it). Its first reason, a
solve in place, is gone: the solve loses in contiguous columns too, and
flows give a system's dense copy a home in the ECS
([flows-spike.md](flows-spike.md)). What's left are gains on storage's own
micro-benchmarks that no real frame has shown mattering. And the scheduler
is to own all parallelism (get-znt.28), as tasks over blocks of world data:
separate pages let each task borrow its page in safe code, where one block
a column would need unsafe code or a split made up front. Revisit when a
frame's profile shows storage's operations as a real cost, after the safe
fix (pages made whole when created, get-fqu) has been measured.

Nothing here is built. This
doc asks why the world's storage can't be indexed like a plain array, and
whether it could be, so that a system like the solver works in the world's
columns instead of copying them. Two spikes measure it, both bench targets
rather than engine code:

- `//engine/std/physics2d/compare:contiguous_spike` (`contiguous_spike.rs`,
  with `contiguous_lanes.rs` included into the solver's `lanes`): the
  solve's whole system as the copy does it, and in place in columns kept
  each way, on the engine's own world, at one thread and at eight.
- `//engine/ecs:contiguous_spike` (`tests/contiguous_spike.rs`): a table's
  columns kept as today's pages, as lean pages, as one heap block a column,
  and in reserved address space. What storage's own operations cost each
  way. Its block column is tested against a model, natively and under Miri
  (`:contiguous_spike_test`, `:miri_contiguous_spike_sb` and `_tb`).[^spike-code]

`solver_layout` (the split-impulse solver, `//engine/std/physics2d:solver_layout`)
gained three in-place rows over contiguous blocks.

**The answer, in short:**

- **Storage could be indexed as a plain array, and would be cheaper for
  it.** A table column can be one block, its pages fixed windows of it
  (page `p` at rows `p * 16 ..`), holes and all. Every operation storage
  makes got cheaper in the spike, against today's pages:
  - spawning into a growing table: −74 to −79%;
  - a frame's despawns and spawns: −54%;
  - rows moving between pages (the spatial re-sort's moves): −40%;
  - an ordered table's re-sort: −36%;
  - walks: −38 to −53%.

  Pages as separate allocations pay for nothing storage does today.
  Borrowing is per table column (one `RwLock` over all its pages), and a
  structural change locks whole tables. A page's two real jobs work as
  windows of a block just as well: it is the unit a parallel walk splits,
  and a spatial neighborhood.
- **The solve still wouldn't work in place faster.** In contiguous world
  columns the solver's passes run within 3 to 6% of the copy at one
  thread. At eight they run 17 to 40% slower than the copy in walk order
  (22% on the settled pile), and 27 to 40% slower than the copy in entity
  order (working-sets.md).
  The solve system as a whole is 4 to 7% slower in place on the two big
  scenes, and 15% faster on the falling pile. The copy isn't, for the
  most part, working around storage. It is a transpose into the layout the
  passes want (a body's state in one 32-byte line, with the step's scratch
  in it) and the order they want (near the contacts' pair order).
  - A world column can't be either of those: it is the component's layout,
    in spatial order.
  - The parts of the copy that are biggest stay either way: the contacts'
    transpose, and resolving each contact's bodies to rows.
- **Recommendation:**
  - No in-place solve.
  - Contiguous columns (b) are worth deciding on storage's own merits, but
    it is the user's decision (get-4yr): they grow the unsafe core, and
    they give up pages as separable allocations.
  - A safe first step needs no decision: making each page whole when it's
    created (get-fqu).
  - For phase 2: contiguity would make a colored walk over world columns
    possible, at 17 to 40% of the solve, so it doesn't remove the copy that
    phase 2's rejection was about ([below](#bearing-on-parallel-relations-phase-2)).

## Why storage can't be indexed as an array today

`World` keeps a table column as `RwLock<Vec<ErasedColumn>>`: a vector of
pages, each its own `ErasedColumn` with its own allocation, grown by
doubling up to the page's rows (`erased.rs`, `world.rs`). Four things stand
between a system and `column[i]`:

1. **Pages are separate allocations**, so a value is a page pointer, then a
   row. That costs a dependent load on every touch:
   - in-place passes in the world's pages: +18 to +25% over the copy;
   - with one block per table, so a two-entry table of base pointers:
     +15 to +17%;
   - one block for every table column: +3 to +6%.
2. **Rows move every step.** The spatial re-sort moves 5 to 18% of a pile's
   rows between pages, and sleeping and waking move rows between tables.
   So contacts can only hold entities, and finding each end's row costs
   something every step:
   - through a map built from a walk of the bodies, about 100 µs for the
     walk and 7 for the map;
   - through each end's entity location, 4.8 ns a pair, 108 µs for the
     settled pile's 22 012 contacts.

   That doesn't change with contiguity.
3. **Derived values aren't stored.** The lanes' solve keeps inverse masses
   in each contact's batch, as Box2D does, so this costs only the prepare:
   130 µs on the settled pile to work them out from `Body` and `Collider`
   per contact end, against reading them from columns that keep them. The
   split-impulse solver reads masses on every touch, and there it costs
   +12% even contiguous.
4. **Two threads writing two rows of one page alias `&mut`.** A parallel
   solve in place needs the column shared, as relaxed atomics (a small
   unsafe cast) or as split borrows trusted to the coloring (unsafe that
   depends on concurrency, which storage.md rules out).

Contiguity removes only the first, and on one thread the first was the
largest part: the pages' 18 to 25% against the layout's 1 to 6%.

## How others lay out columns

Read in their fetched source, 2026-10-02: Flecs 4.0.4, Box2D v3.1.1, Rapier
0.36 and Jolt (the comparisons' Bazel repositories). Bevy, EnTT, hecs,
legion, shipyard and Unity aren't fetched anywhere on this machine and
weren't read.

| project | a column (or body array) is | grows by | removal | pointer stability | split over threads |
|---|---|---|---|---|---|
| **Flecs** | **one array per table column** (`ecs_column_t { void *data }`, `src/storage/table.h:93`), the table's count and capacity shared by all columns | power-of-two growth, a new block and a copy (`ecs_vec_set_size`, `vec.c:179`; `flecs_table_grow_data`, `table.c:1325`): pointers invalidated | swap-remove, last row moved into the hole (`flecs_table_delete`, `table.c:1537`) | none for tables. **Paged where it promises stable pointers**: sparse sets and the entity index (`sparse.c:11-15`, "provide stable pointers"; `ecs_entity_index_page_t`) | a static split of each table's rows into one contiguous range per worker (`ecs_worker_next`, `iter.c:845-897`) |
| **Box2D** | one array per solver-set column (`b2BodyStateArray`, `solver_set.h:19-43`) | 1.5×, new block and `memcpy` (`b2GrowAlloc`, `core.c:163`) | swap-remove with the moved body's index fixed up (`array.h:91`); sleeping copies a body between sets | ids stable, positions not ("pointers into these sets will be orphaned", `solver_set.h:51`) | blocks of a body or constraint array claimed by atomics (`solver.c:836-893`) |
| **Rapier** | `Arena<T>`, a `Vec` of entries with a free list (`data/arena.rs:28`) | `Vec` growth | holes on a free list | handles, not addresses | the solver works on its own copy (`SolverBodies`, rebuilt each step), split by worker (`sync.rs:135`) |
| **Jolt** | an array of `Body*`, each body allocated alone (`BodyManager.h:37`) | never: a fixed maximum, reserved at creation (`BodyManager.cpp:120`) | slots freed onto a list stored in them | `Body*` stable until deleted | ranges of the active-id list, each body found through its pointer |

- **No one reserves address space.** None of the four calls `mmap` or
  `VirtualAlloc` to keep a growing array in place.
- **Contiguous per column is the common case, paged the exception.** Flecs
  keeps table columns contiguous and grows them by moving them, and pages
  only what promises stable pointers. Ours is the reverse: table columns
  paged, nothing promising stable pointers.
- **Splitting one allocation between threads is ordinary.** Flecs and
  Box2D both hand workers disjoint ranges of one array.

## Options

What each would be, and what it costs. "Measured" points at the spikes'
results below; the rest is reasoned.

| | option | unsafe | growth | structural change, spatial re-sort | ordered re-sort | walks, split by page | page borrowing, concurrent change | reload, migration, poison mode | Miri | what a system gets |
|---|---|---|---|---|---|---|---|---|---|---|
| – | **pages (today)** | `erased.rs` as it is | a page allocated, then grown 4, 8, 16 … rows: 33 ns a row (pages of 256), 52 (16) | 31 ns a despawn or spawn; 42 a moved row | 314 µs (10 000 rows) | 21 µs (100 000 rows, pages of 256), 52 (16) | per table column today; per page possible | per page | covered (runbook 002) | a page's slice |
| a | **reserved address space** a column, committed as it grows (`mmap` `PROT_NONE`, then `mprotect`) | new: OS calls (Linux only as written), raw slots, page windows | never moves; but each commit is a system call and fresh memory faults on first touch: 31 ns a row (256), 24 (16), even committed in doubling chunks | 21; 25 | **367** (a fresh reservation) | 15; 25 | windows of one range: disjoint `&mut` by splitting, as now | a migration fills a new reservation: 423 µs, against block 95 (100 000 rows) | **no**: Miri has no `mprotect`; the spike falls back to one allocation under Miri | one base pointer for good; one index space across tables if every table is given a fixed slice of one reservation a component (address space permitting) |
| b | **a block a column**, pages as windows, relocated by `realloc` when full, only under the table's write guards | new: `alloc`/`realloc` as `ErasedColumn` has, slots by (page, row), holes inside the block, page windows carved from one cast | 8.8 ns a row (256), 10.8 (16) | **14**; **25** | **202** | **13**; **24** | the same as (a); a page can't be handed to another table or shared copy-on-write | a migration fills a new block: 95 µs | **yes**, tested (below) | a base pointer and `page * 16 + row`, valid while the system holds the guards (growth needs the write guards) |
| c | **pages during the frame, compacted into a block at its end** | as (b), and a second layout | as today's, then a copy | as today's within a frame | as today's | contiguous only where nothing changed since the last boundary | as today's | as today's | as today's | contiguous only until the table changes: a pile's adds and moves a page every step it falls. The compaction is a copy of every changed column every frame, which is the copy moved into storage and paid for every table. Not built |
| d | **an arena a column handing out pages in order**, never relocated: new chunks as it fills | as (b) | no copy; a new chunk when full | as (b) within a chunk | as (b) | as (b) | as (b) | as (b) | as (b) | contiguous only within a chunk: past the first, a (chunk, offset) index, the two-level lookup the by-table rows measure (+15 to +17% on the passes). Not built |
| e | **(b), with holes holding default values**, so a block is always fully initialized | less: a column is a plain `&mut [T]` once typed, its windows `chunks_mut`, with no uninitialized slots | as (b), plus the default glue (an `extern "C"` call a row) for each new page | a spawn replaces a default (dropping it); a despawn swaps the row with a default | as (b) | as (b) | as (b) | as (b) | as (b) | as (b). Not built: it trades the holes' `MaybeUninit` for a drop and a default per change |

The by-table and one-index-space distinction under (a) is the measured one.
With one block per table (b), a system indexing bodies in several tables
needs the table in the index and a pointer per table, measured at +15 to
+17% on the passes. One index space for every table needs (a) with a fixed
slice per table. Address space is the limit there: 4096 tables of 2^20 rows
of an 8-byte component is 32 TiB, against 128 TiB of user space.

## Spike results: physics

`taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:contiguous_spike`,
2026-10-02, Ryzen 9 7950X, one CCD, the machine otherwise idle (load under 2).
µs, the median over three runs of each run's median of 21. The scenes are
working-sets.md's, turning, on the engine's own world between frames; the
bodies are in one table (and the stand-in for statics), 9.3 rows a page.

Every way's solver output is checked bit for bit against the copy's, by
entity: each body's velocity, spin, motion and turn, and every contact's
impulses and points. So is the world after its write-back. Five
mutations of the spike were each caught by those checks.

The ways:

- **copy**: the copy as phase 1 and 3 of working-sets.md would leave it:
  one page walk over the bodies (every one turns here, checked), buffers
  kept, `Slots`, contacts through it; the lanes' solve on it; a page walk
  writing back each value only where it changed.
- **in place**: one page walk finds each row's index from its page's
  location, `(page * 16 + row)`, pages numbered across tables, holes and
  empty pages included. Contacts are found through a map from it, the
  lanes' own kernels solve in the columns by that index, and positions and
  rotations are written back from the step's scratch (how far each body
  moved and turned, by the same index).
  - The contiguous blocks are filled from the world's pages before each
    solve, untimed: they stand for storage that keeps them so.
  - The world's own pages are reached through `ColumnMut::write_all`.

| way | falling: passes / system | settled: passes / system | pyramid 5050: passes / system |
|---|---|---|---|
| copy | 648 / 1157 | 3773 / 5265 | 2407 / 3172 |
| copy, renumbered in entity order | 649 / 1230 | **3638** / 5353 | 2396 / 3185 |
| in place: contiguous columns, one index space | 666 / **980** | 3986 / 5653 | 2485 / 3294 |
| in place: contiguous, derived values stored | 661 / 976 | 3997 / 5513 | 2481 / 3296 |
| in place: contiguous, a block per table | 752 / 1120 | 4425 / 6068 | 2761 / 3633 |
| in place: the world's own pages | 803 / 1139 | 4725 / 6484 | 2853 / 3699 |
| in place: a state column (32 bytes a row: velocity, spin, scratch) | 663 / 976 | 3822 / 5375 | 2431 / 3238 |
| copy through the in-place driver (a check: the driver costs what the lanes' solve does) | 636 / 1061 | 3716 / 5238 | 2380 / 3141 |

The settled pile's system by part, copy against in place in contiguous
columns: walk 93 / 113, index 7 / 7, contacts 278 / 277, prepare 947 /
1074, passes 3773 / 3986, finish 98 / 78, write-back 93 / 90.

What it shows:

- **Pages are most of what solving in the world costs.** In the world's
  pages the passes are 18 to 25% slower than on the copy; in contiguous
  columns, 3 to 6%. A block per table, which adds one load from a table of
  two pointers before each value, is 15 to 17%.
- **What's left is layout.** The same contiguous rows holding the solver's
  own 32-byte state solve within 1 to 2% of the copy. Velocity and spin in
  their own columns, with the scratch beside them, cost 3 to 6%: a body's
  state is four places instead of one line.
- **And order, which in place can't have.** The copy renumbered in entity
  order is 3.6% faster on the settled pile's passes; the world's rows are
  in spatial order and stay there.
- **The gather mostly stays.** The in-place walk costs what the copy's
  does (it still visits every row to find its index), the contacts'
  transpose is the same 278 µs, and what in place saves is the copy back
  (finish 98 → 78) and velocities' write-back.
- **Falling, in place wins by 15%**, all of it in the prepare (295 → 118):
  the copy builds its 10 000 states every step, in fresh vectors. The
  prepare's allocations make that row the noisiest of the three.

**The passes across 8 kept threads**, each way's contacts batched on the
calling thread and only the stages timed (gravity, warm start, push, move,
relax, bounce, `run_across`'s protocol line for line), each bit for bit the
copy on one thread. In place, the threads share the columns as relaxed
atomics: each column viewed as `AtomicU32`s, the unsafe cast
parallel-relations.md costed.

| way | falling | settled | pyramid |
|---|---|---|---|
| copy, walk order (its `Atom`s, as `solve_across` shares them) | 193 | 686 | 432 |
| copy, entity order | 196 | **599** | **396** |
| in place: contiguous columns as atomic views | 271 | 837 | 504 |
| in place: a state column of `Atom`s, in world order | 198 | 673 | 426 |

In place is 17 to 40% slower than the copy in walk order, and 27 to 40%
slower than in entity order. The column layout's 3 to 6% at one thread
grows to 17 to 40% here, while the 32-byte state column stays level with
the copy.

**Open question:** whether that's false sharing (eight bodies' velocities
share a cache line, against two `Atom`s), which isn't measured.

**The split-impulse solver** (`solver_layout`, the 10 000 columns scene it
was built on, µs, the median of three runs of 31): the copy 592. In place
over contiguous `Velocity` and `Body` blocks:

- inverse masses in the constraints: 594;
- inverse masses in scratch: 590;
- inverse masses derived from `Body` on every touch: 664.

The same in the world's pages: 656, 655 and 923.

physics.md's finding, from the same bench (791 µs then), was that "it
would take storage that is one allocation per column, plus an
inverse-mass column, to solve in place as fast as on the copy." On that
solver this is measured now: contiguous columns with masses in the
constraints are the copy's time. On the solver as built, which already
carries masses in its batches, they are 3 to 6% short of it.

## Spike results: storage

`taskset -c 0-7 ./bazel run --config=bench //engine/ecs:contiguous_spike`,
2026-10-02, the median over three runs of each run's median of 21. A table
of four columns of the physics bodies' sizes (8, 8, 20 and 24 bytes), with
each value's tick beside it. Four ways:

- **paged**: as storage is today, `ErasedColumn` a page;
- **lean pages**: the spike's own block code, a block of exactly one page
  a page. It tells what separate pages cost apart from `ErasedColumn`'s
  checks and its growth by doubling within a page;
- **block (b)**: one heap allocation a column;
- **reserved (a)**: one reserved range a column, committed in doubling
  chunks of at least 64 KiB.

| | paged | lean pages | block (b) | reserved (a) |
|---|---|---|---|---|
| spawn 100 000 into an empty table, pages of 256 (ns a row) | 33.4 | 9.3 | 8.8 | 31.2 |
| the same, pages of 16 | 51.9 | 20.5 | 10.8 | 23.6 |
| 300 despawned and 300 spawned a frame, 10 000 rows in pages of 16 (ns each) | 31.0 | 26.2 | 14.4 | 20.6 |
| 5% of 10 000 rows moved between pages of 16 (ns a move) | 42.0 | 44.5 | 25.0 | 24.8 |
| 18% moved | 42.7 | 44.9 | 25.8 | 25.3 |
| re-sort 10 000 rows in pages of 16 into a new order, every column (µs) | 314 | 303 | 202 | 367 |
| walk 100 000 rows, full pages of 256: `pos += vel` (µs) | 21.3 | 21.4 | 13.1 | 14.6 |
| walk 100 000 rows, pages of 16 holding 10 (µs) | 52.1 | 69.4 | 24.4 | 25.3 |
| carving those pages into 8 tasks' runs of `&mut` (µs) | 24.3 | 46.9 | 18.0 | 18.1 |
| the walk split over 8 threads, spawned per walk (µs) | 102 | 126 | 96 | 93 |
| migrate 100 000 rows of 8 bytes to 12, pages of 256 (µs) | 73 to 297 | 93 | 85 to 95 | 423 |
| 1 000 000 values read at random by (page, row) (µs) | 552 | 539 | 543 | 531 |

What it shows:

- **The block is cheaper than pages at everything storage does.** The
  lean pages say how much of that is contiguity:
  - spawning into full 256-row pages, almost none: lean pages are the
    block's 9 ns, so paged's 33 is `ErasedColumn` growing each page by
    doubling, 4 rows to 256;
  - moves, churn and walks of small pages, all of it: lean pages are as
    slow as paged, or slower.
- **Reserved address space loses wherever it maps fresh memory**: growth,
  a re-sort into a new range, a migration. Committing a page at a time made
  growth 44 to 49 ns a row; doubling chunks, 24 to 31; the heap block's
  `realloc` is 9 to 11
  ([lore](../lore/reserved-address-space-grows-slower-than-a-reallocated-block.md)).
  Where memory is already there it matches the block.
- **Random reads by (page, row) cost the same** when they're independent:
  a page pointer's extra load is hidden. The solver's 18 to 25% is a
  dependent chain (index, page, value, then the next body's), which this
  doesn't model.
- **Migration is the allocator's.** The same paged migration took 73 µs
  in one session and 287 in the next; it allocates every page anew either
  way.

## What landing (b) would take

It is a change to `engine_ecs`'s storage and its unsafe core: the user's
decision (CLAUDE.md, "Growing the unsafe core"). If taken:

- **The unsafe code.** A block column in `erased.rs` beside or replacing
  `ErasedColumn`, about the spike's `Block` and `Region` (some 250 lines):
  - allocation, growth by `realloc` and freeing;
  - a slot by (page, row);
  - moves within a block and between blocks, with fixed-size copies as
    `copy_value`'s;
  - dropping only live slots;
  - typed windows of a page;
  - the windows carved for parallel walks (one cast of the block, then
    `chunks_mut`);
  - `gather` and `migrate` into a new block.

  The invariant grows a dimension: today "the first `len` slots are live",
  then "the first `lens[p]` slots of each window `p`". Everything above it
  (`world.rs`'s `Structural`, the queries' page access, `par`'s carving,
  the spatial and ordered re-sorts, `schema.rs`'s migration) changes from
  a vector of pages to windows of one column. `API_VERSION` bumps.
- **Testing (runbooks 002 and 003):**
  - the `columns` driver of `tests/ops.rs` over the block: moves between
    windows, holes, growth that relocates, `gather` refused partway, and a
    migration that panics, with canaries;
  - under Miri, both models, and in `//engine/ecs/fuzz:columns`, with the
    corpus replayed under Miri;
  - the `world` driver unchanged: it goes through `Structural`;
  - a reload fuzz campaign (runbook 003), since migration's path changes.

  The spike's `Block` passes a model test natively and under Miri (Stacked
  and Tree Borrows, 50 and 64 s), and eleven mutations of it are each
  caught natively. A twelfth, a hole handed out as a value, passes natively
  and is caught only by Miri.
- **What it gives up:**
  - **pages as separable allocations**: storage.md's "Later" pages shared
    between versions of the world, copy-on-write (`Arc<Page>`), and any
    page handed whole to another table;
  - **memory**: up to twice a column's rows in slack from doubling, until
    a shrink policy exists.
- **What a system would get** is a base pointer and `page * 16 + row`,
  valid while it holds the column's guards. A safe API over that is its own
  question:
  - per-access checks for holes cost what the indirection did;
  - indices only from the system's own walk would need branding.

  Today's page windows are what the spike's walks used, safely.

(a) would add `mmap` and `mprotect` behind a platform layer, which Miri
can't model (the spike falls back to an allocation under Miri, so its
reservation path isn't checked), address-space budgeting, and a
reservation kept for re-sorts and migrations to fill.

## Bearing on parallel relations' phase 2

parallel-relations.md's phase 2, a staged run any system could use on its
own copy of world data, was rejected (2026-10-02): parallel work should go
through what the ECS provides. Whether the copy could go was the open
question.

**If storage were contiguous, could the parallel solve go through an ECS
primitive?** Yes, structurally. A colored walk over a relation's rows
(parallel-relations.md's (b), `Colored<R>`) could hand each task a block
of a color's rows and the end columns as relaxed atomic views. Its
footprint would be the queries' own, and the scheduler would need nothing
new.

**At what cost, measured:**

- the passes 17 to 22% slower than the copy in walk order on the settled
  pile and the pyramid, and 27 to 40% slower than the copy in entity order
  (eight threads);
- plus everything the copy's gather also does: the walk that finds rows,
  the contacts' transpose.

That is to save what in place actually saves of the copy. On the settled
pile at one thread that is the copy back (finish, 98 → 78 µs) and
writing velocities back, while its own walk to find rows costs what the
copy's walk does (113 against 93). Only the 32-byte state column, which is
physics's own layout kept as a component, matched the copy. Even then the
copy in entity order beat it by 11% at eight threads.

**With what unsafe:**

- the block column itself, (b) above;
- the atomic view of a column: a cast of `&mut [T]` of `f32` fields to
  `&[AtomicU32]` (`AtomicU32::from_mut_slice` is unstable).

The cast is correct whatever the coloring: relaxed atomics make concurrent
access defined, and the colors buy determinism, not soundness. It's
checkable under Miri on one thread. The other way in, split borrows trusted
to the coloring, is unsafe code whose soundness depends on concurrency,
which storage.md rules out.

**So the copy is a necessity of the solver, not a limitation of storage.**
Contiguity doesn't remove it: the solver wants its own layout and order,
as every engine read for this doc and working-sets.md has them. Box2D's
`b2BodyState`s are that layout as storage; Rapier and Avian copy into it.
The decision pending on the host pool (get-znt.19, get-znt.20) couldn't
wait for storage to make the copy unnecessary. What it was about is how a
system's parallel work over its own copy is declared and scheduled. The
user has since decided it (get-znt.28, 2026-10-02): the copy becomes
declared flows ([flows-spike.md](flows-spike.md)), and parallel work is
declared and run by the scheduler, never by a system.

Contiguity would matter to a relation that makes one cheap pass over world
rows (parallel-relations.md's damage between touching pairs), where a copy
costs more than the pass. No such relation is in a game yet.

## Open questions

- **Open question:** whether a contiguous column's penalty at eight
  threads (the layout's 3 to 6% at one thread becoming 17 to 40%) is false
  sharing between colors' writes to neighbors in a cache line.
- **Open question:** whether storage.md's pages as "the unit of borrowing"
  and of concurrent structural change should still be the plan. The code
  borrows per table column and locks whole tables for structural change
  (get-00a). If page-level borrowing is to come, (b)'s windows allow it;
  copy-on-write pages don't.
- **Open question:** how much of a spawn's cost making each page whole at
  creation would save in the engine (get-fqu). The spike says lean pages
  spawn at 9 ns a row against paged's 33, with full pages.

[^spike-code]: *(History, 2026-10-02.)* The spike's code was removed once its findings were written here: spikes are built to answer a question and then thrown away. Every spike target and command named in this doc builds and runs at commit `c72e8b2` (`git checkout c72e8b2`), the last commit with every spike building.
