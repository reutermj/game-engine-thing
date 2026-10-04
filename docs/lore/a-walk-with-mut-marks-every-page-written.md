# A walk with `&mut` marks every page written

Measured 2026-10-04 in the presentation spike (get-3hd.1,
`//spikes/presentation:extract_bench`, one thread;
[presentation-spike.md](../architecture/presentation-spike.md), section 4).

`Query::for_each_written(since, ..)` skips a page whose tick is no later
than `since`, so a walk for what changed should cost a look per page when
little has. But a system that takes a `&mut` term over a page marks the
whole page written at once, whether or not it writes a row:
`ErasedColumn::as_mut_slice_ticked` sets the column's `written` to `now`
before handing out the `Mut`s, and the per-row ticks record only real
writes. A game that moves 1% of its rows from a `for_each` over
`Query<&mut Place>` therefore leaves every page looking written, and
`for_each_written` checks every row's ticks.

The extract over `(&Place, &Look)` at 100 000 rows, µs a frame:

| 1% of rows moved | full walk | `for_each_written` (delta) |
|---|---|---|
| by a `for_each` with `&mut Place` over all of them | 109 | 340 |
| by `Query::with(entity)` on the movers only, one block | 105 | 11.8 |

Through the walk, change detection costs about 3.4 ns a row, three times
a full extract's 1.1 ns. The per-row check reads two tick arrays through
the query's per-row fetch, not a page's slices. Writing by entity marks
only the movers' pages, and then change detection beats the full walk
nine times over.

**What it means:** change ticks pay off only when the writers touch few
pages. Physics, which writes every awake body each step through a page
walk, makes every page of its components look written every step.
