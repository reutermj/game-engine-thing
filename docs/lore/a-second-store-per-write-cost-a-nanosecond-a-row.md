# A second store per write cost a nanosecond a row

Measured 2026-09-24 with `./bazel run -c opt //engine/std/physics:tax`, the
pile of 10 000 settled, three runs each.

Change detection keeps a tick per value, which `Mut<T>` stamps when written
through. To let a walk skip pages none of whose rows changed
(`Query::for_each_written`), each page's column got a tick of its own, and
the first try stamped it where the row's is: `Mut::deref_mut` did
`*page = (*page).max(now)` beside `*tick = now`, as did `ColumnMut::set`.
Physics's gravity, which writes `v.x` and `v.y` of every body through one
`Mut`, went from 21–22 µs to 33–34, and writing back from 68 to 75: about
1.2 ns a row, for a load, a max and a store to a location the same on every
row of a page.

Stamping the page once where writes are handed out instead (a page's view
or slice taken for writing, or a row's `Mut` by lookup), whether or not one
is then made, took both back to 22 and 68. The page's tick is then "may
have been written since", which is all a skip needs: a page stamped for
nothing costs a look at its rows' ticks, which are exact.

Why the store cost that much wasn't established (measured, not read in the
assembly). The likely cause is aliasing: the page tick is reached through a
`&mut u32` the compiler can't prove apart from the values or the row ticks,
so each write's load and store stay in the loop, twice a row here. Either
way, a per-write cost in `Mut` is paid by every system that writes, so the
cheaper place for bookkeeping is where a page is handed out, once.

See also [a query's row cost its dispatch](a-query-row-cost-its-dispatch-not-its-data.md).
