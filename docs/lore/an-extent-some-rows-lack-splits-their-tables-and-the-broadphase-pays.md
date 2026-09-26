# An extent some rows lack splits their tables, and the broadphase pays

Measured 2026-09-26 with `./bazel run -c opt //engine/ecs:spatial_turn_bench`
(10 000 boxes a unit apart, one in ten turning, creeping):

| how the rotation is kept | tables | `near_pairs` | pairs |
|---|---|---|---|
| in the key, on every row (`Pose`) | 1 | 151 µs | 8613 |
| as a second extent, on the rows that turn | 2 | 186 µs | 8613 |

The same boxes, the same pairs, 23% slower. A component only some rows
have puts those rows in a table of their own, and each spatial table keeps
its own order: two orders of pages over the same ground, each page's box
covering the gaps where the other table's rows are. The broadphase sweeps
both and tests the pages of each against the other's, more page pairs for
the same row pairs.

It doesn't show where every row has the extent or none does (the costs
are the same as one table's), and a spatially separate split (a level's
tiles, apart from its bodies) is what two sides are for. It is the price
of an optional component on a spatial key, and a reason to keep a world's
turning and non-turning bodies from interleaving finely. See
spatial-storage.md, "Bounds from several components".
