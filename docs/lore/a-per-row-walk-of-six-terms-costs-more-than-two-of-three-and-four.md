# A per-row walk of six terms costs more than two of three and four

Measured 2026-10-02 with
`taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:working_set_spike`,
three runs of 21. The solve's gather was run over the awake bodies of a 2D
pile of 10 000, every one of them turning, on spatial pages. Each way
appends the same values, checked bit for bit, and the times are µs for the
bodies plus their angular state:

| how | falling | settled |
|---|---|---|
| two `for_each` walks: `(Body, Velocity, Position)`, then `(Body, Collider, Rotation, Spin)` with a lookup by entity | 117 (38 + 79) | 141 (41 + 100) |
| one `for_each` walk of all six terms | 139 | 161 |
| the same two walks as `for_each_page`, each term's slice taken once a page | 102 (34 + 68) | 128 (41 + 87) |
| the one walk of six terms as `for_each_page` | 94 | 116 |

The surprise is the second row. Merging the walks drops 10 000 lookups by
entity and a second visit to every page, and per row it still came out
about 2 ns slower. As page walks the order flips, and one walk wins by 8
to 12 µs. The write-back behaves the same way: as built with `for_each` it
is 79 µs falling and 118 settled, as page walks 69 and 98.

The cause wasn't found (there is no profiler on the machine). The likeliest
is the per-row path of a six-term tuple, four of whose terms are `&mut`,
each handed out as a `Mut`, not being straight-line code, as with
[a query's row cost its dispatch](a-query-row-cost-its-dispatch-not-its-data.md).
That guess is inferred from the page walk being faster, not read in the
assembly.

So a walk of many terms, or of several written terms, should be measured as
a page walk before a design is judged on it. physics.md found a page walk
no faster than `for_each` for reads (2026-09-24, before rotation), and that
held for the walks it measured then, not for these. Optional query terms
pay only as page walks: working-sets.md, "Spike results: physics".
