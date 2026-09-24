# A query's row cost its dispatch, not its data

Measured 2026-09-24 with `./bazel run -c opt //engine/ecs:query_bench`, 10 000
rows of a spatial table (pages of about 12 rows), copying two components
out per row into a `Vec`:

| `for_each` | ns per row |
|---|---|
| as it was | 5.4 |
| `#[inline(always)]` on `Term::at`/`page` and `Data::at`/`pages` | 3.3 |
| and, for table-only queries, each term's page slice indexed per row | 2.1 |
| the same on a plain table (pages of 256) | 1.0 |
| copying from a `Vec` of the values | 0.4 to 1.4 |

What's surprising is that the first two costs are the same code. `for_each`
already took each term's slice once a page; per row it only matched each
term's `PageView` (read, write, or sparse set) and built an `Option` per
term. Those are generic and small, yet in the crate that monomorphizes them
they weren't inlined through the tuple impls: half the row's cost was calls
(inferred from forcing the inlining, not read in the assembly). With them inlined, the per-row match on the view is still there,
because the compiler can't know a view is a slice. A table-only query (every
term's `Component::STORAGE` is `Table`, a constant) with no sparse filter
matches every row of every page, so `Query::walk` indexes the slices and
dispatches nothing per row.

So before blaming a query's cost on copying or on storage, check whether the
per-row path is straight-line code: the physics mod's gathers and scatters
had been attributed to copying, and most of their cost went with this
(physics.md, "What the ECS costs").

See also [an assert in a hot accessor](an-assert-in-a-hot-accessor-can-cost-a-quarter-of-the-loop.md),
the same kind of cost found in `near_pairs`.
