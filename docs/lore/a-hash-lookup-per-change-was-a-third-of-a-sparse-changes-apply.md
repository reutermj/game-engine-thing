# A hash lookup per change was a third of a sparse change's apply

Measured 2026-09-26 with `./bazel run -c opt //engine/ecs:change_bench`
(get-znt.18), ns per change at 1000 changes a run, medians of three runs:

| apply, per change | insert | remove |
|---|---|---|
| guards in a `HashMap<ComponentId, _>` | 30.5 | 24.8 |
| guards in a `Vec` by component id | 18.7 | 10.4 |

`Structural` kept its sparse guards in a std `HashMap`, and every sparse
change looked its set up twice (`lock_sparse`'s `contains_key`, then
`get_mut`), each a SipHash of a `u32`. That was nothing next to a table
move (about 190 ns), which is where the apply node's cost had been looked
for, but it was a third to a half of a sparse change's, whose real work
(a slot lookup and a push) is a few nanoseconds. Tables were already held
in a `Vec` by id, for the same reason (a hash per spawn's lookup was a
fifth of the spawn); component ids are dense too.

So when a per-item path is a few nanoseconds of real work, look for a
std `HashMap` on it first: the default hasher costs about as much as the
work. Batching the changes (runs, one lookup a run) removed the lookups
from the hot path altogether, down to 5.5 and 4.3 ns; the guards stayed by
id for the paths that still look up per change.
