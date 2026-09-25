# Memory a task allocates is its thread's

Measured 2026-09-24 with `./bazel run -c opt //engine/std/physics:tax --
parallel`, 10 000 bodies settled in a box 401 wide, one run each, on a
Ryzen 9 7950X, µs per step for the solver's gathering (bodies and contacts
copied out of the world into the arrays the solver works on):

| the lists each task fills | 1 thread | 1 thread, split in chunks | 4 threads | 16 threads |
|---|---|---|---|---|
| made by the calling thread, with room for the chunk | 85 | 121 | 171 | 185 |
| made by the task, grown as it pushes | 83 | 133 | 361 | 604 |

Each chunk of the walk fills three `Vec`s (entities, solver bodies, kinds),
joined in order after. Made on the calling thread with their capacity, and
freed there after the join, the split costs what copying the results
together costs. Made empty by the task and grown by `push`, the same work
took twice as long at 4 threads and three times at 16, though at one thread
(every task on the caller) the difference is a tenth.

The arrays' version of the same stage (`tax_par.rs`, `fill`, with
`TASK_ALLOC` set) showed no difference either way: it fills each list with
one `extend` from an iterator of known length, which allocates once. So the
cost is growing, not allocating: reallocations on a worker thread.

Why, inferred rather than traced: glibc gives each thread that allocates its
own arena, so a worker's buffers come from memory fresh to it, which a
reallocation moves again, and the join then frees them from another thread,
into an arena that isn't its own. At a step's rate, that is new pages to
fault in on every step.

## What it means

A task's outputs are made by the thread that makes the task: `make` in
`Query::par_for_each_page` and `par_for_each` is called on the calling
thread for that reason, with the chunk's rows so it can give the room, and
`near_pairs_with` makes its buckets and sort buffers there too
(`SortScratch`). Freeing them back on the calling thread, after the join, is
part of the same rule.
