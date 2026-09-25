# Moving a stage to other cores moves its data

Measured 2026-09-24 with `./bazel run -c opt //engine/std/physics:tax --
parallel` on a Ryzen 9 7950X (16 cores, two dies of 8), 10 000 bodies, the
physics step's stages split across a pool of threads with every result
checked bit for bit against one thread's.

At 10 000 bodies the step's data (a few MB) lives in the cache of the core
that ran the last stage. A stage split across threads pulls its share into
other cores' caches, and whatever runs next on the calling thread pulls it
back. Stages that are mostly moving data lose more than the split saves,
on arrays as in the ECS:

| µs per step, 401 wide, settled | 1 thread | 4 threads | 16 threads |
|---|---|---|---|
| copying bodies and contacts into the solver's arrays, ECS / arrays | 84 / 37 | 170 / 104 | 184 / 115 |
| after the merge, which writes every contact: the contact re-sort, one thread | 483 | 857 | 875 |

The second row is a stage that wasn't split at all: the ordered table of
contacts is rebuilt on one thread whenever a contact begins or ends (every
step, in a creeping pile), and it got 1.8 times slower because the merge
before it had written the contacts' pages from other cores. Splitting the
rebuild's column gathers across threads too made it slower again (370 to
about 490 µs at 4 threads).

What gains is what computes much per byte it touches: the narrowphase
(2.5 times at 16 threads), the broadphase's box tests (1.2 to 1.9 in the
ECS, 3.7 on arrays).

Keeping each chunk on the same thread every step (task `k` always on
thread `k % n`, with workers kept spinning through the solver) helped the
stages that parallelize (the ECS's broadphase 1.3 to 1.8 times at 4
threads) and not the copies.

## What it means

Split a stage when its consumer is split the same way: a parallel solver
whose threads gather their own bodies, not a parallel gather feeding one
solver. A split's cost isn't only its own time; measure the frame, and the
next stage.
