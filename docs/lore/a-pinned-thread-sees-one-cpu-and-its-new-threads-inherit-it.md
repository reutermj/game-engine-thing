# A pinned thread sees one CPU, and its new threads inherit it

`core_affinity::get_core_ids()` is the CPUs the *calling thread* may run
on (`sched_getaffinity(0)`), not the process's, and a thread spawned
inherits its spawner's mask. So once the pool pins the frame's thread to
the CCD's first core (threads.md, "Placement"), a second pool made on
that thread, asking where it may go, is told one CPU: it places the whole
CCD on that CPU, its unpinned workers inherit the same one-CPU mask, and
all eight threads share one core.

Found 2026-10-03 (get-znt.34) as a step_bench that was fast for its first
run and slow for every later one in the same process, each run a fresh
engine and pool. **Measured** (2D pile of 10 000 falling, 8 threads, µs of
passes a step, run by run): 117, then 3170, 3783, 2933. Reading the
process's CPUs once, before any pinning (`engine_threads`'s `allowed`, a
`OnceLock`), gave 117, 119, 118, 118. `pool_test`'s
`a_pool_made_from_a_pinned_thread_is_placed_as_the_first` holds it: with
the CPUs read from the calling thread again, the second pool is placed on
`[0]`. That the unpinned workers then share CPU 0 is inferred from
inheritance, not sampled.

Keeping the first answer isn't enough where the pool comes from a mod:
each load of the `threads` mod is a fresh copy of that `OnceLock`, so a
second engine in one process (pile3d's step_bench, three runs) asked
again from the pinned thread, and its passes took 107 ms a step against
4.3. `allowed` now also takes a mask of one CPU for a pinned thread's and
lets the topology alone place the pool.

## What it means

- Find a placement before pinning anything, or from a thread nothing
  pinned. The game is safe either way (the `threads` mod places its pool
  at load, before any frame), but a pool made again later (`threads 4`
  sent to it, a bench's second run) would not be.
- A thread a pool spawns unpinned is not free: it inherits wherever its
  spawner was confined.
- `/proc/<pid>/task/*/status`'s `Cpus_allowed_list` is the quick check.
