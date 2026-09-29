# The scheduler spreads a pool over both CCDs, and a colored solve halves

Measured 2026-09-29 on the development machine (Ryzen 9 7950X: 16 cores
on two CCDs of 8, each with its own L3; CPUs 0-7 one CCD, 8-15 the other,
16-31 their SMT siblings), `schedutil` governor, threads warmed 300 ms
first. The colored solve across threads (`solver::solve_across`) of the
turning pile of 10 000, `solver_bench` with `THREADS`, kept threads
(`tests/pool.rs`), µs a solve (one thread: 4270), placed only by the
process's affinity (`taskset`):

| threads | one CCD (0-7; 16: 0-7,16-23) | both, a thread a core (0-15) | anywhere (0-31) |
|---|---|---|---|
| 2 | 2508 | – | 3968 |
| 4 | 1397 | – | 2418 |
| 8 | 882 | 1765 | 1797 |
| 16 | 829 | 1512 | 1534 |

Left to the kernel, a pool's threads land on both CCDs, and every stage
then passes bodies across the link between them: two threads anywhere
solve at 1.08 times one, where on one CCD they're 1.7 times. Sixteen
cores on both CCDs are slower than eight on one. Box2D's own
multithreaded step is hit the same way (the comparison, the settled pile
turning: 16 threads 720 µs a step on one CCD with SMT, 1324 on both), so
it's the shape of a colored solve, a barrier a color with bodies read
across it, not our code.

So a host pool for physics has to place its threads (one CCD first), not
leave it to the scheduler, and that takes `sched_setaffinity`: an FFI
call, or a crate that wraps one. The benchmarks here get it from
`taskset` instead (docs/architecture/physics.md, "Solving across threads";
get-znt.5).
