# CPUs 16 to 31 are the same cores as 0 to 15

Measured 2026-09-27 on the development machine (a Ryzen 9 7950X, 16 cores,
two threads each), with `//engine/ecs:spatial_bench` pinned to CPU 3.

The benching rule so far was "Bazel on CPUs 16-31, benches on 0-15", to
keep builds off the cores being measured. On this machine that does the
opposite: `lscpu -e` numbers the second thread of core `n` as CPU `n + 16`,
so CPU 19 is core 3's other half. With one spinning process beside the
bench:

| beside the bench | `near_pairs`, 10 000 rows and walls | re-sort, 10 000 creeping |
|---|---|---|
| nothing | 363-366 µs | 90.3-90.7 µs |
| a spin on CPU 19 (the same core) | 538 | 155.5 |
| a spin on CPU 5 (another core) | 369 | 90.8 |

A build on 16-31 while a bench runs on 0-15 slows it by half, however it
is pinned. Split by core instead: CPUs 0-7 are the first eight cores (one
L3, `lscpu -e`'s last cache column), 8-15 the other eight, and 16-23 and
24-31 their second threads; so benches on 0-7 and builds on 8-15,24-31
share neither a core nor an L3. Or, simplest, don't build while
measuring. Check `lscpu -e` on another machine: the numbering is the
kernel's, not a rule.
