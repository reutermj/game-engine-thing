# Reserved address space grows slower than a reallocated block

Measured 2026-10-02 with
`taskset -c 0-7 ./bazel run --config=bench //engine/ecs:contiguous_spike`,
three runs of 21, on Linux 6.1. The spike keeps a table of four columns (8,
8, 20 and 24 bytes, each value's tick beside it) in pages, and spawns 100
000 rows into it from empty, two ways:

- **reserved**: each column a range of address space reserved up front
  (`mmap`, `PROT_NONE`, `MAP_NORESERVE`), committed with `mprotect` as
  pages are added. It never moves;
- **block**: each column one heap allocation, grown by `realloc` as pages
  are added. It moves when it grows.

ns a spawned row:

| | pages of 256 | pages of 16 |
|---|---|---|
| reserved, committed an OS page (4 KiB) at a time | 49 | 44 |
| reserved, committed in doubling chunks of at least 64 KiB | 31 | 24 |
| block, `realloc` doubling | 9 | 11 |

The reserved range is the usual answer to "contiguous, and never
relocated", and it lost to relocating by three to four times. The
commits' system calls were part of it: chunking them took a third off.
What's left wasn't isolated (no profiler on this machine). The likeliest
cause is that every byte of a fresh mapping faults on first touch and is
zeroed by the kernel, while the allocator hands back memory it already
has. Each rep builds a new table, and each new reservation is fresh
memory. The same held for a whole-table re-sort into a new reservation
(367 µs against the block's 202) and for a migration into one (423 µs
against 95).

Where the memory was already committed, the two were level: moving rows
between pages (25 ns a move each) and walking them.

So a reservation should be kept and refilled rather than made again, if
one is used at all. "It never moves" is bought with the first touch of
every page, every time the range is new. docs/architecture/contiguous-columns.md
has the rest of the comparison.

*(History, 2026-10-02: the spike this was measured in has been removed; it builds and runs at commit `c72e8b2`.)*
