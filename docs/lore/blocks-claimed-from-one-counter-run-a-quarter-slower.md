# Blocks claimed from one counter run a quarter slower

The obvious way for a scheduler to hand out a stage's blocks is one
atomic counter, `fetch_add` a block (Jolt's `FetchNextBatch` is one). On
2D's colored solve it is 17 to 27% slower than taking each thread's own
share first, and not because of the counter: the blocks themselves run
longer.

Measured 2026-10-03 in the dispatch spike
([dispatch-spike.md](../architecture/dispatch-spike.md)), the 2D solve's
passes at 8 threads on one CCD, the same blocks and kernels, a trace of
every block's time. "Own share": Box2D's claim, a thread starting at
`GetWorkerStartIndex` and raising each block's mark forward then back.

| scene | passes, own share | one counter | busy in blocks, own share | one counter |
|---|---|---|---|---|
| 2D pile of 10 000, settled | 642 µs | 818 | 4184 µs | 5505 (+32%) |
| 2D pyramid of 5050 | 417 | 519 | 2737 | 3449 (+26%) |
| 2D pile falling | 177 | 207 | 895 | 1155 (+29%) |
| 3D planks, 1000 | 578 | 673 | 3194 | 3794 (+19%) |
| 3D boxes, 10 000 | 2980 | 2928 | 17 827 | 19 014 (+7%) |

The idle time is about the same either way. With the counter, a block
lands on whichever thread is free first, so the batches of a color, and
the bodies they write, move to another core's L2 at each stage; claimed
from its own share, a thread gets the same blocks pass after pass and
finds them in its cache. The dispatchers alone, with blocks that only
spin, show nothing of this (0.48 against 0.41 µs a stage with 1 µs
blocks): it is the data, not the atomic.

The 3D boxes don't lose: their blocks are large (10 µs a stage), so the
transfer is small next to the work, and the counter's better balance
pays for it (2% faster). 3D's planks and 2D's small blocks lose most.

## What it means

A scheduler's stage dispatch should keep a block on the thread that ran
it last: claim by marks from the thread's own share (Box2D, Box3D,
Rapier 0.36 all do), not by one counter, and not by work stealing, which
moves a block to a thief's cold cache (`steal` in the spike: 1.7 to 2.5
times slower). See also
[moving-a-stage-to-other-cores-moves-its-data.md](moving-a-stage-to-other-cores-moves-its-data.md).
