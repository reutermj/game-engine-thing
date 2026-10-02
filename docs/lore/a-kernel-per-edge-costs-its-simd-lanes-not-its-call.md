# A kernel per edge costs its SIMD lanes, not its call

Measured 2026-10-02 in `//engine/std/physics2d/compare:flows_spike`
(`--config=bench`, `taskset -c 0-7`, Ryzen 9 7950X, the medians of three
runs of 21), building a generic colored primitive
(`flows::Colored::passes`, docs/architecture/flows-spike.md) and running
physics's contact kernels through it, every way bit for bit the solve as
built. The settled pile of 10 000 (22 012 contacts), the solver alone, µs:

| how the kernel is reached | 1 thread | 8 threads |
|---|---|---|
| hand-tuned staged run, 4 lanes | 4902 | 978 |
| generic, a closure per batch of 4 | 4925 to 5240 | 1000 |
| generic, a closure per block of batches of 4 | 4905 | 1003 |
| generic, a batch of 4 through `&dyn Fn` | 5066 | 1038 |
| hand-tuned, 8 lanes | 4893 | 970 |
| generic, a closure per batch of 8 | 5694 | 1053 |
| generic, a closure per block of batches of 8 | 4901 | 991 |
| hand-tuned, 1 lane | 10 540 | 1760 |
| generic, a closure per edge, handed its two states | 10 428 | 1810 |

What it says:

- **A closure per edge is free; one lane per edge isn't.** Handed its two
  states, a kernel per edge is the hand-tuned solve at one lane to within
  3.5%. Both are 1.6 to 2.1 times the four-lane solve, on this scene and
  the two others measured. The cost of the
  per-edge shape is the SIMD it gives up, not the call, which the
  compiler inlines when the primitive is generic over the closure.
- **A call through a pointer costs 1 to 4%**, once a batch.
- **Deciding the pass per item costs at 8 lanes.** The primitive's first
  shape called one closure per batch with the pass as an argument (warm
  start, push, relax or bounce), matched inside it. At four lanes that
  was within noise of the hand-tuned run. At eight it was 7 to 16% slower,
  though the batches were the same. Handing the closure a block of
  batches, matched once and looped inside each arm as `run_across` does,
  brought eight lanes back to the hand-tuned time. Inferred, not traced:
  with the match inside the item loop, the five eight-lane kernels inline
  into one body, and its register allocation or code size suffers where
  the four-lane one doesn't.

So a generic parallel primitive should take its kernel over a slice of
items, with the pass decided outside the loop.

*(History, 2026-10-02: the spike this was measured in has been removed; it builds and runs at commit `c72e8b2`.)*
