# Relaxed atomic floats still vectorize, at a movd a load

Measured 2026-09-29 in `//engine/std/physics/compare:solver_bench`
(`--config=bench`, the turning pile of 10 000 and the 5050 pyramid, Ryzen 9
7950X), building the colored solve across threads
(`solver::solve_across`). Its bodies are shared between threads as
`AtomicU32`s, an `f32`'s bits each, loaded and stored `Relaxed`
(`lanes::Atom`), so that threads writing different bodies of one color
need no unsafe code; the one-thread solve keeps a plain array. The kernels
are one generic function over both (`lanes::Bodies`).

- **It still vectorizes.** The shared kernels' closure has 274 `mulps` to
  38 `mulss` (the one-thread solve, `solve_points`: 345 to 61): LLVM loads
  the lanes one at a time either way (a gather), and the arithmetic on
  them is packed as before.
- **Each load is two instructions.** A relaxed atomic `u32` load is a `mov`
  into an integer register, then a `movd` into an SSE one, where a plain
  `f32` load is one `movss`: 263 `movd` against 44. Stores likewise. LLVM
  also keeps an atomic load whose value is unused (it treats any atomic
  stronger than unordered as possibly writing), so the passes that read
  only velocities load only those (`Bodies::load_v`).
- **What it costs.** The shared path run with all its tasks on the calling
  thread, one after another (`POOL=late`), is 8-9% slower than the
  one-thread solve (pile 4287 against 4678-4708 µs, pyramid 2850 against
  3083-3117); the same with the bodies in a plain array behind a raw
  pointer (an `unsafe` spike, not kept) is 1-2% slower. Across 8 threads
  on one CCD the spike was 2.5-3% faster than the atomics (pile 849
  against 871 µs, pyramid 563 against 581), at 16 about 4%: the loads are
  a small share once the stages are split, and the rest (barriers,
  coloring, the step's serial parts) doesn't change.

So safe sharing costs about 3% of a parallel solve here, which is what
the unsafe version would have bought, with Miri and fuzz coverage of
concurrent code to earn it (physics.md, "Solving across threads").
