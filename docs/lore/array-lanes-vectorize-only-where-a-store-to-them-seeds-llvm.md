# Array lanes vectorize only where a store to them seeds LLVM

Code written for SIMD in plain arrays (a lane type `[f32; 4]` whose every
operation is a loop over the lanes, inlined) isn't vectorized as a whole.
Once inlined, the arrays are split into scalars, and LLVM's SLP
vectorizer rebuilds vectors only from trees that end in stores to
consecutive memory. Arithmetic whose results only go back out lane by
lane, as a gather's bodies scattered to four different places, stays
scalar, four `mulss` where one `mulps` would do.

Found 2026-09-27 in the 2D solver's lanes (`engine/std/physics/solver.rs`,
`lanes`). The passes were vectorized, since each ends by storing its new
impulses into the batch's lanes (the relaxing pass: 110 `mulps`, no
`mulss`). The warm start stores nothing to the batch, only scatters
velocities, and was all scalar: 80 `mulss` and 16 `mulps` in its 420
instructions. Storing its velocities into the batch before scattering
them from there made it 28 `mulps` and no `mulss` in 234 instructions,
and the solve of a turning pile of 10 000 6% faster (4915 to 4628 µs),
the same result bit for bit. The same stores in the passes, already
vectorized, gained nothing (4631 to 4714).

## What it means

- Check the disassembly (`objdump -d`, counting `mulps` against `mulss`
  per function, with the kernel marked `#[inline(never)]` to find it)
  before trusting that array lanes are SIMD.
- A kernel whose results leave only through scattered stores needs a
  seed: a store of the lanes to memory that stays live, such as a field
  of the batch they came from.
