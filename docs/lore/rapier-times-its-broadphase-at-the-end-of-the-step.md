# Rapier times its broadphase at the end of the step, and only with `profiler`

Found 2026-09-25 building `//engine/std/physics/compare` on `rapier2d`
0.36.0, read in its source and measured.

- **Without the `profiler` feature every counter reads 0.** `Timer`'s
  `start`, `pause` and `resume` are `#[cfg(feature = "profiler")]`
  (`src/counters/timer.rs`), so `counters.enable()` alone gives zeros
  and no error. The comparison turns the feature on in its `Cargo.toml`.
- **`cd.broad_phase_time` is not the broadphase.** Rapier updates its BVH
  for the positions just solved at the end of a step, timed as
  `cd.final_broad_phase_time` (`physics_pipeline/substep.rs`), and
  `cd.broad_phase_time` covers only the pair update at the start of the
  next (`physics_pipeline/solve.rs`). On a settled pile of 10 000 the
  first read 0 µs and the second 67; falling, 384 and 283. The
  comparison counts both as the broadphase.
- **The counters are per step**: `counters.reset()` runs at the start of
  every `step`, so read them after each one.
