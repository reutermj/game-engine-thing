# `f32::floor` is a function call on the default target

The build targets baseline x86-64, which has no SSE4.1, so no `roundss`:
`f32::floor` (and `f64::floor`) can't be one instruction, and LLVM lowers it
to a call to `floorf`, through the GOT, into a branchy software version. In
a hot loop that's a call per value, with the caller's registers saved
around it, and it can't be vectorized.

Established by disassembly (2026-09-24): a probe `fn f(v: f32) -> i64 {
v.floor() as i64 }` built with `./bazel build -c opt` compiles to `call
*GOT` then `cvttss2si`, and the binary carries a static `floorf` of about
thirty instructions with branches. Seen again with rustc 1.98 `-O`
directly (`callq *floorf@GOTPCREL(%rip)`). Measured in the spatial re-sort
(`engine/ecs/spatial.rs`), warm, over 10 000 rows with two floors each: 46
µs with `floor`, 23 µs truncating instead; and, separately, in the physics
pile's re-sort with the per-row glue of the time, re-bounding 10 000 rows
went from 166 to 139 µs.

## Resolution

Where the value's sign is known, or can be made known, truncate instead:
`spatial.rs`'s `cell` adds the 2^20-cell offset before casting, so the
cast (which truncates toward zero) is the floor. Where it can't, `let t = v
as i64; if (t as f32) > v { t - 1 } else { t }` is exact for every value a
cast doesn't saturate. Raising the target (`-C target-cpu`) would fix it
everywhere, but ties the build to the machines it runs on.
