# A Tuning in the world moves every body's entity index

Found 2026-10-03 comparing physics3d's mod fingerprint under a variant
(get-emj.90). `pile3d tune order=levels`, which names the default and
changes no physics, made all 120 of the mod's fingerprint lines differ
from the pinned ones, from frame 1, before any contact: positions,
rotations and velocities all moved.

The fingerprint hashes each value by entity (`exact.rs`, `hash`), and
`tune` spawns the `Tuning` as an entity before the scene's bodies. Every
body then gets an index one later than in the pinned run, so every hash
moves though no float does.

**Measured**: the scene with `tune lanes=4` (also the default, named)
and with `tune order=levels` print the same 120 lines; with neither,
the pinned ones. The baseline tool, which measures values and not
entities, prints byte-identical output under `TUNE=order=levels,carry=mean`.

**Resolution**: a variant's mod lines are compared with the scene run
with the default's `Tuning` named, never with the pinned lines
(`exact_main.rs` under `TUNE`, and `exact_test`'s
`the_mod_solves_in_the_order_its_world_sets`).

A second trap from the same work: `MATRIX=1` prints every bound's
`CHECK` line, but a passing test's output is kept by the test harness,
so the listing of a run where most tests pass needs
`--test_arg=--nocapture` too.
