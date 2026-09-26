# Turning spheres roll off a floor in every engine

Measured 2026-09-26 in `//bench/physics3d` (`-c opt`, `--rotate`, one
thread, sleeping off, every engine at its defaults). The rain scene drops
1000 spheres and boxes, alternating, onto a floor 10 wider than the heap on
every side, with no walls. Locked, nothing ever left it. Turning, about one
in twenty did, in each engine, falling off the edge at up to 73 m/s by the
end of the run:

| rain, 1000, turning, no walls | Rapier 0.36 | Jolt 5.6 | Box3D 0.1 | ours |
|---|---|---|---|---|
| bodies off the floor | 48 | 52 | 60 | 59 |
| bodies still moving at the end | 153 | 121 | 141 | 157 |

They are spheres rolling: a sphere touches the floor at one point, and
friction there only makes it roll, it can't stop it. None of the four has
rolling resistance on by default (Box3D's material `rollingResistance` is
0, `types.c`), so a rolling sphere rolls until it meets something. A pile of
1000 turning spheres in a walled box is never at rest (every body under
0.05) within 1000 steps in Jolt, Box3D or ours, and was once, at step 651,
in Rapier.

So the rain scene has low walls now, for every engine (they change nothing
locked), and "settled" on turning spheres measures rolling, not the solver.
A test that a sphere stops needs rolling resistance, which none of these
engines give by default; `engine/std/physics3d/tests/physics3d_test.rs` checks what friction does
do instead: a sliding sphere rolls at 5/7 of its speed, a spinning one at
2/7 of its spin times its radius.
