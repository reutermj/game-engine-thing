# A soft step with gravity all in its first substep never settles

Measured 2026-09-26 with `//engine/std/physics/compare`, `SETTLE=1500`,
the staggered 10 000 pile: our soft step (Box2D v3's, 4 substeps, 30 Hz,
one relax pass) with the step's gravity added before the solve, as our
`integrate_velocities` adds it, never came to rest (a body at 2.3 u/s at
step 400); the same solver adding a quarter of it in each substep was at
rest by 280, as Box2D is.

Why: with gravity all in the first substep, the first substep's impulses
carry the whole load and the other three carry almost none. Friction is
limited by the normal impulse of its own substep, so in substeps 2-4 there
is next to no friction, while the soft contacts still push bodies apart
along tilted normals: the split impulse's old creep (physics.md,
"Settling") come back. Counting impulses over the whole step instead (so
friction has the step's load) settles (230-290) but sinks 0.21-0.26 deep,
since then the soft spring lets go of the whole step's impulse at once.

So the solver takes gravity back out of the velocity it's handed and gives
it a substep at a time (`SolverBody::gravity`), which means a body in free
fall moves g h^2 (1 + 2 + ... + n) a step, n substeps of h, instead of
g dt^2: a little less far, 0.0033 a step at gravity 20 and 5 substeps.
