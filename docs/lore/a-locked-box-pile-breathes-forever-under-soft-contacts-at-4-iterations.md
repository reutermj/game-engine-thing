# A locked box pile breathes forever under soft contacts at 4 iterations

Measured 2026-09-25 in `//bench/physics3d` (`-c opt`, cubes of half-extent
0.5 dropped into a walled box, rotation locked, friction 0.5, sleeping off,
default solver settings). After 1000 steps (16 s) the 1000-cube pile is
still moving in Rapier 0.36 and Box3D 0.1, and at rest in Jolt 5.6:

| 1000 cubes, step 1000 | Rapier | Jolt | Box3D |
|---|---|---|---|
| bodies at 0.05 m/s or faster | 858 | 0 | 444 |
| top speed | 0.64 | 0.00 | 0.43 |
| kinetic energy | 25.2 | 0.0 | 1.9 |

At 10 000 cubes (1500 steps) it is 9370 of 10 000 in Rapier and 9444 in
Box3D, with kinetic energy 722 and 547; Jolt 0. The same scene with spheres
settles in all three.

It is not creep or collapse. Printing the mean vertical velocity and height
of all bodies over the last 8 steps shows the whole pile moving up and down
together: mean vy swings between about -0.21 and +0.24 m/s with a period of
about 12 steps (5 Hz at dt 1/60), and the mean height by about 7 mm around
10.20, the same mean height as Jolt's pile at rest. Sideways velocity is
near zero (mean 0.0002 m/s). Turning Rapier's contact recycling or contact
clustering off doesn't change it.

It is the iteration count, though. With 8 (Rapier's num_solver_iterations,
Box3D's substeps; `--iters8`) the same piles come to rest: 1000 cubes
settle by step 69 in both, 10 000 by step 142 (Rapier) and 154 (Box3D),
with nothing moving at the end. Sleeping doesn't help at 4: a body moving
at 0.2 m/s never falls asleep, so `--sleep` leaves both tables unchanged.

The inferred cause is that both engines make contacts soft: a spring and a
damper, tuned per step (Rapier's contact_softness, Box3D's contactHertz and
damping ratio). A column of face-to-face cubes is then a chain of springs,
and with rotation locked no energy leaks into rocking; 4 iterations leave
the chain's slowest mode under-damped, 8 converge it. Spheres touch at
points spread in every direction, which couples and damps the modes. Jolt
solves hard contacts and then corrects positions, and has no spring to ring.

So on a box pile at default settings, "settled at step" and "moving" in the
bench's quality table measure this mode, not a pile falling apart: mean
height and penetration still show it standing. A solver with soft contacts
should expect the same, and look at the mean vertical velocity over a few
steps to tell the mode from a real failure to converge.
