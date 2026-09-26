# A sphere around every box pairs every body with every wall

Measured 2026-09-26 in `//bench/physics3d` (`-c opt`, ours only, 1000
turning bodies, the mean over a 1000-step run; `--tune=bounds=...`).

The obvious way to give a turning box bounds without changing
`engine/ecs` is to keep the collider as the spatial key's extent and bound
every box by the sphere around it: the bounds no longer depend on the
rotation, so a body that only turns is never re-bounded. It made the
broadphase ten times slower on a pile of spheres, where nothing is a box
but the floor and the walls.

A wall is a box too, and a thin one. The 1000-body pile's floor has half
extents 6, 0.5 and 6, and each wall 0.5, 13 and 6, so their spheres reach
8.5 and 14.4 in every direction, past the whole pile, which is 5 either
side of the middle. Every one of them reaches every body and every page:

| 1000 bodies, turning | pairs a step | broadphase µs | narrowphase µs | step ms |
|---|---|---|---|---|
| spheres, walls' bounds a sphere | 9343 | 483 | 173 | 2.35 |
| spheres, walls' bounds their box | 4819 | 84 | 181 | 2.24 |
| boxes, every box a sphere | 14 417 | 496 | 448 | 3.20 |
| boxes, turning boxes a sphere, walls their box | 10 114 | 138 | 408 | 2.83 |
| boxes, every box the box around it as turned | 2477 | 45 | 295 | 2.58 |

Two things follow. A body that can't turn (a static, a locked body)
should keep the box around it, whatever a turning one gets. And even for
unit cubes, whose sphere is only 1.7 times as wide as the cube, the sphere
costs four times the pairs of the box turned with them: in a pile every
body's neighbours are within a sphere's reach. The step bounds each body
by its box as turned, from its `Collider` and `Rotation`, the key's two
extents (physics.md, "Rotation in 3D").
