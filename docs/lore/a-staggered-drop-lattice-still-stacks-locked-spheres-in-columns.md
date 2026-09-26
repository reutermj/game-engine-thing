# A staggered drop lattice still stacks locked spheres in columns

The 3D counterpart of
[a-pile-41-wide-stands-in-columns-until-it-is-tall.md](a-pile-41-wide-stands-in-columns-until-it-is-tall.md).
Measured 2026-09-25 in `//bench/physics3d` (`-c opt`, 1000 spheres of
radius 0.5 dropped into a 10 x 10 box, rotation locked, friction 0.5, 1000
steps). "Not columns" is the fraction of bodies resting on another dynamic
body whose every such support is more than 0.1 off to the side
(`measure.rs`); a real pile is near 1.

| drop lattice | Rapier | Jolt | Box3D |
|---|---|---|---|
| cell 1.25, alternate layers shifted half a cell, jitter 0.1 | 0.39 | 0.42 | 0.40 |
| each layer shifted by its own random fraction of half a cell | 0.96 | 0.96 | 0.96 |

The half-cell stagger looks like it prevents columns, since no body starts
above the one below it. But layers two apart start aligned to within the
jitter, and the cloud falls as one, so it lands in that order: 60% of the
spheres ended up standing on another almost straight below. They stay
there because with rotation locked a sphere can't roll off. Friction alone
holds it on a support up to atan(0.5) = 26.6 degrees off vertical
(inferred, not measured), which is 0.45 sideways for unit spheres. All
three engines agree, so this is the scene, not a solver.

The rain scene did the same through its spawn grid. Spawning in random
cells of a 1.25 grid, with each cell reserved until its last body fell
clear, gave stacks about four high (0.52 to 0.59 not columns, at most 2 or 3
partners a body). Continuous random spots, refused within 1.1 sideways of
any recent spawn, gave 0.97 to 0.98.

So a regular spawn pattern of any kind can come back as columns when
rotation is locked. Check contacts per body and the sideways offset of
supports on every engine before trusting a pile.
