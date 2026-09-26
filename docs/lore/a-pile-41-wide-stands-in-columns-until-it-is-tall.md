# A pile 41 wide stands in columns until it's tall

Measured 2026-09-25 in `physics_test`, the pile scene (`tests/pile.rs`)
after 600 steps, contacts per body:

| box | 200 bodies | 300 | 500 | 1000 |
|---|---|---|---|---|
| 40 wide | 1.00 | 1.00 | 1.00 | 1.00 |
| 41 wide | 1.00 | 1.00 | 1.08 | 1.24 pressed (1.47 all) |
| 43 wide | 1.00 | 1.00 | 1.20 | 1.37 |

The pile drops rows that alternate circles and boxes, a little apart and
jittered. At 40 wide a row holds an odd count, so each column alternates
too, and without rotation a circle on a box stays put: columns that never
touch, the degenerate scene the physics retrospective found. At 41 a
column is all circles or all boxes, and circle on circle rolls off, so it
collapses into a real pile, but only once it's tall enough for the jitter
to tip it: 200 and 300 bodies at 41 still stand in columns, one contact a
body, an island a column, exactly like 40.

So "one unit wider" makes a real pile of 1000 or 10 000 (what `:tax`
runs), not of the few hundred a test would pick for speed. The sleeping
tests use 1000 at 41, and assert the contacts per body, so a scene that
quietly went back to columns fails rather than passing on the easy case.
Count contacts per body (or islands) before trusting a scene: the
retrospective's advice, which this is one more case of.

**In other engines it doesn't fall at all** (2026-09-25,
`//engine/std/physics/compare`, the "columns 1000" case: this pile body
for body, 400 steps in). Box2D v3.1.1 and Rapier 0.36 with rotation
locked keep the 1000 standing in columns: 1.01 contacts a body and 30 and
29 islands, against our 1.29 and 2. Friction holds a circle on a circle at
the jitter's angles (at most 13° off vertical, where a friction of 0.4
holds to 22°), so
the columns are what the physics predicts, and their falling is our
solver's: the split impulse, which pushes bodies apart along tilted
normals where friction doesn't act. Measured: the same step on arrays
with the split impulse's pseudo velocities thrown away
(`VARIANTS=arrays:nosplit`) keeps the columns standing too, 1.00 contacts
a body and 32 islands. A scene meant to be a real pile in
every engine staggers every other row by half a body (`scene.rs`,
`Scene::Pile { stagger: true }`), so each body lands between two.
