# A soft contact sinks by its load, and only substeps make it stiffer

Measured 2026-09-26 with `//engine/std/physics/compare`'s `SETTLE=1500`
(the 10 000 pile, 401 wide, staggered; our soft step on arrays with
`VARIANTS=arrays:soft/...`): the deepest overlap once at rest.

| stiffness (moving / static contacts) | substeps | deepest | at rest by step |
|---|---|---|---|
| 30 / 60 Hz (Box2D's and Rapier's) | 4 | 0.085 | 210-280 |
| 45 / 90 Hz | 3 | 0.037 | 280 |
| 60 / 120 Hz | 4 | 0.024-0.027 | 230 |
| 75 / 150 Hz | 5 | 0.014 | 240 |
| 90 / 180 Hz | 6 | 0.008 | 960 (one relax pass) |
| 90 or 120 Hz | 4 | 0.013-0.08 | never: energy 0.2-0.5 a body |

A soft contact (Box2D v3's `b2MakeSoft`) is a damped spring on the
penetration, solved implicitly. At rest its push-out velocity (rate times
depth) must balance the share of the accumulated impulse it lets go each
pass, which works out to depth = load / (m omega^2): a spring of stiffness
m omega^2, with m the contact's effective mass. The damping ratio and the
substep count cancel out, so what sinks a pile is its weight over the
square of the stiffness, as the table shows (0.085 at 30 Hz is 0.021 at
60 and 0.009 at 90, predicted; 0.024 and 0.008 measured). This is why Box2D
and Rapier piles sit 0.05-0.06 deep and no setting of theirs (substeps,
iterations) changes it: their 30 Hz is fixed.

Stiffness can't simply be raised: at half the substep rate (120 Hz at 4
substeps of 1/60 s) the pile jitters and never rests. A quarter of the
substep rate holds, so the lever is substeps, each costing a pass over
every contact. The solver has 5 substeps at a quarter of their rate (75
Hz), with two relax passes: one relax pass leaves the pile sliding for
hundreds of steps more (870 at 4 substeps, 60 Hz).

Also tried: relaxing only the impulse added since the substep's warm start
(so the carried load isn't soft, and nothing sinks). It removes the sink
and the damping with it: every such pile never rested (energy 0.8-1.2).
