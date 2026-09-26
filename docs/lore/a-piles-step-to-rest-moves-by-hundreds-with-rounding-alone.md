# A pile's step to rest moves by hundreds with rounding alone

Measured 2026-09-26 with the comparison's `SETTLE=1500` on turning bodies
(`TURN=1`, the arrays), the same solver before and after one change that
computes the same values in another order (each contact point's cross
products with the normal and tangent kept from the start of the step,
rather than recomputed in each pass):

| scene, variant | at rest from, before | after |
|---|---|---|
| pile 1000, as built | 290 | 180 |
| pile 1000, warm-started by position | 280 | 410 |
| pile 10 000, as built | 390 | 550 |
| pile 10 000, an angle instead of a rotation | 300 | 330 |
| pyramid 5050, every variant | 440 | 440 |

A pile is chaotic: which body tips which way depends on the last bits of
a few products, and once one falls differently the pile is another pile.
The step it comes to rest (every body and edge slower than 0.05) is a
property of the pile that happened, not of the solver, to within a
hundred steps or more; a pyramid, which stands, isn't chaotic and doesn't
move.

So a settling comparison of two solver variants on one pile says nothing
about differences smaller than that. What does clear it: never resting
(no warm start, one point a contact), a pyramid that doesn't rest (one
relax pass), a factor of three (turning ignored in the separation). Read
several scenes, and a pyramid, before choosing by rest steps (physics.md,
"Contact points" and "Rotation in the soft step").
