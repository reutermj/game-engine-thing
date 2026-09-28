# Warm-starting substeps from their average makes a solve depend on its order

Measured 2026-09-28 on the 2D solver (get-emj.48; physics.md, "Why
colors let the pyramid fall"). A soft step solves each step in
substeps, and a contact's accumulated impulse runs on from substep to
substep. Ours then warm-started the next step from the step's impulse,
the substeps' sum, a substep's share of it: their average. That looks
equivalent to starting from the last substep, and in a still scene it
is. It isn't while a load changes: a turning 5050 pyramid breathes
slowly, and through half a swing each step's substeps climbed the same
way, ending up to 1.25 times where they started, while the next step
began from the average, 1.15. So 8% of the load was dropped at every
step, a lag of two substeps.

In pair order it barely showed (the pyramid rested from 440 before, 450
after), since one pass from the ground up carries support to the top
and each step nearly converges on its own. Every other order leans on the warm
start, and with the lag the pyramid never rested: graph-colored as Box2D
colors, shuffled, reversed. It looked like a flaw of graph coloring, and
three rules for coloring static contacts, more substeps and Box2D's
softness were tried against it first.

Box2D stores the accumulator as the last substep left it
(`b2StoreImpulsesTask`) and starts from it (`b2PrepareContactsTask`),
keeping the step's sum apart for reporting (`totalNormalImpulse`). Doing
the same for turning points (the last substep's impulse times the
substeps, so the share is that impulse) stood the colored pyramid and
changed pair order's results within the noise, most for the better.

The way to see it: print each substep's impulse summed over the scene
against the step's warm start. A steady climb within steps, reset at
each step boundary, is the lag. And to find an order dependence at all,
solve in orders the scene can't prefer (`arrays:rot/order`): shuffled
and reversed, not just the one being blamed.
