# A swaying stack's energy at one step is where in the swing it was

Measured 2026-09-27 with the 2D comparison (`SETTLE=700`, `TURN=1`,
`SCENES="stack 20"`, Rapier 0.36 and ours). A 20-high stack of turning
unit boxes under gravity 20 is near its buckling load under soft contacts
(physics.md, "Still at rest"), and sways in every engine that stands it
with a period of seconds. Rapier's kinetic energy a body at step 700 is
1.5e-7; the most at any of the looks (every 10 steps) over steps 500-700 is
2.8e-4, three and a half thousand times more. Ours is 1.2e-5 at step 700
and 5.6e-4 over the same looks; a variant of ours (static contacts at 0.4
of the substep rate, warm started from the last substep) read 4.5e-9 at
step 700 on a swing whose envelope was 3e-4.

The quality test bounded the stack by ten times the reference's energy at
the last step, 1.5e-6, which ours "failed" and a variant "passed" by
sampling a different phase. The bound is now on the most over the last
200 steps, from the references' same measure (`Settling::energy_tail`).
Where a scene is truly still (a pyramid, a pile at rest) the two agree to
two digits, so only swaying scenes moved.
