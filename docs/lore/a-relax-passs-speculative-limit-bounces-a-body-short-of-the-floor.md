# A relax pass's speculative limit bounces a body short of the floor

Found 2026-09-29 by the bounce families (physics.md, "Bounces";
get-emj.69): a ball of restitution 0.2 falling onto a floor at gravity 40
left it at 0.36 of its impact speed along the normal, where restitution
gives 0.2, though its energy was less than it came in with.

Traced (`hit circle e=0.2 v=2 phase=0.25 g=40`, one contact at a time,
the closing speed taken before the step's gravity): the ball begins its
step 0.032 above the floor at 1.17 a second, within the speculative margin
(0.05), and falls 0.026 that step, so it can't reach the floor. In the
last substep the pushing pass leaves it alone (its gap, 0.0119, allows
closing at 3.56 a second); positions move, the gap is now 0.0058, and the
relax pass, whose speculative bias is from that separation, lets it close
at no more than 1.73, the gap over a substep, where it moves at 1.83. The
impulse marks the point pushed, and restitution bounces the ball at e
times its closing speed while it is still 0.0058 above the floor. It keeps
that height as well as the bounce: g times 0.0058 is 12% of the energy it
came in with, three times the 4% restitution gives back at e = 0.2.

**Measured**: over the 2D long drops the worst such bounce returns 9% of
its impact energy past e² (closing speed before the step's gravity; 16%
with it grown over the gap), none more than it came in with; at 30 Hz and
gravity 80 a ball keeps up to 1.08 of e² of its height a bounce. Box2D's
speculative distance is 0.02, so at 0.032 its contact isn't there yet and
the ball lands the step after.

**Resolution**: none yet (get-emj.69). Bouncing only what a pushing pass
pushed took it away, and left 169 of 1600 angled bounces flat: a ball a
speculative contact stops at the surface is bounced by that contact
(pong's, get-emj.19), and a relax pass is where some of them stop.
