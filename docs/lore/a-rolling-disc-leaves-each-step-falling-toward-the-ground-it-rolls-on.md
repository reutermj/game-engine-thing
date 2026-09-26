# A rolling disc leaves each step falling toward the ground it rolls on

Measured 2026-09-26 in `solver.rs`'s
`a_spinning_disc_slows_by_friction_until_it_rolls` (a disc of radius 0.5
rolling at 3.3 radians a second, gravity 20, 5 substeps of 1/60 s): at the
end of every step its velocity is 0.083 down, a quarter of a step's
gravity, while it moves down by less than 1e-4.

The contact point's arm is a material point of the disc, fixed when the
contact is found and turned with the disc through the step, as Box2D
(`b2SolveContact`) and Rapier (`update`, the local points transformed by
the bodies' poses) both do. On a box that is right: the corner that was in
contact is still the corner in contact. On a rolling disc the material
point rolls up and away from the ground, `r (1 - cos θ)` after turning θ,
so within the step the contact reads as a gap opening, and the
speculative rule (a gap may close this substep, no faster) lets the disc
keep closing it: the last relax pass leaves the downward speed the gap
allows. The next step finds the contact again at the new lowest point,
touching, and the disc hasn't sunk.

So nothing sinks, but a rolling body's `Velocity` isn't what it looks
like at rest: its vertical speed isn't 0, which a game reading it (or a
threshold on it) would see. Following the arm to first order instead
(`r + θ × r`, `arrays:rot/sep=1` in the comparison) keeps the arm's height
and avoids it, and settled piles more slowly (physics.md, "Rotation").
