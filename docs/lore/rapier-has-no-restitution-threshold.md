# Rapier has no restitution threshold

Found 2026-09-29 building the bounce families (physics.md, "Bounces"):
dropped onto a floor at 0.8 a second, under the 1 a second that Box2D,
Box3D and ours bounce from, a ball of restitution 0.5 bounced back at 0.4
in Rapier 0.36, in 2D and 3D, where the others stopped.

Rapier gates restitution by whether a contact is new, not by speed:

```rust
// rapier 0.36, src/geometry/contact_pair.rs
pub fn is_bouncy(restitution: Real, is_new: bool) -> Real {
    if is_new {
        (restitution > 0.0) as u32 as Real
    } else {
        (restitution >= 1.0) as u32 as Real
    }
}
```

(`is_new` is a point whose warm-start impulse is 0,
`generic_contact_constraint.rs`.) Box2D skips a point whose
`relativeVelocity` is over −`restitutionThreshold` (`b2ApplyRestitution`,
1 by default), and Box3D the same.

**Measured**: of the 2D long grid's drops under the threshold, Rapier
bounces 535 (Box2D 0, ours with the step's gravity out 0), and in 3D 66
of the long grid's (Box3D 4). At e = 1 an old contact bounces too: a
lossless ball left on the floor keeps hopping, and over 20 s rises to 1.2
of its drop on the short series grid.

**Resolution**: a threshold is a design choice, not a law: the bounce
families bound ours by it (nothing under it bounces), and Rapier's count
is recorded beside the bound, not used for it.
