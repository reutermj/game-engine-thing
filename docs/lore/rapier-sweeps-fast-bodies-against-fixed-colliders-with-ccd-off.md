# Rapier sweeps fast bodies against fixed colliders with CCD off

Found 2026-09-28 running the behaviour scenes (physics.md, "Quality beyond
settling"): a ball of radius 0.25 fired at a static wall 0.1 thick, at 10
to 400 units a second and four phases each.

Rapier 0.36 "as shipped", no `ccd_enabled` on any body, never tunnelled,
at any speed or phase; with `ccd_enabled(true)` every number was the same
to four digits. The builder's name suggests CCD is off by default. It is
only the part that sweeps against moving bodies:

```rust
// dynamics/ccd/ccd_solver.rs, rapier2d 0.36.0
/// after the solver, bodies that moved more than half their thinnest extent sweep their colliders
/// Fast dynamic bodies sweep against fixed colliders and soft-body meshes; `ccd_enabled` makes a
/// bullet that also sweeps kinematic/dynamic bodies (never other bullets)
```

(`ccd_enabled` says the same on `RigidBodyBuilder`; `max_ccd_substeps = 0`
turns all of it off.)

**Measured**: Rapier, with and without `ccd_enabled`, 0 of 48 tunnelled
(12 speeds, 4 phases) through the thin wall and 0 of 48 through a unit
wall, like Box2D (continuous on, its default); ours tunnelled from 25 a
second.

**Resolution**: in a comparison, Rapier "without CCD" already has CCD
against statics; to see Rapier tunnel, set
`IntegrationParameters::max_ccd_substeps` to 0. The comparison keeps it as
shipped (`VARIANTS=rapier:ccd` adds the bullet flag).
