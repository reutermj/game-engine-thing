# Credits

Libraries this repository builds against, compares with or draws on, and
the ideas our code takes from them. One section a library, so entries
merge cleanly. Authors and licenses were read from each project's fetched
source, not from memory.

None of them is part of the engine or of any game: each is linked only
into a comparison bench (`//engine/std/physics/compare` for 2D,
`//bench/physics3d` for 3D), so our own solvers can be measured against
established ones on identical scenes.

Where the notices live: every library's license text is in its fetched
source (Bazel's external repository for it), or committed or fetched
beside the bench where the published package has none, and the bench that
links it declares the license files as `data`, so they sit in the
binary's runfiles beside it. That meets the MIT and Apache-2.0 conditions
for the one form in which the code is copied, a bench binary with its
runfiles. Nothing here is distributed as a binary; a release that shipped
one would have to ship those files with it.

## Box2D

- **Project:** Box2D, a 2D physics engine for games.
  <https://box2d.org>, <https://github.com/erincatto/box2d>
- **Author:** Erin Catto (`LICENSE`: "Copyright (c) 2022 Erin Catto").
- **License:** MIT.
- **Version we use:** v3.1.1 (2025-06-04), fetched by checksum in
  `MODULE.bazel` and built by `engine/std/physics/compare/box2d.BUILD.bazel`.
- **What for:** the reference C engine in `//engine/std/physics/compare`,
  and nothing else; see docs/architecture/physics.md, "Against other
  engines".
- **Notice:** MIT asks that the copyright and permission notice go with
  copies of the software. We don't commit Box2D's source or ship a
  binary; the fetched archive keeps its `LICENSE`, which the build exports
  (`@box2d//:LICENSE`) and puts in the comparison binary's runfiles, so it
  goes wherever the binary does.
- **Ideas our physics takes from it** (each named where the code is):
  - sequential impulses with accumulated, clamped impulses and warm
    starting, the solver Box2D is built on, which Erin Catto presented at
    GDC 2006 and in the talks listed at <https://box2d.org/publications/>
    (`engine/std/physics/solver.rs`);
  - speculative contacts, pairs found and solved a margin before they
    touch (`narrow.rs`; Box2D's `B2_SPECULATIVE_DISTANCE`);
  - the soft step, since 2026-09-26 the 2D solver (`solver.rs`): the step
    in substeps, each gravity, warm starting, one pass of soft contacts
    (`b2MakeSoft`'s constants, a push-out speed capped as
    `maxContactPushSpeed` caps it, contacts with a static body twice as
    stiff), positions integrated and separations updated from how far the
    bodies moved, then rigid relaxing passes; and restitution applied once
    after the substeps, from the closing speed before them, to contacts
    that pushed (`b2ApplyRestitution`). Read in v3.1.1's `solver.c` and
    `contact_solver.c`, and described in Erin Catto's "Solver2D" (2024,
    <https://box2d.org/posts/2024/02/solver2d/>). Our stiffness (a quarter
    of the substep rate, 60 Hz where Box2D has 30) and our two relaxing
    passes (Box2D's one) are our own measurements (physics.md,
    "Settling");
  - a restitution threshold, the closing speed below which nothing
    bounces, at Box2D's default of 1 (`solver.rs`, `BOUNCE_THRESHOLD`;
    Box2D's `b2WorldDef::restitutionThreshold`);
  - graph coloring for a parallel solve, with contacts on a static body
    kept out of color 0, and SIMD batches of a color's contacts
    (`engine/std/physics/tests/parallel_solver.rs`, measured only; Box2D
    v3's `constraint_graph.c`, which credits "High-Performance Physical
    Simulations on Next-Generation Architecture with Many Cores",
    Intel Technology Journal).
  - rotation in the 2D step (2026-09-26, physics.md, "Rotation"), read
    in v3.1.1's `manifold.c`, `contact.c`, `contact_solver.c`, `solver.c`
    and `math_functions.h`:
    - a rotation kept as its cosine and sine (`b2Rot`), turned a substep
      at a time by the first-order step and normalized
      (`b2IntegrateRotation`), and a body's final rotation its step's
      turn after its start (`shapes.rs`, `Rot`; `solver.rs`);
    - turned boxes by the separating axis of least overlap, the other
      box's most opposed edge clipped to the reference face's sides, each
      point halfway between the faces and numbered by the edges it came
      from (`b2CollidePolygons`, `b2ClipPolygons`, `B2_MAKE_ID`), and a
      circle against a turned box, or a circle, at one point halfway
      between the surfaces (`b2CollidePolygonAndCircle`,
      `b2CollideCircles`) (`narrow.rs`);
    - warm starting each point from last step's impulse at the point with
      the same feature id, and a point new this step from nothing
      (`b2UpdateContact`; `components.rs`, `ContactPoints::last`);
    - each point's arms from both bodies' centers, its effective masses
      along the normal and the tangent with the arms' cross products, and
      its separation updated in the substeps by the arms turned with their
      bodies, the normal held fixed (`b2PrepareContactsTask`,
      `b2SolveContact`); restitution per point, from its closing speed
      before the step, for points that pushed (`b2ApplyRestitution`)
      (`solver.rs`);
    - a turning body as fast as its edge for sleeping (`maxExtent` in
      `b2FinalizeBodies`) (`lib.rs`, `fall_asleep`);
    - in the comparison, a turning body's mass set with its shape's inertia
      at mass 1 (`b2Body_SetMassData`, `compare/box2d_shim.c`).
  - the 3D step (`//engine/std/physics3d`, experimental) descends from the
    2D one, and so inherits the same ideas: the soft step, speculative
    contacts, and mixing friction and restitution per contact
    (`engine/std/physics3d/solver.rs`, `lib.rs`). What it takes for
    rotation is Box3D's (below).

## Rapier

- **Project:** Rapier, 2D and 3D physics engines in Rust (`rapier2d`
  here). <https://rapier.rs>, <https://github.com/dimforge/rapier>
- **Author:** Sébastien Crozet, of Dimforge (the crate's `authors`, and the
  `LICENSE`: "Copyright 2020 Sébastien Crozet").
- **License:** Apache-2.0.
- **Version we use:** `rapier2d` 0.36.0 from crates.io, pinned in
  `engine/std/physics/compare/Cargo.toml` and `Cargo.lock`. Its
  dependencies (parry2d, nalgebra, simba, glamx and more, by Dimforge and
  others) come the same way, each under its own license, listed with its
  version in `Cargo.lock`.
- **What for:** the reference Rust engine in
  `//engine/std/physics/compare`, and nothing else.
- **Notice:** Apache-2.0 (section 4) asks that redistributions carry a copy
  of the license and keep the notices. The crate as published has no
  `LICENSE` file, so a copy from the repository at the tag we use
  (`v0.36.0`) is kept at `engine/std/physics/compare/licenses/rapier-LICENSE`
  and put in the comparison binary's runfiles. Rapier has no `NOTICE`
  file. We modify none of it.
- **Ideas our physics takes from it:** friction solved only in the
  relaxing passes of the soft step (in 3D too, where Box3D does the same), not in the pass that pushes contacts
  apart (`solver.rs`; Rapier's `IntegrationParameters::friction_in_bias_pass`,
  off by default, whose doc explains that friction reacting to the push
  pumps stacks until they topple). Read in the fetched 0.36.0 source.
  What else it does differently is in physics.md, "Against other engines"
  and "Settling".
- **Measured against, not taken** (physics.md, "Rotation"): parry2d
  0.31's other ways with contact points, read in its fetched source:
  matching last step's points to this step's by position
  (`ContactManifold::match_contacts_using_positions`), a variant of the
  comparison (`arrays:rot/warm=2`), and GJK and EPA then clipping the
  polygonal features the normal picks (`contact_manifold_pfm_pfm`, which
  parry uses for convex shapes without a dedicated routine; for boxes it
  uses SAT, `contact_manifold_cuboid_cuboid`), in
  `//engine/std/physics:narrow_bench`.
- **Rapier 3D:** `rapier3d` 0.36.0, the same authors and license, pinned in
  `bench/physics3d/Cargo.toml`, the comparison engine in `//bench/physics3d`
  (single-threaded, rotations locked). Its license text is fetched pinned by
  sha256 from the v0.36.0 tag (`@rapier_license`, MODULE.bazel) and put in
  that bench's runfiles. It brings parry3d, nalgebra, simba and approx
  (Apache-2.0, Dimforge), and glam, glamx, wide, arrayvec and others under
  MIT, Apache-2.0, Zlib or a choice of them, per each crate's `license`.
- **Measured, not adopted, in 3D** (read in the fetched rapier3d 0.36.0 and
  parry3d 0.31.1): the full separating axis test every step with no cached
  axis (`contact_manifolds_cuboid_cuboid`, the `narrow::BoxBox::Sat`
  variant); reducing a manifold to its deepest point, the farthest from
  it and the farthest either side of the line through them
  (`reduce_manifold_naive`, `narrow::Reduce::Line`, which Jolt's
  `PruneContactPoints` also does). Rapier's default friction, one Coulomb
  constraint at the contact's friction center with a twist constraint,
  is also Box3D's and Jolt's, and is what the 3D solver does; physics.md,
  "Rotation in 3D", has the measurements.

## Jolt Physics

- **Project:** Jolt Physics, release v5.6.0:
  <https://github.com/jrouwe/JoltPhysics>.
- **Authors:** Jorrit Rouwe ("Copyright 2021 Jorrit Rouwe", LICENSE) and
  the project's contributors.
- **Licence:** MIT (LICENSE at the root of the release archive, exported as
  `@jolt//:LICENSE`).
- **What we use it for:** comparison only. `//bench/physics3d` builds it from
  source (`bench/physics3d/jolt.BUILD`) behind a small C shim and runs the
  same scenes on `JobSystemSingleThreaded` with translation-only bodies.
- **Ideas our 3D code adopts:** none; it is the yardstick.
- **Measured, not adopted, in 3D** (read in the fetched 5.6.0 source):
  box against box by GJK and EPA, then the supporting faces clipped, the
  penetration axis kept as the normal (`ConvexShape::sCollideConvexVsConvex`,
  `ManifoldBetweenTwoFaces`; our own implementation,
  `engine/std/physics3d/gjk.rs`, the `narrow::BoxBox::GjkEpa` variant);
  warm starting by the nearest last point within 1 cm
  (`mContactPointPreserveLambdaMaxDistSq`, `Warm::Nearest`); rotation
  stepped by the exact turn about the angular velocity (`Body::AddRotationStep`,
  `solver::Integrate::Exact`). See physics.md, "Rotation in 3D".
- **Measured, not adopted, in 2D:** its position iterations (non-linear
  Gauss-Seidel: `ContactConstraintManager::sSolvePositionConstraint` and
  `AxisConstraintPart::SolvePositionConstraint`, Baumgarte 0.2, a slop and
  a 0.2 cap on the correction), tried as the comparison's
  `VARIANTS=arrays:ngs` (`engine/std/physics/compare/variants.rs`); see
  physics.md, "Settling".

## Box3D

- **Project:** Box3D, release v0.1.0: <https://github.com/erincatto/box3d>.
- **Authors:** Erin Catto ("Copyright (c) 2026 Erin Catto", LICENSE).
- **Licence:** MIT (LICENSE at the root of the release archive, exported as
  `@box3d//:LICENSE`).
- **What we use it for:** comparison only. `//bench/physics3d` builds it from
  source (`bench/physics3d/box3d.BUILD`) behind a small C shim and runs the
  same scenes with one worker and all three angular motion locks.
- **Ideas our 3D code adopts** (read in the fetched v0.1.0 source, and
  named where our code has them):
  - the soft step with rotation (`solver.c`, `contact_solver.c`): each
    point's anchors, effective mass and each body's world inverse inertia
    fixed once a step, and a point's separation within the step its
    separation when found plus its anchors' moves along the normal, from
    each body's accumulated move and turn (`b3SolveContact`); a rotation
    stepped to first order and normalized every substep
    (`b3IntegrateRotation`), at most a quarter turn a step
    (`B3_MAX_ROTATION`) (`engine/std/physics3d/solver.rs`);
  - friction for a whole contact at its points' centroid, a 2x2 tangent
    mass and an impulse clamped to a disc, and twist friction limited by
    each point's normal impulse times its distance from the centroid,
    both only in the relaxing passes (`solver.rs`);
  - box against box by the separating axis test with the last step's
    axis tried first and kept while its separation changes by under the
    linear slop (`b3CollideHulls` and its `b3SATCache`), the reference
    face biased toward `a`'s, edge pairs only where they make a face of
    the Minkowski difference (`b3IsMinkowskiFace`) and only when they
    separate clearly more than a face, the incident face clipped to the
    reference face, points halfway between the faces, and more than four
    reduced to four by area (`b3ReduceManifoldPoints`)
    (`engine/std/physics3d/narrow.rs`);
  - warm starting a point by the features that made it, and friction by
    the contact (`contact.c`), with ids of our own encoding
    (`narrow.rs`, `lib.rs`);
  - bounds of a turned box, center plus or minus |R| h (`b3AABB_Transform`),
    rewritten as it turns (`lib.rs`, `Reach::of`).
  Our stiffness (a fifth of the substep rate, where Box3D has 30 Hz at 4
  substeps) and our two relaxing passes are our own measurements
  (physics.md, "Rotation in 3D").

## Bullet Physics

- **Project:** Bullet, a 3D physics library.
  <https://github.com/bulletphysics/bullet3>
- **Author:** Erwin Coumans and contributors.
- **License:** zlib.
- **What for:** not built or fetched; an idea source only.
- **Ideas our physics takes from it:** the split impulse, correcting
  penetration with a second "push" velocity that moves positions and is
  then thrown away, so correction adds no energy. It was the 2D solver
  until 2026-09-26, and is kept as it was for the experiments that
  measured it (`engine/std/physics/tests/split_impulse.rs`; Bullet's
  `btContactSolverInfo::m_splitImpulse` and its push velocities in
  `btSequentialImpulseConstraintSolver`).
