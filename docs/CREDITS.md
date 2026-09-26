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
  - the 3D step (`//engine/std/physics3d`, experimental) descends from the
    2D one, and so inherits the same ideas: sequential impulses with warm
    starting, speculative contacts, and mixing friction and restitution
    per contact (`engine/std/physics3d/solver.rs`, `lib.rs`).

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
  relaxing passes of the soft step, not in the pass that pushes contacts
  apart (`solver.rs`; Rapier's `IntegrationParameters::friction_in_bias_pass`,
  off by default, whose doc explains that friction reacting to the push
  pumps stacks until they topple). Read in the fetched 0.36.0 source.
  What else it does differently is in physics.md, "Against other engines"
  and "Settling".
- **Rapier 3D:** `rapier3d` 0.36.0, the same authors and license, pinned in
  `bench/physics3d/Cargo.toml`, the comparison engine in `//bench/physics3d`
  (single-threaded, rotations locked). Its license text is fetched pinned by
  sha256 from the v0.36.0 tag (`@rapier_license`, MODULE.bazel) and put in
  that bench's runfiles. It brings parry3d, nalgebra, simba and approx
  (Apache-2.0, Dimforge), and glam, glamx, wide, arrayvec and others under
  MIT, Apache-2.0, Zlib or a choice of them, per each crate's `license`.

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
- **Ideas our 3D code adopts:** none taken from Box3D's source; the step in
  `//engine/std/physics3d` descends from our 2D one, whose ideas are
  Box2D's (below).

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
