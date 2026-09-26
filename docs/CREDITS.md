# Credits

Libraries this repository builds against, compares with or draws on, and
the ideas our code takes from them. One section a library, so entries
merge cleanly. Licenses were read from each project's fetched source,
not from memory.

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
  - a restitution threshold, the closing speed below which nothing
    bounces, at Box2D's default of 1 (`solver.rs`, `BOUNCE_THRESHOLD`;
    Box2D's `b2WorldDef::restitutionThreshold`);
  - graph coloring for a parallel solve, with contacts on a static body
    kept out of color 0, and SIMD batches of a color's contacts
    (`engine/std/physics/tests/parallel_solver.rs`, measured only; Box2D
    v3's `constraint_graph.c`, which credits "High-Performance Physical
    Simulations on Next-Generation Architecture with Many Cores",
    Intel Technology Journal).

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
- **Ideas our physics takes from it:** none yet. What it does differently
  is in physics.md, "Against other engines".

## Bullet Physics

- **Project:** Bullet, a 3D physics library.
  <https://github.com/bulletphysics/bullet3>
- **Author:** Erwin Coumans and contributors.
- **License:** zlib.
- **What for:** not built or fetched; an idea source only.
- **Ideas our physics takes from it:** the split impulse, correcting
  penetration with a second "push" velocity that moves positions and is
  then thrown away, so correction adds no energy (`solver.rs`; Bullet's
  `btContactSolverInfo::m_splitImpulse` and its push velocities in
  `btSequentialImpulseConstraintSolver`).
