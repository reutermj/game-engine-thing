# Credits

Libraries this repository builds against, compares with or draws on, and
the ideas our code takes from them. One section a library, so entries
merge cleanly. Authors and licenses were read from each project's fetched
source, not from memory.

None of them is part of the engine or of any game: each is linked only
into a comparison bench (`//engine/std/physics2d/compare` for 2D,
`//engine/std/physics3d/compare` for 3D), so our own solvers can be measured against
established ones on identical scenes. Flecs, EnTT, Bevy, Unity Physics
and Timely Dataflow, last, were read for the ECS's design, and aren't
built at all.

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
  `MODULE.bazel` and built by `engine/std/physics2d/compare/box2d.BUILD.bazel`.
- **What for:** the reference C engine in `//engine/std/physics2d/compare`,
  and nothing else; see docs/architecture/physics.md, "Against other
  engines".
- **Scenes taken from its samples** (`samples/` in the same archive, read,
  not built; physics.md, "Quality beyond settling"): the behaviour
  scenes' card house (`sample_stacking.cpp`, "Card House", which credits
  PEEL, below), dominoes ("Double Domino", with its knock as a starting
  speed and spin), overlap recovery (`sample_robustness.cpp`, "Overlap
  Recovery"), the wide box on two small ones ("HighMassRatio2") and the
  idea of a heavy box on light ones ("HighMassRatio1"), each rebuilt in
  `engine/std/physics2d/compare/scene.rs` with y turned down (the card house
  also five times larger); its arch ("Arch") needs polygons, so a ladder
  stands in.
- **Notice:** MIT asks that the copyright and permission notice go with
  copies of the software. We don't commit Box2D's source or ship a
  binary; the fetched archive keeps its `LICENSE`, which the build exports
  (`@box2d//:LICENSE`) and puts in the comparison binary's runfiles, so it
  goes wherever the binary does.
- **Ideas our physics takes from it** (each named where the code is):
  - sequential impulses with accumulated, clamped impulses and warm
    starting, the solver Box2D is built on, which Erin Catto presented at
    GDC 2006 and in the talks listed at <https://box2d.org/publications/>
    (`engine/std/physics2d/solver.rs`);
  - speculative contacts, pairs found and solved a margin before they
    touch (`narrow.rs`, the margin `physics_common::MARGIN`; Box2D's
    `B2_SPECULATIVE_DISTANCE`);
  - the soft step, since 2026-09-26 the 2D solver (`solver.rs`): the step
    in substeps, each gravity, warm starting, one pass of soft contacts
    (`b2MakeSoft`'s constants, `physics_common::Softness` since
    2026-10-02, shared by both mods; a push-out speed capped as
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
    bounces, at Box2D's default of 1 (`physics_common`'s `BOUNCE_THRESHOLD`;
    Box2D's `b2WorldDef::restitutionThreshold`);
  - restitution's closing speed taken before the step's gravity, as
    `b2PrepareContactsTask` stores `relativeVelocity` before
    `b2_stageIntegrateVelocities`: one of the options weighed for
    get-emj.56, a variant in 2D and 3D (`physics_common::Closing::Before`;
    physics.md, "Bounces");
  - graph coloring for a parallel solve, with contacts on a static body
    kept out of color 0, and SIMD batches of a color's contacts
    (`engine/std/physics2d/tests/parallel_solver.rs`, measured only; Box2D
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
  - a turning contact's points warm-started from where their last
    substep left their accumulated impulses, not from the step's sum
    (2026-09-28, `solver.rs`, `ContactPoint`): Box2D stores the
    accumulator (`b2StoreImpulsesTask`) and starts the next step from it
    (`b2PrepareContactsTask`), keeping the sum apart for reporting.
    Read in v3.1.1's `contact_solver.c`;
  - the turning 2D solve in lanes (2026-09-27, `solver.rs`, `lanes`):
    contacts in batches of four laid out field by field, a one-point
    contact's second point zeros and a batch's empty lanes at a body
    nothing moves (`b2ContactConstraintSIMD`, `b2PrepareContactsTask`),
    bodies copied into one array of 32-byte states read and written by
    gathering and scattering the batch's lanes (`b2BodyState`,
    `b2GatherBodies`, `b2ScatterBodies`), inverse masses kept by each
    contact, and restitution skipping a batch with nothing to bounce
    (`b2ApplyRestitutionTask`). Read in v3.1.1's `contact_solver.c`. The
    lane array, Box2D's `b2FloatW` and its operations (`b2AddW`,
    `b2MaxW` and the rest) written as plain arrays for LLVM to vectorize,
    is `physics_common::lanes::F` since get-emj.86, for 3D's lanes to
    share. The grouping was first the levels of the sweep in pair order,
    which keep the sweep's result bit for bit: level scheduling, from the
    numerical literature on sparse triangular solves (E. Anderson and
    Y. Saad, "Solving sparse triangular linear systems on parallel
    computers", 1989), not from any engine here (physics.md, "The
    solver's speed"), a variant now (`Wide::Levels`); the default is
    Box2D's graph coloring since get-emj.61 (below).
  - the 3D step (`//engine/std/physics3d`, experimental) descends from the
    2D one, and so inherits the same ideas: the soft step, speculative
    contacts, and mixing friction and restitution per contact
    (`engine/std/physics3d/solver.rs`, `lib.rs`). What it takes for
    rotation is Box3D's (below).
  - the broadphase that keeps its pairs (2026-09-27,
    `engine/ecs/live.rs`): each shape's fat box, its box grown by a
    margin (`B2_AABB_MARGIN`, 0.05 m, the margin we measured best too),
    kept until the box leaves it, and pairs whose fat boxes meet kept
    until they don't, looked for again only for shapes that left theirs
    (`b2UpdateShapeAABBs`, `b2FinalizeBodies`, the move buffer of
    `broad_phase.c`, and `b2Collide`'s fat-box test on each contact).
    Read in v3.1.1's source. Ours also lets a fat box go when it is more
    than twice the margin past its box, and finds new pairs among the
    spatial pages rather than a tree (spatial-storage.md, "Keeping pairs").
  - the colored solve across threads (2026-09-29, `solver.rs`,
    `lanes::run_across`; physics.md, "Solving across threads"): Box2D's
    solver stages, read in v3.1.1's `solver.c` (`b2SolverStage`,
    `b2ExecuteStage`, `b2SolverTask`): its stages in its order, a stage a
    color, a stage's blocks four batches or four a worker
    (`blocksPerWorker`), each worker starting at its share
    (`GetWorkerStartIndex`) and taking blocks forward, then back, until one
    is taken, a stage done when its completion count is its blocks, and
    waiting workers spinning and yielding now and then; over Box2D's graph
    colors (`b2AddContactToGraph`), the turning default's grouping since
    get-emj.61. Ours has no main thread and takes a block by raising a mark
    rather than by compare and swap of its sync index. The comparison runs
    Box2D's own multithreaded step (`THREADS` in its `VARIANTS`) on a task
    system of ours for its `enqueueTask` and `finishTask`
    (`compare/box2d_shim.c`).
- **The staged solve, generic, in a spike** (docs/architecture/flows-spike.md,
  2026-10-02): the same staged run, lifted out of the solver into
  `flows::Colored::passes` (`engine/ecs/tests/flows.rs`, a spike since removed, at commit `c72e8b2`), a stage a color
  of any items and the blocks, starts and marks as above; and Box2D's
  coloring rule (`b2AddContactToGraph`) as `flows::Coloring::greedy`.
- **Adopted in `engine_ecs`** (docs/architecture/flows.md, 2026-10-02,
  get-znt.32): the coloring rule as `shape::Coloring::greedy`, and the
  staged run's program as `shape::Passes`, run on one thread until the
  scheduler runs its stages' blocks across threads (get-znt.34). Ideas
  only; no code is copied. Since get-emj.86 the arrays' solve colors by
  the same `Coloring::greedy` and packs by `Coloring::pack`
  (physics2d's `lanes::group`), so the mod and the arrays share one
  coloring.
- **A kept index, in a spike** (docs/architecture/working-sets.md,
  2026-10-02): the awake set's `localIndex`, appended at creation and
  swap-removed with the moved body's index fixed up (`b2DestroyBody`,
  `body.c`), is the index the working-set spike keeps (`Kept` in
  `engine/std/physics2d/compare/working_set_spike.rs`, a spike since removed, at commit `c72e8b2`). Not adopted.
- **Compared for contiguous columns** (docs/architecture/contiguous-columns.md,
  2026-10-02): a solver set's columns as one array each, grown 1.5 times
  by a new block and a copy (`b2GrowAlloc`, `core.c`), indexed directly by
  the solver (`stepContext->states`, `solver.c`). Not adopted; the
  in-place spike (`engine/std/physics2d/compare/contiguous_spike.rs`, a spike since removed, at commit `c72e8b2`)
  measures that shape in the world's storage.

## Rapier

- **Project:** Rapier, 2D and 3D physics engines in Rust (`rapier2d`
  here). <https://rapier.rs>, <https://github.com/dimforge/rapier>
- **Author:** Sébastien Crozet, of Dimforge (the crate's `authors`, and the
  `LICENSE`: "Copyright 2020 Sébastien Crozet").
- **License:** Apache-2.0.
- **Version we use:** `rapier2d` 0.36.0 from crates.io, pinned in
  `engine/std/physics2d/compare/Cargo.toml` and `Cargo.lock`. Its
  dependencies (parry2d, nalgebra, simba, glamx and more, by Dimforge and
  others) come the same way, each under its own license, listed with its
  version in `Cargo.lock`.
- **What for:** the reference Rust engine in
  `//engine/std/physics2d/compare`, and nothing else.
- **Notice:** Apache-2.0 (section 4) asks that redistributions carry a copy
  of the license and keep the notices. The crate as published has no
  `LICENSE` file, so a copy from the repository at the tag we use
  (`v0.36.0`) is kept at `engine/std/physics2d/compare/licenses/rapier-LICENSE`
  and put in the comparison binary's runfiles. Rapier has no `NOTICE`
  file. We modify none of it.
- **Ideas our physics takes from it:** friction solved only in the
  relaxing passes of the soft step (in 3D too, where Box3D does the
  same), not in the pass that pushes contacts apart (`solver.rs`;
  Rapier's `IntegrationParameters::friction_in_bias_pass`,
  off by default, whose doc explains that friction reacting to the push
  pumps stacks until they topple). Read in the fetched 0.36.0 source.
  What else it does differently is in physics.md, "Against other engines"
  and "Settling".
- **The broadphase that keeps its pairs** (2026-09-27,
  `engine/ecs/live.rs`) takes from 0.36.0's `broad_phase_bvh` that a pair
  can only change if one of its ends changed, so only pairs beside
  changed colliders are looked at (`pair_adjacency`): ours finds a
  changed row's pairs by searching its kept pairs from either end, when
  few rows changed.
- **Measured against, not taken** (physics.md, "Rotation"): parry2d
  0.31's other ways with contact points, read in its fetched source:
  matching last step's points to this step's by position
  (`ContactManifold::match_contacts_using_positions`), a variant of the
  comparison (`arrays:rot/warm=2`), and GJK and EPA then clipping the
  polygonal features the normal picks (`contact_manifold_pfm_pfm`, which
  parry uses for convex shapes without a dedicated routine; for boxes it
  uses SAT, `contact_manifold_cuboid_cuboid`), in
  `//engine/std/physics2d:narrow_bench`.
- **Read, for the solve across threads** (2026-09-29, physics.md,
  "Solving across threads"): 0.36.0's staged island solver
  (`dynamics/solver/staged_island_solver/`): stages that advance on work
  completed rather than on threads arrived, so a thread that comes late
  fast-forwards (`sync.rs`, `StageSync`), which ours does too
  (`solver.rs`, `lanes::run_across`); and its bodies shared between
  workers by raw pointers (`SharedCtx`, `unsafe impl Sync`), which ours
  aren't: relaxed atomics, measured against a raw-pointer spike like it.
- **Rapier 3D:** `rapier3d` 0.36.0, the same authors and license, pinned in
  `engine/std/physics3d/compare/Cargo.toml`, the comparison engine in `//engine/std/physics3d/compare`
  (single-threaded, rotations locked). Its license text is fetched pinned by
  sha256 from the v0.36.0 tag (`@rapier_license`, MODULE.bazel) and put in
  that bench's runfiles. It brings parry3d, nalgebra, simba and approx
  (Apache-2.0, Dimforge), and glam, glamx, wide, arrayvec and others under
  MIT, Apache-2.0, Zlib or a choice of them, per each crate's `license`.
- **Measured, not adopted, in 2D** (physics.md, "Still at rest"): its
  block solver, a contact's two normal constraints solved together as a
  2x2 LCP (`solve_mlcp_two_constraints` and `solve_pair`, the
  `block-solver` feature, on by default in 2D), which Box2D v2.4 had too;
  kept as the variant `arrays:rot/block=1` (`solver.rs`, `block`).
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
- **Compared, not adopted** (docs/architecture/working-sets.md,
  2026-10-02, read in the fetched 0.36.0): a body's `active_set_id` kept by
  swap-remove, with an epoch bumped on renumbering so caches of indices
  can tell they're stale (`island_manager/manager.rs`), beside a dense
  copy of the awake bodies rebuilt every step (`SolverBodies`).
- **Compared for contiguous columns** (docs/architecture/contiguous-columns.md,
  2026-10-02): `Arena<T>`, a `Vec` of entries with a free list
  (`data/arena.rs`), and the per-worker slices of the solver's copy
  (`staged_island_solver/sync.rs`). Not adopted.

## Jolt Physics

- **Project:** Jolt Physics, release v5.6.0:
  <https://github.com/jrouwe/JoltPhysics>.
- **Authors:** Jorrit Rouwe ("Copyright 2021 Jorrit Rouwe", LICENSE) and
  the project's contributors.
- **Licence:** MIT (LICENSE at the root of the release archive, exported as
  `@jolt//:LICENSE`).
- **What we use it for:** comparison only. `//engine/std/physics3d/compare` builds it from
  source (`engine/std/physics3d/compare/jolt.BUILD`) behind a small C shim and runs the
  same scenes on `JobSystemSingleThreaded` with translation-only bodies.
- **Ideas our 3D code adopts:** none; it is the yardstick. (Read for the
  kept broadphase, 2026-09-27: Jolt finds its active bodies' pairs afresh
  each step, `BroadPhaseQuadTree::FindCollidingPairs`, as `near_pairs`
  does, and keeps manifolds instead, in its body pair cache.)
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
  `VARIANTS=arrays:ngs` (`engine/std/physics2d/compare/variants.rs`); see
  physics.md, "Settling".
- **Compared, not adopted** (docs/architecture/working-sets.md,
  2026-10-02): its active bodies as a list of ids with each body's place
  kept by swap-remove (`BodyManager::RemoveBodyFromActiveBodies`), and a
  solve in place through `Body` pointers rather than on a copy.
- **Compared for contiguous columns** (docs/architecture/contiguous-columns.md,
  2026-10-02): bodies allocated one by one, held in an array of pointers
  reserved to a fixed maximum (`BodyManager.cpp`), so a `Body*` never
  moves. Not adopted.

## Box3D

- **Project:** Box3D, release v0.1.0: <https://github.com/erincatto/box3d>.
- **Authors:** Erin Catto ("Copyright (c) 2026 Erin Catto", LICENSE).
- **Licence:** MIT (LICENSE at the root of the release archive, exported as
  `@box3d//:LICENSE`).
- **What we use it for:** comparison only. `//engine/std/physics3d/compare` builds it from
  source (`engine/std/physics3d/compare/box3d.BUILD`) behind a small C shim and runs the
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
    re-bounded by storage as it turns (`components.rs`,
    `Collider::turned_half`, the spatial key's `bounds`);
  - fat boxes kept between steps, as Box2D's (above), with the margin
    Box3D caps shapes at (`B3_MAX_AABB_MARGIN`, 0.05; its margin is an
    eighth of a shape's size below that, ours one for all)
    (`engine/ecs/live.rs`, `physics_common::FAT`, both mods').
  - contact recycling (2026-09-27, physics.md, "Still at rest"): a pair's
    manifold kept while its bodies barely move, each point's anchors
    carried with both bodies and its separation updated from how far they
    came apart along the held normal, as within substeps, until a bound on
    the pair's move passes a distance (`b3CollideTask` in
    `physics_world.c`, `B3_CONTACT_RECYCLE_DISTANCE`) (`lib.rs`,
    `recycle`). Ours sums the bound over the steps since the manifold was
    found, rather than keeping the poses it was found at; its distance is
    0.03 to Box3D's 0.05, and it recycles box pairs only;
  - static contacts no stiffer than a quarter of the substep rate, the
    cap Box3D's `b3MakeSoft( 2.0f * contactHertz, ...)` with `contactHertz`
    at most an eighth of it gives (`physics_world.c`) (physics3d's
    `STATIC_STIFFNESS`).
  Our stiffness between moving bodies (a fifth of the substep rate, where
  Box3D has 30 Hz at 4 substeps) and our two relaxing passes are our own
  measurements (physics.md, "Rotation in 3D").
- **Measured, not adopted** (physics.md, "Still at rest"): its reduction
  to four points as it is, the first point the one farthest along a fixed
  tangent and each choice by a pecking order (a candidate must beat the
  best by 5%, `b3ReduceManifoldPoints`); and warm starting from the last
  substep's impulses (`mp->normalImpulse = cp->normalImpulse`), which
  Box2D and Rapier do too (physics3d's `Carry::Last`).

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
  measured it (`engine/std/physics2d/tests/split_impulse.rs`; Bullet's
  `btContactSolverInfo::m_splitImpulse` and its push velocities in
  `btSequentialImpulseConstraintSolver`).

## PEEL

- **Project:** PEEL, the Physics Engine Evaluation Lab, a set of scenes
  for comparing physics engines, by Pierre Terdiman.
- **License:** not read: nothing of it is fetched or built.
- **What for:** the card house among the behaviour scenes, which reaches
  us through Box2D's sample of it (`sample_stacking.cpp`, "Card House",
  marked "From PEEL"), rebuilt from that sample (physics.md, "Quality
  beyond settling").

## Flecs

- **Project:** Flecs, an entity component system for C and C++.
  <https://www.flecs.dev>, <https://github.com/SanderMertens/flecs>
- **Author:** Sander Mertens (`LICENSE`: "Copyright (c) 2025 Sander
  Mertens", with portions copyright Meta Platforms).
- **License:** MIT.
- **Version read:** v4.1.6, its release tarball (sha256
  `29ccf56961b7ffbd38cce2227a06c0722c7df464422e86619a65ee37bb31bae7`)
  fetched into a scratch directory to read, 2026-09-26. Not fetched by the
  build, not linked, and none of its code is copied.
- **What for:** design reference for the ECS (`engine/ecs`).
- **Ideas the ECS takes from it:**
  - storage chosen per component, archetype tables or a sparse set
    (Flecs's `Sparse` trait): docs/architecture/storage.md;
  - a deferred change that costs no allocation of its own
    (get-znt.18, spike): Flecs queues commands by value in one vector per
    stage and bump-allocates their values from a stack reset at each merge
    (`flecs_cmd_new` in `src/commands.c`, `flecs_stack_alloc` in
    `src/datastructures/stack_allocator.c`). A system's sparse inserts and
    removes are logged as runs of one component's changes, values in a
    typed `Vec` (`SparseRun` in `engine/ecs/query.rs`);
  - no per-entity merging for changes that move no row: Flecs skips its
    per-entity batching for non-fragmenting components (`src/commands.c`,
    "Nothing to batch for non-fragmenting components"), and a run's exact
    footprint is its set, with no per-change table replay (`exact` in
    `engine/ecs/graph.rs`).
- **Compared for working sets** (docs/architecture/working-sets.md,
  2026-10-02, read in v4.0.4, fetched into a Bazel output base, not
  linked): optional terms decided once per table (`set_fields` in the query
  cache, `src/query/engine/cache.c`), and `cascade`, tables grouped by
  their depth in a hierarchy (`flecs_query_cache_group_by_cascade`). The
  doc proposes a depth order for `ChildOf` after it; nothing is built.
- **Compared for flows** (docs/architecture/flows-spike.md, 2026-10-02,
  read in v4.0.4 as above): pipelines ordered by phase with merges
  inferred from the terms systems read and write
  (`src/addons/pipeline/pipeline.c`, `flecs_pipeline_check_term`,
  `flecs_pipeline_build`), and `ecs_run`'s call-scoped `param`
  (`include/flecs/addons/system.h`). Flecs has no value passed between
  systems; nothing is taken.
- **Compared for contiguous columns** (docs/architecture/contiguous-columns.md,
  2026-10-02, read in v4.0.4 as above): a table column as one array
  (`ecs_column_t`, `src/storage/table.h`), grown by a new block and a copy
  (`flecs_table_grow_data`), paged only where it promises stable pointers
  (`src/datastructures/sparse.c`, the entity index), and a table's rows
  split between workers as contiguous ranges (`ecs_worker_next`,
  `src/iter.c`). The doc's option (b) is that column shape; not built.

## Bevy

- **Project:** Bevy, a game engine built on its own ECS (`bevy_ecs`).
  <https://bevy.org>, <https://github.com/bevyengine/bevy>
- **Authors and license:** dual-licensed, MIT or Apache-2.0, "except where
  noted", as its `README.md` and `LICENSE-MIT` state (read 2026-10-02).
- **Version read:** `main` at commit `90942be` (0.20.0-dev), single files
  fetched from GitHub to read, 2026-10-02: `crates/bevy_ecs/src/system/`
  (`combinator.rs`, `input.rs`, `system_param.rs`), `message/messages.rs`,
  `query/par_iter.rs`, `schedule/executor/multi_threaded.rs`. Not fetched by
  the build, not linked, and none of its code is copied.
- **Compared for flows** (docs/architecture/flows-spike.md): system
  piping (`PipeSystem`, a value moved from one system's output to the
  next's `In<T>`, the two scheduled as one node with their accesses
  joined), `Local<T>` (a system's own state, its allocations kept between
  runs) and messages (double-buffered, read by any number of systems).
  The spike's flows take the typed hand-off from `pipe` and the kept
  allocations from `Local`, and differ in making each use a declared edge
  of its own. Nothing is taken as code. The same ideas are in
  `engine/ecs/flows.rs` (docs/architecture/flows.md).

## Unity Physics

- **Project:** Unity Physics, the DOTS physics package (`com.unity.physics`).
- **License:** the Unity Companion License, as its `LICENSE.md` states
  ("Unity Physics copyright © 2024 Unity Technologies ApS"), read
  2026-10-02.
- **Version read:** 1.5.0, from the `needle-mirror/com.unity.physics`
  GitHub mirror (assumed faithful to the package), single files fetched to
  read: `BuildPhysicsWorld.cs`, `PhysicsWorldData.cs`,
  `UnityPhysicsSimulationSystems.cs`, `Simulation.cs`, `Scheduler.cs`.
  Not linked, and none of its code is copied.
- **Compared for flows** (docs/architecture/flows-spike.md): its stages
  are systems handing one `Simulation` on through a singleton and job
  handles (`state.Dependency`), its buffers kept between steps and grown
  only, and its solve in phases of a body each (`DispatchPairSequencer`).
  The Jobs and Entities documentation it builds on was read online, not
  as source.

## Timely Dataflow

- **Project:** Timely Dataflow, a low-latency dataflow system in Rust.
  <https://github.com/TimelyDataflow/timely-dataflow>
- **Author and license:** Frank McSherry, MIT ("Copyright (c) 2014 Frank
  McSherry", its `LICENSE`), read 2026-10-02.
- **Version read:** `master` at `9efd010`, single files fetched to read
  (`communication/src/lib.rs`, `timely/src/dataflow/channels/pushers/tee.rs`).
  Not linked, and none of its code is copied.
- **Compared for flows** (docs/architecture/flows-spike.md): streams
  wired explicitly between operators, messages moved and their buffers
  handed back by swapping (`Push::push(&mut Option<T>)`), and a stream
  with several readers cloned for all but the last. The spike's recycling
  of a flow's allocations is the same idea at a frame's scale, and so is
  `engine/ecs/flows.rs`'s (docs/architecture/flows.md).

## EnTT

- **Project:** EnTT, an entity component system for C++.
  <https://github.com/skypjack/entt>
- **Author:** Michele Caini (`LICENSE`: "Copyright (c) 2017-2026 Michele
  Caini, author of EnTT").
- **License:** MIT.
- **Version read:** v4.0.0, its release tarball (sha256
  `32a2ff2c72cb047dfd57306006ef238820b70da7c6ce4e7e8a507ac63365212e`)
  fetched into a scratch directory to read, 2026-09-26. Not fetched by the
  build, not linked, and none of its code is copied.
- **What for:** design reference for the ECS's sparse sets.
- **Compared, not taken** (get-znt.18, spike): EnTT's sparse set, a paged
  index by entity into packed entities and values, swap-and-pop removal
  (`basic_sparse_set`, `src/entt/entity/sparse_set.hpp`), is the shape
  `SparseSet` already has (an unpaged index). EnTT removes a destroyed
  entity from every pool at once (`basic_registry::destroy`,
  `src/entt/entity/registry.hpp`) and defers nothing; our sets keep a dead
  entity's entry, invisible, and purge once the dead could be a quarter of
  the set (`SparseSet::purge_dead`), since a despawn doesn't hold every
  set's guard.

## hash-prospector

- **Project:** hash-prospector, a search for integer hash functions,
  by Chris Wellons (skeeto), https://github.com/skeeto/hash-prospector.
- **License:** the Unlicense (public domain), as its `UNLICENSE` states
  (read 2026-09-28).
- **What we use:** the constants and shifts of its `lowbias32`, as given
  in its README, for `hashed` in `engine/std/physics2d/compare/scene.rs`,
  which chooses each body of a mixed pile (`Scene::Mixed`) the same way
  in every engine. Nothing is linked.

## FNV

- **Project:** the Fowler/Noll/Vo hash, by Glenn Fowler, Phong Vo and
  Landon Curt Noll, http://www.isthe.com/chongo/tech/comp/fnv/.
- **License:** public domain, "via the Creative Commons CC0 1.0 Universal
  (CC0 1.0) Public Domain Dedication", as its page states (read
  2026-10-02).
- **What we use:** FNV-1a's 64-bit offset basis and prime, for the hashes
  of physics3d's exact fingerprint (`engine/std/physics3d/tests/exact.rs`),
  and in the ECS's component fingerprints (`engine/ecs/component.rs`).
  Nothing is linked.

## SplitMix64

- **Project:** SplitMix64, Guy Steele, Doug Lea and Christine Flood's
  generator (Java's `SplittableRandom`), in Sebastiano Vigna's C,
  https://prng.di.unimi.it/splitmix64.c.
- **License:** public domain: "the author has dedicated all copyright and
  related and neighboring rights to this software to the public domain
  worldwide", as the file states (read 2026-10-02).
- **What we use:** its increment and mixing constants, for the seeded
  scenes of the 3D comparison (`engine/std/physics3d/tests/scenes.rs`)
  and the kernel's inputs of physics3d's exact fingerprint
  (`engine/std/physics3d/tests/exact.rs`). Nothing is linked.
