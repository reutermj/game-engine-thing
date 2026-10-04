# Physics

**Status: built.** Two physics mods, both ours, with every piece of state
in the world: `//engine/std/physics2d`, which both games run on, and
`//engine/std/physics3d`, experimental, with `pile3d` as its scene. This
doc describes what they do now, by topic. Why each choice was made, and
what it was measured against, is in the dated record of the work,
[the physics log](../retrospectives/2026-10-04-physics-log.md) (this
doc's text until 2026-10-04); the sections below point into it, and
[Where the old sections went](#where-the-old-sections-went) maps every
old section name, which code comments still cite, to where it is now.

Elsewhere: what the two mods share and how they differ, row by row, is
[physics-sharing.md](physics-sharing.md)'s parity table; the tests, the
baselines and the comparisons with other engines are
[physics-testing.md](physics-testing.md) and
[runbook 005](../runbooks/005-compare-physics-with-other-engines.md);
the threads they run on are [threads.md](threads.md); the flows their
solves are built from are [flows.md](flows.md). What the work showed
about the design is in the physics retrospectives
([2026-09-23](../retrospectives/2026-09-23-physics.md),
[2026-09-25](../retrospectives/2026-09-25-physics-in-the-ecs.md),
[2026-09-26](../retrospectives/2026-09-26-physics-against-other-engines.md),
[2026-09-27](../retrospectives/2026-09-27-still-at-rest.md)).

## Goals

- **Our own, with its state in components.** Bodies, colliders and
  everything the solver carries from one step to the next live in the
  world or in the mod's state, so reloading physics swaps code under a
  running simulation like any other mod. A library such as Rapier keeps
  its own world beside ours, which would have to be rebuilt after every
  reload or pinned in a resident mod.
- **Deterministic.** The same inputs replay the same simulation, bit for
  bit, at any frame rate and any thread count, on the same build and
  machine (fused multiply-add and libm differ across machines, so that
  isn't promised). Both games run on the lockstep bootstrap and replay
  recorded routes.
- **As good as the references, and held to it.** How soon scenes come to
  rest, how deep they sink, how they bounce and slide are tests bounded by
  what Box2D and Rapier (2D), and Rapier and Box3D (3D), do on the same
  scenes ([Quality as a test](#quality-as-a-test)).
- **2D first.** Both games are 2D. 3D is experimental: a mod like 2D's, on
  the same storage, without 2D's game-facing features
  ([Rotation in 3D](#rotation-in-3d)).

Not built: joints, continuous collision, arbitrary polygons, a character
controller ([Open questions](#open-questions)).[^goals]

## The two mods

Each is an `engine_mod` with an interface crate other mods depend on
(`components.rs`; 3D's also `math.rs`) and an implementation that never
reaches a game's build: a game depends on `//engine/std/physics2d` in its
`mod_deps` and loads it. They share no interface. What they share is the
ECS (spatial keys generic over the dimension, live relations, ordered
tables) and `//engine/std/physics_common`, a library in both
implementations (`Slots`, `Softness`, `Closing`, the soft step's and the
broadphase's constants, `lanes::F` and `levels`), so a change to it
reloads the two physics mods and no game. What may go in it, and the
parity table every physics change keeps current, are
[physics-sharing.md](physics-sharing.md).[^share]

## Components

2D's interface (`engine/std/physics2d/components.rs`), whose doc comments
are the reference:

| component | what |
|---|---|
| `Position` | the collider's centre: a spatial key, bounded by `(Collider, Rotation)` |
| `Velocity`, `Body` | units a second; kind (`DYNAMIC`, `KINEMATIC`, `STATIC`), inverse mass, restitution, friction, gravity scale |
| `Collider` | a box (half extents) or a circle (`shape` a `u8`: the schema has no enums), layer and mask, `sensor`, `senses` |
| `Rotation`, `Spin` | the cosine and sine of the angle; radians a second. A dynamic body turns only with both ([Rotation](#rotation)) |
| `Gravity`, `Sleep`, `Tuning` | the world's settings, each on one entity, none meaning the default: gravity (none, no gravity), sleeping's thresholds, the substeps |
| `ContactPair`, with `Manifold`, `Response`, `Impulse`, `ContactPoints` | a contact, an entity while it lasts, in an ordered table by pair; what a pre-solve hook may change (`Response`); the warm start; the points where an end turns |
| `Overlap` | a sensor's or a sensed pair, an entity while it lasts, by pair |
| `Asleep`, `Slept`, `Still`, `Resting` | sleeping, kept in the world ([Sleeping](#sleeping)) |
| `Touching` | which sides touched something solid last step: "can the player jump" |
| `Contact`, `Trigger` (events) | two colliders began pressing, with the speed they met at; a sensor began overlapping |

`ContactPair::seen_from(me)` and `Overlap::other(me)` give the other end
from either side. `Spatial`, the spatial query, is in the interface too
([Spatial queries](#spatial-queries)). A body's position is physics's:
games keep only game state.

3D's interface (`engine/std/physics3d/components.rs`) has `Position` (the
spatial key), `Velocity`, `Rotation` (a unit quaternion),
`AngularVelocity`, `Body` (inverse mass and inertia about its own axes),
`Collider` (a sphere or a box), `Static`, the contacts (`ContactPair`,
`Manifold`, `Impulse`), and the settings `Gravity` and `Tuning`.

## The step

Physics steps at a fixed 60 Hz (`phase::SIMULATE_HZ`) whatever the frame
rate: its phase, `physics2d::step` (3D: `physics3d::step`), is after
`simulate` and before `late` at `simulate`'s fixed rate
([scheduling.md](scheduling.md#fixed-rates)), and each system reads the
step's length as a `Dt`. A game sets velocities in `update` or
`simulate` and reacts to contacts in `late`.

The step is a chain of the mod's systems, each after the one before:

1. **`integrate_velocities`**: gravity into every awake dynamic body's
   velocity (and, in 2D, the wakes games caused: [Sleeping](#sleeping)).
2. **`find_contacts`**: the [broadphase](#broadphase), the
   [narrowphase](#narrowphase) on each pair, then the world's contacts
   brought in line in one pass (both lists are in pair order): contacts
   that persist updated in place, their impulses kept, the rest spawned
   and despawned. In 2D pairs are filtered by layer and mask, and
   overlaps of sensors and sensed layers become `Overlap`s (a sensor's
   new ones a `Trigger`) and go no further.
3. **The solve**: a pipeline of systems handing [flows](flows.md) along,
   `solve`, the gathers, `prepare`, `passes`, `finish` and the scatters:
   2D's nine over five flows, 3D's eight over four. The diagram of
   which system makes, sees, passes and takes which flow is in each mod's
   `pipeline.rs` header, its one home. The gathers copy the moving bodies
   and the contacts into the solver's dense layout, the [soft
   step](#the-soft-step) runs on that copy, and the scatters write
   velocities, positions, rotations, impulses, `Touching`, `Contact`
   events and sleeping back, a position or rotation only where it
   changed, bit for bit, so a pile at rest costs storage nothing.
   Writing positions re-sorts them at the scatter's apply node, so every
   system after the step finds bodies where they are.

- **`solve` comes first**, though it only reads the step's settings, so a
  **pre-solve hook** ordered `.after("physics2d::find_contacts")
  .before("physics2d::solve")` sees this step's contacts and overlaps and
  may change a contact's `Response` (disable it: a one-way platform; its
  restitution: a bounce pad) before anything is gathered. The
  platformer's walkers meet the player this way. What physics found is as
  of when it found it, so a system reading it belongs after the system
  that found it.
- **The flows are each mod's own**, not its interface's: they carry the
  solver's layout, and a mod that saw them would be rebuilt with every
  change to it. What adopting flows cost and changed:
  [flows.md](flows.md#physicss-adoption-stage-2).
- **Contacts are entities**, which carries them to the next step for warm
  starting and "began pressing", and a reload of physics leaves them in
  place like any other entity.
- **Determinism**: a fixed step, contacts stored and solved in an order
  fixed by the contacts (pair order, or the colors found in it), no
  iteration over hash maps, and the same result at any thread count
  ([Solving across threads](#solving-across-threads)).[^onestep]

## Broadphase

`Position` is a spatial key with `Collider` and `Rotation` as its
extents, so the ECS keeps every table of positions in Z-order with each
page's box ([spatial-storage.md](spatial-storage.md)), and a turned box is
bounded as turned. The broadphase is `Live<Contacts>`, the live relation
each mod declares (`Contacts: Proximity`, on `Position`): pairs whose
boxes, grown by the speculative margin, meet, kept between steps over fat
boxes `FAT` (0.02) past each box and found again only for rows that left
theirs ([live.md](live.md); spatial-storage.md, "Keeping pairs"). There is
no index to build: a static body's pages never change. 2D's sides are
the awake bodies that can move, against statics and sleeping bodies;
3D's, moving bodies against `Static`. Found afresh, the pairs are found
across the world's threads ([Solving across
threads](#solving-across-threads)).[^grid]

## Narrowphase

- **2D**: where neither shape is turned, the axis-aligned tests (box–box,
  box–circle, circle–circle): a normal and a depth, no points. Two boxes
  flush on one axis take the face the box slides along, from the pair's
  relative velocity, so a body running over a row of tiles doesn't snag
  on their seams. Where either is turned, Box2D's separating axis and
  clipping (`narrow::collide_turned`): up to two points, each with its
  arms, separation and a feature id (the edges it came from).
- **3D** (`narrow.rs`): sphere against sphere or box, one point; box
  against box, Box3D's separating-axis test over the 15 axes with the
  last step's axis tried first (both cheaper and hysteresis: a resting
  box's faces don't flicker), the incident face clipped, reduced to four
  points by Box3D's area rule, `u32` feature ids. GJK and EPA are a
  variant (`gjk.rs`). A box pair that has moved less than 0.03
  (`Tuning::recycle`) since its manifold was found keeps it, its anchors
  carried and its separation updated (Box3D's contact recycling), which
  stills chattering planks and halves the narrowphase on a pile.
- **Speculative contacts**: pairs within `MARGIN` (0.05) are contacts
  before they touch, and the solver lets a gap close in a substep and no
  more, so a body stops at the surface instead of after sinking in.

Both mods' narrowphases run across the world's threads in chunks of
pairs.[^narrow]

## Contact points and warm starting

A turning contact's points are kept on it: in 2D a `ContactPoints`
component on every contact, written only where there are points, so a
world where nothing turns pays a turned-or-not test a pair and little
else; in 3D up to four points inline in `Manifold`. Each step's points
are matched to the last step's by feature id, as Box2D and parry do, and
start from their impulses: without warm starting nothing comes to rest
and a big pyramid falls apart. Matching by nearest point is a variant in
both.[^points]

## The solver

### The soft step

Box2D v3's soft step, as Rapier and Box3D also run it. A step is five
substeps (2D's `Tuning::substeps`; 3D's `Tuning`); each applies a
substep's share of gravity, warm-starts, runs one soft pass that pushes
penetration out through the velocity (at most `MAX_PUSH`, 3 a second),
moves the bodies and updates each point's separation from how far its
bodies moved and turned, then two rigid relaxing passes that take the
push's speed back out, with Coulomb friction in the relaxing passes only
(Rapier's rule). Contacts are springs of `Softness` (Box2D's
`b2MakeSoft`, damping ratio 10) as stiff as a share of the substep rate:

- **2D**: a quarter (75 Hz) between moving bodies, a half against a
  static one. A soft contact sinks by its load over mass times ω², so
  stiffness is depth, and the stiffest that holds is a quarter of the
  substep rate: depth is bought with substeps. Five substeps and two
  relaxing passes rest piles as soon as Box2D and Rapier do and sink
  them a quarter as deep.
- **3D**: a fifth (60 Hz) between moving bodies and a quarter against a
  static one (Box3D's cap): a cube on four points rocks at 1.22 times its
  contacts' rate, and at 2D's stiffness a column of turning boxes never
  rests.

A game that stacks tall sets more substeps (`physics2d::Tuning`: a
20-high turning stack rests from step 30 at six, against 240 at five, for
about a fifth more solver; measured 2026-09-28). Friction mixes as the smaller of the two in
2D and `sqrt(a b)` in 3D, restitution as the larger in both: the mixing
rule is undecided (get-emj.81).[^soft]

### Restitution

Applied once after the substeps, to points that pushed, from each
point's closing speed as the step began, before the step's gravity
(`Closing::Before`, the default in both), as Box2D, Box3D and Rapier take
it; nothing closing slower than `BOUNCE_THRESHOLD` (1) bounces. A
lossless ball rebounds to its drop and never above it. 2D passes over a
contact's points four times (`BOUNCE_ITERATIONS`), 3D once. Judged on
families of bounces over a grid of what decides them (physics-testing.md,
"Families of a law").[^bounce]

### Order and lanes

Contacts whose ends turn are solved four at a time: Box2D's wide layout
(batches of four contacts field by field, bodies gathered into lanes and
scattered back, inverse masses kept by each contact) on
`physics_common::lanes::F`, plain arrays LLVM vectorizes to SSE2, no
intrinsics and no unsafe. No two contacts in a batch may share a body
that moves, so contacts are grouped:

- **By Box2D's graph colors**, the default in both mods
  (`engine_ecs::shape`'s `Coloring::greedy`, a contact with a static end
  never in color 0; `Coloring::pack` lays the colors out in batches): a
  pile or pyramid takes six or seven colors a step in 2D and eleven or
  twelve in 3D, which threads can share. Solving the colors in
  turn is the loop one contact at a time over the colors' order, bit for
  bit, which is the test the lanes and the threads are held to.
- **By level of the sweep in pair order** (`physics_common::levels`), a
  variant in both (2D `Wide::Levels`, 3D `order=levels`): the sweep in
  pair order bit for bit, but hundreds of levels a step, too many
  barriers to share.

Each lane is the scalar code, operation for operation (Rust neither
reassociates nor contracts `f32`). Where the lanes can't be the loop, the
loop runs: a world where nothing turns keeps 2D's loop one contact at a
time in pair order, which the games' replays hold to the bit; 3D solves
a step one contact at a time (`in_order`) where its batches would be
under half full, or a body that doesn't move carries a `-0.0`, an
infinity or a NaN.[^lanes]

### What a turning point carries

A point starts the next step from what the last one left it:

- **2D**: its normal impulse from the last substep and its tangent
  averaged over the substeps (`Carry::Normal`); contacts whose ends don't
  turn carry the mean. The mean alone lags a breathing pyramid by two
  substeps and let colored pyramids fall; the last substep for both
  dropped card houses. This option meets every bound in every contact
  order and stands as many card houses as Box2D.
- **3D**: the mean of both (`Carry::Mean`); the last substep, and 2D's
  rule, are variants, which cost 3D's piles bounds they didn't cost 2D's
  (get-emj.97).[^carry]

### Solving across threads

Both mods run across the world's executor, the resident `threads` mod's
pool, through the shapes their systems declare
([flows.md](flows.md#parallel-shapes)); the pool, its placement on one
CCD, its warmth and what it measures are [threads.md](threads.md). What
runs across it:

| stage | 2D | 3D |
|---|---|---|
| gravity | a `ParMap` walk | one thread |
| broadphase | `Live`'s, found afresh across threads (the ECS's own split) | the same |
| colliders' gathers, narrowphase, merge | `ParMap` (while nothing sleeps: waking looks bodies up one at a time) | `ParMap` (the statics' gather on one thread) |
| the solve's gathers and scatters | `ParMap` page walks, each chunk its own part of the output | one thread |
| the fill and the passes | a `Passes` program, the fill its first stage | the same |
| `finish`, the write-back | a `ParMap` over parts, four a thread | the same |

What stays on one thread, and why: the coloring (greedy in pair order,
which is what makes the colors, and the result, the contacts'), seating
each contact in its lane, the bodies' states, `Slots`, the spawns and
despawns a merge records (made in walk order, so ids don't depend on
timing), sleeping's bookkeeping, and a step nothing turns in, solved in
pair order, which no shape can split. 2D keeps a step's passes on one
thread (`Passes::serial`) where a still body carries a `-0.0`, which
batches of one color would each write back and turn into `0.0`
(`staged::shareable`).

**Bit for bit at any thread count**: a stage's blocks share no moving
body, the bodies are shared as relaxed atomics (no unsafe code), and
everything in order is collected per chunk and joined in chunk order.
Held by `quality_test`'s `the_mod_across_threads_is_the_arrays_bit_for_bit`,
`physics2d_test`'s piles on four threads against one (positions,
contacts, events, ticks and, falling asleep, islands),
`reloading_physics_under_the_pool_is_reloading_it_on_one_thread`, and
`physics3d_test`'s
`contacts_found_across_threads_are_one_threads_bit_for_bit`; both
baselines and the 3D fingerprint were byte-identical at `ENGINE_THREADS`
1, 2, 4 and 8 when each split landed. What threads buy, stage by stage,
is threads.md's "Measured".[^threads]

## Sleeping

2D only. On by default at `Sleep::DEFAULT` (slower than 0.05 a second for
0.5 s); a `Sleep` entity changes it and `Sleep::OFF` turns it off. An
island, dynamic bodies joined by pressed contacts, whose bodies have all
been slower than `Sleep::speed` for `Sleep::time` falls asleep: its
velocities are zeroed, each body gets `Asleep { island }` (and `Slept`,
physics's own record), and each contact neither end of which moves, one
of them asleep, gets `Resting`. Both move rows to tables of their own,
which the step's queries exclude, so a sleeping body is skipped by the
table it's in, not looked up, and change detection leaves its rows
alone. A resting contact keeps its impulses, the warm start for when its
ends wake. A pile of 10 000 asleep cost a frame about 25 µs when measured
(2026-09-26): a look at each sleeping table's ticks for what games
changed.

**What's kept, all in the world**: `Asleep` (who's asleep now, which a
game may write), `Slept` (who physics last had asleep: the baseline a
game's changes are found against, with a count in the mod's state of how
many it has given), `Still { since }` (sparse, on awake dynamic bodies
slower than the threshold: the step they went slower), and `Resting`. A
reload carries the rest in the mod's state, so it doesn't show: the
games' replays, reloaded every frame, are bit for bit the runs without
reloads.

**What wakes an island** (the whole island), and where it's seen:

- in `integrate_velocities`, what a game changed since the last look: a
  sleeping body's velocity, position, collider, body, rotation or spin
  written; a body woken (its `Asleep` removed) or unmade; a sleeping body
  despawned (fewer `Slept` than given); `Sleep` despawned or `wake` sent;
- in `find_contacts`: a static spawned, written, moved or despawned into
  or out from under sleeping bodies; a pressed contact ending;
- after the solve: a moving awake body pressing on a sleeping one, or a
  kinematic body moving into one.

A wake seen before the solve takes effect in that step (the bodies get
their gravity and their contacts solved, so a pile whose floor goes falls
as the same pile awake does); one seen after, from the next. A game puts
bodies to sleep by giving them `Asleep` in an island it numbers.

**What it doesn't do**: an island touching a woken one wakes a step
later; a body moving into a sleeping one sees it immovable for one step
(Box2D wakes both in its collide phase); a body a game puts to sleep while
pressing on something wakes in that step; no body can opt out; and a game
writing a velocity every frame wakes its body every time it falls
asleep. Each is in the log's "Sleeping", with how it was found and what
fixing it would take.[^sleep]

## Spatial queries

A `Spatial<Data, Filter, Changes>` is a `Query` whose entities are found
by where their colliders are: a region query over the spatial storage,
then an exact test of each turned or unturned shape. It hands the same
rows and items to the same closures, declares the same footprint plus
reads of `Position` and `Collider` (so its `Data` can't write those two),
and sees every move made before it in the plan and none after, as any
change.

```rust
use physics2d::{Ray, Spatial, Vec2, circle, rect};

// The walkers' ledge check: solid ground just ahead and below?
fn walk(.., mut tiles: Spatial<&Tile>) {
    let mut ground = false;
    tiles.overlapping(rect(ahead, Vec2::ZERO), |_, t| ground |= t.kind == SOLID);
}

// An explosion: everything with health in the radius is hurt.
fn explode(.., mut hit: Spatial<&mut Health, (), Despawns>) {
    hit.overlapping(circle(at, radius), |row, mut health| { /* .. */ });
}

// Line of sight: the first solid thing along the ray, nearest first.
fn look(.., mut blockers: Spatial<&Collider>) {
    let first = blockers.cast(Ray::new(eye, dir, 20.0), |hit, row, c| (!c.sensor).then_some((row.entity(), hit.t)));
}
```

`overlapping(probe, f)` visits what overlaps a `Placed` (`rect`,
`circle`), in entity order; `any_at(point)` is the point case; `cast(ray,
f)` visits hits nearest first until `f` returns `Some`. The filter does
what layers would (`Spatial<(), With<Tile>>` sees only tiles); layers and
masks are for what collides. `Spatial` lives in physics's interface, not
`engine_ecs` (the ECS knows boxes, not shapes), so changing it is an
interface change. It is a `Compose` of two queries, `engine_api`'s
parameter built from others: its footprint is their union, and it only
wraps what its declared parts fetched.[^spatial]

## Rotation

2D bodies turn: boxes tip over edges, stacks and pyramids stand on two
points a contact, discs roll. `Rotation` (the cosine and sine, as Box2D's
`b2Rot`: turning a vector is four multiplies, bounding a box no sine) on
any collider, and `Spin` on a body; a dynamic body with both turns with
its shape's inertia at its mass (a box's `m (w² + h²) / 12`, a disc's
`m r² / 2`); a kinematic one turns at its spin; a static with a
`Rotation` is a turned plank. Within the step a rotation is stepped to
first order and normalized every substep, and a point's separation
follows its arms as the bodies turn (Box2D's `b2SolveContact`).

**The rotation lock is absence**: a body without a `Spin` keeps its
rotation, and one without a `Rotation` is tested as an axis-aligned shape
with no points. A world where nothing turns pays nothing for rotation:
its contacts keep one row at the normal, and a step with no points and
nothing spinning runs a solve compiled without them
(`solve_all::<false>`), the same computation as before rotation, bit for
bit. Giving every body a `Rotation` (what a flag or an infinite inertia
leaves) cost 11% on a pile and 32% on a pyramid. Turning bodies' angular
state is a list beside the bodies inside the solve (`Spinning`), not in
every body. The games' bodies don't turn.[^rotation]

## Rotation in 3D

`//engine/std/physics3d`: spheres and boxes that turn, a mod laid out as
2D's (interface `components.rs` and `math.rs`; implementation `lib.rs`,
`narrow.rs`, `gjk.rs`, `solver.rs`, `pipeline.rs`), on 3D spatial storage
(`Position` bounded by `(Collider, Rotation)`, `|R| h`, so a turn
re-bounds its row through storage).

- **The solver** is 2D's soft step with Box3D's angular terms: per point,
  anchors on both bodies and an effective mass with the angular terms,
  each body's world inverse inertia, all fixed once a step; a point's
  separation within the step is its separation when found plus its
  anchors' moves along the normal. Friction is per contact at the points'
  centroid, clamped to a disc, with twist friction about the normal, in
  the relaxing passes only, as Box3D, Rapier and Jolt have it. A rotation
  is stepped to first order and normalized every substep, at most π/4 a
  step (`MAX_ROTATION`). Its kernels run in four lanes in colors as 2D's
  do ([Order and lanes](#order-and-lanes)), every `Tuning` variant
  included.
- **Every measured choice is a `Tuning` in the world**: one entity, or
  none for the defaults, read every step (`pile3d tune …`, `TUNE=` in the
  tests and benches): substeps, relax passes and stiffness, friction in
  the push, how rotation integrates, inertia per step or substep,
  separations exact or linear, warm starting (ids, nearest, cold), the
  box–box test (SAT cached, SAT, GJK and EPA), the reduction (area,
  line), recycling, carry, closing speed, lanes and order. Settings in
  the world, not in statics, because a reload maps a new image whose
  statics start over; the mod's timings are its state.
- **Reloads are invisible**: `:reload_test` replays a tuned pile of
  boxes, and a sphere spun onto it, while physics3d, the scene and the
  scheduler are swapped for their twins every frame, one a frame and in
  batches, and every frame is the run without reloads bit for bit,
  contacts, manifolds, cached axes and ticks included.
- **`pile3d`** (`tests/pile.rs`) builds the comparison's scenes from
  `tests/scenes.rs`, the file the comparison builds them from in every
  engine; `pile3d_game` runs it on lockstep, with `stats` and `stages`
  messages to physics3d.
- **Held to the bit** by an exact fingerprint (`:exact_test`), beside the
  baselines (physics-testing.md, "The exact fingerprint").
- **Left out**: layers, sensors, overlaps and events, kinematic bodies,
  sleeping, pre-solve hooks, rolling resistance, gyroscopic terms; and
  locked bodies pay for manifolds (a locked rotation is a zero inverse
  inertia: get-emj.80). The parity table has each.[^3d]

## What the ECS costs

The verdict of 2026-09-24 holds: storage is no design blocker. Against
the same step on plain arrays, checked to be the same computation bit
for bit (`//engine/std/physics2d:tax`), the ECS's step is within a few
percent settled and faster at rest, where the arrays' sweep and prune
suffers. The solver works on a copy (gathered, solved many passes over,
written back) because the copy is a transpose into the solver's layout,
not a patch over a storage flaw: solving in place in the world's pages
was slower, and would save only the copy's cost
([working-sets.md](working-sets.md) weighs storage owning it). What
storage still costs is the copies in and out, the spatial re-sort of
what moved, and re-sorting the contacts' ordered table as contacts begin
and end.[^ecs]

## Quality as a test

How soon a scene comes to rest, how deep it sinks while it does and once
it has, what energy is left, whether stacks and pyramids stand and
nothing escapes, how a box slides and a ball bounces, and that sleeping
follows, are tests in 2D and 3D, locked and turning, bounded by what the
reference engines do on the same scenes (each test's comment has the
reference values) and held both ways to our own accepted results, each
within a band set from measured noise. A known failure is an ignored test
naming its bead, not a looser bound.

| target | what |
|---|---|
| `//engine/std/physics2d/compare:quality_test`, `:quality_long_test` (manual) | 2D piles, pyramids, stacks, sleeping, the families, the mod bit for bit the arrays and across threads, the baseline |
| `//engine/std/physics2d/compare:behaviour_test`, `:behaviour_long_test` (manual) | 2D ramps, bounces, mass ratios, overlap recovery, fast bodies, card houses, ladders, dominoes, the families at the edge |
| `//engine/std/physics3d/compare:quality_test`, `:quality_long_test` (manual) | 3D piles of cubes and planks, stacks, the families, the baseline |
| `//engine/std/physics3d/compare:behaviour_test`, `:behaviour_long_test` (manual) | 3D ramps, bounces, mass ratios |
| `//engine/std/physics3d:exact_test` | the 3D fingerprint, and the lanes and the shared states bit for bit |

How the bounds are set, the baseline and its bands, the families and the
exact fingerprint are [physics-testing.md](physics-testing.md); how to run
the comparisons and the debug view (`view.rs`, every engine's bodies and
contacts side by side, as SVG or text) is [runbook
005](../runbooks/005-compare-physics-with-other-engines.md).[^quality]

## Against other engines

The comparisons (`//engine/std/physics2d/compare` against Box2D v3.1.1
and Rapier 2D 0.36; `//engine/std/physics3d/compare` against Rapier 3D
0.36, Jolt 5.6 and Box3D 0.1) run every engine on one thread, ours
included, on the same scenes, matched as the runbook says (2D's timed
row is `Ecs::alone`, with no pool; 3D's is `Config::threads`'
`Threads::One`). A row of ours on its own pool of n threads, pinned to
one CCD, is added and named apart (2D `VARIANTS=threads:<n>`, 3D
`--threads=<n>`); the references in it still run on one.[^cmp-threads]
Where ours stands now:

- **Quality**: at least level everywhere the tests look, and ahead on
  depth: piles and pyramids rest as soon as the references' and sink a
  quarter to a fifth as deep (3D a third), in 2D heavy boxes stand where
  the references crush them, and the hand calculations (ramps, ladders,
  bounces) are met as closely as any reference meets them. Short of the
  references: 3D's card houses and tall columns (get-emj.98,
  get-emj.99), a heavy box on light ones resting late (get-emj.57), and
  no continuous collision (get-emj.59).
- **Time, on one thread**: in 3D ours costs about 1 to 2.3 times Rapier
  and Box3D (below), where it was 1.3 to 3 times on 2026-09-26. Most of it is
  the solver's passes, a quality choice: five substeps of three passes
  against the references' four of two. On eight threads ours is faster
  than every reference, but they ran on one: that row is how ours
  scales, not a like-for-like comparison. 2D's timings weren't re-run
  with this (its timed row was one thread's already): the log's tables
  stand.

**3D, measured 2026-10-04** (get-emj.113): ms a step over the whole run,
`./bazel run --config=bench //engine/std/physics3d/compare:bench -- all
1000,10000 all [--rotate] --threads=8`, 1000 bodies the median of three
runs and 10 000 one, every engine at its defaults. Other agents' work
loaded the machine (load average 6 to 34), so take ±15% as noise; the
cases marked † were run again once it dropped, and planks 10 000
turning, two values each, stays unreliable.

| turning | ours | ours, 8 threads | Rapier | Jolt | Box3D |
|---|---|---|---|---|---|
| spheres 1000 | 1.70 | 0.67 | 0.88 | 1.84 | 1.41 |
| boxes 1000 | 2.08 | 0.70 | 0.92 | 1.35 | 1.09 |
| planks 1000 | 2.95 | 0.90 | 1.21 | 2.18 | 1.56 |
| rain 1000 | 1.58 | 0.65 | 0.87 | 1.23 | 1.06 |
| spheres 10 000 | 20.2 | 5.8 | 18.9 | 28.3 | 18.5 |
| boxes 10 000 | 21.7 | 5.0 | 13.3 | 18.6 | 11.3 |
| planks 10 000 † | 36.2 / 45.6 | 27.5 / 22.8 | 23.9 / 24.7 | 37.4 / 39.2 | 22.6 / 22.9 |
| rain 10 000 | 19.4 | 5.6 | 15.7 | 16.3 | 14.3 |

| locked | ours | ours, 8 threads | Rapier | Jolt | Box3D |
|---|---|---|---|---|---|
| spheres 1000 | 1.05 | 0.49 | 0.63 | 1.20 | 1.02 |
| boxes 1000 | 1.83 | 0.56 | 0.89 | 1.13 | 1.04 |
| spheres 10 000 | 12.8 | 3.4 | 12.2 | 17.3 | 13.4 |
| boxes 10 000 † | 22.3 | 5.2 | 14.8 | 14.0 | 11.8 |
| planks 10 000 | 16.5 | 4.5 | 11.0 | 13.4 | 9.4 |
| rain 10 000 | 11.9 | 3.1 | 8.5 | 11.5 | 7.6 |

Ours by stage on one thread, 10 000 turning, µs a step (broadphase /
narrowphase / solver / copies in and out; outside the systems 491, 428
and 499, the bench no longer printing the re-sorts apart): spheres 572 / 1306 / 16 043 / 875,
boxes 502 / 2047 / 17 451 / 654, planks 730 / 4184 / 28 832 / 987. The
broadphase is under half what it was before `Live` kept pairs, and the
narrowphase half of boxes' before recycling (the log's "Against the
others, turning": 1190 / 4120 / 23 141 / 678 for boxes, 2026-09-26).

The measurements before these, scene by scene, 2D's included, are the
log's "Against other engines", "Against other engines, bodies turning",
"Against the others now", "Against the others, turning" and "How it
scales".

## The games on it

Both games moved onto `//engine/std/physics2d` on 2026-09-23 with every
recorded route passing unchanged, and replay them, reloads included, on
every test run.

- **The platformer**: tiles are static boxes; spikes, the goal and coins
  sensors whose `Trigger`s the rules read in `late`; walkers are bodies
  that collide with tiles only, turn at a wall from `Touching` and at a
  ledge with a `Spatial<&Tile>` point query, and sense the player, so
  meeting it is an `Overlap`, which `walkers::meet`, a pre-solve hook,
  tells into a stomp or a touch.
- **Pong**: the ball is a dynamic circle with restitution 1, friction 0
  and no gravity; the paddles are kinematic boxes moved by velocity; the
  goal lines are sensors. Pong adds speed and spin on a paddle's
  `Contact` and scores on a goal line's `Trigger`.

Neither game's bodies turn, so their replays hold the locked path to the
bit. What porting them showed about the ECS is the log's "The games on
it".[^games]

## Open questions

- **Continuous collision** (get-emj.59): a ball is stopped for certain
  while a step is at most the margin, its radius and half the wall (0.8
  for pong's paddle, 48 a second, against pong's fastest 40), and pinned
  by a test at that limit; Box2D sweeps fast bodies against statics.
- **Kinematic characters**: the platformer's player is a dynamic body
  with no friction; a character controller (slopes, steps, one-way
  platforms) waits for a game that needs it.
- **Friction mixing** (get-emj.81), **3D's carry and bounce passes**
  (get-emj.82), **3D's rotation lock** (get-emj.80) and **the features 3D
  lacks** (get-emj.77 to .79): the parity table's open rows.
- **Kept colors** (get-emj.74): colors kept across steps, so the serial
  coloring goes; it would make colors state a snapshot carries.

## Where the old sections went

Until 2026-10-04 this doc was the log, and code comments cite its
sections by name (`physics.md, "Still at rest"`). Each is in [the
log](../retrospectives/2026-10-04-physics-log.md) under the same name,
whole; this table says where its current content is here. A name with a
row of its own and no section here keeps its old anchor on this page.

| old section, in the log | what it holds | the current design, here |
|---|---|---|
| [Goals](../retrospectives/2026-10-04-physics-log.md#goals), [Components](../retrospectives/2026-10-04-physics-log.md#components), [The step](../retrospectives/2026-10-04-physics-log.md#the-step), [Broadphase](../retrospectives/2026-10-04-physics-log.md#broadphase), [Spatial queries](../retrospectives/2026-10-04-physics-log.md#spatial-queries) (and Parameters made of parameters), [Walkthroughs](../retrospectives/2026-10-04-physics-log.md#walkthroughs) | the design as built 2026-09-23, then amended | [Goals](#goals), [Components](#components), [The step](#the-step), [Broadphase](#broadphase), [Spatial queries](#spatial-queries), [The games on it](#the-games-on-it) |
| [What the ECS costs](../retrospectives/2026-10-04-physics-log.md#what-the-ecs-costs), [The real pile](../retrospectives/2026-10-04-physics-log.md#the-real-pile) (and Tried, and slower) | `:tax`'s tables, the ECS against arrays; solving in place, renumbering, mapping by location, tried and slower | [What the ECS costs](#what-the-ecs-costs) |
| <a id="parallelism"></a>[Parallelism](../retrospectives/2026-10-04-physics-log.md#parallelism) | the first split of every stage across threads (2026-09-24), what serialized each, where the data lives | [Solving across threads](#solving-across-threads); threads.md |
| [Sleeping](../retrospectives/2026-10-04-physics-log.md#sleeping), [The scenes](../retrospectives/2026-10-04-physics-log.md#the-scenes) | how it was built, its tables and costs, the three shapes of its record, its tests and mutations | [Sleeping](#sleeping) |
| <a id="parallel-solving"></a>[Parallel solving](../retrospectives/2026-10-04-physics-log.md#parallel-solving) | the colored, wide and island solves on arrays (2026-09-24): coloring works, islands don't | [Order and lanes](#order-and-lanes), [Solving across threads](#solving-across-threads) |
| [Against other engines](../retrospectives/2026-10-04-physics-log.md#against-other-engines) (How the scenes are matched, Time, Quality, Where the time goes, What would close the gaps, Bringing them in) | the first comparison, locked, with the split impulse (2026-09-25) | [Against other engines](#against-other-engines); runbook 005 |
| <a id="settling"></a>[Settling](../retrospectives/2026-10-04-physics-log.md#settling) (What the other engines do, The options, measured, How it extends to rotation) | the split impulse replaced by the soft step: every option, measured (2026-09-26) | [The soft step](#the-soft-step) |
| [Rotation](../retrospectives/2026-10-04-physics-log.md#rotation) (The narrowphase for turned shapes, Rotation in the soft step, Against other engines, bodies turning, What the 3D spike predicted) | rotation in 2D, each choice against the others (2026-09-26) | [Rotation](#rotation), [Narrowphase](#narrowphase) |
| <a id="the-rotation-lock"></a>[The rotation lock](../retrospectives/2026-10-04-physics-log.md#the-rotation-lock) | the lock by absence, measured against a flag | [Rotation](#rotation) |
| <a id="contact-points"></a>[Contact points](../retrospectives/2026-10-04-physics-log.md#contact-points) | where points are kept, and warm starting by feature id | [Contact points and warm starting](#contact-points-and-warm-starting) |
| <a id="3d-translation-only-spike"></a>[3D, translation only (spike)](../retrospectives/2026-10-04-physics-log.md#3d-translation-only-spike) | 3D before rotation, against Rapier, Jolt and Box3D (2026-09-25) | [Rotation in 3D](#rotation-in-3d) |
| [Rotation in 3D](../retrospectives/2026-10-04-physics-log.md#rotation-in-3d) (What it is, The choices, measured, Against the others, turning) | 3D's choices one by one, and the comparison turning (2026-09-26) | [Rotation in 3D](#rotation-in-3d) |
| <a id="a-mod"></a>[A mod](../retrospectives/2026-10-04-physics-log.md#a-mod) | physics3d as a mod: its pipeline, no static state, through the engine, reloads, what is shared with 2D | [Rotation in 3D](#rotation-in-3d), [The two mods](#the-two-mods) |
| <a id="the-solver-in-lanes"></a>[The solver in lanes](../retrospectives/2026-10-04-physics-log.md#the-solver-in-lanes) | 3D's lanes by level, bit for bit (2026-10-03, get-emj.52) | [Order and lanes](#order-and-lanes) |
| <a id="colouring-the-3d-solve"></a>[Colouring the 3D solve](../retrospectives/2026-10-04-physics-log.md#colouring-the-3d-solve) (The decision: Cm, now) | colored or by level, and the carry, in 3D (2026-10-03, get-emj.90); the known misses (get-emj.96) | [Order and lanes](#order-and-lanes), [What a turning point carries](#what-a-turning-point-carries) |
| <a id="what-3d-asks-of-the-storage-design"></a>[What 3D asks of the storage design](../retrospectives/2026-10-04-physics-log.md#what-3d-asks-of-the-storage-design) | bounds from two extents, turning re-bounds, never spheres, kept pairs | [Broadphase](#broadphase); spatial-storage.md |
| [Quality as a test](../retrospectives/2026-10-04-physics-log.md#quality-as-a-test) | the measures, how bounds were set, the first bounds and what they catch (2026-09-26) | [Quality as a test](#quality-as-a-test); physics-testing.md |
| <a id="quality-beyond-settling"></a>[Quality beyond settling](../retrospectives/2026-10-04-physics-log.md#quality-beyond-settling) (The debug view, The scenes, Results, What they found) | ramps, bounces, mass ratios, overlap, fast bodies, structures, in every engine (2026-09-28) | [Quality as a test](#quality-as-a-test), [Against other engines](#against-other-engines) |
| <a id="bounces"></a>[Bounces](../retrospectives/2026-10-04-physics-log.md#bounces) | the bounce families, and the closing speed decided (2026-09-29) | [Restitution](#restitution) |
| <a id="still-at-rest"></a>[Still at rest](../retrospectives/2026-10-04-physics-log.md#still-at-rest) | recycling, softer static contacts, 2D's substeps as a setting (2026-09-27) | [Narrowphase](#narrowphase), [The soft step](#the-soft-step) |
| <a id="the-solvers-speed"></a>[The solver's speed](../retrospectives/2026-10-04-physics-log.md#the-solvers-speed) (Where the time went, The options, measured, Against the others now) | the lanes, levels then colors (2026-09-27) | [Order and lanes](#order-and-lanes) |
| <a id="why-colors-let-the-pyramid-fall"></a>[Why colors let the pyramid fall](../retrospectives/2026-10-04-physics-log.md#why-colors-let-the-pyramid-fall) | the warm start's lag, found and fixed (2026-09-28, get-emj.48) | [What a turning point carries](#what-a-turning-point-carries) |
| <a id="what-a-turning-point-carries-the-decision-matrix"></a>[What a turning point carries: the decision matrix](../retrospectives/2026-10-04-physics-log.md#what-a-turning-point-carries-the-decision-matrix), <a id="the-decision-b-colored"></a>[The decision: B colored](../retrospectives/2026-10-04-physics-log.md#the-decision-b-colored) | A, B and C, by level and colored, every bound (2026-09-28, get-emj.61) | [What a turning point carries](#what-a-turning-point-carries) |
| [Solving across threads](../retrospectives/2026-10-04-physics-log.md#solving-across-threads) (The profile, What's built, The options, measured, The host's threads, How it scales) | the colored solve across threads in the solver (2026-09-29), before the threads mod | [Solving across threads](#solving-across-threads); threads.md |
| <a id="the-fill-as-the-passes-first-stage"></a>[The fill as the passes' first stage](../retrospectives/2026-10-04-physics-log.md#the-fill-as-the-passes-first-stage) | `prepare` mapped, the fill moved into the passes (2026-10-03, get-znt.40) | [Solving across threads](#solving-across-threads) |
| <a id="the-write-back-across-threads"></a>[The write-back across threads](../retrospectives/2026-10-04-physics-log.md#the-write-back-across-threads) | `finish` as a map over parts (2026-10-03, get-znt.45) | [Solving across threads](#solving-across-threads) |
| <a id="the-3d-narrowphase-across-threads"></a>[The 3D narrowphase across threads](../retrospectives/2026-10-04-physics-log.md#the-3d-narrowphase-across-threads) | 3D's `find_contacts` split (2026-10-03, get-emj.101) | [Solving across threads](#solving-across-threads) |
| <a id="the-2d-solves-gathers-and-write-backs-across-threads"></a>[The 2D solve's gathers and write-backs across threads](../retrospectives/2026-10-04-physics-log.md#the-2d-solves-gathers-and-write-backs-across-threads) | 2D's walks split (2026-10-03, get-emj.103 to .105) | [Solving across threads](#solving-across-threads) |
| [Open questions](../retrospectives/2026-10-04-physics-log.md#open-questions), <a id="spike-results"></a>[Spike results](../retrospectives/2026-10-04-physics-log.md#spike-results), [The games on it](../retrospectives/2026-10-04-physics-log.md#the-games-on-it) | the spike's numbers and sharp edges (2026-09-23), the ports | [Open questions](#open-questions), [The games on it](#the-games-on-it) |

[^goals]: *(History, 2026-09-23 to 2026-10-04.)* The goals were first "2D
    only", 3D waiting for the renderer spike (get-y5t.8), and rotation an
    open question kept out of the MVP; rotation landed on 2026-09-26 in
    both dimensions, and physics3d became a mod the same day. The log's
    "Goals".

[^share]: *(History, 2026-09-26 to 2026-10-02.)* Until `physics_common`
    the mods shared nothing but the storage, and `Slots`, `Softness` and
    the closing speed were copied. A common interface for `Position` or
    `Velocity` was measured and rejected: a 2D interface change rebuilds
    72 actions and reloads 23 game mods, a 3D one 17 and none (the log's
    "A mod").

[^onestep]: *(History, 2026-09-24.)* Physics first stepped once per frame
    by `Clock::dt`, capped at 1/30 s, so a real-time game's simulation
    depended on its frame rate; fixed-rate phases replaced it. Until
    get-znt.33 (2026-10-02, and get-znt.35 for 3D) the solve was one
    system; the pipeline is its code split where its stages were (flows.md,
    "Physics's adoption").

[^grid]: *(History.)* The broadphase was a uniform grid built every step,
    with a second grid published for spatial queries (2026-09-23), until
    positions became a spatial key (2026-09-24); it found every pair
    afresh from the pages every step until `Live<Contacts>` kept them
    (2026-09-27), which at 10 000 settled took it from about 400 µs to
    about 20 (the log's "Solving across threads", "How it scales").

[^narrow]: *(2026-09-26 to 2026-10-03.)* 2D's SAT and clipping measured 63
    ns a turned pair against 370 for GJK and EPA, so SAT for boxes; one
    point a contact toppled every pyramid. In 3D the cached axis kept piles
    at rest that SAT every step didn't, and GJK and EPA never settled a
    pile; area and line reductions were level, eight points bought
    nothing. Recycling at 0.03 rather than Box3D's 0.05, since at 0.05 a
    carried plank point drifts. The log's "The narrowphase for turned
    shapes", "Rotation in 3D", "Still at rest" and "The 3D narrowphase
    across threads".

[^points]: *(2026-09-26.)* Points inline in 2D's `Manifold` cost a world
    where nothing turns 7-15% of a step; a component written only where
    used, 1.6-2.5%. Matching by id or by the nearest point was within the
    noise of a pile's rest, which moves by a hundred steps with rounding
    alone; ids cost nothing and need no threshold. The log's "Contact
    points".

[^soft]: *(History, 2026-09-26.)* The 2D solver was sequential impulses
    with a split impulse (Bullet's), eight velocity iterations and eight
    of pseudo velocities: it rested at the slop, 0.005 deep, but crept for
    thousands of steps, its pushes along tilted normals sliding bodies
    where no friction acted. Every fix to the correction (friction on the
    pseudo velocities, a decaying correction, Jolt's position iterations)
    failed; soft steps settled. It is kept as `tests/split_impulse.rs`.
    The log's "Settling", and for 3D's stiffness, "The choices, measured"
    (choice 6) and "Still at rest".

[^bounce]: *(History, 2026-09-29.)* Restitution took the closing speed
    with the step's gravity already in it (`Closing::Stepped`, still a
    variant), returning exactly one step of gravity more than a bounce came
    in with: a lossless ball climbed to 1.64 of its drop in 2D and 1.31 in
    3D. Grown over the gap (`Closing::Met`) was unbiased but added energy
    where the gap isn't the fall, and no reference does it. The log's
    "Bounces".

[^lanes]: *(History, 2026-09-27 to 2026-10-03.)* The solver was 2 to 4
    times Box2D's and Rapier's with bodies turning: latency, each contact
    waiting on the one before in pair order. Lanes by level halved it,
    bit for bit (2026-09-27); colors became 2D's default with get-emj.61
    (2026-09-28) and 3D's with get-emj.90 (2026-10-03), level in speed on
    one thread and what threads share. Eight lanes were slower than four
    in both. The log's "The solver's speed", "The solver in lanes" and
    "Colouring the 3D solve".

[^carry]: *(2026-09-28, 2026-10-03.)* 2D weighed A (both from the last
    substep), B and C (both means), by level and colored, against every
    bound of four suites: B colored met all 338, and is what lets the
    parallel solve be the one-thread solve. Under B a column crushed by a
    box 1000 times as heavy throws light boxes through the floor
    (get-emj.58). 3D's colored default kept the mean (Cm) and five known
    misses (get-emj.96). The log's "Why colors let the pyramid fall",
    "What a turning point carries: the decision matrix" and "Colouring the
    3D solve".

[^threads]: *(History, 2026-09-24 to 2026-10-03.)* The first split of
    every stage (2026-09-24) made the ECS's step slower and the arrays'
    faster, mostly from moving data between cores; the colored solve then
    ran across threads inside the solver (`solve_across`, 2026-09-29) on
    a test pool, 4.9 times one thread at 8 on one CCD, and showed
    placement on one CCD was most of it. The threads mod, the shapes'
    dispatch and the splits stage by stage followed (2026-10-03,
    get-znt.34, get-znt.40, get-znt.45, get-emj.101, get-emj.103 to .105),
    and `Workers` went (get-znt.31). The log's "Parallelism", "Parallel
    solving", "Solving across threads" and its last four sections;
    threads.md for what was decided.

[^sleep]: *(History, 2026-09-24 to 2026-09-26.)* Sleeping was first a
    lookup per body in the mod's transient state, then storage
    (2026-09-24), with its gaps closed and on by default (2026-09-25), and
    its record moved from a copy in the mod into the world (`Still`,
    `Slept`, 2026-09-26, get-emj.40), at about 4% of a step while a pile
    falls asleep. Waking every touching island at once was tried and never
    let a pile sleep again. The log's "Sleeping", with its tests and
    mutations.

[^spatial]: *(History, 2026-09-23.)* Built in the spike, with
    `ParamDecl::Group` for it. Until 2026-09-24 spatial queries read a grid
    the step published, empty on the first frame, so a walker set off the
    wrong way; they read the world's storage since. Until 2026-10-04
    `Spatial` was a hand-written `Param` over the group; it is a `Compose`
    since `engine_api` stopped exporting `Param` (get-znt.51).

[^rotation]: *(2026-09-26.)* Rotation was costed at "roughly doubling the
    solver" when it was an open question; a turning contact costs 5-7 times
    a locked one in the solver. A rotation or an angle, turned arms or
    first-order ones, measured within the noise; one relax pass never let
    the big pyramid rest. The log's "Rotation".

[^3d]: *(History, 2026-09-25 to 2026-10-03.)* physics3d began as a
    translation-only spike of plain systems on the ECS harness
    (2026-09-25), turned and became a mod on 2026-09-26, kept its settings
    in static `Mutex`es until then, and bounded turned boxes by a derived
    `Reach` component until storage took two extents. It went into lanes
    (get-emj.52) and colors (get-emj.90) on 2026-10-03, and across threads
    the same day. The log's "3D, translation only (spike)", "Rotation in
    3D", "A mod", "The solver in lanes" and "Colouring the 3D solve".

[^ecs]: *(2026-09-24 to 2026-09-27.)* `:tax` measured the ECS within 5% of
    the arrays settled and 1.28 times falling on the columns' pile, and on
    a real pile level settled and 17% under them at rest; the copies cost
    about 150 µs at 10 000 falling, the re-sort 25 to 126. Solving in place
    went from 801 to 882 µs. The log's "What the ECS costs" and "The real
    pile".

[^quality]: *(History, 2026-09-26 to 2026-09-29.)* Until get-emj.37 the
    tests asked only whether a pile came to rest eventually, and a solver
    that crept for thousands of steps passed them. The log's "Quality as a
    test", "Quality beyond settling" and "Bounces" have the first bounds,
    the first results in every engine and what each planted bug failed.

[^games]: *(History, 2026-09-23.)* Porting the games added the conflict
    rule for queries apart by a table component, sensors that need one
    collider that can move, and queries and bundles of eight terms.

[^cmp-threads]: *(History, 2026-10-03 to 2026-10-04.)* From `ed0b68d` until
    get-emj.113 the 3D comparison timed ours on the process's shared pool
    of eight threads (`Ours::new`), against references on one; every 3D
    table in the log predates that, so is one thread's. 2D's timed row was
    always one thread's, so the 2026-10-04 design review's W4 was wrong
    for 2D. The quality tests and the baseline still run ours on the shared
    pool (`ENGINE_THREADS`), which gives the same bits.
