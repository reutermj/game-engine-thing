# Physics

**Status: built** as `//engine/std/physics`, after a spike (results
[below](#spike-results)), and both games run on it
([The games on it](#the-games-on-it)). What the work showed about the
design's flaws is in the
[physics retrospective](../retrospectives/2026-09-23-physics.md).
The third part of the MVP, after the mod loader and the ECS: 2D rigid-body
physics as an engine mod, `//engine/std/physics`, whose every piece of
state is in the world.

## Goals

Decided 2026-09-23:

- **Our own, with its state in components.** Bodies, colliders and
  everything the solver carries from one step to the next live in the
  world or in the physics mod's state, so reloading the solver swaps code
  under a running simulation like any other mod. A library such as Rapier
  keeps its own world beside ours (body sets, broadphase trees, contact
  caches), which would have to be rebuilt after every reload or pinned in a
  resident mod: the one part of the game that doesn't hot-reload.
- **2D only.** Both games are 2D and there is no renderer yet; 3D waits for
  the renderer spike (get-y5t.8).
- **Deterministic.** The same inputs replay the same simulation, since both
  games run on the lockstep bootstrap and their tests replay recorded
  routes.
- **Proved by the games.** The platformer's tile collision, gravity and
  walker contact, and pong's bounces, move onto it, and a stress demo piles
  up many bodies, as a benchmark and a later test of system and data
  parallelism.

Not goals for the MVP: joints, continuous collision detection, arbitrary
polygons. Rotation, once one, is built: [Rotation](#rotation).

## Components

Declared in the physics mod's interface, so a game depends on it with
`mod_deps = ["//engine/std/physics"]` and the game target loads it.

```rust
component! {
    /// Where a body is: its collider's center. Units are the game's.
    pub struct Position: "physics::Position" { pub x: f32, pub y: f32 }
}

component! {
    /// Units per second. Set by the game at will (a jump, a serve); the
    /// solver changes it on contact.
    pub struct Velocity: "physics::Velocity" { pub x: f32, pub y: f32 }
}

component! {
    /// How a body moves. `kind` is `DYNAMIC` (moved by velocity, gravity
    /// and contacts), `KINEMATIC` (moved by velocity only: pushes, is never
    /// pushed) or `STATIC` (never moves; tiles, walls).
    pub struct Body: "physics::Body" {
        pub kind: u8,
        /// 0 means infinite (a dynamic body that can't be pushed).
        pub inv_mass: f32,
        /// 0 is no bounce, 1 a perfect one.
        pub restitution: f32,
        pub friction: f32,
        pub gravity_scale: f32,
    }
}

component! {
    /// The body's shape, centered on its `Position`: a box (`BOX`, half
    /// extents `hx`, `hy`) or a circle (`CIRCLE`, radius `hx`).
    pub struct Collider: "physics::Collider" {
        pub shape: u8,
        pub hx: f32,
        pub hy: f32,
        /// What this collider is (one bit), and what it collides with.
        pub layer: u32,
        pub mask: u32,
        /// Reports overlaps as `Trigger` events and doesn't push: coins,
        /// spikes, the goal, pong's goal lines.
        pub sensor: bool,
        /// Layers it notices overlapping it, colliding or not: each such
        /// overlap is an `Overlap` (a walker senses the player).
        pub senses: u32,
    }
}

component! {
    /// Which way a body faces, as the cosine and sine of its angle: its
    /// collider turned about its position. None is axis-aligned. Part of
    /// the body's box in storage, with the collider.
    pub struct Rotation: "physics::Rotation" { pub c: f32, pub s: f32 }
}

component! {
    /// Radians a second. A dynamic body with a `Rotation` and a `Spin`
    /// turns, with its shape's inertia at its mass; without a `Spin` it
    /// keeps its rotation (the rotation lock).
    pub struct Spin: "physics::Spin" { pub w: f32 }
}

component! {
    /// The world's gravity, on one entity. None means none: pong has no
    /// entity with it.
    pub struct Gravity: "physics::Gravity" { pub x: f32, pub y: f32 }
}

component! {
    /// How the step solves, on one entity; none is `Tuning::DEFAULT`, 5
    /// substeps. More substeps are stiffer contacts, for a game that stacks
    /// tall (six stand a 20-high turning stack at rest from step 60, not
    /// 580), at about a fifth more solver each ([Still at rest](#still-at-rest)).
    pub struct Tuning: "physics::Tuning" { pub substeps: u32 }
}

component! {
    /// Which sides of a body touched something solid on the last step:
    /// the answer to "can the player jump". Kept up to date on bodies that
    /// have it; a game adds it where it wants to ask.
    pub struct Touching: "physics::Touching" {
        pub below: bool, pub above: bool, pub left: bool, pub right: bool,
    }
}

event! {
    /// Two solid colliders began touching this step. `nx, ny` points from
    /// `a` to `b`; `speed` is how fast they met along it, for game rules
    /// (pong's spin, a stomp).
    pub struct Contact: "physics::Contact" {
        pub a: Entity, pub b: Entity, pub nx: f32, pub ny: f32, pub speed: f32,
    }
}

event! {
    /// A sensor began overlapping a collider its mask includes.
    pub struct Trigger: "physics::Trigger" { pub sensor: Entity, pub other: Entity }
}
```

Contacts and overlaps are entities, kept by physics
([relationships.md](relationships.md)):

```rust
component! {
    /// Two solid colliders touching or about to, `a < b`: an entity from
    /// the step physics finds them until the step it doesn't. An ordered
    /// key, so contacts are stored in pair order.
    pub struct ContactPair: "physics::ContactPair", order = key { pub a: Entity, pub b: Entity }
}
// With it on each contact: `Manifold` (normal from a to b, depth, pressed
// now and the step before, how many points), `Response` (friction,
// restitution, disabled: what the solver does with it this step), `Impulse`
// (for warm starting) and `ContactPoints` (up to two points, each with its
// arms, separation, feature id and last impulses: used when an end turns).

component! {
    /// Two colliders overlapping where one is a sensor or senses the
    /// other: an entity while it lasts, in pair order too.
    pub struct Overlap: "physics::Overlap", order = key { pub a: Entity, pub b: Entity }
}
```

`ContactPair::seen_from(me)` and `Overlap::other(me)` give the other end
(and the sign for the normal) from either side, the part every system
reading a pair would otherwise get wrong once.

Shapes are a `u8` and half extents rather than an enum because a
component's fields must be `FieldType`, and the schema has no enums;
migration still works field by field.

`physics::Position` replaces `transform::Position` and the games' own
coordinates (`Player::x`, `Ball::x`): a body's position is the physics
mod's, and game components keep only game state (coins, deaths, score).

## The step

Physics steps at a fixed 60 Hz, whatever the frame rate: its phase and
`simulate` are fixed-rate ([scheduling.md](scheduling.md#fixed-rates)), so
a frame runs them once per 1/60 s its time accumulates, and each system
reads the step with a `Dt` parameter. The same inputs give the same
simulation at any frame rate, on the same machine.[^onestep]

The step is a pipeline of the physics mod's systems in its own phase,
`physics::step`, after `simulate` and before `late`, so a game sets
velocities (input, AI, rules) in `update` or `simulate` and reacts to
contacts in `late`:

1. **`integrate_velocities`**: gravity into every dynamic body's velocity.
2. **`find_contacts`**: the [broadphase](#broadphase) finds candidate
   pairs, then a narrowphase per pair (box–box, box–circle,
   circle–circle): a normal and a depth; and, where either is turned, up
   to two contact points ([Rotation](#rotation)).
   Pairs filtered by layer and mask; overlaps of sensors (and sensed
   layers) become `Overlap`s, a sensor's new ones a `Trigger` too, and go
   no further. The rest bring the world's contacts in line: this step's
   pairs and the stored contacts are both in pair order, so it's one pass,
   updating contacts that persist in place (their impulses carried), and
   despawning and spawning the rest.
3. **`solve`**: a soft step, as Box2D v3's: five substeps of sequential
   impulses over the contacts, each a pass of soft contacts that push
   penetration out through the velocity, then positions, then two rigid
   passes that take the push's speed back out; warm-started from the
   last step's impulses, with Coulomb friction and restitution (once,
   after the substeps) above a small speed threshold
   ([Settling](#settling)). It also moves the bodies (positions change
   within the substeps, so only the solver knows where they end), stores each contact's impulses and whether it's pressed, updates
   `Touching`, and sends `Contact` for pairs that weren't pressing last
   step. Contacts whose `Response` is disabled aren't solved. Writing
   positions makes its apply node re-sort
   them ([Broadphase](#broadphase)), so every system after the step
   finds bodies where they are. It writes only the positions that
   changed, bit for bit, so a pile at rest costs the re-sort nothing
   ([What the ECS costs](#what-the-ecs-costs)).

Each system is a plain system over queries, so the pipeline is the ECS
doing what it's for, and step 3 gathers bodies into local arrays, solves
there, and writes them back, the shape data parallelism (step 3 of
scheduling) will want. The systems pass contacts along as entities, which
also carry them to the next step for warm starting and for "began
touching", and a reload of physics leaves them in place like any other
entities.

**Pre-solve hooks** are systems a game orders between the two:
`.phase("physics::step").after("physics::find_contacts").before("physics::solve")`.
One sees this step's contacts and overlaps, found from where everything
is now, and may change a contact's `Response` (disable it: a one-way
platform; its restitution: a bounce pad). The platformer's walkers meet
the player this way. What physics found is as of when it found it, so a
system reading it belongs after the system that found it: read a step
later, an overlap from before the player respawned killed it a second
time.

**Determinism.** A fixed step, contacts stored and solved in entity pair
order (so results don't depend on when each began, or on table order,
which parallel spawning would make timing-dependent), no iteration over hash maps, and no
`f32` math that varies by thread. The same build on the same machine
replays exactly; across machines is not promised (fused multiply-add and
libm differ).

## Broadphase

`Position` is a spatial key, with `Collider` and `Rotation` as its
extents ([Rotation](#rotation)), so the ECS
keeps every table of positions in spatial order
([spatial-storage.md](spatial-storage.md)). The broadphase is
`near_pairs` over a query of positions and colliders: pairs whose boxes,
grown by the speculative margin, meet, found page by page. There is no
index to rebuild, and static bodies' pages never change.[^grid] Since
2026-09-27 it is found through `Live<Contacts>`, the live relation the
mod declares (`Contacts: Proximity`, on `Position`), which the world keeps
between steps: the same pairs, found from what moved, with fat boxes
`FAT` (0.02) past each box (live.md; spatial-storage.md, "Keeping
pairs").

## Spatial queries

A spatial query is a `Query` whose entities are found by where their
colliders are: a region query over positions and colliders
(`Query::in_region`), then an exact test of each shape. It takes the same
`Data`, `Filter` and `Changes`, hands the same rows and items to the same
closures, and declares the same footprint, plus reads of `Position` and
`Collider`; so its `Data` can't write those two.

```rust
use physics::{Circle, Ray, Spatial};

// The walkers' ledge check: is there ground just ahead, below?
fn walk(.., mut walkers: Query<(&Position, &mut Velocity), With<Walker>>,
            mut ground: Spatial<(), With<Tile>>) {
    walkers.for_each(|_, (p, mut v)| {
        let ahead = Vec2::new(p.x + v.x.signum() * 0.55, p.y + 0.6);
        if !ground.any_at(ahead) {
            v.x = -v.x;
        }
    });
}

// An explosion: everything with health in the radius is hurt; the dead go.
fn explode(.., mut blasts: EventReader<Blast>, mut hit: Spatial<&mut Health, (), Despawns>) {
    for b in blasts.read() {
        hit.overlapping(Circle::new(b.at, b.radius), |row, health| {
            health.hp -= b.damage;
            if health.hp <= 0.0 {
                row.despawn();
            }
        });
    }
}

// Line of sight: the first solid thing along the ray.
fn look(.., mut blockers: Spatial<&Collider>) {
    let first = blockers.cast(Ray::new(eye, dir, 20.0), |hit, row, collider| {
        (!collider.sensor).then_some((row.entity(), hit.t))
    });
}
```

- **`overlapping(shape, f)`** is `for_each` limited to what overlaps a box
  or circle; **`any_at(point)`** and **`overlapping_point`** are the
  common case of a point. **`cast(ray, f)`** visits hits nearest first,
  passing the hit (distance along the ray, normal); returning `Some`
  stops it, as `single` stops after one.
- **The filter does what layers would.** `Spatial<(), With<Tile>>` sees
  only tiles; layers and masks stay for what collides, which is the
  solver's business.
- **Changes go through rows**, as `Changes` declares, landing at the
  system's apply node.
- **Shapes are where the last writer left them.** Positions are re-sorted
  at the apply node of whatever wrote them, so a system sees every move
  made before it in the plan, including a game's respawn, and none made
  after: the visibility rule for every other change (storage.md).

`Spatial` lives in physics's interface, not in `engine_ecs`: the ECS knows
boxes, not shapes. The query code therefore compiles into each caller, so
changing it is an interface change (a game reload), while the solver stays
an implementation change.

### Parameters made of parameters

A `Spatial` is two parameters: a query, and a query of positions and
colliders. `ParamDecl::Group` is a parameter made of others, whose
footprint is their union and whose conflicts are checked among its
members and against the system's other parameters; any crate can build a
parameter from existing ones this way, and `Spatial` was the first.

## Walkthroughs

### The platformer's player

```rust
// Spawned by the rules, once there's a level:
spawner.spawn((
    Player { coins: 0, deaths: 0, won: false },
    Position { x: info.spawn_x, y: info.spawn_y },
    Velocity::default(),
    Body { kind: DYNAMIC, inv_mass: 1.0, friction: 0.0, gravity_scale: 1.0, ..default() },
    Collider { shape: BOX, hx: 0.4, hy: 0.475, layer: PLAYER, mask: TILES | WALKERS | PICKUPS, ..default() },
    Touching::default(),
));

// `play`, in simulate: run and jump are velocity.
fn play(.., mut players: Query<(&Input, &Touching, &mut Velocity), With<Player>>) {
    players.for_each(|_, (input, touching, mut v)| {
        v.x = input.dir * RUN_SPEED;
        if input.jump && touching.below {
            v.y = -JUMP_SPEED;
        }
    });
}

// `pick_up`, in late: coins, spikes and the goal are sensors.
fn pick_up(.., mut triggers: EventReader<Trigger>, mut coins: Query<&Coin, (), Despawns>, ..) { .. }
```

The level's tiles become static bodies with box colliders (one per cell,
as now), and neither the rules nor the walkers rebuild a tile map every
frame. A walker is a dynamic body; `walkers` turns it at a wall from
`Touching`, and at a ledge with the spatial query in
[Spatial queries](#spatial-queries). It collides with tiles only and
senses the player, so meeting it is an `Overlap`; `walkers::meet`, a
pre-solve hook, reads each one and tells a stomp (the player falling,
feet in its top half) from a touch.

### Pong's ball

The ball is a dynamic circle with restitution 1, friction 0 and gravity
scale 0; the paddles are kinematic boxes moved by velocity from their
intent; the top and bottom walls are static boxes; each goal line is a
sensor. `play` shrinks to: on a `Contact` between the ball and a paddle,
add spin and speed; on a `Trigger` from a goal line, score and serve.

### The stress demo

A walled box that a few hundred circles and boxes are dropped into:
`pile`, a test scene in `engine/std/physics/tests`, which the settling,
determinism and reload tests run and `:bench` times. The engine's demo
(`mods/spawner` and `mods/reporter`, in `//game`) is the same idea at a
couple of dozen bodies. It's also the scene system parallelism and
`par_for_each` will be measured on.

## What the ECS costs

**Verdict (2026-09-24): not a design blocker** for single-threaded 2D
rigid bodies up to 10 000. Against the same step on plain arrays, checked
to be the same computation bit for bit, the ECS is within 5% settled, even
at rest, and 1.28× falling; what's left has known causes that need no
change of design (below), on a real pile as on the columns first measured
(see below). The open risks: parallelism, measured on this machine
(the solver's colored solve is 6 to 9 times today's on one CCD,
[Parallel solving](#parallel-solving); the rest of the step, split as
built, gets slower in the ECS where it gets faster on arrays, for reasons
mostly unbuilt, [Parallelism](#parallelism)); contact churn (the contacts'
re-sort, below); scenes unlike a pile (mixed sizes, bodies carrying many
game components); and tuned engines (the
baseline is our own array code, not Box2D; since measured: level on one
thread, [Against other engines](#against-other-engines)). Further
optimization waits for
a game that needs it.

The 10 000 "pile" in the table below is easier than a pile: dropped 331 a
row, an odd number, each column alternates circles and boxes, and without
rotation a circle on a box stays put, so it settles as 331 columns that
never touch, one contact a body. A real pile of 10 000 (a box 401 wide, 332
a row) has about 14 750 contacts, and the solver takes 1303 µs on it where
it takes 713 on the columns ([Parallel solving](#parallel-solving), timed
alone; in the step, below, 1428 and 760). The same comparison on the real
pile is [after the table's causes](#the-real-pile).

2026-09-24. `./bazel run -c opt //engine/std/physics:tax` runs a pile in
the engine to the frame to measure, copies its whole state (bodies, and
contacts with their impulses) into plain arrays, and runs the same steps
both ways: the mod in the world, and the same step on arrays, with the same
narrowphase and solver, contacts in the same order, bodies as indices. It
asserts they end bit for bit the same (they do), so what differs is the
cost of the world, not different work.

µs per step, `-c opt`, one thread, ECS / arrays, the median of three runs
(pages as blocks of the order and sleeping as storage both merged).
Settled is 400 steps after the drop, when every body still creeps 1e-4 to
1e-2 a step; at rest is 4000, when the pile has stopped bit for bit (1000
bodies by about step 2800, 10 000 by 3000):

| | 1000 settled | 1000 at rest | 10 000 falling | 10 000 settled | 10 000 at rest |
|---|---|---|---|---|---|
| frame | 133 / 123 | 126 / 123 | 721 / 564 | 1338 / 1283 | 1263 / 1266 |
| gravity | 2 / 1 | 2 / 1 | 20 / 10 | 20 / 10 | 21 / 10 |
| gathering colliders | 4 / – | 4 / – | 51 / – | 49 / – | 49 / – |
| broadphase | 15 / 25 | 15 / 26 | 121 / 204 | 161 / 285 | 160 / 285 |
| narrowphase | 9 / 18 | 9 / 18 | 34 / 58 | 100 / 183 | 97 / 180 |
| merging contacts | 3 / 1 | 3 / 1 | 15 / 3 | 34 / 11 | 34 / 12 |
| solve: gathering | 7 / 3 | 7 / 3 | 49 / 21 | 67 / 30 | 67 / 30 |
| solver | 74 / 73 | 73 / 72 | 256 / 252 | 763 / 744 | 750 / 731 |
| writing back | 6 / 2 | 6 / 2 | 48 / 15 | 65 / 20 | 62 / 19 |
| outside the systems (the spatial re-sort) | 13 / – | 7 / – | 126 / – | 78 / – | 25 / – |

The 10 000 settled frame was 2909 µs against the same arrays' 1269; it's
1338, 4% over them, where it was 129%, and at rest the two are the same.
Falling, where bodies change pages every step, it's 28% over (721 against
564), where it was 62% before pages were made blocks of the order.[^tax]
The broadphase is about 10 µs slower than before the two-sided
`near_pairs` that sleeping uses, measured in one session: 150 at 10 000
settled and 115 falling, against 161 and 121. About 4 of it is the walls
on the passive side; the rest isn't found (on the dense layout of
`spatial_bench`, a query's own `near_pairs`, the two are the same, 348
against 351 µs).[^merged]
What took it there:

- **The spatial storage, reworked** (upkeep and broadphase:
  [spatial-storage.md](spatial-storage.md#upkeep-reworked)). The re-sort
  is proportional to what changed, about 7 ns a row written (78 µs while
  the 10 000 creep, 24 at rest), and `solve` writes a position only when
  it changes bit for bit. `near_pairs` walks page lanes
  ([Against sweep and prune](spatial-storage.md#against-sweep-and-prune)),
  and pages split at blocks of the order, which halved the rows falling
  bodies move and made `near_pairs` about half the arrays' sweep and
  prune, which keeps its x order between steps
  ([Pages as blocks](spatial-storage.md#pages-as-blocks-of-the-order)).
- **Walking queries.** A query with only table terms and no sparse filter
  matches every row of every page, so `for_each` takes each term's slice
  once a page and indexes it, with no per-row dispatch on the kind of term,
  and the per-row steps are forced inline (lore: [a query's row cost its
  dispatch](../lore/a-query-row-cost-its-dispatch-not-its-data.md)). A
  walk with no sparse filters also skips the filter check (`Query::passes`).
  A row of three terms went from 5.4 ns to 2.1 on a spatial table and 1.0
  on 256-row pages, where a `Vec` of the values copies at 0.4 to 1.4
  (`./bazel run -c opt //engine/ecs:query_bench`).
- **Page walks** (`for_each_page`, `for_each_ordered_page`) hand a system
  each page's columns whole: `&[T]`, or a `ColumnMut<T>` that stamps a
  row's tick on `set` or the whole page's on `write_all`. Merging and
  writing back contacts write every contact they walk, so they stamp by
  page: the merge went
  from 54 µs to 34 at 10 000 against `for_each_ordered` and `Mut`. Where
  rows are only read, a page walk is no faster than `for_each`, which
  physics uses there.
- **Colliders are gathered by four queries**, one for each of with or
  without a `Body` and a `Velocity`, instead of one and then a second walk
  filling in velocities by entity: queries have no optional terms. The
  gathered item holds only what a pair is tested with, not whole
  `Collider`s and `Body`s (44 µs against 64 writing them out).
- **No test per contact for sleeping.** A resting contact is in a table
  the merge and the solve don't match ([Sleeping](#sleeping)), and the
  narrowphase's tests for resting pairs are split off when nothing sleeps:
  always false then, they cost about 7 µs of 110 at 10 000.[^dead-test]

**Tried, and slower:**

- **Solving in place**, the solver reading and writing `Velocity` in the
  world's columns (as cells, since two ends of a contact can share a page)
  through a vector of references by body, generic over where bodies are
  kept and bit for bit the same: the solver went from 801 to 882 µs, and
  building the references cost more than copying the values (42 µs
  against 27). The solver touches each body a dozen times per contact per
  iteration, so an indirection per touch costs more than one copy in and
  one out, and positions have to be written after anyway. Pages are
  separate allocations, so there's no flat index into the world's memory
  to solve over.
  Taken apart one cause at a time, the same solve bit for bit at 10 000
  settled (`./bazel run -c opt //engine/std/physics:solver_layout`), from
  791 µs on the copy:
  - *Layout* isn't it. SoA over the same flat index is 780, and the holes
    of spatial pages (9.5 rows in 16) cost nothing: flat arrays indexed
    `page * 16 + row` are 788.
  - *Pages* are. Every field in 16-row pages is 909 µs and in 256-row
    pages 887, and 836 to 884 even with each body looked up once per
    contact rather than once per touch.
  - *Derived data* is. An inverse mass worked out from `Body` on every
    touch is 930. In the world's own `Velocity` and `Body` pages it's
    1209, and 870 at best, with inverse masses carried in the constraints
    as Box2D does.
  - *The copy itself* is 14 µs gathering and 4.5 writing velocities back,
    in a walk that writes positions anyway.

  The loop is about 85 instructions per contact per iteration (9 loads and
  4 stores of body fields, 2 dependent divides), so whatever a lookup adds
  shows up in the time nearly in full. So the copy is a transpose into the
  solver's layout, not a patch over a storage flaw. It would take storage
  that is one allocation per column, plus an inverse-mass column, to solve
  in place as fast as on the copy, and that would still save only the
  18 µs of copying.
- **Renumbering bodies in contact order.** On the copy, bodies in entity
  order (the order that contacts, sorted by pair, reach them) are 750 µs
  against 791 in the walk's spatial order. That is the ECS's gap over the
  arrays in the solver row above, since the arrays index bodies by entity.
  Renumbering them in the gather cost as much as it saved: 25 to 29 µs
  more gathering, tried both by sorting the slots and by first touch.
- **Mapping entities to rows by their locations**, instead of a vector by
  entity index built from the walk (`Slots`): 7.1 ns a pair against 1.2,
  the map's building included. A location is two dependent loads into
  segments of atomics, and then a lookup by page.
- **Testing each pair where the merge reaches it**, so a persisting
  contact takes its geometry straight from the narrowphase with no list in
  between: narrowphase and merge together went from 133 to 149 µs.
- **Stamping bodies by page** in writing back: 34 to 32 µs, not worth
  marking static bodies' rows written, and it would defeat writing only
  the positions that change.

**What's left**, of the 10 000 falling frame's 157 µs over the arrays
(55 settled), before what the broadphase and narrowphase save:

1. **Copying in and out, 150 µs** (117 over the arrays falling). Colliders out for detection (which
   makes the narrowphase faster than the arrays', whose bodies are spread
   over four arrays: gathering and narrowphase together are 150 µs against
   183), bodies and contacts into the solver's arrays and back, and the
   entity-to-body map, built twice a step. Arrays skip it because a body's
   index is where it's stored; rows in the world can't be that, since
   spatial order moves them. A walk over bodies still costs about twice a
   `Vec`'s: about 15 ns a page, on spatial pages of 12 rows on average.
2. **The re-sort**, 126 µs falling (re-bounding every body about 72,
   moving rows 20), 78 while bodies creep, 25 at rest.
3. **The merge**, contacts updated in the world rather than a list
   replaced, 24 µs over the arrays; and, falling, spawning 331 contacts
   and sending 331 `Contact` events a step, about 35 µs applying them,
   each a boxed closure in the system's log.

The broadphase is now 56 to 59% of the arrays'. The
solver and the narrowphase cost the same either way, since they're the
same code over the same arrays. The scheduler and frame cost about 7 µs.
Bodies carrying many game components aren't measured.

### The real pile

The table above is of the columns (see the verdict). `:tax` now runs the
real pile beside them, one body more a row (41 and 401 wide), with half
again the contacts, which churn as it creeps. µs per step, ECS / arrays,
medians of three runs:

| | 1000 settled | 10 000 falling | 10 000 settled | 10 000 at rest |
|---|---|---|---|---|
| contacts | 1468 | 10 262 | 14 769 | 14 842 |
| frame | 212 / 219 | 811 / 668 | 2873 / 2882 | 2396 / 2871 |
| broadphase | 24 / 54 | 130 / 297 | 405 / 1052 | 396 / 1064 |
| narrowphase | 15 / 25 | 31 / 42 | 253 / 291 | 238 / 283 |
| merging contacts | 5 / 2 | 16 / 4 | 50 / 37 | 50 / 35 |
| solver | 128 / 130 | 283 / 278 | 1428 / 1416 | 1422 / 1401 |
| outside the systems | 18 | 184 | 476 | 36 |

The verdict holds, and more so: on the real pile the ECS's frame is the
arrays' settled and 17% under them at rest, since the arrays' sweep and
prune suffers most from a pile (its x order is a row of 400 bodies deep).
What's new is outside the systems: 476 µs settled, of which about 360 is
**re-sorting the contacts** (an ordered table,
[relationships.md](relationships.md#ordered-tables)), which rearranges the
whole table, every column, whenever a contact begins or ends, and a
creeping pile begins and ends some every step. At rest nothing does, and
it's 36. Keeping ordered tables in order by splicing in what changed,
rather than rebuilding them, is the fix; it's also what limits the step
across threads (next).

## Parallelism

**Status: measured, not on by default** (2026-09-24, get-emj.30): every
stage but the solver can run split across threads, through the ECS, bit
for bit the same as on one thread; on this machine, at 10 000 bodies, it
makes the ECS's step slower, and the same split on arrays faster. What
serializes each stage is below; most of it is unbuilt rather than the
design, but the largest part is how the step moves its data between cores,
which is the same on arrays. The solver's own parallelism is
[Parallel solving](#parallel-solving), measured on arrays; together they
are get-emj.30's answer.

**What's built.** In `engine_ecs`:

- **`Executor`**, a trait for running tasks on threads, and **`Workers`**, a
  system parameter that declares nothing (as `Dt` doesn't) and hands out
  the executor installed in the world with `World::set_executor`, between
  frames, by whoever owns the threads. Without one, everything runs on the
  system's thread. `Scoped` is an executor that spawns its threads per run.
- **`Query::par_for_each`, `par_for_each_page`, `par_for_each_ordered_page`**:
  the walk cut into chunks of about equal rows, a few per thread, each a
  task holding runs of whole pages of every term (so the split costs the
  calling thread a cut per run, not per page). `make` is called on the
  calling thread for each chunk, with the rows it covers, to make what its
  task fills (see [lore](../lore/memory-a-task-allocates-is-its-threads.md)).
  Rows' changes go into a log per chunk, joined in chunk order: the log
  one thread would have written, so despawns free ids in the same order
  (tested: `page_test`). The ordered walk takes at most one ordered table:
  two are merged by key, in runs within pages, and a page can't be two
  tasks'.
- **`near_pairs_with`**: the active sweep in ranges of pages, the passive
  side in ranges of its runs, each task's pairs already split by range of
  lesser entity index, then each range sorted by a task and turned into
  entities into its piece of the output (tested against one thread and
  brute force: `spatial_test`).
- **The spatial re-sort re-bounds pages in parallel** when the world has an
  executor and the table has about 2000 rows or more; moving rows, splits
  and merges stay on one thread (tested: `spatial_test`).

In the physics mod, each stage takes the parallel path when its `Workers`
has more than one thread and nothing sleeps (waking looks sleeping bodies
up as pairs are found); `physics_test` runs a 600-body pile, sensing and
touching, on four threads against one. `:tax -- parallel` measures both
sides at 1 to 16 threads, and checks every run against one thread's, bit
for bit: positions, velocities, and every contact with its entity and
impulses. The arrays run the same parallel algorithm (the same chunks,
the same range-split sort) with the same executor.

**The numbers.** Ryzen 9 7950X (16 cores on two dies of 8, 32 threads),
a pool of threads kept between runs (`tests/pool.rs`),
µs per step, ECS / arrays, medians of three runs, every run
checked. A run that shared the machine with more than a core of anyone
else's work was taken again (another agent was benchmarking on it).
"Split, one thread" runs the parallel code with every task on the calling
thread: what the split costs by itself. 10 000 bodies, 401 wide, settled:

| stage | 1 | split, one thread | 2 | 4 | 8 | 12 | 16 |
|---|---|---|---|---|---|---|---|
| frame | 2918 / 2889 | 3040 / 2930 | 3554 / 2678 | 3255 / 2286 | 3143 / 2034 | 3208 / 1963 | 3271 / 1963 |
| gravity | 23 / 10 | 28 / 9 | 69 / 9 | 45 / 8 | 37 / 8 | 37 / 8 | 40 / 10 |
| gathering colliders | 53 / – | 86 / – | 165 / – | 134 / – | 122 / – | 130 / – | 138 / – |
| broadphase | 407 / 1060 | 430 / 1110 | 405 / 819 | 312 / 533 | 281 / 331 | 328 / 300 | 347 / 289 |
| narrowphase | 256 / 296 | 258 / 288 | 254 / 271 | 157 / 179 | 121 / 125 | 107 / 111 | 101 / 103 |
| merging contacts | 50 / 37 | 53 / 36 | 75 / 34 | 53 / 25 | 48 / 21 | 52 / 19 | 52 / 21 |
| solve: gathering | 84 / 37 | 119 / 49 | 221 / 108 | 170 / 104 | 160 / 105 | 174 / 106 | 184 / 115 |
| solver (one thread) | 1465 / 1410 | 1472 / 1404 | 1428 / 1391 | 1421 / 1391 | 1433 / 1389 | 1437 / 1385 | 1434 / 1383 |
| writing back | 98 / 38 | 111 / 36 | 153 / 49 | 117 / 44 | 96 / 44 | 98 / 39 | 101 / 41 |
| outside the systems | 483 / – | 483 / – | 793 / – | 857 / – | 862 / – | 871 / – | 875 / – |

Spreads were within 10% but at 2 threads and in the broadphase at 4 (up
to 15%). Everything but the solver: the ECS 1453 µs at one thread and
1840 at 16 (0.8×), the arrays 1479 and 570 (2.6×). The other scenes,
frames only: 400 wide (columns) settled 1353 / 1294 at one thread, best
1459 / 1083 at 8; 401 falling 822 / 672, best 999 / 524 at 4; 400
falling 732 / 569, best 887 / 463 at 8. The best stage speedups the ECS
reached anywhere: narrowphase 2.5×, broadphase 1.5× (1.9× below),
the re-sort's re-bounding 1.4× (columns settled, where nothing else
re-sorts); gravity, gathering, the merge and writing back never above
1.0× (the merge 1.2× and writing back 1.4× with affinity, below).

**What serializes each stage:**

- **Gravity** (20 µs, a pass writing every body's velocity in place) is
  too small to split: handing out a run costs 0.5 µs at 2 threads to 5 at
  16 (the pool, empty tasks), the split 4 to 5 µs more, and the pages then
  have to come back to the core that runs the next stage. The arrays gain
  1.1 to 1.7×. Not a design limit; not worth building either, at this size.
- **Gathering colliders, the solver's gathering, writing back** are
  transposes between the world and arrays for one consumer. Split, each
  chunk's results are joined on the calling thread (a copy), and the data
  crosses cores twice. The arrays' gathering loses the same way (0.3 to
  0.5×): it's the stage's shape, not the ECS. With the solver on one
  thread, they belong on its thread; with a parallel solver, each of its
  threads should gather its own bodies, so the data never crosses.
- **The broadphase**: serial parts are listing the active pages (and each
  row's generation), sorting pages by x, and joining the ranges' pairs;
  the sweep itself splits well. It peaks at 1.5× at 8 threads and falls
  after, as pages and ranges per task shrink. The arrays split a sweep 2.6
  times slower to begin with, so gain more. Unbuilt: keeping the page list
  and generations between steps (they change only as pages do).
- **The narrowphase** is a function of the pair: the same code on both
  sides, 2.5 and 2.9× at 16. What's left is the join of found contacts.
- **The merge** updates contacts in place, page by page; the spawns and
  despawns it would have made are recorded per chunk and made on the
  calling thread in walk order, so contacts get the ids one thread gives
  them (spawns reserve ids as they're made: a spawn from a task would take
  the id its timing gave it). Split, it's never faster, and its cost
  shows up after: see the re-sort.
- **Outside the systems**, the re-sorts. The spatial one re-bounds pages
  in parallel, but moves, splits and merges on one thread, since a row
  moving touches two pages and the order. The contacts' re-sort is one
  thread's whole-table rebuild (see above), and it got 1.8 times slower
  (483 to 857 µs) when the merge before it wrote the contacts' pages from
  other cores; splitting its column gathers made it slower still
  ([lore](../lore/moving-a-stage-to-other-cores-moves-its-data.md)). This
  is the biggest single loss, and it's unbuilt, not inherent: an ordered
  table that splices rather than rebuilds would cost little on any thread.

**The pool here wasn't pinned**, where [Parallel solving](#parallel-solving)
pinned its threads to one CCD and found the scheduler's own placement 2.5
times slower, and idle cores clocked down
([lore](../lore/idle-cores-run-a-parallel-solve-at-half-speed.md)): some of
the loss below may be that, on both sides alike; the arrays' gains are
then an underestimate too. Not measured pinned.

**Where the data lives is most of it.** At 10 000 bodies the step's data
fits in the last core's cache; a split stage pulls its share to other
cores, and the next stage on the calling thread pulls it back. Keeping
chunk `k` on thread `k % n` every step (`STICKY`, with workers spinning
through the solver: `SPIN_US=2000`), one run: the real pile's ECS
broadphase 418 to 224 µs at 8 threads (1.9×), the narrowphase 2.3×, writing
back 1.4×, the frame 2966 to 2995; the arrays' frame 2959 to 1886. An
executor with affinity helps what parallelizes, not the copies.

**Systems at once lose physics nothing.** Physics's three systems are a
chain of data dependencies (gravity writes the velocities finding contacts
reads, which writes the contacts the solver reads, whose writes the re-sort
applies), so running one mod's systems at once would give it nothing even
without the rule against it; its step can only overlap other mods' systems
that touch none of its tables. Splitting its systems further would make
more apply nodes on the same chain.

**Whose threads.** The executor is the host's: a resident scheduler's
(get-znt.5), installed in the world between frames. A task runs only inside
the system that made it, borrowing its stack, and returns before it does,
so at a safe point no mod code is on any thread. A reloadable mod can't
own the threads: a thread it spawns keeps its build mapped
([lore](../lore/a-mod-that-spawns-a-thread-is-never-unmapped.md)), and a
pool's idle loop would be its code. Measured in `physics_test`: host threads
that ran physics's tasks, kept or spawned per run, leave nothing mapped
after a reload; a thread-local with a destructor touched in a task keeps
the build mapped, from any thread, the main one included
([lore](../lore/a-thread-local-a-mod-touches-keeps-its-build-mapped.md)).

**Kept threads, not threads per system call.** Threads spawned for each run
(`std::thread::scope`, `Scoped`) cost 23 µs a run at 2 threads to 182 at
16, against the pool's 0.5 to 5; at 16 the real pile's frame is 5640 /
4011 against 3243 / 1927. A step makes about a dozen runs. The pool keeps
threads, so handing them a closure that borrows the caller's stack needs
unsafe code whose soundness depends on concurrency (the run doesn't return
until no worker is in it), which `engine_ecs` doesn't allow itself
([storage.md](storage.md#where-the-unsafe-is-and-isnt)); it's in
`tests/pool.rs` for the benchmarks. The resident scheduler owning a crate's
pool (rayon's, audited) or this one is get-znt.5's decision. One pool must
serve both systems at once and tasks within a system, or they
oversubscribe the cores: a system on a worker making tasks needs work
stealing, which this pool doesn't do.

**Not built, and what each would take:**

- a spawn from a task: ids reserved when the chunks are joined, in chunk
  order, so they don't depend on timing (storage.md's open question on
  spawned ids);
- events from a task: a writer per chunk, sent in chunk order;
- an ordered table that splices what changed instead of rebuilding;
- an executor with affinity (chunks to the threads that had them) and
  work stealing, owned by the scheduler;
- the colored solver of [Parallel solving](#parallel-solving), with the
  gather and write-back done by its own threads, each gathering what it
  solves, so the bodies don't cross cores between the stages. It would
  make the gather's coloring pass part of the step, as that section says.

## Sleeping

**Status: on by default, at `Sleep::DEFAULT` (slower than 0.05 a second
for 0.5 s), which a `Sleep` entity changes and `Sleep::OFF` (no speed)
turns off; sleeping is storage, and a wake is seen by the world's change
detection, and all it keeps is in the world** (2026-09-24, `sleep.rs`,
get-emj.27; its gaps closed, and on by default, 2026-09-25; kept in the
world, 2026-09-26, get-emj.40).[^sleep-default] An island, dynamic bodies joined
by pressed contacts, whose bodies have all been slower than `Sleep::speed`
for `Sleep::time` falls asleep: its velocities are zeroed and each body
gets `Asleep { island }`, which moves it to a table of its own (with
`Slept`, physics's record of it: "Where it's kept", below), and each
contact neither end of which moves, one of them asleep, gets `Resting`
the same way. The step's queries exclude both (`Without<Asleep>`,
`Without<Resting>`), so a sleeping body is skipped by the table it's in,
not looked up: gravity, gathering, the merge, the solver and writing back
never see it, and change detection leaves its rows alone. A resting
contact is kept as it was, impulses and all, the warm start for when its
ends wake. Falling asleep and waking are structural changes, at the apply
node of the system that decided them, and happen only as islands do.

It changes the simulation (a body stops when a threshold says so, not
when the solver does), which is why the pile, the benchmarks' scene,
turns it off unless asked, and why `:tax` measures it apart, the ECS
alone, with no arrays to agree with (`:tax -- sleeping` runs only its
tables: this one, and the real piles before they're asleep, sleeping on
and off, under "Where it's kept" below). From ten steps after the whole pile is asleep,
against the same pile awake at the same step (speed 0.05, 0.5 s), µs per
step, medians of three runs, on the columns (40 and 400 wide) and on real
piles (41 and 401; see [the scenes](#the-scenes)):

| | 1000, 40 | 1000, 41 | 10 000, 400 | 10 000, 401 |
|---|---|---|---|---|
| asleep by step | 630 | 470 | 550 | 810 |
| frame | 11 / 146 | 11 / 241 | 24 / 1498 | 26 / 3125 |
| gravity (and waking) | 1 / 2 | 1 / 2 | 6 / 22 | 6 / 34 |
| broadphase | 0 / 16 | 0 / 26 | 3 / 173 | 3 / 467 |
| solver | 0 / 81 | 0 / 146 | 0 / 841 | 0 / 1589 |
| outside the systems | 9 / 14 | 9 / 23 | 12 / 94 | 12 / 329 |
| deepest overlap | 0.007 / 0.007 | 0.012 / 0.010 | 0.006 / 0.006 | 0.008 / 0.007 |

(Measured with another agent's fuzzing on the same machine, load about
23: the awake numbers are half again what they were alone; the asleep
ones, and what they're compared with below, were taken interleaved.) The
rest of the stages are 0 or 1 µs asleep. What's left asleep is the look
for what games changed (a look at each sleeping page's ticks and each
table's), and the frame's fixed cost outside the systems. Counting
instead, as this did before 2026-09-25, cost the same frame, 25 and 26 µs
at 10 000, with 8 µs in the look for changes against 6 now: the count of
sleeping bodies was a walk of their pages too.[^sleep-counts] With
physics's record of who's asleep in the world (`Slept`, 2026-09-26), the
asleep frame at 10 000 is 26 µs against the copy's 23, measured together
(see "Where it's kept", below): a look at `Slept`'s tables, and its column.

**The broadphase** is `near_pairs(active, passive, grow)`
([spatial-storage.md](spatial-storage.md#two-sides)): awake colliders that
can move on one side, statics and sleeping bodies on the other. Pairs of
two passive rows aren't looked for, and a passive page no active page is
near isn't looked at: 200 bodies falling onto 10 000 asleep pair in 18 µs,
where all of them awake take 650 (`spatial_bench`). A sleeping collider
the broadphase pairs with an awake one is gathered then, by lookup. Statics
went passive too: static against static never made anything (neither
arrives nor pushes), and static against sleeping would be a resting pair
found again every step. With nothing asleep that costs the broadphase
about 4 µs at 10 000 (the walls tested from their side), part of the 10
µs it is over the one-sided broadphase ([What the ECS
costs](#what-the-ecs-costs)).

**What wakes an island** (the whole island, as it fell asleep), and where
it's seen:

- in `integrate_velocities`, what a game changed since `find_contacts`
  last looked (a game's systems, a pre-solve hook between
  `find_contacts` and the solve, a message):
  - a body's `Velocity`, `Position`, `Collider` or `Body` written: pages
    written since (`for_each_written`). The bodies the last solve put to
    sleep were written by it, after that look; theirs count only if
    they're newer than the solve;
  - a body a game woke (removing its `Asleep`) or made something else
    (removing its body): its `Slept` stays, so there are more `Slept`
    than sleeping bodies (two lengths), and a walk of them finds which,
    each with the island it wakes;
  - a body a game despawned: fewer `Slept` than physics has given and
    not taken off (a count in its state), and a walk of the resting
    contacts finds the ends gone, whose other ends' islands wake. A
    spawn reusing the index is another entity, so it's no end of theirs;
  - `Sleep` despawned, or `physics` sent `wake`: everything.
- in `find_contacts`:
  - a static written or spawned (a spawn writes its values) into or out
    from under sleeping bodies, found by `for_each_written` and a region
    query; a static despawned from under them, or no longer a static
    (`left_since` on the statics' tables, then every resting contact's
    ends checked), whose resting contacts are despawned;
  - a static made to move (a body, a shelf given a velocity): the pair is
    found again, not resting, while its `Resting` contact is there;
  - a contact one of its bodies pressed on ending: what it rested on was
    despawned or moved away.
- after the solve, in `solve`: a moving awake body pressing on one of its
  bodies (a still one waits to fall asleep in its own island; still means
  still for `Sleep::time`, so a body just woken counts as moving), or a
  kinematic body moving into one.

One of these is a count, of `Slept`, which only physics gives: a game
putting a body to sleep as another is despawned doesn't make up for the
one gone, since the new one has `Asleep` and not yet `Slept`. The rest
aren't counts, so nothing is fooled by one thing gone and another come in
the same step (a static despawned and another spawned).

**A wake takes effect in the step that saw it**, if it's seen before the
solve: the bodies leave their sleeping tables at the apply node of the
system that woke them, their resting contacts with them, so the solve has
them movable, their contacts solved from where they were, warm starts and
all, and gravity given them as it was to awake bodies (`fall_woken`): a
pile whose floor goes falls in that step as the same pile awake does.
One seen after the solve (a moving body pressing) takes effect from the
next, as it must.

**A game can put bodies to sleep** by giving them `Asleep` (inserting it,
spawning a body with it, or making a body of an entity that has it), in
an island it numbers. Physics finds them as sleeping bodies without
`Slept`, and takes them as they are (giving each one), before it wakes
anything that step: an island woken there has rows that aren't asleep to
physics either, and taking them for new was a bug the count had (a game
waking one body by removing its `Asleep` put its island back to
sleep).[^sleep-counts] Physics numbers its islands after the greatest it
has seen, so a game's numbers don't collide with ones already made, but
may with later ones.

**A reload doesn't show, and nothing is handed over.** All sleeping
keeps is in the world ("Where it's kept", below) or in the mod's
state, which a reload carries as it is: the ticks it looks from (after
`find_contacts`, and after the solve, so the new build doesn't take the
old one's writes for a game's), the step count `Still::since` is in, the
last island made, and how many `Slept` it has given. `Sleepers`, the
transient part, is only a system's list of what it woke, empty between
systems, and `enough`'s cached answer (the steps `Sleep::time` takes). A
build whose state starts empty (the first, or one reset) counts the
`Slept` in the world and its greatest island, and starts its step count
from the latest `Still::since`; no test makes a reset state, since no
build of physics has another state layout. The games' reload replays (//engine/tests:replay.rs) hold physics
to this: a run reloaded every frame is the run without reloads, bit for
bit; `a_reload_between_a_games_change_and_the_next_step_does_not_show`
holds it to what a game changes between frames (a message), reloaded
before the next step; and
`a_reload_keeps_how_long_awake_bodies_have_been_still` to a pile due to
fall asleep.[^sleep-reload][^sleep-adopt]

**Where it's kept** (2026-09-26, get-emj.40): in the world, as
components, the way the other engines keep it on each body (Box2D's
`sleepTime` on `b2Body`, Rapier's `time_since_can_sleep` on each body's
activation). Besides `Asleep` and `Resting`, a step carries two things:

- *How long each awake body has been still*: `Still { since }`, the step
  it went slower than `Sleep::speed`, on awake dynamic bodies that are
  slower, put on as one slows and taken off as it goes faster or falls
  asleep (and all of them when sleeping is turned off, so turned on
  again it starts from moving). Awake islands aren't kept (they're found
  afresh each step), so there's no island to keep it on. Sparse, since a
  settling pile's bodies cross the threshold all the time (at 10 000 in
  a real pile, about 400 a step while it falls and 1000 while it
  settles), and dense each crossing would move a row between tables; a
  step and not seconds, so it's written only as a body crosses. An
  island falls asleep when its body still for the least time has been
  for as many steps as `Sleep::time` took counted in seconds
  (`Sleepers::enough`: the same step as before, where `time / dt` can be
  one early).
- *Who physics has asleep*: `Slept { island }`, dense, put on beside
  `Asleep` and taken off as physics wakes the body, with a count in the
  state of how many physics has given. `Asleep` is who's asleep now,
  which a game writes; `Slept` is who physics last had asleep, the
  baseline a game's changes are found against (see "What wakes an
  island", above): a body a game woke keeps its `Slept`, and its island
  wakes; one a game put to sleep has `Asleep` and not `Slept`, and is
  taken as the game's. Both are found by the tables a query matches
  (`Asleep` without `Slept`, and more `Slept` than `Asleep`), so at rest
  each is a look per table.

The record of who physics has asleep had three shapes, each with `Still`
as above. µs a step at 10 000 in a real pile (401 wide), sleeping on,
against main (a copy of both in the mod), medians of nine runs
interleaved, 60 steps from the step given (`:tax -- sleeping`):

| the asleep record | falling (from 1) | settling (from 60) | falling asleep (from 120) | asleep, ten steps after all of it |
|---|---|---|---|---|
| main: a copy of both, handed over | 980 | 1786 | 1836 | 23 |
| B1: a copy in the mod, handed over | 992 (+1.2%) | 1822 (+2.0%) | 1920 (+4.6%) | 23 |
| B2: `Slept` in the world | 977 (-0.3%) | 1811 (+1.4%) | 1935 (+5.4%) | 26 |
| B3: none, change detection alone | not built: see below | | | |

B2 as it's kept, against main, medians of 13 runs interleaved (the
spread between runs of one build is about 1%; sleeping off, the two are
within it):

| 10 000, 401 wide | main | now | of it, the `sleeping` stage (main / now) |
|---|---|---|---|
| falling (from step 1) | 993 | 980 (-1.3%) | 104 / 99 |
| settling (from step 60) | 1810 | 1812 (+0.1%) | 132 / 146 |
| falling asleep (from step 120) | 1862 | 1943 (+4.4%) | 143 / 196 |
| asleep, ten steps after all of it | 23 | 26 | 0 / 0 |
| 1000, 41 wide, settling | 182 | 184 (+1.1%) | 14 / 16 |
| 1000, 41 wide, falling asleep | 180 | 184 (+2.2%) | 14 / 19 |

The cost is where bodies fall asleep: each island that does takes
`Slept` and gives up its `Still`s, and the stage walks the `Still` set
to find how long each body has been; a pile falling asleep does that for
most of its bodies within a second, and then costs 3 µs a step more
while it sleeps.

B1 keeps a handoff at every reload, which is what the move was to be rid
of. B2 has none, and costs `Slept`'s insert and remove beside `Asleep`'s
(a dense column, so no extra move) and 3 µs asleep: a look at its tables,
and its column in the sleeping tables. B3 would find a game's changes from
ticks alone, as main did for most of them, and doesn't work, for two
reasons the world can't be asked about:

- *Which rows arrived.* A table keeps the tick a row last arrived in it,
  not which row. A body asleep before it's a body, then made one (its
  `Body` inserted), and a sleeping body a game wrote (its `Body` or
  velocity), are both a sleeping row written since the last step whose
  `Asleep` isn't, and one must be taken as the other: the first woken, or
  the second left asleep.
- *The island of a body a game woke.* Removing `Asleep` takes its island
  with it. The body's resting contacts name its neighbors, whose islands
  are its own, which covers physics's islands (they're joined by pressed
  contacts, all resting once asleep), but not a game's, whose bodies need
  not touch: the rest of such an island stays asleep.

Emulated in B2 (a game's sleeping body taken only if its `Asleep` was
written since the solve, and a woken body's island found only through its
resting contacts), every test passes but two, one for each:
`a_body_put_to_sleep_by_a_game_as_another_is_despawned`, at the body made
one while asleep, and `a_games_island_wakes_as_one`, whose pair apart
stays half asleep. B3 wasn't built further, so it has no measurement; its
looks at rest are main's, which puts it near B1 and 23 µs asleep.

A per-row arrival tick in the ECS would answer the first, at a tick
written on every move between tables; the second needs the island kept
somewhere, which is a record. So B2 it is.

**`Touching`** on a sleeping body is as it fell asleep: its query excludes
sleeping bodies, so it isn't reset. A side touched by something that
arrives while it sleeps (a still body settling against it) isn't marked.

**The ECS it needed** ([change detection](spatial-storage.md#change-detection)):
a spawned value and an inserted one are written, so a walk for what's
written sees what arrived; a table keeps the ticks a row last arrived in
it and last left it, which `arrived_since` and `left_since` read with a
look per table; and `Query::written(e)` is one row's tick. Nothing in
`engine_ecs` knows about sleeping.

What it doesn't do:

- **An island touching a woken one wakes a step later.** Islands form
  separately where a still body falls asleep against one already asleep
  (the real pile of 1000 sleeps as three), and one woken doesn't wake the
  others at once: a woken body counts as moving, so it wakes what it
  presses on after the solve. For that step the contact between them is
  solved against an immovable body, warm started with the weight on
  it.[^touching]
- **A body moving into a sleeping one wakes it after the solve**: for one
  step the sleeper is immovable, as a static would be. Box2D wakes both
  when a contact begins touching, in its collide phase; the same in
  `find_contacts` (a found contact that touches, with an awake end moving
  faster than `Sleep::speed`) is the likely next step.
- **A body a game puts to sleep while it presses on something wakes in
  that step**: its pressed contact isn't found again (a sleeping body
  isn't paired with a static or another sleeping body), and a pressed
  contact ending wakes its ends; only physics marks a contact `Resting`.
  So what a game keeps asleep is what it puts to sleep touching nothing
  (in the air, or before it's a body), as the tests do;
  `a_body_woken_starts_its_time_still_afresh` shows one on the floor
  woken so. Marking the game's pressed contacts resting as it's taken
  would fix it.
- **No body can opt out, and a free body slower than `Sleep::speed`
  stops**: a body drifting at 0.04 a second with nothing touching it
  falls asleep in half a second, and stays where it stopped. Box2D has
  the same threshold (a hundredth of a metre a second) and a per-body
  `allowSleep`; neither game has such a body, and the whole world's is
  `Sleep::OFF`.
- **A game that writes a body's velocity every frame wakes it every time
  it falls asleep**: the platformer's `play` sets the player's `v.x` each
  frame, so the player standing still sleeps one step in 32. Harmless (a
  structural move there and back), and the player responds to input in
  the same frame either way; writing only a changed value would keep it
  asleep.

Its tests (`physics_test`, `pile::`), on the columns (200 bodies in 40
wide) and on a real pile (1000 in 41): falling asleep into tables of
their own, every contact resting, the scene checked to be what it says
(contacts a body); staying put with nothing moved or written; no deeper
asleep than awake at the same step; waking where a body lands, a
kinematic body pushes, a static moves in or is spawned there, or the
floor goes (despawned, swapped for another static in the same step,
lowered, or made a body), with the bottom row falling in that step as the
awake pile's does and no contact left on the floor; a game's velocity or
shape change, in the step after an island fell asleep, and from a
pre-solve hook (`pile_hook`); a body despawned from under others, from a
hook too, and in the step a game puts another to sleep; a game removing
`Asleep` and putting it back; bodies spawned asleep, or asleep before
they're bodies, kept so, and falling in the step a game wakes them;
bodies on a shelf that starts moving rising with it in the step it's
found moving; `Touching` kept; a reload keeping it all asleep; turning it
off; shelves keeping one contact per pair; and a replay with sleeping on,
through six kinds of wake, the same bit for bit twice and at 30 frames a
second. The ECS's side is in `page_test`
(`rows_arriving_are_written_and_rows_leaving_are_seen_to_have_left`). Of
the 27 mutations made to what 2026-09-25 changed, all are caught, three
of them only after a test was added or sharpened: a game giving `Asleep`
to a body the solve doesn't write (a body at rest arrives with its
velocity written by the last solve, which hid it); the bottom row's speed
compared with the awake pile's (a looser bound held without gravity); and
a kick in the step after an island fell asleep (the order of adopting and
waking, which the 1000 pile had caught only by when it happened to fall
asleep). The list is in that commit. Of the first version's 24, three survived and weren't re-run: two
only slower (statics on the active side, resting pairs sent to the
narrowphase), and one whose setup the tests don't make (a body woken in
the step taken as still when marking resting contacts).[^prototype]

Moving the bookkeeping into the world (2026-09-26, get-emj.40) added
`core_test`'s `sleeping::` (how many steps is enough, and islands) and,
in `physics_test`, a body's time still starting afresh when it goes
faster, falls asleep or is put to sleep by a game (asleep exactly 30
steps after it's still at 60 a second), physics numbering its islands
after a game's, a game's island apart waking as one, a despawn in the
step after a game put a body to sleep and in the step after the last
island fell asleep, no contact left with an end gone, a game's wake
taking effect before the solve, and turning sleeping off forgetting
`Still` and `Slept`. Of 37 mutations to the new bookkeeping, 23 are
caught (12 of them only by those tests). Of the 14 left, six change
only what a step costs (a walk that finds nothing, a count too high, a
resolve later in the same system) or heal in the next step (the `wake`
message, or the gate, leaving a `Slept` behind); one, what the
`sleeping` message reports between a game's change and the next step
(it counts `Asleep` with `Slept`); three are covered by
another path in every scene the tests make (the merge waking the ends of
a pressed contact that ended; the other order of a kinematic link; a
game's body dropping `Still` as it's taken, which gravity makes moving
anyway as it wakes); and four need a setup no test makes: a state reset
(`load`'s count from the world, since no build of physics has another
state layout), a game putting a body to sleep from a system after the
solve (a message writes at the solve's tick), a contact found between
an old sleeper and a collider that doesn't move, and the known gap above
of a body woken in the step taken as still when marking resting
contacts. The mutations are in the commit's message.

### The scenes

The pile drops rows of bodies that alternate circles and boxes. At 40 or
400 wide a row holds an odd count, so each column alternates too, and
without rotation a circle on a box stays put: the pile stands in columns
that don't touch, a contact a body. At 41 or 401 a column is all circles
or all boxes, which fall into a real pile, but only once it's tall
enough: 200 or 300 bodies at 41 still stand in columns (1.0 contacts a
body), 500 have 1.1, 1000 have 1.5 (2026-09-25, after 600 steps; see [lore](../lore/a-pile-41-wide-stands-in-columns-until-it-is-tall.md)). The
sleeping tests use 1000 for the real pile, and check the count.

## Parallel solving

**Status: measured, not built** (2026-09-24, get-emj.30). The solver is
one thread, sequential impulses over contacts in pair order. How far it
parallelizes was measured on arrays, outside the mod:
`./bazel run -c opt //engine/std/physics:parallel_solver` takes the solver's
input from piles run in the engine (and a scene of separate stacks built
there), and solves it three ways, each checked bit for bit:

- **Colored**, Box2D v3's graph coloring: contacts colored so no two in a
  color share a body that moves (statics don't count), taking the lowest
  color free in pair order. Colors are solved in turn, each one's contacts
  spread over the threads, with a barrier between them. It solves contacts
  in another order than pair order, so it's a different computation from
  today's, but one fixed by the contacts alone: the same colors, the same
  order within each, the same arithmetic on any thread count. It is bit
  for bit the same on 1 to 32 threads in every scene, and over 4000 whole
  steps of a pile on 1 and 16 threads.
- **Wide**: the same, with each color in batches of 8 contacts laid out
  field by field, solved with AVX2, as Box2D does. The same operations
  (no FMA), so it's the colored solve bit for bit.
- **Islands**: groups of bodies joined by contacts, one thread each. They
  share no moving body, so this is today's computation exactly, and
  checked to be.

**Verdict: coloring works; islands don't; the ECS needs nothing new for
it; what's missing is threads.** On one CCD the colored solve of a real
10 000 pile is 6.1× today's solver on 8 threads, and 8.9× wide on 16 (8
cores and their SMT siblings); a 40 000 pile, 8.6× and 14×. It stops at one
CCD, and it needs workers that stay placed and busy, which is the
scheduler's to provide (get-znt.5).

µs per solve, `-c opt`, the median of three runs of 41 solves each, pinned
to fill one CCD first (so 12 and 16 threads are one CCD with SMT; 32 is
both); speedup against `solver::solve` on one thread:

| threads | real pile 10 000 (14 725 contacts) | real pile 40 000 (68 561) | columns 10 000 (10 000) | stacks 10 × 1000 (10 000) | real pile 1000 (1470) |
|---|---|---|---|---|---|
| serial | 1303 | 7018 | 713 | 1309 | 126 |
| colored, 1 | 1015 (1.28×) | 5167 (1.36×) | 769 (0.93×) | 554 (2.4×) | 98 (1.29×) |
| colored, 2 | 548 (2.4×) | 2574 (2.7×) | 391 (1.8×) | 284 (4.6×) | 67 (1.9×) |
| colored, 4 | 324 (4.0×) | 1478 (4.7×) | 204 (3.5×) | 150 (8.7×) | 55 (2.3×) |
| colored, 8 | 213 (6.1×) | 857 (8.2×) | 114 (6.3×) | 86 (15×) | 51 (2.5×) |
| colored, 16 | 212 (6.2×) | 815 (8.6×) | 114 (6.3×) | 81 (16×) | 59 (2.1×) |
| colored, 32 | 371 (3.5×) | 1043 (6.7×) | 132 (5.4×) | 155 (8.5×) | 106 (1.2×) |
| wide, 1 | 932 (1.40×) | 4775 (1.47×) | 789 (0.90×) | 446 (2.9×) | 92 (1.37×) |
| wide, 8 | 188 (6.9×) | 745 (9.4×) | 117 (6.1×) | 71 (18×) | 46 (2.7×) |
| wide, 16 | 146 (8.9×) | 493 (14×) | 85 (8.4×) | 47 (28×) | 45 (2.8×) |
| islands, 8 | 1415 (0.92×) | 8108 (0.87×) | 1041 (0.68×) | 272 (4.8×) | 141 (0.89×) |
| island batches, 8 | 1454 (0.90×) | 8282 (0.85×) | 131 (5.4×) | 168 (7.8×) | 140 (0.90×) |

Runs agreed to within about 2%, except the 10 000 pile on 2 threads
(543–831 µs over the three).

- **Colors.** A real pile needs 7 or 8 (10 000: 4863, 4741, 2909, 1650,
  495, 64 and 3 contacts; 40 000, 8), columns and stacks 2. None overflowed
  Box2D's 24. Keeping contacts with a static out of color 0, as Box2D
  does, gave the same number of colors as plain greedy coloring and
  nearly the same sizes (only Box2D's rule was timed). The
  split impulse solves only contacts sunk past the slop (half, in a real
  pile), per color the same way.
- **Order alone** is part of the speedup. The loop is latency-bound:
  consecutive contacts that share a body wait on each other's writes, and
  independent ones overlap. A color's contacts are all independent, so
  colored on one thread is 1.28–1.36× serial on real piles and 2.4× on
  stacks, whose pair order walks each stack's chain of contacts. The
  columns are the exception, and are why `:tax`'s pile flattered the
  serial solver: their pair order interleaves 331 columns, which is
  already independent work. The same shows in islands: one at a time,
  each walks its own chain, 3× slower on the columns than serial.
  *Island batches* give each thread a run of islands solved together in
  pair order, which keeps the interleaving and serial's arithmetic.
- **It stops at one CCD.** A barrier costs 0.19 µs on 8 cores of one CCD
  and 0.24 on 16 threads of one, but 0.57 across both, and a solve of a
  real pile is 120 of them (1 + 7 colors warm starting + 8 × 7 + 8 × 7).
  The threads at the boundary between CCDs work longest, since bodies
  written in one color are read in the next by a thread across it. Pinned
  one thread per core across both CCDs, the 10 000 pile's colored solve is
  386 µs on 12 threads and 366 on 16, against 213 on 8; unpinned, the
  scheduler's placement, it's 533 on 8 and 548 on 16. SMT helps the wide
  solve (146 against 188), whose gathers stall, and not the scalar one.
  At 40 000 there's more work per barrier, and 16 threads on one CCD still
  gain.
- **SIMD gains little here**: on one thread, from 3% slower (the columns)
  to 20% faster (the stacks), 8% on real piles. Without rotation a
  contact is about 30 floating-point operations between gathering its two
  bodies and scattering them back, which stay scalar loads and stores
  (98 of them in the batch kernel); Box2D's contacts carry rotation and
  two points, and more arithmetic per gather. The copy into batches is
  another transpose a step (92 µs at 10 000 here, unoptimized).
- **Islands don't help a pile.** A real pile is one island (9992 of
  10 000 bodies, every contact but eight). They pay only for separate
  groups: 1000 stacks, 7.8× on 8 threads as batches. The columns are 331
  islands, and batches give them 5.4× bit for bit the same as today: the
  pile the docs measure could be parallelized without changing a result,
  but no real pile could.
- **Settling.** The same pile from its first step, 4000 steps on arrays,
  solved serially and colored: at step 400 the deepest overlap is 0.016
  serially and 0.025 colored, and 9912 against 9772 bodies are slower than
  0.1; from step 2000 both have all 10 000 resting, and from 3000 both
  the same deepest overlap, 0.0051. Colored converges a little slower, as
  a different order of Gauss-Seidel can; neither pile comes to rest bit
  for bit by step 4000 (171 and 119 bodies still move), and the columns
  do at step 2233 serially and 2546 colored.

**What building it costs a step**, serial, in µs, at 10 000 real / 40 000:
coloring 29 / 150; ordering and copying the contacts into color order
40–45 / 320–350, and back 10–13 / 46–76 (both with the arrays allocated
fresh); islands 85 / 490. At 10 000 that's about 85 µs to save about 1100;
at 40 000 it's as long as the solve on 16 threads, the serial part that
would stop further scaling. Recoloring from scratch each step, only 0.06%
of the contacts that persist change color (538 of 869 704 over 60 steps),
and a coloring that keeps each persisting contact's color needs no more
colors (7).

**What the ECS would provide:**

- **Nothing new, for the solve.** The coloring is a pure function of the
  contacts in pair order and of which bodies move, which the storage
  already gives the gather, so the gather can color each contact and
  write it at its color's place instead of the next, and writing back
  reads it from there. That keeps the property the step has now, that
  results don't depend on when a contact began, and a reload or a replay
  needs no stored colors.
- **Colors in storage, only for scale.** At 40 000 bodies the serial
  coloring and copy are the limit. Then a contact's color could
  persist, as Box2D keeps it: rarely changing, it would be cheap to keep in
  storage (a table per color, each in pair order, so the merge walks all
  of them merged, as `for_each_ordered` merges tables now, and the solve
  walks them in turn), with only beginning contacts colored. The price is
  that colors depend on history, so they'd be state a snapshot must carry,
  and a merge of 8 tables instead of a walk of one. Not built, and not
  needed at 10 000.
- **Threads.** A pool that stays alive across steps: spawning scoped
  threads costs 125 µs for 8 and 265 for 16, more than the solve. It
  must not be the physics mod's (a mod that spawns a thread is never
  unmapped: [lore](../lore/a-mod-that-spawns-a-thread-is-never-unmapped.md)),
  so it's the scheduler's workers (get-znt.5) with a data-parallel job
  (step 3 of [scheduling](scheduling.md#toward-parallelism)): run this
  closure on N workers, with a barrier they share. `engine_ecs::Executor`
  and `Workers` ([Parallelism](#parallelism)) are that job's shape without
  the barrier: the solve needs every worker at once, where they run tasks
  in any order. Those workers need to
  be placed on one CCD (the numbers above are pinned; unpinned they're
  2.5× slower) and kept busy between jobs, or the governor clocks them
  down: an idle core runs a solve at half speed
  ([lore](../lore/idle-cores-run-a-parallel-solve-at-half-speed.md)).

**Inherent, and unbuilt.** Inherent: a colored solve is another
computation from today's, so switching changes every recorded replay once
(and deterministically after); it converges slightly slower; a barrier per
color per iteration, and the cross-CCD traffic of bodies shared between
colors, cap it at one CCD for 10 000 bodies; islands can't split a pile;
and SIMD buys little for a contact without rotation. Unbuilt: the pool and
its job API, placement, a parallel or persistent coloring and copy, wide
batches built straight from the world, and anything smarter across CCDs
(each CCD solving its own half of the bodies, meeting only at the seam).
The prototype stayed in the bench: in the mod it would need threads the
mod can't own, and would change the simulation, where the single-threaded
default must stay bit for bit what it is.

## Against other engines

**Status: measured** (2026-09-25; the solver since replaced, and the
settling gap closed: [Settling](#settling), 2026-09-26). `./bazel run -c opt
//engine/std/physics/compare` runs the same scenes in the physics mod, in
the same step on plain arrays (`tests/arrays.rs`, bit for bit the mod's,
checked on every scene without rain), in **Box2D v3.1.1** and in **Rapier
2D 0.36.0**, one thread each, and prints time per step by stage and how
well each settled. How to run it: [runbook
005](../runbooks/005-compare-physics-with-other-engines.md). Credits and
licenses: [CREDITS.md](../CREDITS.md). The goal is to see the gaps, not to
win: parallel comparisons wait for our physics to run in parallel.

**Verdict: on one thread we are level on time, and ahead where the
problem is easier for us.** At 10 000 bodies the three engines are within
15% of each other on piles and pyramids; we are 20–30% faster in rain,
and nearly twice as fast falling, where our broadphase shines and theirs
re-pair everything that moved. Our time is spent differently: about
750 µs of 3400 on storage upkeep that arrays don't pay, a broadphase that
re-finds every pair even when nothing moved (Box2D pays nothing there),
and a scalar solver in pair order. What they do better is **settle**:
both are at rest in 400 steps where we creep for thousands (and so can't
sleep), and the creep is our split impulse's. What we do better is
**overlap**: at rest ours is the slop, 0.005, where their soft contacts
leave 0.02–0.06 under load, and no setting of theirs changes that.

### How the scenes are matched

`scene.rs` builds every scene for every engine, and the bench checks what
it can:

- Every dynamic body has mass 1 whatever its shape, rotation locked
  (Box2D `fixedRotation`, Rapier `lock_rotations`; the bench asserts no
  body turned, which caught Box2D turning them: see
  [lore](../lore/box2d-set-mass-data-unlocks-a-fixed-rotation.md)), and
  friction and restitution as ours has them, mixed as ours mixes them (the
  least friction, the greatest restitution; Box2D takes the square root of
  the product and Rapier the average by default, so both are given the
  rule). Gravity 20, a step of 1/60, sleeping off everywhere unless
  `SLEEP=1`.
- Each engine at its defaults otherwise: ours 8 iterations and a split
  impulse; Box2D 4 substeps of its soft step (contact hertz 30, damping
  ratio 10, push-out at most 3 a second, SSE2, continuous collision on);
  Rapier 4 solver iterations of its soft solver (block solver, contact
  recycling).
- **Scenes:** a real pile (1000 bodies 41 wide, 10 000 401 wide, circles
  and boxes, every other row shifted half a body, so it is a pile in every
  engine: 1.44–1.57 contacts a body, 1–9 islands); `:tax`'s pile as it
  is (the "columns" case, below); a pyramid of unit boxes (base 20, 210
  boxes; base 100, 5050); rain, circles falling onto the heap 2 or 20 a
  step and removed 480 steps later, 1000 alive 81 wide or 10 000 801 wide.
  Rain is circles because locked boxes land flush on boxes and tower out
  of the box. In the engine, rain is a system spawning through a `Spawner`
  and despawning through rows, as a game would.
- **Timing** is the wall clock around the steps; stages are each engine's
  own counters (ours: the mod's timings; Box2D: `b2Profile`; Rapier:
  `counters`, with its `profiler` feature). Broadphase is Box2D's `pairs`
  plus `refit`, and Rapier's pair update plus its end-of-step tree update
  ([lore](../lore/rapier-times-its-broadphase-at-the-end-of-the-step.md));
  narrowphase is Box2D's `collide`; solver is Box2D's constraint stages
  and Rapier's `solver_time`. "Rest" is the step less those three.
- **Quality** is measured by the bench, the same code for every engine,
  from positions and velocities: overlaps by exact shape, contacts a body
  and islands (touching within 0.01), speeds and kinetic energy, and how
  far bodies moved (per second over the 60 steps timed; for the pyramid,
  from where they started).

### Time

µs per step, the median of 3 runs of 60 steps, `-c opt`, one thread;
runs agreed within 2% except Rapier's 10 000 settled (3565–4062). Each
cell is the step, then broadphase, narrowphase, solver and rest:

| scene | ours (ECS) | ours (arrays) | Box2D | Rapier |
|---|---|---|---|---|
| pile 1000, falling | 142: 20, 7, 57, 58 | 135: 62, 9, 57, 7 | 209: 79, 45, 63, 22 | 232: 62, 29, 98, 44 |
| pile 1000, settled (step 400) | 270: 25, 15, 163, 67 | 242: 47, 24, 161, 10 | 296: 0, 79, 202, 16 | 319: 4, 24, 263, 28 |
| pile 1000, at rest (step 4000) | 227: 24, 14, 154, 35 | 219: 35, 22, 153, 8 | 289: 0, 78, 196, 15 | 311: 4, 24, 257, 26 |
| pile 10 000, falling | 1361: 189, 59, 594, 519 | 1265: 521, 85, 593, 66 | 2518: 1110, 562, 628, 217 | 2374: 655, 295, 1022, 401 |
| pile 10 000, settled | 3370: 450, 245, 1802, 873 | 3315: 1144, 281, 1766, 125 | 3307: 0, 1139, 2017, 151 | 3861: 123, 480, 2967, 292 |
| pile 10 000, at rest | 2718: 437, 232, 1690, 358 | 3252: 1163, 280, 1687, 121 | 3376: 0, 1166, 2059, 152 | 3501: 62, 328, 2819, 293 |
| pyramid 210 | 90: 6, 5, 64, 16 | 80: 8, 7, 63, 3 | 110: 0, 34, 72, 4 | 105: 1, 6, 92, 5 |
| pyramid 5050 | 2814: 181, 116, 2251, 265 | 3174: 682, 167, 2258, 68 | 2846: 0, 1020, 1742, 84 | 2753: 21, 162, 2452, 119 |
| rain 1000 | 342: 41, 27, 157, 117 | 281 + 5: 87, 27, 151, 16 | 386 + 1: 105, 85, 174, 22 | 418 + 1: 101, 59, 196, 61 |
| rain 10 000 | 3601: 500, 283, 1608, 1209 | 2953 + 51: 995, 255, 1555, 148 | 4481 + 16: 1406, 1135, 1726, 214 | 5060 + 7: 1163, 884, 2130, 882 |

"+" is adding the step's raindrops and removing the oldest, outside the
step; the engine's is inside it, at the rain system's apply node.
Contacts solved at 10 000 settled: ours 14 159 pressed (15 864 held),
Box2D 16 230 touching (21 221 held; two points for a box pair), Rapier
16 294.

With each engine's default sleeping (`SLEEP=1`, one run), the 10 000 pile
at step 400: Box2D and Rapier asleep, under 1 µs; ours 3682, since it
still creeps. At step 4000 ours is asleep too, 27 µs (the look for what
games changed). Rain never sleeps (4069 / 4703 / 5214).

### Quality

At the end of the steps timed: deepest overlap / mean overlap, mean
speed, kinetic energy a body, and for the pyramids how far the boxes are
from where they started, mean / most:

| scene | ours | Box2D | Rapier |
|---|---|---|---|
| pile 10 000, settled | 0.016 / 0.006, 0.023, 7.7e-3 | 0.057 / 0.008, 0.0000, 1.9e-10 | 0.054 / 0.008, 0.0005, 1.6e-7 |
| pile 10 000, at rest | 0.005 / 0.005, 0.0000, 1.3e-10 | 0.057 / 0.008, 0.0000, 1.5e-10 | 0.054 / 0.008, 0.0001, 1.9e-8 |
| pyramid 5050, 10 s | 0.013 / 0.008, 0.19, 2.7e-2; 0.27 / 0.74 | 0.019 / 0.009, 0, 4e-11; 0.40 / 0.83 | as Box2D |
| pyramid 5050, 60 s | 0.005 / 0.005, 0, 5e-12; 0.17 / 0.50 | unchanged | unchanged |
| rain 10 000 | 0.38 / 0.009 | 0.66 / 0.018 | 0.54 / 0.016 |
| columns 1000 (`:tax`'s pile), contacts a body, islands | 1.29, 2 | 1.01, 30 | 1.01, 29 |

- **We converge slowly and creep.** At step 400 our 10 000 pile still has
  bodies at up to 8.6 a second and a mean drift of 0.02 a second; theirs
  are still. Our 5050 pyramid is still sinking at 10 s (0.19 a second on
  average), and at rest by 60 s. All three pyramids stand.
- **The creep is the split impulse.** The same step on arrays with its
  pseudo velocities thrown away (`VARIANTS=arrays:nosplit`) settles the
  1000 pile to a fastest body of 0.03 a second against 2.4 (energy 1.0e-4
  against 1.1e-2), and keeps `:tax`'s pile standing in columns as Box2D
  and Rapier do, where with the split impulse it falls into a pile:
  pushing bodies apart along tilted normals moves them sideways, where
  friction doesn't act
  ([lore](../lore/a-pile-41-wide-stands-in-columns-until-it-is-tall.md)).
  Without it nothing corrects overlap (0.24 deep), so dropping it isn't
  the fix.
- **Soft contacts sink under load, and substeps don't help.** Box2D and
  Rapier give the same overlaps to four digits on the pyramid (Rapier's
  solver is the same soft step), 0.019 at its base and 0.057 under a
  pile, with the pyramid's top 0.83 lower. Box2D at 8 substeps is as deep
  (0.060); at 2 it is 0.17 on the pile and 0.075 on the pyramid (its top
  3.3 lower), still at rest.
- **Rain:** we overlap about half as deep on impact (0.38 against 0.66
  and 0.54). Nothing escapes the box in any engine.

**Matched quality.** Neither side is simply cheaper-and-worse, so there is
no single matched point: theirs are at rest and deeper, ours tighter and
restless. The nearest: Box2D at 2 substeps costs 2786 µs on the 10 000
pile (ours 3370) and 1992 on the pyramid (ours 2814), still at rest, at 10
and 6 times our overlap. Rapier at 2 iterations isn't at rest (the pyramid
comes apart), and at 8 costs 5382 for nothing its 4 lack.
(`VARIANTS=box2d:1,box2d:2,box2d:8,rapier:1,rapier:2,rapier:8`, one run.)

### Where the time goes, and what they do differently

- **Broadphase.** Ours finds every pair from the spatial pages every step:
  about 450 µs at 10 000 whether the pile creeps or rests (189 falling,
  where the pages hold fewer pairs). Box2D keeps fat boxes (0.1 of margin)
  in dynamic trees and queries only proxies that left theirs: 0 at rest,
  1110 falling, when every proxy moves. Rapier refits a BVH at the end of
  the step and pairs what moved: 62–123 at rest, 655 falling. The arrays'
  sweep and prune is the worst of all on a pile (1144).
- **Narrowphase.** Ours is the cheapest a pair (a normal and a depth, no
  points: 245 µs for 15 864 pairs). Box2D computes a full manifold for
  every pair whose fat boxes overlap, two clipped points for boxes, with
  rotation math even when rotation is locked (1139 for 21 221). Rapier
  reuses the manifolds of pairs that moved less than 0.05 (480).
- **Solver.** Ours: 8 sequential passes over the contacts in pair order,
  then 8 split-impulse passes over those sunk past the slop, scalar: about
  7 ns a contact a pass. Box2D: contacts graph-colored and solved 4 at a
  time (SSE2) within a color, one solving and one relaxing pass a
  substep, soft contacts, from bodies kept in the solver's layout between
  steps; about our speed on a pile (2017 against 1802) and faster on the
  pyramid (1742 against 2251), where our pair order walks each chain of
  contacts one dependent write after another. That order is our loss: on
  one thread the same solver colored is 1.28× faster and colored and wide
  1.4× ([Parallel solving](#parallel-solving)). Rapier's solver is the
  slowest here (2967).
- **Storage upkeep** is ours alone: the ECS's "rest" is 873 µs at 10 000
  settled against the arrays' 125. Of it, outside the systems 549, most
  of it re-sorting the contacts table as contacts begin and end
  (get-emj.31) and the spatial re-sort of creeping bodies; copying bodies
  and contacts into the solver and back about 250 (the arrays 80). At
  rest, when nothing changes, it is 358, and the ECS beats the arrays,
  whose sweep suffers. Box2D's whole "rest" is 151 (finalizing transforms
  and boxes), Rapier's 292 (moving bodies to their final poses).
- **Structural changes** through a `Spawner` and rows cost little we could
  see: rain at 10 000 is 3601 µs with them in the step, where it was 3644
  with them coming through `WorldMut` from a message each step, which cost
  another 418 µs for 20 spawns and 20 despawns (about 10 µs each; Box2D
  0.4, Rapier 0.2). One entity at a time from outside a frame is slow:
  not what a game does, but tools and tests do.

### What would close the gaps, ranked

Estimated from the numbers above, at 10 000 bodies:

1. **Settle as they do (algorithm).** Done, 2026-09-26:
   [Settling](#settling). The split impulse kept a pile
   moving for thousands of steps, which costs quality (drift, bodies
   thrown at 8 a second) and, with sleeping on, nearly all the time:
   Box2D is asleep at step 400 and we pay 3682 µs a step until the creep
   stops. Candidates, each a different simulation for the bench to judge:
   friction solved on the pseudo velocities too, so the correction can't
   slide bodies; a smaller correction once a contact persists; Box2D's
   soft step with a relax pass, which settles but sinks under load.
2. **Ordered tables that splice (ECS, get-emj.31).** Most of the 549 µs
   outside the systems when contacts churn (841 in rain): 10–20% of the
   step.
3. **An incremental broadphase (algorithm, in storage).** Keep last step's
   pairs for pages whose bodies stayed inside grown boxes, as Box2D's fat
   boxes do: up to about 400 µs (12%) on a pile that creeps or rests, and
   nothing when everything falls, where ours already leads. Done
   2026-09-27 (`Live<Contacts>`; spatial-storage.md, "Keeping pairs").
4. **Colored, wide solving on one thread (algorithm).** 1.28–1.4× the
   solver, about 400–500 µs at 10 000 and more on the pyramid; another
   computation, deterministic, and the start of the parallel solve.
   Done otherwise, 2026-09-27, for turning contacts: in lanes by level,
   the same computation, twice as fast ([The solver's speed](#the-solvers-speed)).
5. **Solver arrays kept between steps (ECS).** Most of the 170 µs the
   copies cost over the arrays; the transpose itself is cheap ([What the
   ECS costs](#what-the-ecs-costs)).
6. **Cheaper structural changes from `WorldMut` (ECS).** 10 µs an entity
   at 10 000, for tools and messages; systems don't pay it.

Nothing here says the ECS is the wrong home: the arrays, with no storage
upkeep at all, are within 2% of the ECS at 10 000 settled and slower at
rest. The gaps that matter are algorithms (1, 3, 4) and one storage cost
already known (2).

### Bringing them in

- **Box2D**, C: an `http_archive` pinned by checksum in `MODULE.bazel`,
  and a `cc_library` over its sources (`box2d.BUILD.bazel`) with the flags
  its CMake build uses on Linux in release (C17, `-O3`,
  `-ffp-contract=off`, SSE2, no validation), built by the hermetic llvm
  toolchain with no trouble. The Rust side calls a C shim
  (`box2d_shim.c`) of scalar functions rather than mirroring Box2D's
  definition structs, so the FFI (`box2d.rs`, the only unsafe code in the
  comparison, and in the bench only) is 9 extern functions over numbers
  and float buffers.
- **Rapier**, Rust: a workspace member `Cargo.toml` pinning
  `rapier2d = "=0.36.0"` with `profiler`, and `Cargo.lock` regenerated
  ([runbook 001](../runbooks/001-regenerate-cargo-lock.md)); `rules_rs`
  built its 60-odd crates unpatched.
- Both are visible to `//engine/std/physics/compare` alone. Box2D is
  pinned to its latest release; Rapier to the version current on the day,
  so a rerun compares the same code.

## Settling

**Status: built** (2026-09-26, get-emj.35). The 2D solver is a soft step
(`solver.rs`), Box2D v3's with our own stiffness and relax count, in
place of the split impulse it had until then.[^split] Piles and pyramids
now come to rest as soon as Box2D's and Rapier's do, sink a quarter as
deep, and fall asleep: the 10 000 pile with sleeping on costs 24 µs a
step at step 400, where it cost 3682.

### What the other engines do (read in their fetched source)

- **Box2D v3.1.1** (`solver.c`, `contact_solver.c`): a soft step. 4
  substeps; each applies gravity, warm starts, solves once with soft
  contacts (`b2MakeSoft`: 30 Hz, damping ratio 10, twice as stiff against
  a static body; push-out capped at 3 u/s), integrates positions, updates
  each contact's separation from how far its bodies moved, and relaxes
  once (rigid, no push). Restitution once after the substeps, from the
  closing speed before them, for contacts that pushed. Friction in both
  passes.
- **Rapier 0.36** (`integration_parameters.rs`,
  `staged_island_solver/worker.rs`): the same soft step. Its
  `num_solver_iterations` (4) are substeps, each with forces (gravity) as
  a per-substep velocity increment, `num_internal_pgs_iterations` (1)
  biased passes, positions, `num_internal_stabilization_iterations` (1)
  unbiased ones; contacts 30 Hz and ζ 10, 60 Hz against a fixed body,
  corrective velocity capped at 3, no slop in the bias, restitution after
  all substeps. One difference from Box2D: friction only in the unbiased
  pass (`friction_in_bias_pass: false`, "load-bearing for tall stacks").
- **Box3D 0.1** (`solver.c`, `types.c`): Box2D's soft step in 3D, 1
  iteration and 1 relax a substep, 30 Hz, ζ 10.
- **Jolt 5.6** (`PhysicsSettings.h`, `ContactConstraintManager.cpp`): no
  soft contacts. 10 velocity iterations of sequential impulses, then 2
  position iterations (non-linear Gauss-Seidel) that move bodies apart
  directly by Baumgarte 0.2 of the penetration past a slop of 0.02, at
  most 0.2 a step, from positions recomputed each iteration. Box2D v2.4 did
  the same (3 position iterations, slop 0.005); not fetched here, so from
  memory.

### The options, measured

`SETTLE=1500` on the comparison (runbook 005): each scene stepped 1500
steps and looked at every 10. "At rest" is every body under 0.05 (the
sleep threshold of ours and Box2D's): the first step it was, and the step
it stayed so from, when they differ. Deepest at step 400. µs is the mean
over the 1500 steps on arrays. Every variant is in `compare/variants.rs`.

| solver | pile 10 000: at rest | fastest at 400 | deepest | µs | pile 1000: at rest | pyramid 5050: at rest | deepest | top moved |
|---|---|---|---|---|---|---|---|---|
| split impulse (before) | 990 / never | 2.77 | 0.019 | 3636 | 560 / 1070 | 500 / 1210 | 0.024 | 0.57 |
| (1) + friction on the pseudo velocities (`split/pf=2`) | never | 5.27 | 0.037 | 5103 | never | 500 / 1210 | 0.024 | 0.57 |
| (2) + correction decaying with contact age (`split/decay=0.1`) | 570 / 1360 | 0.26 | 0.066 | 4183 | 430 / 580 | 500 / 1210 | 0.041 | 0.99 |
| (3) velocity iterations, then Jolt's position iterations (`ngs`) | never | 0.96 | 0.051 | 3841 | 370 / 1490 | 500 / 1210 | 0.024 | 0.99 |
| (4) soft, Rapier's settings (`soft/hz=30/relax=1/sub=4`) | 250 / 570 | 0.013 | 0.096 | 3129 | 250 | 90 / 170 | 0.038 | 1.67 |
| (4') soft, Box2D's settings (the same, `bf=1`) | 210 | 0.001 | 0.092 | 3400 | 280 | 90 / 170 | 0.038 | 1.67 |
| (5) soft, 4 substeps at 60 Hz, 1 relax | 560 / 590 | 0.12 | 0.024 | 3097 | 420 / 470 | 340 / 1110 | 0.009 | 0.42 |
| (5) soft, 4 substeps at 60 Hz, 2 relax | 220 | 0.003 | 0.022 | 3605 | 250 | 120 / 180 | 0.009 | 0.42 |
| **(5) soft, 5 substeps at 75 Hz, 2 relax (chosen)** | **230** | **0.001** | **0.013** | **4037** | **230** | **130 / 150** | **0.006** | **0.27** |
| Box2D | 190 | 0.000 | 0.057 | 3387 | 260 | 40 / 70 | 0.019 | 0.83 |
| Rapier | 200 | 0.005 | 0.054 | 3521 | 160 | 230 / 280 | 0.019 | 0.83 |

Also tried, not in the table: friction on the pseudo velocities limited
by the pseudo impulse alone (`pf=1`: the 10 000 pile never rests); a
correction a quarter as strong for contacts pressed last step
(`persist=0.05`: never rests, 0.056 deep); four position iterations
(never rests); stiffer contacts at 4 substeps (90 and 120 Hz: jitter,
energy 0.2-0.5 a body, never at rest); the soft step with the step's
gravity all in its first substep (never rests: docs/lore); and relaxing
only the impulse added since the warm start, so load isn't soft (never
rests).

What it shows:
- **The creep isn't only the split impulse's.** On the pyramid every
  variant of the split impulse is the same to three digits (fastest 0.70
  at step 400): what creeps there is the velocity solve, 8 iterations of
  Gauss-Seidel over 100 rows of boxes with a step's gravity at once.
  Fixes to the correction (1, 2, 3) can't touch it. Substeps do: each
  holds a substep's gravity, and the stack converges in a fraction of the
  steps.
- **Every fix to the correction fails.** Friction on the pseudo velocities
  (1) needs a load to limit it, and the push's own impulse is too small to
  hold while the real one lets bodies stick and slip. Weaker correction
  (2) settles sooner, but only by sinking 3-4 times deeper. Position
  iterations (3) are the same push, done in positions, and creep the same
  way.
- **Soft steps settle; stiffness decides the depth.** A soft contact sinks
  by load / (mass ω²) (docs/lore, measured), so Box2D's and Rapier's 30
  Hz piles sink 0.05-0.1 and no setting of theirs changes it. The
  stiffest that holds is a quarter of the substep rate, so depth is
  bought with substeps: 0.022 at 4, 0.013 at 5.
- **Two relax passes, not one.** At 60 or 75 Hz one relax pass leaves the
  pile sliding for hundreds of steps (590 on the pile, 1110 on the
  pyramid): the stiffer push leaves more velocity to take out.
- **Friction only in the relax passes** (Rapier's rule) is no worse and
  cheaper: on the 10 000 pile at 60 Hz it rested at 230 either way, and
  it saves a friction row in every pushing pass.

**Why this one.** It's the only family that reaches rest as fast as the
references on every scene, and at 5 substeps it is shallower than the
split impulse was on the pyramid even at rest (0.006 against 0.007 at
step 1500), and on the piles at step 400 (0.013 against 0.019), though
not than its 0.005 once at rest, which a soft contact under a pile's
weight can't reach. 5 substeps rather than Box2D's 4 is the price of that
depth. The pyramid stands better than in either reference (its top 0.27
below where it began, theirs 0.83).

**Time.** One thread, `-c opt`, median of 3, the ECS mod (full tables:
"Against other engines", which still shows the split impulse):

| scene | split impulse | soft step (5 substeps) | Box2D | Rapier |
|---|---|---|---|---|
| pile 10 000, falling | 1361 | 1543 | 2519 | 2373 |
| pile 10 000, settled (step 400) | 3370 (creeping) | 3409 (at rest) | 3392 | 3537 |
| pyramid 5050 | 2814 | 2662 | 2789 | 2749 |
| rain 10 000 | 3601 | 4534 | 4480 | 4970 |
| pile 10 000 at step 400, sleeping on | 3682 | 24 | 0 | 1 |

A pass over the contacts costs about what it did, but there are more
passes: 5 substeps of 3 (15, and 5 warm starts) where there were 8 and up
to 8 more over the sunk contacts. The solver is 18% slower on the pile at
step 400 (2376 µs against 2018 on arrays) and 32% in rain, where contacts
churn and every one is solved; it is faster on the pyramid, and the
settled pile presses fewer contacts (12 690 against 14 159), so the step
is level there. Rain overlaps deeper on impact (0.55 against 0.38; Box2D
0.66, Rapier 0.54), since a deep overlap is pushed out at 3 u/s at most;
its mean overlap is less (0.006 against 0.008).

**What else changed.**
- **Free fall is a little shorter a step.** Gravity is spread over the
  substeps (a fifth of the step's in each, as Box2D and Rapier do), so a
  body falls g h² (1 + 2 + 3 + 4 + 5) a step, not g dt²: 0.0033 less at
  gravity 20. `integrate_velocities` still adds the step's gravity before
  contacts are found (so they're found, and bounce, at the speed they
  meet with); the solver takes it back out and spreads it
  (`SolverBody::gravity`). Without that, a soft step never settles
  (docs/lore).
- **Restitution is kept on speculative contacts** (get-emj.19): a body
  met by a speculative contact bounces at the speed it came in with, not
  what the gap left of it (3 from 10, before). That was pong's stalled
  ball (get-az6): at the AI's paddle, frame 200 of the rally route, it
  now leaves at -17.6 where it stopped dead and crawled along the paddle,
  and off the bottom wall at 5.72 where it left at 1.96
  (`pong_test`'s `the_ai_returns_the_ball_at_full_speed`).
- **Resting contacts touch** rather than sit at the slop: the soft step
  has none, like Box2D's, and a box on the floor sinks 3e-5.
- **Bit for bit** the mod and the arrays still agree on every scene
  without rain (`:tax`, and the comparison).
- **The games' routes**: `platformer_test` and `pong_test` pass
  unchanged. The platformer's reload replay stands still in the corner 2
  frames longer before its jump (33 frames, from 31), since its player
  lands from the drop 0.35 deep and is pushed out at 3 u/s; the jump
  still stomps the walker. The physics tests' 41-wide pile now stands in
  columns, as it does in Box2D and Rapier, so the ones that want a pile
  drop it staggered; a floor that falls jammed between the walls falls
  0.99 in 30 steps where it fell 1.0 free (friction now acts on the push
  that holds it between them).
- `:parallel_solver` and `:solver_layout` measured the split impulse,
  and still do, from a copy of it (`tests/split_impulse.rs`): their
  findings are about that computation. Porting them is work for when the
  soft step is parallelized.

### How it extends to rotation

The soft step is what Box2D v3, Box3D and Rapier all run with rotation,
so the path is known: a contact gains points (anchors on each body, up to
two in 2D, four in 3D), each substep integrates a rotation beside the
position, and a point's separation is updated from both bodies' moves and
turns (Box2D's `b2SolveContact`: the base separation plus the relative
displacement of the rotated anchors, along the normal), with angular
terms in the effective mass and the impulse. Nothing in the passes, the
softness, the relax or the restitution depends on bodies not turning;
they become per point. The split impulse and the position iterations
would extend too (Bullet and Box2D v2.4 turn bodies), but carry their
creep with them, and the position iterations need contact points
recomputed every iteration.

[^rotation]: 2026-09-26: until then an open question, proposed to stay out
    of the MVP: "Without it, boxes don't tip over and the stress demo
    stacks like tetris. Adding it is an angle and angular velocity per
    body, inertia, and contact points instead of a manifold's center:
    roughly doubling the solver." A turning contact costs 5–7 times a
    locked one in the solver, not 2, and a world where nothing turns
    costs what it did.

[^split]: 2026-09-26: until then, sequential impulses with a split
    impulse (Bullet's push velocities): eight velocity iterations, then
    eight passes of pseudo velocities pushing apart contacts sunk past a
    slop of 0.005 by 0.2 of the rest a step. It rested at the slop, 0.005
    deep, but crept for thousands of steps: its pushes along tilted
    normals slid bodies where no friction acted, and its velocity solve,
    with a step's gravity at once, didn't converge on tall stacks. Kept,
    unchanged, as `tests/split_impulse.rs`.

## Rotation

**Status: built** (2026-09-26, get-emj.38). Bodies turn: boxes tip over
edges, stacks and pyramids of boxes stand on two points a contact, discs
roll. A body that doesn't turn is the same computation as before, bit for
bit, and every game's recorded routes pass unchanged, since no game's
bodies turn yet. Each choice below was measured against the others it
could have been, on the same scenes; what Box2D v3.1.1 and Rapier 0.36
(parry2d 0.31) do was read in their fetched source.

What was built:

- **Components.** `Rotation` (the cosine and sine of the angle, as Box2D's
  `b2Rot` and Rapier's unit complex keep it: turning a vector is four
  multiplies, and bounding a box no sine) on any collider, and `Spin`
  (radians a second) on a body. A dynamic body with both turns, with its
  shape's inertia at its mass (a box's `m (w² + h²) / 12`, a disc's `m r² /
  2`; `Collider::inertia_per_mass`); a kinematic one turns at its spin; a
  static with a `Rotation` is a turned plank.
- **Storage.** A body's box is its collider turned: `Position`'s extents
  are `(Collider, Rotation)`, a pair each of which a row may lack
  ([spatial-storage.md](spatial-storage.md#bounds-from-several-components)).
- **The narrowphase.** Where either shape is turned, contacts have points
  (`narrow::collide_turned`): turned boxes by Box2D's separating axis and
  clipping (two points), a turned box and a circle or two circles at one.
  Each point has its arms from both centers, its separation and its
  feature id, the edges it came from. Where neither is turned, the tests
  are the ones before rotation, with no points.
- **Contact points** on each contact as a `ContactPoints` component, and
  warm starting by feature id ([Contact points](#contact-points)).
- **The solver.** A contact is solved at its points when an end turns:
  Box2D's angular terms (each point's effective mass with its arms' cross
  products; impulses turning the bodies; the separation following the arms
  as the bodies turn within the step), in the same soft step, substeps,
  relax passes and speculative margin; restitution per point. A contact
  whose ends don't turn keeps its one row at the normal, and a step with
  no points and nothing spinning runs a solve compiled without them
  (`solver::solve_all::<false>`).
- **Sleeping** goes by a turning body's edge as well as its center (Box2D's
  `maxExtent`), stops its spin as it falls asleep, and wakes it when a game
  writes its rotation or spin.
- **Spatial queries** (`Spatial`) test turned colliders by their turned
  shapes: overlaps by separating axes, rays in the box's frame.

### The rotation lock

A body keeps its rotation by not having a `Spin`: the lock is a component
that's absent, not a flag. Box2D locks by a flag on the body
(`fixedRotation`, which leaves its inverse inertia 0, `body.c`), Rapier
by locked axes (`LockedAxes::ROTATION_LOCKED`, which zero the effective
inverse inertia, `rigid_body_components.rs`); in both, every body carries
an orientation. Measured here as its cost (`TURN=2` in the comparison: every
dynamic body given a `Rotation` and no `Spin`, which is what a flag or an
infinite inertia leaves each body with), ours, µs a step, one thread; the
same quality either way, to three digits (a turned test of a box that
isn't turned rounds a little differently):

| scene | locked by having no `Spin` (and no `Rotation`) | every body with a `Rotation`, locked |
|---|---|---|
| pile 10 000, falling | 1637 (narrowphase 64) | 1815 (167) |
| pile 10 000, settled | 3581 (218) | 3988 (509) |
| pyramid 5050 | 2754 (126) | 3643 (754) |
| rain 10 000 (circles) | 4573 | 4466 |

A body with a rotation is tested as a turned shape, whether it turns or
not, and pays for points it never uses: 11% on a pile, 32% on a pyramid of
boxes. A flag could fast-path a rotation that is still the identity, but
then every body still carries one in storage, which the storage bench
measured at half again as much to write and 15% more to re-sort. By
absence, a world where nothing turns pays nothing, and `Rotation` without
`Spin` is still there for what should face a way and stay so.

**The games.** The platformer's player and walkers have no `Spin`, so they
stay upright; pong's ball has none either, and keeps its behaviour exactly:
it is a circle with no friction, so no contact could turn it anyway (a
normal through its center has no arm, and friction 0 no tangent), and
giving it a spin would change nothing but what the step computes. Their
routes (`platformer_test`, `pong_test`, the reload replays) pass
unchanged.

### The narrowphase for turned shapes

Box2D and Rapier both meet boxes by the separating axis: Box2D's
`b2CollidePolygons` (the face of either box the other is farthest out
along, the other's most opposed edge clipped to that face's sides,
`b2ClipPolygons`), parry's `contact_manifold_cuboid_cuboid` the same for
cuboids. For convex shapes without a routine of their own parry takes the
general route, GJK for the distance (or that they overlap) and EPA for the
depth, then clips the faces the normal picks (`contact_manifold_pfm_pfm`).
Both measured on the same 100 000 pairs of turned boxes, from 0.03 apart
to sunk 0.12 (`./bazel run -c opt //engine/std/physics:narrow_bench`, two
runs):

| way | ns a pair | contacts |
|---|---|---|
| **SAT and clipping, as Box2D** | **63** | 99 840 |
| GJK and EPA, then clipping (allocation-free) | 370 | 99 998 |

They agree on the normal and depth for 99.93% of the pairs both find; GJK
also finds 158 pairs corner to corner within the margin, which our clip
drops as disjoint and Box2D keeps through the closest features of the two
faces (`b2SegmentDistance`, for rounded polygons and the speculative
corner case), a branch ours leaves out: such a pair is found the step it
touches, not the step before. Six times faster, so SAT for boxes. How many
points: two, as Box2D and parry make them for faces. Kept to the deepest
alone (`arrays:rot/deepest=1`), no pile or pyramid comes to rest and every
pyramid topples, as a box on one point rocks (below).

### Contact points

Each contact has a `ContactPoints` component: two points, each its arms
from both centers, its separation and its feature id, and the last solve's
impulses at each, by feature; `Manifold::points` says how many are this
step's and `Manifold::solved` how many the last solve solved at, so a
contact without points never reads its `ContactPoints`. The component is
on every contact (contacts stay one ordered table: a component some have
would split them, as it splits spatial tables) but written only where
there are points. The 3D spike measured four points inline on the
contact at 2% of a step; in 2D, where every game's contacts have none
today, the question was what they cost a world where nothing turns.
`:tax`, µs a step at 10 000 settled (the 401-wide pile), the ECS, each
row beside a run of the build before rotation in the same session:

| points | step: before rotation → with | narrowphase | merge | solve: gather | solver |
|---|---|---|---|---|---|
| inline in `Manifold` and `Impulse` (first cut: every contact 76 bytes more, and the solver’s bodies 44 bytes) | 1547 → 1776 (+15%) | 92 → 162 | 35 → 42 | 80 → 130 | 1021 → 1112 |
| inline, the solver’s bodies and contacts made small again | 1547 → 1658 (+7%) | 92 → 108 | 35 → 42 | 80 → 120 | 1021 → 1063 |
| a `ContactPoints` component, written only where used | 1547 → 1572 (+1.6%) | 92 → 99 | 35 → 40 | 80 → 91 | 1021 → 1022 |
| **the same, the solve compiled without points where there are none** | **1519 → 1557 (+2.5%)** | **93 → 105** | **34 → 40** | **76 → 92** | **1002 → 998** |

Bit for bit the same throughout; across `:tax`’s piles the last is 1–3%
slower (the 1000 pile 153 → 158, falling at 10 000 845 → 863). What’s left
is a turned-or-not test per pair (the narrowphase), the contact’s points’
index in what the merge carries, and a fifth column the solve’s gather
walks. What the first cut showed is that a body’s angular state costs a
world where nothing turns wherever it rides along: a `SolverBody` with
angular fields (44 bytes, not 28) cost the solver 4%, so turning bodies
are a list of their own (`solver::Spinning`), and a step with no points
and nothing spinning runs a solve compiled without them
(`solve_all::<false>`). Points as entities (four rows a contact in another
ordered table) the 3D spike already measured as four times the churn; not
tried again. In the comparison, locked, the ECS is within 2% of before on
every pile and pyramid, and 3% slower in rain, where contacts begin and
end every step and each carries its `ContactPoints` column through the
contacts’ re-sort.

**Warm starting** (read in the fetched source): Box2D matches this step's
points to last step's by feature id (`b2UpdateContact`), and a new point
starts from nothing; parry does the same (`ContactManifold::match_contacts`)
and has matching by position too (`match_contacts_using_positions`); and
nothing at all is the third choice. On the arrays, bodies turning, with
`SETTLE=1500` (the step every body, edges included, was slower than 0.05
from; deepest overlap at step 400):

| warm start | pile 1000: at rest from | deepest at 400 | pyramid 5050: at rest from | top moved | pile 10 000: at rest from | deepest at 400 |
|---|---|---|---|---|---|---|
| **by feature id, as Box2D** | **290 / 180** | **0.020 / 0.019** | **440 / 440** | **0.26** | **390 / 550** | **0.024 / 0.025** |
| by the nearest last point, within 0.1 | 280 / 410 | 0.024 / 0.031 | 440 / 440 | 0.26 | 340 / 620 | 0.024 / 0.024 |
| by feature id, a new feature's by the nearest | – / 320 | – / 0.020 | – / 440 | 0.26 | – / 290 | – / 0.027 |
| not at all | never | 0.10 | never: falls apart (top 15.9 lower) | | never | 0.12 |
| Box2D (ids) | 240 | 0.12 | 160 | 1.46 | 1160 | 0.11 |
| Rapier (ids) | 230 | 0.10 | 1100 | 1.50 | 1200 | 0.087 |

Two runs a cell, the same computation but for the order of a few
multiplies (the second after keeping each point's cross products rather
than computing them again in each pass): a pile's step to rest moves by
a hundred or more with rounding alone, so **only what clears that decides**.
Without warm starting nothing comes to rest, and the big pyramid falls
apart: a contact's impulse has to carry its load from one step to the
next, and two points of a box on a box can't rebuild it in a step's
passes. Between ids and positions nothing clears the noise; feature ids,
then, as Box2D and parry have them: cheaper (no distances, no threshold
that depends on the bodies' size), and a match that depends on which
features the points are, not where.


### Rotation in the soft step

Box2D carries a body's turn through the substeps as a rotation stepped by
the first order and normalized (`b2IntegrateRotation`), and a point's
separation by its arms turned with the bodies (`b2SolveContact`). Rapier
integrates the angular velocity as a rotation too, and updates a point's
separation from its local points moved by the bodies' poses (`update`, in
`contact_with_coulomb_friction.rs`): the arms turned, as Box2D. Both relax
once a substep, with friction in it, and bounce once after. On the arrays,
bodies turning, `SETTLE=1500`, `arrays:rot/<key>=<value>`
(`compare/variants.rs`), µs a step over the whole run; two runs where
there are two, as above:

| variant | pile 1000: at rest from | µs | pyramid 5050: at rest from | pile 10 000: at rest from | µs |
|---|---|---|---|---|---|
| **as built: a rotation, arms turned, 2 relax, 2 points** | **290 / 180** | **1026 / 909** | **440 / 440** | **390 / 550** | **12 111 / 10 933** |
| an angle, its sine and cosine each substep (`int=1`) | 220 / 230 | 1031 / 906 | 440 / 440 | 300 / 330 | 12 119 / 10 979 |
| arms moved to first order, `r + θ × r` (`sep=1`) | 400 | 1019 | 440 | 350 | 12 026 |
| arms fixed, turning ignored (`sep=2`) | 330 | 1034 | 440 | 1080 | 11 947 |
| 1 relax pass (`relax=1`) | 310 | 746 | never | 490 | 9252 |
| 3 relax passes (`relax=3`) | 240 | 1285 | 510 | 260 | 15 131 |
| one point a contact, the deepest (`deepest=1`) | never | | never: topples | never | |
| Box2D | 240 | 372 | 160 | 1160 | 4371 |
| Rapier | 230 | 377 | 1100 | 1200 | 5200 |

- **Two points, two relax passes, the arms turned.** One point topples
  every pyramid; one relax pass never lets the big pyramid rest, as
  without rotation ([Settling](#settling)); three cost a quarter more for
  nothing that clears the noise; ignoring the arms' turn takes the big
  pile three times as long.
- **A rotation or an angle; turned arms or first-order ones**: within the
  noise, at the same cost. Box2D's rotation (no sine or cosine) and its
  turned arms were kept. The first-order arm has one thing for it: a
  rolling disc's turned arm lifts off the ground within the step and
  leaves the disc's `Velocity` a quarter of a step's gravity downward
  though it doesn't sink, which the first-order arm doesn't
  (`a_rolling_disc_leaves_a_step_falling_unless_its_arm_is_followed_to_first_order`;
  [lore](../lore/a-rolling-disc-leaves-each-step-falling-toward-the-ground-it-rolls-on.md)).
- **Restitution per point**, once, from each point's closing speed before
  the step, for points that pushed (`b2ApplyRestitution`); a contact of
  bodies that don't turn bounces as before.
- **The speculative margin** needed nothing new: a point's separation is
  tracked through the substeps, and a gap may close no faster than the
  substep allows, per point. Corner-to-corner pairs just apart are the
  one case Box2D keeps and ours drops (the narrowphase, above).
- **Where the time goes.** A turning contact costs 5–7 times one that
  doesn't (two points, each with angular terms, in 20 passes a step): the
  pile of 1000 solves in 909 µs turning against about 100 locked. Box2D
  does the same work in 8 passes and 4 warm starts of SIMD over colored
  contacts; Rapier in 4 substeps of one biased and one unbiased pass. The
  colored, wide solve that was measured at 1.3–1.4 times the scalar one
  without rotation ([Parallel solving](#parallel-solving)) is where the
  gap closes, more so with rotation's longer rows. (It closed by level
  rather than by color, 2026-09-27: [The solver's speed](#the-solvers-speed).)

### Against other engines, bodies turning

The comparison ([Against other engines](#against-other-engines), runbook
005) now runs every case twice: rotation locked everywhere, as before, and
bodies turning (Box2D and Rapier unlocked, each dynamic body given its
shape's inertia at mass 1; ours with a `Rotation` and a `Spin`). The same
scenes, rain still circles (it towered as locked boxes; turning, it could
be boxes). One thread, sleeping off, `-c opt`, the median of 3 runs
(2026-09-26); each cell the step, then broadphase, narrowphase, solver
and the rest, µs:

| scene, turning | ours (ECS) | ours (arrays) | Box2D | Rapier |
|---|---|---|---|---|
| pile 1000, falling | 293: 21, 18, 162, 92 | 290: 87, 24, 161, 17 | 219: 91, 45, 61, 22 | 235: 59, 38, 91, 47 |
| pile 1000, settled | 958: 32, 83, 734, 109 | 955: 66, 113, 728, 47 | 379: 0, 106, 257, 16 | 388: 4, 32, 325, 27 |
| pile 10 000, falling | 3028: 209, 185, 1812, 822 | 3036: 814, 254, 1795, 174 | 2756: 1272, 600, 644, 241 | 2578: 682, 426, 1010, 461 |
| pile 10 000, settled | 10 681: 519, 901, 8215, 1047 | 11 540: 1666, 1234, 8149, 491 | 4392: 0, 1616, 2626, 150 | 4900: 89, 531, 3999, 281 |
| pyramid 210 | 372: 7, 33, 296, 37 | 360: 12, 42, 294, 12 | 111: 0, 35, 72, 4 | 106: 1, 6, 94, 5 |
| pyramid 5050 | 8875: 184, 801, 7324, 565 | 9541: 905, 1050, 7305, 280 | 2925: 0, 1116, 1725, 84 | 2802: 92, 166, 2425, 118 |
| rain 1000 | 786: 48, 36, 511, 191 | 700: 103, 59, 496, 42 | 445: 126, 97, 199, 23 | 538: 124, 130, 219, 64 |
| rain 10 000 | 8487: 528, 386, 5624, 1949 | 7654: 1157, 599, 5466, 431 | 5110: 1587, 1287, 2010, 227 | 6877: 1507, 1867, 2511, 992 |

And how they stood, at the end of the steps timed: deepest / mean overlap,
mean speed, kinetic energy a body (the turning part too), and the most any
box is tilted from resting on a face:

| scene, turning | ours | Box2D | Rapier |
|---|---|---|---|
| pile 10 000, settled | 0.023 / 0.0024, 0.0004, 1.8e-7, 45° | 0.11 / 0.015, 0.0002, 1.3e-5, 45° | 0.087 / 0.015, 0.0002, 5.7e-8, 45° |
| pyramid 5050 | 0.0059 / 0.0025, 0.002, 3.3e-6, 0.3° | 0.041 / 0.014, 0.0005, 2.1e-7, 1.5° | 0.035 / 0.015, 0.016, 2.2e-4, 1.8° |
| pyramid 5050, a minute on | 0.0059 / 0.0025, 0, 3.4e-10, 0.3° | 0.041 / 0.014, 0, 6.5e-10, 1.5° | 0.035 / 0.015, 0.0003, 7.4e-8, 1.8° |
| rain 10 000 | 0.47 / 0.0039 | 0.53 / 0.011 | 0.51 / 0.011 |

(A pile is 45° tilted somewhere in every engine: a box wedged between
circles. The pyramids stand in all three.)

- **The quality holds.** Every engine's turning piles and pyramids come to
  rest (settling tables above: ours at 180–550 steps on the piles against
  Box2D's 240 and 1160 and Rapier's 230 and 1200, at 440 on the big
  pyramid against 160 and 1100). Ours sinks a fifth as deep, and its big
  pyramid leans 0.3° where theirs lean 1.5–1.8°, as its stiffer contacts
  did locked.
- **The time doesn't: the solver is 2–4 times theirs** where contacts
  press (8215 µs against 2626 and 3999 on the settled pile, 7324 against
  1725 and 2425 on the pyramid), and the step 2.2–3 times. Falling at
  10 000, where few contacts press and our broadphase leads, ours is 10–17%
  slower (at 1000, 25–34%). The
  gap is the one [Rotation in the soft step](#rotation-in-the-soft-step)
  names: a turning contact is two points of angular terms in 20 passes a
  step, scalar and in pair order, where Box2D's are 12 passes of SSE2 over
  colored contacts and Rapier's 8. Since halved, the same computation:
  [The solver's speed](#the-solvers-speed).
- **The narrowphase is in their range**: 901 µs on the settled pile against
  Box2D's 1616 (it clips every pair whose fat boxes overlap) and Rapier's
  531 (it reuses manifolds of pairs that barely moved). The broadphase is
  as it was, since boxes are boxes to it.
- **Storage upkeep grows with what turns**: the ECS's rest over the arrays'
  is 556 µs on the settled pile (1047 against 491), where locked it was
  307, the rotations and spins now written and re-bounded each step.
- **Locked, little moved**: against the build before rotation in the same
  session, the ECS's step is within 2% on every pile and pyramid (the
  settled 10 000 pile 3572 → 3484, the 5050 pyramid 2743 → 2741) and 3%
  slower in rain (4476 → 4613: contacts' churn carries `ContactPoints`);
  `:tax` 1–3% slower, bit for bit.


### What the 3D spike predicted, and what 3D needs

The spike ([3D, translation only](#3d-translation-only-spike)) predicted
four things of rotation; in 2D:

- **A spatial key that takes the transform, or extents that are a
  tuple:** extents a pair, measured against a pose key, an angle and
  conservative bounds (spatial-storage.md). 3D wants the same:
  `impl SpatialKey<3> for Position { type Extent = (Collider,
  Orientation); }`, the box of a turned box `|R| h` (nine products, where
  2D's is four), and rows without an orientation bounded as now. A
  sibling spike doing 3D rotation without changing `engine_ecs` would need
  a pose key or conservative bounds, which in 2D cost every row (a pose's
  columns, 50% more to write and 15% to re-sort) or every pair (7 times
  the pairs); the pair lands in `engine_ecs` as generic over dimensions as
  the rest, with `type Extent = Collider` unchanged for keys with one.
- **The solver body growing from 7 floats to about 20, and the copies
  tripling:** avoided, for bodies that don't turn. A body's angular state
  is a list beside the bodies (`Spinning`), dense only inside the solve
  and only if something spins; a 44-byte body had cost the solver 4%. In 3D
  a spinning body adds a world inverse inertia (6 floats) and a
  quaternion, which the same split keeps off the bodies that don't.
- **Per-point feature ids and a clipping narrowphase, several times a
  point-less pair:** SAT and clipping at 63 ns a turned pair, against 370
  for GJK and EPA (and about 10 for an axis-aligned pair). Points in a
  component of their own, written only where used (`ContactPoints`); in 3D
  four, as Box3D and Jolt cap them.
- **Islands and sleeping mattering more:** a turning pile solves in 3–4
  times the time a locked one does, so its sleeping is worth that much
  more; it falls asleep as soon (the pile test).

What the 3D rotation spike (branch `physics3d-rotation`, which left
`engine_ecs` alone and keeps a derived `Reach`, the turned box, as its
key's extent, rewritten as a box turns) asks of the storage, and how the
pair of extents answers:

1. **Bounds from more than one extent, or bounds a system writes
   directly.** Both: `type Extent = (Collider, Orientation)` bounds from
   the key and both, with no derived component to keep in step; and a key
   whose one extent is a box a system writes (`Reach`) still works, since
   `type Extent = Collider` is the pair's first case unchanged.
2. **Turning alone must re-bound a row.** Writing either extent marks the
   row as writing the key does (`engine_ecs`'s
   `a_box_from_two_extents_re_sorts_when_either_is_written`, with its
   three mutations caught: a second extent's ticks unchecked, its name not
   registered as moving rows, its value not passed to `bounds`).
3. **Never sphere bounds for statics.** None here: a turned static is
   bounded as its turned box, one that isn't by its box. The one sphere is
   a look for what to wake around a static a game moved (`any_way`), a
   region query, not what storage keeps.
4. **Fat bounds or kept pairs, later (get-emj.36).** Not precluded:
   `bounds` returns whatever box it likes, a grown one included, and the
   order, lanes and broadphase take it as they take any.
5. **Bigger tuples.** Not needed in 2D: a turning body is six components
   (`Position`, `Velocity`, `Body`, `Collider`, `Rotation`, `Spin`), under
   bundles' and query data's eight; the solve's turning query has four
   terms and the contacts' five. The limit rotation did meet is a system
   parameter group's four (`integrate_velocities` regrouped its queries to
   add the sleeping bodies' rotations). A 3D solve query of eight is at
   the data limit; the pair of extents adds nothing to a body's count
   over a derived `Reach`, and one fewer component to keep in step.


## What changes elsewhere

- `mods/transform` and the old `mods/physics` demo went;
  `//engine/std/physics` has the name `physics`, and `spawner` and
  `reporter` use it. The `mod_deps` examples in mod-deps.md still hold,
  since `physics` declares `Velocity` in its interface.
- The recorded routes in `platformer_test` and `pong_test` pass
  unchanged (see below).

## 3D, translation only (spike)

**Status: a spike** (2026-09-25, branch `spike/physics3d`): what 3D asks of
the storage core, before more is built on 2D alone. Rotation is its own
investigation, so bodies have a 3D position and velocity and no
orientation. `//engine/std/physics3d` is the 2D step's shape in 3D, as plain
systems on the ECS harness rather than a mod: `Position` a 3D spatial key
([spatial-storage.md](spatial-storage.md#in-3d)) with `Collider` (a sphere or
an axis-aligned box) as its extent, statics in tables of their own on the
broadphase's passive side, contacts as entities in an ordered table by pair
(`ContactPair`, `Manifold`, `Impulse`), and the 2D solver (sequential
impulses, 8 iterations, warm-started, a split impulse for penetration,
speculative contacts within 0.05) in 3D. Friction is the one thing 3D
changes in the solver: the tangent is a plane, so the friction impulse is a
vector in it, clamped to a disc, and kept as a world vector that warm
starting projects onto the next step's plane, so no tangent basis has to
stay put. Left out: layers, sensors, kinematic bodies, sleeping, events,
parallelism.

**Against Rapier 3D, Jolt and Box3D** (`./bazel run -c opt
//bench/physics3d:bench`; the harness, scenes and how each engine is
brought in are in `bench/physics3d`, credits in [CREDITS.md](../CREDITS.md)).
One thread, rotations locked, sleep off, every engine at its own defaults
(Rapier 0.36: 4 iterations; Jolt 5.6: 10 velocity and 2 position steps;
Box3D 0.1: 4 substeps; ours 8 + 8), ms per step over the whole run, 10 000
bodies a single run and 1000 the median of three:

| | spheres 1k | spheres 10k | boxes 1k | boxes 10k | rain 1k | rain 10k |
|---|---|---|---|---|---|---|
| ours | 0.38 | 6.1 | 0.36 | 4.9 | 0.26 | 3.6 |
| Rapier | 0.61 | 11.0 | 0.86 | 13.7 | 0.54 | 8.4 |
| Jolt | 1.18 | 16.4 | 1.12 | 13.1 | 0.73 | 9.3 |
| Box3D | 1.01 | 12.2 | 1.02 | 11.5 | 0.64 | 7.4 |

Quality, the harness's own geometry over every engine's positions: every
pile is a pile (about 0.95 of supported bodies rest off-center on what's
below them; 3.3 to 4.1 bodies touched each), nothing escapes, and ours
settles (in 174 to 612 steps) with penetration at its slop, 0.005 at most;
Rapier's and Box3D's box piles never settle at their defaults (they breathe;
see the bench's lore), and their sphere piles reach 0.10 and 0.16 deep at
10 000. With every engine at 8 iterations ours is 2.4 to 4 times faster.

**What the numbers say, and don't.** Ours is 1.6 to 2.8 times faster, and
nearly all of that is the solver: it has no angular terms, no contact
points and one constraint per pair, where the others run their general
solvers, a constraint per contact point with angular terms the locks only
zero (a translation-only step is simply less work, so this is no verdict
on the solvers). The stages that are the storage core's say the
opposite. At 10 000, µs per step:

| | broadphase: ours / Rapier / Box3D | narrowphase: ours / Rapier / Box3D | ours: copies in and out | ours: re-sorts |
|---|---|---|---|---|
| spheres | 1560 / 155 / 326 | 555 / 2413 / 2638 | 210 | 154 |
| boxes | 1129 / 393 / 437 | 336 / 966 / 1979 | 194 | 153 |
| rain | 977 / 277 / 516 | 371 / 1111 / 1282 | 178 | 254 |

The broadphase is 3 to 10 times Rapier's and 2 to 5 times Box3D's, and
the largest thing in our step that isn't the solver. The others keep their
pairs from step to step (Box3D a tree of fattened boxes, re-queried only
for shapes that left theirs, `broad_phase.c`'s move array; Rapier a BVH whose
pairs persist and are re-examined only beside colliders whose boxes
changed, `broad_phase_bvh`, both read in the fetched source), where `near_pairs` finds
every pair afresh from pages every step: in 3D that is 85 000 box pairs for
25 000 contacts among 10 000 spheres, about 18 ns each. Pages of 32 rows
cut it 13 to 15% ([spatial-storage.md](spatial-storage.md#in-3d)); keeping
pairs between pages that haven't changed, open since the 2D broadphase was
reworked, is what 3D makes necessary. Copies in and out of the solver and
the re-sorts cost what they do in 2D per body. Rapier's and Box3D's
narrowphase numbers include contact manifolds with points; ours has none.

**Contacts with several points.** Translation only, a contact needs no
points (an impulse through any point moves a body the same), so a box on
a box is one normal and a depth. Rotation needs them: a box resting on a
box is four points, each with its own impulses to warm-start, matched from
step to step by feature. Measured as storage, four points inline on the
contact (`[f32; 12]` and a count on `Manifold`, four normal impulses on
`Impulse`; 10 000 boxes, 67 000 contacts) cost the merge and the solver's
gather about 5.7 ns a contact a step (0.38 ms of 17.8, 2%), and the
contacts' re-sort, falling, 40 µs more. Inline fixed arrays are what the
schema already has; a `Vec` per contact would be a heap allocation a
contact and a pointer chase in the gather, and points as entities would
be four rows a contact in another ordered table, four times the churn the
contacts' re-sort already pays for (What the ECS costs), with the pair's
points no longer together. Four inline is the recommendation: Box3D caps a
manifold at four (`B3_MAX_MANIFOLD_POINTS`) and Jolt prunes face contacts to
four (`PruneContactPoints`), as read in their fetched source.

**What rotation will add (predicted, not measured;** what it did add is
in "Rotation in 3D", below**):**

- **Orientation and angular state as components**: a quaternion (16
  bytes) and an angular velocity (12), and the solver body grows from 7
  floats to about 20 (a world inverse inertia, 6 floats, recomputed each
  step from the body's and its rotation), so the copies in and out, 190 µs
  here, about triple.
- **The spatial key's bounds depend on two components**, position and
  rotation, and on the collider: `SpatialKey` has one extent, so either a
  `Transform` key (position and rotation together) or extents that are a
  tuple. A body that only turns must still be re-bounded, so fewer rows are
  still bit for bit, and re-bounding a rotated box is a matrix, not an add.
- **Manifolds and the narrowphase**: points (above), per-point feature ids
  for warm starting, and a clipping or GJK/EPA narrowphase, several times
  today's per pair; a per-pair cache (the last separating axis) belongs on
  the contact entity with the points.
- **Islands and sleeping matter more**, since solving a settled pile is
  where the time goes, as it is in 2D.

## Rotation in 3D

**Status: built, experimental, a mod** (2026-09-26, get-emj.38; turning
on branch `physics3d-rotation`, a mod since `physics3d-land`).
`//engine/std/physics3d` bodies turn: spheres and boxes with orientation,
angular velocity and inertia, contacts of up to four points, the soft
step with angular terms. It is a mod as `//engine/std/physics` is, and
hot-reloads under a running pile ([A mod](#a-mod)). Still left out: layers,
sensors, kinematic bodies, sleeping, events, parallelism, rolling
resistance, gyroscopic terms. The comparison runs every engine locked (as
before) or turning (`--rotate`).

### What it is

- **Components** (the interface crate, `components.rs`). `Position` stays
  the spatial key, bounded by the collider turned by the rotation, both
  extents (choice 5, below); `Rotation` (a unit quaternion),
  `AngularVelocity`, `Body` (with the inverse inertia about the body's own
  axes, zero for a body that doesn't turn: `Body::new` locked,
  `Body::solid` a solid of the collider's shape). `fixed` and `dynamic`
  spawn the bundles. The step's settings are components too: `Gravity`,
  and `Tuning`, every choice below that was measured more than one way.
- **Contacts** stay entities in an ordered table by pair. `Manifold` holds
  the normal, the offset between the two centers and up to four points
  inline, each its anchor on `a` (from `a`'s center, in world axes) and
  depth, four floats a point, with a feature id each and the separating
  axis that found it; `Impulse` a normal impulse per point, and friction
  (a vector in the tangent plane) and twist for the whole contact: 31 and
  8 words. The spike measured four points inline at about 2% of a step.
  Since recycling ([Still at rest](#still-at-rest)) a `Manifold` also
  holds each body's rotation when its points were carried and a bound on
  the pair's move since they were found: 40 words.
- **The narrowphase** (`narrow.rs`): sphere against sphere or box, one
  point halfway between the surfaces; box against box, the separating
  axis test over the 15 axes, the last step's axis tried first, the face
  that separates most as the reference (a face of `b` only when clearly
  better), an edge pair only where it makes a face of the Minkowski
  difference and separates clearly more, the incident face clipped to the
  reference face, points halfway between the faces, more than four
  reduced to four. All of that is Box3D's (`b3CollideHulls`), credited in
  [CREDITS.md](../CREDITS.md).
- **The solver** (`solver.rs`) is the 2D soft step with Box3D's angular
  terms: per point, anchors on both bodies and an effective mass with the
  angular terms, and each body's world inverse inertia, all fixed once a
  step; within the step a point's separation is its separation when found
  plus its anchors' moves along the normal, from each body's accumulated
  move and turn (`b3SolveContact`). Friction is per contact at the points'
  centroid (a 2x2 tangent mass, clamped to a disc of friction times the
  points' normal impulse) with twist friction about the normal, only in
  the relaxing passes, as Box3D, Rapier and Jolt all have it. A rotation is
  stepped to first order and normalized every substep, at most a quarter
  turn a step. The passes read a 64-byte copy of each body (velocities,
  moves, inverse mass) and each point's crossed anchors already through
  the inverse inertias, so a pass is dot products and adds: that took the
  solver from 3147 to 2245 µs on 1000 turning boxes.

### A mod

Since 2026-09-26 (`physics3d-land`) the step is a mod, laid out as 2D's:
an interface crate other mods depend on (`components.rs` and `math.rs`,
crate `physics3d`), and the mod (`lib.rs`, `narrow.rs`, `gjk.rs`,
`solver.rs`): three systems, `integrate_velocities`, `find_contacts` and
`solve`, in the phase `physics3d::step`, after `simulate` and before
`late` at the simulation's rate, as `physics::step` is. Its messages are
`stats` (steps, contacts, time per system), `stages` (time per stage, and
the last step's pairs, contacts, points and warm starts) and
`reset_timings`. `pile3d` (`tests/pile.rs`) is its scene mod: it builds
the comparison's scenes from `tests/scenes.rs`, the file the bench builds
them in the other engines from, statics at `build` and each step's
arrivals from a system in `simulate`, where a game's spawns would be; and
single bodies for the tests. `pile3d_game` runs it on lockstep:

    ./bazel run //engine/std/physics3d:pile3d_game
    bazel-bin/engine/modctl/modctl send pile3d build boxes 1000
    bazel-bin/engine/modctl/modctl send lockstep step 300
    bazel-bin/engine/modctl/modctl send pile3d stats
    ./bazel run //engine/std/physics3d:physics3d    # reloads the step

**No static state.** The experimental step kept its settings and timings
in static `Mutex`es, since harness systems are plain functions.[^static3d]
A mod's statics are its image's: a reload maps a new image, whose statics
start over. Where each could live, measured by the reload replay below
(a pile tuned `relax=3,warm=nearest` reloaded every frame) and by what a
step pays to read it:

| settings in | survive a reload | a game or test sets them by | read, a step |
|---|---|---|---|
| a static in the mod | no: the replay differs at frame 3 (planted: the `Tuning` gone at load, what a new image's static is) | a call into the mod's code | a lock |
| the mod's state | yes | a message to physics3d only | a field |
| **a component in the world (`Tuning`, `Gravity`), as 2D's `Gravity` and `Sleep`** | **yes, and the replay compares it** | **writing it, like any data; `pile3d tune ...` does** | **a one-entity query: with a `Tuning` entity or none, every stage the same within 1 µs (1000 boxes, 1000 spheres, two runs each)** |

Timings are different: they are the mod's own bookkeeping, not what a
game sets, so they are its state (`Timings`, `Found`), as 2D's physics
keeps them, and a reload keeps them too.

**Through the engine, at the harness's cost.** The bench's ours is now the
mod in an engine, stepped by `lockstep step 1` (so a step is a frame, with
the schedule and apply nodes in it), as `//engine/std/physics:tax` drives
2D's. The same scenes, turning, whole run in ms a step (one run each, the
harness's the second batch of choice 5):

| | spheres 1000 | boxes 1000 | planks 1000 | rain 1000 | spheres 10 000 | boxes 10 000 | planks 10 000 | rain 10 000 |
|---|---|---|---|---|---|---|---|---|
| harness, plain systems | 1.91 | 2.73 | 3.48 | 1.60 | 22.4 | 31.2 | 46.1 | 20.6 |
| **the mod in the engine** | **1.92** | **2.85** | **3.51** | **1.66** | **25.0** | **29.6** | **46.0** | **21.5** |

Every quality number (pairs, contacts, depths, when it settled, what moved)
is the same in both, to the last digit printed, and the plank that tips
off its turn goes the same 0.7449574 into the floor with its bounds
planted wrong: the same computation. The frame costs the engine 1-4% at
1000 bodies, within the machine's spread at 10 000.

**Reloads are invisible.** `//engine/std/physics3d:reload_test` replays
64 boxes dropped in a walled box with the step tuned away from its
defaults, and a sphere spun onto them at frame 60, 160 frames, while
reloading physics3d, pile3d and the scheduler (each swapped with its
`engine_mod(twin = True)` build) every frame, one a frame in turn, and in
mixed batches now and then, poison mode on; every frame must equal the
run without reloads bit for bit: every physics3d component in storage
order (contacts, manifolds with their cached axes and ids, impulses, the
`Gravity` and `Tuning`), the change ticks, what physics3d and pile3d
report of their state. Planted in physics3d's `load`, each of these fails
all three plans: impulses zeroed, `Manifold` removed from contacts,
contacts despawned, `Tuning` despawned, the cached axis zeroed, steps
reset. `a_tuning_in_the_world_is_the_steps` checks both systems read the
`Tuning` (planted: either ignoring it fails it).

**What is shared with 2D.** Options, with what each would cost:

| | components | a 3D interface change rebuilds (measured, `bazel build //...`, fastbuild) | mods to reload after it | 2D |
|---|---|---|---|---|
| **separate interfaces (built)** | none shared; the ECS's spatial key, `near_pairs`, ordered tables and `pair_key` are the common part | 17 actions, 0.9 s: physics3d, pile3d and their twins, the bench | physics3d, pile3d | untouched: `:tax` bit for bit, the games' replays |
| a common interface (a transform, a velocity) both mods use | `Position`, `Velocity` at most: no layout matches (x, y against x, y, z; a rotation as (cos, sin) against a quaternion; 2D's `Body` has a kind, layers and a gravity scale, 3D's an inertia per axis) | what a 2D interface change rebuilds today: 72 actions, 1.8 s, 23 mod libraries | every mod of pong, the platformer and the demos, which reload with it | every 2D row a z and a quaternion, or a second set of names |
| one mod over a dimension | all, generic over D | everything physics touches | every game's | as above, and `component!` has no generics |

So nothing is shared but the storage: a 3D change never reaches a 2D
game's build or its running mods, which is what mod-deps are for. What is
copied is small: `Slots` (entity to index, 15 lines) and the pattern of
the step's systems.

[^static3d]: 2026-09-26: `TUNING` and `TIMINGS` were static `Mutex`es in
    the experimental step (plain systems on the ECS harness, "one
    simulation runs at a time"). Removed when it became a mod.

### The choices, measured

Every option is behind `physics3d::Tuning` (`--tune=...` on the bench,
`P3_TUNE` in `physics3d_test`), so each can be run again. 1000 turning
bodies, ours only, one thread, `-c opt`; "settled" is the step from which
every body is under 0.05 m/s at its farthest point. Settling a pile is
chaotic: one late wobble moves it by hundreds of steps, so box piles were
run at 900, 1000 and 1100 bodies, and planks (half extents 0.5, 0.125,
0.25), noisier still (the same variant settles anywhere from 260 to never),
are quoted only where a variant fails outright. Times are single runs at
1000 and move by up to 10% between batches; each table's are one batch.

**1. Box against box.** Box3D: the separating axis test with a cached
axis, and clipping. Parry (Rapier): all 15 axes every step, then the
faces clipped. Jolt: GJK and EPA for the penetration axis, then the two
supporting faces clipped along it, the axis kept as the normal
(`gjk.rs`: our implementation of Jolt's approach, with brute-force
Johnson subsets and an allocating EPA, so its time is an upper bound on
Jolt's).

| 1000 bodies | box pile settled (900 / 1000 / 1100) | step ms | narrowphase µs | warm-started | planks | rain step ms |
|---|---|---|---|---|---|---|
| **SAT, last axis first (Box3D)** | **272 / 180 / 198** | **2.72** | **346** | **98%** | settles | **1.58** |
| SAT, every axis every step (Parry) | 249 / 985 / 476 | 3.70 | 1083 | 97% | settles | 1.80 |
| GJK, EPA, clipping (Jolt's way) | never | 26.0 | 23 322 | 89% | never | 9.8 |

The cache is not only cheaper (one axis for a resting pair, not 15): it
is hysteresis. Two faces of a resting box separate within a slop of each
other; without the cache the test flips between them, the points' ids
change and warm starting loses them. The EPA normal, the true least
translation, is not a face normal and moves a little every step, so its
manifold flickers too.

**2. Reducing to four, and storing them.** Box3D: the deepest point, the
farthest from it, the largest triangle, the point adding the most area.
Rapier and Jolt: the deepest, the farthest, and the farthest either side
of the line through them. Or all eight a clipped face gives, inline
(`MAX_POINTS = 8`, a rebuild).

| 1000 bodies | box pile settled | pen max | step ms |
|---|---|---|---|
| **area (Box3D)** | **272 / 180 / 198** | **0.0071** | **2.99** |
| line (Rapier, Jolt) | 232 / 177 / 255 | 0.0071 | 3.08 |
| eight, inline | 265 / 242 / 159 | 0.0076 | 3.08 |

Area and line are level, and eight points buy nothing for their 3%. Area stays,
as the one Box3D pairs with its ids. Points stay inline: a contact's
points are read and written together, and a table of points would be four
rows a contact to churn (the spike's measurement).

**3. Warm starting.** Box3D and Parry: by feature id. Jolt: the nearest
last point within 1 cm, in each body's frame (ours compares anchors in
world axes, the same thing for bodies at rest). Or none.

| 1000 bodies | box pile settled | warm-started | pen max | turning spheres, top speed at the end |
|---|---|---|---|---|
| **feature ids** | **272 / 180 / 198** | **98%** | **0.0071** | **0.057** |
| nearest within 1 cm | 331 / 163 / 202 | 98% | 0.0066 | 0.284 |
| none | never | 0% | 0.023-0.046 | 0.620 |

Ids and nearest are level on boxes; ids stay, since they cost nothing to
match and don't depend on how far a body moved or turned in between.

**4. Stepping rotation, and inertia.** Box3D and Rapier: q + h/2 w q,
normalized every substep. Jolt: the exact turn about w. Or normalized
once, at the end of the step. The world inverse inertia formed once a step
(Box3D, Rapier) or again every substep. Cubes and spheres have the same
inertia about every axis, so only planks can tell these apart, and there
the noise is larger than any difference.

| 1000 bodies | box pile settled | step ms |
|---|---|---|
| **first order, normalized every substep (Box3D)** | **272 / 180 / 198** | **2.99** |
| first order, normalized once a step | 290 / 196 / 175 | 3.07 |
| the exact turn (Jolt) | 190 / 189 / 187 | 3.02 |
| inertia again every substep | 238 / 194 / 214 | 3.33 |

None settles a pile better, and inertia every substep costs 11%. Skipping
the normalization lets a fast spin grow the quaternion
(`a_spinning_body_stays_a_rotation` fails without it). Also measured: a
point's separation to first order in the turn (θ x r for a turn θ, a dot
product with what the row holds, in place of two quaternion rotations):
7% off the solver, and the box pile never settled. Gyroscopic terms
(Box3D and Rapier have them on, Jolt off) are left out: they vanish for
cubes and spheres.

**5. Bounds, without changing `engine/ecs`.** A key's bounds see the key
and one extent; a turned box's depend on its position, its rotation and
its shape. Options: the sphere around every collider, the collider
staying the extent (so a body that only turns is never re-bounded); the
same for turning bodies only, statics and locked bodies keeping their
box; or a `Reach` extent holding the box around the collider as turned
(`|R| h`, Box3D's `b3AABB_Transform`), rewritten by the solver when a box
turns. A pose key (position and rotation in one component) gives the same
bounds as `Reach` and was not built: it changes every query of
`Position`, and a write of either half re-bounds the row, where `Reach`
is written only when a box's box changes.

Ours, turning, µs a step over the run (pairs are the broadphase's, a
step); the contacts found are the same in every row, so quality is too:

| | boxes 1000: pairs | broadphase | narrowphase | step ms | boxes 10 000: pairs | broadphase | step ms | planks 10 000: pairs | broadphase | step ms |
|---|---|---|---|---|---|---|---|---|---|---|
| a sphere around every collider | 14 417 | 496 | 448 | 3.20 | 157 999 | 34 835 | 64.5 | 315 835 | 43 184 | 91.0 |
| a sphere for turning bodies, statics their box | 10 114 | 138 | 408 | 2.83 | 115 613 | 2253 | 31.2 | 268 064 | 5040 | 51.3 |
| **the box as turned (`Reach`)** | **2477** | **45** | **295** | **2.58** | **25 688** | **1180** | **30.7** | **67 876** | **2165** | **48.1** |

The sphere around a wall reaches every body in the pile (docs/lore); even
for unit cubes the sphere, 1.7 times as wide, finds four to six times
the pairs, and the broadphase doubles at 10 000.

**Now: the collider and rotation as the key's two extents.** Once the
storage took a pair of extents (2D's rotation, spatial-storage.md, "Bounds
from several components"), `Position` got `type Extent = (Collider,
Rotation)` and `bounds` computes `|R| h` itself (`Collider::turned_half`),
so a turn re-bounds the row through storage and no derived copy is kept:
the solve writes a rotation only when the body turned, as it writes a
position only when it moved.[^reach3d] The same bounds, so the same pairs
and contacts: every quality number of every scene below is identical,
before and after. Turning, one thread, `-c opt`, two batches each (the
second pair run side by side on separate cores), µs a step:

| | boxes 1000: pairs, broadphase, narrowphase, outside systems, step ms | boxes 10 000 | planks 10 000 | spheres 10 000 |
|---|---|---|---|---|
| `Reach`, the solve rewriting it | 2440, 44-48, 343-350, 21, 2.71-2.75 | 25 688, 1177-1198, 3402-3466, 255-261, 28.7-29.2 | 67 876, 2137-2149, 7298-7995, 986-1639, 45.8-49.5 | 51 269, 1787-1852, 1449-1645, 783-837, 24.5-24.9 |
| **(Collider, Rotation) extents** | **2440, 45, 348-351, 27, 2.73-2.74** | **25 688, 1191-1209, 3498-3727, 331-674, 29.1-31.2** | **67 876, 2147-2194, 7318-7385, 1028-1095, 45.5-46.1** | **51 269, 1753-1790, 1034-1038, 431-472, 22.4-22.6** |

Level, within the batches' spread: the broadphase is the same walk over the
same boxes, and the re-sort ("outside systems") does a little more for
boxes (every turned row is re-bounded, where `Reach` was rewritten only
when its box changed) and less for spheres (a column fewer to move). The
spread at 10 000 (a single run each) is the machine's, not the variant's.

**6. The soft step, as 2D's or not.** 2D chose 5 substeps, contacts at a
quarter of the substep rate (75 Hz), two relaxing passes, friction only in
them. Box3D: 4 substeps at 30 Hz, one relax. Box2D's rule has friction in
the pushing pass too.

| 1000 turning bodies | box pile settled | pen max | step ms | 10 boxes stacked, 600 steps |
|---|---|---|---|---|
| 2D's: 5 x 75 Hz, 2 relax | 216 | 0.0054 | 2.57 | never still, 0.13 m/s |
| **5 x 60 Hz, 2 relax** | **180** | **0.0071** | **2.65** | **at rest by 600** |
| 5 x 45 Hz, 2 relax | 191 | 0.0104 | 2.61 | at rest by 300 |
| 5 x 45 Hz, 1 relax | 199 | 0.0114 | 2.01 | |
| Box3D's: 4 x 30 Hz, 1 relax | 225 | 0.0227 | 1.69 | at rest by 600, 0.026 lower |
| Box3D's, friction pushing too | 258 | 0.0239 | 1.84 | |

The same step, a notch softer. At 2D's stiffness a column of ten turning
boxes never comes to rest: each box rocks on its four points, a mode a
locked box doesn't have. A fifth of the substep rate is the stiffest that
stands the stack, and sinks a third as deep as Box3D's. One relax pass is
a quarter cheaper and settles the pile about as soon, but the stack took
two, so two stay. Friction in the pushing pass is worse here, as in 2D.
Static contacts stayed twice as stiff (0.4) until a five-high stack was
found circling on its corners; they are 0.25 since ([Still at
rest](#still-at-rest)).

### Against the others, turning

`./bazel run -c opt //bench/physics3d:bench -- all 1000,10000 all --rotate`
(and without `--rotate` for locked). One thread, sleeping off, every engine
at its defaults (Rapier 4 substeps at 30 Hz; Jolt 10 velocity and 2
position iterations; Box3D 4 substeps at 30 Hz; ours 5 substeps at 60 Hz, 2
relax); 1000 bodies the median of three runs, 10 000 one. Whole run, ms a
step; ours is the mod in the engine, a step a lockstep frame (run again
2026-09-26 once it was a mod: every quality number is what the plain
systems gave, so the tables below stand):

| turning | ours | Rapier | Jolt | Box3D |
|---|---|---|---|---|
| spheres 1000 | 2.00 | 0.87 | 1.84 | 1.58 |
| boxes 1000 | 2.85 | 0.91 | 1.36 | 1.11 |
| planks 1000 | 3.59 | 1.21 | 2.27 | 1.57 |
| rain 1000 | 1.67 | 0.87 | 1.23 | 1.07 |
| spheres 10 000 | 26.2 | 17.6 | 26.7 | 18.2 |
| boxes 10 000 | 30.6 | 13.6 | 18.9 | 11.5 |
| planks 10 000 | 44.5 | 23.5 | 36.3 | 22.5 |
| rain 10 000 | 21.2 | 15.7 | 16.8 | 14.1 |

| locked | ours | Rapier | Jolt | Box3D |
|---|---|---|---|---|
| spheres 1000 | 1.21 | 0.64 | 1.21 | 1.05 |
| boxes 1000 | 2.62 | 0.90 | 1.13 | 1.07 |
| spheres 10 000 | 14.2 | 13.8 | 19.9 | 12.7 |
| boxes 10 000 | 29.0 | 14.3 | 13.6 | 12.0 |
| planks 10 000 | 21.7 | 10.7 | 13.5 | 9.4 |
| rain 10 000 | 12.0 | 8.5 | 9.6 | 7.6 |

By stage at 10 000 turning, µs a step (Jolt exposes none):

| | broadphase: ours / Rapier / Box3D | narrowphase | solver | ours: copies, re-sorts |
|---|---|---|---|---|
| spheres | 1825 / 198 / 391 | 1604 / 3237 / 4012 | 18 891 / 14 923 / 14 222 | 700, 835 |
| boxes | 1190 / 121 / 202 | 4120 / 1021 / 2048 | 23 141 / 11 928 / 9626 | 678, 269 |
| planks | 2069 / 159 / 300 | 7473 / 5132 / 6367 | 32 291 / 19 742 / 16 375 | 1051, 654 |

Quality, from the harness's own geometry: deepest overlap at the end, and
the step from which every body stays under 0.05 m/s at its farthest point:

| turning | ours | Rapier | Jolt | Box3D |
|---|---|---|---|---|
| boxes 1000: deepest, settled | 0.007, 180 | 0.022, 304 | 0.020, 502 | 0.023, 299 |
| boxes 10 000 | 0.013, 374 | 0.042, 716 | 0.028, 1413 | 0.043, 713 |
| planks 1000 | 0.010, 408 | 0.030, 278 | 0.026, 840 | 0.034, 262 |
| planks 10 000 | 0.024, never (6 moving, 0.34 m/s) | 0.076, 392 | 0.060, never (27) | 0.078, 369 |
| spheres 10 000: deepest, energy at the end | 0.017, 0.12 | 0.063, 0.007 | 0.036, 0.010 | 0.062, 0.017 |
| rain 10 000: deepest | 0.002 | 0.010 | 0.056 | 0.098 |

No engine's turning spheres or rain come to rest: spheres roll, and none
has rolling resistance on (docs/lore). Nothing escapes in any run.

**What it shows.**
- **It works, and piles of boxes settle best.** Turning box piles come to
  rest sooner than in any of the three and sink a third as deep, the depth
  bought, as in 2D, by substeps and a stiffer contact. Planks are the weak
  spot: at 10 000 six still wobble at step 1500, where Rapier and Box3D are
  at rest by 400.
- **It costs 1.3 to 3 times Rapier and Box3D**, most of it the solver,
  and most of that the passes: 5 substeps of 3 passes (and 5 warm starts)
  against their 4 of 2. Per pass, ours is about 1.3 times Box3D's (23.1 ms
  over 15 passes against 9.6 over 8, theirs including integration), scalar
  against their 4-wide SIMD. Box3D's own settings in ours (choice 6) cost
  1.69 ms at 1000 boxes against their 1.09, and sink as deep as theirs.
- **The narrowphase is 2-4 times theirs on boxes** because they keep a
  contact's manifold while its bodies barely move (Box3D recycles it,
  Rapier's `try_update_contacts`, Jolt's body-pair cache), where ours
  clips every pair every step; the cached axis only saves the axis test.
- **The broadphase is still 5-10 times theirs**, as before rotation: they
  keep pairs.
- **Locked bodies now pay for manifolds.** A locked box pile runs the same
  clipping and four-point solve as a turning one: 29 ms at 10 000, where
  the translation-only step took 4.9 (the spike) and 6.3 (its soft step).
  And at 60 Hz a locked 10 000 box pile breathes, 9130 bodies never at rest
  in 1500 steps, as Rapier's and Box3D's do at their defaults
  (docs/lore/a-locked-box-pile-breathes-forever-under-soft-contacts-at-4-iterations.md);
  at 1000 it settles by 95. The translation-only step settled it by 204.

What would close the gaps, in order: manifolds kept while bodies barely
move (the narrowphase, and fewer re-found contacts; done 2026-09-27, box
pairs, halving the narrowphase on piles: [Still at rest](#still-at-rest));
a broadphase that
keeps its pairs (both comparisons now); a colored SIMD solve; stiffness
per contact (locked pairs as stiff as 2D's, turning ones softer); rolling
resistance; sleeping, which piles that settle would fall into.

### What 3D asks of the storage design

What the step worked around, and what it wanted instead, for the design
of rotated bounds (get-emj.38's 2D side). Items 1 and 2 are done: the
storage takes two extents, and physics3d uses them (choice 5).

1. **Bounds from more than one extent** (done). A turned box's bounds
   depend on the key (its position), its rotation and its shape. With one
   extent a key, the step kept a derived `Reach` as the extent;[^reach3d]
   now `(Collider, Rotation)` are the extents and the glue computes `|R| h`
   itself, a page at a time, in the loop that re-boxes pages.
2. **Turning alone re-bounds** (done). A box spinning in place writes no
   position, and its bounds still change; writing the rotation, an extent,
   re-bounds the row (`a_planks_bounds_follow_its_turn`: a plank bounded
   as if unturned goes 0.74 into the floor).
3. **Not a sphere.** Rotation-invariant bounds are the cheap way out, and
   cost 4 to 6 times the pairs (above), ten times the broadphase once
   statics get them too. Whatever the storage offers, statics and bodies
   that can't turn must keep exact boxes.
4. **Fat bounds, or kept pairs** (done, 2026-09-27: `Live<Contacts>`,
   spatial-storage.md, "Keeping pairs"). A turning body is re-bounded
   every step it moves, where Box3D re-inserts a body in its tree only when
   it leaves a box grown by up to 0.05 (`aabbMargin`). The re-bounding
   stays (the order needs exact boxes); the pairs are now kept over fat
   boxes, as Box3D's are.
5. **Wider tuples.** Without `Reach`, a turning body is six components and
   the solve's query five, against limits of eight (bundles, query data)
   and four (a parameter group): the mod hit none of them. Layers,
   sleeping or a kinematic flag would take a turning body to eight or
   nine, where the limit would be raised in `engine_ecs` rather than
   bundles nested.
6. **What needed nothing.** Four points inline as `[f32; 16]` and
   `[u32; 4]` (`OPAQUE` fields), the ordered contact table, change
   detection: unchanged from the spike.

[^reach3d]: 2026-09-26: until then, with one extent a key, the step kept
    the box around each body as turned in a derived `Reach` component (the
    key's extent), rewritten by the solve when a box's box changed: three
    floats a body held twice (there and in the page lanes), and a write
    the solve had to remember. Its measurements are the table above.

## Quality as a test

**Status: built** (2026-09-26, get-emj.37). How soon a scene comes to rest,
how deep it sinks while it does and once it has, what energy is left,
whether stacks and pyramids stand and nothing escapes, and that sleeping
then follows, are tests, in 2D and 3D, locked and turning, bounded by what
the reference engines meet on the same scenes. Until then the tests asked
whether a pile came to rest eventually, on a pile that stood in columns,
and a solver that crept for thousands of steps passed them
([Settling](#settling)).

| target | what | runtime |
|---|---|---|
| `//engine/std/physics/compare:quality_test` | 2D: piles 400-1200, pyramids 120-325, stacks 10 and 20, sleeping, the mod bit for bit the arrays | 6.7 s (fastbuild) |
| `//engine/std/physics/compare:quality_long_test` (manual) | 2D: piles 9000-11 000, the 5050 pyramid | 37 s at `-c opt` |
| `//bench/physics3d:quality_test` | 3D: piles of cubes (turning, locked) and planks 200-500, stacks 10-20 | 21 s, the slowest pile (500 planks) 19 s (fastbuild) |
| `//bench/physics3d:quality_long_test` (manual) | 3D: cubes and planks at 1000 and 10 000 | 99 s at `-c opt`, the known failures included |

**What runs.** The comparisons' own scenes and measures, not copies: 2D's
`scene.rs` (a `Stack` scene added), `quality.rs` and `settle.rs` (the
settling loop `SETTLE` prints, which the tests call), 3D's `scenes.rs` (a
`Stack` kind added) and `measure.rs` (overlap now also looked at every 10
steps while settling, and a stack's top and tilt). A bound is then the
same number the comparison prints for Box2D, Rapier, Box3D and Jolt. 2D
runs on the arrays, bit for bit the mod (`the_mod_is_the_arrays_bit_for_bit`
holds that in the suite; before, only the comparison checked it, when run),
so `SOLVER=<variant>` can put any of `variants.rs` in its place; sleeping,
which the arrays don't have, runs on the mod. 3D runs the mod in the
engine, `TUNE=<Tuning>` tuning it.

**The measures**, each from positions and velocities alone, as every
engine is measured:

- **Steps to rest**: the look (every 10 steps) from which every body, at
  its farthest point, stays under 0.05, the sleep threshold, to the end of
  the run (700 steps in 2D, 2500 at 10 000; the bench's 1000 and 1500 in
  3D).
- **Overlap**: the deepest and the mean over touching pairs at the end, and
  the worst of each at any look while settling, landings included.
- **Energy** at the end, moving and turning, a body.
- **Standing**: how far a pyramid's or a stack's top box moved, the most any
  box leans, bodies out of the scene.
- **That it is a pile**: contacts a body and islands in 2D, partners a body
  and the share not in columns in 3D. A scene that went back to columns
  (docs/lore) fails as not a pile, not as settling well.
- **Sleeping**: every body of a pile of 1000 asleep within half a second
  (the sleep time) of the rest bound.

**How the bounds are set.** Options weighed for each:

- *Steps to rest.* Bounding one run fails on noise: rest moves by 100-200
  steps with rounding alone (docs/lore), and so does the references'. A
  bound at the references' worst run is loose where one of theirs is an
  outlier: Box3D rests turning planks at 209-302 steps, but at 823 at 200
  planks. So piles run at several sizes (each size its own drop), and the
  bound is on the distribution: **the worst of ours within twice, and the
  median of ours within a quarter over, the later of the references'
  medians over the sizes.** Pyramids and stacks stand and don't move with
  rounding: one scene each, within twice the later reference.
- *Depth.* A bound at the references' depth would let ours sink to theirs
  unnoticed, and sinking a quarter to a fifth as deep is what the stiffer
  contacts were chosen for ([Settling](#settling), choice 6 in 3D): **half
  the shallower reference's worst** at rest, and a stack's or pyramid's top
  within half the smaller reference's sinking. While settling, where every
  engine lands as deep (push-out capped at 3 u/s in all), **a quarter over
  the references' worst**, and the mean **within the shallower
  reference's**.
- *Energy.* **Ten times the references' worst** over the runs they came to
  rest on (a pile that breathes isn't at rest), and never under 1e-8 a body,
  below which it is rounding (every body under about 1e-4).
- *Which references.* Box2D and Rapier in 2D. Rapier and Box3D in 3D, the
  soft steps ours is one of; Jolt, whose hard contacts rest a turning box
  pile last (502-1413) and a locked one first, is recorded beside them.

**The bounds and what they came from** (2026-09-26; each test's comment
has every reference value, by size). Rest in steps; the references' medians
over the sizes, then ours, worst / median:

| scene | references (median rest) | bound: worst / median | ours: worst / median | depth at rest: shallower reference / bound / ours |
|---|---|---|---|---|
| 2D pile 400-1200, locked | Box2D 200, Rapier 160 | 400 / 250 | 240 / 210 | 0.057 / 0.028 / 0.014 |
| 2D pile 400-1200, turning | Box2D 210, Rapier 250 | 500 / 312 | 440 / 290 | 0.099 / 0.049 / 0.024 |
| 2D pile 9000-11 000, locked | Box2D 200, Rapier 200 | 400 / 250 | 300 / 230 | 0.058 / 0.029 / 0.016 |
| 2D pile 9000-11 000, turning | rest from Box2D 1760, Rapier 2490, neither staying at rest; first at rest 330, 430 | rest from 2490 / 1760, first at rest 860 / 537 | rest from 1620 / 350, first at rest 450 / 350 | 0.095 / 0.048 / 0.027 |
| 3D cubes 200-500, turning | Rapier 203, Box3D 219 | 438 / 273 | 200 / 187 | 0.014 / 0.007 / 0.0055 |
| 3D cubes 200-500, locked | Rapier 127, Box3D 250 | 500 / 312 | 83 / 69 | 0.0021 / 0.0011 / 0.0006 |
| 3D planks 200-500, turning | Rapier 267, Box3D 302 | 604 / 377 | 354 / 341 | 0.031 / 0.015 / 0.0058 |
| 3D cubes 1000 / 10 000, turning | Rapier 304 / 716, Box3D 299 / 713 | 608 / 1432 | 180 / 374 | 0.022, 0.042 / 0.011, 0.021 / 0.007, 0.013 |

Stacks and pyramids rest within 10-30 steps in every engine but Box2D's
turning 10-high stack (100); ours rest in 10-30, their tops sink a third as
far (a 25-wide turning pyramid 0.017 lower where both references' are
0.094), and nothing leans more than 0.1°.

**Known failures**: none. A known failure is an ignored test naming its
bead (run with `--test_arg=--include-ignored`), not a looser bound.
Until 2026-09-27 there were three: a five-high stack of turning cubes in
3D never rested (get-emj.42), and turning planks in 3D kept up to 1e4
times the references' energy and never rested at 10 000 (get-emj.43),
both fixed by contact recycling and softer static contacts; and a 20-high
stack of turning boxes in 2D rests late at the default five substeps
(get-emj.41), which is now tested at six, set through `physics::Tuning`
as a game that stacks would ([Still at rest](#still-at-rest)).

**What the tests catch** (mutation-checked, 2026-09-26: each bug planted in
the source, or chosen by `SOLVER`/`TUNE`, and the default suite run):

| planted | 2D: fails | 3D: fails | survives |
|---|---|---|---|
| the split impulse this step replaced (`SOLVER=split`) | piles (never at rest at 1000 and 1200; 620 at 800; energy 1e-2), pyramids and stacks (twice the depth) | – | sleeping (the mod), bit for bit |
| one relax pass, not two | piles (rest 510-never at 1000-1200), the turning stack, sleeping | locked cubes, planks, stacks | pyramids, locked stacks; 3D turning cubes |
| softer contacts (30 Hz, the references' stiffness) | every pile, pyramid and stack, on depth | every test | – |
| no warm starting | every test | every test | bit for bit |
| contacts ignoring rotation (a point's separation not following its arms) | the turning stack (energy), a solver unit test | planks (rest 989) | 2D piles and pyramids; 3D cubes and stacks |
| the separation to first order in the turn (3D `anchors=linear`) | – | planks (rest 950-998) | cubes, stacks |
| sleeping ten times slower to take | sleeping | – | – |
| piles dropped unstaggered, or in a lattice (columns) | piles, as not a pile (1.0 contacts a body, 31-32 islands) | cube and plank piles, as not a pile (0.00 not columns) | – |

Contacts ignoring rotation is the weakest catch, as measured before
([Rotation in the soft step](#rotation-in-the-soft-step): `sep=2` moves a
1000-pile's rest within the noise, and a 10 000 pile's threefold): at the
default suite's sizes only a stack's energy and planks see it.
`:quality_long_test`'s turning 10 000 piles are where it shows: one of
the three never rests.

## Quality beyond settling

**Status: built** (2026-09-28, get-emj.13 for the view). [Quality as a
test](#quality-as-a-test) asks whether piles, pyramids and stacks come to
rest, how deep they sink and what energy is left. What a player feels is
something else: whether a box on a slope holds or slides as fast as it
should, a ball bounces as high, a heavy crate stays on a light one,
overlap is pushed apart without a bang, a fast ball stops at a wall, and a
structure that stands on friction stands. These are now scenes, run in
every engine, with tests bounding ours by a hand calculation or by what
the references do on the same scene, and a debug view to look at any of
them.

| target | what | runtime |
|---|---|---|
| `//engine/std/physics/compare:behaviour_test` | 2D: ramps, bounces, mass ratios, overlap, bullets, the card house, the ladder, dominoes; the mod bit for bit the arrays on them | 2-5 s (fastbuild) |
| `//bench/physics3d:behaviour_test` | 3D: ramps, bounces, mass ratios | under 1 s |

### The debug view

`view.rs` in the comparison draws any scene at chosen steps for every
engine side by side: an SVG, a row a step and a column an engine, and, for
whoever reads rather than sees (agents included), the same as text, a
character grid. Bodies come from what every engine reports
(`Sim::bodies`: positions and angles, boxes turned, circles with a radius
line showing their turn), statics from the scene, sleeping bodies from
each engine (grey; `.` in text), and contacts from each engine's own:
ours from the arrays or, on the mod, from the world's `ContactPair`,
`Manifold` and `ContactPoints`; Box2D's from `b2Body_GetContactData`;
Rapier's manifolds. Each point is drawn with its normal, pressed (red,
`*`) or held within the speculative margin (hollow orange, `+`); where
neither end of our contact turns, it keeps no point, only a normal and a
depth, and the view puts one on the smaller body's face (a square). It is
test and bench code: nothing renders in the physics mod.

    VIEW="pile 1000 41" VIEW_STEPS=0,100,400 VIEW_OUT=/tmp VIEW_TEXT=90 ./bazel run -c opt //engine/std/physics/compare

(runbook 005, "The debug view"). Behaviour scenes are drawn turning, as
they run; `TURN=1` turns the others; `ENGINES` picks the columns.

### The scenes

Each is built by `scene.rs` (3D: `scenes.rs`) for every engine, bodies
turning, and measured from positions and velocities every step by
`behave.rs`, the same code for all (the settling measures, depth, contacts
a body and islands, beside). Gravity 20 in 2D and 9.81 in 3D; a body and
what it meets have the same friction and restitution, so every engine's
rule for mixing two gives it.

- **Friction on a ramp.** A unit box on a static ramp 20° steep at
  friction 0.6 (tan 20° = 0.36): it holds, and creeps nothing. At 30° and
  0.2 it slides at g (sin θ − μ cos θ), 6.536 (3D 3.206), from its speed
  at steps 30 and 90. A disc at 30° and 0.6 rolls without slipping at
  (2/3) g sin θ = 6.667 while μ ≥ tan θ / 3, its contact point still
  (`v − ω r` along the slope); at 0.1, below that, it slips and slides at
  g (sin θ − μ cos θ). In 3D a sphere, at (5/7) g sin θ = 3.504.
- **Restitution.** A ball dropped 5 onto a floor, no friction, rebounds to
  e² of the drop: its first apex against e², at e = 0.25, 0.5, 0.75 and 1;
  at 1, over 20 s, the highest and the last apex (a lossless ball gaining
  height is energy from nowhere).
- **Mass ratios.** A unit box 10, 100 and 1000 times as heavy on one of
  mass 1; 100 and 1000 times on a column of five; and Box2D's
  "HighMassRatio2", a 20-wide box 400 times as heavy on two unit boxes 18
  apart. How far the heavy box sinks, how deep, whether it stands, how
  soon it rests, and what still moves in the last second (jitter).
- **Overlap recovery.** Box2D's "Overlap Recovery": a pyramid of unit
  boxes 4 wide spawned a quarter and half a box into each other, and one
  10 wide at half. The fastest any body goes, the step nothing is deeper
  than 0.01, when it rests, and its top against where it rests once apart.
- **Fast bodies.** Pong's ball, radius 0.25, no gravity, restitution 1,
  fired at a static wall 0.1 thick, and at one a unit thick (pong's
  paddle), at 10 to 400 a second, each at four phases (where in a step it
  reaches the wall). Whether it passes through.
- **Structures.** Box2D's card house (from PEEL), five storeys, scaled five
  times so its cards are 2 tall and 0.01 thick, friction 0.7; Box2D's 15
  dominoes, the first knocked over as its impulse knocks it; and a ladder
  in place of Box2D's arch, whose blocks are wedges and physics has boxes
  and circles only: a plank 5 long and 0.2 thick leaning 30° on a
  frictionless wall, which stands on the floor's friction while μ ≥ tan θ /
  2 − hx / (2 hy) = 0.269 (its weight's moment about its foot against the
  wall's push at its top), at 0.4, 0.3, 0.24 and 0.2. Whether they stand,
  how far anything moved, whether the dominoes fall in order, and how fast.

**Checked to be what they say** (the counts, and the view): the ramp
bodies have one contact, the ladder two until it slides and one after,
the card house one island of 2.6 contacts a card in ours (3.0 in Box2D);
the pictures show the cards leaning in pairs under flat ones, the
dominoes lying in a chain at the end, the ladder flat on the floor after
sliding.

### Results, 2D

`BEHAVE=1 VARIANTS=rapier:ccd ./bazel run -c opt //engine/std/physics/compare`
(2026-09-28; Box2D v3.1.1, continuous on; Rapier 0.36 as shipped, its CCD
changing nothing here). Deterministic, one run. The mod and the arrays
agree on every scene to every digit.

| scene | expected | ours | Box2D | Rapier |
|---|---|---|---|---|
| box, 20°, μ 0.6: crept in 2 s | 0 | 0.0001 | 0.00008 | 0.0003 |
| box, 30°, μ 0.2: a | 6.536 | 6.536 | 6.547 | 6.536 |
| disc, 30°, μ 0.6: a; slip | 6.667; 0 | 6.653; 0.073 | 6.641; 0.052 | 6.634; 0.052 |
| disc, 30°, μ 0.1: a (slipping) | 8.268 | 8.277 | 8.284 | 8.274 |
| bounce, e 0.25: first apex / drop | 0.0625 | 0.044 | 0.038 | 0.038 |
| bounce, e 0.5 | 0.25 | 0.244 | 0.229 | 0.229 |
| bounce, e 0.75 | 0.5625 | **0.579** | 0.548 | 0.548 |
| bounce, e 1: first; highest; last, 20 s | 1; 1; 1 | **1.048; 1.64; 1.64** | 0.996; 0.996; 0.919 | as Box2D |
| 10:1: top sank; deepest; at rest from | – | 0.0015; 0.0012; 2 | 0.0097; 0.0077; 3 | 0.0097; 0.0077; 2 |
| 100:1 | – | 0.014; 0.011; **77** | 0.088; 0.072; 23 | 0.089; 0.071; 23 |
| 1000:1 | – | stands, 0.14; 0.11; **never** (0.15 a second) | crushed (1.0 lower); 41 | crushed; 29 |
| 100:1 on five | – | stands, 0.11; **never** (0.06) | topples; 253 | crushes one (0.67); 68 |
| 1000:1 on five | – | crushed, **two boxes out through the floor, 82 a second** | crushed, calm; 220 | crushed, 4.1 a second; 285 |
| wide box on two, 400:1 | – | 0.040; 0.024; **never** (0.5) | 0.176; 0.141; 34 | 0.176; 0.141; 33 |
| overlap, 4 wide at 0.25: fastest; apart at; at rest | at most the cap, 3 | 2.2; 20; 27 | 3.0; 67; 55 | 3.5; 46; 48 |
| overlap, 4 wide at 0.5 | | 2.5; 31; 33 | 3.6; 56; 54 | 4.4; 57; 58 |
| overlap, 10 wide at 0.5 | | 9.4; 43; 101, stands | 16.8; 75; 198, topples, two out | 16.5; 101; 241, topples |
| card house: most moved; at rest | stands | 0.10; 27 | 0.14; 21 | **two cards fall** |
| ladder, μ 0.4 and 0.3: slid | stands (0.269) | 0.00005 | 0.0004 | 0.00001 |
| ladder, μ 0.24 and 0.2: slid | slides | 1.30, 1.35 | 1.31, 1.35 | 1.30, 1.35 |
| dominoes: fell in order; wave; at rest | all | 15; 2.667 a second; 441 | 15; 2.667; 434 | 15; 2.736; 443 |

**Fast bodies**, phases of four that pass through the wall:

| speed (a step) | wall 0.1: ours | wall 1 (pong's paddle): ours | Box2D, Rapier (with or without CCD) |
|---|---|---|---|
| 10 to 21 (0.35) | 0 | 0 | 0 |
| 25, 30 | 1 | 0 | 0 |
| 40 (pong's fastest across), 48 (0.8) | 2 | 0 | 0 |
| 50, 56.6 (pong's fastest diagonal), 80 | 3 | 1 | 0 |
| 160, 400 | 4 | 3 | 0 |

Ours has no continuous collision ([Open questions](#open-questions)): a
ball is stopped only if some step leaves it within the speculative margin
(0.05) of the wall or short of its middle, so it bounces for certain while
a step is at most the margin, its radius and half the wall, 0.35 (21 a
second) and 0.8 (48); past that, whether it tunnels depends on the phase.
Pong's ball meets its paddles at 40 across the court at most, 20% under
the paddle's 48. Box2D sweeps fast bodies against statics
(`b2SolveContinuous`), and Rapier, with CCD off as shipped, does too
([lore](../lore/rapier-sweeps-fast-bodies-against-fixed-colliders-with-ccd-off.md)).

### Results, 3D

`./bazel run -c opt //bench/physics3d:bench -- <scene> <n> all --rotate --behave`
(2026-09-28; Rapier 3D 0.36, Jolt 5.6, Box3D 0.1, each at its defaults).

| scene | expected | ours | Rapier | Jolt | Box3D |
|---|---|---|---|---|---|
| cube, 20°, μ 0.6: crept in 2 s | 0 | 0.000001 | 0.000068 | 0.000001 | 0.000063 |
| cube, 30°, μ 0.2: a | 3.2059 | 3.2059 | 3.2059 | 3.0460 | 3.2059 |
| sphere, 30°, μ 0.6: a; slip | 3.5036; 0 | 3.5025; 0.032 | 3.5045; 0.032 | 3.3288; 0.000 | 3.5023; 0.035 |
| bounce, e 0.25; 0.5 | 0.0625; 0.25 | 0.053; 0.248 | 0.049; 0.238 | 0.051; 0.232 | 0.052; 0.241 |
| bounce, e 0.75 | 0.5625 | **0.575** | 0.554 | 0.527 | 0.557 |
| bounce, e 1: first; highest; last, 20 s | 1; 1; 1 | **1.032; 1.31; 1.31** | 0.996; 0.996; 0.962 | 0.935; 0.935; 0.529 | 0.999; 0.999; 0.998 |
| 10:1: top sank; deepest; at rest from | – | 0.0006; 0.0008; 0 | 0.0026; 0.0030; 1 | 0.0009; 0.0023; 2 | 0.0026; 0.0030; 1 |
| 100:1 | – | 0.0099; 0.0070; 42 | 0.033; 0.028; 55 | 0.030; 0.020; 193 | 0.033; 0.028; 21 |
| 1000:1: crushed in all; at rest from | – | 316 | 210 | 101 | 279 |

Jolt is 5% slow on the ramps because every Jolt body is damped by default
(`BodyCreationSettings::mLinearDamping` and `mAngularDamping`, 0.05 a
second), which the comparison leaves on: at a mean 3 a second over the
steps measured, 0.16 of the 3.21. Its bounces lose height to the same
([lore](../lore/jolt-damps-every-body-by-default.md)).

### What they found, and what is ignored

Ours meets the hand calculations as closely as any reference: every ramp
within 0.2% (the sliding box exact), the ladder standing and sliding
either side of its friction, the dominoes falling at Box2D's speed. It
stands heavy boxes the references crush, sinking 5-10 times less, as its
stiffer contacts did on piles ([Settling](#settling)); pushes overlap
apart more gently than either (at 2.2-2.5 a second where theirs reach
3.0-4.4) and keeps the 10-wide pyramid standing where both topple it; and
stands the card house Rapier drops. Four things are wrong, each an ignored
test naming its bead (run with `--test_arg=--include-ignored`), not a
looser bound:

- **A bounce returns a step's gravity more than it came in with**
  (get-emj.56; 3D get-emj.60). `integrate_velocities` adds the step's
  gravity before contacts are found, and restitution restores e times the
  closing speed before the step, gravity included; the references take it
  before gravity. At 14 a second that is 2.4% of speed a bounce: the
  lossless ball climbs to 1.64 of its drop in 12 bounces (3D 1.31 in 9),
  and e = 0.75 passes e². Below 0.5 every engine loses more to the soft
  contact than this adds, so it hides. Pong is spared: its ball has no
  gravity.
- **A heavy box on light ones never comes to rest** (get-emj.57): at
  100:1 it rests from 77 where both references rest from 23; at 1000:1, on
  the column of five at 100:1, and under the wide box it keeps moving at
  0.15, 0.06 and 0.5 a second, over the sleep threshold, so it never
  sleeps. In 3D it rests (from 42 at 100:1, where Rapier rests from 55).
- **A box 1000 times as heavy crushing a column of five throws two light
  boxes out through the floor**, at up to 82 a second (get-emj.58); the
  references crush it too, calmly. The push-out cap doesn't hold a body
  squeezed between a static and one a thousand times its mass.
- **No continuous collision** (get-emj.59): the limit above, where both
  references never tunnel. Pinned by an active test at the limit
  (`a_ball_bounces_off_a_wall_while_a_step_is_within_the_margin_its_radius_and_half_the_wall`:
  every phase bounces up to it, some tunnel just past it), so building
  continuous collision fails that test, which is when to move it.

**How the bounds are set**, as [Quality as a test](#quality-as-a-test)
sets them, per scene, the reference values beside each in the tests: a
hand calculation within 1% (the references are within 0.5%, Jolt aside);
where there is none, twice the later reference's rest, a quarter over the
smaller reference's fastest, half the smaller reference's sinking and
depth where the references stand the scene, and a coarse bound where they
fail it (half a box, a quarter of one deep, 0.1 for a pyramid that stood
where theirs toppled); a bounce never 1% over e², nor a quarter further
under it than the lower reference. In 3D, Rapier's and Box3D's values set
the bounds and Jolt's, damped, is recorded beside them.

**What the tests catch** (planted, 2026-09-28, each in the source, the
suite run, the source restored):

| planted | 2D fails | 3D fails |
|---|---|---|
| friction halved (on each body; in 3D a quarter on the body, half through the geometric mean) | the box holding, the box sliding, the slipping disc, the ladder, the card house, the dominoes | the cube holding, the cube sliding |
| restitution ignored | the bounce, the bullets (no rebound) | the bounce |
| inverse mass squared (a heavy box heavier) | heavy boxes standing | heavy cubes standing |
| a disc as a ring (a sphere as a hollow one) | the rolling disc | the rolling sphere |
| the push-out cap ten times (30) | overlap recovery (and a solver unit test) | – |
| the speculative margin 0.01 | the bullets, at their check that the limit is 21 and 48 (and three narrowphase unit tests) | – |

The rolling disc survives friction halved (0.3 still rolls it) and
catches only its inertia; the bullets catch the margin only through the
check of the limit their speeds are chosen about.

**What the view showed.** On the settling scenes, nothing the numbers
hadn't: the 1000-body pile at step 400 is a pile in all four (ours, the
arrays, Box2D, Rapier), and the turning 20-wide pyramid stands in each. One
thing the numbers put differently: our pile's top stands 1.3 higher than
Box2D's and Rapier's (its median body 0.4-0.7), which is its shallower
overlap (at rest 0.014 deep at most, against their 0.057-0.067) over some
25 layers, not a looser pile. On the new scenes it showed how the worst
mass ratio fails: "ratio 1000 5" buckles sideways by step 60 and has two
light boxes under the floor by step 120 in ours, where Box2D's column lies
flat on it.

## Still at rest

**Status: built** (2026-09-27, get-emj.41-43; 2D as a setting). The three
known failures of [Quality as a test](#quality-as-a-test) were thought one
cause, a box rocking on its points. Instrumented (every body's motion and
every contact's points, ids and impulses, step by step, read from the
world), they are three:

- **Planks in 3D chatter on flickering manifolds** (get-emj.43). A plank
  at rest has contacts whose loaded points change from one step to the
  next and back: a vertex of the incident face lying on a side plane of
  the reference face is kept one step (its id the vertex) and clipped the
  next (its id the side and the plane), the same point under two ids; and
  a clipped polygon of more than four points reduces to two different
  fours in turn, a point carrying 0.08 of load jumping 0.21 across the
  plank every step or two, as the plank's rock moves which is deepest.
  Warm starting found 1188-1191 of 1191 points a step, so what feeds the
  rock is the moving support, not lost impulses: warm starting by the
  nearest point instead of by id changed nothing. Energy bursts came
  every 9-10 steps, a pile at 1e-7 a body where Rapier's and Box3D's are
  under 1e-11.
- **A short stack in 3D circles on its corners** (get-emj.42). The five
  cubes (each set off by up to 0.04, so the load sits off centre) circle
  as one column at 4.3 Hz, the top 3 mm round at 0.075, the floor
  contact's unloaded corner going round with it, energy steady at 1e-3 a
  body from step 400 to 1000. Ids and points stay put. It hangs on the
  stiffness against the floor: at 0.4 of the substep rate (120 Hz) it
  never stops; at 0.3 it decays (9e-8 a body at 1000); at 0.25 or 0.2 it
  rests at once. A cube on four points rocks at 1.22 times its contacts'
  rate (each point's stiffness is set at its own effective mass, and the
  four sum to 1.5 times the nominal along each rocking axis), so 0.4 puts
  the rocking at 0.49 of the substep rate, where Box2D caps static
  contacts at 0.5 and Box3D at 0.25.
- **A tall stack in 2D sways near its buckling load** (get-emj.41). The
  20-high turning stack bends as a column, each contact a rotational
  spring of m ω² d² / 2 (22 000 N m at 75 Hz), under gravity 20: its
  weight is 92% of the load that buckles such a column (q L³ = 7.84 EI),
  so its first mode is slow (a 12 s period) and keeps 0.6 of its energy a
  swing. The model predicts what was measured: at 60 Hz (0.2) it topples,
  as Box2D's does at 30 Hz; at 90 Hz (six substeps) it stands further
  from buckling, swings every 4 s and rests from 60. Rapier's sways too:
  up to 2.8e-4 a body over its last 200 steps against ours' 5.6e-4, and
  the 1.5e-7 the test's energy bound was ten times was the step where its
  swing turned. What differs is speed: ours passes 0.05 until 580,
  Rapier's from 220.

### What the others do (read in their fetched source)

- **Box3D** recycles a contact whose bodies barely moved (`b3CollideTask`,
  physics_world.c: "Keep anchors but update separation, same as
  sub-stepping. This eliminates jitter"): the manifold isn't found again
  until the pair may have moved 0.05 since it was (a bound on its
  translation plus its turn at its reach), its anchors carried with the
  bodies, its separation updated. Its reduction to four is a pecking
  order ("very important for contact point consistency across time
  steps"): the first point the one farthest along a fixed tangent, not the
  deepest, and each later candidate must beat the best by 5%. Contacts
  are at most an eighth of the substep rate, static ones twice that at
  half the damping. It warm-starts from the last substep's impulses, as
  Box2D and Rapier do; ours from their mean.
- **Rapier 0.36** keeps the arms of a contact whose pair barely moved, and
  in 2D, by default, solves a manifold's two normals together as a 2x2
  LCP (`solve_mlcp_two_constraints`, the `block-solver` feature, off in
  3D), as Box2D v2.4 did. It solves speculative points rigidly, noting
  that a softened touchdown "pumps tall stacks" (ours already does).
- **Jolt** keeps manifolds in its body pair cache while bodies barely
  move.

### The options, measured

Each run over more sizes than the tests, since settling is chaotic
(docs/lore): 3D plank piles at 14 sizes, 150-800 (and 14 more, 175-825,
for the finalists) and 8 of 900-2000, cube piles at 14, stacks 2-25 high;
2D turning piles at 11 sizes, 400-1400. "Chatter" is a pile over 1e-8 a
body at the end, "late" at rest after 604 (the planks' bound). Every
variant is behind `physics3d::Tuning` or the comparison's `arrays:rot/...`.

| 3D, turning | planks 150-800: chatter, late, median rest | planks 900-2000 | cubes 150-800: median rest | stack 5 |
|---|---|---|---|---|
| before (every pair found every step, static 0.4) | 9, 5, 341 | 4, 5, 772 | 200 | never |
| (1) static contacts at 0.25 (Box3D's cap) | 2 of 4 in the test | – | – | rests |
| (2) moving contacts at 0.15, or both at Box3D's 0.125 / 0.25 | 1 of 8 at Box3D's | – | – | rests, but the 20-high topples and piles sink past their bounds |
| (3) three relax passes | 1 of 4 in the test | – | – | rests |
| (4) eight substeps | 2 of 4 in the test | – | – | rests |
| (5) warm starting from the last substep (`carry=last`) | 11, 3, 340 | – | – | rests |
| (6) relax passes alternating direction | 1 of 4 never at rest | – | – | never |
| (7) Box3D's reduction as it is (measured, then removed) | 4 of 8 | – | – | never |
| (8) recycling at 0.02 | 0, 0, 263 | – | 209 | never |
| (8) + (5), at 0.02 / 0.03 / 0.05 | 1 / 0 / 0 of 28 (0.020 deep at 0.05) | at 0.03: 0, 2, 392 | at 0.03: 169 | rests |
| **(8) at 0.03 + (1), as built** | **0, 0, 298** | **0, 0, 288** | **200** | **rests** |
| (8) + (7) + (5) | 0 of 14, 0.022 deep | – | – | – |

Also no change on the stack: anchors to first order, inertia every
substep, the exact turn, friction in the push. Softer damping (ζ 5, and 5
for static contacts, as Box3D has it) moved piles either way, beyond the
noise. Stacks of 16-25 keep 1e-8 a body or more in every row, the 25
never rests: they sway, as 2D's (below), and their bounds allow it.

- **Recycling is what stills the planks**, and alone: a resting pair
  keeps its points, so nothing flickers to rock on. The pecking-order
  reduction halves the chatter without it and adds nothing with it;
  warm starting by position doesn't touch it.
- **The short stack wants softer floor contacts or a warm start from the
  last substep.** The last substep is what the references do, and the
  mean lags a rocking contact by two substeps; but with recycling it let
  piles of 1000 planks rest later (392 against 288, median; 701 at 1000,
  past its bound of 556), so it stays a variant (`Carry::Last`). Static
  contacts at 0.25 are Box3D's cap, and leave the rocking at 0.31 of the
  substep rate.
- **0.03, not Box3D's 0.05**: at 0.05 a carried plank point drifts, and a
  pile is 0.020 deep at rest (the bound is 0.015); 0.02 let one pile of 28
  chatter. Spheres aren't recycled: one point has no features to flicker,
  is found for less than it costs to carry, and a carried point on a
  rolling sphere rolls away from where it touches.

**Why this one.** Recycling and static contacts at 0.25 pass every 3D
quality test, the long ones included, which the ignored ones now are:
10 000 turning planks at rest from 335 (never before; Rapier 392, Box3D
369), 1000 from 324 (408). It is also faster. One thread, `-c opt`, the
bench's whole run, ours only, before and after (2026-09-27):

| turning | whole run ms | narrowphase µs | solver µs | at rest from | deepest | energy a body |
|---|---|---|---|---|---|---|
| boxes 1000 | 2.78 → 2.57 | 343 → 178 | 2282 → 2243 | 180 → 182 | 0.0071 → 0.0073 | 3.5e-11 → 5.8e-10 |
| boxes 10 000 | 30.3 → 27.0 | 3737 → 1763 | 24 245 → 23 365 | 374 → 468 | 0.013 → 0.013 | 9.4e-10 → 9.9e-9 |
| planks 1000 | 3.36 → 3.03 | 596 → 307 | 2561 → 2518 | 408 → 324 | 0.010 → 0.014 | 1.0e-9 → 1.1e-12 |
| planks 10 000 | 44.2 → 38.2 | 7130 → 3667 | 33 594 → 31 687 | never → 335 | 0.024 → 0.035 | 2.0e-6 → 6.1e-10 |
| spheres 1000 | 1.81 → 1.95 | 96 → 93 | 1552 → 1681 | never | 0.0070 → 0.0056 | 8.5e-7 → 7.0e-6 |
| spheres 10 000 | 21.7 → 22.0 | 1109 → 1012 | 18 212 → 18 425 | never | 0.017 → 0.016 | 1.2e-5 → 8.3e-7 |
| rain 10 000 | 20.1 → 19.9 | 2412 → 1884 | 14 877 → 14 976 | never | 0.0022 → 0.0049 | 8.5e-3 → 8.1e-3 |

Locked, boxes and planks 10 000: 29.8 → 29.1 and 21.1 → 20.3 ms (the
narrowphase 3857 → 2021 and 2481 → 1343 µs). The narrowphase halves on
piles, most pairs at rest carried; a contact's `Manifold` is 40 words,
not 31 (each body's rotation when its points were carried, and the bound
on the pair's move). Spheres, which no engine brings to rest (they roll),
move by a run's noise. Piles sink a little deeper against the floor (the
planks at 10 000 0.035, against Rapier's 0.076).

**2D: a setting.** Every option that stands the 20-high stack costs
something the default shouldn't pay without a decision (get-emj.41):

| 2D, turning | stack 20: at rest from, most energy a body in the last 200 | pyramid 5050: at rest from, most in the last 200 | piles 400-1400: median / worst rest | solver µs, pile 10 000 settled / pyramid 5050 |
|---|---|---|---|---|
| as built | 580, 5.6e-4 | 440, 2.3e-8 | 270 / 440 | 9312 / 7416 |
| six substeps | 60, 1.3e-4 | 340, 9.0e-11 | 220 / 340 | 10 913 / 9157 (+17%, +23%) |
| the block solver in the relax passes (`block=1`) | 240, 2.5e-4 | 1130, 1.2e-6 | 280 / 400 | 8343 / 7153 (−10%, −4%) |
| the block solver in the pushing pass too | 240 | – | the 10-high stack and a pile of 1000 past their energy bounds | – |
| warm starting from the last substep[^last-substep] | 240, 2.9e-4 | – | 250 / 540; a locked pile of 1300 never rests | – |
| three relax passes | 230 | 510 | – | +40% |
| contacts at 0.2 (60 Hz) or 0.125 | topples | – | – | – |
| Rapier | 220, 2.8e-4 | 1100, 8.2e-8 | – | – |

Six substeps get better everywhere, and cost a fifth of the solver in
every scene, locked or turning (rain 10 000 2410 → 2949 µs). The block
solver is cheaper than solving the points one after the other, but sets
the big pyramid vibrating for a thousand steps, whole rows at 0.3 (as
Rapier's, whose block solver is on in 2D, rests at 1100 too). The block
solver stays a variant.

**Decided** (2026-09-27, get-emj.41): the substep count is a setting in
the world, `physics::Tuning` on one entity as 3D's `Tuning` is, read by
the solve every step; none, or 0, is the default five. That is what both
references do (Box2D's 4 substeps and Rapier's 4 iterations are each a
default and a knob), and it costs no game what it didn't choose. The
20-high stack's test runs at six through it, on the mod in the engine
(`a_twenty_high_stack_that_turns_rests_as_soon_as_rapiers_at_six_substeps`,
at rest from 60), and `the_mod_solves_at_the_substeps_its_world_sets`
holds the mod at six to the arrays at six bit for bit, and apart from them
at five; with the setting ignored, both fail (the stack at rest from 580).
The arrays (`tests/arrays.rs`) solve at the default only, and refuse a
world with another. The default is to be revisited once the solver's
speed work has landed, on the new base.

**The tests.** A stack's or pyramid's energy is now bounded by the most at
any look over its last 200 steps (`Settling::energy_tail`), from the
references' same measure: at one step a swaying scene says where in its
swing it was. Only the swaying references' values moved (the turning
stacks). 3D keeps its energy at the end: its references are still.
Planted, each fails what it should: recycling off (`recycle=0`), the
planks' energy (1.5e-7) and rest; static contacts at 0.4, the five-high
stack; a recycled point's separation grown three times too fast,
`a_settling_box_keeps_its_contact_and_a_sliding_one_is_found_again`,
`a_box_stack_stands` and every 3D pile; recycling that never ends, four
physics3d tests; the block solver's coupling halved, its unit test.

## The solver's speed

**Status: built** (2026-09-27). With bodies turning, the 2D solver was 2
to 4 times Box2D v3.1.1's and Rapier 2D 0.36's (8215 µs against 2626 and
3999 on the settled turning pile of 10 000, 7324 against 1725 and 2425 on
the 5050 pyramid; [Against other engines, bodies
turning](#against-other-engines-bodies-turning)). It now solves four
contacts at a time, grouped by level of the sweep in pair order, which is
the same computation as before bit for bit, and takes about half the time:
within 10% of Box2D's at Box2D's number of passes, and 1.7 to 1.8 times it
at ours, which are a quality choice ([Settling](#settling), [Still at
rest](#still-at-rest)). Nothing the quality tests bound moved, since
nothing moved at all. Graph coloring, the order threads would share, let
a turning 5050 pyramid fall; that turned out to be a flaw in how a step
warm-starts turning contacts, found and fixed on 2026-09-28 (get-emj.48,
[Why colors let the pyramid fall](#why-colors-let-the-pyramid-fall)),
which did move results, and for the better.

### Where the time went

The solver alone, on the same inputs, timed by stage
(`./bazel run -c opt //engine/std/physics/compare:solver_bench`, which
captures the solver's input from the comparison's turning scenes at the
steps it times; the stages by clocks put in a copy, since removed). µs
per step, one thread, the turning pile of 10 000 (22 136 contacts, every
one with points) and the 5050 pyramid (14 950):

| stage | passes a step | pile, one at a time | pyramid, one at a time | Box2D's, pile (`b2Profile`) |
|---|---|---|---|---|
| preparing contacts | 1 | 1000 | 388 | 211 |
| warm start | 5 (Box2D 4) | 493 | 345 | 214 |
| the pushing pass | 5 (4) | 1645 | 1331 | 935 (with friction) |
| moving bodies | 5 (4) | 103 | 52 | 152 |
| relaxing passes | 10 (4) | 5311 | 5268 | 934 |
| summing the substeps' impulses | 5 | 237 | 158 | – |
| restitution, storing impulses | 1 | 141 | 37 | 149 |
| **all** | | **9255** | **7550** | **2596** |

- **It was latency, not arithmetic.** A relaxing pass was 24 ns a contact
  on the pile and 35 on the pyramid, about 120 to 175 cycles. A contact's
  four impulses (two points' normals, then their friction) each read the
  velocities the last one wrote, and in pair order the next contact
  shares a body with this one (pairs sort by their first body; a pyramid's
  chain of contacts is the order), so every contact waited on the one
  before. Box2D's pass is 44 ns a batch of four.
- **Box2D** (read in v3.1.1's `solver.c`, `contact_solver.c`,
  `constraint_graph.c`): contacts colored as they begin touching and kept
  in their color (12 colors, the last an overflow solved one at a time;
  a contact with a static body never in color 0), each color's contacts in
  batches of 4 (SSE2) or 8 (AVX2) laid out field by field
  (`b2ContactConstraintSIMD`, a one-point contact's second point zeros,
  a color's last batch padded), bodies one array of 32-byte states
  (`b2BodyState`: velocity, turn rate, move and turn this step) gathered
  and scattered by transposes, inverse masses kept by each contact. A step
  is one prepare, 4 × (warm start, pass, relax), restitution (skipping
  batches with none), storing: 12 passes to our 20.
- **Rapier 0.36** (its fetched source; a staged island solver, not the
  upstream `VelocitySolver`): contacts colored in the narrowphase too,
  by `u128` masks per body (dynamic pairs from the lowest color, pairs with
  a fixed body from the highest), each color cut into chunks of 4
  (`wide::f32x4` through simba) with body velocities gathered by
  transposes; 4 substeps × (update and warm start, one biased pass
  without friction, one unbiased with it), restitution once if anything
  bounces: 12 passes.

### The options, measured

Each on the same captured inputs, one at a time before combining; µs per
step, the median of 9 solves of each of 3 inputs; "bit for bit" is every
body's velocity and turn rate against one contact at a time in pair order:

| option | pile 10 000 | pyramid 5050 | bit for bit |
|---|---|---|---|
| one contact at a time in pair order, as it was | 9104 | 7470 | – |
| the lanes' layout alone: batches of one, pair order[^pair-lanes] | 10 335 | 6434 | yes |
| by level, 1 lane | 8793 | 5475 | yes |
| **by level, 4 lanes (built)** | **4485** | **3075** | **yes** |
| by level, 8 lanes (two SSE2 registers) | 4522 | 3139 | yes |
| graph-colored as Box2D, 1 / 4 / 8 lanes | 8748 / 4356 / 4243 | 5456 / 2937 / 2782 | no: another order[^colored-fell] |

Then, each alone on 4 lanes by level, bit for bit unless said:

| change | effect |
|---|---|
| restitution skips a batch no lane of which can bounce, before gathering | 189 → 38 µs of restitution on the pile |
| the warm start's velocities stored in the batch before scattering | the pile 4915 → 4628 µs: the stores seed LLVM's SLP vectorizer, which had left the warm start scalar (80 `mulss`, 16 `mulps`; after, none and 28) |
| the relaxing passes' bias found by the substep's first and read by the second (positions don't move between them) | −2.5% on the pile, −3.6% on the pyramid |
| `prepare` inlined | the prepare's loop 642 → 417 µs |
| **rejected:** the same stores in the passes, which LLVM vectorized already | no gain (4631 → 4714) |
| **rejected:** batches filled one after another, reading contacts in the order they're solved | the prepare 887 → 1223 µs: reading them in pair order and writing to their batch is cheaper |
| **rejected:** hot fields apart from cold ones | not built once batches padded by 128 bytes (17%) measured the same (4915 → 4868): the passes aren't bound by memory |
| six substeps (a `Tuning`) | +17% (5226, 3583) |
| one relaxing pass | −26% (3337, 2289); fails the quality tests ([Rotation in the soft step](#rotation-in-the-soft-step)) |
| Box2D's passes: 4 substeps, 1 relaxing | 2822, 1910 to 2247 against Box2D's 2596 and 1725: within about 10% like for like; fails on depth and rest |

- **Levels, not colors.** The sweep in pair order makes a contact wait
  only on earlier contacts sharing a body it moves. A contact's level is
  one past the latest such contact's (in pair order, the level its bodies'
  last contact left, so one pass finds it), and solving the levels in
  turn, each contact sees the bodies exactly as the sweep would have left
  them: it is the sweep bit for bit, with no two contacts in a level
  sharing a moving body, so a level's contacts can go in lanes. This is
  level scheduling, as sparse triangular solves run in parallel (Anderson
  and Saad, 1989). A pile of 10 000 has about 420 levels, a 5050 pyramid
  590, and their batches are 97% and 94% full. Colors are 6 or 7 and
  solve 3% faster, but in another order, so another computation. Since
  get-emj.48 colors pass every quality test, the long ones included, but
  pair order is still the better order for a pile or a pyramid: it walks
  them row by row from the ground, which colors scatter, and settles the
  5050 pyramid from 450 where colors take 1500 ([Why colors let the
  pyramid fall](#why-colors-let-the-pyramid-fall)).
- **Four lanes, not eight**: SSE2 is x86-64's baseline, and Box2D was
  measured at SSE2; eight are two registers each, 1% slower on the pile
  and 10% slower on the pyramid of 210.
- **Plain arrays, no intrinsics and no unsafe**: `lanes::F` is an array
  of `f32` with each operation a loop over the lanes, and LLVM made
  SSE2 of it (checked in the disassembly: the relaxing pass 110 `mulps`,
  no `mulss`), but only where a vector store seeds it, which is what the
  warm start's stores are for. `std::simd` isn't stable; `std::arch`'s
  loads and stores need `unsafe`.
- **The layout** is Box2D's: bodies copied each step into one array of
  32-byte states (velocity, turn rate, move and turn this step), their
  masses kept by each contact; contacts in batches, field by field, a
  one-point contact's second point and a batch's empty lanes zeros at a
  body nothing moves. The copy is the solver's own layout for the step, as
  decided in [What the ECS costs](#what-the-ecs-costs); nothing persists
  outside the world.
- **A world where nothing turns keeps the loop one contact at a time**:
  it is level with Box2D already, and a lanes kernel adds a row's zero
  turns, which isn't that loop to the bit the games' replays hold to
  (get-emj.51). `Wide::Off` keeps the loop for turning contacts too: the
  variants that only it has (the block solver, other separations) and
  the test that the lanes are it bit for bit.

**The tests.** `the_solve_by_level_is_the_solve_one_contact_at_a_time_bit_for_bit`
(`:quality_test`) holds the default to the loop bit for bit on a turning
pile and pyramid over 150 steps, and checks that colored differs, so an
order is visible to it. Planted, it fails on a level off by one (every
turning quality test fails with it) and on one sum in the pushing pass
reassociated (nothing else sees that). The long quality tests take 24 s
where they took 38.

### Against the others now

The comparison (runbook 005), before and after, the same session, one
thread, `-c opt`, medians of 3 runs; each cell the step, then the solver
stage, µs (the ECS mod; the arrays' solver agrees within 2%):

| scene, turning | ours before | ours after | Box2D | Rapier |
|---|---|---|---|---|
| pile 1000, settled | 953 / 732 | 661 / 457 | 380 / 256 | 383 / 320 |
| pile 10 000, falling | 3116 / 1870 | 2347 / 1110 | 2749 / 642 | 2589 / 1011 |
| pile 10 000, settled | 10 598 / 8463 | 6639 / 4525 | 4452 / 2665 | 4901 / 3991 |
| pyramid 210 | 368 / 294 | 216 / 141 | 111 / 72 | 106 / 93 |
| pyramid 5050 | 8774 / 7334 | 4598 / 3094 | 2974 / 1758 | 2865 / 2489 |
| rain 10 000 | 8606 / 5806 | 6334 / 3502 | 4997 / 1969 | 6593 / 2394 |

Locked, where the code didn't change, within the runs' noise (the pile of
10 000 settled 2415 → 2474 µs of solver, the pyramid 2202 → 2247). `:tax`,
ECS / arrays: the turning pile of 10 000 settled, frame 10 568 / 11 338 →
6750 / 7825, solver 8399 / 7999 → 4576 / 4487, bit for bit as before; 1000
settled, 935 / 1047 → 680 / 787. The solver is now 1.1 to 1.3 times
Rapier's and 1.7 to 1.8 times Box2D's where contacts press, from 20 passes
to their 12 and a prepare twice Box2D's (get-emj.50); the step is 1.5 times
Box2D's on the settled pile, where the narrowphase and upkeep are ours.

**What it means for threads** (get-emj.32, get-znt.5): the levels are
the one-thread schedule. 420 to 590 of them would be as many barriers a
pass, where a barrier costs 0.19 µs on one CCD ([Parallel
solving](#parallel-solving)): 80 to 110 µs a pass, more than the pass. The
parallel solve is colors, whose batches run the same lanes and layout;
only the grouping differs. Colors are another order, and until get-emj.48
they let the big turning pyramid fall; now it stands, later to rest than
in pair order (get-emj.54).

**3D** (engine/std/physics3d) has the same structure, rows in pair order,
and the level schedule applies to it unchanged. Alone, rows reordered by
level one at a time, it gained 1 to 2% (boxes of 10 000 turning 25 072 →
24 793 µs of solver, planks 32 514 → 31 841, bit for bit), so it wasn't
landed: the gain is the lanes, and 3D's kernel is its own (four points,
friction on a disc at the centroid, twist, the `Tuning` variants), a
port, not a change (get-emj.52).

### Why colors let the pyramid fall

**Status: found and fixed** (2026-09-28, get-emj.48). Solved in Box2D's
colors, a turning 5050 pyramid never came to rest and came apart, its top
5 lower and boxes leaning 37°, where Box2D's pyramid, solved in the same
colors, stands. It wasn't the colors. A turning contact's points began
each step from the average of the last step's substeps' impulses, where
Box2D's begin from where the last substep left them; the average lags
two substeps behind, and every order but pair order leaned on it. The
points now carry out their last substep's impulse (times the substeps,
so the next step's share of it is that impulse), and colors stand the
pyramid and pass every quality test.

**Any order but pair order, not colors.** `arrays:rot/order=N`
(`variants.rs`) solves the contacts in another order one at a time (the
solve by level is the sweep of whatever order it is given, bit for bit):
reversed, shuffled afresh each step, or row by row from the top down.
Turning unless said, before the fix, at rest from (and the top box's
move):

| order | pyramid 20 | pyramid 50 (1500 steps) | pyramid 100 (2500 steps) |
|---|---|---|---|
| pair order (by level) | stands (0.011) | 160 (0.065) | 440 (0.26) |
| colored | stands (0.011) | never: the top slid 1.29 | never: the top 5.06, boxes 37° |
| reversed | stands (0.012) | 730: the top slid 0.75 | never: flies apart |
| shuffled each step | stands (0.011) | 680 (0.065) | never: the top slid 0.73 |
| rows from the top down | stands (0.013) | 660: the top slid 0.20 | never: the top slid 2.65, boxes toppled |
| locked: pair / reversed / shuffled / top down | – | 60 / 60 / 80 / 190 (700 steps) | 150 / 150 / 260 / never |

- **It needs size and stiffness, not rotation.** Small pyramids stand in
  every order; the worse the order, the smaller the one that falls.
  Locked, only rows from the top down fail, which in a stack's first
  pass carry nothing down (the top pair both fall at the same speed);
  turning adds a way to fail (a box sliding on the one below), which is
  what colors and shuffling found. Ten substeps (as stiff, since
  stiffness is a share of the substep rate) fell as five.
- **It starts at once.** The pyramid starts flush and sinks onto soft
  contacts under its weight, so in the first steps its top bounces at 1
  to 2 a second in every order. Pair order has it down to 0.04 by step
  100 on the 50 pyramid; colored, reversed and shuffled keep it at 0.5 to
  1.7, and it turns into boxes sliding off each other.
- **What kept it going** (the substeps' normal impulses summed over the
  pyramid, each substep's total against the step's warm start, printed
  step by step): the pyramid breathes, slowly (a period of about 40
  steps), and through the half of a swing where it presses harder, every
  step's substeps climb the same way: colored, steps 304 to 315 each
  ended 1.13 to 1.25 times where it started. The next step then starts
  from their average (at step 307, 1.15 times the last start, where the
  substeps had got to 1.25): 8% of the load thrown away at every step, a
  lag of two substeps in the loop that carries a pyramid's weight from
  step to step.
  Pair order is a pyramid's best order: one pass from the ground up
  carries support to the top, so each step nearly converges and the warm
  start matters little (the gap between the last substep and the average
  falls to 0.2% by step 400). Any other order carries it a row or two a
  pass and leans on the warm start, whose lag let the breathing grow.
- **Box2D** (v3.1.1, read): `b2StoreImpulsesTask` stores each point's
  accumulated impulse as the last relaxing pass left it (`normalImpulse`,
  `tangentImpulse`; the step's sum, `totalNormalImpulse`, is kept apart
  for reporting), and `b2PrepareContactsTask` starts the next step from it
  at full scale, so the accumulators run on across steps as within them.
  3D measured the same choice as `Carry::Last` ([Still at
  rest](#still-at-rest)), and noted the mean's lag of two substeps.

**What else differs from Box2D, checked** (in its `solver.c`,
`contact_solver.c`, `constraint_graph.c`, `world.c`), none the cause:

- *Overflow*: solved first, one contact at a time, as Box2D's; with 64
  colors none of these scenes overflows (they take 6 or 7).
- *Static bodies take no color*, and contacts with one aren't in color 0;
  the lowest free color and Rapier's highest were measured too, and none
  stood the pyramid (get-emj.48's first round).
- *Warm starting* per color, after the substep's gravity and before its
  pushing pass; *relaxing* after positions move; *restitution* once after
  the substeps: Box2D's stages, in its order.
- *Anchors*: fixed for the impulses, turned with the bodies for the
  separation, as `b2SolveContactsTask`.
- *The push split across substeps*: one soft pass with `useBias`, rigid
  relaxing passes without, as Box2D's, but ours relaxes twice and has
  friction only in the relaxing passes. Box2D's friction in the pushing
  pass too was measured and is worse: pair order falls with it (the top
  slid 2.1) and colors fly apart.
- *Clamping*: the accumulated impulse, never a pass's, in both.
- *Softness*: Box2D caps moving contacts at an eighth of the substep rate
  and static ones at a quarter (`world.c`, `contactHertz`); ours are at a
  quarter and a half. This is the rest of the order sensitivity (below),
  not its cause.

**The options, measured**, turning (the 5050 pyramid over 2500 steps;
turning piles 41 wide at 11 sizes, 400-1400, over 700; `:quality_test`
and `:quality_long_test` with `SOLVER=`):

| option | 5050, pair order: rest, top | 5050, colored: rest, top, lean | piles 400-1400, pair / colored: median, worst | quality tests, pair / colored |
|---|---|---|---|---|
| as it was | 440, 0.26 | never, 5.06, 37° | 270, 440 / 270, 700 | pass / fail: the 5050, two piles' energy |
| **points from the last substep (built)** | **450, 0.25** | **1500, 0.30, 0.4°** | **220, 330 / 230, 380** | **pass / pass** |
| Box2D's stiffness (an eighth and a quarter) | 300, 0.89 | 590, 0.91, 2.4° | – | the 5050's top past its bound (0.73) |
| Box2D's stiffness and passes (4 substeps, 1 relax) | – | 1140, 1.54, 1.5° | – | as above |
| three relaxing passes | – | 1520, 0.30, 0.7° | – | – / fail: the 5050's energy, pile 10 000 never at rest; +25% time |
| friction in the pushing pass (Box2D's) | never, 2.14 | flies apart | – | – |
| ten substeps | – | never, 2.69 | – | – |
| the fix + three relaxing passes | – | 340, 0.24, 0.3° | – | – / fail: pile 9000 never at rest |
| the fix + stiffness 0.2 and 0.4 | – | 420, 0.37, 0.5° | – | fail: locked scenes past their bounds |
| the fix + static contacts at a quarter | – | 1360, 0.30, 0.4° | – | – / pass |
| the fix + the second relaxing pass in reverse order | 830, 0.25 | 930, 0.26, 0.4° | – | – |

- **The fix is the warm start alone.** It costs nothing (a multiply a
  point at the end of the step: `:solver_bench`'s pile of 10 000 4416 →
  4482 µs, pyramid 3042 → 3042, noise), and moves only turning contacts'
  points. A contact whose ends don't turn keeps the average, so a world
  where nothing turns is bit for bit as before (the games' replays,
  get-emj.51); it helps there too, measured with a flag since removed,
  and is get-emj.55. `Constraint::jn`, a contact's impulse over the
  step, stays the sum (what pressing and sleeping read).
- **What the tests bound moved, all within bounds, most for the better**
  (pair order, turning): piles 400-1200 at rest from 230, 220, 400, 440,
  290 → 200, 220, 210, 250, 290, their energy at the end 1.6e-7 → 1.6e-8 a
  body at worst, deepest while settling 0.44 → 0.54 (bound 0.66); piles
  9000-11 000 330, 350, 1620 → 370, 310, 360; pyramids 15-25 10, 20, 30 →
  10, 20, 20; the 20-high stack at six substeps (the mod) 60 → 30, and at
  five 580 → 240. Locked: nothing moved.
- **Order still matters, less.** Reversed still brings the big turning
  pyramid down (from the top down it rests at 790, its top slid 0.77,
  past the bound), from the top down the locked one never rests, and
  colors settle it three times later than pair order. What's left is stiffness against passes:
  Box2D's softness or three relaxing passes stand every order, but sink
  the pyramid past its bound or keep a big pile moving. Relaxing
  symmetrically (the second pass back through the colors, as parallel as
  forward) helps colors and hurts pair order. The stiffness is a quality
  choice ([Settling](#settling)); get-emj.54 has the rest.

**Colors as the default, or the parallel path?** Measured, not decided:
colored now passes every quality test the default does, the long ones
included, and is 3-5% faster at four lanes (pile 10 000 4338 against
4482 µs, pyramid 2881 against 3042) and 8-11% at eight (4137, 2709). But
it rests the 5050 pyramid at 1500 against 450 (Box2D 160, Rapier 1100;
the bound 2200), the turning piles a little later at worst (380 against
330), and pyramids 40 and 50 wide later than their bounds allow (40 at
250 against 220; 50 keeps 7.8e-6 a body against 8.1e-7): an ignored test,
`a_pyramid_that_turns_stands_when_its_contacts_are_colored`, and
get-emj.54. For threads, colors are now a solve that stands; as one
thread's default, pair order settles better for 3% more time.

**The tests.** `a_pyramid_that_turns_stands_whatever_order_its_contacts_are_solved_in`
(`:quality_test`) holds a turning pyramid 50 wide to the references'
bounds in pair order, shuffled and from the top down;
`a_big_pyramid_that_turns_stands_when_its_contacts_are_colored`
(`:quality_long_test`) the 5050 pyramid colored. Planted, the average
back: shuffled rests from 680 and top down from 660; the last substep's
impulse without the factor of the substeps: every turning quality test;
the fix in the lanes and not in the loop one contact at a time:
`the_solve_by_level_is_the_solve_one_contact_at_a_time_bit_for_bit`.

## Open questions

- **Rotation**: built, in 2D ([Rotation](#rotation)) and 3D ([Rotation
  in 3D](#rotation-in-3d)).[^rotation]
- **Kinematic characters.** The platformer's player as a dynamic body with
  no friction is the simplest thing that works; a dedicated character
  controller (slopes, steps, one-way platforms) is the usual next step and
  waits for a game that needs it.
- **Tunneling.** No continuous collision: a body moving more than its own
  size per step can pass through a thin collider. Pong's ball tops out at
  40 cells/s, 0.67 cells a step against paddles a cell thick, which is
  within it; substepping is the cheap fix if a game needs more. Measured
  since ([Quality beyond settling](#quality-beyond-settling), get-emj.59):
  a ball is stopped for certain while a step is at most the margin, its
  radius and half the wall, 0.8 for pong's (48 cells/s), and pinned by a
  test at that limit.

## Spike results

2026-09-23, `spike/physics` (since landed as `//engine/std/physics`,
whose tests are the spike's): the mod as designed, on the real engine,
with a stress demo (`pile`) and a platformer in miniature (`runner`: a tile
floor with a pit, a running and jumping player, a coin, a walker that
turns at ledges with a spatial query). `./bazel run -c opt
//engine/std/physics:bench` prints the numbers below; `./bazel run
//engine/std/physics:pile_game` runs the pile to play with over `modctl`.

**What held up:**

- **The solver hot-reloads under a running simulation.** Reloading
  `physics` mid-pile (in the integration test, and by hand with `./bazel
  run //spike/physics:physics_v2` and back, and in `//game` with gravity
  flipped and restored) moves nothing, keeps the
  contact cache and its warm-start impulses, and the pile goes on to
  settle. Nothing about physics needed the loader.
- **Deterministic.** The same drop settles to the same positions on every
  run.
- **Spatial queries fit the query API.** `Spatial<Data, Filter, Changes>`
  is about a hundred lines in physics's interface, over a tuple of two
  queries; the group parameter it needs (`ParamDecl::Group`, tuples of
  parameters) is thirty in `engine_ecs`, with conflicts and footprints
  seeing through it (checked by mutation). The walker's ledge check is
  one line, and rows, `Changes` and conflicts behave as for `Query`.
- **Fast enough.** Framework and all, on one thread:

  | bodies | falling | settled |
  |---|---|---|
  | 250 | 0.06 ms/frame | 0.07 ms/frame |
  | 500 | 0.12 | 0.14 |
  | 1000 | 0.21 | 0.33 |

  Contacts (broadphase and narrowphase) and the solver take about 40%
  each, the index most of the rest. Nothing is tuned.

**Sharp edges found, and what was done:**

- **Bodies snag on the seams between tiles.** A box running along a row of
  tiles meets the next tile's corner, flush with its top, and the axis of
  least overlap calls its side a wall. Where two boxes are flush on one
  axis and neither is inside the other, the narrowphase now takes the face
  the box is sliding along, from the pair's relative velocity: that
  separates running along a floor from falling along a wall, which have
  the same geometry. "Flush" has to be both ways: resting exactly on a
  floor, rounding puts the overlap a hair either side of zero.
  Tile-based games may still want adjacent tiles merged into one collider;
  this makes it an optimization, not a fix.
- **Stacks never settled.** Closing speeds were read contact by contact,
  after warm-starting the contacts before them, so contacts deep in a
  stack saw their neighbors' impulses as their own speed and bounced on
  them. They're read from the velocities before any impulse now. Pinned
  by a ten-body column in `core_test` and the pile tests; with warm
  starting or the split impulse removed, the pile doesn't settle either.
- **A contact "began" one step early, or never.** A speculative contact
  is held before its bodies touch, so "not held last step" is the wrong
  test for a new contact; it's "not pressing last step".
- **`Contact` is per pair**, so a player running across tiles begins a
  contact with each one. "Landed" is `Touching::below` turning true, not a
  `Contact`; the games should read it that way.
- **Solving and moving are one system.** The split impulse's pseudo
  velocities exist only inside the solve, so positions are integrated
  there; the pipeline is four systems (`integrate_velocities`,
  `find_contacts`, `solve`, `publish_index`), not the five the design
  implied.
- **Queries have no optional terms.** A collider may or may not have a
  `Body` or a `Velocity`, so the step reads them through two queries
  (`With` and `Without<Body>`) and looks velocities up per entity. An
  `Option<&T>` term would be the ECS's fix.
- **Bundles stopped at four components**, fewer than a physics body with a
  marker. They go to eight now.
- **A spatial query before the first step sees nothing**: the index is
  published by the step. The walker turned on its first frame. A game that
  queries on its first frame needs the index built at load.
- **A mod's code can't be a separate library shared with its interface.**
  The narrowphase and solver use the interface's shapes, but a second
  build of the mod gets its own copy of the interface crate, so a library
  depending on the first copy links two `physics` crates into one mod.
  The pure modules are compiled into the mod, and into their test crate,
  instead.
- **Tall piles converge slowly.** A thousand bodies, 32 rows deep, aren't
  all at rest after five seconds at eight iterations. More iterations, or
  letting resting bodies sleep, when a game needs it.

## The games on it

2026-09-23. Both games moved onto `//engine/std/physics`, and every
recorded route and replay in `platformer_test` and `pong_test` passes
unchanged: the winning run still wins on frame 255, the stomp and the
walker's kill land on the same frames, and pong's replay still ends on
frame 508.

- **The platformer**: tiles are static boxes, spikes, the goal and coins
  sensors whose `Trigger`s the rules read in `late`, the level's sides two
  tall walls (the old collision code treated off-map as solid), and
  walkers bodies that collide with tiles only. A walker turns at a wall
  from `Touching` and at a ledge with a `Spatial<&Tile>` point query, and
  meets the player by overlap, so a stomp and a touch are told apart
  before either pushes the other. The rules no longer rebuild a tile map
  every frame.
- **Pong**: the ball is a circle with restitution 1 and the paddles
  kinematic boxes; physics does the bounce, and pong adds its speed-up and
  spin on the paddle's `Contact` and scores on a goal line's `Trigger`.
  Everything the ball meets stands a radius back from the lines the court
  is drawn by, so the ball's center turns where the old point ball's did.

**What the ports showed:**

- **A system couldn't read a component on some entities and write it on
  others.** The walkers write walkers' velocities and read the player's;
  pong moves the ball and reads the paddles' positions. The conflict check
  now takes two queries as apart when one requires a table-stored
  component the other excludes (`With<Walker>` and `(With<Player>,
  Without<Walker>)`), as Bevy does: their guards are then on different
  tables. A shared sparse component still conflicts, since its set is one
  guard.
- **Static sensors reported static walls.** Pong's goal lines overlap the
  walls across their ends, and each overlap was a `Trigger`, a point for
  nobody. A sensor pair now needs one collider that can move.
- **Queries had four terms at most**, and the platformer's text interface
  reads five. They go to eight, like bundles.
- **The first frame has no spatial index**, as the spike found: a walker
  set off the wrong way. Walkers start without looking; the index built at
  load is still to do.

[^grid]: *(History, 2026-09-24.)* The broadphase was a uniform grid built
    from every collider each step, and the step's last system,
    `publish_index`, built a second grid as the `SpatialIndex` component
    that spatial queries read: shapes as of the last step, empty on the
    first frame, and invisible to the scheduler as a dependency on
    positions. Both went when positions became a spatial key.

[^onestep]: *(History, 2026-09-24.)* Physics first stepped once per frame
    by `Clock::dt`, capped at 1/30 s, since systems ran once a frame: a
    real-time game's simulation depended on its frame rate, which only
    lockstep hid. Fixed-rate phases replaced it.

[^dead-test]: *(History, 2026-09-24.)* Before sleeping was storage, the
    solve looked each contact's ends up to skip resting ones, and did so
    only while something slept: the test is always false when nothing
    does, yet made per contact in the gathering walk it cost 30 µs of 75
    at 10 000 settled, for a reason not found (measured, not read in the
    assembly). The walk was split on it.

[^sleep-counts]: *(History, 2026-09-25.)* Until then physics counted:
    the sleeping bodies in the world against its own count, to see one a
    game despawned or woke, and the statics against the last step's, to
    see one despawned. A static spawned wasn't seen at all (a spawned
    value wasn't written then), a count is fooled by one gone and another
    come in the same step, and the count's repair put back to sleep the
    rest of an island it had just woken, when a game woke one body by
    removing its `Asleep`. It looked from the tick after the solve, so a
    pre-solve hook's writes were missed, and a wake found in
    `find_contacts` took effect from the next step, the bodies immovable
    for the rest of the one that found it. Before that, sleeping was a
    lookup per body (the prototype); against storage, µs asleep / awake,
    medians of three: at 1000 (40 wide) the frame 51 / 134 and 9 / 133; at
    10 000 (400 wide) the frame 506 / 1349 and 22 / 1340, gravity 20 / 20
    and 8 / 20, the broadphase 149 / 146 and 2 / 160, writing back 61 / 66
    and 1 / 65. Measured before pages were made blocks of the order: at
    1000, 56 / 140 and 10 / 142; at 10 000, 580 / 1472 and 26 / 1503, the
    broadphase 208 / 222 and 2 / 243.

[^sleep-default]: *(2026-09-25.)* Turned on by default after both games
    were checked with it: pong's ball never sleeps (it never goes slower
    than 16 a second) and its paddles are kinematic, which never do, so in
    2500 frames nothing slept and the game's state was the same at every
    look, on or off. The platformer's player standing still sleeps one step
    in 32 (see below) and runs and jumps in the frame it's told to, on the
    same trajectory as awake; every recorded route passes unchanged.
    `pong_test`'s `nothing_in_pong_falls_asleep` and `platformer_test`'s
    `a_player_asleep_jumps_in_the_frame_it_is_told_to_as_it_does_awake`
    pin both (each catches a mutation: kinematic bodies let sleep; the
    jump without the step's gravity, or not waking the player). Off
    by default, from 2026-09-24, while its gaps were open.

[^touching]: *(History, 2026-09-25.)* Waking at once every island a
    resting contact joins to a woken one, transitively (as Box2D's islands
    are one per touching pile), was tried: a real pile that woke then
    never slept again, in 3000 steps. Each island that fell asleep was
    pressed by one not yet still for `Sleep::time`, which woke it and so
    all the islands it touched, resetting each body's time still. The
    bottom row's speeds in the step the floor went, which the lag was
    meant to fix, were the awake pile's already (0.13 to 3.9 a second,
    the warm start of the contacts above solved without the floor), so it
    was taken out.

[^sleep-reload]: *(History, 2026-09-25.)* `Sleepers` was rebuilt from the
    world at every load, so a reload restarted each awake body's time
    still and it fell asleep `Sleep::time` late: the platformer's player,
    standing at the start with physics reloaded every frame, never slept.
    The reload replays found it; `a_reload_keeps_how_long_awake_bodies_have_been_still`
    in //engine/std/physics:physics_test pins it.

[^sleep-adopt]: *(History, 2026-09-26.)* A build handed the copy also
    adopted everything the world had asleep, as the first build does, so
    a body a game put to sleep between frames (a message) was physics's
    own to the next step, which found its values written since and woke
    it: only if a reload came between. Found while measuring whether the
    copy could move into the world (get-emj.40).

[^sleep-copy]: *(History, 2026-09-24 to 2026-09-26.)* Until get-emj.40,
    sleeping's bookkeeping was `Sleepers`, the mod's copy of who's asleep
    by entity index, with how long each awake body had been still (in
    seconds), the transient part the step borrowed; the old build handed
    it to the new one through the mod's state (`unload`, then `load`),
    and a build without one rebuilt it from `Asleep` in the world, losing
    the times still. Removed because both halves went into the world
    (`Still`, `Slept`) at about the cost measured below with runs; the
    measurements that had kept it a copy follow, as they were written.

    **Why the copy is per-entity data outside the world** (2026-09-26,
    get-emj.40): it holds two things a step carries, and neither moves into
    the world for free.

    - *Who's asleep, as physics last saw it*, is the baseline a game's
      changes are found against. The world has who's asleep now; a body a
      game despawned, or woke by removing its `Asleep`, isn't in it, and its
      island wakes only because the copy remembers it. Rebuilt from the world
      at a reload that followed such a change, the island stayed asleep (999
      of the real pile's 1000, against 1 without the reload). So the handoff
      stays whatever happens to the times below.
    - *How long each awake body has been still* would be a component, as
      Box2D keeps `sleepTime` on each body (`b2Body`) and Rapier
      `time_since_can_sleep` on each body's activation, and it would survive
      a reset state too. Awake islands aren't kept (they're found afresh each
      step), so there's no island to keep it on.

    But a settling pile's bodies cross `Sleep::speed` all the time: at 10 000
    in a real pile (401 wide), about 400 a step while it falls and 1000 while
    it settles. Each shape was built as a prototype written only when a body
    crosses (the copy still holding the times, so this is the least each
    costs): `Still { since }`, the step it went slower, sparse and only on
    bodies slower than the threshold (an insert or a remove a crossing), or on
    every awake dynamic body (a column in each body's table, written a
    crossing). `:tax -- sleeping`, µs a step, sleeping on / off, medians of
    three runs on a quiet machine, 60 steps from the step given:

    | 10 000, 401 wide | the copy (now) | sparse `Still` | `Still` on every body |
    |---|---|---|---|
    | falling (from step 1) | 985 / 872 | 1033 / 886 | 1071 / 887 |
    | settling (from step 60) | 1796 / 1658 | 1895 / 1673 | 1904 / 1666 |
    | falling asleep (from step 120) | 1848 / 1614 | 1940 / 1644 | 1947 / 1643 |
    | of it, the `sleeping` stage (settling, on) | 130 | 166 | 149 |
    | asleep, ten steps after all of it (on) | 23 | 23 | 36 |

    Against sleeping off at the same step, the sparse component costs 3 to 5%
    of a step more than the copy, two thirds of it outside the systems,
    where the apply node makes the inserts and removes (a boxed change each),
    and the dense one 3.5 to 7%, and half again the
    asleep pile's step (its bodies' tables are wider). Written every step
    (seconds, not the step it went slower), either costs at least that. At
    1000 (41 wide) settling it's 181, 191 and 187 µs. So the times stay in
    the copy: what components would buy (the times kept through a reset
    state, and seen by games) isn't worth 3 to 5% of every step before a
    pile is asleep.

    **With sparse changes as runs** (2026-09-26, get-znt.18, a spike:
    storage.md, ["Sparse changes are runs, not
    closures"](storage.md#sparse-changes-are-runs-not-closures)), the sparse
    `Still` again, the same prototype, against the copy on the same ECS;
    µs a step, sleeping on / off, medians of seven runs interleaved, 60 steps
    from the step given:

    | 10 000, 401 wide | the copy (ECS as it was / spike) | sparse `Still`, as it was | sparse `Still`, runs |
    |---|---|---|---|
    | falling (from step 1) | 965 / 852, 965 / 854 | 995 / 853 | 980 / 854 |
    | settling (from step 60) | 1759 / 1607, 1758 / 1609 | 1833 / 1611 | 1792 / 1609 |
    | falling asleep (from step 120) | 1807 / 1592, 1816 / 1590 | 1864 / 1581 | 1845 / 1589 |
    | of it, the `sleeping` stage (settling, on) | 128, 129 | 162 | 151 |
    | asleep, ten steps after all of it (on) | 23, 22 | 23 | 23 |

    Over the copy that is 3.5 to 4.6% of a step as it was (this run's
    measure of the 3 to 5% above), and 1.8 to 2.1% with runs; at 1000 (41
    wide) settling, 4.3% and 1.9%. Its extra cost outside the `sleeping` stage,
    mostly the apply, went from about 40 µs a step to 12; what's left is in
    that stage, where each crossing is noticed, looked up (`Query::get`) and
    logged. So the copy
    stays while the spike is undecided, and the component is now on the 2%
    bar rather than over it.

[^prototype]: *(History, 2026-09-24.)* The prototype kept who's asleep only
    in the mod's transient state, by entity, and every walk looked each
    body (and each contact's ends) up in it: asleep, the broadphase,
    gathering and writing back cost what they did awake, and the merge
    and the solver's gathering more. A reload woke everything; a game's
    write or despawn didn't wake anything; `Touching` on sleeping bodies
    was reset each step. Its test's two surviving mutations (sleeping
    bodies not immovable, and written back) went with the lookups.

[^merged]: *(History, 2026-09-24.)* Before sleeping as storage was merged
    with pages as blocks of the order, the latter's `:tax`, medians of
    five: frame 133 / 125, 126 / 123, 730 / 572, 1350 / 1298 and 1288 /
    1289 (the table's columns in order); broadphase 14, 14, 116, 150, 150.
    Merged, `near_pairs` first chose per pair of active pages which side
    to test row by row from: the dense layout went from 351 µs to 375, so
    only passive pages choose now.

[^tax]: *(History, 2026-09-24.)* When `:tax` was written, at 10 000
    bodies settled: frame 2909 µs against the arrays' 1269, gathering
    colliders 144, broadphase 996 / 277, merging 113 / 11, the solver's
    gathering 179 / 29, writing back 184 / 19, and the re-sort 352 (32 of
    them at 1000 bodies), as much at rest as settled. Before that the 10 000
    frame was 4738 µs, until two fixes: the mod mapped entities to array
    indices by sorting and binary search, three times a step, where entity
    ids are small dense integers and a vector by index does it in O(1)
    (`Slots`); and writing back looked up `Touching` on both ends of every
    contact, though most bodies don't have one. The copies and the upkeep
    were then cut apart, and merged the same day: copying in and out alone
    took the frame to 2521, the upkeep and page lanes alone to about 2650.
    Then, with pages split at the median of their keys, the table's 10 000
    columns read, frame first: falling 910 / 563, broadphase 221 / 201,
    outside the systems 196; settled 1454 / 1279, 227 / 282, 101; at rest
    1384 / 1288, 220 / 283, 25 (medians of three runs).

[^colored-fell]: *(History, 2026-09-27.)* When this table was measured,
    colors let a turning 5050 pyramid fall, its top 2 to 5 lower and boxes
    leaning 40°, with Box2D's rule for static contacts, the lowest free
    color or Rapier's highest, six substeps, or Box2D's softness and
    passes; three relaxing passes stood a pyramid 60 wide but not 100.
    The cause was the warm start, not the colors: [Why colors let the
    pyramid fall](#why-colors-let-the-pyramid-fall).

[^last-substep]: *(History, 2026-09-28.)* Built since, for turning
    contacts' points only, not rows at their normal: [Why colors let the
    pyramid fall](#why-colors-let-the-pyramid-fall). Measured here as a
    stack option, it was set aside for the piles' worst; on the lanes it
    moved the turning piles' worst from 440 to 330.

[^pair-lanes]: *(History, 2026-09-27.)* `Wide::Pair`, the lanes' layout
    one contact to a batch in pair order, was built to check the layout
    against the loop bit for bit before any grouping, and removed once
    levels of one lane checked the same; so were three rules for colored
    contacts with a static end (`Statics`: the lowest free color, not
    color 0, the highest), since none stood the 5050 pyramid.
