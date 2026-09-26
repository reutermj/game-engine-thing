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

Not goals for the MVP: rotation (see [Open questions](#open-questions)),
joints, continuous collision detection, arbitrary polygons.

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
    /// The world's gravity, on one entity. None means none: pong has no
    /// entity with it.
    pub struct Gravity: "physics::Gravity" { pub x: f32, pub y: f32 }
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
// now and the step before), `Response` (friction, restitution, disabled:
// what the solver does with it this step) and `Impulse` (for warm starting).

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
   circle–circle): a normal and a depth, and no contact points, which
   without rotation change nothing.
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

`Position` is a spatial key, with `Collider` as its extent, so the ECS
keeps every table of positions in spatial order
([spatial-storage.md](spatial-storage.md)). The broadphase is
`near_pairs` over a query of positions and colliders: pairs whose boxes,
grown by the speculative margin, meet, found page by page. There is no
index to rebuild, and static bodies' pages never change.[^grid]

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
detection** (2026-09-24, `sleep.rs`, get-emj.27; its gaps closed, and on
by default, 2026-09-25).[^sleep-default] An island, dynamic bodies joined
by pressed contacts, whose bodies have all been slower than `Sleep::speed`
for `Sleep::time` falls asleep: its velocities are zeroed and each body
gets `Asleep { island }`, which moves it to a table of its own, and each
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
alone, with no arrays to agree with (`:tax -- sleeping` runs only that
table). From ten steps after the whole pile is asleep,
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
sleeping bodies was a walk of their pages too.[^sleep-counts]

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
  - a body despawned, its `Asleep` removed, or no longer a body: a row
    left the sleeping tables (`Query::left_since`, a tick per table), and
    a walk of them finds which. A spawn reusing a sleeping body's index
    in the step it went wakes its island too (`Sleepers::slot`);
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

None of these is a count, so none is fooled by one thing gone and another
come in the same step (a static despawned and another spawned, a sleeping
body despawned as a game puts another to sleep).

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
an island it numbers. Physics takes them as they are, before it wakes
anything that step: an island woken there has rows that aren't asleep to
physics either, and taking them for new was a bug the count had (a game
waking one body by removing its `Asleep` put its island back to
sleep).[^sleep-counts] Physics numbers its islands after the greatest it
has seen, so a game's numbers don't collide with ones already made, but
may with later ones.

**A reload doesn't show.** `Sleepers`, the mod's copy of who's asleep by
entity index, with how long each awake body has been still, is the
transient part the step borrows, and the old build hands it to the new
one through the mod's state (`unload`, then `load`). The ticks it looks
from (after `find_contacts`, and after the solve) are in the state too,
so the new build doesn't take the old one's writes for a game's. A build
that starts without it (the first, or one whose state was reset) rebuilds
it from `Asleep` in the world: what's asleep stays asleep, and only how
long awake bodies have been still is lost.[^sleep-reload] The games'
reload replays (//engine/tests:replay.rs) hold physics to this: a run
reloaded every frame is the run without reloads, bit for bit.

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
   nothing when everything falls, where ours already leads.
4. **Colored, wide solving on one thread (algorithm).** 1.28–1.4× the
   solver, about 400–500 µs at 10 000 and more on the pyramid; another
   computation, deterministic, and the start of the parallel solve.
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

[^split]: 2026-09-26: until then, sequential impulses with a split
    impulse (Bullet's push velocities): eight velocity iterations, then
    eight passes of pseudo velocities pushing apart contacts sunk past a
    slop of 0.005 by 0.2 of the rest a step. It rested at the slop, 0.005
    deep, but crept for thousands of steps: its pushes along tilted
    normals slid bodies where no friction acted, and its velocity solve,
    with a step's gravity at once, didn't converge on tall stacks. Kept,
    unchanged, as `tests/split_impulse.rs`.

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

**What rotation will add (predicted, not measured):**

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

## Open questions

- **Rotation.** Without it, boxes don't tip over and the stress demo
  stacks like tetris. Adding it is an angle and angular velocity per body,
  inertia, and contact points instead of a manifold's center: roughly
  doubling the solver. Neither game needs it. *(Proposed: not in the MVP.)*
- **Kinematic characters.** The platformer's player as a dynamic body with
  no friction is the simplest thing that works; a dedicated character
  controller (slopes, steps, one-way platforms) is the usual next step and
  waits for a game that needs it.
- **Tunneling.** No continuous collision: a body moving more than its own
  size per step can pass through a thin collider. Pong's ball tops out at
  40 cells/s, 0.67 cells a step against paddles a cell thick, which is
  within it; substepping is the cheap fix if a game needs more.

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
