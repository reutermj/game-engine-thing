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
3. **`solve`**: sequential impulses over the contacts, eight iterations,
   warm-started from the last step's impulses, with Coulomb friction and
   restitution above a small speed threshold. Positional error is
   corrected by a split impulse, so correction doesn't add energy. It also
   moves the bodies (the split impulse's pseudo velocities exist only
   here), stores each contact's impulses and whether it's pressed, updates
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
baseline is our own array code, not Box2D). Further optimization waits for
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

**Status: off unless a game spawns `Sleep`; sleeping is storage**
(2026-09-24, `sleep.rs`, get-emj.27). An island, dynamic bodies joined by
pressed contacts, whose bodies have all been slower than `Sleep::speed`
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
when the solver does), which is why it's opt in, and why `:tax` measures
it apart, the ECS alone, with no arrays to agree with. From when the whole
pile is asleep, against the same pile awake at the same step (speed 0.05,
0.5 s), µs per step, medians of three runs, before (sleeping as a lookup
per body, the prototype) and after:

| | 1000, before | 1000, after | 10 000, before | 10 000, after |
|---|---|---|---|---|
| asleep by step | 630 | 630 | 550 | 550 |
| frame | 51 / 134 | 9 / 133 | 506 / 1349 | 22 / 1340 |
| gravity (and waking) | 2 / 2 | 1 / 2 | 20 / 20 | 8 / 20 |
| gathering colliders | 4 / 4 | 0 / 4 | 49 / 49 | 1 / 49 |
| broadphase | 14 / 14 | 0 / 15 | 149 / 146 | 2 / 160 |
| narrowphase | 4 / 9 | 0 / 10 | 48 / 98 | 0 / 107 |
| merging contacts | 5 / 4 | 0 / 3 | 53 / 36 | 0 / 34 |
| solve: gathering | 9 / 7 | 0 / 6 | 97 / 71 | 1 / 67 |
| solver | 0 / 75 | 0 / 74 | 0 / 773 | 0 / 762 |
| writing back | 6 / 6 | 0 / 6 | 61 / 66 | 1 / 65 |
| outside the systems | 6 / 11 | 7 / 12 | 25 / 78 | 9 / 78 |
| deepest overlap | 0.007 / 0.007 | 0.007 / 0.007 | 0.006 / 0.006 | 0.006 / 0.006 |

(Before is the prototype on pages as blocks of the order, measured in the
same session as after. Its gravity and gathering are from the main table:
its sleeping table didn't show them, and both walked every body asleep or
not.)[^sleep-first]

What's left asleep at 10 000 is about 8 µs looking for what games changed
(a look at each sleeping page's ticks, `for_each_written`, over five
terms), and the frame's fixed cost outside the systems, which grows from 7
to 9 µs between 1000 and 10 000 for a reason not found (nothing re-sorts,
and nothing outside the systems walks the sleeping rows that we know of).

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

**What wakes an island** (the whole island, as it fell asleep):

- a moving awake body pressing on one of its bodies (a still one waits to
  fall asleep in its own island), or a kinematic body moving into one;
- a contact one of its bodies pressed on ending: what it rested on was
  despawned or moved away;
- a game writing one of its bodies' `Velocity`, `Position`, `Collider` or
  `Body`: found by page ticks since the last step
  ([change detection](spatial-storage.md#change-detection)), so it costs
  a look per sleeping page, not a scan of velocities;
- a game despawning one of its bodies, or removing its `Asleep`: the count
  of sleeping bodies in the world no longer matches physics's;
- a game moving a static into it or out from under it (a static written
  since the last step), despawning one under it (the count of statics
  changes, and every resting contact's ends are checked), or making a
  static it rests on move (the pair is found again, not resting, while
  its `Resting` contact is there);
- `physics` sent `wake`, or `Sleep` despawned: everything.

**A reload keeps it asleep.** Who's asleep is in the world, so a new build
reads it at load (`Sleepers`, the mod's copy by entity index, is rebuilt
from `Asleep`), and the tick after the last solve is in the mod's state, so
the new build doesn't take the old one's writes for a game's. Only how
long each awake body has been still is lost, which makes those take
`Sleep::time` longer to sleep.

**`Touching`** on a sleeping body is as it fell asleep: its query excludes
sleeping bodies, so it isn't reset. A side touched by something that
arrives while it sleeps (a still body settling against it) isn't marked.

What it doesn't do:

- **A static spawned into sleeping bodies doesn't wake them**, nor does
  one despawned in the step another is spawned (the count of statics is
  the same). The count and page ticks are what's cheap to watch; spawns
  have no tick.
- **A sleeping body despawned in the step a game puts another to sleep**
  (adding `Asleep`) passes the count check. Only physics should add
  `Asleep`.
- **Waking in `find_contacts`** (a support gone, a static changed) takes
  effect from the next step: the bodies stay in their sleeping tables, and
  immovable, for the rest of the step that noticed.

Its tests (`physics_test`, `pile::`): falling asleep into tables of their
own with every contact resting and overlaps kept; staying put with nothing
written; waking where a body lands, a kinematic body pushes, a static
moves in, or the floor goes (despawned, lowered, or made a body); a game's
velocity or shape change; a body despawned from under others; `Touching`
kept; a reload keeping it all asleep; turning it off; bodies asleep on
shelves that aren't statics (on the active side, but not moving) keeping
one contact per pair. Of 24 mutations, three survive: statics on the
active side and resting pairs sent to the narrowphase, both only slower;
and treating a body woken in the same step as still when marking resting
contacts, which now only delays its contact a step (the next step finds
the pair moving and wakes it through its `Resting` contact), and whose
setup, an island waking as a body touching it falls asleep, the tests
don't make.[^prototype]

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

## What changes elsewhere

- `mods/transform` and the old `mods/physics` demo went;
  `//engine/std/physics` has the name `physics`, and `spawner` and
  `reporter` use it. The `mod_deps` examples in mod-deps.md still hold,
  since `physics` declares `Velocity` in its interface.
- The recorded routes in `platformer_test` and `pong_test` pass
  unchanged (see below).

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

[^sleep-first]: *(History, 2026-09-24.)* Measured first before pages
    were made blocks of the order, µs, asleep / awake: at 1000, frame 56 /
    140 before and 10 / 142 after; at 10 000, 580 / 1472 and 26 / 1503,
    the broadphase 208 / 222 and 2 / 243.

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
