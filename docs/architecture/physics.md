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
   finds bodies where they are.

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
