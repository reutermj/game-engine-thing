//! The physics step, as a pipeline of systems in the `physics::step` phase:
//! gravity into velocities, contacts, the solver (which also moves bodies),
//! then the spatial index. See docs/architecture/physics.md.
//!
//! Everything a step carries to the next is in the world or in this mod's
//! state, so a reload swaps the solver under a running simulation.

mod broad;
mod narrow;
mod solver;

use std::time::Instant;

use clock::Clock;
use engine_api::{Cx, Entity, EventWriter, Mod, Query, Systems, Without, export_mod, field_struct, phase};
use physics::{
    Aabb, Body, Collider, Contact, DYNAMIC, Gravity, KINEMATIC, Placed, Position, STATIC, Shape, SpatialIndex,
    Touching, Trigger, Vec2, Velocity, build_index,
};
use solver::{Constraint, SolverBody};

/// Grid cells for the broadphase and the index: about a player.
const CELL: f32 = 2.0;
/// A stalled frame slows the simulation rather than tunneling.
const MAX_DT: f32 = 1.0 / 30.0;

#[cfg(not(feature = "v2"))]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

field_struct! {
    /// One contact, from `find_contacts` to `solve` and on to the next
    /// step, for warm starting and to tell a new contact from an old one.
    #[derive(Debug, Default, Copy)]
    struct Cached {
        a: Entity,
        b: Entity,
        nx: f32,
        ny: f32,
        depth: f32,
        friction: f32,
        restitution: f32,
        jn: f32,
        jt: f32,
        /// Pressing on each other at the end of the step: the solver pushed,
        /// or they overlap. A speculative contact is held before it touches,
        /// so being held last step doesn't mean touching.
        pressed: bool,
        /// Pressed last step.
        was_pressed: bool,
    }
}

field_struct! {
    #[derive(Debug, Default, Copy, PartialEq)]
    struct Pair {
        a: Entity,
        b: Entity,
    }
}

field_struct! {
    /// Nanoseconds spent in each system, summed: for the benchmark.
    #[derive(Debug, Default, Copy)]
    struct Timings {
        gravity: u64,
        contacts: u64,
        solve: u64,
        index: u64,
    }
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics {
        /// This step's contacts, sorted by pair; last step's until
        /// `find_contacts` runs.
        contacts: Vec<Cached>,
        /// Sensor pairs overlapping last step.
        sensing: Vec<Pair>,
        steps: u64,
        time: Timings,
    }
}

fn dt(clocks: &mut Query<&Clock>) -> Option<f32> {
    clocks.single(|_, c| c.dt.min(MAX_DT)).filter(|&dt| dt > 0.0)
}

fn nanos(since: Instant) -> u64 {
    since.elapsed().as_nanos() as u64
}

/// A collider as the step sees it.
struct Item {
    entity: Entity,
    placed: Placed,
    collider: Collider,
    body: Body,
    v: Vec2,
}

impl Physics {
    fn integrate_velocities(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut clocks: Query<&Clock>,
        mut gravity: Query<&Gravity>,
        mut bodies: Query<(&Body, &mut Velocity)>,
    ) {
        let start = Instant::now();
        let Some(dt) = dt(&mut clocks) else { return };
        self.steps += 1;
        let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
        bodies.for_each(|_, (body, v)| {
            if body.kind == DYNAMIC {
                v.x += g.x * body.gravity_scale * dt;
                v.y += g.y * body.gravity_scale * dt;
            }
        });
        self.time.gravity += nanos(start);
    }

    fn find_contacts(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut clocks: Query<&Clock>,
        mut bodies: Query<(&Position, &Collider, &Body)>,
        mut statics: Query<(&Position, &Collider), Without<Body>>,
        mut velocities: Query<&Velocity>,
        triggers: EventWriter<Trigger>,
    ) {
        let start = Instant::now();
        if dt(&mut clocks).is_none() {
            return;
        }
        let mut items = Vec::new();
        bodies.for_each(|row, (p, c, b)| items.push(item(row.entity(), p, c, *b)));
        statics.for_each(|row, (p, c)| items.push(item(row.entity(), p, c, Body::fixed())));
        for i in &mut items {
            if let Some(v) = velocities.with(i.entity, |_, v| Vec2::new(v.x, v.y)) {
                i.v = v;
            }
        }
        // Entity order, so pairs (and everything after) don't depend on
        // which table a body is in.
        items.sort_by_key(|i| i.entity);

        let boxes: Vec<Aabb> = items.iter().map(|i| i.placed.aabb()).collect();
        let pairs = broad::pairs(CELL, &boxes, |a, b| {
            let (a, b) = (&items[a], &items[b]);
            let meets = a.collider.mask & b.collider.layer != 0 && b.collider.mask & a.collider.layer != 0;
            // Contacts need something to push; a sensor needs something to
            // arrive. Static colliders overlapping (a goal line and the wall
            // across its end) are the level's shape, not an event.
            let pushes = a.body.kind == DYNAMIC || b.body.kind == DYNAMIC;
            let arrives = a.body.kind != STATIC || b.body.kind != STATIC;
            meets && if a.collider.sensor || b.collider.sensor { arrives } else { pushes }
        });

        let previous = std::mem::take(&mut self.contacts);
        let mut sensing = Vec::new();
        for (i, j) in pairs {
            let (a, b) = (&items[i as usize], &items[j as usize]);
            let Some(m) = narrow::collide(&a.placed, &b.placed, b.v - a.v) else { continue };
            if a.collider.sensor || b.collider.sensor {
                if m.depth >= 0.0 {
                    sensing.push(Pair { a: a.entity, b: b.entity });
                }
                continue;
            }
            let old = previous.binary_search_by_key(&(a.entity, b.entity), |c| (c.a, c.b)).ok().map(|k| previous[k]);
            self.contacts.push(Cached {
                a: a.entity,
                b: b.entity,
                nx: m.normal.x,
                ny: m.normal.y,
                depth: m.depth,
                friction: a.body.friction.min(b.body.friction),
                restitution: a.body.restitution.max(b.body.restitution),
                jn: old.map_or(0.0, |c| c.jn),
                jt: old.map_or(0.0, |c| c.jt),
                pressed: false,
                was_pressed: old.is_some_and(|c| c.pressed),
            });
        }
        for pair in &sensing {
            if !self.sensing.contains(pair) {
                let (sensor, other) = if items[items.binary_search_by_key(&pair.a, |i| i.entity).unwrap()].collider.sensor {
                    (pair.a, pair.b)
                } else {
                    (pair.b, pair.a)
                };
                triggers.send(Trigger { sensor, other });
            }
        }
        self.sensing = sensing;
        self.time.contacts += nanos(start);
    }

    fn solve(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut clocks: Query<&Clock>,
        mut moving: Query<(&Body, &mut Velocity, &mut Position)>,
        mut touching: Query<&mut Touching>,
        contacts: EventWriter<Contact>,
    ) {
        let start = Instant::now();
        let Some(dt) = dt(&mut clocks) else { return };
        let mut entities = Vec::new();
        let mut bodies = Vec::new();
        moving.for_each(|row, (body, v, _)| {
            entities.push(row.entity());
            let inv_mass = if body.kind == DYNAMIC { body.inv_mass } else { 0.0 };
            bodies.push(SolverBody { v: Vec2::new(v.x, v.y), inv_mass, pseudo: Vec2::ZERO });
        });
        // Bodies with no velocity (statics) all stand for one immovable
        // body at the end.
        let mut order: Vec<usize> = (0..entities.len()).collect();
        order.sort_by_key(|&i| entities[i]);
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        let index_of = |e: Entity| {
            order.binary_search_by_key(&e, |&i| entities[i]).map_or(still, |k| order[k] as u32)
        };

        let mut constraints: Vec<Constraint> = self
            .contacts
            .iter()
            .map(|c| Constraint {
                a: index_of(c.a),
                b: index_of(c.b),
                normal: Vec2::new(c.nx, c.ny),
                depth: c.depth,
                friction: c.friction,
                restitution: c.restitution,
                jn: c.jn,
                jt: c.jt,
                speed: 0.0,
            })
            .collect();
        solver::solve(&mut bodies, &mut constraints, dt);

        let mut i = 0;
        moving.for_each(|_, (body, v, p)| {
            let b = bodies[i];
            i += 1;
            if body.kind == STATIC {
                return;
            }
            (v.x, v.y) = (b.v.x, b.v.y);
            // A kinematic body gets no pseudo velocity: nothing pushes it.
            let step = if body.kind == KINEMATIC { b.v } else { b.v + b.pseudo };
            p.x += step.x * dt;
            p.y += step.y * dt;
        });

        touching.for_each(|_, t| *t = Touching::default());
        for (c, k) in self.contacts.iter_mut().zip(&constraints) {
            (c.jn, c.jt) = (k.jn, k.jt);
            c.pressed = k.jn > 0.0 || c.depth >= 0.0;
            if !c.pressed {
                continue;
            }
            let n = Vec2::new(c.nx, c.ny);
            touching.with(c.a, |_, t| mark(t, n));
            touching.with(c.b, |_, t| mark(t, -n));
            if !c.was_pressed {
                contacts.send(Contact { a: c.a, b: c.b, nx: c.nx, ny: c.ny, speed: k.speed });
            }
        }
        self.time.solve += nanos(start);
    }

    fn publish_index(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut shapes: Query<(&Position, &Collider)>,
        mut index: Query<&mut SpatialIndex>,
    ) {
        let start = Instant::now();
        let mut colliders = Vec::new();
        shapes.for_each(|row, (p, c)| colliders.push((row.entity(), placed(p, c))));
        colliders.sort_by_key(|(e, _)| *e);
        index.single(|_, index| build_index(CELL, colliders, index));
        self.time.index += nanos(start);
    }
}

fn placed(p: &Position, c: &Collider) -> Placed {
    Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y) }
}

fn item(entity: Entity, p: &Position, c: &Collider, body: Body) -> Item {
    Item { entity, placed: placed(p, c), collider: *c, body, v: Vec2::ZERO }
}

/// Marks the side of a body its contact is on: `n` points from it toward
/// what it touches.
fn mark(t: &mut Touching, n: Vec2) {
    if n.y > 0.5 {
        t.below = true;
    } else if n.y < -0.5 {
        t.above = true;
    } else if n.x > 0.5 {
        t.right = true;
    } else if n.x < -0.5 {
        t.left = true;
    }
}

impl Mod for Physics {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        const STEP: &str = "physics::step";
        s.phase(STEP).after(phase::SIMULATE).before(phase::LATE);
        s.add("integrate_velocities", Self::integrate_velocities).phase(STEP);
        s.add("find_contacts", Self::find_contacts).phase(STEP).after("physics::integrate_velocities");
        s.add("solve", Self::solve).phase(STEP).after("physics::find_contacts");
        s.add("publish_index", Self::publish_index).phase(STEP).after("physics::solve");
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        if world.single::<&SpatialIndex, ()>(|_, _| ()).is_none() {
            world.spawn((SpatialIndex::default(),));
        }
    }

    /// `stats`: the build, steps run, contacts held, and time per system.
    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message.trim() {
            "stats" => {
                let per = |ns: u64| ns as f64 / self.steps.max(1) as f64 / 1e3;
                let t = self.time;
                Ok(format!(
                    "build {BUILD} steps {} contacts {} us/step gravity {:.1} contacts {:.1} solve {:.1} index {:.1}",
                    self.steps,
                    self.contacts.len(),
                    per(t.gravity),
                    per(t.contacts),
                    per(t.solve),
                    per(t.index)
                ))
            }
            "reset_timings" => {
                (self.time, self.steps) = (Timings::default(), 0);
                Ok("reset".into())
            }
            other => Err(format!("physics doesn't understand {other:?}; try stats")),
        }
    }
}

export_mod!(Physics);
