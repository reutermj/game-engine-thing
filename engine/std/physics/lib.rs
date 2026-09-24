//! The physics step, as a pipeline of systems in the `physics::step` phase:
//! gravity into velocities, contacts, then the solver (which also moves
//! bodies). Positions are kept in spatial order by the ECS, so the
//! broadphase is `near_pairs` and moving bodies re-sorts them at the
//! solver's apply node. See docs/architecture/physics.md.

//!
//! Everything a step carries to the next is in the world or in this mod's
//! state, so a reload swaps the solver under a running simulation.

mod narrow;
mod solver;

use std::time::Instant;

use engine_api::{Cx, Despawns, Dt, Entity, EventWriter, Mod, Query, Spawner, Systems, Without, export_mod, field_struct, phase};
use physics::{
    Body, Collider, Contact, ContactPair, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Overlap, Placed, Position,
    Response, STATIC, Shape, Touching, Trigger, Vec2, Velocity,
};
use solver::{Constraint, SolverBody};


#[cfg(not(feature = "v2"))]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

field_struct! {
    /// Nanoseconds spent in each system, summed: for the benchmark.
    #[derive(Debug, Default, Copy)]
    struct Timings {
        gravity: u64,
        contacts: u64,
        solve: u64,
        /// Within `contacts`: colliders gathered and sorted, the
        /// broadphase, the narrowphase, and the merge with the world's.
        gather: u64,
        broadphase: u64,
        narrowphase: u64,
        merge: u64,
        /// Within `solve`: bodies and contacts gathered, the solver, and
        /// the results written back.
        solve_gather: u64,
        solver: u64,
        write_back: u64,
    }
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics {
        steps: u64,
        time: Timings,
    }
}

/// Entities to positions in a list, by entity index: ids are small dense
/// integers, so a vector by index finds one in O(1), where sorting the list
/// and binary-searching it was most of gathering. The generation is kept,
/// so a stale id (despawned since, its index reused) finds nothing.
struct Slots(Vec<(u32, u32)>);

impl Slots {
    fn of(entities: impl Iterator<Item = Entity> + Clone) -> Slots {
        let len = entities.clone().map(|e| e.index as usize + 1).max().unwrap_or(0);
        let mut slots = vec![(u32::MAX, u32::MAX); len];
        for (k, e) in entities.enumerate() {
            slots[e.index as usize] = (e.generation, k as u32);
        }
        Slots(slots)
    }

    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }
}

fn nanos(since: Instant) -> u64 {
    since.elapsed().as_nanos() as u64
}

/// A collider as the step sees it: only what pairs are tested with, since
/// gathering is writing every collider out, and with a whole `Collider`
/// and `Body` it took 64 µs at 10 000 against 44 (2026-09-24).
struct Item {
    entity: Entity,
    placed: Placed,
    collider: Layers,
    body: Material,
    v: Vec2,
}

/// A `Collider`'s layers and roles.
struct Layers {
    layer: u32,
    mask: u32,
    senses: u32,
    sensor: bool,
}

/// A `Body`'s part in a pair.
struct Material {
    kind: u8,
    friction: f32,
    restitution: f32,
}

impl Physics {
    fn integrate_velocities(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        dt: Dt,
        mut gravity: Query<&Gravity>,
        mut bodies: Query<(&Body, &mut Velocity)>,
    ) {
        let start = Instant::now();
        let dt = *dt;
        self.steps += 1;
        let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
        bodies.for_each(|_, (body, mut v)| {
            if body.kind == DYNAMIC {
                v.x += g.x * body.gravity_scale * dt;
                v.y += g.y * body.gravity_scale * dt;
            }
        });
        self.time.gravity += nanos(start);
    }

    /// Finds this step's contacts and overlaps, and brings the world's in
    /// line with them in one pass each: both are in pair order, the
    /// world's by storage (ordered tables) and this step's by the
    /// broadphase. A contact that persists keeps its entity and impulses.
    #[allow(clippy::type_complexity)]
    fn find_contacts(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        // Every collider, by whether it has a body and a velocity: queries
        // have no optional terms, and one query per case gathers each in one
        // walk, where filling velocities in after was a second walk and a
        // lookup per collider.
        (mut moving, mut held, mut drifting, mut statics): (
            Query<(&Position, &Collider, &Body, &Velocity)>,
            Query<(&Position, &Collider, &Body), Without<Velocity>>,
            Query<(&Position, &Collider, &Velocity), Without<Body>>,
            Query<(&Position, &Collider), Without<(Body, Velocity)>>,
        ),
        mut shapes: Query<(&Position, &Collider)>,
        (mut contacts, new_contacts): (
            Query<(&ContactPair, &mut Manifold, &mut Response), (), Despawns>,
            Spawner<(ContactPair, Manifold, Response, Impulse)>,
        ),
        (mut overlaps, new_overlaps): (Query<&Overlap, (), Despawns>, Spawner<(Overlap,)>),
        triggers: EventWriter<Trigger>,
    ) {
        let start = Instant::now();
        let mut items = Vec::with_capacity(moving.len() + held.len() + drifting.len() + statics.len());
        let v = |v: &Velocity| Vec2::new(v.x, v.y);
        moving.for_each(|row, (p, c, b, u)| items.push(item(row.entity(), p, c, *b, v(u))));
        held.for_each(|row, (p, c, b)| items.push(item(row.entity(), p, c, *b, Vec2::ZERO)));
        drifting.for_each(|row, (p, c, u)| items.push(item(row.entity(), p, c, Body::fixed(), v(u))));
        statics.for_each(|row, (p, c)| items.push(item(row.entity(), p, c, Body::fixed(), Vec2::ZERO)));
        let slots = Slots::of(items.iter().map(|i| i.entity));
        // Pairs come in entity order, so what's found doesn't depend on the
        // order items were gathered in.
        let index = |e: Entity| slots.get(e).expect("a collider has a position");
        let arrives = |a: &Item, b: &Item| a.body.kind != STATIC || b.body.kind != STATIC;
        let collides = |a: &Item, b: &Item| {
            let meets = a.collider.mask & b.collider.layer != 0 && b.collider.mask & a.collider.layer != 0;
            // Contacts need something to push; a sensor needs something to
            // arrive. Static colliders overlapping (a goal line and the wall
            // across its end) are the level's shape, not an event.
            let pushes = a.body.kind == DYNAMIC || b.body.kind == DYNAMIC;
            meets && if a.collider.sensor || b.collider.sensor { arrives(a, b) } else { pushes }
        };
        let senses = |a: &Item, b: &Item| {
            (a.collider.senses & b.collider.layer != 0 || b.collider.senses & a.collider.layer != 0) && arrives(a, b)
        };
        // The storage's own order is the broadphase: pairs whose boxes, grown
        // by the speculative margin, meet. In entity order, lesser first.
        let gathered = Instant::now();
        let pairs: Vec<(u32, u32)> = shapes.near_pairs(narrow::MARGIN).into_iter().map(|(a, b)| (index(a), index(b))).collect();
        let paired = Instant::now();

        let mut found: Vec<(ContactPair, Manifold, Response)> = Vec::new();
        // Each overlap, and whether it's a sensor's, which triggers.
        let mut overlapping: Vec<(Overlap, bool)> = Vec::new();
        for (i, j) in pairs {
            let (a, b) = (&items[i as usize], &items[j as usize]);
            let (collide, sense) = (collides(a, b), senses(a, b));
            if !collide && !sense {
                continue;
            }
            let Some(m) = narrow::collide(&a.placed, &b.placed, b.v - a.v) else { continue };
            let sensor = collide && (a.collider.sensor || b.collider.sensor);
            if (sensor || sense) && m.depth >= 0.0 {
                overlapping.push((Overlap { a: a.entity, b: b.entity }, sensor));
            }
            if collide && !sensor {
                found.push((
                    ContactPair { a: a.entity, b: b.entity },
                    Manifold { nx: m.normal.x, ny: m.normal.y, depth: m.depth, pressed: false, was_pressed: false },
                    Response {
                        friction: a.body.friction.min(b.body.friction),
                        restitution: a.body.restitution.max(b.body.restitution),
                        disabled: false,
                    },
                ));
            }
        }

        let narrowed = Instant::now();
        let key = |p: &ContactPair| (p.a, p.b);
        let mut next = 0;
        let spawn = |(pair, m, r): (ContactPair, Manifold, Response)| {
            new_contacts.spawn((pair, m, r, Impulse::default()));
        };
        // Every contact is either written or despawned, so each page is
        // stamped written as a whole, not row by row: 54 µs to 34 at 10 000
        // contacts, against `for_each_ordered` and `Mut` (2026-09-24).
        contacts.for_each_ordered_page(|page, (pair, mut m, mut r)| {
            let (m, r) = (m.write_all(), r.write_all());
            for i in page.rows() {
                while found.get(next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                    spawn(found[next]);
                    next += 1;
                }
                match found.get(next) {
                    Some(f) if f.0 == pair[i] => {
                        m[i] = Manifold { was_pressed: m[i].pressed, ..f.1 };
                        r[i] = f.2;
                        next += 1;
                    }
                    _ => page.row(i).despawn(),
                }
            }
        });
        found[next..].iter().copied().for_each(spawn);

        let key = |o: &Overlap| (o.a, o.b);
        let mut next = 0;
        let begin = |(o, sensor): (Overlap, bool)| {
            new_overlaps.spawn((o,));
            if sensor {
                let (sensor, other) = if items[index(o.a) as usize].collider.sensor { (o.a, o.b) } else { (o.b, o.a) };
                triggers.send(Trigger { sensor, other });
            }
        };
        overlaps.for_each_ordered(|row, o| {
            while overlapping.get(next).is_some_and(|f| key(&f.0) < key(o)) {
                begin(overlapping[next]);
                next += 1;
            }
            if overlapping.get(next).is_some_and(|f| f.0 == *o) {
                next += 1;
            } else {
                row.despawn();
            }
        });
        overlapping[next..].iter().copied().for_each(begin);
        let t = &mut self.time;
        t.gather += (gathered - start).as_nanos() as u64;
        t.broadphase += (paired - gathered).as_nanos() as u64;
        t.narrowphase += (narrowed - paired).as_nanos() as u64;
        t.merge += nanos(narrowed);
        t.contacts += nanos(start);
    }

    fn solve(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        dt: Dt,
        mut moving: Query<(&Body, &mut Velocity, &mut Position)>,
        mut touching: Query<&mut Touching>,
        mut contacts: Query<(&ContactPair, &mut Manifold, &Response, &mut Impulse)>,
        began: EventWriter<Contact>,
    ) {
        let start = Instant::now();
        let dt = *dt;
        let mut bodies = Vec::with_capacity(moving.len() + 1);
        let mut entities = Vec::with_capacity(moving.len());
        moving.for_each(|row, (body, v, _)| {
            entities.push(row.entity());
            let inv_mass = if body.kind == DYNAMIC { body.inv_mass } else { 0.0 };
            bodies.push(SolverBody { v: Vec2::new(v.x, v.y), inv_mass, pseudo: Vec2::ZERO });
        });
        // Bodies with no velocity (statics) all stand for one immovable
        // body at the end.
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        let slots = Slots::of(entities.iter().copied());
        let index_of = |e: Entity| slots.get(e).unwrap_or(still);

        // In pair order, which storage keeps: the solve doesn't depend on
        // when each contact began. (History, 2026-09-24: contacts were
        // copied out of the walk and mapped after, which measured faster
        // until `for_each` walked slices.)
        let mut constraints: Vec<Constraint> = Vec::with_capacity(contacts.len());
        contacts.for_each_ordered(|_, (pair, m, r, j)| {
            if r.disabled {
                return;
            }
            constraints.push(Constraint {
                a: index_of(pair.a),
                b: index_of(pair.b),
                normal: Vec2::new(m.nx, m.ny),
                depth: m.depth,
                friction: r.friction,
                restitution: r.restitution,
                jn: j.normal,
                jt: j.tangent,
                speed: 0.0,
            });
        });
        let gathered = Instant::now();
        solver::solve(&mut bodies, &mut constraints, dt);
        let after_solver = Instant::now();

        let mut i = 0;
        moving.for_each(|_, (body, mut v, mut p)| {
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

        // Most bodies don't ask (the pile's none), and a failed lookup per
        // end of every contact was half of writing back.
        let mut asking = Vec::new();
        touching.for_each(|row, mut t| {
            *t = Touching::default();
            asking.push(row.entity());
        });
        let asking = Slots::of(asking.iter().copied());
        let mut solved = constraints.iter();
        // Every contact's impulse and pressing are written, so pages are
        // stamped whole, as in the merge.
        contacts.for_each_ordered_page(|page, (pair, mut m, r, mut j)| {
            let (m, j) = (m.write_all(), j.write_all());
            for i in page.rows() {
                if r[i].disabled {
                    (m[i].pressed, j[i]) = (false, Impulse::default());
                    continue;
                }
                let k = solved.next().expect("a constraint per contact solved");
                j[i] = Impulse { normal: k.jn, tangent: k.jt };
                m[i].pressed = k.jn > 0.0 || m[i].depth >= 0.0;
                if !m[i].pressed {
                    continue;
                }
                let (pair, m) = (pair[i], m[i]);
                let n = Vec2::new(m.nx, m.ny);
                for (e, n) in [(pair.a, n), (pair.b, -n)] {
                    if asking.get(e).is_some() {
                        touching.with(e, |_, mut t| mark(&mut t, n));
                    }
                }
                if !m.was_pressed {
                    began.send(Contact { a: pair.a, b: pair.b, nx: m.nx, ny: m.ny, speed: k.speed });
                }
            }
        });
        let t = &mut self.time;
        t.solve_gather += (gathered - start).as_nanos() as u64;
        t.solver += (after_solver - gathered).as_nanos() as u64;
        t.write_back += nanos(after_solver);
        t.solve += nanos(start);
    }
}


fn placed(p: &Position, c: &Collider) -> Placed {
    Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y) }
}

fn item(entity: Entity, p: &Position, c: &Collider, body: Body, v: Vec2) -> Item {
    Item {
        entity,
        placed: placed(p, c),
        collider: Layers { layer: c.layer, mask: c.mask, senses: c.senses, sensor: c.sensor },
        body: Material { kind: body.kind, friction: body.friction, restitution: body.restitution },
        v,
    }
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
        // At `simulate`'s rate, so the two make one group: a game's rules,
        // then the step, once per step.
        s.phase(STEP).after(phase::SIMULATE).before(phase::LATE).fixed_hz(phase::SIMULATE_HZ);
        s.add("integrate_velocities", Self::integrate_velocities).phase(STEP);
        s.add("find_contacts", Self::find_contacts).phase(STEP).after("physics::integrate_velocities");
        s.add("solve", Self::solve).phase(STEP).after("physics::find_contacts");
    }

    /// `stats`: the build, steps run, contacts held, and time per system.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        match message.trim() {
            "stats" => {
                let per = |ns: u64| ns as f64 / self.steps.max(1) as f64 / 1e3;
                let mut contacts = 0;
                cx.world().for_each::<&ContactPair>(|_, _| contacts += 1);
                let t = self.time;
                Ok(format!(
                    "build {BUILD} steps {} contacts {} us/step gravity {:.1} contacts {:.1} solve {:.1}",
                    self.steps,
                    contacts,
                    per(t.gravity),
                    per(t.contacts),
                    per(t.solve)
                ))
            }
            "stages" => {
                let per = |ns: u64| ns as f64 / self.steps.max(1) as f64 / 1e3;
                let t = self.time;
                Ok(format!(
                    "gather {:.1} broadphase {:.1} narrowphase {:.1} merge {:.1} solve_gather {:.1} solver {:.1} write_back {:.1}",
                    per(t.gather),
                    per(t.broadphase),
                    per(t.narrowphase),
                    per(t.merge),
                    per(t.solve_gather),
                    per(t.solver),
                    per(t.write_back)
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
