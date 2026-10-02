//! SPIKE, not engine code: the measurements behind
//! docs/architecture/working-sets.md. Kept as a bench target so its numbers
//! can be taken again; nothing depends on it.
//!
//! The question: the solve copies awake bodies out of the world into dense
//! arrays every step (`solve` in lib.rs), with an entity-to-index map
//! (`Slots`) built from the walk. Would a dense working set that storage
//! keeps, its index by entity kept across steps, do that copy for less?
//!
//! The 2D mod runs a scene in the engine to a step; then, on that world,
//! between frames, harness systems (`engine_ecs::harness`, the engine's own
//! queries over its own storage) gather the solver's input and scatter its
//! output back, each way in turn, on the same world:
//!
//! - **hand**: `solve`'s gather and write-back as built, one thread: bodies
//!   in walk order, `Slots` rebuilt from the walk, fresh vectors.
//! - **hand, buffers kept**: the same into vectors kept from the last step.
//! - **hand, two page walks**: as buffers kept, the bodies and the turning
//!   bodies each walked a page at a time (`for_each_page`).
//! - **hand, one page walk**: one walk over both at once, what a query with
//!   optional `Rotation` and `Spin` would walk (exact here: every awake
//!   body turns, checked).
//! - **hand, sorted by entity**: as buffers kept, then renumbered in entity
//!   order each step: the kept index's order without keeping anything.
//! - **kept index**: an index by entity kept across steps, slots in entity
//!   order (Box2D's `localIndex`, Rapier's `active_set_id`): the gather
//!   writes each row to its slot, found by its entity; the scatter reads it
//!   the same way. Membership is checked by `arrived_since`/`left_since`
//!   and the index rebuilt only when a body came or went.
//! - **slot column**: as the kept index, the slot read beside the row
//!   instead of found by entity: what a slot kept in a column of the
//!   bodies' own tables would cost (a stand-in: a vector in walk order).
//! - **derived kept**: as the kept index, with what the gather derives
//!   (inverse mass by kind, gravity, kinds) kept from the last step, and
//!   only the velocity gathered, while no `Body` was written.
//!
//! Each way's solve is checked bit for bit against hand's, by entity, and
//! timed: slots in entity order put the bodies in another order in the
//! solver's arrays. Scatters write the solved values, and the world is put
//! back after each (untimed), so every way sees the same world.
//!
//! Then what an index kept by storage would add to spawning and despawning
//! (its insert and swap-remove, with the moved slot fixed up), against the
//! world's own cost of both, on a world of bodies.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:working_set_spike
//!
//! `REPS` (21), the median of each; `ONLY=falling`, `settled` or `pyramid`;
//! `THREADS` (unset) also times the solve across that many kept threads.

#[allow(dead_code)]
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
#[path = "../tests/pool.rs"]
mod pool;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
#[allow(dead_code)]
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Despawns, Entity, Executor, Query, Spawner, With, Without, Workers, World};
use physics2d::{
    Asleep, Body, Collider, ContactPair, ContactPoints, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Position, Response, Resting,
    Rotation, STATIC, Spin, Tuning, Vec2, Velocity,
};
pub use sim::{Dyn, Sim};

use scene::Scene;
use solver::{Constraint, ContactPoint, Points, SolverBody, Spinning};

const DT: f32 = 1.0 / 60.0;

/// `Slots` as lib.rs has it: entities to places, by entity index.
#[derive(Clone, Default)]
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

    #[inline]
    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }

    fn insert(&mut self, e: Entity, k: u32) {
        let i = e.index as usize;
        if self.0.len() <= i {
            self.0.resize(i + 1, (u32::MAX, u32::MAX));
        }
        self.0[i] = (e.generation, k);
    }

    fn clear(&mut self, e: Entity) {
        self.0[e.index as usize] = (u32::MAX, u32::MAX);
    }
}

/// An index a world would keep: slots by entity, dense, filled in the order
/// rows arrive and emptied by swap-remove, the last slot moved into the
/// hole and its entity's entry fixed up (Box2D's `b2DestroyBody`, Jolt's
/// `RemoveBodyFromActiveBodies`, Rapier's `rigid_body_removed_or_disabled`).
#[derive(Clone, Default)]
struct Kept {
    by_entity: Slots,
    entities: Vec<Entity>,
}

impl Kept {
    fn add(&mut self, e: Entity) {
        self.by_entity.insert(e, self.entities.len() as u32);
        self.entities.push(e);
    }

    fn remove(&mut self, e: Entity) {
        let Some(k) = self.by_entity.get(e) else { return };
        self.by_entity.clear(e);
        let last = self.entities.pop().expect("a slot per entity");
        if last != e {
            self.entities[k as usize] = last;
            self.by_entity.insert(last, k);
        }
    }
}

/// What a gather hands the solver, and how it was indexed.
#[derive(Clone, Default)]
struct Copy {
    entities: Vec<Entity>,
    bodies: Vec<SolverBody>,
    kinds: Vec<(bool, u8)>,
    spinning: Vec<Spinning>,
    reach: Vec<f32>,
    constraints: Vec<Constraint>,
    points: Vec<Points>,
    slots: Slots,
    /// For `HandEntity`: each walk position's slot.
    walk_slot: Vec<u32>,
}

/// Microseconds of each stage, for one way, one rep.
#[derive(Clone, Copy, Default, Debug)]
struct Times {
    index: f64,
    bodies: f64,
    turning: f64,
    contacts: f64,
    solver: f64,
    scatter: f64,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Way {
    Hand,
    HandKept,
    Kept,
    Column,
    Derived,
    HandEntity,
    OneWalk,
    HandPage,
}

const WAYS: [Way; 8] = [Way::Hand, Way::HandKept, Way::HandPage, Way::OneWalk, Way::HandEntity, Way::Kept, Way::Column, Way::Derived];

impl Way {
    fn name(self) -> &'static str {
        match self {
            Way::Hand => "hand (as built)",
            Way::HandKept => "hand, buffers kept",
            Way::Kept => "kept index, by entity",
            Way::Column => "slot column",
            Way::Derived => "kept index, derived kept",
            Way::HandEntity => "hand, sorted by entity each step",
            Way::OneWalk => "hand, one page walk (optional terms)",
            Way::HandPage => "hand, two page walks",
        }
    }
}

/// Everything the spike's systems share with `main`.
struct Spike {
    way: Way,
    copies: HashMap<&'static str, Copy>,
    /// The kept index, kept across reps as a world would keep it across
    /// steps, and the tick its membership was last checked at.
    kept: Kept,
    kept_since: u32,
    rebuilt: u32,
    /// For `Column`: each awake body's slot, in walk order.
    column: Vec<u32>,
    /// For `Derived`: the tick `Body` was last checked at.
    derived_since: u32,
    times: Times,
    /// The bodies as they were, by entity, to put back.
    saved: HashMap<Entity, (Velocity, Position, Option<(Rotation, Spin)>)>,
}

type Spikes = Arc<Mutex<Spike>>;

type Moving<'w, 'a> = Query<'w, (&'a Body, &'a mut Velocity, &'a mut Position), Without<Asleep>>;
type Turning<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a mut Rotation, &'a mut Spin), Without<Asleep>>;
type Contacts<'w, 'a> = Query<'w, (&'a ContactPair, &'a Manifold, &'a Response, &'a Impulse, &'a ContactPoints), Without<Resting>>;

fn us(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1e6
}

fn solver_body(body: &Body, v: &Velocity, g: Vec2) -> SolverBody {
    // As lib.rs's `solve`: the gravity `integrate_velocities` added, for
    // the solver to spread over its substeps.
    let (inv_mass, g) = if body.kind == DYNAMIC { (body.inv_mass, g) } else { (0.0, Vec2::ZERO) };
    let gravity = Vec2::new(g.x * body.gravity_scale * DT, g.y * body.gravity_scale * DT);
    SolverBody::new(Vec2::new(v.x, v.y), inv_mass, gravity)
}

fn constraint(
    index: impl Fn(Entity) -> u32,
    pair: &ContactPair,
    m: &Manifold,
    r: &Response,
    j: &Impulse,
    cp: &ContactPoints,
    points: &mut Vec<Points>,
) -> Constraint {
    let c = Constraint {
        a: index(pair.a),
        b: index(pair.b),
        normal: Vec2::new(m.nx, m.ny),
        depth: m.depth,
        friction: r.friction,
        restitution: r.restitution,
        jn: j.normal,
        jt: j.tangent,
        speed: 0.0,
        points: 0,
    };
    if m.points == 0 {
        return c;
    }
    let mut pts = Points { count: m.points, ..Points::default() };
    for (i, p) in pts.point.iter_mut().enumerate().take(m.points as usize) {
        let (ra, rb) = cp.anchors(i);
        let (jn, jt) = cp.last(m.solved, cp.ids[i]);
        *p = ContactPoint { ra, rb, separation: cp.separations[i], jn, jt };
    }
    points.push(pts);
    c.with_points(points.len() - 1)
}

/// The gather, each way: bodies, their angular state, then the contacts.
fn gather(
    spikes: &Spikes,
    (moving, masses): (&mut Moving, &mut Query<&Body, Without<Asleep>>),
    turning: &mut Turning,
    contacts: &mut Contacts,
    gravity: &mut Query<&Gravity>,
) {
    let mut s = spikes.lock().unwrap();
    let way = s.way;
    let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
    let mut copy = match way {
        Way::Hand => Copy::default(),
        _ => s.copies.remove(way.name()).unwrap_or_default(),
    };
    let mut t = Times::default();
    let start = Instant::now();
    match way {
        Way::OneWalk => unreachable!("gathered by `gather_one`"),
        Way::Hand | Way::HandKept | Way::HandEntity | Way::HandPage => {
            // lib.rs's `solve`, one thread, word for word but the timers.
            let n = moving.len();
            if way == Way::Hand {
                copy.bodies = Vec::with_capacity(n + 1);
                copy.entities = Vec::with_capacity(n);
                copy.kinds = Vec::with_capacity(n + 1);
            } else {
                copy.bodies.clear();
                copy.entities.clear();
                copy.kinds.clear();
            }
            let Copy { entities, bodies, kinds, .. } = &mut copy;
            if way == Way::HandPage {
                moving.for_each_page(|page, (body, v, _)| {
                    let v = v.as_slice();
                    for (r, &e) in page.entities().iter().enumerate() {
                        entities.push(e);
                        bodies.push(solver_body(&body[r], &v[r], g));
                        kinds.push((body[r].kind != STATIC, body[r].kind));
                    }
                });
            } else {
                moving.for_each(|row, (body, v, _)| {
                    entities.push(row.entity());
                    bodies.push(solver_body(body, &v, g));
                    kinds.push((body.kind != STATIC, body.kind));
                });
            }
            bodies.push(SolverBody::default());
            kinds.push((false, STATIC));
            let walked = Instant::now();
            t.bodies = (walked - start).as_secs_f64() * 1e6;
            copy.slots = Slots::of(copy.entities.iter().copied());
            if way == Way::HandEntity {
                // Renumbered in entity order: the index scanned by entity
                // index gives each its rank, and the gathered values are
                // moved to it.
                let n = copy.entities.len();
                copy.walk_slot.resize(n, 0);
                let (mut entities, mut bodies, mut kinds) = (Vec::with_capacity(n), Vec::with_capacity(n + 1), Vec::with_capacity(n + 1));
                for (k, at) in copy.slots.0.iter_mut().filter(|(_, i)| *i != u32::MAX).enumerate() {
                    let i = at.1 as usize;
                    copy.walk_slot[i] = k as u32;
                    entities.push(copy.entities[i]);
                    bodies.push(copy.bodies[i]);
                    kinds.push(copy.kinds[i]);
                    at.1 = k as u32;
                }
                bodies.push(SolverBody::default());
                kinds.push((false, STATIC));
                (copy.entities, copy.bodies, copy.kinds) = (entities, bodies, kinds);
            }
            t.index = us(walked);
        }
        Way::Kept | Way::Column | Way::Derived => {
            // Membership: a world would keep the index as rows arrive and
            // leave; here a look per table says whether any did, and the
            // index is rebuilt if so (never, in these scenes: nothing
            // spawns, despawns or sleeps).
            let now = moving.now();
            if s.kept.entities.is_empty() || moving.arrived_since(s.kept_since) || moving.left_since(s.kept_since) {
                let mut all = Vec::with_capacity(moving.len());
                moving.for_each_page(|page, _| all.extend_from_slice(page.entities()));
                // Slots in entity order: the order a pile's bodies arrived.
                all.sort();
                s.kept = Kept::default();
                all.iter().for_each(|&e| s.kept.add(e));
                s.rebuilt += 1;
                let kept = &s.kept;
                let mut column = Vec::with_capacity(all.len());
                moving.for_each_page(|page, _| column.extend(page.entities().iter().map(|&e| kept.by_entity.get(e).unwrap())));
                s.column = column;
                // A rebuilt index has no derived values to keep.
                copy.bodies.clear();
            }
            s.kept_since = now;
            t.index = us(start);
            let walk = Instant::now();
            let n = s.kept.entities.len();
            // The bodies and kinds stay where they were: a slot per body.
            let fresh = copy.bodies.len() != n + 1;
            if fresh {
                copy.bodies = vec![SolverBody::default(); n + 1];
                copy.kinds = vec![(false, STATIC); n + 1];
            }
            copy.entities.clone_from(&s.kept.entities);
            let Copy { bodies, kinds, .. } = &mut copy;
            let index = &s.kept.by_entity;
            match way {
                Way::Kept => moving.for_each_page(|page, (body, v, _)| {
                    for (r, &e) in page.entities().iter().enumerate() {
                        let k = index.get(e).expect("a slot per awake body") as usize;
                        bodies[k] = solver_body(&body[r], &v[r], g);
                        kinds[k] = (body[r].kind != STATIC, body[r].kind);
                    }
                }),
                Way::Column => {
                    let mut at = s.column.iter();
                    moving.for_each_page(|page, (body, v, _)| {
                        for r in page.rows() {
                            let k = *at.next().unwrap() as usize;
                            bodies[k] = solver_body(&body[r], &v[r], g);
                            kinds[k] = (body[r].kind != STATIC, body[r].kind);
                        }
                    })
                }
                _ => {
                    // What's derived from `Body` (and gravity, not watched
                    // here) is kept while no `Body` was written since the
                    // last look; then only the velocity is gathered. The
                    // output (`moved`) needs no clearing: the solver starts
                    // it from zero.
                    let mut written = fresh;
                    if !written {
                        masses.for_each_written(s.derived_since, |_, _| written = true);
                    }
                    moving.for_each_page(|page, (body, v, _)| {
                        for (r, &e) in page.entities().iter().enumerate() {
                            let k = index.get(e).expect("a slot per awake body") as usize;
                            if written {
                                bodies[k] = solver_body(&body[r], &v[r], g);
                                kinds[k] = (body[r].kind != STATIC, body[r].kind);
                            } else {
                                bodies[k].v = Vec2::new(v[r].x, v[r].y);
                            }
                        }
                    })
                }
            }
            bodies[n] = SolverBody::default();
            kinds[n] = (false, STATIC);
            t.bodies = us(walk);
            copy.slots = s.kept.by_entity.clone();
        }
    }
    // Angular state: in walk order, each by its slot (lib.rs as built).
    let turn = Instant::now();
    copy.spinning.clear();
    copy.reach.clear();
    {
        let Copy { spinning, reach, bodies, slots, .. } = &mut copy;
        let index: &Slots = match way {
            Way::Hand | Way::HandKept | Way::HandEntity | Way::HandPage => slots,
            _ => &s.kept.by_entity,
        };
        if way == Way::HandPage {
            turning.for_each_page(|page, (body, c, _, spin)| {
                let spin = spin.as_slice();
                for (r, &e) in page.entities().iter().enumerate() {
                    let Some(k) = index.get(e) else { continue };
                    spinning.push(match body[r].kind {
                        DYNAMIC => Spinning::new(k, spin[r].w, bodies[k as usize].inv_mass * c[r].inertia_per_mass()),
                        KINEMATIC => Spinning::new(k, spin[r].w, 0.0),
                        _ => continue,
                    });
                    reach.push(c[r].reach());
                }
            });
        } else {
            turning.for_each(|row, (body, c, _, spin)| {
                let Some(k) = index.get(row.entity()) else { return };
                spinning.push(match body.kind {
                    DYNAMIC => Spinning::new(k, spin.w, bodies[k as usize].inv_mass * c.inertia_per_mass()),
                    KINEMATIC => Spinning::new(k, spin.w, 0.0),
                    _ => return,
                });
                reach.push(c.reach());
            });
        }
    }
    t.turning = us(turn);
    let walk = Instant::now();
    copy.constraints.clear();
    copy.points.clear();
    {
        let still = copy.entities.len() as u32;
        let Copy { constraints, points, slots, .. } = &mut copy;
        let index: &Slots = match way {
            Way::Hand | Way::HandKept | Way::HandEntity | Way::HandPage => slots,
            _ => &s.kept.by_entity,
        };
        let index_of = |e: Entity| index.get(e).unwrap_or(still);
        if way == Way::Hand {
            *constraints = Vec::with_capacity(contacts.len());
            *points = Vec::new();
        }
        contacts.for_each_ordered_page(|page, (pair, m, r, j, cp)| {
            for i in page.rows() {
                if !r[i].disabled {
                    constraints.push(constraint(index_of, &pair[i], &m[i], &r[i], &j[i], &cp[i], points));
                }
            }
        });
    }
    t.contacts = us(walk);
    if way == Way::Derived {
        s.derived_since = masses.now();
    }
    s.times = t;
    s.copies.insert(way.name(), copy);
}

/// The write-back, each way: velocities, positions only where they moved,
/// rotations and spins only where they changed (lib.rs as built).
fn scatter(spikes: &Spikes, moving: &mut Moving, turning: &mut Turning) {
    let mut s = spikes.lock().unwrap();
    let way = s.way;
    let copy = s.copies.remove(way.name()).expect("gathered");
    let start = Instant::now();
    let write = |body: &Body, b: &SolverBody, mut v: engine_ecs::Mut<'_, Velocity>, mut p: engine_ecs::Mut<'_, Position>| {
        (v.x, v.y) = (b.v.x, b.v.y);
        let step = if body.kind == KINEMATIC { b.v * DT } else { b.displacement(DT) };
        let to = (p.x + step.x, p.y + step.y);
        if (to.0.to_bits(), to.1.to_bits()) != (p.x.to_bits(), p.y.to_bits()) {
            (p.x, p.y) = to;
        }
    };
    let index: &Slots = match way {
        Way::Hand | Way::HandKept | Way::HandEntity | Way::HandPage => &copy.slots,
        _ => &s.kept.by_entity,
    };
    if way == Way::HandPage {
        let mut each = copy.spinning.iter();
        turning.for_each_page(|page, (body, _, mut q, mut spin)| {
            for (r, &e) in page.entities().iter().enumerate() {
                let Some(k) = index.get(e) else { continue };
                if body[r].kind != DYNAMIC && body[r].kind != KINEMATIC {
                    continue;
                }
                let b = each.next().expect("a spinning body per one gathered");
                let (q0, w0) = (q.as_slice()[r], spin.as_slice()[r].w);
                let Some(to) = b.turned_from(q0.rot(), w0).filter(|_| copy.kinds[k as usize].0) else { continue };
                if w0.to_bits() != b.w.to_bits() {
                    spin.set(r, Spin { w: b.w });
                }
                if (to.c.to_bits(), to.s.to_bits()) != (q0.c.to_bits(), q0.s.to_bits()) {
                    q.set(r, Rotation::of(to));
                }
            }
        });
    } else if !copy.spinning.is_empty() {
        let mut each = copy.spinning.iter();
        turning.for_each(|row, (body, _, mut q, mut spin)| {
            let Some(k) = index.get(row.entity()) else { return };
            if body.kind != DYNAMIC && body.kind != KINEMATIC {
                return;
            }
            let b = each.next().expect("a spinning body per one gathered");
            debug_assert_eq!(b.body, k);
            let Some(to) = b.turned_from(q.rot(), spin.w).filter(|_| copy.kinds[k as usize].0) else { return };
            if spin.w.to_bits() != b.w.to_bits() {
                spin.w = b.w;
            }
            if (to.c.to_bits(), to.s.to_bits()) != (q.c.to_bits(), q.s.to_bits()) {
                *q = Rotation::of(to);
            }
        });
    }
    match way {
        Way::HandPage => {
            let mut k = 0;
            moving.for_each_page(|page, (body, mut v, mut p)| {
                for r in page.rows() {
                    let (b, moves) = (&copy.bodies[k], copy.kinds[k].0);
                    k += 1;
                    if moves {
                        v.set(r, Velocity { x: b.v.x, y: b.v.y });
                        let step = if body[r].kind == KINEMATIC { b.v * DT } else { b.displacement(DT) };
                        let p0 = p.as_slice()[r];
                        let to = (p0.x + step.x, p0.y + step.y);
                        if (to.0.to_bits(), to.1.to_bits()) != (p0.x.to_bits(), p0.y.to_bits()) {
                            p.set(r, Position { x: to.0, y: to.1 });
                        }
                    }
                }
            });
        }
        Way::Hand | Way::HandKept => {
            let mut i = 0;
            moving.for_each(|_, (body, v, p)| {
                let (b, moves) = (copy.bodies[i], copy.kinds[i].0);
                i += 1;
                if moves {
                    write(body, &b, v, p);
                }
            });
        }
        Way::Column | Way::HandEntity => {
            let mut at = if way == Way::Column { s.column.iter() } else { copy.walk_slot.iter() };
            moving.for_each(|_, (body, v, p)| {
                let k = *at.next().unwrap() as usize;
                if copy.kinds[k].0 {
                    write(body, &copy.bodies[k], v, p);
                }
            });
        }
        _ => {
            moving.for_each(|row, (body, v, p)| {
                let k = index.get(row.entity()).unwrap() as usize;
                if copy.kinds[k].0 {
                    write(body, &copy.bodies[k], v, p);
                }
            });
        }
    }
    s.times.scatter = us(start);
    s.copies.insert(way.name(), copy);
}

/// Every awake body with its angular state, in one walk: what one query
/// with optional `Rotation` and `Spin` would walk. In these scenes every
/// awake body turns, so it is exact (checked).
type All<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a mut Velocity, &'a mut Position, &'a mut Rotation, &'a mut Spin), Without<Asleep>>;

fn gather_one(
    spikes: &Spikes,
    (all, moving): (&mut All, &mut Query<&Body, (With<Velocity>, Without<Asleep>)>),
    contacts: &mut Contacts,
    gravity: &mut Query<&Gravity>,
) {
    let mut s = spikes.lock().unwrap();
    assert_eq!(all.len(), moving.len(), "every awake body turns");
    let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
    let mut copy = s.copies.remove(Way::OneWalk.name()).unwrap_or_default();
    let mut t = Times::default();
    let start = Instant::now();
    let Copy { entities, bodies, kinds, spinning, reach, .. } = &mut copy;
    entities.clear();
    bodies.clear();
    kinds.clear();
    spinning.clear();
    reach.clear();
    // By page: each term's slice taken once a page, as `for_each_page`
    // hands them, so the walk's cost is the rows', not six terms' per-row
    // fetch.
    all.for_each_page(|page, (body, c, v, _, _, spin)| {
        let (v, spin) = (v.as_slice(), spin.as_slice());
        for (r, &e) in page.entities().iter().enumerate() {
            let k = entities.len() as u32;
            entities.push(e);
            let b = solver_body(&body[r], &v[r], g);
            bodies.push(b);
            kinds.push((body[r].kind != STATIC, body[r].kind));
            let sp = match body[r].kind {
                DYNAMIC => Spinning::new(k, spin[r].w, b.inv_mass * c[r].inertia_per_mass()),
                KINEMATIC => Spinning::new(k, spin[r].w, 0.0),
                _ => continue,
            };
            spinning.push(sp);
            reach.push(c[r].reach());
        }
    });

    bodies.push(SolverBody::default());
    kinds.push((false, STATIC));
    let walked = Instant::now();
    t.bodies = (walked - start).as_secs_f64() * 1e6;
    copy.slots = Slots::of(copy.entities.iter().copied());
    t.index = us(walked);
    let walk = Instant::now();
    copy.constraints.clear();
    copy.points.clear();
    {
        let still = copy.entities.len() as u32;
        let Copy { constraints, points, slots, .. } = &mut copy;
        let index_of = |e: Entity| slots.get(e).unwrap_or(still);
        contacts.for_each_ordered_page(|page, (pair, m, r, j, cp)| {
            for i in page.rows() {
                if !r[i].disabled {
                    constraints.push(constraint(index_of, &pair[i], &m[i], &r[i], &j[i], &cp[i], points));
                }
            }
        });
    }
    t.contacts = us(walk);
    s.times = t;
    s.copies.insert(Way::OneWalk.name(), copy);
}

fn scatter_one(spikes: &Spikes, all: &mut All) {
    let mut s = spikes.lock().unwrap();
    let copy = s.copies.remove(Way::OneWalk.name()).expect("gathered");
    let start = Instant::now();
    let mut k = 0;
    let mut each = copy.spinning.iter();
    // By page, each value written only where it changed, so a write still
    // stamps only its row.
    all.for_each_page(|page, (body, _, mut v, mut p, mut q, mut spin)| {
        for r in page.rows() {
            let (b, moves) = (&copy.bodies[k], copy.kinds[k].0);
            k += 1;
            if body[r].kind == DYNAMIC || body[r].kind == KINEMATIC {
                let sb = each.next().expect("a spinning body per one gathered");
                let (q0, w0) = (q.as_slice()[r], spin.as_slice()[r].w);
                if let Some(to) = sb.turned_from(q0.rot(), w0).filter(|_| moves) {
                    if w0.to_bits() != sb.w.to_bits() {
                        spin.set(r, Spin { w: sb.w });
                    }
                    if (to.c.to_bits(), to.s.to_bits()) != (q0.c.to_bits(), q0.s.to_bits()) {
                        q.set(r, Rotation::of(to));
                    }
                }
            }
            if moves {
                v.set(r, Velocity { x: b.v.x, y: b.v.y });
                let step = if body[r].kind == KINEMATIC { b.v * DT } else { b.displacement(DT) };
                let p0 = p.as_slice()[r];
                let to = (p0.x + step.x, p0.y + step.y);
                if (to.0.to_bits(), to.1.to_bits()) != (p0.x.to_bits(), p0.y.to_bits()) {
                    p.set(r, Position { x: to.0, y: to.1 });
                }
            }
        }
    });
    s.times.scatter = us(start);
    s.copies.insert(Way::OneWalk.name(), copy);
}

fn save(spikes: &Spikes, moving: &mut Moving, turning: &mut Turning) {
    let mut s = spikes.lock().unwrap();
    s.saved.clear();
    let mut saved = HashMap::new();
    moving.for_each(|row, (_, v, p)| {
        saved.insert(row.entity(), (*v, *p, None));
    });
    turning.for_each(|row, (_, _, q, spin)| {
        if let Some(x) = saved.get_mut(&row.entity()) {
            x.2 = Some((*q, *spin));
        }
    });
    s.saved = saved;
}

fn restore(spikes: &Spikes, moving: &mut Moving, turning: &mut Turning) {
    let s = spikes.lock().unwrap();
    let bits = |v: &[f32]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    moving.for_each(|row, (_, mut v, mut p)| {
        let (v0, p0, _) = &s.saved[&row.entity()];
        if bits(&[v.x, v.y]) != bits(&[v0.x, v0.y]) {
            *v = *v0;
        }
        if bits(&[p.x, p.y]) != bits(&[p0.x, p0.y]) {
            *p = *p0;
        }
    });
    turning.for_each(|row, (_, _, mut q, mut spin)| {
        let Some((q0, s0)) = s.saved[&row.entity()].2 else { return };
        if bits(&[q.c, q.s]) != bits(&[q0.c, q0.s]) {
            *q = q0;
        }
        if spin.w.to_bits() != s0.w.to_bits() {
            *spin = s0;
        }
    });
}

/// Each body's solved state, by entity, as bits: what two ways must agree on.
fn solved(copy: &Copy) -> HashMap<Entity, Vec<u32>> {
    let mut out: HashMap<Entity, Vec<u32>> = HashMap::new();
    for (k, &e) in copy.entities.iter().enumerate() {
        let b = &copy.bodies[k];
        out.insert(e, [b.v.x, b.v.y, b.moved.x, b.moved.y, b.inv_mass].iter().map(|x| x.to_bits()).collect());
    }
    for sp in &copy.spinning {
        let e = copy.entities[sp.body as usize];
        let x = out.get_mut(&e).unwrap();
        x.extend([sp.w, sp.inv_inertia, sp.turned.c, sp.turned.s, sp.angle].iter().map(|x| x.to_bits()));
    }
    out
}

fn contacts_solved(copy: &Copy) -> Vec<u32> {
    let mut out = Vec::new();
    for c in &copy.constraints {
        out.extend([c.jn, c.jt, c.speed].iter().map(|x| x.to_bits()));
    }
    for p in &copy.points {
        for q in &p.point {
            out.extend([q.jn, q.jt].iter().map(|x| x.to_bits()));
        }
        out.push(p.solved as u32);
    }
    out
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

fn schedule(world: &World, spikes: &Spikes, f: fn(&Spikes, &mut Moving, &mut Turning), name: &str) -> Schedule {
    let s = spikes.clone();
    let sys = move |_: &mut Cx, mut moving: Moving, mut turning: Turning| f(&s, &mut moving, &mut turning);
    Schedule { systems: vec![sys.system(world, name)] }
}

/// The solver's input and output, every way, on the world as it stands.
fn measure(label: &str, world: &World, reps: usize, gang: Option<&Workers>) {
    let spikes: Spikes = Arc::new(Mutex::new(Spike {
        way: Way::Hand,
        copies: HashMap::new(),
        kept: Kept::default(),
        kept_since: 0,
        rebuilt: 0,
        column: Vec::new(),
        derived_since: 0,
        times: Times::default(),
        saved: HashMap::new(),
    }));
    let s = spikes.clone();
    let gather_sys =
        move |_: &mut Cx,
              mut moving: Moving,
              mut masses: Query<&Body, Without<Asleep>>,
              mut turning: Turning,
              mut contacts: Contacts,
              mut gravity: Query<&Gravity>| { gather(&s, (&mut moving, &mut masses), &mut turning, &mut contacts, &mut gravity) };
    let gathers = Schedule { systems: vec![gather_sys.system(world, "gather")] };
    let s = spikes.clone();
    let gather_one_sys =
        move |_: &mut Cx,
              mut all: All,
              mut moving: Query<&Body, (With<Velocity>, Without<Asleep>)>,
              mut contacts: Contacts,
              mut gravity: Query<&Gravity>| { gather_one(&s, (&mut all, &mut moving), &mut contacts, &mut gravity) };
    let gathers_one = Schedule { systems: vec![gather_one_sys.system(world, "gather_one")] };
    let s = spikes.clone();
    let scatter_one_sys = move |_: &mut Cx, mut all: All| scatter_one(&s, &mut all);
    let scatters_one = Schedule { systems: vec![scatter_one_sys.system(world, "scatter_one")] };
    let scatters = schedule(world, &spikes, scatter, "scatter");
    let saves = schedule(world, &spikes, save, "save");
    let restores = schedule(world, &spikes, restore, "restore");
    let s = spikes.clone();
    let tuning_sys = move |_: &mut Cx, mut t: Query<&Tuning>| {
        let _ = &s;
        TUNING.with(|x| x.set(Some(t.single(|_, t| *t).unwrap_or(Tuning::DEFAULT))));
    };
    Schedule { systems: vec![tuning_sys.system(world, "tuning")] }.run_sequential(world);
    let params = solver::Params::of(&TUNING.with(|x| x.get()).unwrap());
    saves.run_sequential(world);
    let one = Workers::new(None);
    let mut times: HashMap<Way, Vec<Times>> = HashMap::new();
    let mut threaded: HashMap<Way, Vec<f64>> = HashMap::new();
    for rep in 0..reps {
        let mut reference: Option<(HashMap<Entity, Vec<u32>>, Vec<u32>)> = None;
        // Each rep in another order of ways, so none is always first after
        // the world was put back.
        let mut ways = WAYS.to_vec();
        ways.rotate_left(rep % WAYS.len());
        for way in ways {
            spikes.lock().unwrap().way = way;
            if way == Way::OneWalk { &gathers_one } else { &gathers }.run_sequential(world);
            let mut copy = spikes.lock().unwrap().copies.remove(way.name()).unwrap();
            let input = copy.clone();
            let start = Instant::now();
            solver::solve_across(&params, (&mut copy.bodies, &mut copy.spinning), &mut copy.constraints, &mut copy.points, DT, &one);
            let solve = us(start);
            if let Some(gang) = gang {
                warm(gang);
                let mut again = input.clone();
                let start = Instant::now();
                solver::solve_across(
                    &params,
                    (&mut again.bodies, &mut again.spinning),
                    &mut again.constraints,
                    &mut again.points,
                    DT,
                    gang,
                );
                threaded.entry(way).or_default().push(us(start));
                assert!(solved(&again) == solved(&copy), "{label}: {} across threads isn't the solve on one", way.name());
            }
            let out = (solved(&copy), contacts_solved(&copy));
            // Hand's is the reference, whichever ran first.
            if way == Way::Hand {
                reference = Some(out.clone());
            }
            RESULTS.with(|r| r.borrow_mut().push((way, out)));
            {
                let mut s = spikes.lock().unwrap();
                s.times.solver = solve;
                s.copies.insert(way.name(), copy);
            }
            if way == Way::OneWalk { &scatters_one } else { &scatters }.run_sequential(world);
            restores.run_sequential(world);
            let t = spikes.lock().unwrap().times;
            times.entry(way).or_default().push(t);
        }
        let reference = reference.expect("hand ran");
        RESULTS.with(|r| {
            for (way, out) in r.borrow_mut().drain(..) {
                assert!(out.0 == reference.0, "{label}: {} solves the bodies otherwise than hand", way.name());
                assert!(out.1 == reference.1, "{label}: {} solves the contacts otherwise than hand", way.name());
            }
        });
    }
    let s = spikes.lock().unwrap();
    let copy = &s.copies[Way::Hand.name()];
    println!(
        "\n### {label}: {} awake bodies, {} turning, {} contacts ({} with points); index rebuilt {} times in {} reps",
        copy.entities.len(),
        copy.spinning.len(),
        copy.constraints.len(),
        copy.points.len(),
        s.rebuilt,
        reps
    );
    println!("\nµs, the median of {reps}; every way's solve bit for bit hand's, by entity\n");
    print!("| way | index | bodies | turning | contacts | gather, all | solver (1 thread) |");
    if gang.is_some() {
        print!(" solver (threads) |");
    }
    println!(" write-back | gather + write-back |");
    print!("|---|---|---|---|---|---|---|");
    if gang.is_some() {
        print!("---|");
    }
    println!("---|---|");
    for way in WAYS {
        let ts = &times[&way];
        let m = |f: fn(&Times) -> f64| median(ts.iter().map(f).collect());
        let all = m(|t| t.index + t.bodies + t.turning + t.contacts);
        let copy = m(|t| t.index + t.bodies + t.turning + t.contacts + t.scatter);
        print!(
            "| {} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} | {:.0} |",
            way.name(),
            m(|t| t.index),
            m(|t| t.bodies),
            m(|t| t.turning),
            m(|t| t.contacts),
            all,
            m(|t| t.solver)
        );
        if gang.is_some() {
            print!(" {:.0} |", median(threaded[&way].clone()));
        }
        println!(" {:.1} | {:.1} |", m(|t| t.scatter), copy);
    }
}

/// Keeps the pool's threads busy for a while, so its cores are clocked up
/// before a solve across them
/// (docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md).
fn warm(gang: &Workers) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(30) {
        gang.run(gang.threads(), |_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_micros(500) {
                std::hint::spin_loop();
            }
        });
    }
}

thread_local! {
    static TUNING: std::cell::Cell<Option<Tuning>> = const { std::cell::Cell::new(None) };
    static RESULTS: std::cell::RefCell<Vec<(Way, (HashMap<Entity, Vec<u32>>, Vec<u32>))>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// The mod's own stages over a window, µs a step: the hand-built copy as
/// the step runs it, beside the frame.
fn in_situ(ecs: &mut ecs::Ecs, steps: u32) -> String {
    ecs.reset();
    ecs.step(steps);
    let stages = ecs.engine().send("physics2d", "stages").unwrap();
    let per = |k: &str| {
        let mut words = stages.split_whitespace();
        words.find(|w| *w == k).unwrap_or_else(|| panic!("no {k} in {stages}"));
        words.next().unwrap().parse::<f64>().unwrap()
    };
    format!(
        "in the mod over {steps} steps, µs a step: find_contacts' gather {:.1}; solve's gather {:.1} (bodies {:.1}, turning {:.1}), solver {:.0}, write-back {:.1} (contacts, touching and sleeping included)",
        per("gather"),
        per("solve_gather"),
        per("solve_bodies"),
        per("solve_turning"),
        per("solver"),
        per("write_back"),
    )
}

/// What a kept index adds to spawning and despawning: a world of `n`
/// bodies, `k` despawned and `k` spawned a frame, with and without the
/// index's insert and swap-remove done beside each.
fn churn(n: usize, k: usize, frames: usize) {
    for keep in [false, true] {
        let world = World::new();
        let index: Arc<Mutex<Kept>> = Arc::default();
        let body = |i: usize| {
            let x = (i % 100) as f32 * 1.1;
            let y = (i / 100) as f32 * 1.1;
            (Position { x, y }, Velocity::default(), Body::default(), Collider::circle(0.5))
        };
        {
            let index = index.clone();
            let spawn = move |_: &mut Cx, spawner: Spawner<(Position, Velocity, Body, Collider)>| {
                let mut index = index.lock().unwrap();
                for i in 0..n {
                    let e = spawner.spawn(body(i));
                    if keep {
                        index.add(e);
                    }
                }
            };
            Schedule { systems: vec![spawn.system(&world, "fill")] }.run_sequential(&world);
        }
        let frame = AtomicCount::default();
        let index2 = index.clone();
        let f2 = frame.clone();
        let step = move |_: &mut Cx,
                         mut bodies: Query<&Position, With<Velocity>, Despawns>,
                         spawner: Spawner<(Position, Velocity, Body, Collider)>| {
            let f = f2.next();
            let mut index = index2.lock().unwrap();
            let mut gone = 0;
            // Every so many rows, so the despawns are spread over pages.
            let stride = (bodies.len() / k).max(1);
            let mut i = 0;
            let mut ids = Vec::with_capacity(k);
            bodies.for_each(|row, _| {
                if gone < k && i % stride == f % stride {
                    row.despawn();
                    ids.push(row.entity());
                    gone += 1;
                }
                i += 1;
            });
            if keep {
                ids.iter().for_each(|&e| index.remove(e));
            }
            for j in 0..k {
                let e = spawner.spawn(body(f * k + j));
                if keep {
                    index.add(e);
                }
            }
        };
        let sched = Schedule { systems: vec![step.system(&world, "churn")] };
        let mut each = Vec::new();
        for _ in 0..frames {
            let start = Instant::now();
            sched.run_sequential(&world);
            each.push(start.elapsed().as_secs_f64() * 1e9 / (2 * k) as f64);
        }
        let index = index.lock().unwrap();
        if keep {
            assert_eq!(index.entities.len(), n, "the index holds every body");
            for (k, &e) in index.entities.iter().enumerate() {
                assert_eq!(index.by_entity.get(e), Some(k as u32));
            }
        }
        println!(
            "| {} | {n} | {k} | {:.1} |",
            if keep { "with the kept index's insert and swap-remove" } else { "the world alone" },
            median(each)
        );
    }
}

#[derive(Clone, Default)]
struct AtomicCount(Arc<std::sync::atomic::AtomicUsize>);

impl AtomicCount {
    fn next(&self) -> usize {
        self.0.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
    }
}

/// The index's own operations, alone: what a world would add per arrival
/// and departure.
fn index_ops(n: usize) {
    let mut kept = Kept::default();
    let es: Vec<Entity> = (0..n as u32).map(|i| Entity { index: i, generation: 0 }).collect();
    let start = Instant::now();
    es.iter().for_each(|&e| kept.add(e));
    let add = start.elapsed().as_secs_f64() * 1e9 / n as f64;
    // Removed in a scattered order, as despawns come.
    let start = Instant::now();
    for i in 0..n {
        kept.remove(es[(i * 7919) % n]);
    }
    let remove = start.elapsed().as_secs_f64() * 1e9 / n as f64;
    assert!(kept.entities.is_empty());
    println!("\nThe index alone, {n} entities: {add:.1} ns an insert, {remove:.1} ns a swap-remove with its fix-up.");
}

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(21);
    let only = std::env::var("ONLY").unwrap_or_default();
    let threads: Option<usize> = std::env::var("THREADS").ok().and_then(|t| t.parse().ok());
    let pool = threads.map(|n| Arc::new(pool::Pool::new(n)));
    let gang = pool.as_ref().map(|p| Workers::new(Some(p.clone() as Arc<dyn Executor>)));
    if let (Some(p), Some(n)) = (&pool, threads) {
        let warm = Instant::now();
        while warm.elapsed() < Duration::from_millis(300) {
            p.run(n, &|_| {
                let t = Instant::now();
                while t.elapsed() < Duration::from_millis(1) {
                    std::hint::spin_loop();
                }
            });
        }
    }
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    let pile = Scene::Pile { n: 10000, width: 401.0, stagger: true };
    let pyramid = Scene::Pyramid { base: 100 };
    // Each case: its scene, the steps before the window, and the window.
    let cases = [
        ("pile 10 000, falling, step 31", pile, 1, 30),
        ("pile 10 000, settled, step 430", pile, 400, 30),
        ("pyramid 5050, step 630", pyramid, 600, 30),
    ];
    for (name, scene, before, window) in cases.iter().filter(|c| only.is_empty() || c.0.contains(only.as_str())) {
        let mut ecs = ecs::Ecs::new(&manifest, scene, false, true);
        ecs.step(*before);
        let situ = in_situ(&mut ecs, *window);
        println!("\n{name}: {situ}");
        measure(name, ecs.engine().world(), reps, gang.as_ref());
    }
    println!("\n### What a kept index adds to structural changes\n");
    println!("ns a spawn or despawn, the median of 41 frames\n");
    println!("| | bodies | despawned and spawned a frame | ns each |");
    println!("|---|---|---|---|");
    churn(10_000, 300, 41);
    churn(10_000, 3000, 41);
    index_ops(1_000_000);
}
