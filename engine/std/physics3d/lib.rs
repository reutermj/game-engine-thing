//! A spike: translation-only 3D rigid bodies in the ECS, to see what 3D asks
//! of the storage core (spatial-storage.md, "In 3D"). Spheres and
//! axis-aligned boxes, no rotation; speculative contacts; the 2D solver
//! (sequential impulses, a split impulse) in 3D; contacts as entities in an
//! ordered table, as in 2D. Not a mod: plain systems for the ECS harness, so
//! the storage is the engine's and the rest is small. Positions are a 3D
//! spatial key, so the broadphase is `near_pairs` and the solver's apply node
//! re-sorts them, as in 2D.
//!
//! What it leaves out of the 2D step: layers and sensors, kinematic bodies,
//! sleeping, events, parallelism, and the tile-seam rule for boxes.

pub mod narrow;
pub mod solver;

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{IntoSystem, Schedule};
use engine_ecs::{Bounds, Despawns, Dt, Entity, OrderKey, Query, SpatialKey, Spawner, With, World, component, near_pairs, pair_key};
use solver::{Constraint, SolverBody};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3 { x, y, z }
    }

    pub const fn splat(v: f32) -> Vec3 {
        Vec3 { x: v, y: v, z: v }
    }

    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn len(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn clamp(self, lo: Vec3, hi: Vec3) -> Vec3 {
        Vec3::new(self.x.clamp(lo.x, hi.x), self.y.clamp(lo.y, hi.y), self.z.clamp(lo.z, hi.z))
    }

    pub fn get(self, axis: usize) -> f32 {
        [self.x, self.y, self.z][axis]
    }

    pub fn set(&mut self, axis: usize, v: f32) {
        match axis {
            0 => self.x = v,
            1 => self.y = v,
            _ => self.z = v,
        }
    }
}

impl std::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl std::ops::Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl std::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

pub const SPHERE: u8 = 0;
pub const BOX: u8 = 1;

/// y is up.
pub const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

component! {
    /// A body's center.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Position: "physics3d::Position", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

impl SpatialKey<3> for Position {
    type Extent = Collider;
    #[inline]
    fn bounds(&self, c: Option<&Collider>) -> Bounds<3> {
        let half = c.map_or([0.0; 3], |c| if c.shape == BOX { [c.hx, c.hy, c.hz] } else { [c.hx; 3] });
        Bounds::around([self.x, self.y, self.z], half)
    }
}

impl Position {
    pub fn at(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

component! {
    /// Units per second; a body without one is static.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Velocity: "physics3d::Velocity" { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Body: "physics3d::Body" { pub inv_mass: f32, pub friction: f32, pub restitution: f32 }
}

component! {
    /// A sphere (radius `hx`) or a box (half extents), centered on the position.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Collider: "physics3d::Collider" { pub shape: u8, pub hx: f32, pub hy: f32, pub hz: f32 }
}

component! {
    /// A body that never moves: in tables of its own, the broadphase's
    /// passive side, so pairs of two statics aren't looked for.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Static: "physics3d::Static" {}
}

component! {
    /// Two bodies touching or about to, lesser entity first: an entity
    /// while it lasts, in an ordered table by pair, as in 2D.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ContactPair: "physics3d::ContactPair", order = key { pub a: Entity, pub b: Entity }
}

impl OrderKey for ContactPair {
    #[inline]
    fn key(&self) -> u128 {
        pair_key(self.a, self.b)
    }
}

component! {
    /// The contact's geometry this step, and what the pair does with it.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Manifold: "physics3d::Manifold" {
        pub nx: f32, pub ny: f32, pub nz: f32, pub depth: f32,
        pub friction: f32, pub restitution: f32,
    }
}

component! {
    /// The last step's impulses, for warm starting: along the normal, and
    /// friction's as a vector in the tangent plane (see the solver).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Impulse: "physics3d::Impulse" { pub normal: f32, pub tx: f32, pub ty: f32, pub tz: f32 }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Sphere(f32),
    Box(Vec3),
}

impl Collider {
    pub fn sphere(r: f32) -> Collider {
        Collider { shape: SPHERE, hx: r, hy: r, hz: r }
    }

    pub fn cuboid(h: Vec3) -> Collider {
        Collider { shape: BOX, hx: h.x, hy: h.y, hz: h.z }
    }

    pub fn of(&self) -> Shape {
        if self.shape == BOX { Shape::Box(Vec3::new(self.hx, self.hy, self.hz)) } else { Shape::Sphere(self.hx) }
    }
}

impl Body {
    pub fn new(inv_mass: f32) -> Body {
        Body { inv_mass, friction: 0.5, restitution: 0.0 }
    }
}

/// Nanoseconds in each stage, summed over steps: for the benchmark. A
/// static because harness systems are plain functions; one simulation runs
/// at a time.
#[derive(Clone, Copy, Debug, Default)]
pub struct Timings {
    pub steps: u64,
    pub gravity: u64,
    pub gather: u64,
    pub broadphase: u64,
    pub narrowphase: u64,
    pub merge: u64,
    pub solve_gather: u64,
    pub solver: u64,
    pub write_back: u64,
    /// Found this step: pairs from the broadphase, and contacts.
    pub pairs: usize,
    pub contacts: usize,
}

pub static TIMINGS: Mutex<Timings> = Mutex::new(Timings {
    steps: 0,
    gravity: 0,
    gather: 0,
    broadphase: 0,
    narrowphase: 0,
    merge: 0,
    solve_gather: 0,
    solver: 0,
    write_back: 0,
    pairs: 0,
    contacts: 0,
});

fn nanos(from: Instant, to: Instant) -> u64 {
    (to - from).as_nanos() as u64
}

/// Entities to positions in a list, by entity index, as in 2D.
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

fn integrate(_: &mut engine_ecs::harness::Cx, dt: Dt, mut bodies: Query<(&Body, &mut Velocity)>) {
    let start = Instant::now();
    let g = GRAVITY * *dt;
    bodies.for_each(|_, (body, mut v)| {
        if body.inv_mass > 0.0 {
            (v.x, v.y, v.z) = (v.x + g.x, v.y + g.y, v.z + g.z);
        }
    });
    let mut t = TIMINGS.lock().unwrap();
    t.steps += 1;
    t.gravity += nanos(start, Instant::now());
}

struct Item {
    entity: Entity,
    at: Vec3,
    shape: Shape,
    friction: f32,
    restitution: f32,
}

type Moving<'w, 'a> = Query<'w, (&'a Position, &'a Collider, &'a Body, &'a Velocity)>;
type Statics<'w, 'a> = Query<'w, (&'a Position, &'a Collider, &'a Body), With<Static>>;

/// Finds this step's contacts, and brings the world's in line in one pass,
/// both in pair order, as the 2D step's `find_contacts` does.
fn find_contacts(
    _: &mut engine_ecs::harness::Cx,
    mut moving: Moving<'_, '_>,
    mut statics: Statics<'_, '_>,
    mut contacts: Query<(&ContactPair, &mut Manifold), (), Despawns>,
    new_contacts: Spawner<(ContactPair, Manifold, Impulse)>,
) {
    let start = Instant::now();
    let mut items = Vec::with_capacity(moving.len() + statics.len());
    let item = |entity, p: &Position, c: &Collider, b: &Body| Item {
        entity,
        at: p.at(),
        shape: c.of(),
        friction: b.friction,
        restitution: b.restitution,
    };
    moving.for_each(|row, (p, c, b, _)| items.push(item(row.entity(), p, c, b)));
    statics.for_each(|row, (p, c, b)| items.push(item(row.entity(), p, c, b)));
    let slots = Slots::of(items.iter().map(|i| i.entity));
    let gathered = Instant::now();
    let near = near_pairs(&moving, &statics, narrow::MARGIN);
    let paired = Instant::now();
    let mut found: Vec<(ContactPair, Manifold)> = Vec::with_capacity(near.len());
    for &(a, b) in &near {
        let (i, j) = (&items[slots.get(a).expect("gathered") as usize], &items[slots.get(b).expect("gathered") as usize]);
        if let Some(m) = narrow::collide(i.at, i.shape, j.at, j.shape) {
            // Mixed as Box2D does: friction by geometric mean, restitution
            // by the larger.
            let manifold = Manifold {
                nx: m.normal.x,
                ny: m.normal.y,
                nz: m.normal.z,
                depth: m.depth,
                friction: (i.friction * j.friction).sqrt(),
                restitution: i.restitution.max(j.restitution),
            };
            found.push((ContactPair { a, b }, manifold));
        }
    }
    let narrowed = Instant::now();
    let key = |p: &ContactPair| (p.a, p.b);
    let spawn = |(pair, m): (ContactPair, Manifold)| {
        new_contacts.spawn((pair, m, Impulse::default()));
    };
    let mut next = 0;
    contacts.for_each_ordered_page(|page, (pair, mut m)| {
        let m = m.write_all();
        for i in page.rows() {
            while found.get(next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                spawn(found[next]);
                next += 1;
            }
            match found.get(next) {
                Some(f) if f.0 == pair[i] => {
                    m[i] = f.1;
                    next += 1;
                }
                _ => page.row(i).despawn(),
            }
        }
    });
    found[next..].iter().copied().for_each(spawn);
    let mut t = TIMINGS.lock().unwrap();
    let end = Instant::now();
    t.gather += nanos(start, gathered);
    t.broadphase += nanos(gathered, paired);
    t.narrowphase += nanos(paired, narrowed);
    t.merge += nanos(narrowed, end);
    (t.pairs, t.contacts) = (near.len(), found.len());
}

fn solve(
    _: &mut engine_ecs::harness::Cx,
    dt: Dt,
    mut moving: Query<(&Body, &mut Velocity, &mut Position)>,
    mut contacts: Query<(&ContactPair, &Manifold, &mut Impulse)>,
) {
    let start = Instant::now();
    let dt = *dt;
    let mut bodies = Vec::with_capacity(moving.len() + 1);
    let mut entities = Vec::with_capacity(moving.len());
    moving.for_each(|row, (body, v, _)| {
        entities.push(row.entity());
        bodies.push(SolverBody { v: Vec3::new(v.x, v.y, v.z), inv_mass: body.inv_mass, pseudo: Vec3::ZERO });
    });
    // Statics all stand for one immovable body at the end.
    let still = bodies.len() as u32;
    bodies.push(SolverBody::default());
    let slots = Slots::of(entities.iter().copied());
    let index = |e: Entity| slots.get(e).unwrap_or(still);
    let mut constraints = Vec::with_capacity(contacts.len());
    contacts.for_each_ordered(|_, (pair, m, j)| {
        constraints.push(Constraint {
            a: index(pair.a),
            b: index(pair.b),
            normal: Vec3::new(m.nx, m.ny, m.nz),
            depth: m.depth,
            friction: m.friction,
            restitution: m.restitution,
            jn: j.normal,
            jt: Vec3::new(j.tx, j.ty, j.tz),
        })
    });
    let gathered = Instant::now();
    solver::solve(&mut bodies, &mut constraints, dt);
    let solved = Instant::now();
    let mut k = 0;
    contacts.for_each_ordered_page(|page, (_, _, mut j)| {
        let j = j.write_all();
        for i in page.rows() {
            let c = &constraints[k];
            j[i] = Impulse { normal: c.jn, tx: c.jt.x, ty: c.jt.y, tz: c.jt.z };
            k += 1;
        }
    });
    let mut k = 0;
    moving.for_each(|_, (_, mut v, mut p)| {
        let b = &bodies[k];
        k += 1;
        (v.x, v.y, v.z) = (b.v.x, b.v.y, b.v.z);
        let step = b.v + b.pseudo;
        // Written only when it moves, as in 2D: a write re-bounds the row.
        let to = (p.x + step.x * dt, p.y + step.y * dt, p.z + step.z * dt);
        if (to.0.to_bits(), to.1.to_bits(), to.2.to_bits()) != (p.x.to_bits(), p.y.to_bits(), p.z.to_bits()) {
            (p.x, p.y, p.z) = to;
        }
    });
    let end = Instant::now();
    let mut t = TIMINGS.lock().unwrap();
    t.solve_gather += nanos(start, gathered);
    t.solver += nanos(gathered, solved);
    t.write_back += nanos(solved, end);
}

/// The step: gravity, contacts, the solve, each a system whose apply node
/// lands its changes (the contacts' spawns and despawns, the solve's moves,
/// re-sorted) before the next runs.
pub fn step(world: &World) -> Schedule {
    Schedule {
        systems: vec![
            integrate.system(world, "physics3d::integrate"),
            find_contacts.system(world, "physics3d::find_contacts"),
            solve.system(world, "physics3d::solve"),
        ],
    }
}
