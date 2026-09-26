//! Experimental 3D rigid bodies in the ECS, to see what 3D asks of the
//! storage core (spatial-storage.md, "In 3D") and how its physics compares
//! with engines people ship (//bench/physics3d). Spheres and boxes that
//! turn; speculative contacts of up to four points; the 2D step's soft
//! solver with angular terms; contacts as entities in an ordered table, as
//! in 2D. Not a mod: plain systems for the ECS harness, so the storage is
//! the engine's and the rest is small. Positions are a 3D spatial key, so
//! the broadphase is `near_pairs` and the solver's apply node re-sorts
//! them, as in 2D.
//!
//! What it leaves out of the 2D step: layers and sensors, kinematic bodies,
//! sleeping, events, parallelism, and the tile-seam rule for boxes. Design
//! and measurements: physics.md, "Rotation in 3D".

pub mod gjk;
pub mod math;
pub mod narrow;
pub mod solver;

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{IntoSystem, Schedule};
use engine_ecs::{Bounds, Despawns, Dt, Entity, OrderKey, Query, SpatialKey, Spawner, With, World, component, near_pairs, pair_key};
pub use math::{Mat3, Quat, Vec3};
use narrow::{Narrow, Solid};
use solver::{Constraint, ContactPoint, SolverBody};

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
    type Extent = Reach;
    #[inline]
    fn bounds(&self, r: Option<&Reach>) -> Bounds<3> {
        let r = r.copied().unwrap_or_default();
        Bounds::around([self.x, self.y, self.z], [r.x, r.y, r.z])
    }
}

component! {
    /// Half the size of a body's bounds along each world axis: the key's
    /// extent, so writing it re-bounds the row. The collider can't be the
    /// extent once bodies turn, since a turned box's bounds depend on its
    /// rotation too and a key's bounds see one extent. Which bounds a body
    /// gets is `Tuning::bounds` (physics.md, "Rotation in 3D").
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Reach: "physics3d::Reach" { pub x: f32, pub y: f32, pub z: f32 }
}

/// Which bounds a body's `Reach` holds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BoundsOf {
    /// The box around the collider as it is turned now, rewritten as it
    /// turns: what Box3D, Jolt and Rapier all do.
    #[default]
    Turned,
    /// A cube around the sphere around the collider, for bodies that can
    /// turn: never rewritten, but a box's is up to 1.7 times as wide.
    /// Bodies that can't turn (statics, locked) get the box around them.
    Sphere,
    /// A sphere around every collider, statics too: what the collider as
    /// the extent could give.
    SphereAll,
}

impl Reach {
    /// The bounds of `c` turned to `q`, for a body that `turns` or not.
    pub fn of(c: &Collider, q: Quat, turns: bool, how: BoundsOf) -> Reach {
        let sphere = how == BoundsOf::SphereAll || (how == BoundsOf::Sphere && turns);
        if c.shape != BOX || sphere {
            let r = c.reach();
            return Reach { x: r, y: r, z: r };
        }
        let m = q.matrix().cols;
        let h = Vec3::new(c.hx, c.hy, c.hz);
        // Each world axis: the box's axes' reach along it, |R| h.
        let along = |k: usize| h.x * m[0].get(k).abs() + h.y * m[1].get(k).abs() + h.z * m[2].get(k).abs();
        Reach { x: along(0), y: along(1), z: along(2) }
    }
}

impl Position {
    pub fn at(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

component! {
    /// A body's orientation, a unit quaternion: `(x, y, z)` the vector
    /// part, `w` the scalar.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Rotation: "physics3d::Rotation" { pub x: f32, pub y: f32, pub z: f32, pub w: f32 }
}

impl Default for Rotation {
    fn default() -> Rotation {
        Rotation::from(Quat::IDENTITY)
    }
}

impl From<Quat> for Rotation {
    fn from(q: Quat) -> Rotation {
        Rotation { x: q.v.x, y: q.v.y, z: q.v.z, w: q.w }
    }
}

impl Rotation {
    pub fn quat(&self) -> Quat {
        Quat { v: Vec3::new(self.x, self.y, self.z), w: self.w }
    }
}

component! {
    /// Units per second; a body without one is static.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Velocity: "physics3d::Velocity" { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    /// Radians per second about each world axis.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct AngularVelocity: "physics3d::AngularVelocity" { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    /// `(ix, iy, iz)` is the inverse inertia about the body's own axes:
    /// zero for a body that doesn't turn.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Body: "physics3d::Body" {
        pub inv_mass: f32, pub friction: f32, pub restitution: f32,
        pub ix: f32, pub iy: f32, pub iz: f32,
    }
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
    /// The contact this step: its normal (from `a` to `b`), `a`'s center
    /// less `b`'s (so each point's anchor on `b` is its anchor on `a` plus
    /// this), and up to four points inline, each its anchor on `a` (from
    /// its center, in world axes) and depth, four floats a point, with its
    /// feature id; and the separating axis that found it, for the next step
    /// to try first. Inline because a contact's points are read and
    /// written together (spatial-storage.md, "In 3D").
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Manifold: "physics3d::Manifold" {
        pub nx: f32, pub ny: f32, pub nz: f32,
        pub ox: f32, pub oy: f32, pub oz: f32,
        pub friction: f32, pub restitution: f32,
        pub count: u32,
        pub points: [f32; 4 * narrow::MAX_POINTS],
        pub ids: [u32; narrow::MAX_POINTS],
        pub axis: u32, pub axis_sep: f32,
    }
}

impl Manifold {
    pub fn normal(&self) -> Vec3 {
        Vec3::new(self.nx, self.ny, self.nz)
    }

    pub fn offset(&self) -> Vec3 {
        Vec3::new(self.ox, self.oy, self.oz)
    }

    /// Point `k`'s anchor on `a` and its depth.
    pub fn point(&self, k: usize) -> (Vec3, f32) {
        let p = &self.points[4 * k..4 * k + 4];
        (Vec3::new(p[0], p[1], p[2]), p[3])
    }

    pub fn deepest(&self) -> f32 {
        (0..self.count as usize).map(|k| self.point(k).1).fold(f32::MIN, f32::max)
    }
}

component! {
    /// The last step's impulses, for warm starting: along the normal at
    /// each point, and friction's for the whole contact, as a vector in the
    /// tangent plane and a twist about the normal (see the solver).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Impulse: "physics3d::Impulse" { pub normal: [f32; narrow::MAX_POINTS], pub tx: f32, pub ty: f32, pub tz: f32, pub twist: f32 }
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

    /// The radius of a sphere around it, whichever way it's turned.
    pub fn reach(&self) -> f32 {
        if self.shape == BOX { (self.hx * self.hx + self.hy * self.hy + self.hz * self.hz).sqrt() } else { self.hx }
    }
}

/// What a static body is spawned with.
pub fn fixed(at: Vec3, rot: Quat, c: Collider) -> (Position, Rotation, Reach, Collider, Body, Static) {
    let reach = Reach::of(&c, rot, false, tuning().bounds);
    (Position { x: at.x, y: at.y, z: at.z }, Rotation::from(rot), reach, c, Body::new(0.0), Static {})
}

/// What a moving body is spawned with, at rest.
pub fn dynamic(at: Vec3, rot: Quat, c: Collider, body: Body) -> (Position, Rotation, Reach, Collider, Body, Velocity, AngularVelocity) {
    let reach = Reach::of(&c, rot, body.turns(), tuning().bounds);
    (Position { x: at.x, y: at.y, z: at.z }, Rotation::from(rot), reach, c, body, Velocity::default(), AngularVelocity::default())
}

impl Body {
    /// A body that doesn't turn, as the translation-only step had them.
    pub fn new(inv_mass: f32) -> Body {
        Body { inv_mass, friction: 0.5, restitution: 0.0, ..Default::default() }
    }

    /// A solid body of the collider's shape, turning: a sphere's inertia is
    /// 2/5 m r², a box's m/3 (b² + c²) about each axis from its half extents.
    pub fn solid(inv_mass: f32, c: &Collider) -> Body {
        let (x, y, z) = if c.shape == BOX {
            (3.0 / (c.hy * c.hy + c.hz * c.hz), 3.0 / (c.hx * c.hx + c.hz * c.hz), 3.0 / (c.hx * c.hx + c.hy * c.hy))
        } else {
            let i = 2.5 / (c.hx * c.hx);
            (i, i, i)
        };
        Body { ix: x * inv_mass, iy: y * inv_mass, iz: z * inv_mass, ..Body::new(inv_mass) }
    }

    pub fn inv_inertia(&self) -> Vec3 {
        Vec3::new(self.ix, self.iy, self.iz)
    }

    pub fn turns(&self) -> bool {
        self.inv_inertia() != Vec3::ZERO
    }
}

/// How the step is done, where more than one way was measured: the
/// defaults are what was chosen (physics.md, "Rotation in 3D"). A static
/// so the bench can switch it; one simulation runs at a time.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Tuning {
    pub narrow: Narrow,
    pub warm: Warm,
    pub solver: solver::Tuning,
    pub bounds: BoundsOf,
}

/// How a contact's points find the last step's impulses.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Warm {
    /// By feature id (Box3D, Rapier).
    #[default]
    Ids,
    /// The nearest last point within 1 cm (Jolt's rule, but in world axes
    /// relative to `a`, where Jolt's is in each body's own).
    Nearest,
    /// Not at all.
    Cold,
}

impl Tuning {
    /// From a list like "sub=4,relax=1,stiff=0.125,warm=cold": how the
    /// bench names a variant. Unknown keys are refused.
    pub fn parse(s: &str) -> Result<Tuning, String> {
        use narrow::{BoxBox, Reduce};
        use solver::{Anchors, Inertia, Integrate};
        let mut t = Tuning::default();
        for kv in s.split(',').filter(|kv| !kv.is_empty()) {
            let (k, v) = kv.split_once('=').ok_or(format!("{kv}: not key=value"))?;
            let num = || v.parse::<f32>().map_err(|e| format!("{kv}: {e}"));
            match (k, v) {
                ("sub", _) => t.solver.substeps = num()? as usize,
                ("relax", _) => t.solver.relax = num()? as usize,
                ("stiff", _) => t.solver.stiffness = num()?,
                ("static", _) => t.solver.static_stiffness = num()?,
                ("fpush", _) => t.solver.friction_in_push = num()? != 0.0,
                ("int", "linear") => t.solver.integrate = Integrate::Linear,
                ("int", "once") => t.solver.integrate = Integrate::LinearOnce,
                ("int", "exact") => t.solver.integrate = Integrate::Exact,
                ("inertia", "step") => t.solver.inertia = Inertia::Step,
                ("inertia", "substep") => t.solver.inertia = Inertia::Substep,
                ("anchors", "exact") => t.solver.anchors = Anchors::Exact,
                ("anchors", "linear") => t.solver.anchors = Anchors::Linear,
                ("warm", "ids") => t.warm = Warm::Ids,
                ("warm", "nearest") => t.warm = Warm::Nearest,
                ("warm", "cold") => t.warm = Warm::Cold,
                ("bb", "sat") => t.narrow.box_box = BoxBox::Sat,
                ("bb", "cached") => t.narrow.box_box = BoxBox::SatCached,
                ("bb", "gjk") => t.narrow.box_box = BoxBox::GjkEpa,
                ("reduce", "area") => t.narrow.reduce = Reduce::Area,
                ("reduce", "line") => t.narrow.reduce = Reduce::Line,
                ("bounds", "turned") => t.bounds = BoundsOf::Turned,
                ("bounds", "sphere") => t.bounds = BoundsOf::Sphere,
                ("bounds", "sphere_all") => t.bounds = BoundsOf::SphereAll,
                _ => return Err(format!("{kv}: unknown")),
            }
        }
        Ok(t)
    }
}

pub static TUNING: Mutex<Option<Tuning>> = Mutex::new(None);

pub fn tuning() -> Tuning {
    TUNING.lock().unwrap().unwrap_or_default()
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
    /// Found this step: pairs from the broadphase, contacts, and points.
    pub pairs: usize,
    pub contacts: usize,
    pub points: usize,
    /// Of the points of contacts that were there last step, how many found
    /// their last impulse.
    pub kept: usize,
    pub matched: usize,
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
    points: 0,
    kept: 0,
    matched: 0,
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
    solid: Solid,
    friction: f32,
    restitution: f32,
}

type Moving<'w, 'a> = Query<'w, (&'a Position, &'a Rotation, &'a Collider, &'a Body, &'a Velocity)>;
type Statics<'w, 'a> = Query<'w, (&'a Position, &'a Rotation, &'a Collider, &'a Body), With<Static>>;

/// This step's manifold as stored: anchors on `a`, from its center.
fn stored(m: &narrow::Manifold, a: &Item, b: &Item) -> Manifold {
    let mut points = [0.0; 4 * narrow::MAX_POINTS];
    let mut ids = [0; narrow::MAX_POINTS];
    for (k, p) in m.points().iter().enumerate() {
        let r = p.at - a.solid.at;
        points[4 * k..4 * k + 4].copy_from_slice(&[r.x, r.y, r.z, p.depth]);
        ids[k] = p.id;
    }
    let o = a.solid.at - b.solid.at;
    // Mixed as Box2D does: friction by geometric mean, restitution by the
    // larger.
    Manifold {
        nx: m.normal.x,
        ny: m.normal.y,
        nz: m.normal.z,
        ox: o.x,
        oy: o.y,
        oz: o.z,
        friction: (a.friction * b.friction).sqrt(),
        restitution: a.restitution.max(b.restitution),
        count: m.count as u32,
        points,
        ids,
        axis: m.axis,
        axis_sep: m.axis_sep,
    }
}

/// The last step's impulses carried to this step's points: each point's
/// from the last point it matches, and the contact's friction whole.
fn warm(prev: Option<(&Manifold, &Impulse)>, m: &Manifold, how: Warm, count: &mut usize) -> Impulse {
    let Some((old, j)) = prev else { return Impulse::default() };
    if how == Warm::Cold {
        return Impulse::default();
    }
    let mut out = Impulse { tx: j.tx, ty: j.ty, tz: j.tz, twist: j.twist, ..Default::default() };
    for k in 0..m.count as usize {
        let matched = (0..old.count as usize).find(|&o| match how {
            Warm::Ids => old.ids[o] == m.ids[k],
            _ => {
                let d = old.point(o).0 - m.point(k).0;
                d.dot(d) < 1e-4
            }
        });
        if let Some(o) = matched {
            out.normal[k] = j.normal[o];
            *count += 1;
        }
    }
    out
}

/// Finds this step's contacts, and brings the world's in line in one pass,
/// both in pair order, as the 2D step's `find_contacts` does.
fn find_contacts(
    _: &mut engine_ecs::harness::Cx,
    mut moving: Moving<'_, '_>,
    mut statics: Statics<'_, '_>,
    mut contacts: Query<(&ContactPair, &mut Manifold, &mut Impulse), (), Despawns>,
    new_contacts: Spawner<(ContactPair, Manifold, Impulse)>,
) {
    let how = tuning();
    let start = Instant::now();
    let mut items = Vec::with_capacity(moving.len() + statics.len());
    let item = |entity, p: &Position, q: &Rotation, c: &Collider, b: &Body| Item {
        entity,
        solid: Solid { at: p.at(), rot: q.quat().matrix(), shape: c.of() },
        friction: b.friction,
        restitution: b.restitution,
    };
    moving.for_each(|row, (p, q, c, b, _)| items.push(item(row.entity(), p, q, c, b)));
    statics.for_each(|row, (p, q, c, b)| items.push(item(row.entity(), p, q, c, b)));
    let slots = Slots::of(items.iter().map(|i| i.entity));
    // The last step's contacts, for their separating axes and impulses: in
    // pair order, as `near` is.
    let mut old = Vec::with_capacity(contacts.len());
    contacts.for_each_ordered(|_, (pair, m, j)| old.push((*pair, *m, *j)));
    let gathered = Instant::now();
    let near = near_pairs(&moving, &statics, narrow::MARGIN);
    let paired = Instant::now();
    let key = |p: &ContactPair| (p.a, p.b);
    let mut found: Vec<(ContactPair, Manifold, Impulse)> = Vec::with_capacity(near.len());
    let mut o = 0;
    let (mut points, mut kept, mut matched) = (0, 0, 0);
    for &(a, b) in &near {
        let (i, j) = (&items[slots.get(a).expect("gathered") as usize], &items[slots.get(b).expect("gathered") as usize]);
        let pair = ContactPair { a, b };
        while old.get(o).is_some_and(|x: &(ContactPair, Manifold, Impulse)| key(&x.0) < (a, b)) {
            o += 1;
        }
        let prev = old.get(o).filter(|x| x.0 == pair).map(|x| (&x.1, &x.2));
        let cache = prev.map_or((0, 0.0), |(m, _)| (m.axis, m.axis_sep));
        if let Some(m) = narrow::collide(&i.solid, &j.solid, cache, how.narrow) {
            let manifold = stored(&m, i, j);
            points += m.count;
            kept += if prev.is_some() { m.count } else { 0 };
            found.push((pair, manifold, warm(prev, &manifold, how.warm, &mut matched)));
        }
    }
    let narrowed = Instant::now();
    let spawn = |f: (ContactPair, Manifold, Impulse)| {
        new_contacts.spawn(f);
    };
    let mut next = 0;
    contacts.for_each_ordered_page(|page, (pair, mut m, mut j)| {
        let (m, j) = (m.write_all(), j.write_all());
        for i in page.rows() {
            while found.get(next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                spawn(found[next]);
                next += 1;
            }
            match found.get(next) {
                Some(f) if f.0 == pair[i] => {
                    (m[i], j[i]) = (f.1, f.2);
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
    (t.pairs, t.contacts, t.points, t.kept, t.matched) = (near.len(), found.len(), points, kept, matched);
}

type Bodies<'w, 'a> =
    Query<'w, (&'a Body, &'a Collider, &'a mut Velocity, &'a mut AngularVelocity, &'a mut Position, &'a mut Rotation, &'a mut Reach)>;

fn solve(
    _: &mut engine_ecs::harness::Cx,
    dt: Dt,
    mut moving: Bodies<'_, '_>,
    mut contacts: Query<(&ContactPair, &Manifold, &mut Impulse)>,
) {
    let how = tuning();
    let start = Instant::now();
    let dt = *dt;
    let mut bodies = Vec::with_capacity(moving.len() + 1);
    let mut entities = Vec::with_capacity(moving.len());
    moving.for_each(|row, (body, _, v, w, _, q, _)| {
        entities.push(row.entity());
        let gravity = if body.inv_mass > 0.0 { GRAVITY * dt } else { Vec3::ZERO };
        let (v, w) = (Vec3::new(v.x, v.y, v.z), Vec3::new(w.x, w.y, w.z));
        bodies.push(SolverBody::new(v, w, body.inv_mass, body.inv_inertia(), q.quat(), gravity));
    });
    // Statics all stand for one immovable body at the end.
    let still = bodies.len() as u32;
    bodies.push(SolverBody::default());
    let slots = Slots::of(entities.iter().copied());
    let index = |e: Entity| slots.get(e).unwrap_or(still);
    let mut constraints = Vec::with_capacity(contacts.len());
    contacts.for_each_ordered(|_, (pair, m, j)| {
        let mut points = [ContactPoint::default(); narrow::MAX_POINTS];
        for (k, p) in points[..m.count as usize].iter_mut().enumerate() {
            let (ra, depth) = m.point(k);
            *p = ContactPoint { ra, depth, jn: j.normal[k], speed: 0.0 };
        }
        constraints.push(Constraint {
            a: index(pair.a),
            b: index(pair.b),
            normal: m.normal(),
            offset: m.offset(),
            friction: m.friction,
            restitution: m.restitution,
            count: m.count as usize,
            points,
            jt: Vec3::new(j.tx, j.ty, j.tz),
            twist: j.twist,
        })
    });
    let gathered = Instant::now();
    solver::solve(&mut bodies, &mut constraints, dt, &how.solver);
    let solved = Instant::now();
    let mut k = 0;
    contacts.for_each_ordered_page(|page, (_, _, mut j)| {
        let j = j.write_all();
        for i in page.rows() {
            let c = &constraints[k];
            let mut normal = [0.0; narrow::MAX_POINTS];
            for (n, p) in normal.iter_mut().zip(&c.points[..c.count]) {
                *n = p.jn;
            }
            j[i] = Impulse { normal, tx: c.jt.x, ty: c.jt.y, tz: c.jt.z, twist: c.twist };
            k += 1;
        }
    });
    let mut k = 0;
    moving.for_each(|_, (body, c, mut v, mut w, mut p, mut q, mut reach)| {
        let b = &bodies[k];
        k += 1;
        (v.x, v.y, v.z) = (b.v.x, b.v.y, b.v.z);
        (w.x, w.y, w.z) = (b.w.x, b.w.y, b.w.z);
        // Written only when it moves, as in 2D: a write re-bounds the row.
        let to = (p.x + b.moved.x, p.y + b.moved.y, p.z + b.moved.z);
        if (to.0.to_bits(), to.1.to_bits(), to.2.to_bits()) != (p.x.to_bits(), p.y.to_bits(), p.z.to_bits()) {
            (p.x, p.y, p.z) = to;
        }
        if b.turned != Quat::IDENTITY {
            let turned = b.rotation();
            *q = Rotation::from(turned);
            // Only a box turned re-bounds, and only when its box changed.
            if how.bounds == BoundsOf::Turned && c.shape == BOX {
                let r = Reach::of(c, turned, body.turns(), how.bounds);
                if (r.x.to_bits(), r.y.to_bits(), r.z.to_bits()) != (reach.x.to_bits(), reach.y.to_bits(), reach.z.to_bits()) {
                    *reach = r;
                }
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solid_bodies_have_the_inertia_of_their_shape() {
        // A unit cube of mass 2: I = m (1² + 1²) / 12 = 1/3 about each axis.
        let b = Body::solid(0.5, &Collider::cuboid(Vec3::splat(0.5)));
        assert!((b.inv_inertia() - Vec3::splat(3.0)).len() < 1e-5, "{b:?}");
        // A plank 2 x 0.5 x 1 of mass 1: I_x = (0.25 + 1) / 12.
        let b = Body::solid(1.0, &Collider::cuboid(Vec3::new(1.0, 0.25, 0.5)));
        let i = Vec3::new((0.25 + 1.0) / 12.0, (4.0 + 1.0) / 12.0, (4.0 + 0.25) / 12.0);
        assert!((b.inv_inertia() - Vec3::new(1.0 / i.x, 1.0 / i.y, 1.0 / i.z)).len() < 1e-3, "{b:?}");
        // A sphere of radius 0.5 and mass 1: I = 2/5 m r² = 0.1.
        let b = Body::solid(1.0, &Collider::sphere(0.5));
        assert!((b.inv_inertia() - Vec3::splat(10.0)).len() < 1e-4, "{b:?}");
        assert!(!Body::new(1.0).turns());
    }

    #[test]
    fn a_turned_boxs_reach_is_the_box_around_it() {
        let c = Collider::cuboid(Vec3::new(1.0, 0.25, 0.5));
        let q = Quat::axis_angle(Vec3::Z, std::f32::consts::FRAC_PI_2);
        let r = Reach::of(&c, q, true, BoundsOf::Turned);
        assert!((Vec3::new(r.x, r.y, r.z) - Vec3::new(0.25, 1.0, 0.5)).len() < 1e-5, "{r:?}");
        let s = Reach::of(&c, q, true, BoundsOf::Sphere);
        assert!((s.x - c.reach()).abs() < 1e-6 && s.x == s.y && s.y == s.z);
        let fixed = Reach::of(&c, Quat::IDENTITY, false, BoundsOf::Sphere);
        assert_eq!((fixed.x, fixed.y, fixed.z), (1.0, 0.25, 0.5), "a body that can't turn keeps its box");
    }
}
