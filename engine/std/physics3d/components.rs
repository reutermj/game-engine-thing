//! What `physics3d` owns and other mods use: 3D bodies that turn, their
//! colliders, the contacts between them, and the settings the step reads
//! from the world (`Gravity`, `Tuning`), as 2D keeps `Gravity` and `Sleep`.
//! Experimental: see docs/architecture/physics.md, "Rotation in 3D".
//!
//! y is up. A body is a `Position`, a `Rotation`, a `Collider` and a
//! `Body`; a moving one has a `Velocity` and an `AngularVelocity` too, and
//! a still one a `Static` instead (`fixed` and `dynamic` spawn each).

mod math;

pub use math::{Mat3, Quat, Vec3};

use engine_api::{Bounds, Entity, OrderKey, SpatialKey, component, pair_key};

pub const SPHERE: u8 = 0;
pub const BOX: u8 = 1;

/// The most points a contact has.
pub const MAX_POINTS: usize = 4;

component! {
    /// The world's gravity: one entity with it, as in 2D. A world without
    /// one has none.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Gravity: "physics3d::Gravity" { pub x: f32, pub y: f32, pub z: f32 }
}

impl Gravity {
    /// y is up.
    pub const EARTH: Gravity = Gravity { x: 0.0, y: -9.81, z: 0.0 };

    pub fn vec(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

component! {
    /// A body's center.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Position: "physics3d::Position", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

/// A position's box is its collider's, turned by its rotation: both are
/// extents, so turning a body re-bounds its row in storage as moving it
/// does, and no copy of the turned box is kept (spatial-storage.md,
/// "Bounds from several components"). Without a collider, a point.
impl SpatialKey<3> for Position {
    type Extent = (Collider, Rotation);
    // Inline, as `SpatialKey` says: the glue calls it for every row it
    // re-bounds.
    #[inline]
    fn bounds(&self, (c, q): (Option<&Collider>, Option<&Rotation>)) -> Bounds<3> {
        let h = match (c, q) {
            (Some(c), Some(q)) if c.shape == BOX => c.turned_half(q.quat()),
            (Some(c), _) => Vec3::new(c.hx, c.hy, c.hz),
            (None, _) => Vec3::ZERO,
        };
        Bounds::around([self.x, self.y, self.z], [h.x, h.y, h.z])
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
        pub points: [f32; 4 * MAX_POINTS],
        pub ids: [u32; MAX_POINTS],
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
    pub struct Impulse: "physics3d::Impulse" { pub normal: [f32; MAX_POINTS], pub tx: f32, pub ty: f32, pub tz: f32, pub twist: f32 }
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

    /// Half the box around it turned to `q`, along each world axis: the
    /// box's axes' reach along each, |R| h (Box3D's `b3AABB_Transform`).
    /// A sphere's is its radius, whichever way it's turned.
    #[inline]
    pub fn turned_half(&self, q: Quat) -> Vec3 {
        if self.shape != BOX {
            return Vec3::splat(self.hx);
        }
        let m = q.matrix().cols;
        let along = |k: usize| self.hx * m[0].get(k).abs() + self.hy * m[1].get(k).abs() + self.hz * m[2].get(k).abs();
        Vec3::new(along(0), along(1), along(2))
    }

    /// The radius of a sphere around it, whichever way it's turned.
    pub fn reach(&self) -> f32 {
        if self.shape == BOX { (self.hx * self.hx + self.hy * self.hy + self.hz * self.hz).sqrt() } else { self.hx }
    }
}

/// What a static body is spawned with.
pub fn fixed(at: Vec3, rot: Quat, c: Collider) -> (Position, Rotation, Collider, Body, Static) {
    (Position { x: at.x, y: at.y, z: at.z }, Rotation::from(rot), c, Body::new(0.0), Static {})
}

/// What a moving body is spawned with, at rest.
pub fn dynamic(at: Vec3, rot: Quat, c: Collider, body: Body) -> (Position, Rotation, Collider, Body, Velocity, AngularVelocity) {
    (Position { x: at.x, y: at.y, z: at.z }, Rotation::from(rot), c, body, Velocity::default(), AngularVelocity::default())
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

/// How a rotation is stepped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Integrate {
    /// q + h/2 w q, normalized every substep (Box3D, Rapier).
    #[default]
    Linear,
    /// The same, normalized once, at the end of the step.
    LinearOnce,
    /// The exact turn about w by |w| h (Jolt).
    Exact,
}

/// How a point's separation follows its bodies' turns within a step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Anchors {
    /// Each anchor turned by its body's rotation so far (Box3D).
    #[default]
    Exact,
    /// To first order: a body turned by the small rotation θ moves an
    /// anchor r by θ x r, so the separation moves by θ . (r x n), a dot
    /// product with what the row already holds.
    Linear,
}

/// When a body's world inverse inertia is formed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Inertia {
    /// Once a step, from its rotation then (Box3D, Rapier).
    #[default]
    Step,
    /// Again every substep, from its rotation so far.
    Substep,
}

/// How two boxes are collided.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BoxBox {
    /// All 15 axes every step (Parry).
    Sat,
    /// The last step's axis first, kept if its separation changed by
    /// under the linear slop (Box3D).
    #[default]
    SatCached,
    /// GJK and EPA for the normal, then the faces clipped (Jolt).
    GjkEpa,
}

/// How more than four points are reduced to four.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Reduce {
    /// The deepest, the farthest from it, the largest triangle, the most
    /// area added (Box3D).
    #[default]
    Area,
    /// The deepest, the farthest from it, and the farthest either side of
    /// the line through them (Rapier's and Jolt's).
    Line,
}

/// Substeps a step, and relaxing passes a substep.
pub const SUBSTEPS: u32 = 5;
pub const RELAX_ITERATIONS: u32 = 2;
/// Of the substep rate, between two moving bodies; against a static one.
pub const STIFFNESS: f32 = 0.2;
pub const STATIC_STIFFNESS: f32 = 0.4;

component! {
    /// How the step is done, where more than one way was measured: one
    /// entity with it in the world, or none for the defaults, which are
    /// what was chosen (physics.md, "Rotation in 3D"). The bench and the
    /// tests switch a variant by writing it, as a game would; the step
    /// reads it every step, so a write takes effect at the next.
    ///
    /// The choices are the enums above as codes (`Warm::Nearest as u8`),
    /// since a component's fields are plain data; a code no variant has
    /// reads as the default, so a build that drops a variant runs the
    /// chosen way rather than failing on a world an older one wrote.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Tuning: "physics3d::Tuning" {
        pub substeps: u32,
        pub relax: u32,
        pub stiffness: f32,
        pub static_stiffness: f32,
        /// Friction in the pushing pass too (Box2D's), not only relaxing
        /// (Rapier's and Box3D's).
        pub friction_in_push: bool,
        pub integrate: u8,
        pub inertia: u8,
        pub anchors: u8,
        pub warm: u8,
        pub box_box: u8,
        pub reduce: u8,
    }
}

impl Default for Tuning {
    fn default() -> Tuning {
        Tuning {
            substeps: SUBSTEPS,
            relax: RELAX_ITERATIONS,
            stiffness: STIFFNESS,
            static_stiffness: STATIC_STIFFNESS,
            friction_in_push: false,
            integrate: Integrate::default() as u8,
            inertia: Inertia::default() as u8,
            anchors: Anchors::default() as u8,
            warm: Warm::default() as u8,
            box_box: BoxBox::default() as u8,
            reduce: Reduce::default() as u8,
        }
    }
}

/// The variant a code names, or the default (see `Tuning`).
fn code<T: Copy + Default>(all: &[T], k: u8) -> T {
    all.get(k as usize).copied().unwrap_or_default()
}

impl Tuning {
    pub fn integrate(&self) -> Integrate {
        code(&[Integrate::Linear, Integrate::LinearOnce, Integrate::Exact], self.integrate)
    }

    pub fn inertia(&self) -> Inertia {
        code(&[Inertia::Step, Inertia::Substep], self.inertia)
    }

    pub fn anchors(&self) -> Anchors {
        code(&[Anchors::Exact, Anchors::Linear], self.anchors)
    }

    pub fn warm(&self) -> Warm {
        code(&[Warm::Ids, Warm::Nearest, Warm::Cold], self.warm)
    }

    pub fn box_box(&self) -> BoxBox {
        code(&[BoxBox::Sat, BoxBox::SatCached, BoxBox::GjkEpa], self.box_box)
    }

    pub fn reduce(&self) -> Reduce {
        code(&[Reduce::Area, Reduce::Line], self.reduce)
    }

    /// From a list like "sub=4,relax=1,stiff=0.125,warm=cold": how the
    /// bench names a variant. Unknown keys are refused.
    pub fn parse(s: &str) -> Result<Tuning, String> {
        let mut t = Tuning::default();
        for kv in s.split(',').filter(|kv| !kv.is_empty()) {
            let (k, v) = kv.split_once('=').ok_or(format!("{kv}: not key=value"))?;
            let num = || v.parse::<f32>().map_err(|e| format!("{kv}: {e}"));
            match (k, v) {
                ("sub", _) => t.substeps = num()? as u32,
                ("relax", _) => t.relax = num()? as u32,
                ("stiff", _) => t.stiffness = num()?,
                ("static", _) => t.static_stiffness = num()?,
                ("fpush", _) => t.friction_in_push = num()? != 0.0,
                ("int", "linear") => t.integrate = Integrate::Linear as u8,
                ("int", "once") => t.integrate = Integrate::LinearOnce as u8,
                ("int", "exact") => t.integrate = Integrate::Exact as u8,
                ("inertia", "step") => t.inertia = Inertia::Step as u8,
                ("inertia", "substep") => t.inertia = Inertia::Substep as u8,
                ("anchors", "exact") => t.anchors = Anchors::Exact as u8,
                ("anchors", "linear") => t.anchors = Anchors::Linear as u8,
                ("warm", "ids") => t.warm = Warm::Ids as u8,
                ("warm", "nearest") => t.warm = Warm::Nearest as u8,
                ("warm", "cold") => t.warm = Warm::Cold as u8,
                ("bb", "sat") => t.box_box = BoxBox::Sat as u8,
                ("bb", "cached") => t.box_box = BoxBox::SatCached as u8,
                ("bb", "gjk") => t.box_box = BoxBox::GjkEpa as u8,
                ("reduce", "area") => t.reduce = Reduce::Area as u8,
                ("reduce", "line") => t.reduce = Reduce::Line as u8,
                _ => return Err(format!("{kv}: unknown")),
            }
        }
        Ok(t)
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
    fn a_turned_boxs_bounds_are_the_box_around_it() {
        let c = Collider::cuboid(Vec3::new(1.0, 0.25, 0.5));
        let q = Quat::axis_angle(Vec3::Z, std::f32::consts::FRAC_PI_2);
        let h = c.turned_half(q);
        assert!((h - Vec3::new(0.25, 1.0, 0.5)).len() < 1e-5, "{h:?}");
        let at = Position { x: 1.0, y: 2.0, z: 3.0 };
        let b = at.bounds((Some(&c), Some(&Rotation::from(q))));
        assert!((b.max[1] - 3.0).abs() < 1e-5 && (b.max[0] - 1.25).abs() < 1e-5, "{b:?}");
        let b = at.bounds((Some(&c), None));
        assert_eq!((b.max[0], b.max[1], b.max[2]), (2.0, 2.25, 3.5), "without a rotation, the box as it is");
        let s = Collider::sphere(0.5).turned_half(q);
        assert_eq!(s, Vec3::splat(0.5), "a sphere is its radius, turned or not");
    }

    #[test]
    fn a_tuning_names_its_variants_and_reads_unknown_codes_as_the_default() {
        let t = Tuning::parse("sub=4,warm=cold,bb=gjk,int=exact,anchors=linear").unwrap();
        assert_eq!(
            (t.substeps, t.warm(), t.box_box(), t.integrate(), t.anchors()),
            (4, Warm::Cold, BoxBox::GjkEpa, Integrate::Exact, Anchors::Linear)
        );
        assert_eq!(Tuning::parse("").unwrap(), Tuning::default());
        assert!(Tuning::parse("warm=hot").is_err() && Tuning::parse("sub").is_err());
        let odd = Tuning { warm: 9, box_box: 9, ..Tuning::default() };
        assert_eq!((odd.warm(), odd.box_box()), (Warm::Ids, BoxBox::SatCached));
    }
}
