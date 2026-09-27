//! What `physics` owns and other mods use: bodies, colliders, the events a
//! step sends, and spatial queries. See docs/architecture/physics.md.
//!
//! Axes follow both games' screens: y grows downward, so `Touching::below`
//! is the +y side and gravity is usually positive y.

use engine_api::{Bounds, Entity, OrderKey, SpatialKey, component, event, pair_key};

mod shapes;
mod spatial;

pub use shapes::*;
pub use spatial::*;

/// `Body::kind`: moved by velocity, gravity and contacts.
pub const DYNAMIC: u8 = 0;
/// Moved by velocity only: it pushes dynamic bodies and is never pushed.
pub const KINEMATIC: u8 = 1;
/// Never moves: tiles, walls.
pub const STATIC: u8 = 2;

/// `Collider::shape`.
pub const BOX: u8 = 0;
pub const CIRCLE: u8 = 1;

component! {
    /// Where a body is: its collider's center. Tables of positions are kept
    /// in spatial order (docs/architecture/spatial-storage.md), so region
    /// queries and the broadphase walk only nearby pages.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Position: "physics::Position", order = spatial {
        pub x: f32,
        pub y: f32,
    }
}

/// A position's box is its collider's, turned by its rotation if it has one,
/// or a point without a collider. The rotation is an extent, not part of
/// the key, so a body that doesn't turn carries none and is bounded as it
/// was before rotation (physics.md, "Rotation", and spatial-storage.md,
/// "Bounds from several components").
impl SpatialKey for Position {
    type Extent = (Collider, Rotation);
    // Inline, as `SpatialKey` says: the glue calls it for every row it
    // re-bounds.
    #[inline]
    fn bounds(&self, (collider, rotation): (Option<&Collider>, Option<&Rotation>)) -> Bounds {
        let half = match (collider, rotation) {
            (Some(c), Some(q)) if c.shape == BOX => {
                let h = turned_half(Vec2::new(c.hx, c.hy), q.rot());
                [h.x, h.y]
            }
            _ => collider.map_or([0.0, 0.0], |c| if c.shape == BOX { [c.hx, c.hy] } else { [c.hx, c.hx] }),
        };
        Bounds::around([self.x, self.y], half)
    }
}

component! {
    /// Which way a body faces, as the cosine and sine of its angle (`Rot`):
    /// its collider turned about its position. A collider without one is
    /// axis-aligned. Part of the body's box in spatial storage, so writing
    /// it re-bounds the row, as moving it does.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Rotation: "physics::Rotation" {
        pub c: f32,
        pub s: f32,
    }
}

impl Default for Rotation {
    fn default() -> Rotation {
        Rotation { c: 1.0, s: 0.0 }
    }
}

impl Rotation {
    pub fn from_angle(a: f32) -> Rotation {
        Rotation::of(Rot::from_angle(a))
    }

    pub fn of(q: Rot) -> Rotation {
        Rotation { c: q.c, s: q.s }
    }

    pub fn rot(&self) -> Rot {
        Rot { c: self.c, s: self.s }
    }

    /// Radians, positive from +x toward +y (clockwise on a y-down screen).
    pub fn angle(&self) -> f32 {
        self.rot().angle()
    }
}

component! {
    /// How fast a body turns, in radians a second, positive from +x toward
    /// +y. A dynamic body with a `Rotation` and a `Spin` turns: contacts
    /// push it round, with the inertia of its shape and mass (a box's
    /// `m (w² + h²) / 12`, a disc's `m r² / 2`). Without a `Spin` a body
    /// keeps the rotation it has: that is the rotation lock, for a
    /// platformer's player, its walkers, and anything a game wants upright.
    /// A kinematic body with one turns at it; nothing turns it.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Spin: "physics::Spin" {
        pub w: f32,
    }
}

component! {
    /// Units per second. The game sets it at will (a jump, a serve); the
    /// solver changes it on contact. A static body needs none.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Velocity: "physics::Velocity" {
        pub x: f32,
        pub y: f32,
    }
}

component! {
    /// How a body moves. A shape without a `Body` is static.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Body: "physics::Body" {
        pub kind: u8,
        /// 0 is infinite: a dynamic body nothing pushes.
        pub inv_mass: f32,
        /// 0 is no bounce, 1 a perfect one. A contact uses the larger.
        pub restitution: f32,
        /// A contact uses the smaller of its bodies' frictions.
        pub friction: f32,
        pub gravity_scale: f32,
    }
}

impl Default for Body {
    fn default() -> Body {
        Body { kind: DYNAMIC, inv_mass: 1.0, restitution: 0.0, friction: 0.5, gravity_scale: 1.0 }
    }
}

impl Body {
    pub fn fixed() -> Body {
        Body { kind: STATIC, inv_mass: 0.0, ..Body::default() }
    }

    pub fn kinematic() -> Body {
        Body { kind: KINEMATIC, inv_mass: 0.0, gravity_scale: 0.0, ..Body::default() }
    }
}

component! {
    /// A body's shape, centered on its `Position`: a box (half extents
    /// `hx`, `hy`) or a circle (radius `hx`).
    #[derive(Debug, PartialEq, Copy)]
    pub struct Collider: "physics::Collider" {
        pub shape: u8,
        pub hx: f32,
        pub hy: f32,
        /// What this collider is, as bits, and what it collides with: a
        /// pair collides when each one's mask has the other's layer.
        pub layer: u32,
        pub mask: u32,
        /// Reports overlaps as `Trigger`s instead of pushing.
        pub sensor: bool,
        /// Layers this collider notices overlapping it, whether or not it
        /// collides with them: each such overlap is an `Overlap`.
        pub senses: u32,
    }
}

impl Default for Collider {
    /// A unit box on layer 1 that collides with everything. Zero masks
    /// would collide with nothing, a silent way to lose a body.
    fn default() -> Collider {
        Collider { shape: BOX, hx: 0.5, hy: 0.5, layer: 1, mask: u32::MAX, sensor: false, senses: 0 }
    }
}

impl Collider {
    pub fn rect(hx: f32, hy: f32) -> Collider {
        Collider { shape: BOX, hx, hy, ..Collider::default() }
    }

    pub fn circle(r: f32) -> Collider {
        Collider { shape: CIRCLE, hx: r, hy: r, ..Collider::default() }
    }

    pub fn sensor(self) -> Collider {
        Collider { sensor: true, ..self }
    }

    pub fn on(self, layer: u32, mask: u32) -> Collider {
        Collider { layer, mask, ..self }
    }

    pub fn sensing(self, layers: u32) -> Collider {
        Collider { senses: layers, ..self }
    }

    /// The inverse inertia of a body of this shape per unit of inverse
    /// mass: a box's inertia is `m (w² + h²) / 12`, a disc's `m r² / 2`.
    pub fn inertia_per_mass(&self) -> f32 {
        if self.shape == BOX { 3.0 / (self.hx * self.hx + self.hy * self.hy) } else { 2.0 / (self.hx * self.hx) }
    }

    /// How far its edge reaches from its center: how fast the edge of a
    /// body turning at `w` moves is `w` times this.
    pub fn reach(&self) -> f32 {
        if self.shape == BOX { self.hx.hypot(self.hy) } else { self.hx }
    }
}

component! {
    /// Two solid colliders touching, or about to (a gap the step may close):
    /// an entity physics keeps from the step it finds them until the step
    /// it doesn't, with a `Manifold`, a `Response` and an `Impulse`. `a <
    /// b`, and contacts are stored in that order, so "every contact of `a`"
    /// is a range (`pairs_from(a)`).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ContactPair: "physics::ContactPair", order = key {
        pub a: Entity,
        pub b: Entity,
    }
}

impl OrderKey for ContactPair {
    // Inline, as `OrderKey` says.
    #[inline]
    fn key(&self) -> u128 {
        pair_key(self.a, self.b)
    }
}

impl ContactPair {
    /// The other end, and the sign that turns the contact's normal into
    /// one pointing away from `me`, if `me` is an end.
    pub fn seen_from(&self, me: Entity) -> Option<(Entity, f32)> {
        if me == self.a {
            Some((self.b, 1.0))
        } else if me == self.b {
            Some((self.a, -1.0))
        } else {
            None
        }
    }
}

component! {
    /// A contact's geometry this step: the unit normal from `a` to `b`, and
    /// how deep they overlap (negative: the gap between them).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Manifold: "physics::Manifold" {
        pub nx: f32,
        pub ny: f32,
        /// With points, the deepest point's.
        pub depth: f32,
        /// Pressing on each other at the end of the step: the solver pushed,
        /// or they overlap. A speculative contact is held before it touches,
        /// so being held doesn't mean touching.
        pub pressed: bool,
        /// Pressed the step before.
        pub was_pressed: bool,
        /// How many of its `ContactPoints` are this step's: 0 unless either
        /// end is turned.
        pub points: u8,
        /// How many points its last solve solved at, whose impulses are in
        /// its `ContactPoints`: 0 if it was solved at its normal, neither end
        /// turning. Here rather than there, so a contact without points
        /// never reads its `ContactPoints`.
        pub solved: u8,
    }
}

component! {
    /// A contact's points, when either end is turned: up to two, each with
    /// its arms from both bodies' centers, its separation and its feature
    /// id; and the impulses its last solve left at each, by feature, which
    /// the next starts from. Two bodies that don't turn need no points (an
    /// impulse through any point moves them the same), and most contacts
    /// have none: so points are a component of their own, on every contact
    /// (contacts stay one table) but written only where they're used, rather
    /// than inline in `Manifold`, which cost a world where nothing turns 7%
    /// of its step (physics.md, "Contact points").
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ContactPoints: "physics::ContactPoints" {
        /// Each point's arm from `a`'s center and from `b`'s, on the world's
        /// axes: `[ax, ay, bx, by]` for the first point, then the second.
        pub anchors: [f32; 8],
        /// Each point's separation along the normal (negative: overlap).
        pub separations: [f32; 2],
        /// Each point's feature id: the edges or corners of the two shapes it
        /// came from, as Box2D numbers them, which is what warm starting
        /// matches from one step to the next.
        pub ids: [u16; 2],
        /// The last solve's impulses at its points (`Manifold::solved` of
        /// them), along the normal and the tangent, by their features.
        pub normals: [f32; 2],
        pub tangents: [f32; 2],
        pub solved_ids: [u16; 2],
    }
}

impl ContactPoints {
    /// Point `i`'s arms from `a` and from `b`.
    pub fn anchors(&self, i: usize) -> (Vec2, Vec2) {
        let a = &self.anchors[4 * i..4 * i + 4];
        (Vec2::new(a[0], a[1]), Vec2::new(a[2], a[3]))
    }

    /// The last solve's impulses at the point with feature `id`, if it
    /// solved one (of `solved`): Box2D's matching (`b2UpdateContact`). A
    /// point new this step starts from nothing.
    pub fn last(&self, solved: u8, id: u16) -> (f32, f32) {
        let at = self.solved_ids[..solved as usize].iter().position(|&s| s == id);
        at.map_or((0.0, 0.0), |k| (self.normals[k], self.tangents[k]))
    }
}

component! {
    /// How the solver treats a contact this step: `find_contacts` sets it
    /// from the bodies (the smaller friction, the larger restitution), and a
    /// system between it and `solve` may change it.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Response: "physics::Response" {
        pub friction: f32,
        pub restitution: f32,
        /// Not solved this step: the bodies pass through each other.
        pub disabled: bool,
    }
}

component! {
    /// A contact's impulses from its last solve, which the next starts from
    /// (with points, their sums; each point's is in `ContactPoints`).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Impulse: "physics::Impulse" {
        pub normal: f32,
        pub tangent: f32,
    }
}

component! {
    /// Two colliders overlapping where one is a sensor, or senses the
    /// other's layer: an entity while it lasts, `a < b`, stored in that
    /// order like contacts.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Overlap: "physics::Overlap", order = key {
        pub a: Entity,
        pub b: Entity,
    }
}

impl OrderKey for Overlap {
    #[inline]
    fn key(&self) -> u128 {
        pair_key(self.a, self.b)
    }
}

impl Overlap {
    /// The other end, if `me` is one.
    pub fn other(&self, me: Entity) -> Option<Entity> {
        if me == self.a {
            Some(self.b)
        } else if me == self.b {
            Some(self.a)
        } else {
            None
        }
    }
}

component! {
    /// The world's gravity, on one entity. No entity, no gravity.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Gravity: "physics::Gravity" {
        pub x: f32,
        pub y: f32,
    }
}

component! {
    /// Sleeping's thresholds, on one entity; no entity, `Sleep::DEFAULT`.
    /// An island (dynamic bodies joined by contacts) all of whose bodies
    /// have been slower than `speed` for `time` seconds falls asleep: its
    /// bodies stop, and aren't simulated until something moves against one,
    /// what one rests on goes, a game writes to one, or `physics` is sent
    /// `wake`. A `speed` of 0 turns it off (`Sleep::OFF`): nothing is slower
    /// than that. See docs/architecture/physics.md, "Sleeping".
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Sleep: "physics::Sleep" {
        pub speed: f32,
        pub time: f32,
    }
}

impl Sleep {
    /// What physics sleeps by with no `Sleep` entity: slower than 0.05 a
    /// second (a cell, in both games' units) for half a second.
    pub const DEFAULT: Sleep = Sleep { speed: 0.05, time: 0.5 };
    pub const OFF: Sleep = Sleep { speed: 0.0, time: 0.0 };
}

component! {
    /// How the step solves, on one entity; no entity, `Tuning::DEFAULT`.
    /// The step reads it every step, so a write takes effect at the next,
    /// as with 3D's `physics3d::Tuning`. `substeps` is how many the soft
    /// step splits a step into: a contact is a spring as stiff as a share
    /// of the substep rate, so more substeps stand tall stacks sooner (a
    /// 20-high stack of turning boxes rests from step 60 at six, 580 at
    /// five) and cost the solver about a fifth more each; 0 reads as the
    /// default. See docs/architecture/physics.md, "Still at rest".
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tuning: "physics::Tuning" {
        pub substeps: u32,
    }
}

impl Tuning {
    /// Five where Box2D has four: at five a pile of 10 000 sinks 0.013 deep
    /// where at four it sinks 0.024, for 11% more time a step (physics.md,
    /// "Settling").
    pub const DEFAULT: Tuning = Tuning { substeps: 5 };

    /// The substeps it names, the default's for 0.
    pub fn substeps(&self) -> usize {
        if self.substeps == 0 { Tuning::DEFAULT.substeps as usize } else { self.substeps as usize }
    }
}

component! {
    /// On a sleeping body, put there by physics as its island falls asleep
    /// and taken off as it wakes: so sleeping bodies are tables of their
    /// own, which the step's walks skip by what they match rather than
    /// looking each body up. `island` is the bodies it wakes with. A game
    /// wakes a body by removing it, or by writing its velocity, position,
    /// collider or body, and puts one to sleep by giving it one, in an
    /// island it numbers (physics numbers its own after the greatest it has
    /// seen).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Asleep: "physics::Asleep" {
        pub island: u32,
    }
}

component! {
    /// Physics's record of a body it has taken as asleep (its own, or one a
    /// game gave `Asleep`), and the island it wakes with: put on beside
    /// `Asleep`, and taken off as physics wakes it. What `Asleep` alone
    /// can't say, since a game writes that: which sleeping bodies are new
    /// to physics (with `Asleep` and without this), and which it had asleep
    /// that a game woke or unmade (this without `Asleep`, or without a
    /// body). Only physics gives it, so its count falling says a game
    /// despawned a sleeping body. Tables of its own, so both are found by
    /// what a query matches (docs/architecture/physics.md, "Sleeping").
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Slept: "physics::Slept" {
        pub island: u32,
    }
}

component! {
    /// On an awake dynamic body slower than `Sleep::speed`, since physics's
    /// step `since`: how long it has been still, which is what its island
    /// falls asleep by. Physics puts it on as a body goes slower and takes
    /// it off as it goes faster or falls asleep. Sparse, since the bodies
    /// crossing the threshold, hundreds a step in a settling pile, would
    /// each be a move between tables (docs/architecture/physics.md,
    /// "Sleeping").
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Still: "physics::Still", storage = sparse {
        pub since: u64,
    }
}

component! {
    /// On a contact neither end of which moves, one of them asleep: kept as
    /// it is, impulses and all (the warm start for when they wake), and not
    /// looked for, merged or solved until an end wakes.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Resting: "physics::Resting" {}
}

component! {
    /// Which sides of a body touched something solid on the last step. Kept
    /// up to date on the bodies that have it; a sleeping body's is as it
    /// was when it fell asleep.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Touching: "physics::Touching" {
        /// -x
        pub left: bool,
        /// +x
        pub right: bool,
        /// -y
        pub above: bool,
        /// +y: standing on something, with gravity down the screen.
        pub below: bool,
    }
}

event! {
    /// Two solid colliders began touching this step. `nx, ny` is the unit
    /// normal from `a` to `b`; `speed` how fast they closed along it.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Contact: "physics::Contact" {
        pub a: Entity,
        pub b: Entity,
        pub nx: f32,
        pub ny: f32,
        pub speed: f32,
    }
}

event! {
    /// A sensor began overlapping a collider it collides with.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Trigger: "physics::Trigger" {
        pub sensor: Entity,
        pub other: Entity,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Entity = Entity { index: 1, generation: 0 };
    const B: Entity = Entity { index: 2, generation: 0 };
    const C: Entity = Entity { index: 3, generation: 0 };

    #[test]
    fn a_contact_seen_from_either_end_points_away_from_it() {
        let pair = ContactPair { a: A, b: B };
        assert_eq!(pair.seen_from(A), Some((B, 1.0)));
        assert_eq!(pair.seen_from(B), Some((A, -1.0)));
        assert_eq!(pair.seen_from(C), None);
    }

    #[test]
    fn an_overlap_seen_from_either_end_is_the_other() {
        let o = Overlap { a: A, b: B };
        assert_eq!((o.other(A), o.other(B), o.other(C)), (Some(B), Some(A), None));
    }

    #[test]
    fn contacts_are_stored_in_pair_order() {
        let key = |a, b| ContactPair { a, b }.key();
        assert!(key(A, C) < key(B, A), "by the first end first");
        assert!(key(A, B) < key(A, C), "then by the second");
        assert!(key(A, Entity { index: 1, generation: 1 }) < key(A, B), "as `Entity` orders");
    }
}
