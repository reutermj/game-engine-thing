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

/// A position's box is its collider's, or a point without one.
impl SpatialKey for Position {
    type Extent = Collider;
    fn bounds(&self, collider: Option<&Collider>) -> Bounds {
        let half = collider.map_or([0.0, 0.0], |c| if c.shape == BOX { [c.hx, c.hy] } else { [c.hx, c.hx] });
        Bounds::around([self.x, self.y], half)
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
        pub depth: f32,
        /// Pressing on each other at the end of the step: the solver pushed,
        /// or they overlap. A speculative contact is held before it touches,
        /// so being held doesn't mean touching.
        pub pressed: bool,
        /// Pressed the step before.
        pub was_pressed: bool,
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
    /// A contact's impulses from its last solve, which the next starts from.
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
    /// Which sides of a body touched something solid on the last step. Kept
    /// up to date on the bodies that have it.
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
