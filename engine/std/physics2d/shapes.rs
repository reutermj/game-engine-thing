//! 2D vectors, rotations, and the shape tests queries need: overlap,
//! containment and ray casts, for boxes and circles, turned or not. In the
//! interface because `Spatial` runs in its callers.

use std::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

use crate::{BOX, Collider};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub const fn new(x: f32, y: f32) -> Vec2 {
        Vec2 { x, y }
    }

    pub fn dot(self, o: Vec2) -> f32 {
        self.x * o.x + self.y * o.y
    }

    pub fn len(self) -> f32 {
        self.dot(self).sqrt()
    }

    /// Rotated a quarter turn: the tangent to a contact normal.
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }

    pub fn abs(self) -> Vec2 {
        Vec2::new(self.x.abs(), self.y.abs())
    }

    pub fn clamp(self, min: Vec2, max: Vec2) -> Vec2 {
        Vec2::new(self.x.clamp(min.x, max.x), self.y.clamp(min.y, max.y))
    }

    /// The 2D cross product, `self.x * o.y - self.y * o.x`: the torque of a
    /// force `o` at arm `self`.
    #[inline]
    pub fn cross(self, o: Vec2) -> f32 {
        self.x * o.y - self.y * o.x
    }

    /// `w` cross `self`, the velocity of a point at arm `self` on a body
    /// turning at `w` (Box2D's `b2CrossSV`).
    #[inline]
    pub fn turned_by(self, w: f32) -> Vec2 {
        Vec2::new(-w * self.y, w * self.x)
    }
}

/// A rotation as the cosine and sine of its angle, as Box2D's `b2Rot` and
/// Rapier's unit complex number keep it: turning a vector is four
/// multiplies, and composing two a few more, with no trigonometry. See
/// physics.md, "Rotation". Positive angles turn +x toward +y.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rot {
    pub c: f32,
    pub s: f32,
}

impl Default for Rot {
    fn default() -> Rot {
        Rot::IDENTITY
    }
}

impl Rot {
    pub const IDENTITY: Rot = Rot { c: 1.0, s: 0.0 };

    pub fn from_angle(a: f32) -> Rot {
        let (s, c) = a.sin_cos();
        Rot { c, s }
    }

    pub fn angle(self) -> f32 {
        self.s.atan2(self.c)
    }

    #[inline]
    pub fn rotate(self, v: Vec2) -> Vec2 {
        Vec2::new(self.c * v.x - self.s * v.y, self.s * v.x + self.c * v.y)
    }

    /// The inverse rotation of `v`: into this rotation's frame.
    #[inline]
    pub fn unrotate(self, v: Vec2) -> Vec2 {
        Vec2::new(self.c * v.x + self.s * v.y, -self.s * v.x + self.c * v.y)
    }

    /// This rotation after `first`.
    #[inline]
    pub fn after(self, first: Rot) -> Rot {
        Rot { c: self.c * first.c - self.s * first.s, s: self.s * first.c + self.c * first.s }
    }

    /// Scaled back to unit length, which rounding drifts from.
    #[inline]
    pub fn normalized(self) -> Rot {
        let len = (self.c * self.c + self.s * self.s).sqrt();
        let k = if len > 0.0 { 1.0 / len } else { 0.0 };
        Rot { c: self.c * k, s: self.s * k }
    }

    /// Turned a further `da` radians, by the first-order step of the
    /// rotation's derivative, then normalized: Box2D's
    /// `b2IntegrateRotation`. Exact to second order in `da`, and the
    /// normalizing keeps it a rotation however many steps it takes.
    #[inline]
    pub fn integrate(self, da: f32) -> Rot {
        Rot { c: self.c - da * self.s, s: self.s + da * self.c }.normalized()
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Vec2;
    fn mul(self, k: f32) -> Vec2 {
        Vec2::new(self.x * k, self.y * k)
    }
}

impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, o: Vec2) {
        *self = *self + o;
    }
}

impl SubAssign for Vec2 {
    fn sub_assign(&mut self, o: Vec2) {
        *self = *self - o;
    }
}

/// A shape at a position: a collider in the world, or what a query probes
/// with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// Half extents.
    Box(Vec2),
    Circle(f32),
}

impl Shape {
    pub fn of(c: &Collider) -> Shape {
        if c.shape == BOX { Shape::Box(Vec2::new(c.hx, c.hy)) } else { Shape::Circle(c.hx) }
    }

    pub fn half_extents(self) -> Vec2 {
        match self {
            Shape::Box(h) => h,
            Shape::Circle(r) => Vec2::new(r, r),
        }
    }
}

/// A shape placed somewhere: what `Spatial::overlapping` takes. `rot` is
/// `None` for a shape that isn't turned, whose tests are the axis-aligned
/// ones, bit for bit what they were before rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placed {
    pub shape: Shape,
    pub at: Vec2,
    pub rot: Option<Rot>,
}

impl Placed {
    pub fn aabb(&self) -> Aabb {
        let h = self.half_extents();
        Aabb { min: self.at - h, max: self.at + h }
    }

    /// Half the box around the shape as it's turned.
    pub fn half_extents(&self) -> Vec2 {
        match (self.shape, self.rot) {
            (Shape::Box(h), Some(q)) => turned_half(h, q),
            (shape, _) => shape.half_extents(),
        }
    }

    /// The same shape turned by `q`.
    pub fn turned(self, q: Rot) -> Placed {
        Placed { rot: Some(q), ..self }
    }
}

/// Half the box around a box of half extents `h` turned by `q`.
#[inline]
pub fn turned_half(h: Vec2, q: Rot) -> Vec2 {
    let (c, s) = (q.c.abs(), q.s.abs());
    Vec2::new(c * h.x + s * h.y, s * h.x + c * h.y)
}

/// An axis-aligned box, by its corners.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x && o.min.x <= self.max.x && self.min.y <= o.max.y && o.min.y <= self.max.y
    }
}

/// A probe for `Spatial::overlapping`: a box by its center and half extents.
pub fn rect(center: Vec2, half: Vec2) -> Placed {
    Placed { shape: Shape::Box(half), at: center, rot: None }
}

/// A probe for `Spatial::overlapping`: a circle.
pub fn circle(center: Vec2, radius: f32) -> Placed {
    Placed { shape: Shape::Circle(radius), at: center, rot: None }
}

/// Whether two placed shapes overlap. Touching counts, so a probe exactly
/// on a tile's edge finds it: what a ledge check wants.
pub fn overlaps(a: &Placed, b: &Placed) -> bool {
    if a.rot.is_some() || b.rot.is_some() {
        return turned_overlap(a, b);
    }
    match (a.shape, b.shape) {
        (Shape::Box(_), Shape::Box(_)) => a.aabb().overlaps(&b.aabb()),
        (Shape::Circle(ra), Shape::Circle(rb)) => (b.at - a.at).len() <= ra + rb,
        (Shape::Box(h), Shape::Circle(r)) => box_circle(a.at, h, b.at, r),
        (Shape::Circle(r), Shape::Box(h)) => box_circle(b.at, h, a.at, r),
    }
}

fn box_circle(at: Vec2, half: Vec2, center: Vec2, r: f32) -> bool {
    let closest = center.clamp(at - half, at + half);
    (center - closest).len() <= r
}

/// `overlaps` where either shape is turned: a circle is tested in the box's
/// frame, and two boxes on the four axes of their faces (separating axes).
fn turned_overlap(a: &Placed, b: &Placed) -> bool {
    let q = |p: &Placed| p.rot.unwrap_or_default();
    match (a.shape, b.shape) {
        (Shape::Circle(ra), Shape::Circle(rb)) => (b.at - a.at).len() <= ra + rb,
        (Shape::Box(h), Shape::Circle(r)) => box_circle(Vec2::ZERO, h, q(a).unrotate(b.at - a.at), r),
        (Shape::Circle(r), Shape::Box(h)) => box_circle(Vec2::ZERO, h, q(b).unrotate(a.at - b.at), r),
        (Shape::Box(ha), Shape::Box(hb)) => {
            let (qa, qb) = (q(a), q(b));
            let d = b.at - a.at;
            let axes = [
                qa.rotate(Vec2::new(1.0, 0.0)),
                qa.rotate(Vec2::new(0.0, 1.0)),
                qb.rotate(Vec2::new(1.0, 0.0)),
                qb.rotate(Vec2::new(0.0, 1.0)),
            ];
            // A box's reach along `n`: its half extents on its own axes,
            // projected.
            let reach = |q: Rot, h: Vec2, n: Vec2| {
                let local = q.unrotate(n);
                local.x.abs() * h.x + local.y.abs() * h.y
            };
            axes.iter().all(|&n| d.dot(n).abs() <= reach(qa, ha, n) + reach(qb, hb, n))
        }
    }
}

/// How far apart two placed shapes are, negative where they overlap (by
/// how far one would have to move to clear the other): for measuring piles,
/// turned or not. Between boxes, their faces' separating axes, so it's
/// exact when they overlap and a lower bound when apart at a corner.
pub fn separation(a: &Placed, b: &Placed) -> f32 {
    let q = |p: &Placed| p.rot.unwrap_or_default();
    let d = b.at - a.at;
    match (a.shape, b.shape) {
        (Shape::Circle(ra), Shape::Circle(rb)) => d.len() - ra - rb,
        (Shape::Box(h), Shape::Circle(r)) => box_circle_separation(h, q(a).unrotate(d), r),
        (Shape::Circle(r), Shape::Box(h)) => box_circle_separation(h, q(b).unrotate(-d), r),
        (Shape::Box(ha), Shape::Box(hb)) => {
            let (qa, qb) = (q(a), q(b));
            let reach = |q: Rot, h: Vec2, n: Vec2| {
                let local = q.unrotate(n);
                local.x.abs() * h.x + local.y.abs() * h.y
            };
            [qa.rotate(Vec2::new(1.0, 0.0)), qa.rotate(Vec2::new(0.0, 1.0)), qb.rotate(Vec2::new(1.0, 0.0)), qb.rotate(Vec2::new(0.0, 1.0))]
                .iter()
                .map(|&n| d.dot(n).abs() - reach(qa, ha, n) - reach(qb, hb, n))
                .fold(f32::NEG_INFINITY, f32::max)
        }
    }
}

/// A circle at `c` (in the box's frame) against a box of half extents `h`.
fn box_circle_separation(h: Vec2, c: Vec2, r: f32) -> f32 {
    let d = c.abs();
    let outside = Vec2::new((d.x - h.x).max(0.0), (d.y - h.y).max(0.0));
    if outside == Vec2::ZERO { (d.x - h.x).max(d.y - h.y) - r } else { outside.len() - r }
}

/// A ray from `origin` along `dir` (normalized on construction), up to
/// `max` along it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    pub origin: Vec2,
    pub dir: Vec2,
    pub max: f32,
}

impl Ray {
    pub fn new(origin: Vec2, dir: Vec2, max: f32) -> Ray {
        let len = dir.len();
        assert!(len > 0.0, "a ray needs a direction");
        Ray { origin, dir: dir * (1.0 / len), max }
    }

    pub fn at(&self, t: f32) -> Vec2 {
        self.origin + self.dir * t
    }

    /// The box the ray sweeps: what the index is searched with.
    pub fn aabb(&self) -> Aabb {
        let end = self.at(self.max);
        Aabb {
            min: Vec2::new(self.origin.x.min(end.x), self.origin.y.min(end.y)),
            max: Vec2::new(self.origin.x.max(end.x), self.origin.y.max(end.y)),
        }
    }
}

/// Where a ray hit: its distance along the ray, and the surface normal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub t: f32,
    pub normal: Vec2,
}

/// The ray's first hit on `shape`, if within its length. A ray starting
/// inside a shape hits it at 0, with the normal opposing the ray.
pub fn raycast(ray: &Ray, shape: &Placed) -> Option<Hit> {
    let hit = match (shape.shape, shape.rot) {
        // In the box's frame, where it is axis-aligned, and the normal
        // turned back out.
        (Shape::Box(h), Some(q)) => {
            let local = Ray { origin: q.unrotate(ray.origin - shape.at), dir: q.unrotate(ray.dir), max: ray.max };
            ray_box(&local, -h, h).map(|hit| Hit { normal: q.rotate(hit.normal), ..hit })
        }
        (Shape::Box(h), None) => ray_box(ray, shape.at - h, shape.at + h),
        (Shape::Circle(r), _) => ray_circle(ray, shape.at, r),
    }?;
    (hit.t <= ray.max).then_some(hit)
}

fn ray_box(ray: &Ray, min: Vec2, max: Vec2) -> Option<Hit> {
    // Slabs: the ray is inside the box between the latest entry and the
    // earliest exit over both axes.
    let (mut enter, mut exit) = (f32::NEG_INFINITY, f32::INFINITY);
    let mut normal = -ray.dir;
    for (o, d, lo, hi, axis) in
        [(ray.origin.x, ray.dir.x, min.x, max.x, Vec2::new(1.0, 0.0)), (ray.origin.y, ray.dir.y, min.y, max.y, Vec2::new(0.0, 1.0))]
    {
        if d == 0.0 {
            if o < lo || o > hi {
                return None;
            }
            continue;
        }
        let (t0, t1) = ((lo - o) / d, (hi - o) / d);
        let (near, far, n) = if t0 < t1 { (t0, t1, -axis) } else { (t1, t0, axis) };
        if near > enter {
            enter = near;
            normal = n;
        }
        exit = exit.min(far);
    }
    if enter > exit || exit < 0.0 {
        return None;
    }
    if enter < 0.0 {
        return Some(Hit { t: 0.0, normal: -ray.dir });
    }
    Some(Hit { t: enter, normal })
}

fn ray_circle(ray: &Ray, center: Vec2, r: f32) -> Option<Hit> {
    let m = ray.origin - center;
    let c = m.dot(m) - r * r;
    if c <= 0.0 {
        return Some(Hit { t: 0.0, normal: -ray.dir });
    }
    let b = m.dot(ray.dir);
    let disc = b * b - c;
    if b > 0.0 || disc < 0.0 {
        return None;
    }
    let t = -b - disc.sqrt();
    let normal = (ray.at(t) - center) * (1.0 / r);
    Some(Hit { t, normal })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn a_ray_hits_a_box_face_with_its_normal() {
        let r = Ray::new(Vec2::new(0.0, 0.5), Vec2::new(1.0, 0.0), 10.0);
        let hit = raycast(&r, &rect(Vec2::new(3.0, 0.5), Vec2::new(1.0, 1.0))).unwrap();
        assert!(close(hit.t, 2.0), "{hit:?}");
        assert_eq!(hit.normal, Vec2::new(-1.0, 0.0));
    }

    #[test]
    fn a_ray_misses_what_is_beside_it_or_beyond_its_length() {
        let r = Ray::new(Vec2::ZERO, Vec2::new(1.0, 0.0), 1.5);
        assert!(raycast(&r, &rect(Vec2::new(3.0, 0.0), Vec2::new(1.0, 1.0))).is_none(), "beyond");
        assert!(raycast(&r, &rect(Vec2::new(1.0, 5.0), Vec2::new(1.0, 1.0))).is_none(), "beside");
        assert!(raycast(&r, &circle(Vec2::new(-3.0, 0.0), 1.0)).is_none(), "behind");
    }

    #[test]
    fn a_ray_hits_a_circle_on_its_surface() {
        let r = Ray::new(Vec2::new(0.0, 0.0), Vec2::new(0.0, 2.0), 10.0);
        let hit = raycast(&r, &circle(Vec2::new(0.0, 5.0), 1.0)).unwrap();
        assert!(close(hit.t, 4.0), "{hit:?}");
        assert!(close(hit.normal.y, -1.0));
    }

    #[test]
    fn a_ray_starting_inside_hits_at_once() {
        let r = Ray::new(Vec2::new(3.0, 0.0), Vec2::new(1.0, 0.0), 1.0);
        assert_eq!(raycast(&r, &rect(Vec2::new(3.0, 0.0), Vec2::new(1.0, 1.0))).unwrap().t, 0.0);
        assert_eq!(raycast(&r, &circle(Vec2::new(3.0, 0.0), 1.0)).unwrap().t, 0.0);
    }

    #[test]
    fn a_turned_box_overlaps_what_its_corners_reach_and_not_its_box() {
        // A unit square turned 45 degrees reaches 0.707 along the axes,
        // and its axis-aligned box's corner (0.7, 0.7) is outside it.
        let diamond = rect(Vec2::ZERO, Vec2::new(0.5, 0.5)).turned(Rot::from_angle(std::f32::consts::FRAC_PI_4));
        assert!(overlaps(&diamond, &circle(Vec2::new(0.8, 0.0), 0.1)), "its corner reaches along x");
        assert!(!overlaps(&diamond, &circle(Vec2::new(0.62, 0.62), 0.1)), "the box's corner is empty");
        assert!(overlaps(&diamond, &rect(Vec2::new(1.1, 0.0), Vec2::new(0.4, 0.1))));
        assert!(!overlaps(&diamond, &rect(Vec2::new(0.75, 0.75), Vec2::new(0.1, 0.1))), "diagonal, past the edge");
        let h = diamond.half_extents();
        assert!(close(h.x, 0.70710677) && close(h.y, 0.70710677), "{h:?}");
    }

    #[test]
    fn a_ray_hits_a_turned_box_on_its_turned_face() {
        let diamond = rect(Vec2::new(3.0, 0.0), Vec2::new(0.5, 0.5)).turned(Rot::from_angle(std::f32::consts::FRAC_PI_4));
        let hit = raycast(&Ray::new(Vec2::ZERO, Vec2::new(1.0, 0.0), 10.0), &diamond).unwrap();
        assert!(close(hit.t, 3.0 - 0.70710677), "{hit:?}");
        assert!(close(hit.normal.x, -0.70710677) && close(hit.normal.y.abs(), 0.70710677), "{hit:?}");
    }

    #[test]
    fn rotations_compose_and_integrate_to_a_unit_length() {
        let (a, b) = (Rot::from_angle(0.3), Rot::from_angle(0.5));
        assert!(close(b.after(a).angle(), 0.8));
        assert!(close(a.unrotate(a.rotate(Vec2::new(1.0, 2.0))).y, 2.0));
        let mut q = Rot::IDENTITY;
        for _ in 0..100 {
            q = q.integrate(0.01);
        }
        assert!(close(q.c * q.c + q.s * q.s, 1.0), "{q:?}");
        // Each step turns by atan(0.01), a hair under 0.01.
        assert!((q.angle() - 100.0 * 0.01f32.atan()).abs() < 1e-4, "{q:?}");
    }

    #[test]
    fn touching_counts_as_overlapping() {
        let tile = rect(Vec2::new(0.5, 0.5), Vec2::new(0.5, 0.5));
        assert!(overlaps(&rect(Vec2::new(1.5, 0.5), Vec2::new(0.5, 0.5)), &tile));
        assert!(overlaps(&circle(Vec2::new(0.5, 2.0), 1.0), &tile));
        assert!(!overlaps(&circle(Vec2::new(2.0, 2.0), 1.0), &tile), "the corner is farther than the radius");
    }
}
