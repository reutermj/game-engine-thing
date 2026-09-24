//! 2D vectors and the shape tests queries need: overlap, containment and
//! ray casts, for boxes and circles without rotation. In the interface
//! because `Spatial` runs in its callers.

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

/// A shape placed somewhere: what `Spatial::overlapping` takes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placed {
    pub shape: Shape,
    pub at: Vec2,
}

impl Placed {
    pub fn aabb(&self) -> Aabb {
        let h = self.shape.half_extents();
        Aabb { min: self.at - h, max: self.at + h }
    }
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
    Placed { shape: Shape::Box(half), at: center }
}

/// A probe for `Spatial::overlapping`: a circle.
pub fn circle(center: Vec2, radius: f32) -> Placed {
    Placed { shape: Shape::Circle(radius), at: center }
}

/// Whether two placed shapes overlap. Touching counts, so a probe exactly
/// on a tile's edge finds it: what a ledge check wants.
pub fn overlaps(a: &Placed, b: &Placed) -> bool {
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
    let hit = match shape.shape {
        Shape::Box(h) => ray_box(ray, shape.at - h, shape.at + h),
        Shape::Circle(r) => ray_circle(ray, shape.at, r),
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
    fn touching_counts_as_overlapping() {
        let tile = rect(Vec2::new(0.5, 0.5), Vec2::new(0.5, 0.5));
        assert!(overlaps(&rect(Vec2::new(1.5, 0.5), Vec2::new(0.5, 0.5)), &tile));
        assert!(overlaps(&circle(Vec2::new(0.5, 2.0), 1.0), &tile));
        assert!(!overlaps(&circle(Vec2::new(2.0, 2.0), 1.0), &tile), "the corner is farther than the radius");
    }
}
