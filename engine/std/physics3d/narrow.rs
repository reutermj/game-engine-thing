//! Contact between two shapes in 3D: the normal from `a` to `b` and how
//! deep they overlap. Translation only, as in 2D (physics.md): with no
//! rotation an impulse through any point moves a body the same, so a
//! contact needs no points, even a box resting face to face on another,
//! which a rotating engine gives four (see spatial-storage.md, "In 3D").
//!
//! Pairs within `MARGIN` of touching count, with a negative depth: the
//! speculative contacts of the 2D step.

use crate::{Shape, Vec3};

pub const MARGIN: f32 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Manifold {
    /// Unit, from `a` toward `b`.
    pub normal: Vec3,
    /// Positive when overlapping; negative up to `-MARGIN` when apart.
    pub depth: f32,
}

pub fn collide(pa: Vec3, a: Shape, pb: Vec3, b: Shape) -> Option<Manifold> {
    match (a, b) {
        (Shape::Box(ha), Shape::Box(hb)) => box_box(pa, ha, pb, hb),
        (Shape::Sphere(ra), Shape::Sphere(rb)) => sphere_sphere(pa, ra, pb, rb),
        (Shape::Box(h), Shape::Sphere(r)) => box_sphere(pa, h, pb, r),
        (Shape::Sphere(r), Shape::Box(h)) => box_sphere(pb, h, pa, r).map(|m| Manifold { normal: -m.normal, ..m }),
    }
}

/// +1 for zero, so coincident centers still get a normal.
fn sign(v: f32) -> f32 {
    if v < 0.0 { -1.0 } else { 1.0 }
}

/// Along the axis of least overlap. The 2D step's seam rule (a box running
/// along a row of tiles) isn't here: the scenes have no tile maps.
fn box_box(pa: Vec3, ha: Vec3, pb: Vec3, hb: Vec3) -> Option<Manifold> {
    let d = pb - pa;
    let o = [ha.x + hb.x - d.x.abs(), ha.y + hb.y - d.y.abs(), ha.z + hb.z - d.z.abs()];
    if o.iter().any(|&o| o < -MARGIN) {
        return None;
    }
    let axis = if o[0] < o[1] && o[0] < o[2] {
        0
    } else if o[1] <= o[2] {
        1
    } else {
        2
    };
    let mut normal = Vec3::ZERO;
    normal.set(axis, sign(d.get(axis)));
    Some(Manifold { normal, depth: o[axis] })
}

fn sphere_sphere(pa: Vec3, ra: f32, pb: Vec3, rb: f32) -> Option<Manifold> {
    let d = pb - pa;
    let dist = d.len();
    let depth = ra + rb - dist;
    if depth < -MARGIN {
        return None;
    }
    let normal = if dist > 0.0 { d * (1.0 / dist) } else { Vec3::new(0.0, 1.0, 0.0) };
    Some(Manifold { normal, depth })
}

/// Normal from the box toward the sphere.
fn box_sphere(pb: Vec3, h: Vec3, c: Vec3, r: f32) -> Option<Manifold> {
    let local = c - pb;
    let closest = local.clamp(-h, h);
    if closest == local {
        // The center is inside the box: out through the nearest face.
        let f = [h.x - local.x.abs(), h.y - local.y.abs(), h.z - local.z.abs()];
        let axis = if f[0] < f[1] && f[0] < f[2] {
            0
        } else if f[1] <= f[2] {
            1
        } else {
            2
        };
        let mut normal = Vec3::ZERO;
        normal.set(axis, sign(local.get(axis)));
        return Some(Manifold { normal, depth: f[axis] + r });
    }
    let d = local - closest;
    let dist = d.len();
    let depth = r - dist;
    if depth < -MARGIN {
        return None;
    }
    Some(Manifold { normal: d * (1.0 / dist), depth })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boxes_push_apart_along_the_shallowest_axis() {
        let m = collide(Vec3::ZERO, Shape::Box(Vec3::splat(0.5)), Vec3::new(0.2, 0.9, -0.1), Shape::Box(Vec3::splat(0.5))).unwrap();
        assert_eq!(m.normal, Vec3::new(0.0, 1.0, 0.0));
        assert!((m.depth - 0.1).abs() < 1e-5, "{m:?}");
        let m = collide(Vec3::ZERO, Shape::Box(Vec3::splat(0.5)), Vec3::new(0.2, 0.1, -0.95), Shape::Box(Vec3::splat(0.5))).unwrap();
        assert_eq!(m.normal, Vec3::new(0.0, 0.0, -1.0));
    }

    #[test]
    fn spheres_meet_along_their_centers() {
        let m = collide(Vec3::ZERO, Shape::Sphere(0.5), Vec3::new(0.0, 0.6, 0.8), Shape::Sphere(0.5)).unwrap();
        assert!((m.normal - Vec3::new(0.0, 0.6, 0.8)).len() < 1e-6);
        assert!(m.depth.abs() < 1e-6);
        assert!(collide(Vec3::ZERO, Shape::Sphere(0.5), Vec3::new(0.0, 1.2, 0.0), Shape::Sphere(0.5)).is_none());
    }

    #[test]
    fn a_sphere_against_a_box_face_edge_and_corner() {
        let b = Shape::Box(Vec3::splat(1.0));
        let face = collide(Vec3::ZERO, b, Vec3::new(0.0, 0.0, 1.4), Shape::Sphere(0.5)).unwrap();
        assert_eq!(face.normal, Vec3::new(0.0, 0.0, 1.0));
        assert!((face.depth - 0.1).abs() < 1e-5);
        let corner = collide(Vec3::ZERO, b, Vec3::splat(1.2), Shape::Sphere(0.5)).unwrap();
        assert!((corner.normal - Vec3::splat(1.0 / 3f32.sqrt())).len() < 1e-5);
        let flipped = collide(Vec3::new(0.0, 0.0, 1.4), Shape::Sphere(0.5), Vec3::ZERO, b).unwrap();
        assert_eq!(flipped.normal, Vec3::new(0.0, 0.0, -1.0));
    }
}
