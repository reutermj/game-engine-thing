//! Contact between two shapes: the normal from `a` to `b` and how deep they
//! overlap. Without rotation a contact needs no points, since an impulse
//! through any point moves a body the same.
//!
//! Pairs within `MARGIN` of touching count, with a negative depth: a
//! speculative contact, which lets the solver stop a body at the surface
//! instead of after it has sunk in, and keeps resting contacts from
//! flickering in and out between steps. Speculative contacts are as Box2D
//! has them, Erin Catto's (docs/CREDITS.md).

use physics::{Placed, Shape, Vec2};

pub const MARGIN: f32 = 0.05;
/// An overlap this thin is two faces resting flush, not one box inside
/// another: more than the solver's slop, which resting contacts sink to.
pub const FLUSH: f32 = 0.01;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Manifold {
    /// Unit, from `a` toward `b`.
    pub normal: Vec2,
    /// Positive when overlapping; negative up to `-MARGIN` when apart.
    pub depth: f32,
}

/// `rel` is `b`'s velocity relative to `a`'s: what settles which face two
/// boxes meet on when their geometry can't (see `box_box`).
pub fn collide(a: &Placed, b: &Placed, rel: Vec2) -> Option<Manifold> {
    match (a.shape, b.shape) {
        (Shape::Box(ha), Shape::Box(hb)) => box_box(a.at, ha, b.at, hb, rel),
        (Shape::Circle(ra), Shape::Circle(rb)) => circle_circle(a.at, ra, b.at, rb),
        (Shape::Box(h), Shape::Circle(r)) => box_circle(a.at, h, b.at, r),
        (Shape::Circle(r), Shape::Box(h)) => box_circle(b.at, h, a.at, r).map(|m| Manifold { normal: -m.normal, ..m }),
    }
}

/// +1 for zero, so coincident centers still get a normal.
fn sign(v: f32) -> f32 {
    if v < 0.0 { -1.0 } else { 1.0 }
}

fn box_box(pa: Vec2, ha: Vec2, pb: Vec2, hb: Vec2, rel: Vec2) -> Option<Manifold> {
    let d = pb - pa;
    let ox = ha.x + hb.x - d.x.abs();
    let oy = ha.y + hb.y - d.y.abs();
    if ox < -MARGIN || oy < -MARGIN {
        return None;
    }
    // The axis of least overlap is the one they came together along,
    // except at a seam or a corner, where neither box is inside the other
    // and they're flush on one axis: a box running along a row of tiles
    // meets the next tile's corner, and would take its side for a wall and
    // snag. There the contact is on the face the box slides along, the axis
    // it's moving least on. Flush is either way: resting exactly on a
    // floor, rounding makes the overlap a hair over or under from frame to
    // frame.
    let flush = |o: f32| o.abs() <= FLUSH;
    let corner = (flush(ox) || flush(oy)) && ox <= FLUSH && oy <= FLUSH;
    let on_x = if corner { rel.y.abs() > rel.x.abs() } else { ox < oy };
    if on_x {
        Some(Manifold { normal: Vec2::new(sign(d.x), 0.0), depth: ox })
    } else {
        Some(Manifold { normal: Vec2::new(0.0, sign(d.y)), depth: oy })
    }
}

fn circle_circle(pa: Vec2, ra: f32, pb: Vec2, rb: f32) -> Option<Manifold> {
    let d = pb - pa;
    let dist = d.len();
    let depth = ra + rb - dist;
    if depth < -MARGIN {
        return None;
    }
    let normal = if dist > 0.0 { d * (1.0 / dist) } else { Vec2::new(0.0, 1.0) };
    Some(Manifold { normal, depth })
}

/// Normal from the box toward the circle.
fn box_circle(pb: Vec2, h: Vec2, c: Vec2, r: f32) -> Option<Manifold> {
    let local = c - pb;
    let closest = local.clamp(-h, h);
    if closest == local {
        // The center is inside the box: out through the nearest face.
        let (fx, fy) = (h.x - local.x.abs(), h.y - local.y.abs());
        return Some(if fx < fy {
            Manifold { normal: Vec2::new(sign(local.x), 0.0), depth: fx + r }
        } else {
            Manifold { normal: Vec2::new(0.0, sign(local.y)), depth: fy + r }
        });
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
    use physics::{circle, rect};

    use super::*;

    #[test]
    fn boxes_push_apart_along_the_shallower_axis() {
        // b sits on a, sunk 0.1 into it.
        let m =
            collide(&rect(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.5)), &rect(Vec2::new(0.2, -0.9), Vec2::new(0.5, 0.5)), Vec2::ZERO).unwrap();
        assert_eq!(m.normal, Vec2::new(0.0, -1.0));
        assert!((m.depth - 0.1).abs() < 1e-5, "{m:?}");
    }

    /// The next tile's velocity relative to a box running right at 7.
    const RUNNING: Vec2 = Vec2::new(-7.0, 0.0);

    #[test]
    fn a_box_running_along_a_row_of_tiles_rests_on_the_next_one_too() {
        let next = rect(Vec2::new(1.5, 0.5), Vec2::new(0.5, 0.5));
        // Sunk 0.005 into the floor, 0.03 short of the next tile's side.
        let sunk = rect(Vec2::new(0.57, -0.495), Vec2::new(0.4, 0.5));
        assert_eq!(collide(&sunk, &next, RUNNING).unwrap().normal, Vec2::new(0.0, 1.0));
        // Flush with the floor, a hair above it by rounding.
        let hair = rect(Vec2::new(0.57, -0.5000001), Vec2::new(0.4, 0.5));
        assert_eq!(collide(&hair, &next, RUNNING).unwrap().normal, Vec2::new(0.0, 1.0));
        // Exactly corner to corner.
        let corner = rect(Vec2::new(0.6, -0.5), Vec2::new(0.4, 0.5));
        assert_eq!(collide(&corner, &next, RUNNING).unwrap().normal, Vec2::new(0.0, 1.0));
    }

    #[test]
    fn a_box_falling_along_a_wall_of_tiles_slides_past_the_next_one() {
        // Flush against the wall's face, meeting the next tile's corner.
        let falling = Vec2::new(0.0, -7.0);
        let corner = rect(Vec2::new(0.6, -0.5), Vec2::new(0.4, 0.5));
        let next = rect(Vec2::new(1.5, 0.5), Vec2::new(0.5, 0.5));
        assert_eq!(collide(&corner, &next, falling).unwrap().normal, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn a_wall_the_box_is_well_inside_of_is_a_wall_however_it_moves() {
        let player = rect(Vec2::new(0.57, -0.495), Vec2::new(0.4, 0.5));
        let wall = rect(Vec2::new(1.5, -0.5), Vec2::new(0.5, 0.5));
        assert_eq!(collide(&player, &wall, RUNNING).unwrap().normal, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn a_near_miss_is_a_speculative_contact_and_a_far_one_nothing() {
        let a = rect(Vec2::ZERO, Vec2::new(0.5, 0.5));
        let m = collide(&a, &rect(Vec2::new(1.02, 0.0), Vec2::new(0.5, 0.5)), Vec2::ZERO).unwrap();
        assert!(m.depth < 0.0 && m.depth > -MARGIN, "{m:?}");
        assert!(collide(&a, &rect(Vec2::new(1.2, 0.0), Vec2::new(0.5, 0.5)), Vec2::ZERO).is_none());
    }

    #[test]
    fn a_circle_against_a_box_edge_and_corner() {
        let b = rect(Vec2::ZERO, Vec2::new(1.0, 1.0));
        let edge = collide(&b, &circle(Vec2::new(0.0, 1.4), 0.5), Vec2::ZERO).unwrap();
        assert_eq!(edge.normal, Vec2::new(0.0, 1.0));
        assert!((edge.depth - 0.1).abs() < 1e-5);
        let corner = collide(&b, &circle(Vec2::new(1.3, 1.3), 0.5), Vec2::ZERO).unwrap();
        assert!((corner.normal.x - corner.normal.y).abs() < 1e-5 && corner.normal.x > 0.0);
        // Swapped, the normal flips: still from a to b.
        let flipped = collide(&circle(Vec2::new(0.0, 1.4), 0.5), &b, Vec2::ZERO).unwrap();
        assert_eq!(flipped.normal, Vec2::new(0.0, -1.0));
    }

    #[test]
    fn a_circle_whose_center_is_inside_a_box_leaves_through_the_nearest_face() {
        let m = collide(&rect(Vec2::ZERO, Vec2::new(2.0, 1.0)), &circle(Vec2::new(1.8, 0.0), 0.5), Vec2::ZERO).unwrap();
        assert_eq!(m.normal, Vec2::new(1.0, 0.0));
        assert!((m.depth - 0.7).abs() < 1e-5, "{m:?}");
    }
}
