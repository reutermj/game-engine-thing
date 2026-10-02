//! Contact between two shapes: the normal from `a` to `b` and how deep they
//! overlap, and, when either shape is turned, up to two contact points.
//! Two shapes that aren't turned get no points, and the tests they always
//! had: without rotation a contact needs none, since an impulse through any
//! point moves a body the same, and the platformer's seams depend on
//! `box_box`'s choice of face.
//!
//! Pairs within `MARGIN` of touching count, with a negative depth: a
//! speculative contact, which lets the solver stop a body at the surface
//! instead of after it has sunk in, and keeps resting contacts from
//! flickering in and out between steps. Speculative contacts are as Box2D
//! has them, Erin Catto's (docs/CREDITS.md).
//!
//! Turned boxes meet by Box2D's `b2CollidePolygons`: the separating axis
//! of least overlap among both boxes' faces, the other box's edge most
//! against it clipped to the face's sides, and each clipped end a point,
//! numbered by the edges it came from (`b2ClipPolygons`'s feature ids), so
//! the next step's solve can start from this one's impulses at the same
//! corners (physics.md, "Contact points"). A turned box and a circle meet
//! at one point, as `b2CollidePolygonAndCircle` has it, in the box's frame.

pub use physics_common::MARGIN;
use physics2d::{Placed, Rot, Shape, Vec2};

/// An overlap this thin is two faces resting flush, not one box inside
/// another: more than the solver's slop, which resting contacts sink to.
pub const FLUSH: f32 = 0.01;

/// A contact point: its arms from each shape's center (world axes), its
/// separation along the normal (negative: overlap), and its feature id.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    pub ra: Vec2,
    pub rb: Vec2,
    pub separation: f32,
    pub id: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Manifold {
    /// Unit, from `a` toward `b`.
    pub normal: Vec2,
    /// Positive when overlapping; negative up to `-MARGIN` when apart.
    pub depth: f32,
}

/// A contact with points: `collide_turned`'s.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Turned {
    /// Unit, from `a` toward `b`.
    pub normal: Vec2,
    /// The deepest point's: positive when overlapping.
    pub depth: f32,
    pub count: u8,
    pub points: [Point; 2],
}

/// `rel` is `b`'s velocity relative to `a`'s: what settles which face two
/// boxes meet on when their geometry can't (see `box_box`).
/// For shapes that aren't turned (their `rot` is ignored): `collide_turned`
/// is for those that are, and for bodies that turn.
pub fn collide(a: &Placed, b: &Placed, rel: Vec2) -> Option<Manifold> {
    let m = |normal, depth| Manifold { normal, depth };
    match (a.shape, b.shape) {
        (Shape::Box(ha), Shape::Box(hb)) => box_box(a.at, ha, b.at, hb, rel).map(|(n, d)| m(n, d)),
        (Shape::Circle(ra), Shape::Circle(rb)) => circle_circle(a.at, ra, b.at, rb).map(|(n, d)| m(n, d)),
        (Shape::Box(h), Shape::Circle(r)) => box_circle(a.at, h, b.at, r).map(|(n, d)| m(n, d)),
        (Shape::Circle(r), Shape::Box(h)) => box_circle(b.at, h, a.at, r).map(|(n, d)| m(-n, d)),
    }
}

/// +1 for zero, so coincident centers still get a normal.
fn sign(v: f32) -> f32 {
    if v < 0.0 { -1.0 } else { 1.0 }
}

fn box_box(pa: Vec2, ha: Vec2, pb: Vec2, hb: Vec2, rel: Vec2) -> Option<(Vec2, f32)> {
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
    if on_x { Some((Vec2::new(sign(d.x), 0.0), ox)) } else { Some((Vec2::new(0.0, sign(d.y)), oy)) }
}

fn circle_circle(pa: Vec2, ra: f32, pb: Vec2, rb: f32) -> Option<(Vec2, f32)> {
    let d = pb - pa;
    let dist = d.len();
    let depth = ra + rb - dist;
    if depth < -MARGIN {
        return None;
    }
    let normal = if dist > 0.0 { d * (1.0 / dist) } else { Vec2::new(0.0, 1.0) };
    Some((normal, depth))
}

/// Normal from the box toward the circle.
fn box_circle(pb: Vec2, h: Vec2, c: Vec2, r: f32) -> Option<(Vec2, f32)> {
    let local = c - pb;
    let closest = local.clamp(-h, h);
    if closest == local {
        // The center is inside the box: out through the nearest face.
        let (fx, fy) = (h.x - local.x.abs(), h.y - local.y.abs());
        return Some(if fx < fy { (Vec2::new(sign(local.x), 0.0), fx + r) } else { (Vec2::new(0.0, sign(local.y)), fy + r) });
    }
    let d = local - closest;
    let dist = d.len();
    let depth = r - dist;
    if depth < -MARGIN {
        return None;
    }
    Some((d * (1.0 / dist), depth))
}

/// The contact between two shapes either of which is turned, with its
/// points; or between any two, with points, for bodies that turn.
pub fn collide_turned(a: &Placed, b: &Placed) -> Option<Turned> {
    let q = |p: &Placed| p.rot.unwrap_or(Rot::IDENTITY);
    match (a.shape, b.shape) {
        (Shape::Circle(ra), Shape::Circle(rb)) => circles(a.at, ra, b.at, rb),
        (Shape::Box(h), Shape::Circle(r)) => turned_box_circle(a.at, q(a), h, b.at, r),
        (Shape::Circle(r), Shape::Box(h)) => turned_box_circle(b.at, q(b), h, a.at, r).map(flipped),
        (Shape::Box(ha), Shape::Box(hb)) => boxes(a.at, q(a), ha, b.at, q(b), hb),
    }
}

/// The same contact seen from its other end.
fn flipped(m: Turned) -> Turned {
    let mut out = Turned { normal: -m.normal, ..m };
    for p in &mut out.points[..m.count as usize] {
        std::mem::swap(&mut p.ra, &mut p.rb);
    }
    out
}

/// A point halfway between `ca` on the first shape's surface and `cb` on
/// the second's (world), as Box2D puts them, with arms from centers `pa`
/// and `pb`.
fn point(pa: Vec2, pb: Vec2, ca: Vec2, cb: Vec2, separation: f32, id: u16) -> Point {
    let mid = (ca + cb) * 0.5;
    Point { ra: mid - pa, rb: mid - pb, separation, id }
}

fn one(normal: Vec2, p: Point) -> Turned {
    Turned { normal, depth: -p.separation, count: 1, points: [p, Point::default()] }
}

fn circles(pa: Vec2, ra: f32, pb: Vec2, rb: f32) -> Option<Turned> {
    let d = pb - pa;
    let dist = d.len();
    let separation = dist - ra - rb;
    if separation > MARGIN {
        return None;
    }
    let n = if dist > 0.0 { d * (1.0 / dist) } else { Vec2::new(0.0, 1.0) };
    Some(one(n, point(pa, pb, pa + n * ra, pb - n * rb, separation, 0)))
}

/// Normal from the box (at `pb`, facing `q`) toward the circle.
fn turned_box_circle(pb: Vec2, q: Rot, h: Vec2, c: Vec2, r: f32) -> Option<Turned> {
    let local = q.unrotate(c - pb);
    let closest = local.clamp(-h, h);
    let (n, separation, surface) = if closest == local {
        // The center is inside the box: out through the nearest face.
        let (fx, fy) = (h.x - local.x.abs(), h.y - local.y.abs());
        if fx < fy {
            (Vec2::new(sign(local.x), 0.0), -fx - r, Vec2::new(sign(local.x) * h.x, local.y))
        } else {
            (Vec2::new(0.0, sign(local.y)), -fy - r, Vec2::new(local.x, sign(local.y) * h.y))
        }
    } else {
        let d = local - closest;
        let dist = d.len();
        (d * (1.0 / dist), dist - r, closest)
    };
    if separation > MARGIN {
        return None;
    }
    let n = q.rotate(n);
    Some(one(n, point(pb, c, pb + q.rotate(surface), c - n * r, separation, 0)))
}

/// A box's corners in its own frame, counter-clockwise as Box2D's
/// `b2MakeBox` has them: face `i` runs from corner `i` to corner `i + 1`,
/// facing `FACES[i]`.
fn corners(h: Vec2) -> [Vec2; 4] {
    [Vec2::new(-h.x, -h.y), Vec2::new(h.x, -h.y), Vec2::new(h.x, h.y), Vec2::new(-h.x, h.y)]
}

const FACES: [Vec2; 4] = [Vec2::new(0.0, -1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0), Vec2::new(-1.0, 0.0)];

/// A box in some frame: its corners and its faces' normals.
struct Quad {
    v: [Vec2; 4],
    n: [Vec2; 4],
}

fn next(i: usize) -> usize {
    (i + 1) % 4
}

/// Box2D's `B2_MAKE_ID`: the two features a point came from.
fn id(a: usize, b: usize) -> u16 {
    ((a as u16) << 8) | b as u16
}

/// The face of `p1` along whose normal `p2` is farthest out, and how far:
/// `b2FindMaxSeparation`.
fn max_separation(p1: &Quad, p2: &Quad) -> (f32, usize) {
    let mut best = (f32::NEG_INFINITY, 0);
    for i in 0..4 {
        let si = p2.v.iter().map(|&v| p1.n[i].dot(v - p1.v[i])).fold(f32::INFINITY, f32::min);
        if si > best.0 {
            best = (si, i);
        }
    }
    best
}

/// The face of `p` most against `normal`: the incident face.
fn incident(p: &Quad, normal: Vec2) -> usize {
    (0..4).min_by(|&i, &j| normal.dot(p.n[i]).total_cmp(&normal.dot(p.n[j]))).unwrap_or(0)
}

/// Two turned boxes, in `a`'s frame: Box2D's `b2CollidePolygons` for boxes
/// (no rounding, so without its closest-features case for rounded corners,
/// which for a box only sharpens a speculative normal).
fn boxes(pa: Vec2, qa: Rot, ha: Vec2, pb: Vec2, qb: Rot, hb: Vec2) -> Option<Turned> {
    let at = qa.unrotate(pb - pa);
    let qr = Rot { c: qa.c * qb.c + qa.s * qb.s, s: qa.c * qb.s - qa.s * qb.c };
    let a = Quad { v: corners(ha), n: FACES };
    let b = Quad { v: corners(hb).map(|v| at + qr.rotate(v)), n: FACES.map(|n| qr.rotate(n)) };
    let (sep_a, face_a) = max_separation(&a, &b);
    let (sep_b, face_b) = max_separation(&b, &a);
    if sep_a > MARGIN || sep_b > MARGIN {
        return None;
    }
    // `a`'s face unless `b`'s separates more, as Box2D v3 chooses.
    let flip = sep_b > sep_a;
    let (face_a, face_b) = if flip { (incident(&a, b.n[face_b]), face_b) } else { (face_a, incident(&b, a.n[face_a])) };
    let (normal, clipped) = clip(&a, &b, face_a, face_b, flip)?;
    let mut m = Turned { normal: qa.rotate(normal), depth: f32::NEG_INFINITY, count: 2, ..Turned::default() };
    for (p, (local, separation, id)) in m.points.iter_mut().zip(clipped) {
        let world = pa + qa.rotate(local);
        *p = Point { ra: world - pa, rb: world - pb, separation, id };
        m.depth = m.depth.max(-separation);
    }
    Some(m)
}

/// The incident face of one box clipped to the sides of the other's
/// reference face: Box2D's `b2ClipPolygons`, for shapes with no radius.
/// Both boxes in one frame; the normal and points come back in it, the
/// normal from `a` to `b`, each point halfway between the faces.
fn clip(a: &Quad, b: &Quad, face_a: usize, face_b: usize, flip: bool) -> Option<(Vec2, [(Vec2, f32, u16); 2])> {
    let (p1, p2, i11, i21) = if flip { (b, a, face_b, face_a) } else { (a, b, face_a, face_b) };
    let (i12, i22) = (next(i11), next(i21));
    let normal = p1.n[i11];
    let (v11, v12, v21, v22) = (p1.v[i11], p1.v[i12], p2.v[i21], p2.v[i22]);
    let tangent = Vec2::new(-normal.y, normal.x);
    let (lower1, upper1) = (0.0, (v12 - v11).dot(tangent));
    // The incident face runs against the tangent: counter-clockwise both.
    let (upper2, lower2) = ((v21 - v11).dot(tangent), (v22 - v11).dot(tangent));
    if upper2 < lower1 || upper1 < lower2 {
        return None;
    }
    let lerp = |from: Vec2, to: Vec2, t: f32| from + (to - from) * t;
    let lower = if lower2 < lower1 && upper2 - lower2 > f32::EPSILON { lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2)) } else { v22 };
    let upper = if upper2 > upper1 && upper2 - lower2 > f32::EPSILON { lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2)) } else { v21 };
    let (sep_lower, sep_upper) = ((lower - v11).dot(normal), (upper - v11).dot(normal));
    let (lower, upper) = (lower - normal * (0.5 * sep_lower), upper - normal * (0.5 * sep_upper));
    Some(if flip {
        (-normal, [(upper, sep_upper, id(i21, i12)), (lower, sep_lower, id(i22, i11))])
    } else {
        (normal, [(lower, sep_lower, id(i11, i22)), (upper, sep_upper, id(i12, i21))])
    })
}

#[cfg(test)]
mod tests {
    use physics2d::{circle, rect};

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

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    /// A unit box on a floor whose top is at y = 0 (y down), sunk 0.01, at
    /// `angle`.
    fn on_floor(angle: f32, x: f32) -> (Placed, Placed) {
        let floor = rect(Vec2::new(0.0, 1.0), Vec2::new(5.0, 1.0));
        let q = Rot::from_angle(angle);
        let reach = physics2d::turned_half(Vec2::new(0.5, 0.5), q).y;
        (floor, rect(Vec2::new(x, -reach + 0.01), Vec2::new(0.5, 0.5)).turned(q))
    }

    #[test]
    fn a_box_flat_on_a_floor_touches_at_both_corners() {
        let (floor, b) = on_floor(0.0, 0.3);
        let m = collide_turned(&floor, &b).unwrap();
        assert_eq!(m.count, 2, "{m:?}");
        assert!(close(m.normal.x, 0.0) && close(m.normal.y, -1.0), "from the floor up to the box: {m:?}");
        assert!(close(m.depth, 0.01), "{m:?}");
        let mut xs: Vec<f32> = m.points.iter().map(|p| (floor.at + p.ra).x).collect();
        xs.sort_by(f32::total_cmp);
        assert!(close(xs[0], -0.2) && close(xs[1], 0.8), "at the box's bottom corners: {xs:?}");
        for p in &m.points {
            assert!(close(p.separation, -0.01), "{p:?}");
            // Both arms end at the same point.
            let (wa, wb) = (floor.at + p.ra, b.at + p.rb);
            assert!(close(wa.x, wb.x) && close(wa.y, wb.y), "{p:?}");
        }
        assert_ne!(m.points[0].id, m.points[1].id);
    }

    #[test]
    fn a_tilted_box_rests_on_the_corner_it_leans_on() {
        let (floor, b) = on_floor(0.3, 0.0);
        let m = collide_turned(&floor, &b).unwrap();
        let deepest = m.points[..m.count as usize].iter().map(|p| p.separation).fold(f32::INFINITY, f32::min);
        assert!(close(deepest, -0.01), "the lowest corner, sunk 0.01: {m:?}");
        // The other end of the edge is well above: a speculative point, if
        // it's kept at all.
        assert!(m.points[..m.count as usize].iter().filter(|p| p.separation < 0.0).count() == 1, "{m:?}");
    }

    #[test]
    fn feature_ids_stay_with_the_corners_as_a_box_slides_and_turns() {
        let ids = |angle: f32, x: f32| {
            let (floor, b) = on_floor(angle, x);
            let m = collide_turned(&floor, &b).unwrap();
            let mut by_x: Vec<(f32, u16)> = m.points[..m.count as usize].iter().map(|p| ((floor.at + p.ra).x, p.id)).collect();
            by_x.sort_by(|a, b| a.0.total_cmp(&b.0));
            by_x.into_iter().map(|(_, id)| id).collect::<Vec<u16>>()
        };
        assert_eq!(ids(0.0, 0.0), ids(0.0, 0.3));
        assert_eq!(ids(0.0, 0.0), ids(0.02, 0.0));
        assert_eq!(ids(0.0, 0.0), ids(-0.02, 0.1));
    }

    #[test]
    fn swapping_the_shapes_reverses_the_normal_and_the_arms() {
        let (floor, b) = on_floor(0.2, 0.1);
        let (m, w) = (collide_turned(&floor, &b).unwrap(), collide_turned(&b, &floor).unwrap());
        assert!(close(m.normal.x, -w.normal.x) && close(m.normal.y, -w.normal.y), "{m:?} {w:?}");
        assert!(close(m.depth, w.depth));
        let c = circle(Vec2::new(0.3, -0.45), 0.5);
        let (m, w) = (collide_turned(&b, &c).unwrap(), collide_turned(&c, &b).unwrap());
        assert!(close(m.normal.x, -w.normal.x) && close(m.points[0].ra.x, w.points[0].rb.x), "{m:?} {w:?}");
    }

    #[test]
    fn a_circle_on_a_turned_box_touches_its_face() {
        // A box turned a quarter is the same box: the circle sits on top.
        let b = rect(Vec2::ZERO, Vec2::new(1.0, 0.5)).turned(Rot::from_angle(std::f32::consts::PI));
        let m = collide_turned(&b, &circle(Vec2::new(0.2, -0.95), 0.5)).unwrap();
        assert!(close(m.normal.y, -1.0) && close(m.depth, 0.05), "{m:?}");
        assert!(close(m.points[0].ra.x, 0.2), "{m:?}");
    }

    #[test]
    fn shapes_not_turned_get_points_for_bodies_that_turn_and_the_same_normal() {
        let (a, b) = (rect(Vec2::ZERO, Vec2::new(1.0, 1.0)), rect(Vec2::new(0.3, -1.9), Vec2::new(1.0, 1.0)));
        let (m, plain) = (collide_turned(&a, &b).unwrap(), collide(&a, &b, Vec2::ZERO).unwrap());
        assert_eq!(m.count, 2);
        assert!(close(m.normal.y, plain.normal.y) && close(m.depth, plain.depth), "{m:?} {plain:?}");
    }
}
