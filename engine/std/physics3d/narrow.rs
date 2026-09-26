//! Contact between two shapes in 3D: the normal from `a` to `b` and up to
//! four points, each halfway between the surfaces, with how deep it is and
//! a feature id, so the next step can find its impulse again.
//!
//! Pairs within `MARGIN` of touching count, with a negative depth: the
//! speculative contacts of the 2D step.
//!
//! Box against box is the separating axis test and clipping, as Box3D
//! (`b3CollideHulls`, convex_manifold.c) and Parry
//! (`contact_manifold_cuboid_cuboid`) do it: the 15 axes (three faces of
//! each box, nine edge pairs), the face whose plane separates most as the
//! reference, the other box's face most against it clipped to it, and an
//! edge pair only when its axis separates clearly more (Box3D's biases).
//! More than four points are reduced to four as Box3D does
//! (`b3ReduceManifoldPoints`). What else was measured, and why this:
//! physics.md, "Rotation in 3D". `gjk.rs` is Jolt's way, kept to measure
//! against.

use crate::{Mat3, Shape, Vec3};

pub const MARGIN: f32 = 0.05;
/// Box3D's linear slop: the scale of the biases that keep a manifold from
/// flickering between features of nearly the same separation.
pub const SLOP: f32 = 0.005;
pub const MAX_POINTS: usize = 4;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    /// Halfway between the surfaces, in the world: Box3D's choice, so a
    /// point stays put when which box is the reference swaps.
    pub at: Vec3,
    /// Positive when overlapping.
    pub depth: f32,
    /// The features that made it (see `face_contact`), for warm starting.
    pub id: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Manifold {
    /// Unit, from `a` toward `b`.
    pub normal: Vec3,
    pub points: [Point; MAX_POINTS],
    pub count: usize,
    /// The separating axis that made it and its separation (Box3D's
    /// `b3SATCache`), for the next step to try first: 0 for none.
    pub axis: u32,
    pub axis_sep: f32,
}

impl Manifold {
    fn one(normal: Vec3, at: Vec3, depth: f32) -> Manifold {
        let mut m = Manifold { normal, count: 1, ..Default::default() };
        m.points[0] = Point { at, depth, id: 0 };
        m
    }

    pub fn points(&self) -> &[Point] {
        &self.points[..self.count]
    }

    pub fn deepest(&self) -> f32 {
        self.points().iter().map(|p| p.depth).fold(f32::MIN, f32::max)
    }
}

/// A shape where it is.
#[derive(Clone, Copy, Debug)]
pub struct Solid {
    pub at: Vec3,
    /// Columns: the body's axes in the world.
    pub rot: Mat3,
    pub shape: Shape,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoxBox {
    /// All 15 axes every step (Parry).
    Sat,
    /// The last step's axis first, kept if its separation changed by
    /// under `SLOP` (Box3D).
    SatCached,
    /// GJK and EPA for the normal, then the faces clipped (Jolt).
    GjkEpa,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reduce {
    /// The deepest, the farthest from it, the largest triangle, the most
    /// area added (Box3D).
    Area,
    /// The deepest, the farthest from it, and the farthest either side of
    /// the line through them (Rapier's and Jolt's).
    Line,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Narrow {
    pub box_box: BoxBox,
    pub reduce: Reduce,
}

impl Default for Narrow {
    fn default() -> Narrow {
        Narrow { box_box: BoxBox::SatCached, reduce: Reduce::Area }
    }
}

pub fn collide(a: &Solid, b: &Solid, cache: (u32, f32), how: Narrow) -> Option<Manifold> {
    match (a.shape, b.shape) {
        (Shape::Sphere(ra), Shape::Sphere(rb)) => sphere_sphere(a.at, ra, b.at, rb),
        (Shape::Box(h), Shape::Sphere(r)) => box_sphere(a, h, b.at, r),
        (Shape::Sphere(r), Shape::Box(h)) => box_sphere(b, h, a.at, r).map(|m| Manifold { normal: -m.normal, ..m }),
        (Shape::Box(ha), Shape::Box(hb)) => match how.box_box {
            BoxBox::Sat => box_box(a, ha, b, hb, None, how.reduce),
            BoxBox::SatCached => box_box(a, ha, b, hb, Some(cache), how.reduce),
            BoxBox::GjkEpa => crate::gjk::box_box(a, ha, b, hb, how.reduce),
        },
    }
}

fn sphere_sphere(pa: Vec3, ra: f32, pb: Vec3, rb: f32) -> Option<Manifold> {
    let d = pb - pa;
    let dist = d.len();
    let depth = ra + rb - dist;
    if depth < -MARGIN {
        return None;
    }
    let n = if dist > 0.0 { d * (1.0 / dist) } else { Vec3::Y };
    Some(Manifold::one(n, (pa + n * ra + pb - n * rb) * 0.5, depth))
}

/// Normal from the box toward the sphere.
fn box_sphere(b: &Solid, h: Vec3, c: Vec3, r: f32) -> Option<Manifold> {
    let r_ = b.rot.cols;
    let rel = c - b.at;
    let local = Vec3::new(rel.dot(r_[0]), rel.dot(r_[1]), rel.dot(r_[2]));
    let closest = local.clamp(-h, h);
    let (n, surface, depth) = if closest == local {
        // The center is inside the box: out through the nearest face.
        let f = [h.x - local.x.abs(), h.y - local.y.abs(), h.z - local.z.abs()];
        let axis = least(f);
        let mut n = Vec3::ZERO;
        n.set(axis, sign(local.get(axis)));
        let mut s = local;
        s.set(axis, sign(local.get(axis)) * h.get(axis));
        (n, s, f[axis] + r)
    } else {
        let d = local - closest;
        let dist = d.len();
        if r - dist < -MARGIN {
            return None;
        }
        (d * (1.0 / dist), closest, r - dist)
    };
    let n = b.rot.apply(n);
    let on_box = b.at + b.rot.apply(surface);
    Some(Manifold::one(n, (on_box + c - n * r) * 0.5, depth))
}

/// +1 for zero, so coincident centers still get a normal.
fn sign(v: f32) -> f32 {
    if v < 0.0 { -1.0 } else { 1.0 }
}

fn least(f: [f32; 3]) -> usize {
    if f[0] < f[1] && f[0] < f[2] {
        0
    } else if f[1] <= f[2] {
        1
    } else {
        2
    }
}

/// Which axis, in a cache: a face of `a` (1 + i), of `b` (4 + j), or the
/// edge pair (7 + 3i + j).
const FACE_A: u32 = 1;
const FACE_B: u32 = 4;
const EDGES: u32 = 7;

/// The boxes as the separating axis test sees them: the half extents, the
/// absolute cosines between their axes, and the center offset in each frame.
struct Pair<'a> {
    a: &'a Solid,
    b: &'a Solid,
    ha: [f32; 3],
    hb: [f32; 3],
    d: Vec3,
    abs: [[f32; 3]; 3],
    da: [f32; 3],
    db: [f32; 3],
}

impl Pair<'_> {
    /// How far apart along `axis`, and its direction from `a` toward `b`.
    fn separation(&self, axis: u32) -> Option<(f32, Vec3)> {
        let (ra, rb) = (self.a.rot.cols, self.b.rot.cols);
        if axis < FACE_B {
            let i = (axis - FACE_A) as usize;
            let reach = self.hb[0] * self.abs[i][0] + self.hb[1] * self.abs[i][1] + self.hb[2] * self.abs[i][2];
            return Some((self.da[i].abs() - self.ha[i] - reach, ra[i] * sign(self.da[i])));
        }
        if axis < EDGES {
            let j = (axis - FACE_B) as usize;
            let reach = self.ha[0] * self.abs[0][j] + self.ha[1] * self.abs[1][j] + self.ha[2] * self.abs[2][j];
            return Some((self.db[j].abs() - self.hb[j] - reach, rb[j] * sign(self.db[j])));
        }
        let (i, j) = (((axis - EDGES) / 3) as usize, ((axis - EDGES) % 3) as usize);
        let c = ra[i].cross(rb[j]);
        let l = c.len();
        // Edges near parallel: their faces' axes decide.
        if l < 1e-4 {
            return None;
        }
        let n = c * (1.0 / l);
        let n = n * sign(self.d.dot(n));
        let reach = |r: [Vec3; 3], h: [f32; 3]| h[0] * r[0].dot(n).abs() + h[1] * r[1].dot(n).abs() + h[2] * r[2].dot(n).abs();
        Some((self.d.dot(n) - reach(ra, self.ha) - reach(rb, self.hb), n))
    }
}

fn box_box(a: &Solid, ha: Vec3, b: &Solid, hb: Vec3, cache: Option<(u32, f32)>, reduce: Reduce) -> Option<Manifold> {
    let (ra, rb) = (a.rot.cols, b.rot.cols);
    let d = b.at - a.at;
    let mut abs = [[0.0; 3]; 3];
    for (i, row) in abs.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = ra[i].dot(rb[j]).abs();
        }
    }
    let p = Pair {
        a,
        b,
        ha: [ha.x, ha.y, ha.z],
        hb: [hb.x, hb.y, hb.z],
        d,
        abs,
        da: [d.dot(ra[0]), d.dot(ra[1]), d.dot(ra[2])],
        db: [d.dot(rb[0]), d.dot(rb[1]), d.dot(rb[2])],
    };
    if let Some((axis, sep)) = cache
        && axis != 0
        && let Some((s, n)) = p.separation(axis)
    {
        if s > MARGIN {
            return None;
        }
        if (s - sep).abs() < SLOP
            && let Some(m) = contact(&p, axis, s, n, reduce)
        {
            return Some(m);
        }
    }
    let mut face = (f32::MIN, 0, Vec3::ZERO);
    for axis in FACE_A..EDGES {
        let (s, n) = p.separation(axis).expect("faces always have an axis");
        if s > MARGIN {
            return None;
        }
        // Faces of `b` only when clearly better: Box3D prefers `a`'s.
        let bias = if axis < FACE_B { 0.0 } else { 0.5 * SLOP };
        if s > face.0 + bias {
            face = (s, axis, n);
        }
    }
    // Every axis separates or not; only edge pairs that can touch (see
    // `edge_contact`) are candidates for the contact.
    let mut edge = (f32::MIN, 0, Vec3::ZERO);
    let mut best = None;
    for axis in EDGES..EDGES + 9 {
        if let Some((s, n)) = p.separation(axis) {
            if s > MARGIN {
                return None;
            }
            if s > edge.0
                && let Some(e) = edge_contact(&p, axis, s, n)
            {
                (edge, best) = ((s, axis, n), Some(e));
            }
        }
    }
    let m = contact(&p, face.1, face.0, face.2, reduce);
    // Box3D's rule: an edge pair only if the face gave nothing or the edge
    // axis separates clearly more.
    if let Some(e) = best
        && (m.is_none() || edge.0 > 0.9 * face.0 + 0.5 * SLOP)
    {
        return Some(e);
    }
    m
}

fn contact(p: &Pair, axis: u32, sep: f32, n: Vec3, reduce: Reduce) -> Option<Manifold> {
    let mut m = if axis < FACE_B {
        face_contact(p.a, p.ha, (axis - FACE_A) as usize, n, p.b, p.hb, 0, reduce)
    } else if axis < EDGES {
        face_contact(p.b, p.hb, (axis - FACE_B) as usize, -n, p.a, p.ha, 1, reduce).map(|m| Manifold { normal: -m.normal, ..m })
    } else {
        edge_contact(p, axis, sep, n)
    }?;
    (m.axis, m.axis_sep) = (axis, sep);
    Some(m)
}

/// A polygon vertex while clipping: where it is, the feature its outgoing
/// side lies on (an incident edge 0-3, or a reference side 4-7), and its id.
#[derive(Clone, Copy, Default)]
struct V {
    p: Vec3,
    out: u8,
    id: u8,
}

/// Up to 8 points: a quad clipped by four planes.
#[derive(Clone, Copy, Default)]
struct Poly {
    v: [V; 8],
    n: usize,
}

impl Poly {
    fn push(&mut self, v: V) {
        self.v[self.n] = v;
        self.n += 1;
    }

    /// Sutherland-Hodgman against plane `k`: keeps where m.x <= o. A new
    /// vertex's id is the side it cut and the plane, so the same pair of
    /// features gives the same id every step; its outgoing feature is the
    /// same side when entering, the plane when leaving.
    fn clip(&self, m: Vec3, o: f32, k: u8) -> Poly {
        let mut out = Poly::default();
        for i in 0..self.n {
            let (p, q) = (self.v[i], self.v[(i + 1) % self.n]);
            let (dp, dq) = (m.dot(p.p) - o, m.dot(q.p) - o);
            if dp <= 0.0 {
                out.push(p);
            }
            if (dp <= 0.0) != (dq <= 0.0) {
                let x = p.p + (q.p - p.p) * (dp / (dp - dq));
                let side = if dp > 0.0 { p.out } else { 4 + k };
                out.push(V { p: x, out: side, id: 8 + p.out * 4 + k });
            }
        }
        out
    }
}

/// The reference box's face `i` facing along `n`, and the incident box's
/// face most against `n` clipped to it. An id is which box is the
/// reference (1 bit), its face and the incident face (3 bits each) and the
/// clipped vertex (6), so a point keeps its id while the same features
/// touch.
#[allow(clippy::too_many_arguments)]
pub(crate) fn face_contact(
    r: &Solid,
    rh: [f32; 3],
    i: usize,
    n: Vec3,
    inc: &Solid,
    ih: [f32; 3],
    flag: u32,
    reduce: Reduce,
) -> Option<Manifold> {
    let (rc, ic) = (r.rot.cols, inc.rot.cols);
    let k = (0..3).max_by(|&x, &y| ic[x].dot(n).abs().total_cmp(&ic[y].dot(n).abs())).unwrap_or(0);
    let s = -sign(ic[k].dot(n));
    let center = inc.at + ic[k] * (s * ih[k]);
    let (u, v) = (ic[(k + 1) % 3] * ih[(k + 1) % 3], ic[(k + 2) % 3] * ih[(k + 2) % 3]);
    let mut poly = Poly::default();
    for (j, (a, b)) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)].into_iter().enumerate() {
        poly.push(V { p: center + u * a + v * b, out: j as u8, id: j as u8 });
    }
    let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
    for (plane, (axis, s)) in [(i1, 1.0), (i1, -1.0), (i2, 1.0), (i2, -1.0)].into_iter().enumerate() {
        let m = rc[axis] * s;
        poly = poly.clip(m, m.dot(r.at) + rh[axis], plane as u8);
    }
    let face = r.at + n * rh[i];
    let ref_face = (2 * i + (rc[i].dot(n) < 0.0) as usize) as u32;
    let inc_face = (2 * k + (s < 0.0) as usize) as u32;
    let head = (flag << 12) | (ref_face << 9) | (inc_face << 6);
    let mut all = [Point::default(); 8];
    let mut count = 0;
    for x in &poly.v[..poly.n] {
        let sep = n.dot(x.p - face);
        if sep <= MARGIN {
            all[count] = Point { at: x.p - n * (0.5 * sep), depth: -sep, id: head | x.id as u32 };
            count += 1;
        }
    }
    if count == 0 {
        return None;
    }
    let mut m = Manifold { normal: n, count: count.min(MAX_POINTS), ..Default::default() };
    if count > MAX_POINTS {
        m.points[..4].copy_from_slice(&reduce_to_four(&all[..count], n, reduce));
    } else {
        m.points[..count].copy_from_slice(&all[..count]);
    }
    Some(m)
}

/// Whether the edges of `a` between faces with normals `a1`, `a2` and of
/// `b` between `b1`, `b2` cross on the Gauss map: whether they make a face
/// of the Minkowski difference, so that their axis is one the boxes can
/// touch along: Box3D's `b3IsMinkowskiFace` (convex_manifold.c). Every
/// axis still separates or not; this only decides which edge pairs may
/// make the contact. On our scenes it changed nothing measurable (physics.md,
/// "Rotation in 3D"): the guard that did is the one in `edge_contact`.
fn builds_face(a1: Vec3, a2: Vec3, b1: Vec3, b2: Vec3) -> bool {
    let (c, d) = (-b1, -b2);
    let (bxa, dxc) = (a2.cross(a1), d.cross(c));
    let (cba, dba, adc, bdc) = (c.dot(bxa), d.dot(bxa), a1.dot(dxc), a2.dot(dxc));
    cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0
}

/// One point, halfway between the closest points of the two edges; none if
/// the edges don't make a face of the Minkowski difference or their closest
/// points aren't on them. Clamping such points back onto the edges, as a
/// segment-to-segment distance would, made contacts between edges that
/// don't touch: box piles went from settling in about 200 steps to never
/// (`every_point_is_where_both_boxes_are`).
fn edge_contact(p: &Pair, axis: u32, sep: f32, n: Vec3) -> Option<Manifold> {
    let (ra, rb) = (p.a.rot.cols, p.b.rot.cols);
    let (i, j) = (((axis - EDGES) / 3) as usize, ((axis - EDGES) % 3) as usize);
    // The edge of each box farthest along the normal toward the other, and
    // the normals of the faces either side of it.
    let mut ea = p.a.at;
    let mut eb = p.b.at;
    let (mut fa, mut fb) = ([Vec3::ZERO; 2], [Vec3::ZERO; 2]);
    let (mut ka, mut kb) = (0, 0);
    for k in 0..3 {
        if k != i {
            let s = sign(ra[k].dot(n));
            ea += ra[k] * (p.ha[k] * s);
            fa[ka] = ra[k] * s;
            ka += 1;
        }
        if k != j {
            let s = -sign(rb[k].dot(n));
            eb += rb[k] * (p.hb[k] * s);
            fb[kb] = rb[k] * s;
            kb += 1;
        }
    }
    if !builds_face(fa[0], fa[1], fb[0], fb[1]) {
        return None;
    }
    let (u, v) = (ra[i], rb[j]);
    // Closest points of the lines ea + s u and eb + t v, clamped to the
    // edges.
    let w = ea - eb;
    let (b_, d_, e_) = (u.dot(v), u.dot(w), v.dot(w));
    let den = 1.0 - b_ * b_;
    let (s, t) = if den > 1e-6 { ((b_ * e_ - d_) / den, (e_ - b_ * d_) / den) } else { (0.0, e_) };
    if s.abs() > p.ha[i] || t.abs() > p.hb[j] {
        return None;
    }
    let (pa, pb) = (ea + u * s, eb + v * t);
    let mut m = Manifold::one(n, (pa + pb) * 0.5, -sep);
    m.points[0].id = (1 << 13) | axis;
    (m.axis, m.axis_sep) = (axis, sep);
    Some(m)
}

/// Four of more than four points, keeping the deepest and as much of the
/// area as four can.
pub fn reduce_to_four(all: &[Point], n: Vec3, how: Reduce) -> [Point; 4] {
    let planar = |a: Vec3, b: Vec3| {
        let d = b - a;
        let d = d - n * d.dot(n);
        d.dot(d)
    };
    let best = |score: &dyn Fn(&Point) -> f32| {
        let mut k = 0;
        for (j, p) in all.iter().enumerate() {
            if score(p) > score(&all[k]) {
                k = j;
            }
        }
        k
    };
    let p1 = best(&|p| p.depth);
    let a = all[p1].at;
    let p2 = best(&|p| planar(a, p.at) + 4.0 * p.depth.max(0.0).powi(2));
    let b = all[p2].at;
    let (p3, p4) = match how {
        Reduce::Area => {
            let area = |x: Vec3, y: Vec3, z: Vec3| (y - x).cross(z - x).dot(n);
            let p3 = best(&|p| area(a, b, p.at).abs());
            let c = all[p3].at;
            // How much area a point adds outside the triangle: the most
            // negative of its areas with each edge, on the triangle's side.
            let s = sign(area(a, b, c));
            let outside = |p: &Point| 0.0 - (s * area(a, b, p.at)).min(s * area(b, c, p.at)).min(s * area(c, a, p.at));
            (p3, best(&outside))
        }
        Reduce::Line => {
            let across = (b - a).cross(n);
            let off = |p: &Point| across.dot(p.at - a);
            (best(&|p| 0.0 - off(p)), best(&off))
        }
    };
    [all[p1], all[p2], all[p3], all[p4]]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Quat;

    fn cube(at: Vec3, q: Quat) -> Solid {
        Solid { at, rot: q.matrix(), shape: Shape::Box(Vec3::splat(0.5)) }
    }

    fn hit(a: &Solid, b: &Solid, box_box: BoxBox) -> Manifold {
        collide(a, b, (0, 0.0), Narrow { box_box, reduce: Reduce::Area }).expect("touching")
    }

    #[test]
    fn a_box_on_a_box_touches_at_its_four_corners() {
        let (a, b) = (cube(Vec3::ZERO, Quat::IDENTITY), cube(Vec3::new(0.1, 0.99, -0.2), Quat::IDENTITY));
        for how in [BoxBox::Sat, BoxBox::SatCached, BoxBox::GjkEpa] {
            let m = hit(&a, &b, how);
            assert_eq!(m.count, 4, "{how:?} {m:?}");
            assert!((m.normal - Vec3::Y).len() < 1e-5, "{how:?} {m:?}");
            for p in m.points() {
                assert!((p.depth - 0.01).abs() < 1e-4, "{how:?} {p:?}");
                // Halfway between the faces, inside both.
                assert!((p.at.y - 0.495).abs() < 1e-4 && p.at.x.abs() <= 0.5 + 1e-5 && p.at.z.abs() <= 0.5 + 1e-5, "{how:?} {p:?}");
            }
        }
    }

    #[test]
    fn a_box_on_its_edge_touches_at_two_points() {
        let q = Quat::axis_angle(Vec3::Z, std::f32::consts::FRAC_PI_4);
        let (a, b) = (cube(Vec3::ZERO, Quat::IDENTITY), cube(Vec3::new(0.0, 0.5 + 0.5f32.sqrt() - 0.01, 0.0), q));
        for how in [BoxBox::Sat, BoxBox::GjkEpa] {
            let m = hit(&a, &b, how);
            assert_eq!(m.count, 2, "{how:?} {m:?}");
            assert!((m.normal - Vec3::Y).len() < 1e-4, "{how:?} {m:?}");
            assert!(m.points().iter().all(|p| (p.depth - 0.01).abs() < 1e-3 && p.at.x.abs() < 1e-3), "{how:?} {m:?}");
        }
    }

    #[test]
    fn a_box_on_its_corner_touches_at_one() {
        // Turned so a corner points straight down.
        let q = Quat::axis_angle(Vec3::new(1.0, 0.0, -1.0), 0.9553166);
        let b = cube(Vec3::new(0.0, 0.5 + 0.75f32.sqrt() - 0.01, 0.0), q);
        let m = hit(&cube(Vec3::ZERO, Quat::IDENTITY), &b, BoxBox::Sat);
        assert_eq!(m.count, 1, "{m:?}");
        assert!((m.points[0].depth - 0.01).abs() < 1e-3, "{m:?}");
    }

    #[test]
    fn crossed_edges_touch_at_one_point_between_them() {
        // Both on an edge, one turned a quarter about y: the edges cross.
        let qa = Quat::axis_angle(Vec3::X, std::f32::consts::FRAC_PI_4);
        let qb = Quat::axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_2).times(qa);
        let gap = 2.0 * 0.5f32.sqrt() - 0.02;
        let m = hit(&cube(Vec3::ZERO, qa), &cube(Vec3::new(0.0, gap, 0.0), qb), BoxBox::Sat);
        assert_eq!(m.count, 1, "{m:?}");
        assert!((m.normal - Vec3::Y).len() < 1e-4 && (m.points[0].depth - 0.02).abs() < 1e-4, "{m:?}");
        assert!((m.points[0].at - Vec3::new(0.0, gap / 2.0, 0.0)).len() < 1e-4, "{m:?}");
    }

    #[test]
    fn a_box_tipped_on_a_wide_floor_touches_the_floor_not_its_far_edges() {
        // The floor is a box too, with edges 20 away: the contact is its
        // face, near the box.
        let floor = Solid { at: Vec3::new(0.0, -0.5, 0.0), rot: Quat::IDENTITY.matrix(), shape: Shape::Box(Vec3::new(20.0, 0.5, 20.0)) };
        let q = Quat::axis_angle(Vec3::new(1.0, 0.0, 0.3), 0.5);
        let b = cube(Vec3::new(3.0, 0.62, 1.0), q);
        let m = hit(&floor, &b, BoxBox::Sat);
        assert!((m.normal - Vec3::Y).len() < 1e-5, "{m:?}");
        assert!(m.points().iter().all(|p| (p.at - b.at).len() < 1.0), "{m:?}");
    }

    #[test]
    fn edges_make_a_minkowski_face_only_where_their_arcs_cross() {
        let r = 0.5f32.sqrt();
        // The top edge of a box turned 45 degrees about x, and the bottom
        // edge of one turned about z above it: crossed.
        let (a1, a2) = (Vec3::new(0.0, r, r), Vec3::new(0.0, r, -r));
        assert!(builds_face(a1, a2, Vec3::new(-r, -r, 0.0), Vec3::new(r, -r, 0.0)));
        // The same edge against the top edge of that box: they face the
        // same way and can't touch.
        assert!(!builds_face(a1, a2, Vec3::new(-r, r, 0.0), Vec3::new(r, r, 0.0)));
    }

    #[test]
    fn eight_points_reduce_to_the_four_that_keep_the_most_area() {
        // A box turned 45 degrees about y on another: an octagon of eight.
        let a = cube(Vec3::ZERO, Quat::IDENTITY);
        let b = cube(Vec3::new(0.0, 0.99, 0.0), Quat::axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_4));
        let m = hit(&a, &b, BoxBox::Sat);
        assert_eq!(m.count, 4);
        // The quad's area, its points taken round the centroid.
        let c = m.points().iter().fold(Vec3::ZERO, |s, p| s + p.at) * 0.25;
        let mut pts: Vec<Vec3> = m.points().iter().map(|p| p.at - c).collect();
        pts.sort_by(|p, q| p.z.atan2(p.x).total_cmp(&q.z.atan2(q.x)));
        let area: f32 = (0..4).map(|k| pts[k].cross(pts[(k + 1) % 4]).y.abs() * 0.5).sum();
        // The octagon is 0.828 square; the best four of its corners make
        // 0.586 (a square on alternate corners); two sides' worth, 0.414.
        assert!(area > 0.55, "kept {area} of the area: {m:?}");
    }

    #[test]
    fn ids_stay_put_while_the_same_features_touch() {
        let a = cube(Vec3::ZERO, Quat::IDENTITY);
        let ids = |x: f32| {
            let m = hit(&a, &cube(Vec3::new(x, 0.99, 0.3), Quat::axis_angle(Vec3::Y, 0.2)), BoxBox::Sat);
            let mut ids: Vec<u32> = m.points().iter().map(|p| p.id).collect();
            ids.sort();
            ids
        };
        assert_eq!(ids(0.1), ids(0.105));
        assert_ne!(ids(0.1), ids(-0.3), "other corners, other ids");
    }

    #[test]
    fn the_cached_axis_gives_what_the_full_test_does() {
        let a = cube(Vec3::ZERO, Quat::IDENTITY);
        let b = cube(Vec3::new(0.1, 0.98, 0.2), Quat::axis_angle(Vec3::new(0.2, 1.0, 0.1), 0.3));
        let full = hit(&a, &b, BoxBox::Sat);
        let cached = collide(&a, &b, (full.axis, full.axis_sep), Narrow::default()).unwrap();
        assert_eq!(full, cached);
        // A stale axis is tried and dropped.
        let stale = collide(&a, &b, (FACE_A, full.axis_sep + 1.0), Narrow::default()).unwrap();
        assert_eq!(full.normal, stale.normal);
    }

    /// How far `p` is outside the cube `s` (0 inside).
    fn outside(s: &Solid, p: Vec3) -> f32 {
        let r = s.rot.cols;
        let d = p - s.at;
        let l = Vec3::new(d.dot(r[0]), d.dot(r[1]), d.dot(r[2]));
        (l - l.clamp(Vec3::splat(-0.5), Vec3::splat(0.5))).len()
    }

    #[test]
    fn every_point_is_where_both_boxes_are() {
        // Halfway between two surfaces within the margin: never farther
        // than the margin from either box. An edge pair whose closest
        // points lie off the edges, clamped back onto them, made points
        // up to a box away from both (and piles that never settled).
        let mut seed = 0x1234_5678u32;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as f32 / u32::MAX as f32
        };
        let mut seen = 0;
        for _ in 0..4000 {
            let qa = Quat::axis_angle(Vec3::new(rand() - 0.5, rand() - 0.5, rand() - 0.5), rand() * 6.0);
            let qb = Quat::axis_angle(Vec3::new(rand() - 0.5, rand() - 0.5, rand() - 0.5), rand() * 6.0);
            let dir = Vec3::new(rand() - 0.5, rand() - 0.5, rand() - 0.5).normalize();
            let (a, b) = (cube(Vec3::ZERO, qa), cube(dir * (0.9 + rand() * 0.6), qb));
            let Some(m) = collide(&a, &b, (0, 0.0), Narrow { box_box: BoxBox::Sat, reduce: Reduce::Area }) else { continue };
            seen += 1;
            for p in m.points() {
                assert!(outside(&a, p.at) < MARGIN && outside(&b, p.at) < MARGIN, "{p:?} of {m:?}");
            }
        }
        assert!(seen > 1000, "{seen}");
    }

    #[test]
    fn gjk_and_the_separating_axis_test_agree() {
        let mut seed = 0x9e37_79b9u32;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as f32 / u32::MAX as f32
        };
        let mut compared = 0;
        for _ in 0..500 {
            let qa = Quat::axis_angle(Vec3::new(rand() - 0.5, rand() - 0.5, rand() - 0.5), rand() * 6.0);
            let qb = Quat::axis_angle(Vec3::new(rand() - 0.5, rand() - 0.5, rand() - 0.5), rand() * 6.0);
            let (a, b) = (cube(Vec3::ZERO, qa), cube(Vec3::new(rand() - 0.5, 0.8 + rand() * 0.5, rand() - 0.5), qb));
            let sat = collide(&a, &b, (0, 0.0), Narrow { box_box: BoxBox::Sat, reduce: Reduce::Area });
            let gjk = collide(&a, &b, (0, 0.0), Narrow { box_box: BoxBox::GjkEpa, reduce: Reduce::Area });
            let (Some(s), Some(g)) = (sat, gjk) else {
                assert_eq!(sat.is_some(), gjk.is_some(), "{sat:?} {gjk:?}");
                continue;
            };
            // Deeper, several axes separate nearly as much, and the two
            // choose between them differently; contacts are shallow.
            if s.deepest() > 0.05 {
                continue;
            }
            compared += 1;
            assert!(s.normal.dot(g.normal) > 0.95, "normals {s:?} {g:?}");
            assert!((s.deepest() - g.deepest()).abs() < 0.01, "depths {s:?} {g:?}");
        }
        assert!(compared > 50, "{compared}");
    }
}
