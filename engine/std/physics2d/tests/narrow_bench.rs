//! The turned-box narrowphase two ways, on the same pairs:
//! `./bazel run --config=bench //engine/std/physics2d:narrow_bench`.
//!
//! - **SAT and clipping**, Box2D's `b2CollidePolygons`: the face of least
//!   separation on either box, the other box's most opposed face clipped to
//!   its sides. What `narrow::collide_turned` does.
//! - **GJK and EPA**, then the same clipping of the faces the normal picks:
//!   the general convex route, as parry's `contact_manifold_pfm_pfm` takes
//!   it for shapes with no dedicated routine (for boxes parry uses SAT too,
//!   `contact_manifold_cuboid_cuboid`). GJK finds the distance between the
//!   shapes, or that they overlap, from their support points alone; EPA
//!   expands the overlap's simplex to the depth.
//!
//! Pairs of boxes 0.9 across at random angles, placed from just apart (the
//! speculative margin) to sunk a tenth of a box, as a turning pile's pairs
//! are. Prints ns a pair each way, and how often GJK and EPA's normal and
//! depth agree with SAT's.

#[allow(dead_code)] // The shapes that aren't turned: not what this measures.
#[path = "../narrow.rs"]
mod narrow;

use std::time::Instant;

use physics2d::{Placed, Rot, Vec2, rect};

fn lcg(s: &mut u64) -> f32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 40) as f32 / (1u64 << 24) as f32
}

/// A box's corners in the world, counter-clockwise.
fn corners(p: &Placed) -> [Vec2; 4] {
    let (physics2d::Shape::Box(h), q) = (p.shape, p.rot.unwrap_or_default()) else { unreachable!("boxes") };
    [Vec2::new(-h.x, -h.y), Vec2::new(h.x, -h.y), Vec2::new(h.x, h.y), Vec2::new(-h.x, h.y)].map(|v| p.at + q.rotate(v))
}

fn support(v: &[Vec2; 4], d: Vec2) -> Vec2 {
    let mut best = v[0];
    for &p in &v[1..] {
        if p.dot(d) > best.dot(d) {
            best = p;
        }
    }
    best
}

/// A point of B - A furthest along `d`.
fn support_diff(a: &[Vec2; 4], b: &[Vec2; 4], d: Vec2) -> Vec2 {
    support(b, d) - support(a, -d)
}

/// The point of segment `p q` nearest the origin, and how far along.
fn nearest_on(p: Vec2, q: Vec2) -> (Vec2, f32) {
    let e = q - p;
    let t = (-p.dot(e) / e.dot(e).max(f32::MIN_POSITIVE)).clamp(0.0, 1.0);
    (p + e * t, t)
}

/// The normal from A to B and the separation (negative: overlap), by GJK
/// and, where they overlap, EPA.
fn gjk_epa(a: &[Vec2; 4], b: &[Vec2; 4]) -> Option<(Vec2, f32)> {
    let center = |v: &[Vec2; 4]| (v[0] + v[2]) * 0.5;
    let mut v = center(b) - center(a);
    if v == Vec2::ZERO {
        v = Vec2::new(1.0, 0.0);
    }
    // A simplex of up to three points of B - A, in place, as a fast GJK
    // keeps it: allocating one a pair cost this bench most of its time.
    let mut simplex = [support_diff(a, b, v), Vec2::ZERO, Vec2::ZERO];
    let mut len = 1;
    v = simplex[0];
    for _ in 0..32 {
        let w = support_diff(a, b, -v);
        // No point of B - A is nearer the origin than `v` by more than this:
        // `v` is the nearest, and they're apart.
        if v.dot(v) - v.dot(w) <= 1e-6 * v.dot(v).max(1e-12) {
            let d = v.len();
            return (d > 1e-6).then(|| (v * (1.0 / d), d));
        }
        simplex[len] = w;
        len += 1;
        match len {
            2 => {
                let (p, t) = nearest_on(simplex[0], simplex[1]);
                if t == 0.0 {
                    len = 1;
                } else if t == 1.0 {
                    (simplex[0], len) = (simplex[1], 1);
                }
                v = p;
            }
            _ => {
                let (p0, p1, p2) = (simplex[0], simplex[1], simplex[2]);
                // The origin inside the triangle: they overlap.
                let side = |p: Vec2, q: Vec2| (q - p).cross(-p);
                let (s0, s1, s2) = (side(p0, p1), side(p1, p2), side(p2, p0));
                if (s0 >= 0.0 && s1 >= 0.0 && s2 >= 0.0) || (s0 <= 0.0 && s1 <= 0.0 && s2 <= 0.0) {
                    return Some(epa(a, b, [p0, p1, p2]));
                }
                let edges = [(p0, p1), (p1, p2), (p2, p0)];
                let (best, (p, _)) = edges
                    .iter()
                    .map(|&(p, q)| nearest_on(p, q))
                    .enumerate()
                    .min_by(|x, y| x.1.0.dot(x.1.0).total_cmp(&y.1.0.dot(y.1.0)))
                    .unwrap();
                (simplex[0], simplex[1], len) = (edges[best].0, edges[best].1, 2);
                v = p;
            }
        }
        if v.dot(v) < 1e-12 {
            // On the boundary: touching, as good as overlapping by nothing.
            return None;
        }
    }
    let d = v.len();
    Some((v * (1.0 / d), d))
}

/// The overlap's depth and normal, expanding the polygon around the origin
/// toward B - A's nearest face.
fn epa(a: &[Vec2; 4], b: &[Vec2; 4], start: [Vec2; 3]) -> (Vec2, f32) {
    // In place too: B - A of two boxes has at most eight corners.
    let mut poly = [Vec2::ZERO; 36];
    poly[..3].copy_from_slice(&start);
    if (poly[1] - poly[0]).cross(poly[2] - poly[0]) < 0.0 {
        poly.swap(1, 2);
    }
    for len in 3..poly.len() {
        let (mut best, mut dist, mut normal) = (0, f32::INFINITY, Vec2::ZERO);
        for i in 0..len {
            let (p, q) = (poly[i], poly[(i + 1) % len]);
            let e = q - p;
            let n = Vec2::new(e.y, -e.x) * (1.0 / e.len());
            let d = n.dot(p);
            if d < dist {
                (best, dist, normal) = (i, d, n);
            }
        }
        let w = support_diff(a, b, normal);
        if w.dot(normal) - dist < 1e-5 || len + 1 == poly.len() {
            return (-normal, -dist);
        }
        poly.copy_within(best + 1..len, best + 2);
        poly[best + 1] = w;
    }
    (Vec2::ZERO, 0.0)
}

/// The face of the box with corners `v` whose normal is most along `n`,
/// and that alignment.
fn face(v: &[Vec2; 4], n: Vec2) -> (usize, f32) {
    (0..4)
        .map(|i| {
            let e = v[(i + 1) % 4] - v[i];
            (i, Vec2::new(e.y, -e.x).dot(n) / e.len())
        })
        .max_by(|x, y| x.1.total_cmp(&y.1))
        .unwrap()
}

/// GJK and EPA's normal, then the faces it picks clipped against each
/// other: two points, as `collide_turned` makes them.
fn gjk_manifold(a: &Placed, b: &Placed) -> Option<(Vec2, f32, [Vec2; 2])> {
    let (ca, cb) = (corners(a), corners(b));
    let (n, sep) = gjk_epa(&ca, &cb)?;
    if sep > narrow::MARGIN {
        return None;
    }
    let ((fa, da), (fb, db)) = (face(&ca, n), face(&cb, -n));
    // The reference face is the one more nearly along the normal.
    let (r, i, reference_n) = if da >= db { (&ca, &cb, fa) } else { (&cb, &ca, fb) };
    let incident = face(i, -Vec2::new((r[(reference_n + 1) % 4] - r[reference_n]).y, -(r[(reference_n + 1) % 4] - r[reference_n]).x)).0;
    let (v11, v12) = (r[reference_n], r[(reference_n + 1) % 4]);
    let (v21, v22) = (i[incident], i[(incident + 1) % 4]);
    let t = (v12 - v11) * (1.0 / (v12 - v11).len());
    let (lo, hi) = (0.0, (v12 - v11).dot(t));
    let clip = |p: Vec2, q: Vec2| {
        let (sp, sq) = ((p - v11).dot(t), (q - v11).dot(t));
        let f = |s: f32| p + (q - p) * ((s - sp) / (sq - sp));
        let (a, b) = if sp < lo { (f(lo), q) } else { (p, q) };
        let b = if sq > hi { f(hi) } else { b };
        [a, b]
    };
    Some((n, sep, clip(v21, v22)))
}

fn main() {
    let n = 100_000;
    let mut seed = 7;
    let half = Vec2::new(0.45, 0.45);
    let mut pairs = Vec::with_capacity(n);
    while pairs.len() < n {
        let qa = Rot::from_angle(lcg(&mut seed) * std::f32::consts::TAU);
        let qb = Rot::from_angle(lcg(&mut seed) * std::f32::consts::TAU);
        let dir = Rot::from_angle(lcg(&mut seed) * std::f32::consts::TAU).rotate(Vec2::new(1.0, 0.0));
        let a = rect(Vec2::ZERO, half).turned(qa);
        // Out along `dir` until apart, then back in by up to a tenth.
        let mut d = 0.0;
        while physics2d::separation(&a, &rect(dir * d, half).turned(qb)) < 0.0 {
            d += 0.01;
        }
        let d = d - lcg(&mut seed) * 0.15 + 0.03;
        pairs.push((a, rect(dir * d, half).turned(qb)));
    }
    let reps = 5;
    let mut sat = Vec::new();
    let t = Instant::now();
    for _ in 0..reps {
        sat.clear();
        sat.extend(pairs.iter().map(|(a, b)| narrow::collide_turned(a, b)));
    }
    let sat_ns = t.elapsed().as_nanos() as f64 / (reps * n) as f64;
    let mut gjk = Vec::new();
    let t = Instant::now();
    for _ in 0..reps {
        gjk.clear();
        gjk.extend(pairs.iter().map(|(a, b)| gjk_manifold(a, b)));
    }
    let gjk_ns = t.elapsed().as_nanos() as f64 / (reps * n) as f64;
    let (mut both, mut agree, mut only_one) = (0, 0, 0);
    for (s, g) in sat.iter().zip(&gjk) {
        match (s, g) {
            (Some(s), Some(g)) => {
                both += 1;
                if s.normal.dot(g.0) > 0.9999 && (s.depth + g.1).abs() < 1e-3 {
                    agree += 1;
                }
            }
            (None, None) => {}
            _ => only_one += 1,
        }
    }
    println!("{n} pairs of turned boxes 0.9 across, from {:.2} apart to sunk {:.2}", 0.03, 0.12);
    println!();
    println!("| way | ns a pair | contacts |");
    println!("|---|---|---|");
    println!("| SAT and clipping (Box2D's) | {sat_ns:.1} | {} |", sat.iter().flatten().count());
    println!("| GJK and EPA, then clipping | {gjk_ns:.1} | {} |", gjk.iter().flatten().count());
    println!();
    println!("Where both find a contact ({both}), the same normal and depth: {agree}; found by one only: {only_one}");
}
