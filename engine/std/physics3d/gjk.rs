//! Box against box as Jolt finds it (ConvexShape.cpp,
//! `sCollideConvexVsConvex`): GJK for the closest points while apart, EPA
//! for the normal and depth once they overlap, then each box's face most
//! along that normal, the better aligned one the reference and the other
//! clipped to it (Jolt's `ManifoldBetweenTwoFaces`), or the single closest
//! pair of points when neither face lies along it. Kept to measure against
//! the separating axis test in narrow.rs, which is what the step uses
//! (physics.md, "Rotation in 3D"). No convex radius: Jolt shrinks its boxes
//! by one and adds it back, which rounds their edges; these stay sharp.

use crate::Reduce;
use crate::Vec3;
use crate::narrow::{MARGIN, Manifold, Point, Solid, face_contact};

/// A point of the Minkowski difference a - b, and the points of each it
/// came from.
#[derive(Clone, Copy, Debug, Default)]
struct Sv {
    w: Vec3,
    a: Vec3,
    b: Vec3,
}

fn sgn(v: f32) -> f32 {
    if v < 0.0 { -1.0 } else { 1.0 }
}

/// The corner of a box farthest along `d`.
fn corner(s: &Solid, h: Vec3, d: Vec3) -> Vec3 {
    let c = s.rot.cols;
    s.at + c[0] * (h.x * sgn(c[0].dot(d))) + c[1] * (h.y * sgn(c[1].dot(d))) + c[2] * (h.z * sgn(c[2].dot(d)))
}

struct Boxes<'a> {
    a: &'a Solid,
    ha: Vec3,
    b: &'a Solid,
    hb: Vec3,
}

impl Boxes<'_> {
    fn support(&self, d: Vec3) -> Sv {
        let (a, b) = (corner(self.a, self.ha, d), corner(self.b, self.hb, -d));
        Sv { w: a - b, a, b }
    }
}

/// The point of the simplex's hull nearest the origin, and the fewest of
/// its points whose hull holds it, with their weights: every subset tried,
/// the nearest whose weights are all positive kept (Johnson's algorithm by
/// brute force, cheap for four points).
fn nearest(s: &[Sv]) -> (Vec3, Vec<(Sv, f32)>) {
    let n = s.len();
    let mut best: Option<(f32, Vec3, Vec<(Sv, f32)>)> = None;
    for mask in 1u32..(1 << n) {
        let idx: Vec<usize> = (0..n).filter(|i| mask & (1 << i) != 0).collect();
        let Some(w) = affine(&idx.iter().map(|&i| s[i].w).collect::<Vec<_>>()) else { continue };
        if w.iter().any(|&x| x < -1e-6) {
            continue;
        }
        let p = idx.iter().zip(&w).fold(Vec3::ZERO, |acc, (&i, &x)| acc + s[i].w * x);
        let d = p.dot(p);
        if best.as_ref().is_none_or(|b| d < b.0 - 1e-12) {
            best = Some((d, p, idx.iter().zip(&w).map(|(&i, &x)| (s[i], x)).collect()));
        }
    }
    let (_, p, keep) = best.unwrap_or((0.0, s[0].w, vec![(s[0], 1.0)]));
    (p, keep)
}

/// The weights of the point of the points' affine hull nearest the origin,
/// or none if the points are degenerate.
fn affine(p: &[Vec3]) -> Option<Vec<f32>> {
    let e: Vec<Vec3> = p[1..].iter().map(|&q| q - p[0]).collect();
    let k = e.len();
    // G l = -E^T p0, G the Gram matrix of the edges.
    let g = |i: usize, j: usize| e[i].dot(e[j]);
    let r: Vec<f32> = e.iter().map(|x| -x.dot(p[0])).collect();
    let l: Vec<f32> = match k {
        0 => vec![],
        1 => {
            let d = g(0, 0);
            if d < 1e-12 {
                return None;
            }
            vec![r[0] / d]
        }
        2 => {
            let det = g(0, 0) * g(1, 1) - g(0, 1) * g(0, 1);
            if det.abs() < 1e-12 {
                return None;
            }
            vec![(r[0] * g(1, 1) - r[1] * g(0, 1)) / det, (g(0, 0) * r[1] - g(0, 1) * r[0]) / det]
        }
        _ => {
            let m = [[g(0, 0), g(0, 1), g(0, 2)], [g(1, 0), g(1, 1), g(1, 2)], [g(2, 0), g(2, 1), g(2, 2)]];
            let det3 = |m: [[f32; 3]; 3]| {
                m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                    + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
            };
            let det = det3(m);
            if det.abs() < 1e-12 {
                return None;
            }
            (0..3)
                .map(|c| {
                    let mut mc = m;
                    for (row, rv) in mc.iter_mut().zip(&r) {
                        row[c] = *rv;
                    }
                    det3(mc) / det
                })
                .collect()
        }
    };
    let mut w = vec![1.0 - l.iter().sum::<f32>()];
    w.extend(l);
    Some(w)
}

pub fn box_box(a: &Solid, ha: Vec3, b: &Solid, hb: Vec3, reduce: Reduce) -> Option<Manifold> {
    let s = Boxes { a, ha, b, hb };
    let mut v = a.at - b.at;
    if v.dot(v) < 1e-12 {
        v = Vec3::X;
    }
    let mut simplex: Vec<Sv> = Vec::with_capacity(4);
    let mut keep: Vec<(Sv, f32)> = Vec::new();
    let mut overlap = false;
    for _ in 0..32 {
        let p = s.support(-v);
        // Apart by more than the margin along v: done.
        if v.dot(p.w) > MARGIN * v.len() {
            return None;
        }
        // No nearer point of a - b along -v: v is the nearest.
        if v.dot(v) - v.dot(p.w) <= 1e-6 * v.dot(v) && !keep.is_empty() {
            break;
        }
        simplex.push(p);
        let (nv, k) = nearest(&simplex);
        simplex = k.iter().map(|(sv, _)| *sv).collect();
        keep = k;
        v = nv;
        if simplex.len() == 4 || v.dot(v) < 1e-10 {
            overlap = true;
            break;
        }
    }
    let (n, depth, pa, pb) = if overlap {
        epa(&s, simplex)?
    } else {
        let d = v.len();
        let pa = keep.iter().fold(Vec3::ZERO, |acc, (sv, w)| acc + sv.a * *w);
        let pb = keep.iter().fold(Vec3::ZERO, |acc, (sv, w)| acc + sv.b * *w);
        (-v * (1.0 / d), -d, pa, pb)
    };
    // The face of each box most along the normal: the better aligned is the
    // reference, if it lies along it at all.
    let (ra, rb) = (a.rot.cols, b.rot.cols);
    let best = |r: [Vec3; 3]| (0..3).map(|k| (r[k].dot(n).abs(), k)).fold((0.0, 0), |x, y| if y.0 > x.0 { y } else { x });
    let ((ca, i), (cb, j)) = (best(ra), best(rb));
    let (hav, hbv) = ([ha.x, ha.y, ha.z], [hb.x, hb.y, hb.z]);
    let face = if ca >= cb {
        (ca > 0.9).then(|| face_contact(a, hav, i, ra[i] * sgn(ra[i].dot(n)), b, hbv, 0, reduce)).flatten()
    } else {
        (cb > 0.9)
            .then(|| face_contact(b, hbv, j, rb[j] * -sgn(rb[j].dot(n)), a, hav, 1, reduce).map(|m| Manifold { normal: -m.normal, ..m }))
            .flatten()
    };
    // Jolt keeps the penetration axis as the normal, each clipped point's
    // depth measured along it.
    let face = face.map(|m| {
        let cos = m.normal.dot(n);
        let mut m = Manifold { normal: n, ..m };
        for p in m.points[..m.count].iter_mut() {
            p.depth /= cos;
        }
        m
    });
    face.or_else(|| {
        let mut m = Manifold { normal: n, count: 1, ..Default::default() };
        m.points[0] = Point { at: (pa + pb) * 0.5, depth, id: 1 << 14 };
        Some(m)
    })
}

/// The expanding polytope: from a simplex holding the origin, the face of
/// a - b nearest it, whose normal is the way to push `b` out of `a`. The
/// normal, the depth, and the deepest points of each box.
fn epa(s: &Boxes, mut simplex: Vec<Sv>) -> Option<(Vec3, f32, Vec3, Vec3)> {
    // Grow a touching or flat simplex into a tetrahedron.
    for d in [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z] {
        if simplex.len() == 4 {
            break;
        }
        let p = s.support(d);
        let fresh = match simplex.len() {
            0 => true,
            1 => (p.w - simplex[0].w).len() > 1e-6,
            2 => (simplex[1].w - simplex[0].w).cross(p.w - simplex[0].w).len() > 1e-6,
            _ => (simplex[1].w - simplex[0].w).cross(simplex[2].w - simplex[0].w).dot(p.w - simplex[0].w).abs() > 1e-9,
        };
        if fresh {
            simplex.push(p);
        }
    }
    if simplex.len() < 4 {
        return None;
    }
    let mut verts = simplex;
    let mut faces: Vec<[usize; 3]> = vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]];
    let plane = |verts: &[Sv], f: [usize; 3]| {
        let (a, b, c) = (verts[f[0]].w, verts[f[1]].w, verts[f[2]].w);
        let n = (b - a).cross(c - a).normalize();
        (n, n.dot(a))
    };
    // Wound so every normal points out of the tetrahedron: away from its
    // center, not the origin, which may lie on a face when they touch.
    let center = verts.iter().fold(Vec3::ZERO, |c, v| c + v.w) * 0.25;
    for f in faces.iter_mut() {
        let (n, d) = plane(&verts, *f);
        if n.dot(center) > d {
            f.swap(1, 2);
        }
    }
    for _ in 0..64 {
        let (k, n, d) = faces
            .iter()
            .enumerate()
            .map(|(k, &f)| {
                let (n, d) = plane(&verts, f);
                // A face with no area has no normal: never the nearest.
                (k, n, if n == Vec3::ZERO { f32::MAX } else { d })
            })
            .fold((usize::MAX, Vec3::ZERO, f32::MAX), |x, y| if y.2 < x.2 { y } else { x });
        if k == usize::MAX {
            return None;
        }
        let p = s.support(n);
        if p.w.dot(n) - d < 1e-4 {
            // The origin projected on the face, in its barycentric weights.
            let f = faces[k];
            let o = n * d;
            let (a, b, c) = (verts[f[0]], verts[f[1]], verts[f[2]]);
            let area = |x: Vec3, y: Vec3, z: Vec3| (y - x).cross(z - x).dot(n);
            let total = area(a.w, b.w, c.w);
            let (u, v) = (area(o, b.w, c.w) / total, area(a.w, o, c.w) / total);
            let w = 1.0 - u - v;
            return Some((n, d, a.a * u + b.a * v + c.a * w, a.b * u + b.b * v + c.b * w));
        }
        // Every face the new point sees goes; the edges around them, the
        // horizon, make new faces with it.
        let mut horizon: Vec<(usize, usize)> = Vec::new();
        faces.retain(|&f| {
            let (fnorm, fd) = plane(&verts, f);
            if fnorm.dot(p.w) - fd > 0.0 {
                for e in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
                    if let Some(at) = horizon.iter().position(|&h| h == (e.1, e.0)) {
                        horizon.swap_remove(at);
                    } else {
                        horizon.push(e);
                    }
                }
                false
            } else {
                true
            }
        });
        verts.push(p);
        let new = verts.len() - 1;
        faces.extend(horizon.iter().map(|&(x, y)| [x, y, new]));
    }
    None
}
