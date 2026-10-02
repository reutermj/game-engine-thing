//! The contact solver over arrays: bodies and contacts gathered from the
//! world, solved, written back. A body is a velocity and an angular one,
//! an inverse mass and a world inverse inertia; a contact a normal and up
//! to four points.
//!
//! The 2D solver's soft step (engine/std/physics2d, solver.rs; Box2D v3's,
//! physics.md "Settling") with rotation, as Box3D (contact_solver.c,
//! solver.c) runs it: `SUBSTEPS` substeps, each gravity, warm start, one
//! soft pushing pass, positions and rotations moved, `RELAX_ITERATIONS`
//! rigid passes; then restitution once, from each point's closing speed
//! before the step. Everything a point needs is fixed once a step: its
//! anchors on both bodies (relative to their centers), its effective mass,
//! and each body's world inverse inertia. Within the step a point's
//! separation is its separation when found plus how far its anchors have
//! moved along the normal, from each body's accumulated move and turn
//! (Box3D's `b3SolveContact`), so contacts aren't found again per substep.
//!
//! Friction is per contact, not per point, as Box3D, Rapier and Jolt all
//! do it: one tangent impulse at the points' centroid, clamped to a disc
//! of radius friction times the points' total normal impulse (a circular
//! Coulomb cone, so a body slides no faster diagonally), and a twist
//! impulse about the normal, limited by each point's normal impulse times
//! its distance from the centroid. Only in the relax passes, as in 2D and
//! in Box3D. The tangent impulse is kept as a world vector, so warm
//! starting needs no tangent basis that stays put from step to step: the
//! last step's is projected onto this step's plane.

pub use physics_common::{BOUNCE_THRESHOLD, DAMPING_RATIO, MAX_PUSH, Softness};

use crate::narrow::MAX_POINTS;
use crate::{Anchors, Carry, Closing, Inertia, Integrate, Mat3, Quat, Vec3};
/// The most a body turns in a step, as in Box3D, Rapier and Jolt (a
/// quarter turn is where a first-order rotation step goes badly wrong).
pub const MAX_ROTATION: f32 = 0.25 * std::f32::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tuning {
    pub substeps: usize,
    pub relax: usize,
    pub stiffness: f32,
    pub static_stiffness: f32,
    /// Friction in the pushing pass too (Box2D's), not only relaxing
    /// (Rapier's and Box3D's).
    pub friction_in_push: bool,
    pub integrate: Integrate,
    pub inertia: Inertia,
    pub anchors: Anchors,
    pub carry: Carry,
    pub closing: Closing,
}

impl Tuning {
    pub fn of(t: &crate::Tuning) -> Tuning {
        Tuning {
            substeps: t.substeps as usize,
            relax: t.relax as usize,
            stiffness: t.stiffness,
            static_stiffness: t.static_stiffness,
            friction_in_push: t.friction_in_push,
            integrate: t.integrate(),
            inertia: t.inertia(),
            anchors: t.anchors(),
            carry: t.carry(),
            closing: t.closing(),
        }
    }
}

impl Default for Tuning {
    fn default() -> Tuning {
        Tuning::of(&crate::Tuning::default())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SolverBody {
    pub v: Vec3,
    pub w: Vec3,
    /// 0 for static bodies.
    pub inv_mass: f32,
    /// Its own inverse inertia, about its axes: zero when it can't turn.
    pub inv_inertia: Vec3,
    /// Its rotation when the step began.
    pub q: Quat,
    /// The step's gravity, already in `v`: given back a substep at a time
    /// (see the 2D solver).
    pub gravity: Vec3,
    /// The world inverse inertia, and output: how far the body moved and
    /// turned this step.
    pub inv_i: Mat3,
    pub moved: Vec3,
    pub turned: Quat,
}

impl SolverBody {
    pub fn new(v: Vec3, w: Vec3, inv_mass: f32, inv_inertia: Vec3, q: Quat, gravity: Vec3) -> SolverBody {
        SolverBody { v, w, inv_mass, inv_inertia, q, gravity, inv_i: Mat3::ZERO, moved: Vec3::ZERO, turned: Quat::IDENTITY }
    }

    /// Where it is turned to now.
    pub fn rotation(&self) -> Quat {
        self.turned.times(self.q).normalize()
    }

    fn form_inertia(&mut self, q: Quat) {
        self.inv_i = if self.inv_inertia == Vec3::ZERO { Mat3::ZERO } else { Mat3::rotated_diagonal(&q.matrix(), self.inv_inertia) };
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactPoint {
    /// From `a`'s center to the point, in the world, when found.
    pub ra: Vec3,
    pub depth: f32,
    /// In, the last step's normal impulse here (warm starting); out, this
    /// step's.
    pub jn: f32,
    /// Output: the closing speed along the normal before solving.
    pub speed: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Constraint {
    pub a: u32,
    pub b: u32,
    pub normal: Vec3,
    /// `a`'s center less `b`'s when found: `b`'s anchors are `a`'s plus
    /// this.
    pub offset: Vec3,
    pub friction: f32,
    pub restitution: f32,
    pub count: usize,
    pub points: [ContactPoint; MAX_POINTS],
    /// Accumulated friction and twist impulses over the step, in and out
    /// as the points' normal ones. `jt` is in the tangent plane.
    pub jt: Vec3,
    pub twist: f32,
}

/// What the passes read and write of a body, in a cache line of its own:
/// the rest of `SolverBody` (its inertia, gravity, rotation) is only read
/// when the rows are made.
#[derive(Clone, Copy, Default)]
#[repr(align(64))]
struct Hot {
    v: Vec3,
    w: Vec3,
    moved: Vec3,
    inv_mass: f32,
    /// How far it has turned this step, as a rotation vector, the sum of
    /// w h: what `Anchors::Linear` reads.
    theta: Vec3,
}

/// A contact point as the substeps solve it. The crossed anchors and what
/// the inverse inertias make of them are worked out once a step, so a pass
/// is dot products and adds, no matrices.
#[derive(Clone, Copy, Default)]
struct PointRow {
    ra: Vec3,
    rb: Vec3,
    /// ra x n and rb x n; I_a (ra x n) and I_b (rb x n).
    rna: Vec3,
    rnb: Vec3,
    ia: Vec3,
    ib: Vec3,
    /// Separation when found, less the anchors' offset along the normal:
    /// the anchors' moves add to it.
    base: f32,
    mass: f32,
    jn: f32,
    /// Distance from the centroid, which twist friction acts at.
    lever: f32,
}

/// A contact's friction, at the points' centroid: the tangents crossed with
/// its anchors on each body, and what the inverse inertias make of them.
#[derive(Clone, Copy, Default)]
struct Tangents {
    ca: [Vec3; 2],
    cb: [Vec3; 2],
    ia: [Vec3; 2],
    ib: [Vec3; 2],
}

struct Row {
    a: usize,
    b: usize,
    n: Vec3,
    t: [Vec3; 2],
    /// Its points, in `points`.
    start: usize,
    count: usize,
    soft: Softness,
    friction: f32,
    tan: Tangents,
    /// The inverse of the 2x2 tangent mass (xx, xy, yy).
    tmass: [f32; 3],
    jt: [f32; 2],
    /// I_a n and I_b n, for twist.
    na: Vec3,
    nb: Vec3,
    twist_mass: f32,
    twist: f32,
}

pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
    let n_sub = how.substeps.max(1);
    let h = dt / n_sub as f32;
    let inv_h = 1.0 / h;
    let moving = Softness::new(how.stiffness * inv_h, DAMPING_RATIO, h);
    let fixed = Softness::new(how.static_stiffness * inv_h, DAMPING_RATIO, h);
    let share = 1.0 / n_sub as f32;
    for b in bodies.iter_mut() {
        b.form_inertia(b.q);
    }
    let mut points = Vec::with_capacity(contacts.len() * 2);
    let mut rows: Vec<Row> = contacts.iter_mut().map(|c| row(bodies, c, &mut points, share, (moving, fixed), (how.closing, dt))).collect();
    let mut hot: Vec<Hot> =
        bodies.iter().map(|b| Hot { v: b.v - b.gravity, w: b.w, moved: Vec3::ZERO, inv_mass: b.inv_mass, theta: Vec3::ZERO }).collect();
    // Apart from the hot bodies: only `Anchors::Exact` reads it in a pass.
    let mut turned = vec![Quat::IDENTITY; bodies.len()];
    let exact = how.anchors == Anchors::Exact;
    let max_w = MAX_ROTATION / dt;

    for sub in 0..n_sub {
        for (x, b) in hot.iter_mut().zip(bodies.iter()) {
            x.v += b.gravity * share;
            let w = x.w.len();
            if w > max_w {
                x.w = x.w * (max_w / w);
            }
        }
        for r in &rows {
            warm_start(&mut hot, r, &points);
        }
        pass(&mut hot, &turned, &mut rows, &mut points, inv_h, true, how.friction_in_push, exact);
        let last = sub + 1 == n_sub;
        for ((x, b), q) in hot.iter_mut().zip(bodies.iter_mut()).zip(turned.iter_mut()) {
            x.moved += x.v * h;
            if b.inv_inertia == Vec3::ZERO {
                continue;
            }
            x.theta += x.w * h;
            *q = match how.integrate {
                Integrate::Linear => q.integrate(x.w, h).normalize(),
                Integrate::LinearOnce => {
                    let q = q.integrate(x.w, h);
                    if last { q.normalize() } else { q }
                }
                Integrate::Exact => q.integrate_exact(x.w, h),
            };
            if how.inertia == Inertia::Substep {
                b.form_inertia(q.times(b.q).normalize());
            }
        }
        if how.inertia == Inertia::Substep {
            for r in rows.iter_mut() {
                refresh(bodies, r, &mut points);
            }
        }
        for _ in 0..how.relax {
            pass(&mut hot, &turned, &mut rows, &mut points, inv_h, false, true, exact);
        }
        for (r, c) in rows.iter().zip(contacts.iter_mut()) {
            for (p, cp) in points[r.start..r.start + r.count].iter().zip(c.points.iter_mut()) {
                cp.jn += p.jn;
            }
            c.jt += r.t[0] * r.jt[0] + r.t[1] * r.jt[1];
            c.twist += r.twist;
        }
    }

    // Restitution, once, from the closing speed before the step, for the
    // points that pushed, as in 2D.
    for (r, c) in rows.iter().zip(contacts.iter_mut()) {
        if c.restitution == 0.0 {
            continue;
        }
        for (p, cp) in points[r.start..r.start + r.count].iter_mut().zip(c.points.iter_mut()) {
            if cp.speed <= BOUNCE_THRESHOLD || cp.jn == 0.0 || p.mass == 0.0 {
                continue;
            }
            let vn = normal_speed(&hot, r, p);
            let jn = (p.jn - p.mass * (vn - c.restitution * cp.speed)).max(0.0);
            let d = jn - p.jn;
            p.jn = jn;
            cp.jn += d;
            push(&mut hot, r.a, r.b, r.n * d, p.ia * d, p.ib * d);
        }
    }
    // `Carry::Last`: the next step warm-starts from the last substep's
    // impulses, bounce included, as Box3D keeps them, scaled to a whole
    // step's (the next takes a substep's share). The default is the mean
    // the substeps summed to above: the last was measured, and stood a
    // five-high stack sooner but let piles of a thousand planks rock
    // longer (physics.md, "Still at rest").
    if how.carry == Carry::Last {
        let k = n_sub as f32;
        for (r, c) in rows.iter().zip(contacts.iter_mut()) {
            for (p, cp) in points[r.start..r.start + r.count].iter().zip(c.points.iter_mut()) {
                cp.jn = p.jn * k;
            }
            c.jt = (r.t[0] * r.jt[0] + r.t[1] * r.jt[1]) * k;
            c.twist = r.twist * k;
        }
    }
    for ((b, x), q) in bodies.iter_mut().zip(&hot).zip(turned) {
        (b.v, b.w, b.moved, b.turned) = (x.v, x.w, x.moved, q);
    }
}

/// A point's closing speed as restitution takes it (`Closing`, as 2D's
/// `Closing::speed`): from its closing speed with the step's gravity in it
/// (`c`), the share of that the gravity gave (`g`), its gap as found and
/// the step.
#[inline(always)]
fn closing(how: Closing, c: f32, g: f32, sep: f32, dt: f32) -> f32 {
    let before = c - g;
    match how {
        Closing::Stepped => c,
        Closing::Before => before,
        Closing::Half => c - 0.5 * g,
        Closing::Met if g > 0.0 && sep > 0.0 => {
            let v = before.max(0.0);
            let fall = (v * dt + 0.5 * g * dt).min(sep);
            (v * v + 2.0 * (g / dt) * fall).sqrt()
        }
        Closing::Met => before,
    }
}

/// A contact as the substeps solve it, from what was found, and its
/// impulses zeroed to be summed again.
fn row(
    bodies: &[SolverBody],
    c: &mut Constraint,
    points: &mut Vec<PointRow>,
    share: f32,
    (moving, fixed): (Softness, Softness),
    (how, dt): (Closing, f32),
) -> Row {
    let (ai, bi) = (c.a as usize, c.b as usize);
    let (a, b) = (&bodies[ai], &bodies[bi]);
    let n = c.normal;
    let t1 = n.perp();
    let t = [t1, n.cross(t1)];
    let m = a.inv_mass + b.inv_mass;
    let count = c.count.min(MAX_POINTS);
    let start = points.len();
    let mut centroid = Vec3::ZERO;
    for cp in c.points[..count].iter_mut() {
        let (ra, rb) = (cp.ra, cp.ra + c.offset);
        let (rna, rnb) = (ra.cross(n), rb.cross(n));
        let (ia, ib) = (a.inv_i.apply(rna), b.inv_i.apply(rnb));
        let k = m + ia.dot(rna) + ib.dot(rnb);
        let speed = -(b.v - a.v).dot(n) - b.w.dot(rnb) + a.w.dot(rna);
        cp.speed = closing(how, speed, -(b.gravity - a.gravity).dot(n), -cp.depth, dt);
        // The last step's impulse was over the whole step: a substep's
        // share of it is where each substep starts.
        let jn = cp.jn * share;
        let mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        points.push(PointRow { ra, rb, rna, rnb, ia, ib, base: -cp.depth - n.dot(c.offset), mass, jn, lever: 0.0 });
        cp.jn = 0.0;
        centroid += ra;
    }
    let ca = centroid * (1.0 / count.max(1) as f32);
    let cb = ca + c.offset;
    for p in points[start..].iter_mut() {
        let d = p.ra - ca;
        p.lever = (d - n * d.dot(n)).len();
    }
    let tan = Tangents {
        ca: [ca.cross(t[0]), ca.cross(t[1])],
        cb: [cb.cross(t[0]), cb.cross(t[1])],
        ia: [a.inv_i.apply(ca.cross(t[0])), a.inv_i.apply(ca.cross(t[1]))],
        ib: [b.inv_i.apply(cb.cross(t[0])), b.inv_i.apply(cb.cross(t[1]))],
    };
    let k = |i: usize, j: usize| tan.ia[i].dot(tan.ca[j]) + tan.ib[i].dot(tan.cb[j]);
    let (k11, k12, k22) = (m + k(0, 0), k(0, 1), m + k(1, 1));
    let det = k11 * k22 - k12 * k12;
    let tmass = if det > 0.0 { [k22 / det, -k12 / det, k11 / det] } else { [0.0; 3] };
    let (na, nb) = (a.inv_i.apply(n), b.inv_i.apply(n));
    let kt = na.dot(n) + nb.dot(n);
    let r = Row {
        a: ai,
        b: bi,
        n,
        t,
        start,
        count,
        soft: if a.inv_mass == 0.0 || b.inv_mass == 0.0 { fixed } else { moving },
        friction: c.friction,
        tan,
        tmass,
        jt: [c.jt.dot(t[0]) * share, c.jt.dot(t[1]) * share],
        na,
        nb,
        twist_mass: if kt > 0.0 { 1.0 / kt } else { 0.0 },
        twist: c.twist * share,
    };
    (c.jt, c.twist) = (Vec3::ZERO, 0.0);
    r
}

/// The row's angular terms again, from the bodies' inverse inertias now
/// (`Inertia::Substep`); the masses stay the step's.
fn refresh(bodies: &[SolverBody], r: &mut Row, points: &mut [PointRow]) {
    let (a, b) = (&bodies[r.a], &bodies[r.b]);
    for p in points[r.start..r.start + r.count].iter_mut() {
        (p.ia, p.ib) = (a.inv_i.apply(p.rna), b.inv_i.apply(p.rnb));
    }
    for i in 0..2 {
        (r.tan.ia[i], r.tan.ib[i]) = (a.inv_i.apply(r.tan.ca[i]), b.inv_i.apply(r.tan.cb[i]));
    }
    (r.na, r.nb) = (a.inv_i.apply(r.n), b.inv_i.apply(r.n));
}

#[inline(always)]
fn normal_speed(hot: &[Hot], r: &Row, p: &PointRow) -> f32 {
    let (a, b) = (&hot[r.a], &hot[r.b]);
    (b.v - a.v).dot(r.n) + b.w.dot(p.rnb) - a.w.dot(p.rna)
}

fn warm_start(hot: &mut [Hot], r: &Row, points: &[PointRow]) {
    for p in &points[r.start..r.start + r.count] {
        push(hot, r.a, r.b, r.n * p.jn, p.ia * p.jn, p.ib * p.jn);
    }
    let (j1, j2) = (r.jt[0], r.jt[1]);
    push(hot, r.a, r.b, r.t[0] * j1 + r.t[1] * j2, r.tan.ia[0] * j1 + r.tan.ia[1] * j2, r.tan.ib[0] * j1 + r.tan.ib[1] * j2);
    push(hot, r.a, r.b, Vec3::ZERO, r.na * r.twist, r.nb * r.twist);
}

/// One pass of sequential impulses over the contacts: soft and pushing
/// out when `push_out`, else rigid; with friction when `friction`.
#[allow(clippy::too_many_arguments)]
fn pass(
    hot: &mut [Hot],
    turned: &[Quat],
    rows: &mut [Row],
    points: &mut [PointRow],
    inv_h: f32,
    push_out: bool,
    friction: bool,
    exact: bool,
) {
    for r in rows.iter_mut() {
        let mut total = 0.0;
        let mut twisting = 0.0;
        for p in points[r.start..r.start + r.count].iter_mut() {
            if p.mass == 0.0 {
                continue;
            }
            let (a, b) = (&hot[r.a], &hot[r.b]);
            // Where the anchors are now, by each body's move and turn.
            let sep = if exact {
                p.base + (b.moved - a.moved + turned[r.b].rotate(p.rb) - turned[r.a].rotate(p.ra)).dot(r.n)
            } else {
                p.base + (b.moved - a.moved).dot(r.n) + (p.rb - p.ra).dot(r.n) + b.theta.dot(p.rnb) - a.theta.dot(p.rna)
            };
            // A gap may close this substep, and no more: speculative, in
            // either pass.
            let (bias, mass, relax) = if sep > 0.0 {
                (sep * inv_h, 1.0, 0.0)
            } else if push_out {
                ((r.soft.rate * sep).max(-MAX_PUSH), r.soft.mass, r.soft.impulse)
            } else {
                (0.0, 1.0, 0.0)
            };
            let vn = normal_speed(hot, r, p);
            let jn = (p.jn - p.mass * mass * (vn + bias) - relax * p.jn).max(0.0);
            let dj = jn - p.jn;
            p.jn = jn;
            push(hot, r.a, r.b, r.n * dj, p.ia * dj, p.ib * dj);
            total += jn;
            twisting += jn * p.lever;
        }
        if friction && r.tmass != [0.0; 3] {
            rub(hot, r, total, twisting);
        }
    }
}

/// Friction at the centroid, clamped to its disc, then twist.
fn rub(hot: &mut [Hot], r: &mut Row, total: f32, twisting: f32) {
    let (a, b) = (&hot[r.a], &hot[r.b]);
    let dv = b.v - a.v;
    let v1 = dv.dot(r.t[0]) + b.w.dot(r.tan.cb[0]) - a.w.dot(r.tan.ca[0]);
    let v2 = dv.dot(r.t[1]) + b.w.dot(r.tan.cb[1]) - a.w.dot(r.tan.ca[1]);
    let m = r.tmass;
    let mut j = [r.jt[0] - (m[0] * v1 + m[1] * v2), r.jt[1] - (m[1] * v1 + m[2] * v2)];
    let limit = r.friction * total;
    let len2 = j[0] * j[0] + j[1] * j[1];
    if len2 > limit * limit {
        let s = limit / len2.sqrt();
        j = [j[0] * s, j[1] * s];
    }
    let (d1, d2) = (j[0] - r.jt[0], j[1] - r.jt[1]);
    r.jt = j;
    push(hot, r.a, r.b, r.t[0] * d1 + r.t[1] * d2, r.tan.ia[0] * d1 + r.tan.ia[1] * d2, r.tan.ib[0] * d1 + r.tan.ib[1] * d2);

    if r.twist_mass > 0.0 {
        let wn = hot[r.b].w.dot(r.n) - hot[r.a].w.dot(r.n);
        let limit = r.friction * twisting;
        let t = (r.twist - r.twist_mass * wn).clamp(-limit, limit);
        let d = t - r.twist;
        r.twist = t;
        push(hot, r.a, r.b, Vec3::ZERO, r.na * d, r.nb * d);
    }
}

/// An impulse: `linear` on `b` and its opposite on `a`, with the angular
/// velocity it gives each (already through their inverse inertias).
#[inline(always)]
fn push(hot: &mut [Hot], a: usize, b: usize, linear: Vec3, wa: Vec3, wb: Vec3) {
    let (ma, mb) = (hot[a].inv_mass, hot[b].inv_mass);
    hot[a].v -= linear * ma;
    hot[a].w -= wa;
    hot[b].v += linear * mb;
    hot[b].w += wb;
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// A unit cube of mass 1 (inverse inertia 6 about each axis) moving at
    /// `v`, turning at `w`, with no gravity this step.
    fn cube(v: Vec3, w: Vec3) -> SolverBody {
        SolverBody::new(v, w, 1.0, Vec3::splat(6.0), Quat::IDENTITY, Vec3::ZERO)
    }

    /// Body 1 on the ground (body 0) at its four bottom corners, `depth`
    /// deep: the normal from 1 to 0 points down.
    fn on_ground(depth: f32, restitution: f32) -> Constraint {
        let mut c = Constraint { a: 1, b: 0, normal: -Vec3::Y, friction: 0.5, restitution, count: 4, ..Default::default() };
        for (p, (x, z)) in c.points.iter_mut().zip([(0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)]) {
            p.ra = Vec3::new(x, -0.5, z);
            p.depth = depth;
        }
        c
    }

    /// One point straight under its center: where an impulse turns nothing,
    /// so what a step does is exact. Four corners, solved one after
    /// another, turn the box a little until the passes converge, which
    /// takes more than a step (the tests in tests/physics3d_test.rs).
    fn under(depth: f32, restitution: f32) -> Constraint {
        let mut c = on_ground(depth, restitution);
        c.count = 1;
        c.points[0].ra = Vec3::new(0.0, -0.5, 0.0);
        c
    }

    fn solve1(bodies: &mut [SolverBody], c: &mut [Constraint]) {
        solve(bodies, c, DT, &Tuning::default());
    }

    #[test]
    fn a_landing_box_stops() {
        let mut bodies = [SolverBody::default(), cube(Vec3::new(0.0, -10.0, 0.0), Vec3::ZERO)];
        solve1(&mut bodies, &mut [under(0.0, 0.0)]);
        assert!(bodies[1].v.len() < 1e-3 && bodies[1].w.len() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn a_box_landing_on_a_corner_tips() {
        // One corner, at +x: the box turns about z, its other side falling.
        let mut c = on_ground(0.0, 0.0);
        c.count = 1;
        let mut bodies = [SolverBody::default(), cube(Vec3::new(0.0, -2.0, 0.0), Vec3::ZERO)];
        solve1(&mut bodies, &mut [c]);
        let b = bodies[1];
        // Pushed up at (0.5, -0.5, 0.5): about (-1, 0, 1), the far side
        // falling.
        let axis = Vec3::new(-1.0, 0.0, 1.0).normalize();
        assert!(b.w.dot(axis) > 1.0 && (b.w - axis * b.w.dot(axis)).len() < 1e-3, "{b:?}");
        // The corner itself has all but stopped: v + w x r along the normal
        // (friction there leaves a little).
        let r = c.points[0].ra;
        assert!((b.v + b.w.cross(r)).y.abs() < 0.2, "{b:?}");
        assert!(b.turned.v.dot(axis) > 0.0, "turned as it spins: {b:?}");
    }

    #[test]
    fn friction_is_a_disc_not_a_box() {
        // Sliding diagonally at 10 while pressed down at 1: friction can take
        // 0.5 of the speed, along the slide, not 0.5 on each axis.
        let d = 10.0 / 2f32.sqrt();
        let mut bodies = [SolverBody::default(), cube(Vec3::new(d, -1.0, d), Vec3::ZERO)];
        solve1(&mut bodies, &mut [under(0.0, 0.0)]);
        let v = bodies[1].v;
        assert!(((v.x * v.x + v.z * v.z).sqrt() - 9.5).abs() < 1e-2, "{v:?}");
        assert!((v.x - v.z).abs() < 1e-4);
    }

    #[test]
    fn warm_starting_projects_last_steps_friction_onto_this_plane() {
        let mut bodies = [SolverBody::default(), cube(Vec3::ZERO, Vec3::ZERO)];
        let mut c = [Constraint { jt: Vec3::new(0.0, 3.0, 0.0), ..on_ground(0.0, 0.0) }];
        solve1(&mut bodies, &mut c);
        // Along the normal, the last step's friction is no friction.
        assert!(c[0].jt.len() < 1e-6, "{:?}", c[0]);
    }

    #[test]
    fn penetration_is_pushed_out_at_most_max_push_and_leaves_no_speed() {
        let mut bodies = [SolverBody::default(), cube(Vec3::ZERO, Vec3::ZERO)];
        solve1(&mut bodies, &mut [under(0.2, 0.0)]);
        assert!(bodies[1].v.len() < 1e-3 && bodies[1].w.len() < 1e-3, "{:?}", bodies[1]);
        let moved = bodies[1].moved.y;
        assert!(moved > 0.9 * MAX_PUSH * DT && moved <= MAX_PUSH * DT + 1e-6, "moved {moved}");
    }

    #[test]
    fn a_bouncy_box_bounces_back_at_its_restitution() {
        let mut bodies = [SolverBody::default(), cube(Vec3::new(0.0, -10.0, 0.0), Vec3::ZERO)];
        solve1(&mut bodies, &mut [under(0.0, 0.5)]);
        assert!((bodies[1].v.y - 5.0).abs() < 1e-2 && bodies[1].w.len() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn the_world_inverse_inertia_is_the_bodys_own_turned() {
        let q = Quat::axis_angle(Vec3::new(1.0, 2.0, 0.5), 0.8);
        let mut bodies = [SolverBody::default(), SolverBody::new(Vec3::ZERO, Vec3::ZERO, 1.0, Vec3::new(1.0, 4.0, 9.0), q, Vec3::ZERO)];
        solve1(&mut bodies, &mut []);
        let v = Vec3::new(0.3, -1.0, 0.2);
        let expect = q.rotate(Vec3::new(1.0, 4.0, 9.0).times(q.conj().rotate(v)));
        assert!((bodies[1].inv_i.apply(v) - expect).len() < 1e-4, "{:?}", bodies[1].inv_i);
    }

    #[test]
    fn a_spinning_body_stays_a_rotation() {
        let mut bodies = [cube(Vec3::ZERO, Vec3::new(3.0, 20.0, -7.0))];
        for _ in 0..200 {
            bodies[0].q = bodies[0].rotation();
            solve1(&mut bodies, &mut []);
            let t = bodies[0].turned;
            assert!((t.v.dot(t.v) + t.w * t.w - 1.0).abs() < 1e-4, "{t:?}");
        }
    }

    #[test]
    fn a_body_turns_at_most_a_quarter_turn_a_step() {
        let mut bodies = [cube(Vec3::ZERO, Vec3::new(0.0, 100.0, 0.0))];
        solve1(&mut bodies, &mut []);
        assert!((bodies[0].w.y - MAX_ROTATION / DT).abs() < 1e-3, "{:?}", bodies[0].w);
    }

    #[test]
    fn twist_friction_holds_up_to_the_lever_times_the_load() {
        // Pressed down at 1 over four corners 0.707 from the centroid: twist
        // can take up to 0.5 * 0.707 of angular impulse, 6 times that of spin.
        let mut bodies = [SolverBody::default(), cube(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 20.0, 0.0))];
        solve1(&mut bodies, &mut [on_ground(0.0, 0.0)]);
        let expect = 20.0 - 6.0 * 0.5 * 0.5f32.sqrt();
        assert!((bodies[1].w.y - expect).abs() < 0.05, "{:?} not {expect}", bodies[1].w);
    }
}
