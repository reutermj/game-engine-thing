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
//!
//! The passes run four contacts at a time (`lanes`), grouped by level of
//! the sweep in pair order, so the result is the loop one contact at a
//! time in pair order (`one_at_a_time`) bit for bit, under every `Tuning`;
//! a step whose levels would leave the batches under half full runs that
//! loop instead. Physics.md, "The solver in lanes".

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
    /// Contacts solved this many at a time (`lanes`; `Lanes::width`), 1,
    /// 4 or 8, by level of the sweep in pair order, so bit for bit 0: one
    /// at a time in pair order (`one_at_a_time`), the reference the lanes
    /// are held to.
    pub lanes: usize,
    /// Where the lanes' batches would be under half full, the contacts
    /// one at a time instead (`lanes::solve`): the same result, sooner.
    /// Off only to hold the lanes themselves to the loop on few contacts.
    pub sparse_alone: bool,
}

impl Tuning {
    pub fn of(t: &crate::Tuning) -> Tuning {
        Tuning {
            lanes: t.lanes().width(),
            sparse_alone: true,
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

/// The step's solve, at `how.lanes`: the same result whichever, to the
/// bit.
pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
    match how.lanes {
        8 => lanes::solve::<8>(bodies, contacts, dt, how),
        4 => lanes::solve::<4>(bodies, contacts, dt, how),
        1 => lanes::solve::<1>(bodies, contacts, dt, how),
        _ => one_at_a_time(bodies, contacts, dt, how),
    }
}

/// The solve one contact at a time in pair order: the reference the
/// lanes are held to, and where they fall back to (`lanes::solve`).
pub fn one_at_a_time(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
    let Begun { n_sub, h, inv_h, share, max_w, soft, mut hot } = begin(bodies, dt, how);
    let mut points = Vec::with_capacity(contacts.len() * 2);
    let mut rows: Vec<Row> = contacts
        .iter_mut()
        .map(|c| {
            let mut ps = [PointRow::default(); MAX_POINTS];
            let r = Row { start: points.len(), ..row(bodies, c, &mut ps, share, soft, (how.closing, dt)) };
            points.extend_from_slice(&ps[..r.count]);
            r
        })
        .collect();
    // Apart from the hot bodies: only `Anchors::Exact` reads it in a pass.
    let mut turned = vec![Quat::IDENTITY; bodies.len()];
    let exact = how.anchors == Anchors::Exact;

    for sub in 0..n_sub {
        give_gravity(&mut hot, bodies, share, max_w);
        for r in &rows {
            warm_start(&mut hot, r, &points);
        }
        pass(&mut hot, &turned, &mut rows, &mut points, inv_h, true, how.friction_in_push, exact);
        move_bodies(&mut hot, bodies, &mut turned, how, h, sub + 1 == n_sub);
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

/// The step's start, the same whether contacts go one at a time or in
/// lanes: its constants, the bodies' world inverse inertias, and the
/// bodies as the passes take them. The contacts' rows are each way's
/// (`row`).
struct Begun {
    n_sub: usize,
    h: f32,
    inv_h: f32,
    share: f32,
    max_w: f32,
    soft: (Softness, Softness),
    hot: Vec<Hot>,
}

#[inline(always)]
fn begin(bodies: &mut [SolverBody], dt: f32, how: &Tuning) -> Begun {
    let n_sub = how.substeps.max(1);
    let h = dt / n_sub as f32;
    let inv_h = 1.0 / h;
    let moving = Softness::new(how.stiffness * inv_h, DAMPING_RATIO, h);
    let fixed = Softness::new(how.static_stiffness * inv_h, DAMPING_RATIO, h);
    let share = 1.0 / n_sub as f32;
    for b in bodies.iter_mut() {
        b.form_inertia(b.q);
    }
    let hot: Vec<Hot> =
        bodies.iter().map(|b| Hot { v: b.v - b.gravity, w: b.w, moved: Vec3::ZERO, inv_mass: b.inv_mass, theta: Vec3::ZERO }).collect();
    Begun { n_sub, h, inv_h, share, max_w: MAX_ROTATION / dt, soft: (moving, fixed), hot }
}

/// A substep's share of the step's gravity given back, and each body's
/// turn rate capped (`MAX_ROTATION`): the substep's first stage, over
/// bodies, the same whether contacts go one at a time or in lanes.
#[inline(always)]
fn give_gravity(hot: &mut [Hot], bodies: &[SolverBody], share: f32, max_w: f32) {
    for (x, b) in hot.iter_mut().zip(bodies.iter()) {
        x.v += b.gravity * share;
        let w = x.w.len();
        if w > max_w {
            x.w = x.w * (max_w / w);
        }
    }
}

/// Each body moved and turned by its velocities for the substep, and
/// under `Inertia::Substep` its world inverse inertia turned with it.
#[inline(always)]
fn move_bodies(hot: &mut [Hot], bodies: &mut [SolverBody], turned: &mut [Quat], how: &Tuning, h: f32, last: bool) {
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
}

/// A point's closing speed as restitution takes it (`Closing`, 2D's
/// `Closing::speed`): from its closing speed with the step's gravity in it
/// (`c`), the share of that the gravity gave (`g`), its gap as found and
/// the step. The interface's `Closing` is the four options `Tuning` can
/// name, without 2D's `Less` and `Gate`; it stays in the interface, which
/// can't depend on physics_common, so it is mapped here. None of the four
/// reads the restitution, so it is passed as 0.
#[inline(always)]
fn closing(how: Closing, c: f32, g: f32, sep: f32, dt: f32) -> f32 {
    let how = match how {
        Closing::Stepped => physics_common::Closing::Stepped,
        Closing::Before => physics_common::Closing::Before,
        Closing::Half => physics_common::Closing::Half,
        Closing::Met => physics_common::Closing::Met,
    };
    how.speed(c, g, sep, dt, 0.0)
}

/// A contact as the substeps solve it, from what was found, its points
/// into `points` (its row's `start` left for the caller), and its
/// impulses zeroed to be summed again.
#[inline(always)]
fn row(
    bodies: &[SolverBody],
    c: &mut Constraint,
    points: &mut [PointRow; MAX_POINTS],
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
    let mut centroid = Vec3::ZERO;
    for (cp, p) in c.points[..count].iter_mut().zip(points.iter_mut()) {
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
        *p = PointRow { ra, rb, rna, rnb, ia, ib, base: -cp.depth - n.dot(c.offset), mass, jn, lever: 0.0 };
        cp.jn = 0.0;
        centroid += ra;
    }
    let ca = centroid * (1.0 / count.max(1) as f32);
    let cb = ca + c.offset;
    for p in points[..count].iter_mut() {
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
        start: 0,
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

/// The solve `N` contacts at a time, in lanes of `physics_common::lanes::F`
/// (physics.md, "The solver in lanes"), as Box2D v3's wide solver lays it
/// out (`contact_solver.c`'s `b2ContactConstraintSIMD`, Erin Catto's;
/// docs/CREDITS.md): contacts field by field in batches, bodies gathered
/// into lanes and scattered back. Grouped by level of the sweep in pair
/// order (`physics_common::levels`), so each lane sees its bodies as the
/// sweep would have left them, and each lane's arithmetic is the scalar
/// code's, operation for operation: `one_at_a_time` bit for bit, under
/// every `Tuning` (`exact_test`'s equivalence test holds it to that).
///
/// Where the scalar code skips (a point that isn't there or has no mass,
/// a contact with no friction or no twist, a point that doesn't bounce),
/// a lane computes and its result is left out by a select, never by a
/// zero impulse: a zero pushed still turns a velocity's `-0.0` into
/// `0.0`, which the sweep wouldn't have.
mod lanes {
    use super::*;
    use engine_ecs::shape::UNSOLVED;
    use physics_common::lanes::F;

    /// `Vec3`, lane by lane: each operation spelled as `Vec3`'s is, so
    /// each lane is it to the bit (Rust neither reassociates nor
    /// contracts `f32`).
    #[derive(Clone, Copy)]
    struct V<const N: usize> {
        x: F<N>,
        y: F<N>,
        z: F<N>,
    }

    impl<const N: usize> V<N> {
        const ZERO: V<N> = V { x: F::ZERO, y: F::ZERO, z: F::ZERO };

        #[inline(always)]
        fn dot(self, o: V<N>) -> F<N> {
            self.x * o.x + self.y * o.y + self.z * o.z
        }

        #[inline(always)]
        fn cross(self, o: V<N>) -> V<N> {
            V { x: self.y * o.z - self.z * o.y, y: self.z * o.x - self.x * o.z, z: self.x * o.y - self.y * o.x }
        }

        /// `Vec3 * f32`.
        #[inline(always)]
        fn scale(self, s: F<N>) -> V<N> {
            V { x: self.x * s, y: self.y * s, z: self.z * s }
        }

        #[inline(always)]
        fn get(&self, l: usize) -> Vec3 {
            Vec3::new(self.x.0[l], self.y.0[l], self.z.0[l])
        }

        #[inline(always)]
        fn set(&mut self, l: usize, v: Vec3) {
            (self.x.0[l], self.y.0[l], self.z.0[l]) = (v.x, v.y, v.z);
        }

        #[inline(always)]
        fn select(on: &[bool; N], a: V<N>, b: V<N>) -> V<N> {
            V { x: sel(on, a.x, b.x), y: sel(on, a.y, b.y), z: sel(on, a.z, b.z) }
        }
    }

    impl<const N: usize> std::ops::Add for V<N> {
        type Output = V<N>;
        #[inline(always)]
        fn add(self, o: V<N>) -> V<N> {
            V { x: self.x + o.x, y: self.y + o.y, z: self.z + o.z }
        }
    }

    impl<const N: usize> std::ops::Sub for V<N> {
        type Output = V<N>;
        #[inline(always)]
        fn sub(self, o: V<N>) -> V<N> {
            V { x: self.x - o.x, y: self.y - o.y, z: self.z - o.z }
        }
    }

    /// `a` in the lanes `on`, else `b`.
    #[inline(always)]
    fn sel<const N: usize>(on: &[bool; N], a: F<N>, b: F<N>) -> F<N> {
        let mut r = b.0;
        for l in 0..N {
            if on[l] {
                r[l] = a.0[l];
            }
        }
        F(r)
    }

    #[inline(always)]
    fn all<const N: usize>(on: &[bool; N]) -> bool {
        on.iter().all(|x| *x)
    }

    #[inline(always)]
    fn any<const N: usize>(on: &[bool; N]) -> bool {
        on.iter().any(|x| *x)
    }

    #[inline(always)]
    fn mask<const N: usize>(f: impl Fn(usize) -> bool) -> [bool; N] {
        std::array::from_fn(f)
    }

    /// `f32::max` itself, lane by lane, not `F::max`'s select: the two
    /// may differ on `max(-0.0, 0.0)` and on a NaN, and the scalar code
    /// calls this one.
    #[inline(always)]
    fn max<const N: usize>(a: F<N>, b: F<N>) -> F<N> {
        F(std::array::from_fn(|l| a.0[l].max(b.0[l])))
    }

    #[inline(always)]
    fn div<const N: usize>(a: F<N>, b: F<N>) -> F<N> {
        F(std::array::from_fn(|l| a.0[l] / b.0[l]))
    }

    #[inline(always)]
    fn sqrt<const N: usize>(a: F<N>) -> F<N> {
        F(a.0.map(f32::sqrt))
    }

    /// `Quat::rotate`, lane by lane: `v` the rotations' vector parts, `w`
    /// their scalars.
    #[inline(always)]
    fn rotate<const N: usize>((v, w): (V<N>, F<N>), p: V<N>) -> V<N> {
        let t = v.cross(p).scale(F::splat(2.0));
        p + t.scale(w) + v.cross(t)
    }

    /// A point of each lane's contact: `PointRow`, lane by lane.
    #[derive(Clone, Copy)]
    struct Pt<const N: usize> {
        rna: V<N>,
        rnb: V<N>,
        ia: V<N>,
        ib: V<N>,
        base: F<N>,
        /// `(rb - ra) . n`, which `Anchors::Linear`'s separation adds.
        arm: F<N>,
        mass: F<N>,
        jn: F<N>,
        lever: F<N>,
        /// The relaxing passes' bias, which the substep's first finds and
        /// the others read (`STORE`, `LOAD`).
        bias: F<N>,
        /// The substeps' impulses summed, as `ContactPoint::jn` carries
        /// them, and the closing speed before the step.
        sum_jn: F<N>,
        speed: F<N>,
        /// The lanes whose contact has this point, and of those the ones
        /// with mass, which the passes solve.
        on: [bool; N],
        solid: [bool; N],
    }

    impl<const N: usize> Pt<N> {
        const EMPTY: Pt<N> = Pt {
            rna: V::ZERO,
            rnb: V::ZERO,
            ia: V::ZERO,
            ib: V::ZERO,
            base: F::ZERO,
            arm: F::ZERO,
            mass: F::ZERO,
            jn: F::ZERO,
            lever: F::ZERO,
            bias: F::ZERO,
            sum_jn: F::ZERO,
            speed: F::ZERO,
            on: [false; N],
            solid: [false; N],
        };
    }

    /// `N` contacts, field by field: `Row`, lane by lane, its points
    /// `pts[start..start + points]`, as many as its lanes' most. A lane
    /// with no contact (the last of a level) points both ends at a body
    /// past the real ones, whose writes nothing reads.
    #[derive(Clone, Copy)]
    struct Batch<const N: usize> {
        a: [u32; N],
        b: [u32; N],
        ma: F<N>,
        mb: F<N>,
        n: V<N>,
        t: [V<N>; 2],
        rate: F<N>,
        soft_mass: F<N>,
        soft_impulse: F<N>,
        friction: F<N>,
        restitution: F<N>,
        /// `Tangents`.
        tca: [V<N>; 2],
        tcb: [V<N>; 2],
        tia: [V<N>; 2],
        tib: [V<N>; 2],
        tmass: [F<N>; 3],
        jt: [F<N>; 2],
        na: V<N>,
        nb: V<N>,
        twist_mass: F<N>,
        twist: F<N>,
        /// The substeps' friction and twist summed, as `Constraint::jt`
        /// and `twist` carry them.
        sum_jt: V<N>,
        sum_twist: F<N>,
        /// The lanes with a contact; of those, the ones that rub (a
        /// tangent mass) and twist (a twist mass), as `pass` and `rub`
        /// test them.
        real: [bool; N],
        rubs: [bool; N],
        twists: [bool; N],
        /// Whether any lane can bounce.
        bounces: bool,
        start: usize,
        points: usize,
    }

    impl<const N: usize> Batch<N> {
        fn empty(nowhere: u32) -> Batch<N> {
            Batch {
                a: [nowhere; N],
                b: [nowhere; N],
                ma: F::ZERO,
                mb: F::ZERO,
                n: V::ZERO,
                t: [V::ZERO; 2],
                rate: F::ZERO,
                soft_mass: F::ZERO,
                soft_impulse: F::ZERO,
                friction: F::ZERO,
                restitution: F::ZERO,
                tca: [V::ZERO; 2],
                tcb: [V::ZERO; 2],
                tia: [V::ZERO; 2],
                tib: [V::ZERO; 2],
                tmass: [F::ZERO; 3],
                jt: [F::ZERO; 2],
                na: V::ZERO,
                nb: V::ZERO,
                twist_mass: F::ZERO,
                twist: F::ZERO,
                sum_jt: V::ZERO,
                sum_twist: F::ZERO,
                real: [false; N],
                rubs: [false; N],
                twists: [false; N],
                bounces: false,
                start: 0,
                points: 0,
            }
        }
    }

    /// A batch's bodies' velocities and turn rates, at each end.
    #[derive(Clone, Copy)]
    struct Ends<const N: usize> {
        va: V<N>,
        wa: V<N>,
        vb: V<N>,
        wb: V<N>,
    }

    impl<const N: usize> Ends<N> {
        #[inline(always)]
        fn gather(hot: &[Hot], o: &Batch<N>) -> Ends<N> {
            let mut e = Ends { va: V::ZERO, wa: V::ZERO, vb: V::ZERO, wb: V::ZERO };
            for l in 0..N {
                let (a, b) = (&hot[o.a[l] as usize], &hot[o.b[l] as usize]);
                e.va.set(l, a.v);
                e.wa.set(l, a.w);
                e.vb.set(l, b.v);
                e.wb.set(l, b.w);
            }
            e
        }

        /// All of `a`'s lanes, then `b`'s: no two lanes move one body, and
        /// a body that doesn't move is written back as it was read
        /// (`shareable`), so the order is no one's business.
        #[inline(always)]
        fn scatter(&self, hot: &mut [Hot], o: &Batch<N>) {
            for l in 0..N {
                let x = &mut hot[o.a[l] as usize];
                (x.v, x.w) = (self.va.get(l), self.wa.get(l));
            }
            for l in 0..N {
                let x = &mut hot[o.b[l] as usize];
                (x.v, x.w) = (self.vb.get(l), self.wb.get(l));
            }
        }

        /// `push`, lane by lane.
        #[inline(always)]
        fn push(&mut self, o: &Batch<N>, linear: V<N>, wa: V<N>, wb: V<N>) {
            self.va = self.va - linear.scale(o.ma);
            self.wa = self.wa - wa;
            self.vb = self.vb + linear.scale(o.mb);
            self.wb = self.wb + wb;
        }

        /// `push` in the lanes `on` alone.
        #[inline(always)]
        fn push_on(&mut self, on: &[bool; N], o: &Batch<N>, linear: V<N>, wa: V<N>, wb: V<N>) {
            if all(on) {
                return self.push(o, linear, wa, wb);
            }
            let mut e = *self;
            e.push(o, linear, wa, wb);
            self.va = V::select(on, e.va, self.va);
            self.wa = V::select(on, e.wa, self.wa);
            self.vb = V::select(on, e.vb, self.vb);
            self.wb = V::select(on, e.wb, self.wb);
        }
    }

    /// A field of each lane's body at `at` (`Hot::moved`, `theta`).
    #[inline(always)]
    fn field<const N: usize>(hot: &[Hot], at: &[u32; N], f: impl Fn(&Hot) -> Vec3) -> V<N> {
        let mut v = V::ZERO;
        for l in 0..N {
            v.set(l, f(&hot[at[l] as usize]));
        }
        v
    }

    #[inline(always)]
    fn quats<const N: usize>(turned: &[Quat], at: &[u32; N]) -> (V<N>, F<N>) {
        let (mut v, mut w) = (V::ZERO, F::ZERO);
        for l in 0..N {
            let q = turned[at[l] as usize];
            v.set(l, q.v);
            w.0[l] = q.w;
        }
        (v, w)
    }

    /// Whether the lanes can be the sweep: every body that doesn't move
    /// has velocities with no `-0.0` and nothing infinite or NaN, and so
    /// does its gravity. A batch's lanes each write such a body back as
    /// they read it, which is what the sweep leaves it as only then: a
    /// zero impulse on it turns a `-0.0` into `0.0`, and the sweep would
    /// read it changed by the contacts before, the lanes not (2D's
    /// `shareable` is the same rule for its threads). Moving bodies are
    /// the levels' business.
    fn shareable(bodies: &[SolverBody], moves: &[bool]) -> bool {
        let plain = |x: f32| x.is_finite() && !(x == 0.0 && x.is_sign_negative());
        let plain3 = |v: Vec3| plain(v.x) && plain(v.y) && plain(v.z);
        bodies.iter().zip(moves).all(|(b, m)| *m || (plain3(b.v) && plain3(b.w) && plain3(b.gravity)))
    }

    pub fn solve<const N: usize>(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
        // What a push changes: the velocity through the inverse mass, the
        // turn rate through the world inverse inertia, zero exactly when
        // the body's own is (`form_inertia`).
        let moves: Vec<bool> = bodies.iter().map(|b| b.inv_mass != 0.0 || b.inv_inertia != Vec3::ZERO).collect();
        if !shareable(bodies, &moves) {
            return one_at_a_time(bodies, contacts, dt, how);
        }
        let mut levels = physics_common::levels(contacts.len(), |i| (contacts[i].a, contacts[i].b), &moves);
        // A contact neither end of which moves still sums its impulses as
        // the sweep does; it changes no body, so any level will do.
        let still = levels.of.iter().filter(|k| **k == UNSOLVED).count();
        if still > 0 {
            levels.of.iter_mut().filter(|k| **k == UNSOLVED).for_each(|k| *k = 0);
            if levels.count.is_empty() {
                levels.count.push(0);
            }
            levels.count[0] += still;
        }
        let mut place = Vec::new();
        let layout = levels.pack(N, &mut place);
        // Lanes only where they pay: a batch costs about two and a half
        // contacts solved one at a time, so batches under half full (a
        // stack, a chain, a handful of contacts: a level a contact) are
        // solved one at a time, which is the same result (physics.md, "The
        // solver in lanes").
        if how.sparse_alone && contacts.len() * 2 < layout.items() * N {
            return one_at_a_time(bodies, contacts, dt, how);
        }
        let Begun { n_sub, h, inv_h, share, max_w, soft, mut hot } = begin(bodies, dt, how);
        let exact = how.anchors == Anchors::Exact;
        // One more body than there are, standing still: where a batch's
        // empty lanes point.
        let nowhere = bodies.len() as u32;
        hot.push(Hot::default());
        let mut turned = vec![Quat::IDENTITY; bodies.len() + 1];
        let mut out = vec![Batch::<N>::empty(nowhere); layout.items()];
        for (c, at) in contacts.iter().zip(&place) {
            let (batch, _) = at.expect("every contact has a level");
            let o = &mut out[batch as usize];
            o.points = o.points.max(c.count.min(MAX_POINTS));
        }
        let mut start = 0;
        for o in out.iter_mut() {
            o.start = start;
            start += o.points;
        }
        let mut pts = vec![Pt::<N>::EMPTY; start];
        // Each point's anchors, which only `Anchors::Exact` reads.
        let mut anchors = vec![[V::<N>::ZERO; 2]; if exact { start } else { 0 }];
        // Each contact's row made in pair order, as the contacts are
        // stored, and written straight into its lane: no rows kept.
        let mut ps = [PointRow::default(); MAX_POINTS];
        for (c, at) in contacts.iter_mut().zip(&place) {
            let r = row(bodies, c, &mut ps, share, soft, (how.closing, dt));
            let (batch, l) = at.expect("every contact has a level");
            let (o, l) = (&mut out[batch as usize], l as usize);
            put(o, l, &r, c, bodies);
            for (k, p) in ps[..r.count].iter().enumerate() {
                let q = &mut pts[o.start + k];
                q.rna.set(l, p.rna);
                q.rnb.set(l, p.rnb);
                q.ia.set(l, p.ia);
                q.ib.set(l, p.ib);
                (q.base.0[l], q.arm.0[l], q.mass.0[l], q.jn.0[l]) = (p.base, (p.rb - p.ra).dot(r.n), p.mass, p.jn);
                (q.lever.0[l], q.speed.0[l]) = (p.lever, c.points[k].speed);
                (q.on[l], q.solid[l]) = (true, p.mass != 0.0);
                if exact {
                    anchors[o.start + k][0].set(l, p.ra);
                    anchors[o.start + k][1].set(l, p.rb);
                }
            }
        }

        for sub in 0..n_sub {
            give_gravity(&mut hot, bodies, share, max_w);
            for o in out.iter() {
                warm_start(o, &pts, &mut hot);
            }
            let s = (&mut pts[..], &anchors[..], &mut hot[..], &turned[..], inv_h);
            match (how.friction_in_push, exact) {
                (false, false) => passes::<N, true, false, false, COMPUTE>(&mut out, s),
                (false, true) => passes::<N, true, false, true, COMPUTE>(&mut out, s),
                (true, false) => passes::<N, true, true, false, COMPUTE>(&mut out, s),
                (true, true) => passes::<N, true, true, true, COMPUTE>(&mut out, s),
            }
            move_bodies(&mut hot, bodies, &mut turned, how, h, sub + 1 == n_sub);
            if how.inertia == Inertia::Substep {
                for o in out.iter_mut() {
                    refresh(o, &mut pts, bodies);
                }
            }
            for r in 0..how.relax {
                // Bodies don't move between relaxing passes, so neither do
                // separations: the first finds each point's bias, the
                // others read it, the same bits.
                let s = (&mut pts[..], &anchors[..], &mut hot[..], &turned[..], inv_h);
                match (r == 0, exact) {
                    (true, false) => passes::<N, false, true, false, STORE>(&mut out, s),
                    (true, true) => passes::<N, false, true, true, STORE>(&mut out, s),
                    (false, _) => passes::<N, false, true, false, LOAD>(&mut out, s),
                }
            }
            for o in out.iter_mut() {
                for p in pts[o.start..o.start + o.points].iter_mut() {
                    p.sum_jn = p.sum_jn + p.jn;
                }
                o.sum_jt = o.sum_jt + (o.t[0].scale(o.jt[0]) + o.t[1].scale(o.jt[1]));
                o.sum_twist = o.sum_twist + o.twist;
            }
        }
        for o in out.iter() {
            restitute(o, &mut pts, &mut hot);
        }

        // As `one_at_a_time` leaves them, `Carry::Last` included.
        let (last, k) = (how.carry == Carry::Last, n_sub as f32);
        for (c, at) in contacts.iter_mut().zip(&place) {
            let (batch, l) = at.expect("every contact has a level");
            let (o, l) = (&out[batch as usize], l as usize);
            let count = c.count.min(MAX_POINTS);
            for (j, cp) in c.points[..count].iter_mut().enumerate() {
                let p = &pts[o.start + j];
                cp.jn = if last { p.jn.0[l] * k } else { p.sum_jn.0[l] };
            }
            if last {
                c.jt = (o.t[0].get(l) * o.jt[0].0[l] + o.t[1].get(l) * o.jt[1].0[l]) * k;
                c.twist = o.twist.0[l] * k;
            } else {
                (c.jt, c.twist) = (o.sum_jt.get(l), o.sum_twist.0[l]);
            }
        }
        for ((b, x), q) in bodies.iter_mut().zip(&hot).zip(turned) {
            (b.v, b.w, b.moved, b.turned) = (x.v, x.w, x.moved, q);
        }
    }

    /// A contact's row into lane `l` of its batch, its points apart.
    #[inline(always)]
    fn put<const N: usize>(o: &mut Batch<N>, l: usize, r: &Row, c: &Constraint, bodies: &[SolverBody]) {
        (o.a[l], o.b[l]) = (r.a as u32, r.b as u32);
        (o.ma.0[l], o.mb.0[l]) = (bodies[r.a].inv_mass, bodies[r.b].inv_mass);
        o.n.set(l, r.n);
        o.t[0].set(l, r.t[0]);
        o.t[1].set(l, r.t[1]);
        (o.rate.0[l], o.soft_mass.0[l], o.soft_impulse.0[l]) = (r.soft.rate, r.soft.mass, r.soft.impulse);
        (o.friction.0[l], o.restitution.0[l]) = (r.friction, c.restitution);
        for i in 0..2 {
            o.tca[i].set(l, r.tan.ca[i]);
            o.tcb[i].set(l, r.tan.cb[i]);
            o.tia[i].set(l, r.tan.ia[i]);
            o.tib[i].set(l, r.tan.ib[i]);
            o.tmass[i].0[l] = r.tmass[i];
            o.jt[i].0[l] = r.jt[i];
        }
        o.tmass[2].0[l] = r.tmass[2];
        o.na.set(l, r.na);
        o.nb.set(l, r.nb);
        (o.twist_mass.0[l], o.twist.0[l]) = (r.twist_mass, r.twist);
        (o.real[l], o.rubs[l], o.twists[l]) = (true, r.tmass != [0.0; 3], r.twist_mass > 0.0);
        o.bounces |= c.restitution != 0.0;
    }

    /// `warm_start`, lane by lane.
    #[inline(always)]
    fn warm_start<const N: usize>(o: &Batch<N>, pts: &[Pt<N>], hot: &mut [Hot]) {
        let mut e = Ends::gather(hot, o);
        for p in &pts[o.start..o.start + o.points] {
            e.push_on(&p.on, o, o.n.scale(p.jn), p.ia.scale(p.jn), p.ib.scale(p.jn));
        }
        let (j1, j2) = (o.jt[0], o.jt[1]);
        e.push(o, o.t[0].scale(j1) + o.t[1].scale(j2), o.tia[0].scale(j1) + o.tia[1].scale(j2), o.tib[0].scale(j1) + o.tib[1].scale(j2));
        e.push(o, V::ZERO, o.na.scale(o.twist), o.nb.scale(o.twist));
        e.scatter(hot, o);
    }

    /// How a pass finds a point's bias (`pass`'s `SEP`): from its
    /// separation, storing it too, or as the last pass stored it.
    const COMPUTE: u8 = 0;
    const STORE: u8 = 1;
    const LOAD: u8 = 2;

    type Solving<'a, const N: usize> = (&'a mut [Pt<N>], &'a [[V<N>; 2]], &'a mut [Hot], &'a [Quat], f32);

    /// A pass over every batch, in order: the levels in turn.
    #[inline(always)]
    fn passes<const N: usize, const PUSH: bool, const FRICTION: bool, const EXACT: bool, const SEP: u8>(
        out: &mut [Batch<N>],
        (pts, anchors, hot, turned, inv_h): Solving<'_, N>,
    ) {
        for o in out.iter_mut() {
            pass::<N, PUSH, FRICTION, EXACT, SEP>(o, pts, anchors, hot, turned, inv_h);
        }
    }

    /// `pass`, lane by lane: soft and pushing out when `PUSH`, else rigid;
    /// with friction when `FRICTION`; separations by `Anchors::Exact` when
    /// `EXACT`, else `Anchors::Linear`, or none read when `SEP` is `LOAD`.
    #[inline(always)]
    fn pass<const N: usize, const PUSH: bool, const FRICTION: bool, const EXACT: bool, const SEP: u8>(
        o: &mut Batch<N>,
        pts: &mut [Pt<N>],
        anchors: &[[V<N>; 2]],
        hot: &mut [Hot],
        turned: &[Quat],
        inv_h: f32,
    ) {
        let mut e = Ends::gather(hot, o);
        // Only what this pass's separation reads.
        let dmoved = if SEP == LOAD { V::ZERO } else { field(hot, &o.b, |x| x.moved) - field(hot, &o.a, |x| x.moved) };
        let (theta_a, theta_b) =
            if SEP == LOAD || EXACT { (V::ZERO, V::ZERO) } else { (field(hot, &o.a, |x| x.theta), field(hot, &o.b, |x| x.theta)) };
        let (qa, qb) =
            if EXACT && SEP != LOAD { (quats(turned, &o.a), quats(turned, &o.b)) } else { ((V::ZERO, F::ZERO), (V::ZERO, F::ZERO)) };
        let (n, one) = (o.n, F::splat(1.0));
        let (mut total, mut twisting) = (F::ZERO, F::ZERO);
        for k in 0..o.points {
            let i = o.start + k;
            let p = &mut pts[i];
            if !any(&p.solid) {
                continue;
            }
            let (bias, mass, relax) = if SEP == LOAD {
                (p.bias, one, F::ZERO)
            } else {
                let sep = if EXACT {
                    let [ra, rb] = anchors[i];
                    p.base + (dmoved + rotate(qb, rb) - rotate(qa, ra)).dot(n)
                } else {
                    p.base + dmoved.dot(n) + p.arm + theta_b.dot(p.rnb) - theta_a.dot(p.rna)
                };
                if PUSH {
                    let soft = max(o.rate * sep, F::splat(-MAX_PUSH));
                    (
                        sep.positive_then(sep * F::splat(inv_h), soft),
                        sep.positive_then(one, o.soft_mass),
                        sep.positive_then(F::ZERO, o.soft_impulse),
                    )
                } else {
                    let bias = sep.positive_then(sep * F::splat(inv_h), F::ZERO);
                    if SEP == STORE {
                        p.bias = bias;
                    }
                    (bias, one, F::ZERO)
                }
            };
            let vn = (e.vb - e.va).dot(n) + e.wb.dot(p.rnb) - e.wa.dot(p.rna);
            let jn = max(p.jn - p.mass * mass * (vn + bias) - relax * p.jn, F::ZERO);
            let dj = jn - p.jn;
            let solid = p.solid;
            p.jn = sel(&solid, jn, p.jn);
            e.push_on(&solid, o, n.scale(dj), p.ia.scale(dj), p.ib.scale(dj));
            // A lane without the point adds a zero, which leaves a sum of
            // `max`es (never `-0.0`) as it was.
            total = total + sel(&solid, jn, F::ZERO);
            twisting = twisting + sel(&solid, jn * p.lever, F::ZERO);
        }
        if FRICTION && any(&o.rubs) {
            rub(o, &mut e, total, twisting);
        }
        e.scatter(hot, o);
    }

    /// `rub`, lane by lane: friction at the centroid, clamped to its disc,
    /// then twist.
    #[inline(always)]
    fn rub<const N: usize>(o: &mut Batch<N>, e: &mut Ends<N>, total: F<N>, twisting: F<N>) {
        let dv = e.vb - e.va;
        let v1 = dv.dot(o.t[0]) + e.wb.dot(o.tcb[0]) - e.wa.dot(o.tca[0]);
        let v2 = dv.dot(o.t[1]) + e.wb.dot(o.tcb[1]) - e.wa.dot(o.tca[1]);
        let m = o.tmass;
        let j = [o.jt[0] - (m[0] * v1 + m[1] * v2), o.jt[1] - (m[1] * v1 + m[2] * v2)];
        let limit = o.friction * total;
        let len2 = j[0] * j[0] + j[1] * j[1];
        let limit2 = limit * limit;
        // The disc's clamp, where the scalar code takes its branch; the
        // other lanes' quotient (perhaps of zeros) is left out.
        let over = mask(|l| len2.0[l] > limit2.0[l]);
        let j = if any(&over) {
            let s = div(limit, sqrt(len2));
            [sel(&over, j[0] * s, j[0]), sel(&over, j[1] * s, j[1])]
        } else {
            j
        };
        let (d1, d2) = (j[0] - o.jt[0], j[1] - o.jt[1]);
        let rubs = o.rubs;
        o.jt = [sel(&rubs, j[0], o.jt[0]), sel(&rubs, j[1], o.jt[1])];
        let linear = o.t[0].scale(d1) + o.t[1].scale(d2);
        e.push_on(&rubs, o, linear, o.tia[0].scale(d1) + o.tia[1].scale(d2), o.tib[0].scale(d1) + o.tib[1].scale(d2));

        let twists = mask(|l| rubs[l] && o.twists[l]);
        if any(&twists) {
            let wn = e.wb.dot(o.n) - e.wa.dot(o.n);
            let limit = o.friction * twisting;
            let t = (o.twist - o.twist_mass * wn).clamp(-limit, limit);
            let d = t - o.twist;
            o.twist = sel(&twists, t, o.twist);
            e.push_on(&twists, o, V::ZERO, o.na.scale(d), o.nb.scale(d));
        }
    }

    /// `refresh`, lane by lane, each lane through `Mat3::apply` itself.
    fn refresh<const N: usize>(o: &mut Batch<N>, pts: &mut [Pt<N>], bodies: &[SolverBody]) {
        for l in (0..N).filter(|l| o.real[*l]) {
            let (a, b) = (&bodies[o.a[l] as usize].inv_i, &bodies[o.b[l] as usize].inv_i);
            for p in pts[o.start..o.start + o.points].iter_mut().filter(|p| p.on[l]) {
                p.ia.set(l, a.apply(p.rna.get(l)));
                p.ib.set(l, b.apply(p.rnb.get(l)));
            }
            for i in 0..2 {
                o.tia[i].set(l, a.apply(o.tca[i].get(l)));
                o.tib[i].set(l, b.apply(o.tcb[i].get(l)));
            }
            o.na.set(l, a.apply(o.n.get(l)));
            o.nb.set(l, b.apply(o.n.get(l)));
        }
    }

    /// Restitution, lane by lane, as `one_at_a_time` takes it.
    fn restitute<const N: usize>(o: &Batch<N>, pts: &mut [Pt<N>], hot: &mut [Hot]) {
        if !o.bounces {
            return;
        }
        let pts = &mut pts[o.start..o.start + o.points];
        // The scalar code's `continue`, negated as written, so a NaN goes
        // the same way. Each point's own sum is the only one it changes.
        let bounce = |p: &Pt<N>| -> [bool; N] {
            mask(|l| {
                p.on[l] && o.restitution.0[l] != 0.0 && !(p.speed.0[l] <= BOUNCE_THRESHOLD || p.sum_jn.0[l] == 0.0 || p.mass.0[l] == 0.0)
            })
        };
        if !pts.iter().any(|p| any(&bounce(p))) {
            return;
        }
        let mut e = Ends::gather(hot, o);
        for p in pts.iter_mut() {
            let on = bounce(p);
            if !any(&on) {
                continue;
            }
            let vn = (e.vb - e.va).dot(o.n) + e.wb.dot(p.rnb) - e.wa.dot(p.rna);
            let jn = max(p.jn - p.mass * (vn - o.restitution * p.speed), F::ZERO);
            let d = jn - p.jn;
            p.jn = sel(&on, jn, p.jn);
            p.sum_jn = sel(&on, p.sum_jn + d, p.sum_jn);
            e.push_on(&on, o, o.n.scale(d), p.ia.scale(d), p.ib.scale(d));
        }
        e.scatter(hot, o);
    }
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
