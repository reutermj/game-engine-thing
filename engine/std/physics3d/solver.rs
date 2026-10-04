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
//! loop instead. Physics.md, "The solver in lanes". Grouped by Box2D's
//! colors instead (`Order::Colored`, the default since get-emj.90), the
//! result is that loop over the colors' order (`in_order`), another
//! computation: physics.md, "Colouring the 3D solve". The lanes' solve is a program of
//! stages (`lanes::Staged`), which the mod runs on `Passes` and `solve`
//! runs here.

pub use physics_common::{BOUNCE_THRESHOLD, DAMPING_RATIO, MAX_PUSH, Softness};

// The mod's pipeline's; the tests that compile this file by path don't
// take the solve apart.
#[allow(unused_imports)]
pub use lanes::{Part, Staged, Step, parts};

use crate::narrow::MAX_POINTS;
use crate::{Anchors, Carry, Closing, Inertia, Integrate, Mat3, Order, Quat, Vec3};
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
    /// Which order, and so which result: by level the sweep in pair
    /// order, colored the sweep over the colors' order (`in_order`).
    pub order: Order,
}

impl Tuning {
    pub fn of(t: &crate::Tuning) -> Tuning {
        Tuning {
            lanes: t.lanes().width(),
            sparse_alone: true,
            order: t.order(),
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
/// bit; `how.order` says which result.
pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
    match how.lanes {
        8 => lanes::solve::<8>(bodies, contacts, dt, how),
        4 => lanes::solve::<4>(bodies, contacts, dt, how),
        1 => lanes::solve::<1>(bodies, contacts, dt, how),
        _ => in_order(bodies, contacts, dt, how),
    }
}

/// The solve one contact at a time in `how.order`'s order: pair order by
/// level, the colors' order colored (`lanes::order`). The reference the
/// lanes are held to, and where they fall back to (`lanes::solve`).
pub fn in_order(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
    if how.order == Order::Levels {
        return one_at_a_time(bodies, contacts, dt, how);
    }
    // A contact's row is made from the bodies alone, before any pass, so
    // making the rows in another order changes no bit of them.
    let order = lanes::order(bodies, contacts, how);
    let mut sorted: Vec<Constraint> = order.iter().map(|&i| contacts[i]).collect();
    one_at_a_time(bodies, &mut sorted, dt, how);
    for (&i, c) in order.iter().zip(sorted) {
        contacts[i] = c;
    }
}

/// The solve one contact at a time in the order `contacts` are in.
pub fn one_at_a_time(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
    let Begun { n_sub, h, inv_h, share, max_w, soft } = begin(bodies, dt, how);
    let mut hot = hot_of(bodies);
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
    // longer (physics.md, "Still at rest"). `Carry::Normal` is the last
    // for the normal impulses alone, the mean for friction and twist.
    if how.carry != Carry::Mean {
        let k = n_sub as f32;
        for (r, c) in rows.iter().zip(contacts.iter_mut()) {
            for (p, cp) in points[r.start..r.start + r.count].iter().zip(c.points.iter_mut()) {
                cp.jn = p.jn * k;
            }
            if how.carry == Carry::Last {
                c.jt = (r.t[0] * r.jt[0] + r.t[1] * r.jt[1]) * k;
                c.twist = r.twist * k;
            }
        }
    }
    for ((b, x), q) in bodies.iter_mut().zip(&hot).zip(turned) {
        (b.v, b.w, b.moved, b.turned) = (x.v, x.w, x.moved, q);
    }
}

/// The step's start, the same whether contacts go one at a time or in
/// lanes: its constants and the bodies' world inverse inertias. The
/// bodies as the passes take them, and the contacts' rows, are each
/// way's (`hot_of`, `lanes::Staged::prepare`; `row`).
#[derive(Clone, Copy)]
struct Begun {
    n_sub: usize,
    h: f32,
    inv_h: f32,
    share: f32,
    max_w: f32,
    soft: (Softness, Softness),
}

impl Begun {
    /// Before any step: what `lanes::Staged` holds until its first.
    const NONE: Begun = {
        let soft = Softness { rate: 0.0, mass: 0.0, impulse: 0.0 };
        Begun { n_sub: 0, h: 0.0, inv_h: 0.0, share: 0.0, max_w: 0.0, soft: (soft, soft) }
    };
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
    Begun { n_sub, h, inv_h, share, max_w: MAX_ROTATION / dt, soft: (moving, fixed) }
}

/// The bodies as the loop one contact at a time takes them, less the
/// step's gravity, which the substeps give back.
fn hot_of(bodies: &[SolverBody]) -> Vec<Hot> {
    bodies.iter().map(|b| Hot { v: b.v - b.gravity, w: b.w, moved: Vec3::ZERO, inv_mass: b.inv_mass, theta: Vec3::ZERO }).collect()
}

/// A substep's share of the step's gravity given back, and each body's
/// turn rate capped (`MAX_ROTATION`): the substep's first stage, over
/// bodies.
#[inline(always)]
fn give_gravity(hot: &mut [Hot], bodies: &[SolverBody], share: f32, max_w: f32) {
    for (x, b) in hot.iter_mut().zip(bodies.iter()) {
        give(&mut x.v, &mut x.w, b.gravity, share, max_w);
    }
}

/// `give_gravity` of one body: the same whether contacts go one at a time
/// or in lanes (`lanes::Kernels::each`).
#[inline(always)]
fn give(v: &mut Vec3, w: &mut Vec3, gravity: Vec3, share: f32, max_w: f32) {
    *v += gravity * share;
    let len = w.len();
    if len > max_w {
        *w = *w * (max_w / len);
    }
}

/// Each body moved and turned by its velocities for the substep, and
/// under `Inertia::Substep` its world inverse inertia turned with it.
#[inline(always)]
fn move_bodies(hot: &mut [Hot], bodies: &mut [SolverBody], turned: &mut [Quat], how: &Tuning, h: f32, last: bool) {
    for ((x, b), q) in hot.iter_mut().zip(bodies.iter_mut()).zip(turned.iter_mut()) {
        let turns = move_one((&mut x.moved, &mut x.theta, q), (x.v, x.w), b, how, h, last);
        if turns && how.inertia == Inertia::Substep {
            b.form_inertia(q.times(b.q).normalize());
        }
    }
}

/// `move_bodies` of one body, less its inertia: its move, its turn as a
/// rotation vector (`theta`) and as a rotation (`q`). Whether it turns.
#[inline(always)]
fn move_one(
    (moved, theta, q): (&mut Vec3, &mut Vec3, &mut Quat),
    (v, w): (Vec3, Vec3),
    b: &SolverBody,
    how: &Tuning,
    h: f32,
    last: bool,
) -> bool {
    *moved += v * h;
    if b.inv_inertia == Vec3::ZERO {
        return false;
    }
    *theta += w * h;
    *q = match how.integrate {
        Integrate::Linear => q.integrate(w, h).normalize(),
        Integrate::LinearOnce => {
            let q = q.integrate(w, h);
            if last { q.normalize() } else { q }
        }
        Integrate::Exact => q.integrate_exact(w, h),
    };
    true
}

/// A body's world inverse inertia once it has turned by `turned` this
/// step, as `move_bodies` forms it under `Inertia::Substep`, and zero, as
/// `begin` left it, for one that can't turn.
#[inline(always)]
fn turned_inertia(b: &SolverBody, turned: Quat) -> Mat3 {
    if b.inv_inertia == Vec3::ZERO { Mat3::ZERO } else { Mat3::rotated_diagonal(&turned.times(b.q).normalize().matrix(), b.inv_inertia) }
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
    soft: (Softness, Softness),
    closing: (Closing, f32),
) -> Row {
    let mut speeds = [0.0; MAX_POINTS];
    let r = row_of(bodies, c, (points, &mut speeds), share, soft, closing);
    for (cp, speed) in c.points[..r.count].iter_mut().zip(speeds) {
        (cp.speed, cp.jn) = (speed, 0.0);
    }
    (c.jt, c.twist) = (Vec3::ZERO, 0.0);
    r
}

/// `row` with the contact only read: each point's closing speed into
/// `speeds`, where `row` writes it to the contact. The staged solve's fill
/// makes rows from contacts it may only read; copying each to give `row`
/// made the fill 2209-2460 µs a step on one thread, against 2080-2197
/// (step_bench, 10 000 boxes falling, 2026-10-03), and the solve slower
/// than before the fill was a stage.
#[inline(always)]
fn row_of(
    bodies: &[SolverBody],
    c: &Constraint,
    (points, speeds): (&mut [PointRow; MAX_POINTS], &mut [f32; MAX_POINTS]),
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
    for ((cp, p), s) in c.points[..count].iter().zip(points.iter_mut()).zip(speeds.iter_mut()) {
        let (ra, rb) = (cp.ra, cp.ra + c.offset);
        let (rna, rnb) = (ra.cross(n), rb.cross(n));
        let (ia, ib) = (a.inv_i.apply(rna), b.inv_i.apply(rnb));
        let k = m + ia.dot(rna) + ib.dot(rnb);
        let speed = -(b.v - a.v).dot(n) - b.w.dot(rnb) + a.w.dot(rna);
        *s = closing(how, speed, -(b.gravity - a.gravity).dot(n), -cp.depth, dt);
        // The last step's impulse was over the whole step: a substep's
        // share of it is where each substep starts.
        let jn = cp.jn * share;
        let mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        *p = PointRow { ra, rb, rna, rnb, ia, ib, base: -cp.depth - n.dot(c.offset), mass, jn, lever: 0.0 };
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
    Row {
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
    }
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
///
/// Grouped by Box2D's colors instead (`Order::Colored`, `Coloring::greedy`),
/// the same kernels give the sweep over the colors' order (`order`) bit
/// for bit, by the same argument: no two contacts of a color share a body
/// that moves.
///
/// The step is a program of stages (`Staged::program`) over the batches
/// and the bodies' `State`s, each kernel generic over where the states
/// are (`Bodies`): the mod runs it on `Passes`, plain on one thread and
/// shared on several (`Shared`); `solve` runs it here, plain (`run`).
pub mod lanes {
    use std::ops::Range;
    use std::sync::atomic::{AtomicU32, Ordering};

    use super::*;
    use engine_ecs::shape::{Colored, Coloring, EMPTY, Shareable, Stage, UNSOLVED};
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

    /// `N` contacts, field by field: `Row`, lane by lane, its points apart
    /// (`Item::pts`), as many as its lanes' most. A lane
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
        fn gather<B: Bodies + ?Sized>(s: &B, o: &Batch<N>) -> Ends<N> {
            let mut e = Ends { va: V::ZERO, wa: V::ZERO, vb: V::ZERO, wb: V::ZERO };
            for l in 0..N {
                let ((va, wa), (vb, wb)) = (s.load_v(o.a[l] as usize), s.load_v(o.b[l] as usize));
                e.va.set(l, va);
                e.wa.set(l, wa);
                e.vb.set(l, vb);
                e.wb.set(l, wb);
            }
            e
        }

        /// All of `a`'s lanes, then `b`'s: no two lanes move one body, and
        /// a body that doesn't move is written back as it was read
        /// (`shareable`), so the order is no one's business.
        #[inline(always)]
        fn scatter<B: Bodies + ?Sized>(&self, s: &mut B, o: &Batch<N>) {
            for l in 0..N {
                s.store_v(o.a[l] as usize, self.va.get(l), self.wa.get(l));
            }
            for l in 0..N {
                s.store_v(o.b[l] as usize, self.vb.get(l), self.wb.get(l));
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

    /// A field of each lane's body at `at` (`State::moved`, `theta`).
    #[inline(always)]
    fn field<const N: usize, B: Bodies + ?Sized>(s: &B, at: &[u32; N], f: impl Fn(&State) -> Vec3) -> V<N> {
        let mut v = V::ZERO;
        for l in 0..N {
            v.set(l, f(&s.load(at[l] as usize)));
        }
        v
    }

    #[inline(always)]
    fn quats<const N: usize, B: Bodies + ?Sized>(s: &B, at: &[u32; N]) -> (V<N>, F<N>) {
        let (mut v, mut w) = (V::ZERO, F::ZERO);
        for l in 0..N {
            let q = s.load(at[l] as usize).turned;
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
    /// the groups' business.
    fn shareable(bodies: &[SolverBody], moves: &[bool]) -> bool {
        let plain = |x: f32| x.is_finite() && !(x == 0.0 && x.is_sign_negative());
        let plain3 = |v: Vec3| plain(v.x) && plain(v.y) && plain(v.z);
        bodies.iter().zip(moves).all(|(b, m)| *m || (plain3(b.v) && plain3(b.w) && plain3(b.gravity)))
    }

    /// What a push changes: the velocity through the inverse mass, the
    /// turn rate through the world inverse inertia, zero exactly when the
    /// body's own is (`form_inertia`).
    fn moving(bodies: &[SolverBody], moves: &mut Vec<bool>) {
        moves.clear();
        moves.extend(bodies.iter().map(|b| b.inv_mass != 0.0 || b.inv_inertia != Vec3::ZERO));
    }

    /// The contacts grouped as `order` says: by level of the sweep in pair
    /// order, or in Box2D's colors, a contact with an end that doesn't
    /// move kept out of color 0 (`b2AddContactToGraph`'s rule, as 2D's
    /// colors). `masks` is the coloring's scratch.
    fn group(contacts: &[Constraint], moves: &[bool], order: Order, coloring: &mut Coloring, masks: &mut Vec<u64>) {
        let ends = |i: usize| (contacts[i].a, contacts[i].b);
        match order {
            Order::Levels => *coloring = physics_common::levels(contacts.len(), ends, moves),
            Order::Colored => coloring.greedy(contacts.len(), ends, moves, true, masks),
        }
        // A contact neither end of which moves still sums its impulses as
        // the sweep does; it changes no body, so any group will do.
        let still = coloring.of.iter().filter(|k| **k == UNSOLVED).count();
        if still > 0 {
            coloring.of.iter_mut().filter(|k| **k == UNSOLVED).for_each(|k| *k = 0);
            if coloring.count.is_empty() {
                coloring.count.push(0);
            }
            coloring.count[0] += still;
        }
    }

    /// The order `solve` takes contacts in under `how.order`, as indices
    /// into `contacts`: the overflow's, then each group's in pair order (a
    /// contact neither end of which moves in group 0). Solved one at a
    /// time in this order (`in_order`), a step is the lanes' bit for bit.
    /// Found by `group`, so an order and a grouping can't disagree.
    pub fn order(bodies: &[SolverBody], contacts: &[Constraint], how: &Tuning) -> Vec<usize> {
        let (mut moves, mut coloring, mut place) = (Vec::new(), Coloring::default(), Vec::new());
        moving(bodies, &mut moves);
        group(contacts, &moves, how.order, &mut coloring, &mut Vec::new());
        // One a batch: each contact's batch is its place in the order.
        coloring.pack(1, &mut place);
        let mut at: Vec<usize> = (0..contacts.len()).collect();
        at.sort_by_key(|&i| place[i].expect("every contact has a group").0);
        at
    }

    /// The solve `N` contacts at a time: the program of stages, on this
    /// thread.
    pub fn solve<const N: usize>(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32, how: &Tuning) {
        let mut staged = Staged::<N>::default();
        if !staged.prepare(bodies, contacts, dt, how) {
            return in_order(bodies, contacts, dt, how);
        }
        let mut program = Vec::new();
        staged.program(&mut program);
        let (_, mut items, states, kernels) = staged.split(bodies, contacts);
        run(&program, &mut items, states, &kernels);
        staged.finish(&mut parts(1, bodies, contacts)[0]);
    }

    /// `Passes::run`'s stages on one thread: an `Items` or `All` stage the
    /// batches in order (the overflow's, then each group's), an `Each`
    /// stage the bodies' states.
    fn run<const N: usize>(program: &[Stage<Step>], items: &mut [Item<'_, N>], states: &mut [State], kernels: &Kernels<'_, N>) {
        for stage in program {
            match *stage {
                Stage::Items(k) | Stage::All(k) => kernels.block(k, 0, items, states),
                Stage::Each(k, n) => kernels.each(k, 0..n, states),
            }
        }
    }

    /// A body as the passes read and write it, in a cache line of its own:
    /// `Hot` less the inverse mass, which each batch keeps for its ends,
    /// with how far it has turned (`turned`), which the separation under
    /// `Anchors::Exact` reads: the bodies' state for `Passes`.
    #[derive(Clone, Copy, Debug)]
    #[repr(align(64))]
    pub struct State {
        v: Vec3,
        w: Vec3,
        moved: Vec3,
        theta: Vec3,
        turned: Quat,
    }

    impl State {
        /// One standing still, unturned: where a batch's empty lanes point.
        const STILL: State = State { v: Vec3::ZERO, w: Vec3::ZERO, moved: Vec3::ZERO, theta: Vec3::ZERO, turned: Quat::IDENTITY };

        fn bits(&self) -> [f32; 16] {
            let (v, w, m, t, q) = (self.v, self.w, self.moved, self.theta, self.turned);
            [v.x, v.y, v.z, w.x, w.y, w.z, m.x, m.y, m.z, t.x, t.y, t.z, q.v.x, q.v.y, q.v.z, q.w]
        }

        fn of(b: [f32; 16]) -> State {
            State {
                v: Vec3::new(b[0], b[1], b[2]),
                w: Vec3::new(b[3], b[4], b[5]),
                moved: Vec3::new(b[6], b[7], b[8]),
                theta: Vec3::new(b[9], b[10], b[11]),
                turned: Quat { v: Vec3::new(b[12], b[13], b[14]), w: b[15] },
            }
        }
    }

    /// Where the kernels find bodies: the states themselves, or the same
    /// shared between threads (`Shared`). Each kernel is written once over
    /// this, so the two are one computation (2D's `lanes::Bodies`).
    pub trait Bodies {
        fn load(&self, i: usize) -> State;
        /// Its velocity and turn rate alone, for the passes, which read no
        /// more: shared, the other loads not made.
        fn load_v(&self, i: usize) -> (Vec3, Vec3);
        fn store_v(&mut self, i: usize, v: Vec3, w: Vec3);
        fn store(&mut self, i: usize, s: State);
    }

    impl Bodies for [State] {
        #[inline(always)]
        fn load(&self, i: usize) -> State {
            self[i]
        }

        #[inline(always)]
        fn load_v(&self, i: usize) -> (Vec3, Vec3) {
            (self[i].v, self[i].w)
        }

        #[inline(always)]
        fn store_v(&mut self, i: usize, v: Vec3, w: Vec3) {
            (self[i].v, self[i].w) = (v, w);
        }

        #[inline(always)]
        fn store(&mut self, i: usize, s: State) {
            self[i] = s;
        }
    }

    /// A `State` threads share, field by field an `f32`'s bits, loaded and
    /// stored relaxed: on x86 a plain `mov` each, and no unsafe code (2D's
    /// `Atom`). Which thread writes which body is the colors' business:
    /// within a stage no two blocks write one body that moves, and one
    /// that doesn't is written back as it was read (`shareable`).
    #[repr(C, align(64))]
    pub struct Atom([AtomicU32; 16]);

    impl Atom {
        #[inline(always)]
        fn get(&self, k: usize) -> f32 {
            f32::from_bits(self.0[k].load(Ordering::Relaxed))
        }

        #[inline(always)]
        fn put(&self, k: usize, x: f32) {
            self.0[k].store(x.to_bits(), Ordering::Relaxed)
        }
    }

    impl Shareable for State {
        type Shared = Atom;

        fn share(&self) -> Atom {
            Atom(self.bits().map(|x| AtomicU32::new(x.to_bits())))
        }

        fn load(shared: &Atom) -> State {
            State::of(std::array::from_fn(|k| shared.get(k)))
        }

        fn store(shared: &Atom, s: State) {
            for (k, x) in s.bits().into_iter().enumerate() {
                shared.put(k, x);
            }
        }
    }

    /// The states as the threads of a solve share them.
    pub struct Shared<'a>(pub &'a [Atom]);

    impl Bodies for Shared<'_> {
        #[inline(always)]
        fn load(&self, i: usize) -> State {
            <State as Shareable>::load(&self.0[i])
        }

        #[inline(always)]
        fn load_v(&self, i: usize) -> (Vec3, Vec3) {
            let a = &self.0[i];
            (Vec3::new(a.get(0), a.get(1), a.get(2)), Vec3::new(a.get(3), a.get(4), a.get(5)))
        }

        #[inline(always)]
        fn store_v(&mut self, i: usize, v: Vec3, w: Vec3) {
            let a = &self.0[i];
            for (k, x) in [v.x, v.y, v.z, w.x, w.y, w.z].into_iter().enumerate() {
                a.put(k, x);
            }
        }

        #[inline(always)]
        fn store(&mut self, i: usize, s: State) {
            <State as Shareable>::store(&self.0[i], s)
        }
    }

    /// A stage of the step's program, as `Passes` hands it to the kernels.
    /// 2D's `staged::Step` names neither the refresh nor the sums, and
    /// caps no turn rate, so 3D's is its own.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum Step {
        /// Over every batch at once: the batches filled from the contacts
        /// their lanes seat.
        Fill,
        /// Over bodies: a substep's share of gravity, the turn rate capped.
        Gravity,
        Warm,
        Push,
        /// Over bodies: moved and turned; whether it's the step's last
        /// substep (`Integrate::LinearOnce` normalizes there).
        Move(bool),
        /// The rows' angular terms from the bodies' turns
        /// (`Inertia::Substep`).
        Refresh,
        /// A relaxing pass: whether it's the substep's first, which finds
        /// each point's bias for the others to read.
        Relax(bool),
        /// The substep's impulses into the step's sums.
        Sum,
        Bounce,
    }

    /// A batch, its points and their anchors (empty but under
    /// `Anchors::Exact`), as a stage over batches gets them.
    pub struct Item<'a, const N: usize> {
        o: &'a mut Batch<N>,
        pts: &'a mut [Pt<N>],
        anchors: &'a mut [[V<N>; 2]],
    }

    /// `f` of each batch, its points and their anchors, in order: generic,
    /// so the kernel it calls is inlined into the loop (a kernel behind
    /// `&dyn Fn` cost 1 to 4%, flows.md).
    #[inline(always)]
    fn per_batch<const N: usize>(items: &mut [Item<'_, N>], mut f: impl FnMut(&mut Batch<N>, &mut [Pt<N>], &[[V<N>; 2]])) {
        for it in items.iter_mut() {
            f(&mut *it.o, &mut *it.pts, &*it.anchors);
        }
    }

    /// A step's contacts as the passes solve them: grouped, in batches of
    /// lanes, the bodies as states; kept from step to step for their
    /// allocations (a flow's payload in the mod).
    pub struct Staged<const N: usize> {
        how: Tuning,
        begun: Begun,
        dt: f32,
        moves: Vec<bool>,
        coloring: Coloring,
        masks: Vec<u64>,
        /// Each batch's contacts, lane by lane (`Coloring::seat`): what
        /// the fill fills it from.
        seats: Vec<u32>,
        /// Each contact's seat: what the write-back finds a contact's lane
        /// by, going through the contacts in their order. Written by the
        /// fill as it seats each, through the shared reference every
        /// kernel has (every contact has a group, so every one each step),
        /// as 2D's is, which measured it against forming it in `prepare`.
        seat_of: Vec<AtomicU32>,
        layout: Colored,
        states: Vec<State>,
        /// The batches, their points and the points' anchors (which only
        /// `Anchors::Exact` reads), written whole by the fill
        /// (`Step::Fill`), so kept as the last step left them, grown but
        /// never emptied here. A batch's points are as many as its lanes'
        /// most, from `starts[batch]` to the next batch's.
        out: Vec<Batch<N>>,
        pts: Vec<Pt<N>>,
        anchors: Vec<[V<N>; 2]>,
        starts: Vec<usize>,
    }

    impl<const N: usize> Default for Staged<N> {
        fn default() -> Self {
            Staged {
                how: Tuning::default(),
                begun: Begun::NONE,
                dt: 0.0,
                moves: Vec::new(),
                coloring: Coloring::default(),
                masks: Vec::new(),
                seats: Vec::new(),
                seat_of: Vec::new(),
                layout: Colored::default(),
                states: Vec::new(),
                out: Vec::new(),
                pts: Vec::new(),
                anchors: Vec::new(),
                starts: Vec::new(),
            }
        }
    }

    /// The passes' kernels, over what they read besides the batches and
    /// states: `Sync`, for a primitive that may share them out.
    pub struct Kernels<'a, const N: usize> {
        bodies: &'a [SolverBody],
        how: Tuning,
        begun: Begun,
        /// What the fill reads, and where it writes each contact's seat.
        contacts: &'a [Constraint],
        seats: &'a [u32],
        coloring: &'a Coloring,
        seat_of: &'a [AtomicU32],
        dt: f32,
    }

    /// `vec` at `n` long, what it held kept and only new room written:
    /// for what a stage writes whole.
    fn room<T: Clone>(vec: &mut Vec<T>, n: usize, empty: T) {
        if vec.len() < n {
            vec.resize(n, empty);
        }
        vec.truncate(n);
    }

    impl<const N: usize> Staged<N> {
        /// The step's start but the batches' fill, which is the program's
        /// first stage (`Step::Fill`): the contacts grouped and seated in
        /// their batches' lanes, the bodies into states, room for the
        /// batches and their points, and whether the step goes in lanes at
        /// all. If not, nothing has been changed, and the step is
        /// `in_order`'s, the same result one contact at a time. The
        /// contacts aren't changed either way: the fill reads them, and
        /// `finish` writes them.
        pub fn prepare(&mut self, bodies: &mut [SolverBody], contacts: &[Constraint], dt: f32, how: &Tuning) -> bool {
            moving(bodies, &mut self.moves);
            if !shareable(bodies, &self.moves) {
                return false;
            }
            group(contacts, &self.moves, how.order, &mut self.coloring, &mut self.masks);
            self.layout = self.coloring.seat(N, &mut self.seats);
            // Lanes only where they pay: a batch costs about two and a half
            // contacts solved one at a time, so batches under half full (a
            // stack by level, a chain, a handful of contacts: a group a
            // contact) are solved one at a time, which is the same result
            // (physics.md, "The solver in lanes").
            if how.sparse_alone && contacts.len() * 2 < self.layout.items() * N {
                return false;
            }
            (self.how, self.dt) = (*how, dt);
            self.begun = begin(bodies, dt, how);
            // One more body than there are, standing still: where a batch's
            // empty lanes point.
            let nowhere = bodies.len() as u32;
            self.states.clear();
            self.states.extend(bodies.iter().map(|b| State { v: b.v - b.gravity, w: b.w, ..State::STILL }));
            self.states.push(State::STILL);
            let starts = &mut self.starts;
            starts.clear();
            starts.push(0);
            let mut at = 0;
            for seats in self.seats.as_chunks::<N>().0 {
                at += seats.iter().filter(|i| **i != EMPTY).map(|&i| contacts[i as usize].count.min(MAX_POINTS)).max().unwrap_or(0);
                starts.push(at);
            }
            if self.seat_of.len() < contacts.len() {
                self.seat_of.resize_with(contacts.len(), AtomicU32::default);
            }
            self.seat_of.truncate(contacts.len());
            room(&mut self.out, self.layout.items(), Batch::<N>::empty(nowhere));
            room(&mut self.pts, at, Pt::<N>::EMPTY);
            room(&mut self.anchors, if how.anchors == Anchors::Exact { at } else { 0 }, [V::<N>::ZERO; 2]);
            true
        }

        /// The batches' fill, then the substeps and restitution as stages,
        /// in order, into `out`: `one_at_a_time`'s loops, each a stage over
        /// batches or bodies.
        pub fn program(&self, out: &mut Vec<Stage<Step>>) {
            out.clear();
            out.push(Stage::All(Step::Fill));
            let (n_sub, bodies) = (self.begun.n_sub, self.states.len() - 1);
            for sub in 0..n_sub {
                out.push(Stage::Each(Step::Gravity, bodies));
                out.push(Stage::Items(Step::Warm));
                out.push(Stage::Items(Step::Push));
                out.push(Stage::Each(Step::Move(sub + 1 == n_sub), bodies));
                if self.how.inertia == Inertia::Substep {
                    out.push(Stage::Items(Step::Refresh));
                }
                for r in 0..self.how.relax {
                    out.push(Stage::Items(Step::Relax(r == 0)));
                }
                out.push(Stage::Items(Step::Sum));
            }
            out.push(Stage::Items(Step::Bounce));
        }

        /// The groups' layout, the batches with their points, the states
        /// and the kernels, for the program: the bodies and contacts as
        /// `prepare` left them, for the fill.
        #[allow(clippy::type_complexity)]
        pub fn split<'a>(
            &'a mut self,
            bodies: &'a [SolverBody],
            contacts: &'a [Constraint],
        ) -> (&'a Colored, Vec<Item<'a, N>>, &'a mut [State], Kernels<'a, N>) {
            let mut items = Vec::with_capacity(self.out.len());
            let (mut pts, mut anchors) = (&mut self.pts[..], &mut self.anchors[..]);
            // A batch's points follow the last batch's (`prepare`), and so
            // do their anchors, where there are any.
            let exact = !anchors.is_empty();
            for (o, n) in self.out.iter_mut().zip(self.starts.windows(2).map(|w| w[1] - w[0])) {
                let (p, rest) = std::mem::take(&mut pts).split_at_mut(n);
                pts = rest;
                let (a, rest) = std::mem::take(&mut anchors).split_at_mut(if exact { n } else { 0 });
                anchors = rest;
                items.push(Item { o, pts: p, anchors: a });
            }
            let kernels = Kernels {
                bodies,
                how: self.how,
                begun: self.begun,
                contacts,
                seats: &self.seats,
                coloring: &self.coloring,
                seat_of: &self.seat_of,
                dt: self.dt,
            };
            (&self.layout, items, &mut self.states, kernels)
        }

        /// The groups and batches the step was laid out in: groups (levels
        /// or colors), the overflow's batches, all batches, and the most
        /// and fewest batches a group has (what threads would share a
        /// stage of).
        pub fn layout(&self) -> [usize; 5] {
            let c = &self.layout.colors;
            let (most, least) = (c.iter().copied().max().unwrap_or(0), c.iter().copied().min().unwrap_or(0));
            [c.len(), self.layout.overflow, self.layout.items(), most, least]
        }

        /// The step's impulses into a part's contacts, and the states into
        /// its bodies, as `one_at_a_time` leaves them, `Carry` and
        /// `Inertia::Substep` included; and each point's closing speed,
        /// which `row` gave the fill's copy of the contact. Contact by
        /// contact, each finding its lane by its seat (`seat_of`): every
        /// value a part writes is its lane's or its state's, so any cut
        /// into parts, on any threads, writes what one part of everything
        /// does.
        pub fn finish(&self, part: &mut Part<'_>) {
            let Part { contacts: (c0, contacts), bodies: (b0, bodies) } = part;
            let (carry, k) = (self.how.carry, self.begun.n_sub as f32);
            for (i, c) in (*c0..).zip(contacts.iter_mut()) {
                // Else its seat would be a step's before, never written.
                debug_assert_ne!(self.coloring.of[i], UNSOLVED, "every contact has a group");
                let to = self.seat_of[i].load(Ordering::Relaxed) as usize;
                let (batch, l) = (to / N, to % N);
                let (o, pts) = (&self.out[batch], &self.pts[self.starts[batch]..]);
                let count = c.count.min(MAX_POINTS);
                for (p, cp) in pts.iter().zip(c.points[..count].iter_mut()) {
                    cp.jn = if carry != Carry::Mean { p.jn.0[l] * k } else { p.sum_jn.0[l] };
                    cp.speed = p.speed.0[l];
                }
                if carry == Carry::Last {
                    c.jt = (o.t[0].get(l) * o.jt[0].0[l] + o.t[1].get(l) * o.jt[1].0[l]) * k;
                    c.twist = o.twist.0[l] * k;
                } else {
                    (c.jt, c.twist) = (o.sum_jt.get(l), o.sum_twist.0[l]);
                }
            }
            let substep = self.how.inertia == Inertia::Substep;
            for (b, x) in bodies.iter_mut().zip(&self.states[*b0..]) {
                (b.v, b.w, b.moved, b.turned) = (x.v, x.w, x.moved, x.turned);
                // As the last substep's move left it (`move_bodies`).
                if substep {
                    b.inv_i = turned_inertia(b, x.turned);
                }
            }
        }
    }

    /// What a task of the write-back writes (`Staged::finish`): a run of
    /// the contacts and a run of the bodies, each with its first index.
    pub struct Part<'a> {
        contacts: (usize, &'a mut [Constraint]),
        bodies: (usize, &'a mut [SolverBody]),
    }

    /// The bodies and contacts cut alike into `n` parts, one at least.
    pub fn parts<'a>(n: usize, mut bodies: &'a mut [SolverBody], mut contacts: &'a mut [Constraint]) -> Vec<Part<'a>> {
        let n = n.max(1);
        let (nb, nc) = (bodies.len(), contacts.len());
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let (b, c) = (nb * k / n, nc * k / n);
            let (bs, rest) = std::mem::take(&mut bodies).split_at_mut(nb * (k + 1) / n - b);
            bodies = rest;
            let (cs, rest) = std::mem::take(&mut contacts).split_at_mut(nc * (k + 1) / n - c);
            contacts = rest;
            out.push(Part { contacts: (c, cs), bodies: (b, bs) });
        }
        out
    }

    impl<const N: usize> Kernels<'_, N> {
        /// A stage over consecutive batches from `at`, the states plain or
        /// shared: which pass it is decided once a block, as 2D's.
        #[inline(always)]
        pub fn block<B: Bodies + ?Sized>(&self, k: Step, at: usize, items: &mut [Item<'_, N>], s: &mut B) {
            let inv_h = self.begun.inv_h;
            let exact = self.how.anchors == Anchors::Exact;
            let i = items;
            match k {
                Step::Fill if at == 0 && i.len() * N == self.seats.len() => self.fill_all(i),
                Step::Fill => self.fill(at, i),
                Step::Warm => per_batch(i, |o, p, _| warm_start(o, p, s)),
                Step::Push => match (self.how.friction_in_push, exact) {
                    (false, false) => per_batch(i, |o, p, a| pass::<N, true, false, false, COMPUTE, B>(o, p, a, s, inv_h)),
                    (false, true) => per_batch(i, |o, p, a| pass::<N, true, false, true, COMPUTE, B>(o, p, a, s, inv_h)),
                    (true, false) => per_batch(i, |o, p, a| pass::<N, true, true, false, COMPUTE, B>(o, p, a, s, inv_h)),
                    (true, true) => per_batch(i, |o, p, a| pass::<N, true, true, true, COMPUTE, B>(o, p, a, s, inv_h)),
                },
                // Bodies don't move between relaxing passes, so neither do
                // separations: the first finds each point's bias, the
                // others read it, the same bits.
                Step::Relax(first) => match (first, exact) {
                    (true, false) => per_batch(i, |o, p, a| pass::<N, false, true, false, STORE, B>(o, p, a, s, inv_h)),
                    (true, true) => per_batch(i, |o, p, a| pass::<N, false, true, true, STORE, B>(o, p, a, s, inv_h)),
                    (false, _) => per_batch(i, |o, p, a| pass::<N, false, true, false, LOAD, B>(o, p, a, s, inv_h)),
                },
                Step::Refresh => per_batch(i, |o, p, _| refresh(o, p, self.bodies, s)),
                Step::Sum => per_batch(i, |o, p, _| sum(o, p)),
                Step::Bounce => per_batch(i, |o, p, _| restitute(o, p, s)),
                Step::Gravity | Step::Move(_) => unreachable!("a stage over bodies"),
            }
        }

        /// Batches `at..` from the contacts their lanes seat, each batch,
        /// its points and their anchors written whole, so what they held
        /// before is never read; and each contact's seat, for the
        /// write-back.
        #[inline(always)]
        fn fill(&self, at: usize, items: &mut [Item<'_, N>]) {
            for (batch, (it, seats)) in (at..).zip(items.iter_mut().zip(self.seats[at * N..].as_chunks::<N>().0)) {
                self.empty(it);
                for (l, &i) in seats.iter().enumerate() {
                    if i != EMPTY {
                        self.enter(it, l, &self.contacts[i as usize]);
                        self.seat_of[i as usize].store((batch * N + l) as u32, Ordering::Relaxed);
                    }
                }
            }
        }

        /// `fill` of every batch, contact by contact in pair order, each
        /// into the seat the coloring gives it: the same values, read in
        /// the order the contacts are stored, as 2D's `fill_all`; but it
        /// writes any batch, so it's the fill of a block that has them all.
        #[inline(always)]
        fn fill_all(&self, items: &mut [Item<'_, N>]) {
            items.iter_mut().for_each(|it| self.empty(it));
            for ((c, to), seat) in self.contacts.iter().zip(self.coloring.seats(N)).zip(self.seat_of) {
                let to = to.expect("every contact has a group");
                self.enter(&mut items[to / N], to % N, c);
                seat.store(to as u32, Ordering::Relaxed);
            }
        }

        #[inline(always)]
        fn empty(&self, it: &mut Item<'_, N>) {
            *it.o = Batch::empty(self.nowhere());
            it.o.points = it.pts.len();
            it.pts.fill(Pt::EMPTY);
            it.anchors.fill([V::ZERO; 2]);
        }

        /// The body past the real ones, where a batch's empty lanes point.
        #[inline(always)]
        fn nowhere(&self) -> u32 {
            self.bodies.len() as u32
        }

        /// Contact `c`'s row made, as `prepare` made it before the fill
        /// was a stage, into lane `l` of its batch and points. The contact
        /// is only read (`row_of`): `finish` writes what `row` would have
        /// to it, the closing speeds included.
        #[inline(always)]
        fn enter(&self, it: &mut Item<'_, N>, l: usize, c: &Constraint) {
            let Begun { share, soft, .. } = self.begun;
            let (mut ps, mut speeds) = ([PointRow::default(); MAX_POINTS], [0.0; MAX_POINTS]);
            let r = row_of(self.bodies, c, (&mut ps, &mut speeds), share, soft, (self.how.closing, self.dt));
            put(it.o, l, &r, c, self.bodies);
            let exact = !it.anchors.is_empty();
            for (k, p) in ps[..r.count].iter().enumerate() {
                let q = &mut it.pts[k];
                q.rna.set(l, p.rna);
                q.rnb.set(l, p.rnb);
                q.ia.set(l, p.ia);
                q.ib.set(l, p.ib);
                (q.base.0[l], q.arm.0[l], q.mass.0[l], q.jn.0[l]) = (p.base, (p.rb - p.ra).dot(r.n), p.mass, p.jn);
                (q.lever.0[l], q.speed.0[l]) = (p.lever, speeds[k]);
                (q.on[l], q.solid[l]) = (true, p.mass != 0.0);
                if exact {
                    it.anchors[k][0].set(l, p.ra);
                    it.anchors[k][1].set(l, p.rb);
                }
            }
        }

        /// A stage over the states `r`, plain or shared: gravity's share,
        /// or moving and turning them, each body as `one_at_a_time`'s loop
        /// takes it (`give`, `move_one`).
        #[inline(always)]
        pub fn each<B: Bodies + ?Sized>(&self, k: Step, r: Range<usize>, s: &mut B) {
            let Begun { h, share, max_w, .. } = self.begun;
            match k {
                Step::Gravity => {
                    for i in r {
                        let (mut v, mut w) = s.load_v(i);
                        give(&mut v, &mut w, self.bodies[i].gravity, share, max_w);
                        s.store_v(i, v, w);
                    }
                }
                Step::Move(last) => {
                    for i in r {
                        let mut x = s.load(i);
                        move_one((&mut x.moved, &mut x.theta, &mut x.turned), (x.v, x.w), &self.bodies[i], &self.how, h, last);
                        s.store(i, x);
                    }
                }
                _ => unreachable!("a stage over batches"),
            }
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
    fn warm_start<const N: usize, B: Bodies + ?Sized>(o: &Batch<N>, pts: &[Pt<N>], s: &mut B) {
        let mut e = Ends::gather(s, o);
        for p in pts {
            e.push_on(&p.on, o, o.n.scale(p.jn), p.ia.scale(p.jn), p.ib.scale(p.jn));
        }
        let (j1, j2) = (o.jt[0], o.jt[1]);
        e.push(o, o.t[0].scale(j1) + o.t[1].scale(j2), o.tia[0].scale(j1) + o.tia[1].scale(j2), o.tib[0].scale(j1) + o.tib[1].scale(j2));
        e.push(o, V::ZERO, o.na.scale(o.twist), o.nb.scale(o.twist));
        e.scatter(s, o);
    }

    /// How a pass finds a point's bias (`pass`'s `SEP`): from its
    /// separation, storing it too, or as the last pass stored it.
    const COMPUTE: u8 = 0;
    const STORE: u8 = 1;
    const LOAD: u8 = 2;

    /// `pass`, lane by lane: soft and pushing out when `PUSH`, else rigid;
    /// with friction when `FRICTION`; separations by `Anchors::Exact` when
    /// `EXACT`, else `Anchors::Linear`, or none read when `SEP` is `LOAD`.
    /// `pts` are the batch's own, and `anchors` their anchors.
    #[inline(always)]
    fn pass<const N: usize, const PUSH: bool, const FRICTION: bool, const EXACT: bool, const SEP: u8, B: Bodies + ?Sized>(
        o: &mut Batch<N>,
        pts: &mut [Pt<N>],
        anchors: &[[V<N>; 2]],
        s: &mut B,
        inv_h: f32,
    ) {
        let mut e = Ends::gather(s, o);
        // Only what this pass's separation reads.
        let dmoved = if SEP == LOAD { V::ZERO } else { field(s, &o.b, |x| x.moved) - field(s, &o.a, |x| x.moved) };
        let (theta_a, theta_b) =
            if SEP == LOAD || EXACT { (V::ZERO, V::ZERO) } else { (field(s, &o.a, |x| x.theta), field(s, &o.b, |x| x.theta)) };
        let (qa, qb) = if EXACT && SEP != LOAD { (quats(s, &o.a), quats(s, &o.b)) } else { ((V::ZERO, F::ZERO), (V::ZERO, F::ZERO)) };
        let (n, one) = (o.n, F::splat(1.0));
        let (mut total, mut twisting) = (F::ZERO, F::ZERO);
        for k in 0..o.points {
            let p = &mut pts[k];
            if !any(&p.solid) {
                continue;
            }
            let (bias, mass, relax) = if SEP == LOAD {
                (p.bias, one, F::ZERO)
            } else {
                let sep = if EXACT {
                    let [ra, rb] = anchors[k];
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
        e.scatter(s, o);
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

    /// `refresh`, lane by lane, each lane through `Mat3::apply` itself;
    /// each end's world inverse inertia formed from its turn so far as
    /// `move_bodies` forms it (`turned_inertia`), since the states carry
    /// the turn and not the matrix.
    fn refresh<const N: usize, B: Bodies + ?Sized>(o: &mut Batch<N>, pts: &mut [Pt<N>], bodies: &[SolverBody], s: &B) {
        for l in (0..N).filter(|l| o.real[*l]) {
            let inertia = |i: u32| turned_inertia(&bodies[i as usize], s.load(i as usize).turned);
            let (a, b) = (&inertia(o.a[l]), &inertia(o.b[l]));
            for p in pts.iter_mut().filter(|p| p.on[l]) {
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
    fn restitute<const N: usize, B: Bodies + ?Sized>(o: &Batch<N>, pts: &mut [Pt<N>], s: &mut B) {
        if !o.bounces {
            return;
        }
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
        let mut e = Ends::gather(s, o);
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
        e.scatter(s, o);
    }

    /// The substep's impulses into the step's sums, as `one_at_a_time`
    /// sums them.
    fn sum<const N: usize>(o: &mut Batch<N>, pts: &mut [Pt<N>]) {
        for p in pts.iter_mut() {
            p.sum_jn = p.sum_jn + p.jn;
        }
        o.sum_jt = o.sum_jt + (o.t[0].scale(o.jt[0]) + o.t[1].scale(o.jt[1]));
        o.sum_twist = o.sum_twist + o.twist;
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

    /// Box2D's coloring, which the colored order is (`lanes::order`): the
    /// lowest color neither moving end has yet, a contact with a static
    /// end never in color 0 (`b2AddContactToGraph`), the colors in turn,
    /// pair order within each. Bodies 0, 1 and 2 move, 3 is static.
    #[test]
    fn contacts_are_colored_as_box2d_colors_them() {
        let mut bodies = [cube(Vec3::ZERO, Vec3::ZERO); 4];
        bodies[3] = SolverBody::default();
        let pairs = [(0, 3), (0, 1), (1, 2), (2, 3), (1, 3)];
        let contacts: Vec<Constraint> = pairs.iter().map(|&(a, b)| Constraint { a, b, ..under(0.0, 0.0) }).collect();
        let colored = Tuning { order: Order::Colored, ..Tuning::default() };
        // Colors 1, 0, 1, 2, 2: (0, 3) kept out of 0, (1, 3) past 0 and 1
        // at body 1.
        assert_eq!(lanes::order(&bodies, &contacts, &colored), [1, 0, 2, 3, 4]);
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
