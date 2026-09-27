//! The contact solver: the step's bodies and contacts, gathered from the
//! world by the mod and written back after. A body is a velocity and an
//! inverse mass, and if it turns an angular velocity and an inverse
//! inertia; a contact between bodies that don't turn is a normal and a
//! depth, and one that turns a normal and up to two points.
//!
//! A soft step, as Box2D v3 takes it (Erin Catto's "Solver2D"): the step
//! split into `SUBSTEPS`, each of them
//! 1. gravity for the substep, and last substep's impulses again (warm
//!    starting);
//! 2. one pass of sequential impulses whose contacts are soft springs: a
//!    penetrating contact pushes out with a velocity (at most `MAX_PUSH`)
//!    through the real velocity, softened so pushing can't overshoot;
//! 3. positions (and rotations) moved by the velocities, and every
//!    contact's separation updated from how far its bodies moved (and how
//!    its points' arms turned), without finding contacts again;
//! 4. `RELAX_ITERATIONS` rigid passes, pushing nothing, which take the
//!    push's speed back out, so correction adds no energy;
//!
//! then one pass of restitution, from each contact's closing speed before
//! the step. Why a soft step, and what it replaced (a split impulse, which
//! kept piles creeping for thousands of steps): physics.md, "Settling".
//!
//! Friction is solved only in the relaxing passes, as Rapier does
//! (`friction_in_bias_pass` off): friction reacting to the push moves
//! bodies sideways, and it's cheaper. Sequential impulses, clamped
//! accumulated impulses and warm starting, speculative contacts and the
//! restitution threshold are Catto's too, and so is the way rotation
//! enters: each point with its arms from both bodies' centers, an effective
//! mass with the arms' cross products, and a separation that follows the
//! arms as the bodies turn within the step (Box2D's `b2PrepareContacts`
//! and `b2SolveContact`, `contact_solver.c`). See docs/CREDITS.md, and
//! physics.md, "Rotation", for what was measured against what.
//!
//! A contact whose ends can't turn is solved exactly as before rotation,
//! with one row at its normal: the rotating path is taken only by contacts
//! with points and an end that turns, so a world where nothing turns is
//! the same computation bit for bit.

use physics::{Rot, Vec2};

/// The default's substeps (`physics::Tuning`, which a world can change):
/// a soft contact is only as stiff as its substeps are short
/// (`STIFFNESS`).
pub const SUBSTEPS: usize = physics::Tuning::DEFAULT.substeps as usize;
/// Two where Box2D has one: with one, a pile of 10 000 still has bodies
/// sliding at step 400; with two it's at rest by 240.
pub const RELAX_ITERATIONS: usize = 2;
/// How stiff a contact between two moving bodies is, as a share of the
/// substep rate: 75 Hz at 5 substeps of 1/60 s. Soft contacts sink under
/// load by load / (mass * omega^2), so this is as stiff as holds: at half
/// the substep rate the pile jitters and never rests. Box2D has 30 Hz, and
/// piles sink 0.06 deep in it (docs/lore on soft contacts).
pub const STIFFNESS: f32 = 0.25;
/// Contacts with something that doesn't move are twice as stiff, as in
/// Box2D: nothing on the other side gives.
pub const STATIC_STIFFNESS: f32 = 0.5;
/// Heavily damped, as in Box2D: a contact pushes out without bouncing.
pub const DAMPING_RATIO: f32 = 10.0;
/// The fastest a contact pushes bodies apart, so a deep overlap comes
/// apart over a few steps, not in one throw.
pub const MAX_PUSH: f32 = 3.0;
/// Closing speeds below this don't bounce, so resting bodies settle.
pub const BOUNCE_THRESHOLD: f32 = 1.0;
/// Restitution passes over a contact's two points: one (Box2D's) bounces a
/// box landing flat at 10 with restitution 0.5 back at 4.4, turning it at
/// 1.2 a second; four at 5.0, not turning it
/// (`a_turning_body_bounces_at_its_restitution_at_each_point`).
pub const BOUNCE_ITERATIONS: usize = 4;

/// How a contact point's separation follows its bodies turning within a
/// step, between finding contacts and the next time they're found. The
/// others than the default are the comparison's variants.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Separation {
    /// The arms turned by how far each body turned: Box2D's. The default.
    Turned,
    /// The arms moved by the turn to first order (`dθ × r`), as a small
    /// angle would: cheaper by a rotation a point, off by `r dθ² / 2`.
    Linear,
    /// Turning ignored: only the bodies' centers' moves count.
    Fixed,
}

/// How a turning body's rotation is carried through the substeps.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Integrate {
    /// As a rotation, turned a substep at a time and normalized: Box2D's
    /// `b2IntegrateRotation`. The default.
    Rotation,
    /// As an angle, summed, with its sine and cosine taken where a
    /// rotation is needed: Rapier's angular velocity integration is the
    /// exponential map, which in 2D is this.
    Angle,
}

/// The solver's constants: `PARAMS`, as a world's `Tuning` sets them
/// (`of`), or the comparison's variants.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    pub substeps: usize,
    pub relax: usize,
    pub separation: Separation,
    pub integrate: Integrate,
    /// Warm-start points from last step's impulses (else from nothing).
    pub warm: bool,
    /// Restitution passes over a contact's two points.
    pub bounce: usize,
    /// `STIFFNESS` and `STATIC_STIFFNESS`.
    pub stiffness: f32,
    pub static_stiffness: f32,
    /// A contact's two points solved together in the relax passes, as one
    /// 2x2 problem (`block`); else one after the other, as Box2D v3 does.
    /// Off: measured, it stood a 20-high stack sooner and set a 5050
    /// pyramid vibrating (physics.md, "Still at rest").
    pub block: bool,
}

pub const PARAMS: Params = Params {
    substeps: SUBSTEPS,
    relax: RELAX_ITERATIONS,
    separation: Separation::Turned,
    integrate: Integrate::Rotation,
    warm: true,
    bounce: BOUNCE_ITERATIONS,
    stiffness: STIFFNESS,
    static_stiffness: STATIC_STIFFNESS,
    block: false,
};

impl Params {
    #[allow(dead_code)] // The benches that compile this file solve at the default.
    pub fn of(t: &physics::Tuning) -> Params {
        Params { substeps: t.substeps(), ..PARAMS }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SolverBody {
    pub v: Vec2,
    /// 0 for static and kinematic bodies.
    pub inv_mass: f32,
    /// The velocity gravity gave the body this step, already in `v`: the
    /// solver takes it back out and gives it a substep at a time, so a
    /// resting contact holds its weight in every substep. Gravity is added
    /// before the solve so that contacts are found, and bounce, at the
    /// speed they meet with.
    pub gravity: Vec2,
    /// Output: how far the body moved this step.
    pub moved: Vec2,
}

impl SolverBody {
    pub fn new(v: Vec2, inv_mass: f32, gravity: Vec2) -> SolverBody {
        SolverBody { v, inv_mass, gravity, moved: Vec2::ZERO }
    }

    /// How far a dynamic body moves this step.
    pub fn displacement(&self, _dt: f32) -> Vec2 {
        self.moved
    }
}

/// A body that turns, beside the bodies rather than in each, since most
/// don't: a `SolverBody` of angular state grew the bodies half again, and
/// every contact's pass reads them (physics.md, "Rotation"). In, its
/// angular velocity and inverse inertia (0 for one contacts don't turn,
/// which may still turn at `w`: a kinematic body); out, both, and how far
/// it turned, as a rotation (Box2D's `deltaRotation`) and as an angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spinning {
    /// Its index among the bodies.
    pub body: u32,
    pub w: f32,
    pub inv_inertia: f32,
    pub turned: Rot,
    pub angle: f32,
}

impl Spinning {
    pub fn new(body: u32, w: f32, inv_inertia: f32) -> Spinning {
        Spinning { body, w, inv_inertia, turned: Rot::IDENTITY, angle: 0.0 }
    }

    /// Where a body facing `q` faces after the step.
    pub fn rotation(&self, q: Rot) -> Rot {
        self.turned.after(q).normalized()
    }

    /// Where a body facing `q` faces after the step, if the step turned it:
    /// not if it neither turned nor spins, so what didn't turn isn't
    /// written.
    #[allow(dead_code)]
    pub fn turned_from(&self, q: Rot, spin: f32) -> Option<Rot> {
        (!(self.w == 0.0 && self.turned == Rot::IDENTITY && spin == 0.0)).then(|| self.rotation(q))
    }
}

/// For `tests/arrays.rs`, which builds over this solver and the split
/// impulse's, which has no rotation: what it needs of each, by the same
/// names in both.
#[allow(dead_code)]
impl Constraint {
    /// Solved at `points[at]` if an end turns.
    pub fn with_points(self, at: usize) -> Constraint {
        Constraint { points: at as u32 + 1, ..self }
    }
}

/// One contact point as the mod hands it over: its arms from each body's
/// center (world axes), separation, and impulses (in, last step's by
/// feature; out, this step's).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactPoint {
    pub ra: Vec2,
    pub rb: Vec2,
    pub separation: f32,
    pub jn: f32,
    pub jt: f32,
}

/// A contact's points, beside the contacts rather than in each: most
/// contacts have none (their ends don't turn), and a `Constraint` stays as
/// small as it was before rotation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Points {
    pub count: u8,
    pub point: [ContactPoint; 2],
    /// Output: whether it was solved at its points, because an end turns;
    /// if not, at its normal, and the points' impulses are 0.
    pub solved: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Constraint {
    pub a: u32,
    pub b: u32,
    pub normal: Vec2,
    pub depth: f32,
    pub friction: f32,
    pub restitution: f32,
    /// Accumulated impulses over the step: in, the last step's (warm
    /// starting); out, this step's. Solved at its points, their sums out.
    pub jn: f32,
    pub jt: f32,
    /// Output: the closing speed along the normal before solving (at its
    /// points, the fastest point's).
    pub speed: f32,
    /// Its points in the slice handed to `solve_points`, plus one: 0 for a
    /// contact with none.
    pub points: u32,
}

/// A soft contact's constants for a substep of `h`: how fast it pushes
/// out per unit of penetration, and how much of a rigid impulse it takes
/// (`mass`) and of its accumulated one it lets go (`impulse`). Box2D's
/// `b2MakeSoft`.
#[derive(Clone, Copy, Debug)]
struct Softness {
    rate: f32,
    mass: f32,
    impulse: f32,
}

impl Softness {
    fn new(hertz: f32, zeta: f32, h: f32) -> Softness {
        let omega = 2.0 * std::f32::consts::PI * hertz;
        let a1 = 2.0 * zeta + h * omega;
        let a2 = h * omega * a1;
        let a3 = 1.0 / (1.0 + a2);
        Softness { rate: omega / a1, mass: a2 * a3, impulse: a3 }
    }
}

/// No points: a row at the contact's normal alone.
const LINEAR: u32 = u32::MAX;

/// A contact as the substeps solve it.
struct Row {
    /// `u32`, not `usize`: with its points' index the row is still no bigger
    /// than it was before rotation.
    a: u32,
    b: u32,
    normal: Vec2,
    /// 1 / (the ends' inverse masses), 0 when neither moves. For a row with
    /// points, only whether either moves: each point has its own.
    mass: f32,
    /// Separation (minus depth) when found: bodies' movement adds to it.
    base: f32,
    soft: Softness,
    friction: f32,
    /// This substep's accumulated impulses.
    jn: f32,
    jt: f32,
    /// Its points in `turning`, or `LINEAR`.
    points: u32,
}

impl Row {
    #[inline(always)]
    fn a(&self) -> usize {
        self.a as usize
    }

    #[inline(always)]
    fn b(&self) -> usize {
        self.b as usize
    }
}

/// The bodies as the substeps move them: `SolverBody` itself, as small as
/// before rotation, apart from what only turning rows read (`Ang`).
type Lin = SolverBody;

/// A body's angular state as the substeps carry it.
#[derive(Clone, Copy)]
struct Ang {
    w: f32,
    inv_inertia: f32,
    turned: Rot,
    angle: f32,
}

/// A contact point as the substeps solve it: Box2D's
/// `b2ContactConstraintPoint`.
#[derive(Clone, Copy, Default)]
struct Point {
    ra: Vec2,
    rb: Vec2,
    /// The separation less the arms' ends' offset along the normal, so the
    /// separation now is this plus their offset now.
    base: f32,
    normal_mass: f32,
    tangent_mass: f32,
    /// The arms' cross products with the normal and the tangent: how an
    /// impulse along each turns each body, and how each body's turning
    /// moves the point along it. Kept, as Box2D's wide solver keeps them,
    /// so a pass multiplies instead of crossing.
    rna: f32,
    rnb: f32,
    rta: f32,
    rtb: f32,
    jn: f32,
    jt: f32,
    /// The closing speed before the step, for restitution.
    speed: f32,
    /// Whether it pushed in any substep: restitution only bounces what hit.
    pushed: bool,
}

#[derive(Clone, Copy, Default)]
struct Turning {
    /// Where its points are in the slice `solve_points` has.
    at: usize,
    count: usize,
    p: [Point; 2],
}

/// Nothing turns: the solver the variants and experiments call.
#[allow(dead_code)]
pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    solve_with(&PARAMS, (bodies, &mut []), contacts, &mut [], dt);
}

/// Bodies some of which turn (`spinning`), and contacts some of which have
/// `points` (by `Constraint::points`), at the default `Tuning`: the arrays'
/// solver, where the mod solves by its world's (`solve_with`).
#[allow(dead_code)]
pub fn solve_points(bodies: &mut [SolverBody], spinning: &mut [Spinning], contacts: &mut [Constraint], points: &mut [Points], dt: f32) {
    solve_with(&PARAMS, (bodies, spinning), contacts, points, dt);
}

/// `solve_points`, with other constants: the comparison's variants.
#[inline(always)]
pub fn solve_with(
    params: &Params,
    (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
    contacts: &mut [Constraint],
    points: &mut [Points],
    dt: f32,
) {
    // Nothing turns and no contact has points: the step compiled without
    // them, which is what a world before rotation ran (the tests for them,
    // though never taken, cost the 5050 pyramid's solve 3%).
    if points.is_empty() && spinning.is_empty() {
        solve_all::<false>(params, (bodies, spinning), contacts, points, dt);
    } else {
        solve_all::<true>(params, (bodies, spinning), contacts, points, dt);
    }
}

#[inline(always)]
fn solve_all<const POINTS: bool>(
    params: &Params,
    (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
    contacts: &mut [Constraint],
    points: &mut [Points],
    dt: f32,
) {
    let substeps = params.substeps;
    let h = dt / substeps as f32;
    let inv_h = 1.0 / h;
    let moving = Softness::new(params.stiffness * inv_h, DAMPING_RATIO, h);
    let fixed = Softness::new(params.static_stiffness * inv_h, DAMPING_RATIO, h);
    let share = 1.0 / substeps as f32;
    let warm = if params.warm { share } else { 0.0 };
    // Every body's angular state, if any turns: indexed like the bodies,
    // since contacts name bodies by index.
    let mut ang: Vec<Ang> = Vec::new();
    if !spinning.is_empty() {
        ang = vec![Ang { w: 0.0, inv_inertia: 0.0, turned: Rot::IDENTITY, angle: 0.0 }; bodies.len()];
        for s in spinning.iter() {
            ang[s.body as usize] = Ang { w: s.w, inv_inertia: s.inv_inertia, turned: Rot::IDENTITY, angle: 0.0 };
        }
    }
    let spins = |i: usize| ang.get(i).is_some_and(|a| a.inv_inertia > 0.0 || a.w != 0.0);
    let mut turning: Vec<Turning> = Vec::new();
    let mut rows: Vec<Row> = contacts
        .iter_mut()
        .map(|c| {
            let (a, b) = (c.a as usize, c.b as usize);
            let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
            let k = ia + ib;
            let turns = POINTS && c.points > 0 && (spins(a) || spins(b));
            let at = if turns {
                let at = c.points as usize - 1;
                let (t, speed) = prepare((&bodies[a], &ang[a]), (&bodies[b], &ang[b]), c.normal, &points[at], at, warm);
                c.speed = speed;
                turning.push(t);
                (turning.len() - 1) as u32
            } else {
                c.speed = -(bodies[b].v - bodies[a].v).dot(c.normal);
                LINEAR
            };
            // The last step's impulse was over the whole step: a substep's
            // share of it is where each substep starts.
            let row = Row {
                a: a as u32,
                b: b as u32,
                normal: c.normal,
                mass: if k > 0.0 { 1.0 / k } else { 0.0 },
                base: -c.depth,
                soft: if ia == 0.0 || ib == 0.0 { fixed } else { moving },
                friction: c.friction,
                jn: c.jn * share,
                jt: c.jt * share,
                points: at,
            };
            (c.jn, c.jt) = (0.0, 0.0);
            row
        })
        .collect();
    for p in points.iter_mut() {
        p.solved = false;
        for q in &mut p.point {
            (q.jn, q.jt) = (0.0, 0.0);
        }
    }
    for t in &turning {
        points[t.at].solved = true;
    }
    let lin: &mut [Lin] = bodies;
    for b in lin.iter_mut() {
        b.v -= b.gravity;
        b.moved = Vec2::ZERO;
    }

    for _ in 0..substeps {
        for b in lin.iter_mut() {
            b.v += b.gravity * share;
        }
        for r in rows.iter().filter(|r| r.mass != 0.0) {
            if !POINTS || r.points == LINEAR {
                apply(lin, r, r.normal * r.jn + r.normal.perp() * r.jt);
            } else {
                let t = r.normal.perp();
                let pts = &turning[r.points as usize];
                for p in &pts.p[..pts.count] {
                    let turn = (p.rna * p.jn + p.rta * p.jt, p.rnb * p.jn + p.rtb * p.jt);
                    apply_at(lin, &mut ang, r, r.normal * p.jn + t * p.jt, turn);
                }
            }
        }
        passes::<POINTS>(params, lin, &mut ang, &mut rows, &mut turning, inv_h, true);
        for b in lin.iter_mut() {
            b.moved += b.v * h;
        }
        for s in spinning.iter() {
            let b = &mut ang[s.body as usize];
            b.angle += h * b.w;
            b.turned = match params.integrate {
                Integrate::Rotation => b.turned.integrate(h * b.w),
                Integrate::Angle => Rot::from_angle(b.angle),
            };
        }
        for _ in 0..params.relax {
            passes::<POINTS>(params, lin, &mut ang, &mut rows, &mut turning, inv_h, false);
        }
        for (r, c) in rows.iter().zip(contacts.iter_mut()) {
            if !POINTS || r.points == LINEAR {
                c.jn += r.jn;
                c.jt += r.jt;
            } else {
                let t = &turning[r.points as usize];
                for (p, out) in t.p[..t.count].iter().zip(&mut points[t.at].point) {
                    out.jn += p.jn;
                    out.jt += p.jt;
                }
            }
        }
    }

    // Restitution, once, from the closing speed before the step, for the
    // contacts that pushed: a speculative contact that stopped a body at
    // the surface bounces it at the speed it came in at, not at what was
    // left of it. Per point with points, as Box2D's `b2ApplyRestitution`.
    for (r, c) in rows.iter_mut().zip(contacts.iter_mut()) {
        if POINTS && r.points != LINEAR {
            let t = &mut turning[r.points as usize];
            let out = &mut points[t.at];
            if c.restitution != 0.0 && r.mass != 0.0 {
                bounce(lin, &mut ang, r, t, (c.restitution, params.bounce), out);
            }
            c.jn = out.point[..t.count].iter().map(|p| p.jn).sum();
            c.jt = out.point[..t.count].iter().map(|p| p.jt).sum();
            continue;
        }
        if c.restitution == 0.0 || c.speed <= BOUNCE_THRESHOLD || c.jn == 0.0 || r.mass == 0.0 {
            continue;
        }
        let vn = (lin[r.b()].v - lin[r.a()].v).dot(r.normal);
        let jn = (r.jn - r.mass * (vn - c.restitution * c.speed)).max(0.0);
        let d = jn - r.jn;
        r.jn = jn;
        c.jn += d;
        apply(lin, r, r.normal * d);
    }

    for s in spinning.iter_mut() {
        let a = &ang[s.body as usize];
        (s.w, s.turned, s.angle) = (a.w, a.turned, a.angle);
    }
}

/// A turning contact's points for the substeps, warm-started with `warm`
/// of last step's impulses, and its fastest closing speed.
fn prepare((a, qa): (&SolverBody, &Ang), (b, qb): (&SolverBody, &Ang), n: Vec2, from: &Points, at: usize, warm: f32) -> (Turning, f32) {
    let t = n.perp();
    let (ma, mb, ia, ib) = (a.inv_mass, b.inv_mass, qa.inv_inertia, qb.inv_inertia);
    let mut out = Turning { at, count: from.count as usize, p: [Point::default(); 2] };
    let mut fastest = f32::NEG_INFINITY;
    for (p, cp) in out.p.iter_mut().zip(&from.point).take(out.count) {
        let (ra, rb) = (cp.ra, cp.rb);
        let (rna, rnb) = (ra.cross(n), rb.cross(n));
        let (rta, rtb) = (ra.cross(t), rb.cross(t));
        let kn = ma + mb + ia * rna * rna + ib * rnb * rnb;
        let kt = ma + mb + ia * rta * rta + ib * rtb * rtb;
        let vr = (b.v + rb.turned_by(qb.w)) - (a.v + ra.turned_by(qa.w));
        let speed = -vr.dot(n);
        fastest = fastest.max(speed);
        *p = Point {
            ra,
            rb,
            base: cp.separation - (rb - ra).dot(n),
            normal_mass: if kn > 0.0 { 1.0 / kn } else { 0.0 },
            tangent_mass: if kt > 0.0 { 1.0 / kt } else { 0.0 },
            rna,
            rnb,
            rta,
            rtb,
            jn: cp.jn * warm,
            jt: cp.jt * warm,
            speed,
            pushed: false,
        };
    }
    (out, fastest)
}

/// `pass`, with no test for points in a step where no contact has them,
/// so a world where nothing turns runs the loop it ran before rotation.
#[inline(always)]
fn passes<const POINTS: bool>(
    params: &Params,
    lin: &mut [Lin],
    ang: &mut [Ang],
    rows: &mut [Row],
    turning: &mut [Turning],
    inv_h: f32,
    push: bool,
) {
    if !POINTS || turning.is_empty() {
        pass::<false>(params, lin, ang, rows, turning, inv_h, push);
    } else {
        pass::<true>(params, lin, ang, rows, turning, inv_h, push);
    }
}

/// One pass of sequential impulses over the contacts: soft and pushing
/// out when `push`, else rigid, with friction.
fn pass<const POINTS: bool>(
    params: &Params,
    lin: &mut [Lin],
    ang: &mut [Ang],
    rows: &mut [Row],
    turning: &mut [Turning],
    inv_h: f32,
    push: bool,
) {
    for r in rows.iter_mut() {
        if r.mass == 0.0 {
            continue;
        }
        if POINTS && r.points != LINEAR {
            pass_points(params, lin, ang, r, &mut turning[r.points as usize], inv_h, push);
            continue;
        }
        let (a, b) = (&lin[r.a()], &lin[r.b()]);
        let sep = r.base + (b.moved - a.moved).dot(r.normal);
        // A gap may close this substep, and no more: speculative, in
        // either pass.
        let (bias, mass, relax) = if sep > 0.0 {
            (sep * inv_h, 1.0, 0.0)
        } else if push {
            ((r.soft.rate * sep).max(-MAX_PUSH), r.soft.mass, r.soft.impulse)
        } else {
            (0.0, 1.0, 0.0)
        };
        let vn = (b.v - a.v).dot(r.normal);
        let jn = (r.jn - r.mass * mass * (vn + bias) - relax * r.jn).max(0.0);
        let d = jn - r.jn;
        r.jn = jn;
        apply(lin, r, r.normal * d);
        if push {
            continue;
        }

        let t = r.normal.perp();
        let vt = (lin[r.b()].v - lin[r.a()].v).dot(t);
        let limit = r.friction * r.jn;
        let jt = (r.jt - r.mass * vt).clamp(-limit, limit);
        let d = jt - r.jt;
        r.jt = jt;
        apply(lin, r, t * d);
    }
}

/// Arm `r` on a body that has turned `a` since the arm was measured.
#[inline(always)]
fn arm(params: &Params, a: &Ang, r: Vec2) -> Vec2 {
    match params.separation {
        Separation::Turned => a.turned.rotate(r),
        Separation::Linear => r + r.turned_by(a.angle),
        Separation::Fixed => r,
    }
}

/// `pass` for a contact with points: Box2D's `b2SolveContact`, all normals
/// then (when relaxing) all frictions, each point at its own arms.
#[inline(always)]
fn pass_points(params: &Params, lin: &mut [Lin], ang: &mut [Ang], r: &Row, t: &mut Turning, inv_h: f32, push: bool) {
    let n = r.normal;
    let dp = lin[r.b()].moved - lin[r.a()].moved;
    let (qa, qb) = (ang[r.a()], ang[r.b()]);
    if !push && params.block && t.count == 2 && block(lin, ang, r, t, inv_h, dp, params) {
        rub_points(lin, ang, r, t);
        return;
    }
    for p in &mut t.p[..t.count] {
        // Box2D's: the separation found, plus how far the arms' ends moved
        // apart along the normal, the normal held fixed through the step.
        let sep = p.base + (dp + arm(params, &qb, p.rb) - arm(params, &qa, p.ra)).dot(n);
        let (bias, mass, relax) = if sep > 0.0 {
            (sep * inv_h, 1.0, 0.0)
        } else if push {
            ((r.soft.rate * sep).max(-MAX_PUSH), r.soft.mass, r.soft.impulse)
        } else {
            (0.0, 1.0, 0.0)
        };
        let vn = (lin[r.b()].v - lin[r.a()].v).dot(n) + ang[r.b()].w * p.rnb - ang[r.a()].w * p.rna;
        let jn = (p.jn - p.normal_mass * mass * (vn + bias) - relax * p.jn).max(0.0);
        let d = jn - p.jn;
        p.jn = jn;
        p.pushed |= jn > 0.0;
        apply_at(lin, ang, r, n * d, (p.rna * d, p.rnb * d));
    }
    if push {
        return;
    }
    rub_points(lin, ang, r, t);
}

/// A relax pass over a two-point contact's normals as one 2x2 LCP
/// (`Params::block`): Box2D v2.4's block solver
/// (`b2ContactSolver::SolveVelocityConstraints`), as Rapier 2D solves a
/// manifold's pairs by default (`solve_pair`), its four cases in turn.
/// False, and nothing applied, where the two rows are near dependent
/// (Box2D's condition number, 1000) or no case holds: the caller then
/// solves them one after the other. Only the relax passes: in the pushing
/// pass too it measured worse (physics.md, "Still at rest").
fn block(lin: &mut [Lin], ang: &mut [Ang], r: &Row, t: &mut Turning, inv_h: f32, dp: Vec2, params: &Params) -> bool {
    let n = r.normal;
    let (a, b) = (r.a(), r.b());
    let (ma, mb, ia, ib) = (lin[a].inv_mass, lin[b].inv_mass, ang[a].inv_inertia, ang[b].inv_inertia);
    let (p1, p2) = (t.p[0], t.p[1]);
    let k11 = ma + mb + ia * p1.rna * p1.rna + ib * p1.rnb * p1.rnb;
    let k22 = ma + mb + ia * p2.rna * p2.rna + ib * p2.rnb * p2.rnb;
    let k12 = ma + mb + ia * p1.rna * p2.rna + ib * p1.rnb * p2.rnb;
    let det = k11 * k22 - k12 * k12;
    if k11 * k11 >= 1000.0 * det {
        return false;
    }
    let (qa, qb) = (ang[a], ang[b]);
    // A gap may close this substep and no more, as in `pass_points`.
    let bias = |p: &Point| {
        let sep = p.base + (dp + arm(params, &qb, p.rb) - arm(params, &qa, p.ra)).dot(n);
        if sep > 0.0 { sep * inv_h } else { 0.0 }
    };
    let vn = |p: &Point| (lin[b].v - lin[a].v).dot(n) + ang[b].w * p.rnb - ang[a].w * p.rna;
    // b' = vn + bias - K a, so that K x + b' is the normal speed (plus bias)
    // once the accumulated impulses are x.
    let (x1, x2) = (p1.jn, p2.jn);
    let b1 = vn(&p1) + bias(&p1) - (k11 * x1 + k12 * x2);
    let b2 = vn(&p2) + bias(&p2) - (k12 * x1 + k22 * x2);
    let inv = 1.0 / det;
    let both = ((k12 * b2 - k22 * b1) * inv, (k12 * b1 - k11 * b2) * inv);
    let first = (-b1 / k11, 0.0);
    let second = (0.0, -b2 / k22);
    let x = if both.0 >= 0.0 && both.1 >= 0.0 {
        both
    } else if first.0 >= 0.0 && k12 * first.0 + b2 >= 0.0 {
        first
    } else if second.1 >= 0.0 && k12 * second.1 + b1 >= 0.0 {
        second
    } else if b1 >= 0.0 && b2 >= 0.0 {
        (0.0, 0.0)
    } else {
        return false;
    };
    let (d1, d2) = (x.0 - x1, x.1 - x2);
    (t.p[0].jn, t.p[1].jn) = x;
    t.p[0].pushed |= x.0 > 0.0;
    t.p[1].pushed |= x.1 > 0.0;
    apply_at(lin, ang, r, n * (d1 + d2), (p1.rna * d1 + p2.rna * d2, p1.rnb * d1 + p2.rnb * d2));
    true
}

/// Friction at each point of a turning contact, as in `pass_points`.
#[inline(always)]
fn rub_points(lin: &mut [Lin], ang: &mut [Ang], r: &Row, t: &mut Turning) {
    let n = r.normal;
    let tangent = n.perp();
    for p in &mut t.p[..t.count] {
        let vt = (lin[r.b()].v - lin[r.a()].v).dot(tangent) + ang[r.b()].w * p.rtb - ang[r.a()].w * p.rta;
        let limit = r.friction * p.jn;
        let jt = (p.jt - p.tangent_mass * vt).clamp(-limit, limit);
        let d = jt - p.jt;
        p.jt = jt;
        apply_at(lin, ang, r, tangent * d, (p.rta * d, p.rtb * d));
    }
}

/// Restitution at each point of a turning contact that closed faster than
/// the threshold and pushed, from its speed before the step.
fn bounce(lin: &mut [Lin], ang: &mut [Ang], r: &Row, t: &mut Turning, (restitution, passes): (f32, usize), out: &mut Points) {
    let n = r.normal;
    // Two points solved one after the other, once each, leave the second's
    // impulse turning the body against the first's: iterated, they
    // converge on bouncing together. Box2D's `b2ApplyRestitution` notes
    // this and passes once.
    let passes = if t.count > 1 { passes } else { 1 };
    for _ in 0..passes {
        for (p, out) in t.p[..t.count].iter_mut().zip(&mut out.point) {
            if p.speed <= BOUNCE_THRESHOLD || !p.pushed {
                continue;
            }
            let vn = (lin[r.b()].v - lin[r.a()].v).dot(n) + ang[r.b()].w * p.rnb - ang[r.a()].w * p.rna;
            let jn = (p.jn - p.normal_mass * (vn - restitution * p.speed)).max(0.0);
            let d = jn - p.jn;
            p.jn = jn;
            out.jn += d;
            apply_at(lin, ang, r, n * d, (p.rna * d, p.rnb * d));
        }
    }
}

/// Applies `impulse` to `b` and its opposite to `a`.
#[inline(always)]
fn apply(lin: &mut [Lin], r: &Row, impulse: Vec2) {
    let (ia, ib) = (lin[r.a()].inv_mass, lin[r.b()].inv_mass);
    lin[r.a()].v -= impulse * ia;
    lin[r.b()].v += impulse * ib;
}

/// Applies `impulse` to `b` and its opposite to `a`, turning each by its arm's
/// cross product with it (`turn`, for `a` and for `b`).
#[inline(always)]
fn apply_at(lin: &mut [Lin], ang: &mut [Ang], r: &Row, impulse: Vec2, (turn_a, turn_b): (f32, f32)) {
    let (a, b) = (r.a(), r.b());
    let ia = lin[a].inv_mass;
    lin[a].v -= impulse * ia;
    ang[a].w -= ang[a].inv_inertia * turn_a;
    let ib = lin[b].inv_mass;
    lin[b].v += impulse * ib;
    ang[b].w += ang[b].inv_inertia * turn_b;
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn ground() -> SolverBody {
        SolverBody::default()
    }

    /// Moving down (y down) at `vy`, with no gravity this step.
    fn falling(vy: f32) -> SolverBody {
        SolverBody::new(Vec2::new(0.0, vy), 1.0, Vec2::ZERO)
    }

    /// Body 0 is the ground, below body 1 (y down): the normal from 1 to 0
    /// points down.
    fn on_ground(depth: f32, restitution: f32) -> Constraint {
        Constraint { a: 1, b: 0, normal: Vec2::new(0.0, 1.0), depth, restitution, friction: 0.5, ..Default::default() }
    }

    #[test]
    fn a_landing_body_stops_without_bouncing() {
        let mut bodies = [ground(), falling(10.0)];
        let mut c = [on_ground(0.0, 0.0)];
        solve(&mut bodies, &mut c, DT);
        assert!(bodies[1].v.y.abs() < 1e-4, "{:?}", bodies[1]);
        assert!((c[0].speed - 10.0).abs() < 1e-4);
    }

    #[test]
    fn a_bouncy_body_bounces_back_at_its_restitution() {
        let mut bodies = [ground(), falling(10.0)];
        solve(&mut bodies, &mut [on_ground(0.0, 0.5)], DT);
        assert!((bodies[1].v.y + 5.0).abs() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn a_speculative_contact_stops_a_body_at_the_surface() {
        let mut bodies = [ground(), falling(10.0)];
        // 0.05 apart: it may move 0.05 this step, and stops there.
        solve(&mut bodies, &mut [on_ground(-0.05, 0.0)], DT);
        assert!(bodies[1].v.y.abs() < 1e-3, "{:?}", bodies[1]);
        assert!((bodies[1].moved.y - 0.05).abs() < 1e-4, "{:?}", bodies[1]);
    }

    #[test]
    fn a_speculative_contact_bounces_at_the_landing_speed() {
        // Restitution from the speed it came in at, not from the gap's
        // (get-emj.19: it rebounded at 3, the gap over the step).
        let mut bodies = [ground(), falling(10.0)];
        solve(&mut bodies, &mut [on_ground(-0.05, 1.0)], DT);
        assert!((bodies[1].v.y + 10.0).abs() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn penetration_is_pushed_out_at_most_max_push_and_leaves_no_speed() {
        let mut bodies = [ground(), falling(0.0)];
        solve(&mut bodies, &mut [on_ground(0.2, 0.0)], DT);
        assert!(bodies[1].v.y.abs() < 1e-4, "no real speed from the correction: {:?}", bodies[1]);
        let moved = -bodies[1].moved.y;
        assert!(moved > 0.9 * MAX_PUSH * DT && moved <= MAX_PUSH * DT + 1e-6, "moved {moved}");
    }

    #[test]
    fn a_shallow_penetration_comes_apart_softly() {
        // Pushed at the soft rate, not all at once.
        let mut bodies = [ground(), falling(0.0)];
        solve(&mut bodies, &mut [on_ground(0.01, 0.0)], DT);
        let moved = -bodies[1].moved.y;
        assert!(moved > 0.001 && moved < 0.01, "moved {moved}");
    }

    #[test]
    fn friction_is_limited_by_the_normal_impulse() {
        // Sliding at 10 while pressed down at 1: friction can take 0.5 of it.
        let mut bodies = [ground(), SolverBody::new(Vec2::new(10.0, 1.0), 1.0, Vec2::ZERO)];
        solve(&mut bodies, &mut [on_ground(0.0, 0.0)], DT);
        assert!((bodies[1].v.x - 9.5).abs() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn gravity_is_held_in_every_substep_and_warm_starting_carries_it() {
        // A resting body under gravity: last step's impulse already holds
        // it, a substep's share at a time.
        let g = Vec2::new(0.0, 40.0 * DT);
        let mut bodies = [ground(), SolverBody::new(g, 1.0, g)];
        let mut c = [on_ground(0.0, 0.0)];
        c[0].jn = 40.0 * DT;
        solve(&mut bodies, &mut c, DT);
        assert!(bodies[1].v.y.abs() < 1e-4, "{:?}", bodies[1]);
        // Sunk only as far as a soft contact gives under the weight.
        assert!(bodies[1].moved.y.abs() < 1e-4, "{:?}", bodies[1]);
        assert!((c[0].jn - 40.0 * DT).abs() < 1e-4, "no more and no less needed: {:?}", c[0]);
    }

    /// A disc of radius 0.5 spun at 10 on the ground, gravity 20, for 240
    /// steps, its contact found again each step at its lowest point: the
    /// disc and its spin after.
    fn roll(params: &Params) -> (SolverBody, f32) {
        let r = 0.5;
        let g = Vec2::new(0.0, 20.0 * DT);
        let mut bodies = [ground(), SolverBody::new(Vec2::ZERO, 1.0, Vec2::ZERO)];
        let mut spin = [Spinning::new(1, 10.0, 2.0 / (r * r))];
        let mut last = ContactPoint { ra: Vec2::new(0.0, r), rb: Vec2::ZERO, separation: 0.0, jn: 20.0 * DT, jt: 0.0 };
        for _ in 0..240 {
            bodies[1].gravity = g;
            bodies[1].v += g;
            let mut c = [Constraint { points: 1, jn: last.jn, jt: last.jt, ..on_ground(0.0, 0.0) }];
            let mut points = [Points { count: 1, point: [last, ContactPoint::default()], solved: false }];
            solve_with(params, (&mut bodies, &mut spin), &mut c, &mut points, DT);
            assert!(points[0].solved);
            // Found again where the disc got to: its lowest point, closer
            // to the ground by how far it fell.
            let separation = last.separation - bodies[1].moved.y;
            last = ContactPoint { jn: points[0].point[0].jn, jt: points[0].point[0].jt, separation, ..last };
        }
        assert!(last.separation.abs() < 1e-3, "{last:?}");
        (bodies[1], spin[0].w)
    }

    #[test]
    fn a_spinning_disc_slows_by_friction_until_it_rolls() {
        // Friction at the contact slows the spin and speeds the disc along
        // until it rolls, which conserves angular momentum about the
        // contact point: I w0 = (I + m r²) w, so w = w0 / 3 for a disc.
        let r = 0.5;
        let (b, w) = roll(&PARAMS);
        assert!((w - 10.0 / 3.0).abs() < 0.05, "{b:?} {w}");
        // Rolling: the contact point is still, v + w × r = 0, and the disc
        // stays on the ground.
        assert!((b.v.x - w * r).abs() < 0.01 && b.moved.y.abs() < 1e-4, "{b:?} {w}");
    }

    #[test]
    fn a_turning_body_bounces_at_its_restitution_at_each_point() {
        // A box landing flat at 10 on two points, restitution 0.5: back up
        // at 5, and not turned by it, the two points pushing alike.
        let land = |params: &Params| {
            let mut bodies = [ground(), falling(10.0)];
            let mut spin = [Spinning::new(1, 0.0, 3.0 / (0.5 * 0.5 + 0.5 * 0.5))];
            let point = |x: f32| ContactPoint { ra: Vec2::new(x, 0.5), rb: Vec2::new(x, -1.0), separation: 0.0, jn: 0.0, jt: 0.0 };
            let mut c = [Constraint { points: 1, ..on_ground(0.0, 0.5) }];
            let mut points = [Points { count: 2, point: [point(-0.5), point(0.5)], solved: false }];
            solve_with(params, (&mut bodies, &mut spin), &mut c, &mut points, DT);
            assert!((c[0].speed - 10.0).abs() < 1e-4, "the fastest point's closing speed");
            (bodies[1].v, spin[0].w)
        };
        let (v, w) = land(&PARAMS);
        assert!((v.y + 5.0).abs() < 1e-2 && w.abs() < 1e-3, "{v:?} {w}");
        // Once over the points, as Box2D passes: short, and turned.
        let (v, w) = land(&Params { bounce: 1, ..PARAMS });
        assert!((v.y + 4.4).abs() < 1e-2 && w.abs() > 1.0, "{v:?} {w}");
    }

    #[test]
    fn the_block_solver_stops_both_points_of_a_box_landing_on_one_corner_first() {
        // A box landing on its two bottom corners, the left one reaching the
        // ground at 4 and the right at 2 (it turns at 2 a second), with one
        // relax pass a substep: solved together, both corners stop; one after
        // the other, the second's impulse turns the box back against the
        // first's and it keeps turning.
        let land = |params: &Params| {
            let mut bodies = [ground(), falling(3.0)];
            let inv_i = 3.0 / (0.5 * 0.5 + 0.5 * 0.5);
            let mut spin = [Spinning::new(1, -2.0, inv_i)];
            let point = |x: f32| ContactPoint { ra: Vec2::new(x, 0.5), rb: Vec2::new(x, -1.0), separation: 0.0, jn: 0.0, jt: 0.0 };
            let mut c = [Constraint { points: 1, friction: 0.0, ..on_ground(0.0, 0.0) }];
            let mut points = [Points { count: 2, point: [point(-0.5), point(0.5)], solved: false }];
            solve_with(params, (&mut bodies, &mut spin), &mut c, &mut points, DT);
            (bodies[1].v, spin[0].w)
        };
        let one = Params { relax: 1, ..PARAMS };
        let (v, w) = land(&Params { block: true, ..one });
        assert!(v.len() < 1e-4 && w.abs() < 1e-4, "block: {v:?} {w}");
        let (v, w) = land(&Params { block: false, ..one });
        assert!(w.abs() > 1e-3, "one after the other: {v:?} {w}");
    }

    #[test]
    fn a_rolling_disc_leaves_a_step_falling_unless_its_arm_is_followed_to_first_order() {
        // docs/lore/a-rolling-disc-leaves-each-step-falling-toward-the-ground-it-rolls-on.md
        let (turned, _) = roll(&PARAMS);
        let (linear, _) = roll(&Params { separation: Separation::Linear, ..PARAMS });
        assert!((turned.v.y - 20.0 * DT / 4.0).abs() < 0.01, "{turned:?}");
        assert!(linear.v.y.abs() < 1e-3, "{linear:?}");
    }

    #[test]
    fn a_tuning_sets_the_substeps_and_none_reads_as_the_default() {
        assert_eq!(Params::of(&physics::Tuning::default()), PARAMS);
        assert_eq!(Params::of(&physics::Tuning::DEFAULT), PARAMS);
        assert_eq!(Params::of(&physics::Tuning { substeps: 6 }), Params { substeps: 6, ..PARAMS });
    }

    #[test]
    fn a_free_body_falls_a_substep_at_a_time() {
        let g = Vec2::new(0.0, 40.0 * DT);
        let mut bodies = [SolverBody::new(g, 1.0, g)];
        solve(&mut bodies, &mut [], DT);
        assert!((bodies[0].v.y - g.y).abs() < 1e-6);
        // Each substep moves at its own speed: g h (1 + 2 + ... + SUBSTEPS).
        let (h, n) = (DT / SUBSTEPS as f32, SUBSTEPS as f32);
        assert!((bodies[0].moved.y - 40.0 * h * h * n * (n + 1.0) / 2.0).abs() < 1e-6, "{:?}", bodies[0]);
    }
}
