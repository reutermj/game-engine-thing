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
//!
//! Where something turns, the passes run four contacts at a time
//! (`lanes`), grouped by Box2D's graph coloring (`Wide::Colored`): about
//! twice as fast as one contact at a time (`solve_all`, which the variants
//! and the tests still run), and bit for bit that loop over the contacts in
//! the colors' order (`order`), which is what a parallel solve over the
//! same colors has to be too. Grouped by level of the pair-order sweep
//! (`Wide::Levels`) it is instead the loop in pair order bit for bit, a
//! variant. Why colors: physics.md, "The solver's speed". The mod runs the
//! colored solve taken apart into its pipeline's stages (`lanes::staged`),
//! its passes a shape the scheduler runs across its threads, the solve on
//! one thread bit for bit (physics.md, "The step", "Solving across
//! threads").

pub use physics_common::{BOUNCE_THRESHOLD, Closing, DAMPING_RATIO, MAX_PUSH, Softness};
use physics2d::{Rot, Vec2};

/// The default's substeps (`physics2d::Tuning`, which a world can change):
/// a soft contact is only as stiff as its substeps are short
/// (`STIFFNESS`).
pub const SUBSTEPS: usize = physics2d::Tuning::DEFAULT.substeps as usize;
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
    /// How a step where something turns orders and lays out its contacts
    /// (`Wide`).
    pub wide: Wide,
    /// What a turning contact's points carry to the next step (`Carry`).
    pub carry: Carry,
    /// What restitution takes a contact's closing speed from (`Closing`).
    pub closing: Closing,
}

/// What a turning contact's points carry out of a step, for the next to
/// warm-start from (`ContactPoint::jn`). The normal's last substep and the
/// tangent's mean is the default, the only choice that meets every bound
/// in every order, colors included (get-emj.61; physics.md, "What a
/// turning point carries: the decision matrix"); the others are the
/// comparison's variants.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Carry {
    /// Both impulses as the last substep left them (times the substeps), as
    /// Box2D carries them: drops more card houses, and colored settles
    /// pyramids 40 and 50 wide later than their bounds (get-emj.54).
    Last,
    /// The normal's last, the tangent's mean over the substeps: the
    /// default. The last normal carries a pyramid's weight from step to
    /// step without the mean's lag; the mean tangent lets a point that
    /// swung to its friction limit within a step stick again.
    Normal,
    /// Both means over the substeps, as before get-emj.48: lets a turning
    /// pyramid solved in any order but pair order fall.
    Mean,
}

impl Carry {
    /// What's carried, from the last substep's impulses and the means, each
    /// (normal, tangent) and counted as a whole step's.
    #[inline(always)]
    fn of(self, last: (f32, f32), mean: (f32, f32)) -> (f32, f32) {
        match self {
            Carry::Last => last,
            Carry::Normal => (last.0, mean.1),
            Carry::Mean => mean,
        }
    }
}

/// How a step where something turns solves its contacts (physics.md, "The
/// solver's speed").
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Wide {
    /// One contact at a time in pair order: the solve as first built, which
    /// the others are checked against, and the only one with `Params::block`
    /// and a `Params::separation` other than `Turned`.
    Off,
    /// By level of the pair-order sweep (`lanes`), that many lanes at once
    /// (1, 4 or 8): `Off`'s computation, bit for bit, at 4 about twice as
    /// fast. A variant: its levels are one thread's schedule, hundreds of
    /// them a pass, too many to share between threads.
    Levels(usize),
    /// Graph-colored as Box2D v3 colors, that many lanes at once: `Off`
    /// over the contacts in the colors' order (`order`), bit for bit. The
    /// default, 4 wide (SSE2's width, x86-64's baseline, where 8 is two
    /// registers): 6 or 7 colors a pass, which threads can share, so a
    /// parallel solve can be this one's computation exactly.
    Colored(usize),
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
    wide: Wide::Colored(4),
    carry: Carry::Normal,
    closing: Closing::Before,
};

impl Params {
    #[allow(dead_code)] // The benches that compile this file solve at the default.
    pub fn of(t: &physics2d::Tuning) -> Params {
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
/// center (world axes), separation, and impulses: in, by feature, what the
/// last step left for this one to start from; out, what this step leaves,
/// which is its last substep's impulse (restitution's included) times the
/// substeps, not their sum. A substep's share of it is where the next
/// step's first substep starts, so the accumulated impulses run on across
/// steps as within them, as Box2D's do (`b2StoreImpulsesTask`,
/// `b2PrepareContactsTask`). Their average would lag two substeps behind
/// at every step, which lets a turning pyramid solved in any order but pair
/// order fall (physics.md, "Why colors let the pyramid fall"). Counted as
/// a whole step's, it still starts a world that changes its substeps where
/// it was. The contact's own impulse (`Constraint::jn`) is the sum, what
/// the step pushed.
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

/// The order a step solves contacts in when grouped by `wide` (`Levels` or
/// `Colored`), as indices into `contacts`: solved one at a time in this
/// order (`Wide::Off`), a step is the grouped one bit for bit. The tests'
/// (`variants.rs`, `rot/order=4`).
#[allow(dead_code)]
pub fn order(wide: Wide, bodies: &[SolverBody], spinning: &[Spinning], contacts: &[Constraint]) -> Vec<usize> {
    lanes::order(wide, bodies, spinning, contacts)
}

/// The solve taken apart for the mod's pipeline of systems.
#[allow(unused_imports)]
pub use lanes::staged;

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
    // though never taken, cost the 5050 pyramid's solve 3%). It stays one
    // contact at a time: level with Box2D there already, and the lanes'
    // arithmetic for a row at its normal isn't this loop's to the bit
    // (it adds its zero turns), where the games' replays hold to it.
    if points.is_empty() && spinning.is_empty() {
        solve_all::<false>(params, (bodies, spinning), contacts, points, dt);
    } else if params.wide == Wide::Off || params.separation != Separation::Turned || params.block {
        solve_all::<true>(params, (bodies, spinning), contacts, points, dt);
    } else {
        match params.wide {
            Wide::Colored(8) | Wide::Levels(8) => lanes::solve::<8>(params, (bodies, spinning), contacts, points, dt),
            Wide::Colored(4) | Wide::Levels(4) => lanes::solve::<4>(params, (bodies, spinning), contacts, points, dt),
            _ => lanes::solve::<1>(params, (bodies, spinning), contacts, points, dt),
        }
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
                let bounce = (params.closing, dt, c.restitution);
                let (t, speed) = prepare((&bodies[a], &ang[a]), (&bodies[b], &ang[b]), c.normal, &points[at], at, warm, bounce);
                c.speed = speed;
                turning.push(t);
                (turning.len() - 1) as u32
            } else {
                c.speed = closing(params.closing, (&bodies[a], &bodies[b]), c, dt);
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
            // The points carry out where the substeps left off, not their
            // sum: `ContactPoint::jn`.
            for (p, out) in t.p[..t.count].iter().zip(&mut out.point) {
                let last = (p.jn * substeps as f32, p.jt * substeps as f32);
                (out.jn, out.jt) = params.carry.of(last, (out.jn, out.jt));
            }
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

/// A contact at its normal's closing speed, as restitution takes it
/// (`Closing`).
#[inline(always)]
fn closing(how: Closing, (a, b): (&SolverBody, &SolverBody), c: &Constraint, dt: f32) -> f32 {
    let (speed, gravity) = (-(b.v - a.v).dot(c.normal), -(b.gravity - a.gravity).dot(c.normal));
    how.speed(speed, gravity, -c.depth, dt, c.restitution)
}

/// A turning contact's points for the substeps, warm-started with `warm`
/// of last step's impulses, and its fastest closing speed as restitution
/// takes it (`Closing`, with the step and the contact's restitution).
#[inline(always)]
fn prepare(
    (a, qa): (&SolverBody, &Ang),
    (b, qb): (&SolverBody, &Ang),
    n: Vec2,
    from: &Points,
    at: usize,
    warm: f32,
    (how, dt, e): (Closing, f32, f32),
) -> (Turning, f32) {
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
        let speed = how.speed(-vr.dot(n), -(b.gravity - a.gravity).dot(n), cp.separation, dt, e);
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

/// The solve of a step where something turns, in lanes: Box2D v3's layout
/// (`contact_solver.c`: `b2ContactConstraintSIMD`,
/// `b2SolveContactTwoPointsTask`), in plain arrays rather than intrinsics.
/// Each step the contacts are split into groups in which no two share a
/// body that moves, each group's contacts, in pair order, packed `N` to a
/// batch, field by field. A pass solves a batch's lanes at once: its bodies
/// gathered from one array of body states, each operation a loop over the
/// lanes, and scattered back.
///
/// The groups are Box2D's graph colors (`constraint_graph.c`,
/// `Wide::Colored`, by `engine_ecs::shape`'s `Coloring`), or the levels
/// of the pair-order sweep (`group`): a contact one level past the latest
/// contact before it that shares a body with it, which is level
/// scheduling, as sparse triangular solves are run in parallel (Anderson
/// and Saad, 1989). Either way no two contacts in a group share a body
/// that moves, so solving the groups in turn, with `pass_points`'
/// arithmetic operation for operation, is the sweep one
/// contact at a time over the groups' order (`order`) bit for bit, whatever
/// the width; for the levels, that order changes no body's, so it is the
/// sweep in pair order. See physics.md, "The solver's speed".
mod lanes {
    use super::*;
    use engine_api::shape::{Colored, Coloring, EMPTY, OVERFLOW, Stage, UNSOLVED};
    use physics_common::lanes::F;
    use std::ops::Range;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A body as the passes read and write it, Box2D's `b2BodyState`: what
    /// a pass reads of a body in one 32-byte line, its masses kept by each
    /// contact instead.
    #[derive(Clone, Copy, Default)]
    #[repr(C, align(32))]
    pub struct State {
        v: Vec2,
        w: f32,
        moved: Vec2,
        turned: Rot,
    }

    /// A batch's bodies at one end, their velocities.
    #[derive(Clone, Copy)]
    struct Vel<const N: usize> {
        x: F<N>,
        y: F<N>,
        w: F<N>,
    }

    /// Where the passes find bodies: the one-thread solve's own array, or
    /// the same states shared between threads (`Shared`). Each kernel is
    /// written once over this, so the two are one computation.
    pub trait Bodies {
        fn load(&self, i: usize) -> State;
        /// Its velocity and turn rate alone, for the passes that read no
        /// more: shared, the loads not made (LLVM keeps an atomic load it
        /// doesn't use, where it drops a plain one).
        fn load_v(&self, i: usize) -> (Vec2, f32);
        fn store_v(&mut self, i: usize, v: Vec2, w: f32);
    }

    impl Bodies for [State] {
        #[inline(always)]
        fn load(&self, i: usize) -> State {
            self[i]
        }

        #[inline(always)]
        fn load_v(&self, i: usize) -> (Vec2, f32) {
            let b = self[i];
            (b.v, b.w)
        }

        #[inline(always)]
        fn store_v(&mut self, i: usize, v: Vec2, w: f32) {
            let b = &mut self[i];
            (b.v, b.w) = (v, w);
        }
    }

    /// A `State` threads share, field by field an `f32`'s bits, loaded and
    /// stored relaxed: on x86 a plain `mov` each, and no unsafe code. Which
    /// thread writes which body is the colors' business: within a stage no
    /// two threads write one field of one body, and a stage's writes reach
    /// the next through the stage's completion (`engine_ecs`'s dispatch).
    #[repr(C, align(32))]
    pub struct Atom([AtomicU32; 7]);

    impl Atom {
        fn new(s: &State) -> Atom {
            Atom([s.v.x, s.v.y, s.w, s.moved.x, s.moved.y, s.turned.c, s.turned.s].map(|x| AtomicU32::new(x.to_bits())))
        }

        #[inline(always)]
        fn get(&self, k: usize) -> f32 {
            f32::from_bits(self.0[k].load(Ordering::Relaxed))
        }

        #[inline(always)]
        fn put(&self, k: usize, x: f32) {
            self.0[k].store(x.to_bits(), Ordering::Relaxed)
        }

        #[inline(always)]
        fn state(&self) -> State {
            let (v, moved) = (Vec2::new(self.get(0), self.get(1)), Vec2::new(self.get(3), self.get(4)));
            State { v, w: self.get(2), moved, turned: Rot { c: self.get(5), s: self.get(6) } }
        }
    }

    /// The bodies as the threads of a solve share them.
    pub struct Shared<'a>(pub &'a [Atom]);

    impl Bodies for Shared<'_> {
        #[inline(always)]
        fn load(&self, i: usize) -> State {
            self.0[i].state()
        }

        #[inline(always)]
        fn load_v(&self, i: usize) -> (Vec2, f32) {
            let a = &self.0[i];
            (Vec2::new(a.get(0), a.get(1)), a.get(2))
        }

        #[inline(always)]
        fn store_v(&mut self, i: usize, v: Vec2, w: f32) {
            let a = &self.0[i];
            a.put(0, v.x);
            a.put(1, v.y);
            a.put(2, w);
        }
    }

    #[inline(always)]
    fn gather<const N: usize, S: Bodies + ?Sized>(s: &S, at: &[u32; N]) -> (Vel<N>, [F<N>; 4]) {
        let (mut v, mut pose) = (Vel { x: F::ZERO, y: F::ZERO, w: F::ZERO }, [F::ZERO; 4]);
        for l in 0..N {
            let b = s.load(at[l] as usize);
            (v.x.0[l], v.y.0[l], v.w.0[l]) = (b.v.x, b.v.y, b.w);
            (pose[0].0[l], pose[1].0[l], pose[2].0[l], pose[3].0[l]) = (b.moved.x, b.moved.y, b.turned.c, b.turned.s);
        }
        (v, pose)
    }

    #[inline(always)]
    fn gather_v<const N: usize, S: Bodies + ?Sized>(s: &S, at: &[u32; N]) -> Vel<N> {
        let mut v = Vel { x: F::ZERO, y: F::ZERO, w: F::ZERO };
        for l in 0..N {
            let (b, w) = s.load_v(at[l] as usize);
            (v.x.0[l], v.y.0[l], v.w.0[l]) = (b.x, b.y, w);
        }
        v
    }

    #[inline(always)]
    fn scatter<const N: usize, S: Bodies + ?Sized>(s: &mut S, at: &[u32; N], v: &Vel<N>) {
        for l in 0..N {
            s.store_v(at[l] as usize, Vec2::new(v.x.0[l], v.y.0[l]), v.w.0[l]);
        }
    }

    /// A point of each lane's contact: `Point`, lane by lane.
    #[derive(Clone, Copy)]
    struct Pt<const N: usize> {
        rax: F<N>,
        ray: F<N>,
        rbx: F<N>,
        rby: F<N>,
        base: F<N>,
        normal_mass: F<N>,
        tangent_mass: F<N>,
        rna: F<N>,
        rnb: F<N>,
        rta: F<N>,
        rtb: F<N>,
        jn: F<N>,
        jt: F<N>,
        /// The substeps' impulses summed, as `Constraint::jn` carries them.
        sum_jn: F<N>,
        sum_jt: F<N>,
        speed: F<N>,
        pushed: [bool; N],
        /// The relax passes' bias, found by the substep's first.
        bias: F<N>,
    }

    /// `N` contacts, field by field. A lane with no contact (the last of a
    /// color) is all zeros at a body nothing else in it moves, so solving
    /// it changes nothing; a contact with one point has a second of zeros.
    #[derive(Clone, Copy)]
    pub struct Batch<const N: usize> {
        a: [u32; N],
        b: [u32; N],
        nx: F<N>,
        ny: F<N>,
        /// Inverse masses and inertias, each end's.
        ma: F<N>,
        mb: F<N>,
        ia: F<N>,
        ib: F<N>,
        friction: F<N>,
        restitution: F<N>,
        rate: F<N>,
        soft_mass: F<N>,
        soft_impulse: F<N>,
        p: [Pt<N>; 2],
        /// Whether it has two points, which restitution passes over
        /// `Params::bounce` times, and whether it's a row at its normal (its
        /// ends don't turn), which bounces if it pushed over the step.
        two: [bool; N],
        linear: [bool; N],
        /// The ends' velocities as the warm start leaves them: kept, so
        /// that its lanes are computed as vectors (the stores seed LLVM's
        /// SLP vectorizer, which left them scalar without).
        ends: [Vel<N>; 2],
    }

    impl<const N: usize> Batch<N> {
        fn empty(nowhere: u32) -> Batch<N> {
            let p = Pt {
                rax: F::ZERO,
                ray: F::ZERO,
                rbx: F::ZERO,
                rby: F::ZERO,
                base: F::ZERO,
                normal_mass: F::ZERO,
                tangent_mass: F::ZERO,
                rna: F::ZERO,
                rnb: F::ZERO,
                rta: F::ZERO,
                rtb: F::ZERO,
                jn: F::ZERO,
                jt: F::ZERO,
                sum_jn: F::ZERO,
                sum_jt: F::ZERO,
                speed: F::ZERO,
                pushed: [false; N],
                bias: F::ZERO,
            };
            Batch {
                a: [nowhere; N],
                b: [nowhere; N],
                nx: F::ZERO,
                ny: F::ZERO,
                ma: F::ZERO,
                mb: F::ZERO,
                ia: F::ZERO,
                ib: F::ZERO,
                friction: F::ZERO,
                restitution: F::ZERO,
                rate: F::ZERO,
                soft_mass: F::ZERO,
                soft_impulse: F::ZERO,
                p: [p; 2],
                two: [false; N],
                linear: [false; N],
                ends: [Vel { x: F::ZERO, y: F::ZERO, w: F::ZERO }; 2],
            }
        }
    }

    /// Where a lane's results go: its contact, and its points (`NONE` for a
    /// row at its normal) and how many; and its closing speed, which
    /// `finish` writes to the contact.
    #[derive(Clone, Copy)]
    struct Lane {
        contact: u32,
        at: u32,
        count: u8,
        speed: f32,
    }

    impl Lane {
        const NONE: Lane = Lane { contact: NONE, at: NONE, count: 0, speed: 0.0 };
    }

    /// A `Lane` as relaxed atomics of its fields' bits: the staged solve's
    /// fill writes each batch's lanes through the shared reference every
    /// kernel has, as the `Move` stage writes the angles, while the passes
    /// get the batches alone. Apart from the batches so the passes stream
    /// nothing they don't read and `finish` reads the lanes packed; not in
    /// a stage's items, which would have the passes go through a reference
    /// a batch. (History, 2026-10-03: as items of a batch and its lanes,
    /// references to each, the settled pile's solve on one thread was
    /// 1.4-1.8% slower than before the fill was a stage, and with these
    /// 0.7%; with the lanes in the batch, `finish` read a line more a
    /// batch, 155 µs against 112.)
    #[derive(Default)]
    struct LaneCell([AtomicU32; 4]);

    impl LaneCell {
        #[inline(always)]
        fn set(&self, lane: Lane) {
            let bits = [lane.contact, lane.at, lane.count as u32, lane.speed.to_bits()];
            for (x, b) in self.0.iter().zip(bits) {
                x.store(b, Ordering::Relaxed);
            }
        }

        #[inline(always)]
        fn get(&self) -> Lane {
            let [contact, at, count, speed] = self.0.each_ref().map(|x| x.load(Ordering::Relaxed));
            Lane { contact, at, count: count as u8, speed: f32::from_bits(speed) }
        }
    }

    const NONE: u32 = u32::MAX;
    /// How a pass finds a point's bias (`pass`'s `SEP`): from its separation,
    /// storing it too, or as the last pass stored it.
    const COMPUTE: u8 = 0;
    const STORE: u8 = 1;
    const LOAD: u8 = 2;

    /// Each contact's group, in which no two share a body that moves: a
    /// body that doesn't move (a static, a kinematic, the one standing for
    /// sleeping bodies) can be in any number of a group's contacts, since
    /// none of them changes it.
    ///
    /// Colored (`Wide::Colored`): Box2D v3's greedy coloring in pair order,
    /// not color 0 for a contact with an end that doesn't move
    /// (`b2AddContactToGraph`), which is the generic primitive's
    /// (`Coloring::greedy`), the same call the mod's pipeline makes, so the
    /// arrays and the mod can't color apart. By level (`Wide::Levels`), a
    /// variant: `physics_common::levels`, which 3D's lanes solve in too. A
    /// pile of 10 000 has about 420 levels of 50 contacts, its batches 97%
    /// full; a 5050 pyramid 590, 94%. Both are packed into batches alike
    /// (`Coloring::pack`).
    fn group(contacts: &[Constraint], moves: &[bool], wide: Wide) -> Coloring {
        let ends = |i: usize| (contacts[i].a, contacts[i].b);
        if let Wide::Colored(_) = wide {
            let mut coloring = Coloring::default();
            coloring.greedy(contacts.len(), ends, moves, true, &mut Vec::new());
            return coloring;
        }
        physics_common::levels(contacts.len(), ends, moves)
    }

    /// What `solve` solves them in: the overflow's first, then each group's
    /// in pair order, and last the contacts it leaves unsolved, which change
    /// no body. Found by `group` over the same bodies, so an order and a
    /// grouping can't disagree.
    pub fn order(wide: Wide, bodies: &[SolverBody], spinning: &[Spinning], contacts: &[Constraint]) -> Vec<usize> {
        let mut inertia = vec![0.0f32; bodies.len()];
        for sp in spinning {
            inertia[sp.body as usize] = sp.inv_inertia;
        }
        let moves: Vec<bool> = bodies.iter().zip(inertia.iter()).map(|(b, i)| b.inv_mass > 0.0 || *i > 0.0).collect();
        let coloring = group(contacts, &moves, wide);
        let n_groups = coloring.count.len();
        let rank = |k: u32| match k {
            OVERFLOW => 0,
            UNSOLVED => n_groups + 1,
            k => k as usize + 1,
        };
        let mut at: Vec<usize> = (0..contacts.len()).collect();
        // Stable, so pair order within each.
        at.sort_by_key(|&i| rank(coloring.of[i]));
        at
    }

    /// What every way of solving starts from: the bodies as states, and
    /// what stays fixed through the step.
    struct Head {
        s: Vec<State>,
        k: Fixed,
    }

    /// The step's constants and each body's inverse inertia: what a
    /// contact's start reads besides the states' turn rates, apart from the
    /// states so a stage can read it while the states are lent to the
    /// passes (`staged`'s fill).
    struct Fixed {
        inertia: Vec<f32>,
        nowhere: u32,
        h: f32,
        inv_h: f32,
        share: f32,
        warm: f32,
        soft: (Softness, Softness),
    }

    impl Head {
        fn empty() -> Head {
            let soft = Softness { rate: 0.0, mass: 0.0, impulse: 0.0 };
            let k = Fixed { inertia: Vec::new(), nowhere: 0, h: 0.0, inv_h: 0.0, share: 0.0, warm: 0.0, soft: (soft, soft) };
            Head { s: Vec::new(), k }
        }

        /// The step's constants, and the bodies as states into `s` and
        /// `inertia`, keeping their allocations; whether each body moves
        /// into `moves`, which the contacts are grouped by (`head`, or the
        /// staged solve's caller).
        #[inline(always)]
        fn start(&mut self, params: &Params, (bodies, spinning): (&[SolverBody], &[Spinning]), dt: f32, moves: &mut Vec<bool>) {
            let substeps = params.substeps;
            let h = dt / substeps as f32;
            let inv_h = 1.0 / h;
            let moving = Softness::new(params.stiffness * inv_h, DAMPING_RATIO, h);
            let fixed = Softness::new(params.static_stiffness * inv_h, DAMPING_RATIO, h);
            let share = 1.0 / substeps as f32;
            let k = &mut self.k;
            (k.h, k.inv_h, k.share, k.soft) = (h, inv_h, share, (moving, fixed));
            k.warm = if params.warm { share } else { 0.0 };
            // One more state than bodies, standing still: where a batch's
            // empty lanes point, so writing them back can't undo a real
            // lane's write.
            k.nowhere = bodies.len() as u32;
            let (s, inertia) = (&mut self.s, &mut k.inertia);
            s.clear();
            s.extend(bodies.iter().map(|b| State { v: b.v, ..State::default() }));
            s.push(State::default());
            inertia.clear();
            inertia.resize(bodies.len(), 0.0);
            for sp in spinning.iter() {
                s[sp.body as usize].w = sp.w;
                inertia[sp.body as usize] = sp.inv_inertia;
            }
            moves.clear();
            moves.extend(bodies.iter().zip(inertia.iter()).map(|(b, i)| b.inv_mass > 0.0 || *i > 0.0));
        }
    }

    /// The step's start, and its contacts grouped and laid out in batches:
    /// the overflow's first, one contact each (as Box2D solves its overflow
    /// first), then each group's, in pair order; and each contact's batch
    /// and lane, `None` for one left unsolved.
    #[inline(always)]
    #[allow(clippy::type_complexity)]
    fn head<const N: usize>(
        params: &Params,
        (bodies, spinning): (&[SolverBody], &[Spinning]),
        contacts: &[Constraint],
        dt: f32,
    ) -> (Head, Colored, Vec<Option<(u32, u32)>>) {
        let mut hd = Head::empty();
        let mut moves = Vec::new();
        hd.start(params, (bodies, spinning), dt, &mut moves);
        let mut place = Vec::new();
        let layout = group(contacts, &moves, params.wide).pack(N, &mut place);
        (hd, layout, place)
    }

    /// A contact as the substeps start it: its points (or its row's), where
    /// they are in `points` (`NONE` for a row at its normal) and how many,
    /// and its closing speed. `(jn, jt)` are its impulses as the last step
    /// left them, which it may no longer hold (`setup` clears them).
    #[inline(always)]
    fn start<B: Bodies + ?Sized>(
        params: &Params,
        c: &Constraint,
        (jn, jt): (f32, f32),
        bodies: &[SolverBody],
        (k, s): (&Fixed, &B),
        points: &[Points],
        dt: f32,
    ) -> ([Point; 2], u32, usize, f32) {
        let (a, b) = (c.a as usize, c.b as usize);
        let inertia = &k.inertia;
        let w = |i: usize| s.load_v(i).1;
        let spins = |i: usize| inertia[i] > 0.0 || w(i) != 0.0;
        let turns = c.points > 0 && (spins(a) || spins(b));
        let mut pts = [Point::default(); 2];
        if turns {
            let at = c.points as usize - 1;
            let ang = |i: usize| Ang { w: w(i), inv_inertia: inertia[i], turned: Rot::IDENTITY, angle: 0.0 };
            let bounce = (params.closing, dt, c.restitution);
            let (t, speed) = prepare((&bodies[a], &ang(a)), (&bodies[b], &ang(b)), c.normal, &points[at], at, k.warm, bounce);
            (t.p, at as u32, t.count, speed)
        } else {
            let speed = super::closing(params.closing, (&bodies[a], &bodies[b]), c, dt);
            let m = bodies[a].inv_mass + bodies[b].inv_mass;
            let mass = if m > 0.0 { 1.0 / m } else { 0.0 };
            pts[0] = Point {
                base: -c.depth,
                normal_mass: mass,
                tangent_mass: mass,
                jn: jn * k.share,
                jt: jt * k.share,
                speed,
                ..Point::default()
            };
            (pts, NONE, 1, speed)
        }
    }

    /// Contact `i` into lane `l` of its batch, as `start` started it; the
    /// lane's record, for the caller to keep.
    #[inline(always)]
    fn put<const N: usize>(
        o: &mut Batch<N>,
        l: usize,
        (i, c): (usize, &Constraint),
        bodies: &[SolverBody],
        k: &Fixed,
        (pts, at, n_points, speed): (&[Point; 2], u32, usize, f32),
    ) -> Lane {
        let (a, b) = (c.a as usize, c.b as usize);
        let (ma, mb) = (bodies[a].inv_mass, bodies[b].inv_mass);
        let (moving, fixed) = k.soft;
        let soft = if ma == 0.0 || mb == 0.0 { fixed } else { moving };
        o.a[l] = a as u32;
        o.b[l] = b as u32;
        (o.nx.0[l], o.ny.0[l]) = (c.normal.x, c.normal.y);
        (o.ma.0[l], o.mb.0[l]) = (ma, mb);
        (o.ia.0[l], o.ib.0[l]) = (k.inertia[a], k.inertia[b]);
        (o.friction.0[l], o.restitution.0[l]) = (c.friction, c.restitution);
        (o.rate.0[l], o.soft_mass.0[l], o.soft_impulse.0[l]) = (soft.rate, soft.mass, soft.impulse);
        (o.two[l], o.linear[l]) = (n_points > 1, at == NONE);
        for (q, p) in o.p.iter_mut().zip(pts.iter()).take(n_points) {
            (q.rax.0[l], q.ray.0[l], q.rbx.0[l], q.rby.0[l]) = (p.ra.x, p.ra.y, p.rb.x, p.rb.y);
            (q.base.0[l], q.normal_mass.0[l], q.tangent_mass.0[l]) = (p.base, p.normal_mass, p.tangent_mass);
            (q.rna.0[l], q.rnb.0[l], q.rta.0[l], q.rtb.0[l]) = (p.rna, p.rnb, p.rta, p.rtb);
            (q.jn.0[l], q.jt.0[l], q.speed.0[l]) = (p.jn, p.jt, p.speed);
        }
        Lane { contact: i as u32, at, count: n_points as u8, speed }
    }

    /// What the solve one contact at a time gives a contact neither end of
    /// which moves: last step's impulse, a substep's share of it summed, and
    /// its points that share times the substeps (`ContactPoint::jn`), which
    /// `kept` keeps for them.
    #[inline(always)]
    fn unsolved(
        params: &Params,
        c: &mut Constraint,
        (pts, at, n_points): (&[Point; 2], u32, usize),
        kept: &mut Vec<(usize, [(f32, f32); 2])>,
    ) {
        let substeps = params.substeps;
        let sum = |x: f32| (0..substeps).fold(0.0, |acc, _| acc + x);
        if at == NONE {
            (c.jn, c.jt) = (sum(pts[0].jn), sum(pts[0].jt));
        } else {
            let (mut j, mut last) = ([(0.0, 0.0); 2], [(0.0, 0.0); 2]);
            for ((j, last), p) in j.iter_mut().zip(last.iter_mut()).zip(pts.iter()).take(n_points) {
                *j = (sum(p.jn), sum(p.jt));
                *last = params.carry.of((p.jn * substeps as f32, p.jt * substeps as f32), *j);
            }
            c.jn = j[..n_points].iter().map(|j| j.0).sum();
            c.jt = j[..n_points].iter().map(|j| j.1).sum();
            kept.push((at as usize, last));
        }
    }

    /// Every point's impulses cleared, as a step starts them, but for those
    /// of contacts that aren't solved (`kept`); `solved` those solved at
    /// their points.
    #[inline(always)]
    fn clear(points: &mut [Points], solved: impl IntoIterator<Item = usize>, kept: &[(usize, [(f32, f32); 2])]) {
        for p in points.iter_mut() {
            p.solved = false;
            for q in p.point.iter_mut() {
                (q.jn, q.jt) = (0.0, 0.0);
            }
        }
        for at in solved {
            points[at].solved = true;
        }
        for (at, j) in kept {
            for (q, j) in points[*at].point.iter_mut().zip(j) {
                (q.jn, q.jt) = *j;
            }
        }
    }

    /// `clear` of a run of the points, the first `at`: each cleared, then
    /// those of `solved` and `kept` in the run, which are in point order.
    #[inline(always)]
    fn clear_run(points: &mut [Points], at: usize, solved: &[usize], kept: &[(usize, [(f32, f32); 2])]) {
        for p in points.iter_mut() {
            p.solved = false;
            for q in p.point.iter_mut() {
                (q.jn, q.jt) = (0.0, 0.0);
            }
        }
        let run = at..at + points.len();
        let from = solved.partition_point(|p| *p < run.start);
        for p in solved[from..].iter().take_while(|p| run.contains(p)) {
            points[p - at].solved = true;
        }
        let from = kept.partition_point(|(p, _)| *p < run.start);
        for (p, j) in kept[from..].iter().take_while(|(p, _)| run.contains(p)) {
            for (q, j) in points[p - at].point.iter_mut().zip(j) {
                (q.jn, q.jt) = *j;
            }
        }
    }

    /// Each contact in pair order, as the substeps start it, into the batch
    /// and lane `place` gives it, or left unsolved where it gives none:
    /// `setup`'s loop. Read in the order they're stored, written where
    /// they're solved. (History, 2026-09-27: filled batch by batch instead,
    /// reading contacts in the order they're solved, the prepare took 1223
    /// µs against 887 on a pile of 10 000; the staged solve fills batch by
    /// batch now, `fill`, measured in physics.md, "Solving across threads".)
    /// Into `solved`, the points solved at; into `kept`, the impulses
    /// points end with where they aren't solved.
    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    fn enter<const N: usize>(
        params: &Params,
        hd: &Head,
        bodies: &[SolverBody],
        (contacts, points): (&mut [Constraint], &[Points]),
        dt: f32,
        mut place: impl FnMut(usize) -> Option<(usize, usize)>,
        (out, lanes): (&mut [Batch<N>], &mut [[Lane; N]]),
        (solved, kept): (&mut Vec<usize>, &mut Vec<(usize, [(f32, f32); 2])>),
    ) {
        for (i, c) in contacts.iter_mut().enumerate() {
            let (jn, jt) = (c.jn, c.jt);
            (c.jn, c.jt) = (0.0, 0.0);
            let (pts, at, n_points, speed) = start(params, c, (jn, jt), bodies, (&hd.k, &hd.s[..]), points, dt);
            c.speed = speed;
            if at != NONE {
                solved.push(at as usize);
            }
            let Some((batch, l)) = place(i) else {
                unsolved(params, c, (&pts, at, n_points), kept);
                continue;
            };
            lanes[batch][l] = put(&mut out[batch], l, (i, c), bodies, &hd.k, (&pts, at, n_points, speed));
        }
    }

    /// Batches `at..` as the substeps start them, each from the contacts
    /// its lanes seat (`Coloring::seat`, `EMPTY` an empty lane): `enter`,
    /// batch by batch, for the staged solve's first stage, which threads
    /// share by batches. Each batch and its lanes (`lanes[at..]`) are
    /// written whole, so what they held before is never read; and each
    /// contact's seat, into `seat_of`, for the write-back. Reads the
    /// contacts and points as the last step left them, which
    /// `staged::Staged::finish` changes only after.
    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    fn fill<const N: usize, B: Bodies + ?Sized>(
        params: &Params,
        (k, s): (&Fixed, &B),
        bodies: &[SolverBody],
        (contacts, points): (&[Constraint], &[Points]),
        dt: f32,
        (seats, lanes, seat_of, at): (&[u32], &[[LaneCell; N]], &[AtomicU32], usize),
        out: &mut [Batch<N>],
    ) {
        let (seats, lanes) = (seats[at * N..].as_chunks::<N>().0, &lanes[at..]);
        for (batch, ((o, seats), cells)) in (at..).zip(out.iter_mut().zip(seats).zip(lanes)) {
            *o = Batch::empty(k.nowhere);
            for (l, (&i, cell)) in seats.iter().zip(cells).enumerate() {
                if i == EMPTY {
                    cell.set(Lane::NONE);
                    continue;
                }
                let c = &contacts[i as usize];
                let (pts, at, n_points, speed) = start(params, c, (c.jn, c.jt), bodies, (k, s), points, dt);
                cell.set(put(o, l, (i as usize, c), bodies, k, (&pts, at, n_points, speed)));
                seat_of[i as usize].store((batch * N + l) as u32, Ordering::Relaxed);
            }
        }
    }

    /// `fill` of every batch, contact by contact in pair order, each into
    /// the seat `coloring` gives it: the same values, read in the order the
    /// contacts are stored. On one thread a quarter faster than `fill`,
    /// whose reads jump between the colors' contacts (the settled pile of
    /// 10 000: 390 µs against 530, step_bench, 2026-10-03), but it writes
    /// any batch, so it's the fill of a block that has them all.
    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    fn fill_all<const N: usize, B: Bodies + ?Sized>(
        params: &Params,
        (k, s): (&Fixed, &B),
        bodies: &[SolverBody],
        (contacts, points): (&[Constraint], &[Points]),
        dt: f32,
        (coloring, lanes, seat_of): (&Coloring, &[[LaneCell; N]], &[AtomicU32]),
        out: &mut [Batch<N>],
    ) {
        out.fill(Batch::empty(k.nowhere));
        lanes.iter().flatten().for_each(|cell| cell.set(Lane::NONE));
        for ((i, c), to) in contacts.iter().enumerate().zip(coloring.seats(N)) {
            let Some(to) = to else { continue };
            let (pts, at, n_points, speed) = start(params, c, (c.jn, c.jt), bodies, (k, s), points, dt);
            let (batch, l) = (to / N, to % N);
            lanes[batch][l].set(put(&mut out[batch], l, (i, c), bodies, k, (&pts, at, n_points, speed)));
            seat_of[i].store(to as u32, Ordering::Relaxed);
        }
    }

    /// A step's contacts as the passes solve them on one thread, between
    /// `setup` and `finish`.
    struct Solve<const N: usize> {
        s: Vec<State>,
        out: Vec<Batch<N>>,
        lanes: Vec<[Lane; N]>,
        gravity: Vec<Vec2>,
        angle: Vec<f32>,
        h: f32,
        inv_h: f32,
        share: f32,
    }

    #[inline(always)]
    pub fn solve<const N: usize>(
        params: &Params,
        (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
        contacts: &mut [Constraint],
        points: &mut [Points],
        dt: f32,
    ) {
        let mut p = setup::<N>(params, (bodies, spinning), contacts, points, dt);
        run(&mut p, params, spinning);
        let Solve { s, out, lanes, angle, .. } = p;
        finish(params, out.iter().zip(lanes.iter().copied()), (bodies, spinning), (&s[..], angle), contacts, points);
    }

    /// The substeps' start: bodies into states, contacts colored (or
    /// levelled) and into their batches' lanes.
    #[inline(always)]
    fn setup<const N: usize>(
        params: &Params,
        (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
        contacts: &mut [Constraint],
        points: &mut [Points],
        dt: f32,
    ) -> Solve<N> {
        let (hd, layout, place) = head::<N>(params, (bodies, spinning), contacts, dt);
        let mut out: Vec<Batch<N>> = vec![Batch::empty(hd.k.nowhere); layout.items()];
        let mut lanes: Vec<[Lane; N]> = vec![[Lane::NONE; N]; layout.items()];

        let mut kept: Vec<(usize, [(f32, f32); 2])> = Vec::new();
        let mut solved: Vec<usize> = Vec::new();
        let place = |i: usize| place[i].map(|(batch, l)| (batch as usize, l as usize));
        enter(params, &hd, bodies, (contacts, points), dt, place, (&mut out, &mut lanes), (&mut solved, &mut kept));
        clear(points, solved, &kept);
        let Head { mut s, k: Fixed { h, inv_h, share, .. } } = hd;
        let gravity: Vec<Vec2> = bodies.iter().map(|b| b.gravity).collect();
        for (st, g) in s.iter_mut().zip(gravity.iter()) {
            st.v -= *g;
        }
        let angle = vec![0.0f32; spinning.len()];
        Solve { s, out, lanes, gravity, angle, h, inv_h, share }
    }

    /// The substeps' passes and restitution, on one thread.
    #[inline(always)]
    fn run<const N: usize>(p: &mut Solve<N>, params: &Params, spinning: &[Spinning]) {
        let Solve { s, out, gravity, angle, h, inv_h, share, .. } = p;
        let (s, h, inv_h, share, substeps) = (s.as_mut_slice(), *h, *inv_h, *share, params.substeps);

        for _ in 0..substeps {
            for (st, g) in s.iter_mut().zip(gravity.iter()) {
                st.v += *g * share;
            }
            for o in out.iter_mut() {
                warm_start(o, &mut *s);
            }
            let last = params.relax == 0;
            for o in out.iter_mut() {
                pass::<N, true, COMPUTE>(o, &mut *s, inv_h, last);
            }
            for st in s.iter_mut() {
                st.moved += st.v * h;
            }
            for (sp, angle) in spinning.iter().zip(angle.iter_mut()) {
                let b = &mut s[sp.body as usize];
                *angle += h * b.w;
                b.turned = match params.integrate {
                    Integrate::Rotation => b.turned.integrate(h * b.w),
                    Integrate::Angle => Rot::from_angle(*angle),
                };
            }
            for r in 0..params.relax {
                let last = r + 1 == params.relax;
                // Positions don't move between relax passes, so neither do
                // separations: the first finds each point's bias, the
                // others read it.
                if r == 0 {
                    for o in out.iter_mut() {
                        pass::<N, false, STORE>(o, &mut *s, inv_h, last);
                    }
                } else {
                    for o in out.iter_mut() {
                        pass::<N, false, LOAD>(o, &mut *s, inv_h, last);
                    }
                }
            }
        }

        // Restitution, once, as `bounce`.
        for o in out.iter_mut() {
            if o.restitution.0.iter().any(|e| *e != 0.0) {
                restitute(o, &mut *s, params.bounce);
            }
        }
    }

    /// The step's impulses into the contacts and points, from the batches'
    /// lanes in order, each point solved at marked so (as `clear` marks
    /// it), and the states back into the bodies.
    #[inline(always)]
    fn finish<'a, const N: usize>(
        params: &Params,
        batches: impl Iterator<Item = (&'a Batch<N>, impl IntoIterator<Item = Lane>)>,
        (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
        (s, angle): (&(impl Bodies + ?Sized), impl IntoIterator<Item = f32>),
        contacts: &mut [Constraint],
        points: &mut [Points],
    ) {
        let substeps = params.substeps;
        for (o, lanes) in batches {
            for (l, lane) in lanes.into_iter().enumerate() {
                if lane.contact == NONE {
                    continue;
                }
                let c = &mut contacts[lane.contact as usize];
                (c.jn, c.jt) = impulses(o, l, &lane);
                c.speed = lane.speed;
                if lane.at != NONE {
                    let to = &mut points[lane.at as usize];
                    to.solved = true;
                    carried(params.carry, substeps, o, l, lane.count as usize, to);
                }
            }
        }
        for (i, b) in bodies.iter_mut().enumerate() {
            let st = s.load(i);
            (b.v, b.moved) = (st.v, st.moved);
        }
        for (sp, angle) in spinning.iter_mut().zip(angle) {
            let st = s.load(sp.body as usize);
            (sp.w, sp.turned, sp.angle) = (st.w, st.turned, angle);
        }
    }

    /// A contact's impulses over the step, from its lane: `finish`'s.
    #[inline(always)]
    fn impulses<const N: usize>(o: &Batch<N>, l: usize, lane: &Lane) -> (f32, f32) {
        if lane.at == NONE {
            return (o.p[0].sum_jn.0[l], o.p[0].sum_jt.0[l]);
        }
        let n = lane.count as usize;
        (o.p[..n].iter().map(|p| p.sum_jn.0[l]).sum(), o.p[..n].iter().map(|p| p.sum_jt.0[l]).sum())
    }

    /// What a contact's points carry to the next step, from its lane:
    /// `finish`'s.
    #[inline(always)]
    fn carried<const N: usize>(carry: Carry, substeps: usize, o: &Batch<N>, l: usize, n: usize, to: &mut Points) {
        for (q, p) in to.point.iter_mut().zip(o.p.iter()).take(n) {
            let last = (p.jn.0[l] * substeps as f32, p.jt.0[l] * substeps as f32);
            (q.jn, q.jt) = carry.of(last, (p.sum_jn.0[l], p.sum_jt.0[l]));
        }
    }

    /// The ends' velocities after an impulse `(ix, iy)` on `b` and its
    /// opposite on `a`, turning each by `ta` and `tb`: `apply_at`.
    #[inline(always)]
    fn apply<const N: usize>(o: &Batch<N>, va: &mut Vel<N>, vb: &mut Vel<N>, (ix, iy): (F<N>, F<N>), (ta, tb): (F<N>, F<N>)) {
        va.x = va.x - ix * o.ma;
        va.y = va.y - iy * o.ma;
        va.w = va.w - o.ia * ta;
        vb.x = vb.x + ix * o.mb;
        vb.y = vb.y + iy * o.mb;
        vb.w = vb.w + o.ib * tb;
    }

    /// Last substep's impulses again, as the substeps' warm start.
    #[inline(always)]
    fn warm_start<const N: usize>(o: &mut Batch<N>, s: &mut (impl Bodies + ?Sized)) {
        let (mut va, mut vb) = (gather_v(&*s, &o.a), gather_v(&*s, &o.b));
        let (tx, ty) = (-o.ny, o.nx);
        for p in o.p.iter() {
            let i = (o.nx * p.jn + tx * p.jt, o.ny * p.jn + ty * p.jt);
            let turn = (p.rna * p.jn + p.rta * p.jt, p.rnb * p.jn + p.rtb * p.jt);
            apply(o, &mut va, &mut vb, i, turn);
        }
        o.ends = [va, vb];
        scatter(s, &o.a, &o.ends[0]);
        scatter(s, &o.b, &o.ends[1]);
    }

    /// `pass_points`, lane by lane: soft and pushing out when `PUSH`, else
    /// rigid, with friction; `last`, the substep's last, sums its impulses.
    #[inline(always)]
    fn pass<const N: usize, const PUSH: bool, const SEP: u8>(o: &mut Batch<N>, s: &mut (impl Bodies + ?Sized), inv_h: f32, last: bool) {
        let ((mut va, pa), (mut vb, pb)) = (gather(&*s, &o.a), gather(&*s, &o.b));
        let (dx, dy) = (pb[0] - pa[0], pb[1] - pa[1]);
        let (nx, ny) = (o.nx, o.ny);
        let one = F::splat(1.0);
        let (rate, soft_mass, soft_impulse, ma, mb, ia, ib) = (o.rate, o.soft_mass, o.soft_impulse, o.ma, o.mb, o.ia, o.ib);
        let apply = |va: &mut Vel<N>, vb: &mut Vel<N>, (ix, iy): (F<N>, F<N>), (ta, tb): (F<N>, F<N>)| {
            va.x = va.x - ix * ma;
            va.y = va.y - iy * ma;
            va.w = va.w - ia * ta;
            vb.x = vb.x + ix * mb;
            vb.y = vb.y + iy * mb;
            vb.w = vb.w + ib * tb;
        };
        for p in o.p.iter_mut() {
            let (bias, mass, relax) = if !PUSH && SEP == LOAD {
                (p.bias, one, F::ZERO)
            } else {
                // The arms turned with their bodies: `Rot::rotate`.
                let (bx, by) = (pb[2] * p.rbx - pb[3] * p.rby, pb[3] * p.rbx + pb[2] * p.rby);
                let (ax, ay) = (pa[2] * p.rax - pa[3] * p.ray, pa[3] * p.rax + pa[2] * p.ray);
                let sep = p.base + (((dx + bx) - ax) * nx + ((dy + by) - ay) * ny);
                // A gap may close this substep, and no more: speculative, in
                // either pass.
                let spec = sep * F::splat(inv_h);
                if PUSH {
                    let soft = (rate * sep).max(F::splat(-MAX_PUSH));
                    (sep.positive_then(spec, soft), sep.positive_then(one, soft_mass), sep.positive_then(F::ZERO, soft_impulse))
                } else {
                    let bias = sep.positive_then(spec, F::ZERO);
                    if SEP == STORE {
                        p.bias = bias;
                    }
                    (bias, one, F::ZERO)
                }
            };
            let vn = ((vb.x - va.x) * nx + (vb.y - va.y) * ny) + vb.w * p.rnb - va.w * p.rna;
            let jn = (p.jn - p.normal_mass * mass * (vn + bias) - relax * p.jn).max(F::ZERO);
            let d = jn - p.jn;
            p.jn = jn;
            for l in 0..N {
                p.pushed[l] |= jn.0[l] > 0.0;
            }
            apply(&mut va, &mut vb, (nx * d, ny * d), (p.rna * d, p.rnb * d));
        }
        if !PUSH {
            let (tx, ty) = (-ny, nx);
            let friction = o.friction;
            for p in o.p.iter_mut() {
                let vt = ((vb.x - va.x) * tx + (vb.y - va.y) * ty) + vb.w * p.rtb - va.w * p.rta;
                let limit = friction * p.jn;
                let jt = (p.jt - p.tangent_mass * vt).clamp(-limit, limit);
                let d = jt - p.jt;
                p.jt = jt;
                apply(&mut va, &mut vb, (tx * d, ty * d), (p.rta * d, p.rtb * d));
            }
        }
        if last {
            for p in o.p.iter_mut() {
                p.sum_jn = p.sum_jn + p.jn;
                p.sum_jt = p.sum_jt + p.jt;
            }
        }
        scatter(s, &o.a, &va);
        scatter(s, &o.b, &vb);
    }

    /// `bounce`, lane by lane, and for a row at its normal the solve as
    /// first built's rule: it bounces if it pushed over the step.
    fn restitute<const N: usize>(o: &mut Batch<N>, s: &mut (impl Bodies + ?Sized), passes: usize) {
        // Whether a lane's point bounces: a later pass only ever bounces
        // what the first could, so a batch the first can't is left alone.
        let bounces = |o: &Batch<N>, k: usize, l: usize, pass: usize| {
            let p = &o.p[k];
            let hit = if o.linear[l] { p.sum_jn.0[l] != 0.0 } else { p.pushed[l] };
            let again = pass == 0 || o.two[l];
            o.restitution.0[l] != 0.0 && again && p.speed.0[l] > BOUNCE_THRESHOLD && hit
        };
        if !(0..2).any(|k| (0..N).any(|l| bounces(o, k, l, 0))) {
            return;
        }
        let (mut va, mut vb) = (gather_v(&*s, &o.a), gather_v(&*s, &o.b));
        let (nx, ny) = (o.nx, o.ny);
        for pass in 0..passes {
            for k in 0..2 {
                let mut on = [false; N];
                for (l, on) in on.iter_mut().enumerate() {
                    *on = bounces(o, k, l, pass);
                }
                if !on.contains(&true) {
                    continue;
                }
                let p = o.p[k];
                let vn = ((vb.x - va.x) * nx + (vb.y - va.y) * ny) + vb.w * p.rnb - va.w * p.rna;
                let jn = (p.jn - p.normal_mass * (vn - o.restitution * p.speed)).max(F::ZERO);
                let (mut d, mut kept) = (jn - p.jn, jn);
                for l in 0..N {
                    if !on[l] {
                        (d.0[l], kept.0[l]) = (0.0, p.jn.0[l]);
                    }
                }
                o.p[k].jn = kept;
                o.p[k].sum_jn = p.sum_jn + d;
                apply(o, &mut va, &mut vb, (nx * d, ny * d), (p.rna * d, p.rnb * d));
            }
        }
        scatter(s, &o.a, &va);
        scatter(s, &o.b, &vb);
    }

    /// `solve` taken apart where the mod's pipeline of systems takes it
    /// apart (`pipeline.rs`; docs/architecture/flows.md, "Physics's
    /// adoption"): `begin` and `place` are `setup` less the batches' fill,
    /// which is the passes' first stage (`Step::Fill`, over every batch at
    /// once), the passes are `run`'s stages for a generic primitive to run
    /// (`Passes`), `finish` is `finish`, with what `setup` did to the
    /// contacts and points. Its coloring is the caller's, by the generic
    /// primitive (`Coloring::greedy`, the call `group` makes), seated by
    /// it too (`Coloring::seat`, `pack` the other way round). Bit for bit
    /// `solve`: the mod's tests hold it to the arrays, which solve by
    /// `solve`.
    #[allow(dead_code)] // Only the mod runs it; the benches compile this file too.
    pub mod staged {
        use super::*;

        /// A pass of the substeps, as the primitive hands it to the kernels.
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub enum Step {
            /// The batches filled from the contacts their lanes seat: the
            /// first stage, over every batch at once.
            Fill,
            Gravity,
            Warm,
            /// The pushing pass; whether it's the substep's last.
            Push(bool),
            Move,
            /// A relaxing pass: whether it's the substep's first (which
            /// finds the bias) and its last.
            Relax(bool, bool),
            Bounce,
        }

        /// The batches' fill, then the substeps as `run` makes its passes,
        /// each over every batch, a color at a time, or over `n` states
        /// (the bodies', and for `Move` the turning bodies' after them), in
        /// order, into `out`.
        pub fn program(params: &Params, states: usize, spins: usize, out: &mut Vec<Stage<Step>>) {
            out.clear();
            out.push(Stage::All(Step::Fill));
            for _ in 0..params.substeps {
                out.push(Stage::Each(Step::Gravity, states));
                out.push(Stage::Items(Step::Warm));
                out.push(Stage::Items(Step::Push(params.relax == 0)));
                out.push(Stage::Each(Step::Move, states + spins));
                for r in 0..params.relax {
                    out.push(Stage::Items(Step::Relax(r == 0, r + 1 == params.relax)));
                }
            }
            out.push(Stage::Items(Step::Bounce));
        }

        pub use super::{Atom, Bodies, Shared, State};

        /// Whether the passes across threads are the passes on one thread
        /// bit for bit on these bodies: no body that doesn't move has a
        /// negative zero where a batch writes it back or gravity adds to it.
        /// Batches of one color write such a body back unchanged, the same
        /// value whoever writes last, but for `-0.0 - -0.0`, which is `0.0`
        /// (get-znt.39). Conservative: a body that turns but doesn't
        /// translate counts as still.
        pub fn shareable(bodies: &[SolverBody], spinning: &[Spinning]) -> bool {
            let neg = |x: f32| x.to_bits() == (-0.0f32).to_bits();
            let still = |b: &SolverBody| b.inv_mass == 0.0;
            bodies.iter().all(|b| !still(b) || ![b.v.x, b.v.y, b.gravity.x, b.gravity.y].into_iter().any(neg))
                && spinning.iter().all(|sp| !still(&bodies[sp.body as usize]) || sp.inv_inertia > 0.0 || !neg(sp.w))
        }

        /// A `State` as threads share it, and back: what a primitive that
        /// shares the states between threads makes them into (the mod's
        /// `Shareable`).
        impl State {
            pub fn share(&self) -> Atom {
                Atom::new(self)
            }

            pub fn load(shared: &Atom) -> State {
                shared.state()
            }

            pub fn store(shared: &Atom, s: State) {
                for (k, x) in [s.v.x, s.v.y, s.w, s.moved.x, s.moved.y, s.turned.c, s.turned.s].into_iter().enumerate() {
                    shared.put(k, x);
                }
            }
        }

        /// What the bodies' stages write besides velocities, field by field:
        /// on threads, the moving and the turning of one body are two
        /// ranges of one stage, which a whole state's store would race.
        pub trait Moves: Bodies {
            fn store_moved(&mut self, i: usize, moved: Vec2);
            fn store_turned(&mut self, i: usize, turned: Rot);
        }

        impl Moves for [State] {
            #[inline(always)]
            fn store_moved(&mut self, i: usize, moved: Vec2) {
                self[i].moved = moved;
            }

            #[inline(always)]
            fn store_turned(&mut self, i: usize, turned: Rot) {
                self[i].turned = turned;
            }
        }

        impl Moves for Shared<'_> {
            #[inline(always)]
            fn store_moved(&mut self, i: usize, moved: Vec2) {
                self.0[i].put(3, moved.x);
                self.0[i].put(4, moved.y);
            }

            #[inline(always)]
            fn store_turned(&mut self, i: usize, turned: Rot) {
                self.0[i].put(5, turned.c);
                self.0[i].put(6, turned.s);
            }
        }

        /// A step's contacts as the passes solve them: batches in lanes, the
        /// bodies as states, kept from step to step for their allocations
        /// (a flow's payload).
        pub struct Staged<const N: usize> {
            /// Its states (`s`) are the passes': handed to kernels plain on
            /// one thread, shared by the primitive on several.
            hd: Head,
            moves: Vec<bool>,
            gravity: Vec<Vec2>,
            /// The batches and their lanes (`LaneCell`), written whole by the
            /// fill (`Step::Fill`), so kept as the last step left them, grown
            /// but never emptied here.
            items: Vec<Batch<N>>,
            lanes: Vec<[LaneCell; N]>,
            /// Each batch's contacts, lane by lane (`Coloring::seat`): what
            /// the fill fills it from.
            seats: Vec<u32>,
            /// Each contact's seat (`Coloring::seats`), `EMPTY` for one no
            /// batch solves: what the write-back finds a contact's lane by,
            /// going through the contacts in their order. Written by the
            /// fill as it seats each, through the shared reference every
            /// kernel has, as the lanes are, and by `place` for the rest:
            /// so every one each step. (Formed in `place` from the
            /// coloring's seats instead, it was 11-14 µs more of serial
            /// `prepare` on the settled pile of 10 000, at any thread count.)
            seat_of: Vec<AtomicU32>,
            /// Each turning body's angle over the step, as bits: the `Move`
            /// stage writes them through the shared reference every kernel
            /// has, whether its states are plain or shared. Once a substep a
            /// turning body: too few to be worth a plain path of their own.
            angle: Vec<AtomicU32>,
            /// The contacts no batch solves (`place`): the points `clear`
            /// marks solved, and the impulses it keeps (`enter`'s).
            solved: Vec<usize>,
            kept: Vec<(usize, [(f32, f32); 2])>,
        }

        impl<const N: usize> Default for Staged<N> {
            fn default() -> Self {
                Staged {
                    hd: Head::empty(),
                    moves: Vec::new(),
                    gravity: Vec::new(),
                    items: Vec::new(),
                    lanes: Vec::new(),
                    seats: Vec::new(),
                    seat_of: Vec::new(),
                    angle: Vec::new(),
                    solved: Vec::new(),
                    kept: Vec::new(),
                }
            }
        }

        /// The passes' kernels, over what they read besides the batches and
        /// states: `Sync`, for a primitive that may share them out.
        pub struct Kernels<'a, const N: usize> {
            params: Params,
            k: &'a Fixed,
            gravity: &'a [Vec2],
            angle: &'a [AtomicU32],
            spinning: &'a [Spinning],
            /// What the fill reads.
            bodies: &'a [SolverBody],
            contacts: &'a [Constraint],
            points: &'a [Points],
            seats: &'a [u32],
            lanes: &'a [[LaneCell; N]],
            seat_of: &'a [AtomicU32],
            coloring: &'a Coloring,
            dt: f32,
        }

        impl<const N: usize> Staged<N> {
            /// Whether a step is solved in lanes, `N` wide: as `solve_with`
            /// chooses `lanes::solve`. Every other step is `solve_with`'s,
            /// one contact at a time.
            pub fn takes(params: &Params, spinning: &[Spinning], points: &[Points]) -> bool {
                !(points.is_empty() && spinning.is_empty())
                    && params.wide == Wide::Colored(N)
                    && params.separation == Separation::Turned
                    && !params.block
            }

            /// The bodies as states: `setup`'s start. Returns whether each
            /// body moves, which the caller colors the contacts by.
            pub fn begin(&mut self, params: &Params, (bodies, spinning): (&[SolverBody], &[Spinning]), dt: f32) -> &[bool] {
                self.hd.start(params, (bodies, spinning), dt, &mut self.moves);
                &self.moves
            }

            /// The rest of `setup` but the fill: each batch's contacts
            /// seated as `coloring` lays them out, room for the batches, the
            /// contacts no batch solves as `enter` leaves them, and the
            /// states less the step's gravity. The layout, for the passes.
            pub fn place(
                &mut self,
                params: &Params,
                (bodies, spins): (&[SolverBody], usize),
                (contacts, points): (&mut [Constraint], &[Points]),
                coloring: &Coloring,
                dt: f32,
            ) -> Colored {
                let layout = coloring.seat(N, &mut self.seats);
                if self.seat_of.len() < contacts.len() {
                    self.seat_of.resize_with(contacts.len(), AtomicU32::default);
                }
                self.seat_of.truncate(contacts.len());
                // Only new room is written here: the fill writes every batch
                // whole, so what one held last step is never read, and
                // emptying them first was a fifth of the fill on one thread.
                let (n, nowhere) = (layout.items(), self.hd.k.nowhere);
                if self.items.len() < n {
                    self.items.resize(n, Batch::empty(nowhere));
                    self.lanes.resize_with(n, || std::array::from_fn(|_| LaneCell::default()));
                }
                self.items.truncate(n);
                self.lanes.truncate(n);
                self.solved.clear();
                self.kept.clear();
                for (i, _) in coloring.of.iter().enumerate().filter(|(_, k)| **k == UNSOLVED) {
                    self.seat_of[i].store(EMPTY, Ordering::Relaxed);
                    let c = &mut contacts[i];
                    let (jn, jt) = (c.jn, c.jt);
                    (c.jn, c.jt) = (0.0, 0.0);
                    let (pts, at, n_points, speed) = start(params, c, (jn, jt), bodies, (&self.hd.k, &self.hd.s[..]), points, dt);
                    c.speed = speed;
                    if at != NONE {
                        self.solved.push(at as usize);
                    }
                    unsolved(params, c, (&pts, at, n_points), &mut self.kept);
                }
                // `finish` finds a part's among them by point (`clear_run`).
                debug_assert!(self.solved.is_sorted() && self.kept.is_sorted_by_key(|(at, _)| *at), "in their contacts' order");
                // The states less the step's gravity, which the substeps
                // give back a share at a time.
                self.gravity.clear();
                self.gravity.extend(bodies.iter().map(|b| b.gravity));
                for (st, g) in self.hd.s.iter_mut().zip(self.gravity.iter()) {
                    st.v -= *g;
                }
                self.angle.clear();
                self.angle.extend((0..spins).map(|_| AtomicU32::new(0.0f32.to_bits())));
                layout
            }

            /// How many states the passes run over: the bodies and one
            /// standing still.
            pub fn states(&self) -> usize {
                self.hd.s.len()
            }

            /// The batches, the states and the kernels, for the passes: the
            /// bodies, contacts and points as `place` left them, for the
            /// fill.
            pub fn split<'a>(
                &'a mut self,
                params: &Params,
                (bodies, spinning): (&'a [SolverBody], &'a [Spinning]),
                (contacts, points): (&'a [Constraint], &'a [Points]),
                coloring: &'a Coloring,
                dt: f32,
            ) -> (&'a mut [Batch<N>], &'a mut [State], Kernels<'a, N>) {
                let kernels = Kernels {
                    params: *params,
                    k: &self.hd.k,
                    gravity: &self.gravity,
                    angle: &self.angle,
                    spinning,
                    bodies,
                    contacts,
                    points,
                    seats: &self.seats,
                    lanes: &self.lanes,
                    seat_of: &self.seat_of,
                    coloring,
                    dt,
                };
                (&mut self.items, &mut self.hd.s, kernels)
            }

            /// The step's impulses into a part's contacts and points, and
            /// the states into its bodies and turning bodies: `finish`'s
            /// writes, contact by contact, each finding its lane by its seat
            /// (`seat_of`); and before them `clear`'s of the part's points,
            /// here and not in `place` since the fill reads the points it
            /// clears. Every value a part writes is its lane's or its
            /// state's, so any cut into parts, on any threads, writes what
            /// one part of everything does.
            pub fn finish(&self, params: &Params, part: &mut Part<'_>) {
                let Part { contacts: (c0, contacts), points: (p0, points), bodies: (b0, bodies), spinning: (s0, spinning) } = part;
                clear_run(points, *p0, &self.solved, &self.kept);
                for (i, c) in (*c0..).zip(contacts.iter_mut()) {
                    let to = self.seat_of[i].load(Ordering::Relaxed);
                    if to == EMPTY {
                        continue;
                    }
                    let (batch, l) = (to as usize / N, to as usize % N);
                    let (o, lane) = (&self.items[batch], self.lanes[batch][l].get());
                    debug_assert_eq!(lane.contact as usize, i, "a contact's seat, its lane");
                    (c.jn, c.jt) = impulses(o, l, &lane);
                    c.speed = lane.speed;
                    if lane.at != NONE {
                        // Its own points, so in this part's run (`parts`).
                        let to = &mut points[lane.at as usize - *p0];
                        to.solved = true;
                        carried(params.carry, params.substeps, o, l, lane.count as usize, to);
                    }
                }
                let s = &self.hd.s[..];
                for (i, b) in (*b0..).zip(bodies.iter_mut()) {
                    let st = s.load(i);
                    (b.v, b.moved) = (st.v, st.moved);
                }
                for (j, sp) in (*s0..).zip(spinning.iter_mut()) {
                    let st = s.load(sp.body as usize);
                    (sp.w, sp.turned, sp.angle) = (st.w, st.turned, f32::from_bits(self.angle[j].load(Ordering::Relaxed)));
                }
            }
        }

        /// What a task of the write-back writes (`Staged::finish`): a run
        /// of the contacts, of their points, of the bodies and of the
        /// turning bodies, each with its first index.
        pub struct Part<'a> {
            contacts: (usize, &'a mut [Constraint]),
            points: (usize, &'a mut [Points]),
            bodies: (usize, &'a mut [SolverBody]),
            spinning: (usize, &'a mut [Spinning]),
        }

        /// The contacts, the bodies and the turning bodies cut alike into
        /// `n` parts (one at least), each part's points its contacts':
        /// the points are in their contacts' order (`gather_contacts`
        /// pushes a contact's as it goes), so a run of contacts has a run
        /// of points, from its first contact with any.
        pub fn parts<'a>(
            n: usize,
            (bodies, spinning): (&'a mut [SolverBody], &'a mut [Spinning]),
            (contacts, points): (&'a mut [Constraint], &'a mut [Points]),
        ) -> Vec<Part<'a>> {
            let n = n.max(1);
            let points_from = |at: usize| contacts[at..].iter().find(|c| c.points > 0).map_or(points.len(), |c| c.points as usize - 1);
            let cut = |k: usize| {
                let at = contacts.len() * k / n;
                [at, points_from(at), bodies.len() * k / n, spinning.len() * k / n]
            };
            let cuts: Vec<[usize; 4]> = (0..=n).map(cut).collect();
            assert!(cuts.windows(2).all(|w| w[0][1] <= w[1][1]), "the points in their contacts' order");
            /// The run `from..to` off the front of `rest`, which starts at
            /// `from`.
            fn run<'a, T>(rest: &mut &'a mut [T], from: usize, to: usize) -> (usize, &'a mut [T]) {
                let (head, tail) = std::mem::take(rest).split_at_mut(to - from);
                *rest = tail;
                (from, head)
            }
            let (mut cs, mut ps, mut bs, mut ss) = (contacts, points, bodies, spinning);
            let mut out = Vec::with_capacity(n);
            for w in cuts.windows(2) {
                let ([c, p, b, s], [c1, p1, b1, s1]) = (w[0], w[1]);
                out.push(Part {
                    contacts: run(&mut cs, c, c1),
                    points: run(&mut ps, p, p1),
                    bodies: run(&mut bs, b, b1),
                    spinning: run(&mut ss, s, s1),
                });
            }
            out
        }

        impl<const N: usize> Kernels<'_, N> {
            /// A stage over consecutive batches from `at`, over the states
            /// plain or shared: which pass it is decided once a block, as
            /// `run_across` decided it, since deciding it a batch at a
            /// time, or calling a kernel an edge, lost the lanes' gain
            /// (flows-spike.md).
            #[inline(always)]
            pub fn block<B: Bodies + ?Sized>(&self, k: Step, at: usize, items: &mut [Batch<N>], s: &mut B) {
                let inv_h = self.k.inv_h;
                match k {
                    Step::Fill => {
                        let (read, ks) = ((self.contacts, self.points), (self.k, &*s));
                        if at == 0 && items.len() * N == self.seats.len() {
                            fill_all(&self.params, ks, self.bodies, read, self.dt, (self.coloring, self.lanes, self.seat_of), items);
                        } else {
                            fill(&self.params, ks, self.bodies, read, self.dt, (self.seats, self.lanes, self.seat_of, at), items);
                        }
                    }
                    Step::Warm => items.iter_mut().for_each(|o| warm_start(o, s)),
                    Step::Push(last) => items.iter_mut().for_each(|o| pass::<N, true, COMPUTE>(o, s, inv_h, last)),
                    Step::Relax(true, last) => items.iter_mut().for_each(|o| pass::<N, false, STORE>(o, s, inv_h, last)),
                    Step::Relax(false, last) => items.iter_mut().for_each(|o| pass::<N, false, LOAD>(o, s, inv_h, last)),
                    Step::Bounce => {
                        for o in items.iter_mut() {
                            if o.restitution.0.iter().any(|e| *e != 0.0) {
                                restitute(o, s, self.params.bounce);
                            }
                        }
                    }
                    Step::Gravity | Step::Move => unreachable!("a stage over states"),
                }
            }

            /// A stage over states `r` of `states`, plain or shared:
            /// gravity's share, or moving them and then turning the turning
            /// bodies (`r` past the states), as `run`'s loops.
            #[inline(always)]
            pub fn each<B: Moves + ?Sized>(&self, k: Step, r: Range<usize>, s: &mut B, states: usize) {
                let (h, share) = (self.k.h, self.k.share);
                match k {
                    Step::Gravity => {
                        for i in r.start..r.end.min(self.gravity.len()) {
                            let (mut v, w) = s.load_v(i);
                            v += self.gravity[i] * share;
                            s.store_v(i, v, w);
                        }
                    }
                    Step::Move => {
                        for i in r.start.min(states)..r.end.min(states) {
                            let mut st = s.load(i);
                            st.moved += st.v * h;
                            s.store_moved(i, st.moved);
                        }
                        for j in r.start.max(states) - states..r.end.max(states) - states {
                            let (angle, body) = (&self.angle[j], self.spinning[j].body as usize);
                            let st = s.load(body);
                            let turned_by = f32::from_bits(angle.load(Ordering::Relaxed)) + h * st.w;
                            angle.store(turned_by.to_bits(), Ordering::Relaxed);
                            let turned = match self.params.integrate {
                                Integrate::Rotation => st.turned.integrate(h * st.w),
                                Integrate::Angle => Rot::from_angle(turned_by),
                            };
                            s.store_turned(body, turned);
                        }
                    }
                    _ => unreachable!("a stage over batches"),
                }
            }
        }
    }
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
        assert_eq!(Params::of(&physics2d::Tuning::default()), PARAMS);
        assert_eq!(Params::of(&physics2d::Tuning::DEFAULT), PARAMS);
        assert_eq!(Params::of(&physics2d::Tuning { substeps: 6 }), Params { substeps: 6, ..PARAMS });
    }

    /// The colors are Box2D's rule (`b2AddContactToGraph`): in pair order,
    /// the lowest color neither moving end has, never color 0 with an end
    /// that doesn't move. The equivalence tests solve the same order both
    /// ways, so they can't see a change to it; this can.
    #[test]
    fn contacts_are_colored_as_box2d_colors_them() {
        // Bodies 0-2 move, 3 and 4 are static.
        let bodies = [falling(0.0), falling(0.0), falling(0.0), ground(), ground()];
        let pair = |a: u32, b: u32| Constraint { a, b, normal: Vec2::new(0.0, 1.0), ..Default::default() };
        // Both static: unsolved, last. On a static: color 1. Free: 0. Both
        // ends taken, 0 and 1: 2. On a static, 0 taken anyway: 1.
        let contacts = [pair(3, 4), pair(0, 3), pair(1, 2), pair(0, 1), pair(2, 3)];
        assert_eq!(order(Wide::Colored(4), &bodies, &[], &contacts), [2, 1, 4, 3, 0]);
        // By level, pair order but for the unsolved.
        assert_eq!(order(Wide::Levels(4), &bodies, &[], &contacts), [1, 2, 3, 4, 0]);
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
