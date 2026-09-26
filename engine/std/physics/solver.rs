//! The contact solver: the step's bodies and contacts, gathered from the
//! world by the mod and written back after. No rotation, so a body is a
//! velocity and an inverse mass, and a contact a normal and a depth.
//!
//! A soft step, as Box2D v3 takes it (Erin Catto's "Solver2D"): the step
//! split into `SUBSTEPS`, each of them
//! 1. gravity for the substep, and last substep's impulses again (warm
//!    starting);
//! 2. one pass of sequential impulses whose contacts are soft springs: a
//!    penetrating contact pushes out with a velocity (at most `MAX_PUSH`)
//!    through the real velocity, softened so pushing can't overshoot;
//! 3. positions moved by the velocities, and every contact's separation
//!    updated from how far its bodies moved, without finding contacts
//!    again;
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
//! restitution threshold are Catto's too. See docs/CREDITS.md.

use physics::Vec2;

/// Five where Box2D has four: a soft contact is only as stiff as its
/// substeps are short (`STIFFNESS`), and at five a pile of 10 000 sinks
/// 0.013 deep where at four it sinks 0.024, for 11% more time a step
/// (physics.md, "Settling").
pub const SUBSTEPS: usize = 5;
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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Constraint {
    pub a: u32,
    pub b: u32,
    pub normal: Vec2,
    pub depth: f32,
    pub friction: f32,
    pub restitution: f32,
    /// Accumulated impulses over the step: in, the last step's (warm
    /// starting); out, this step's.
    pub jn: f32,
    pub jt: f32,
    /// Output: the closing speed along the normal before solving.
    pub speed: f32,
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

/// A contact as the substeps solve it.
struct Row {
    a: usize,
    b: usize,
    normal: Vec2,
    /// 1 / (the ends' inverse masses), 0 when neither moves.
    mass: f32,
    /// Separation (minus depth) when found: bodies' movement adds to it.
    base: f32,
    soft: Softness,
    friction: f32,
    /// This substep's accumulated impulses.
    jn: f32,
    jt: f32,
}

pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    let h = dt / SUBSTEPS as f32;
    let inv_h = 1.0 / h;
    let moving = Softness::new(STIFFNESS * inv_h, DAMPING_RATIO, h);
    let fixed = Softness::new(STATIC_STIFFNESS * inv_h, DAMPING_RATIO, h);
    let share = 1.0 / SUBSTEPS as f32;
    let mut rows: Vec<Row> = contacts
        .iter_mut()
        .map(|c| {
            let (a, b) = (c.a as usize, c.b as usize);
            let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
            c.speed = -(bodies[b].v - bodies[a].v).dot(c.normal);
            let k = ia + ib;
            // The last step's impulse was over the whole step: a substep's
            // share of it is where each substep starts.
            let row = Row {
                a,
                b,
                normal: c.normal,
                mass: if k > 0.0 { 1.0 / k } else { 0.0 },
                base: -c.depth,
                soft: if ia == 0.0 || ib == 0.0 { fixed } else { moving },
                friction: c.friction,
                jn: c.jn * share,
                jt: c.jt * share,
            };
            (c.jn, c.jt) = (0.0, 0.0);
            row
        })
        .collect();
    for b in bodies.iter_mut() {
        b.v -= b.gravity;
        b.moved = Vec2::ZERO;
    }

    for _ in 0..SUBSTEPS {
        for b in bodies.iter_mut() {
            b.v += b.gravity * share;
        }
        for r in rows.iter().filter(|r| r.mass != 0.0) {
            apply(bodies, r, r.normal * r.jn + r.normal.perp() * r.jt);
        }
        pass(bodies, &mut rows, inv_h, true);
        for b in bodies.iter_mut() {
            b.moved += b.v * h;
        }
        for _ in 0..RELAX_ITERATIONS {
            pass(bodies, &mut rows, inv_h, false);
        }
        for (r, c) in rows.iter().zip(contacts.iter_mut()) {
            c.jn += r.jn;
            c.jt += r.jt;
        }
    }

    // Restitution, once, from the closing speed before the step, for the
    // contacts that pushed: a speculative contact that stopped a body at
    // the surface bounces it at the speed it came in at, not at what was
    // left of it.
    for (r, c) in rows.iter_mut().zip(contacts.iter_mut()) {
        if c.restitution == 0.0 || c.speed <= BOUNCE_THRESHOLD || c.jn == 0.0 || r.mass == 0.0 {
            continue;
        }
        let vn = (bodies[r.b].v - bodies[r.a].v).dot(r.normal);
        let jn = (r.jn - r.mass * (vn - c.restitution * c.speed)).max(0.0);
        let d = jn - r.jn;
        r.jn = jn;
        c.jn += d;
        apply(bodies, r, r.normal * d);
    }
}

/// One pass of sequential impulses over the contacts: soft and pushing
/// out when `push`, else rigid, with friction.
fn pass(bodies: &mut [SolverBody], rows: &mut [Row], inv_h: f32, push: bool) {
    for r in rows.iter_mut() {
        if r.mass == 0.0 {
            continue;
        }
        let (a, b) = (&bodies[r.a], &bodies[r.b]);
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
        apply(bodies, r, r.normal * d);
        if push {
            continue;
        }

        let t = r.normal.perp();
        let vt = (bodies[r.b].v - bodies[r.a].v).dot(t);
        let limit = r.friction * r.jn;
        let jt = (r.jt - r.mass * vt).clamp(-limit, limit);
        let d = jt - r.jt;
        r.jt = jt;
        apply(bodies, r, t * d);
    }
}

/// Applies `impulse` to `b` and its opposite to `a`.
#[inline(always)]
fn apply(bodies: &mut [SolverBody], r: &Row, impulse: Vec2) {
    let (ia, ib) = (bodies[r.a].inv_mass, bodies[r.b].inv_mass);
    bodies[r.a].v -= impulse * ia;
    bodies[r.b].v += impulse * ib;
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
