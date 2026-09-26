//! Sequential impulses over arrays: the step's bodies and contacts, gathered
//! from the world by the mod and written back after. No rotation, so a
//! body is a velocity and an inverse mass. Sequential impulses, with
//! accumulated impulses clamped and warm started from the last step, and
//! the restitution threshold, are Erin Catto's, as Box2D has them.
//!
//! Penetration is corrected with a split impulse: a second, pseudo velocity
//! that moves positions but is thrown away after the step, so pushing
//! bodies apart never adds speed (Baumgarte bias on the real velocity makes
//! stacks jitter and bounce). The split impulse is Bullet's (Erwin
//! Coumans's push velocities). See docs/CREDITS.md.

use physics::Vec2;

pub const ITERATIONS: usize = 8;
/// Penetration left uncorrected, so resting contacts stay touching.
pub const SLOP: f32 = 0.005;
/// How much of the penetration beyond `SLOP` a step corrects.
pub const BETA: f32 = 0.2;
/// Closing speeds below this don't bounce, so resting bodies settle.
pub const BOUNCE_THRESHOLD: f32 = 1.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SolverBody {
    pub v: Vec2,
    /// 0 for static and kinematic bodies.
    pub inv_mass: f32,
    /// Output: the position correction for this step, as a velocity.
    pub pseudo: Vec2,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Constraint {
    pub a: u32,
    pub b: u32,
    pub normal: Vec2,
    pub depth: f32,
    pub friction: f32,
    pub restitution: f32,
    /// Accumulated impulses: in, the last step's (warm starting); out, this
    /// step's.
    pub jn: f32,
    pub jt: f32,
    /// Output: the closing speed along the normal before solving.
    pub speed: f32,
}

pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    // What each contact's normal velocity must reach: stopped at the
    // surface for a speculative contact, bounced for a fast one. From the
    // velocities before any impulse: read after warm-starting its
    // neighbors, a contact deep in a stack sees their impulses as its own
    // closing speed, bounces on it, and the stack never settles.
    let mut targets = vec![0.0; contacts.len()];
    for (c, target) in contacts.iter_mut().zip(&mut targets) {
        let (a, b) = (bodies[c.a as usize], bodies[c.b as usize]);
        let vn = (b.v - a.v).dot(c.normal);
        c.speed = -vn;
        *target = if c.depth < 0.0 {
            // Allowed to close the gap, not more.
            c.depth / dt
        } else if -vn > BOUNCE_THRESHOLD {
            -c.restitution * vn
        } else {
            0.0
        };
    }
    for c in contacts.iter() {
        let impulse = c.normal * c.jn + c.normal.perp() * c.jt;
        apply(bodies, c, impulse, |b| &mut b.v);
    }

    for _ in 0..ITERATIONS {
        for (c, &target) in contacts.iter_mut().zip(&targets) {
            let k = mass(bodies, c);
            if k == 0.0 {
                continue;
            }
            let rel = bodies[c.b as usize].v - bodies[c.a as usize].v;
            let jn = (c.jn + (target - rel.dot(c.normal)) / k).max(0.0);
            let dn = jn - c.jn;
            c.jn = jn;
            apply(bodies, c, c.normal * dn, |b| &mut b.v);

            let t = c.normal.perp();
            let rel = bodies[c.b as usize].v - bodies[c.a as usize].v;
            let limit = c.friction * c.jn;
            let jt = (c.jt - rel.dot(t) / k).clamp(-limit, limit);
            let dt_ = jt - c.jt;
            c.jt = jt;
            apply(bodies, c, t * dt_, |b| &mut b.v);
        }
    }

    // The split impulse: only for contacts sunk past the slop.
    let mut pj = vec![0.0f32; contacts.len()];
    for _ in 0..ITERATIONS {
        for (c, pj) in contacts.iter().zip(&mut pj) {
            let k = mass(bodies, c);
            if k == 0.0 || c.depth <= SLOP {
                continue;
            }
            let bias = BETA * (c.depth - SLOP) / dt;
            let rel = bodies[c.b as usize].pseudo - bodies[c.a as usize].pseudo;
            let new = (*pj + (bias - rel.dot(c.normal)) / k).max(0.0);
            let d = new - *pj;
            *pj = new;
            apply(bodies, c, c.normal * d, |b| &mut b.pseudo);
        }
    }
}

fn mass(bodies: &[SolverBody], c: &Constraint) -> f32 {
    bodies[c.a as usize].inv_mass + bodies[c.b as usize].inv_mass
}

/// Applies `impulse` to `b` and its opposite to `a`, through `field`.
fn apply(bodies: &mut [SolverBody], c: &Constraint, impulse: Vec2, field: impl Fn(&mut SolverBody) -> &mut Vec2) {
    let (ia, ib) = (bodies[c.a as usize].inv_mass, bodies[c.b as usize].inv_mass);
    *field(&mut bodies[c.a as usize]) -= impulse * ia;
    *field(&mut bodies[c.b as usize]) += impulse * ib;
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn ground() -> SolverBody {
        SolverBody::default()
    }

    fn falling(vy: f32) -> SolverBody {
        SolverBody { v: Vec2::new(0.0, vy), inv_mass: 1.0, pseudo: Vec2::ZERO }
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
    fn a_speculative_contact_lets_a_body_close_the_gap_and_no_more() {
        let mut bodies = [ground(), falling(10.0)];
        // 0.05 apart: it may move 0.05 this step, 3 units/s at 60 Hz.
        solve(&mut bodies, &mut [on_ground(-0.05, 1.0)], DT);
        assert!((bodies[1].v.y - 3.0).abs() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn penetration_is_corrected_by_pseudo_velocity_alone() {
        let mut bodies = [ground(), falling(0.0)];
        solve(&mut bodies, &mut [on_ground(0.2, 0.0)], DT);
        assert_eq!(bodies[1].v, Vec2::ZERO, "no real speed from the correction");
        let moved = bodies[1].pseudo.y * DT;
        assert!((moved + BETA * (0.2 - SLOP)).abs() < 1e-4, "moved {moved}");
    }

    #[test]
    fn friction_is_limited_by_the_normal_impulse() {
        // Sliding at 10 while pressed down at 1: friction can take 0.5 of it.
        let mut bodies = [ground(), SolverBody { v: Vec2::new(10.0, 1.0), inv_mass: 1.0, pseudo: Vec2::ZERO }];
        solve(&mut bodies, &mut [on_ground(0.0, 0.0)], DT);
        assert!((bodies[1].v.x - 9.5).abs() < 1e-3, "{:?}", bodies[1]);
    }

    #[test]
    fn warm_starting_applies_last_steps_impulse_first() {
        // A resting body under gravity: last step's impulse already holds it.
        let mut bodies = [ground(), falling(40.0 * DT)];
        let mut c = [on_ground(0.0, 0.0)];
        c[0].jn = 40.0 * DT;
        let before = c[0].jn;
        solve(&mut bodies, &mut c, DT);
        assert!(bodies[1].v.y.abs() < 1e-4);
        assert!((c[0].jn - before).abs() < 1e-4, "no further impulse needed: {:?}", c[0]);
    }
}
