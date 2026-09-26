//! Sequential impulses over arrays, the 2D solver (engine/std/physics,
//! solver.rs) in 3D: bodies and contacts gathered from the world, solved,
//! written back. No rotation, so a body is a velocity and an inverse mass,
//! and a contact a normal, a depth and its impulses. The structure (warm
//! starting, targets from the velocities before any impulse, a split impulse
//! for penetration) is Box2D's, by way of the 2D solver.
//!
//! Friction is the one thing 3D changes: the tangent is a plane, not a line.
//! The friction impulse is kept as a vector in that plane and clamped to a
//! disc of radius `friction * normal impulse` (a circular Coulomb cone), not
//! each of two tangent axes on its own, a box that would let a body slide
//! faster diagonally (`friction_is_a_disc_not_a_box`). Stored as a world vector, not
//! two numbers in a tangent basis, so warm starting needs no basis that
//! stays put from step to step: last step's is projected onto this step's
//! plane.

use crate::Vec3;

pub const ITERATIONS: usize = 8;
/// Penetration left uncorrected, so resting contacts stay touching.
pub const SLOP: f32 = 0.005;
/// How much of the penetration beyond `SLOP` a step corrects.
pub const BETA: f32 = 0.2;
/// Closing speeds below this don't bounce, so resting bodies settle.
pub const BOUNCE_THRESHOLD: f32 = 1.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SolverBody {
    pub v: Vec3,
    /// 0 for static bodies.
    pub inv_mass: f32,
    /// Output: the position correction for this step, as a velocity.
    pub pseudo: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Constraint {
    pub a: u32,
    pub b: u32,
    pub normal: Vec3,
    pub depth: f32,
    pub friction: f32,
    pub restitution: f32,
    /// Accumulated impulses: in, the last step's (warm starting); out, this
    /// step's. `jt` is in the tangent plane.
    pub jn: f32,
    pub jt: Vec3,
}

pub fn solve(bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    // What each contact's normal velocity must reach, from the velocities
    // before any impulse, as in 2D (see its solver for why).
    let mut targets = vec![0.0; contacts.len()];
    for (c, target) in contacts.iter_mut().zip(&mut targets) {
        let (a, b) = (bodies[c.a as usize], bodies[c.b as usize]);
        let vn = (b.v - a.v).dot(c.normal);
        *target = if c.depth < 0.0 {
            c.depth / dt
        } else if -vn > BOUNCE_THRESHOLD {
            -c.restitution * vn
        } else {
            0.0
        };
        c.jt = c.jt - c.normal * c.jt.dot(c.normal);
    }
    for c in contacts.iter() {
        let impulse = c.normal * c.jn + c.jt;
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

            let rel = bodies[c.b as usize].v - bodies[c.a as usize].v;
            let slip = rel - c.normal * rel.dot(c.normal);
            let limit = c.friction * c.jn;
            let mut jt = c.jt - slip * (1.0 / k);
            let len2 = jt.dot(jt);
            if len2 > limit * limit {
                jt = jt * (limit / len2.sqrt());
            }
            let d = jt - c.jt;
            c.jt = jt;
            apply(bodies, c, d, |b| &mut b.v);
        }
    }

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
fn apply(bodies: &mut [SolverBody], c: &Constraint, impulse: Vec3, field: impl Fn(&mut SolverBody) -> &mut Vec3) {
    let (ia, ib) = (bodies[c.a as usize].inv_mass, bodies[c.b as usize].inv_mass);
    let fa = field(&mut bodies[c.a as usize]);
    *fa = *fa - impulse * ia;
    let fb = field(&mut bodies[c.b as usize]);
    *fb = *fb + impulse * ib;
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn falling(v: Vec3) -> SolverBody {
        SolverBody { v, inv_mass: 1.0, pseudo: Vec3::ZERO }
    }

    /// Body 0 is the ground, below body 1 (y up): the normal from 1 to 0
    /// points down.
    fn on_ground(depth: f32) -> Constraint {
        Constraint { a: 1, b: 0, normal: Vec3::new(0.0, -1.0, 0.0), depth, friction: 0.5, ..Default::default() }
    }

    #[test]
    fn a_landing_body_stops() {
        let mut bodies = [SolverBody::default(), falling(Vec3::new(0.0, -10.0, 0.0))];
        solve(&mut bodies, &mut [on_ground(0.0)], DT);
        assert!(bodies[1].v.y.abs() < 1e-4, "{:?}", bodies[1]);
    }

    #[test]
    fn friction_is_a_disc_not_a_box() {
        // Sliding diagonally at 10 while pressed down at 1: friction can take
        // 0.5 of the speed, along the slide, not 0.5 on each axis.
        let d = 10.0 / 2f32.sqrt();
        let mut bodies = [SolverBody::default(), falling(Vec3::new(d, -1.0, d))];
        solve(&mut bodies, &mut [on_ground(0.0)], DT);
        let v = bodies[1].v;
        assert!(((v.x * v.x + v.z * v.z).sqrt() - 9.5).abs() < 1e-3, "{v:?}");
        assert!((v.x - v.z).abs() < 1e-5);
    }

    #[test]
    fn warm_starting_projects_last_steps_friction_onto_this_plane() {
        let mut bodies = [SolverBody::default(), falling(Vec3::ZERO)];
        let mut c = [Constraint { jt: Vec3::new(0.0, 3.0, 0.0), ..on_ground(0.0) }];
        solve(&mut bodies, &mut c, DT);
        // Along the normal, last step's friction is no friction.
        assert!(c[0].jt.len() < 1e-6, "{:?}", c[0]);
    }

    #[test]
    fn penetration_is_corrected_by_pseudo_velocity_alone() {
        let mut bodies = [SolverBody::default(), falling(Vec3::ZERO)];
        solve(&mut bodies, &mut [on_ground(0.2)], DT);
        assert_eq!(bodies[1].v, Vec3::ZERO);
        assert!((bodies[1].pseudo.y * DT - BETA * (0.2 - SLOP)).abs() < 1e-4);
    }
}
