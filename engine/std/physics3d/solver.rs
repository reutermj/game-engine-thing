//! The contact solver over arrays: bodies and contacts gathered from the
//! world, solved, written back. No rotation, so a body is a velocity and an
//! inverse mass, and a contact a normal, a depth and its impulses.
//!
//! The 2D solver's soft step (engine/std/physics, solver.rs; Box2D v3's,
//! physics.md "Settling") in 3D: `SUBSTEPS` substeps, each gravity, warm
//! start, one soft pushing pass, positions, `RELAX_ITERATIONS` rigid
//! passes with friction; then restitution once, from the closing speed
//! before the step. The constants are the 2D ones; why each is what it is
//! is there.
//!
//! Friction is the one thing 3D changes: the tangent is a plane, not a line.
//! The friction impulse is kept as a vector in that plane and clamped to a
//! disc of radius `friction * normal impulse` (a circular Coulomb cone), not
//! each of two tangent axes on its own, a box that would let a body slide
//! faster diagonally (`friction_is_a_disc_not_a_box`). Stored as a world
//! vector, not two numbers in a tangent basis, so warm starting needs no
//! basis that stays put from step to step: last step's is projected onto
//! this step's plane.

use crate::Vec3;

pub const SUBSTEPS: usize = 5;
pub const RELAX_ITERATIONS: usize = 2;
/// Of the substep rate, between two moving bodies; against a static one.
pub const STIFFNESS: f32 = 0.25;
pub const STATIC_STIFFNESS: f32 = 0.5;
pub const DAMPING_RATIO: f32 = 10.0;
/// The fastest a contact pushes bodies apart.
pub const MAX_PUSH: f32 = 3.0;
/// Closing speeds below this don't bounce, so resting bodies settle.
pub const BOUNCE_THRESHOLD: f32 = 1.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SolverBody {
    pub v: Vec3,
    /// 0 for static bodies.
    pub inv_mass: f32,
    /// The step's gravity, already in `v`: given back a substep at a time
    /// (see the 2D solver).
    pub gravity: Vec3,
    /// Output: how far the body moved this step.
    pub moved: Vec3,
}

impl SolverBody {
    pub fn new(v: Vec3, inv_mass: f32, gravity: Vec3) -> SolverBody {
        SolverBody { v, inv_mass, gravity, moved: Vec3::ZERO }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Constraint {
    pub a: u32,
    pub b: u32,
    pub normal: Vec3,
    pub depth: f32,
    pub friction: f32,
    pub restitution: f32,
    /// Accumulated impulses over the step: in, the last step's (warm
    /// starting); out, this step's. `jt` is in the tangent plane.
    pub jn: f32,
    pub jt: Vec3,
    /// Output: the closing speed along the normal before solving.
    pub speed: f32,
}

/// Box2D's `b2MakeSoft`, as in 2D.
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

struct Row {
    a: usize,
    b: usize,
    normal: Vec3,
    mass: f32,
    base: f32,
    soft: Softness,
    friction: f32,
    jn: f32,
    jt: Vec3,
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
            let jt = c.jt - c.normal * c.jt.dot(c.normal);
            let row = Row {
                a,
                b,
                normal: c.normal,
                mass: if k > 0.0 { 1.0 / k } else { 0.0 },
                base: -c.depth,
                soft: if ia == 0.0 || ib == 0.0 { fixed } else { moving },
                friction: c.friction,
                jn: c.jn * share,
                jt: jt * share,
            };
            (c.jn, c.jt) = (0.0, Vec3::ZERO);
            row
        })
        .collect();
    for b in bodies.iter_mut() {
        b.v = b.v - b.gravity;
        b.moved = Vec3::ZERO;
    }

    for _ in 0..SUBSTEPS {
        for b in bodies.iter_mut() {
            b.v = b.v + b.gravity * share;
        }
        for r in rows.iter().filter(|r| r.mass != 0.0) {
            apply(bodies, r, r.normal * r.jn + r.jt);
        }
        pass(bodies, &mut rows, inv_h, true);
        for b in bodies.iter_mut() {
            b.moved = b.moved + b.v * h;
        }
        for _ in 0..RELAX_ITERATIONS {
            pass(bodies, &mut rows, inv_h, false);
        }
        for (r, c) in rows.iter().zip(contacts.iter_mut()) {
            c.jn += r.jn;
            c.jt = c.jt + r.jt;
        }
    }

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

fn pass(bodies: &mut [SolverBody], rows: &mut [Row], inv_h: f32, push: bool) {
    for r in rows.iter_mut() {
        if r.mass == 0.0 {
            continue;
        }
        let (a, b) = (&bodies[r.a], &bodies[r.b]);
        let sep = r.base + (b.moved - a.moved).dot(r.normal);
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

        let rel = bodies[r.b].v - bodies[r.a].v;
        let slip = rel - r.normal * rel.dot(r.normal);
        let limit = r.friction * r.jn;
        let mut jt = r.jt - slip * r.mass;
        let len2 = jt.dot(jt);
        if len2 > limit * limit {
            jt = jt * (limit / len2.sqrt());
        }
        let d = jt - r.jt;
        r.jt = jt;
        apply(bodies, r, d);
    }
}

#[inline(always)]
fn apply(bodies: &mut [SolverBody], r: &Row, impulse: Vec3) {
    let (ia, ib) = (bodies[r.a].inv_mass, bodies[r.b].inv_mass);
    bodies[r.a].v = bodies[r.a].v - impulse * ia;
    bodies[r.b].v = bodies[r.b].v + impulse * ib;
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    fn falling(v: Vec3) -> SolverBody {
        SolverBody::new(v, 1.0, Vec3::ZERO)
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
        // Along the normal, the last step's friction is no friction.
        assert!(c[0].jt.len() < 1e-6, "{:?}", c[0]);
    }

    #[test]
    fn penetration_is_pushed_out_at_most_max_push_and_leaves_no_speed() {
        let mut bodies = [SolverBody::default(), falling(Vec3::ZERO)];
        solve(&mut bodies, &mut [on_ground(0.2)], DT);
        assert!(bodies[1].v.len() < 1e-4, "{:?}", bodies[1]);
        let moved = bodies[1].moved.y;
        assert!(moved > 0.9 * MAX_PUSH * DT && moved <= MAX_PUSH * DT + 1e-6, "moved {moved}");
    }
}
