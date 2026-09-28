//! Rapier 3D: its own PhysicsWorld, default integration parameters unless
//! [`Iters::Eight`], and its Counters on for per-stage times.

use rapier3d::prelude::*;

use crate::{Backend, Config, GRAVITY, Iters, Shape, Spec, State};

pub struct Rapier {
    world: PhysicsWorld,
    dynamic: Vec<RigidBodyHandle>,
    count: u32,
    sleep: bool,
    rotate: bool,
}

impl Rapier {
    pub fn new(config: &Config) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(GRAVITY[0], GRAVITY[1], GRAVITY[2]);
        if config.iters == Iters::Eight {
            world.integration_parameters.num_solver_iterations = 8;
        }
        // Needs the crate's `profiler` feature, or every timer reads zero.
        world.physics_pipeline.counters.enable();
        Rapier { world, dynamic: Vec::new(), count: 0, sleep: config.sleep, rotate: config.rotate }
    }
}

impl Backend for Rapier {
    fn name(&self) -> String {
        "rapier".into()
    }

    fn solver(&self) -> String {
        let p = &self.world.integration_parameters;
        format!(
            "{} solver iterations x ({} PGS + {} stabilization)",
            p.num_solver_iterations, p.num_internal_pgs_iterations, p.num_internal_stabilization_iterations
        )
    }

    fn add(&mut self, bodies: &[Spec]) -> Vec<u32> {
        let mut handles = Vec::with_capacity(bodies.len());
        for s in bodies {
            let pos = Vector::new(s.pos[0], s.pos[1], s.pos[2]);
            // The axis times the angle, as Rapier takes a rotation.
            let (q, angle) = (s.rot, 2.0 * s.rot[3].clamp(-1.0, 1.0).acos());
            let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt();
            let turn = if norm > 1e-6 { Vector::new(q[0], q[1], q[2]) * (angle / norm) } else { Vector::ZERO };
            let body = if s.fixed {
                RigidBodyBuilder::fixed().translation(pos).rotation(turn)
            } else {
                let b = RigidBodyBuilder::dynamic()
                    .translation(pos)
                    .rotation(turn)
                    .linvel(Vector::new(s.vel[0], s.vel[1], s.vel[2]))
                    .can_sleep(self.sleep);
                if self.rotate { b } else { b.lock_rotations() }
            };
            let collider = match s.shape {
                Shape::Sphere(r) => ColliderBuilder::ball(r),
                Shape::Box([x, y, z]) => ColliderBuilder::cuboid(x, y, z),
            }
            .friction(s.friction)
            .restitution(s.restitution)
            // Sets the density that gives this mass; ignored on a fixed body.
            .mass(s.mass);
            let (handle, _) = self.world.insert(body, collider);
            if !s.fixed {
                self.dynamic.push(handle);
            }
            handles.push(self.count);
            self.count += 1;
        }
        handles
    }

    fn step(&mut self, dt: f32) {
        self.world.integration_parameters.dt = dt;
        self.world.step();
    }

    fn state(&self, out: &mut Vec<State>) {
        out.clear();
        for &h in &self.dynamic {
            let b = &self.world.bodies[h];
            let (p, v, q, w) = (b.translation(), b.linvel(), b.rotation(), b.angvel());
            out.push(State { pos: [p.x, p.y, p.z], vel: [v.x, v.y, v.z], rot: [q.x, q.y, q.z, q.w], ang: [w.x, w.y, w.z] });
        }
    }

    fn touching(&self) -> usize {
        self.world.narrow_phase.contact_pairs().filter(|p| p.has_any_active_contact()).count()
    }

    fn stages(&self) -> Vec<(&'static str, f64)> {
        let c = &self.world.physics_pipeline.counters;
        let us = |t: &rapier3d::counters::Timer| t.time_ms() * 1000.0;
        vec![
            ("step", us(&c.step_time)),
            ("collision detection", us(&c.stages.collision_detection_time)),
            ("broad phase", us(&c.cd.broad_phase_time)),
            ("narrow phase", us(&c.cd.narrow_phase_time)),
            ("islands", us(&c.stages.island_construction_time)),
            ("solver", us(&c.stages.solver_time)),
        ]
    }
}
