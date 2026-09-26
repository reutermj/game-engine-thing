//! Rapier 3D: its own PhysicsWorld, default integration parameters unless
//! [`Iters::Eight`], and its Counters on for per-stage times.

use rapier3d::prelude::*;

use crate::{Backend, Config, FRICTION, GRAVITY, Iters, MASS, RESTITUTION, Shape, Spec};

pub struct Rapier {
    world: PhysicsWorld,
    dynamic: Vec<RigidBodyHandle>,
    count: u32,
    sleep: bool,
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
        Rapier { world, dynamic: Vec::new(), count: 0, sleep: config.sleep }
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
            let body = if s.fixed {
                RigidBodyBuilder::fixed().translation(pos)
            } else {
                RigidBodyBuilder::dynamic()
                    .translation(pos)
                    .linvel(Vector::new(s.vel[0], s.vel[1], s.vel[2]))
                    .lock_rotations()
                    .can_sleep(self.sleep)
            };
            let collider = match s.shape {
                Shape::Sphere(r) => ColliderBuilder::ball(r),
                Shape::Box([x, y, z]) => ColliderBuilder::cuboid(x, y, z),
            }
            .friction(FRICTION)
            .restitution(RESTITUTION)
            // Sets the density that gives this mass; ignored on a fixed body.
            .mass(MASS);
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

    fn state(&self, out: &mut Vec<([f32; 3], [f32; 3])>) {
        out.clear();
        for &h in &self.dynamic {
            let b = &self.world.bodies[h];
            let (p, v) = (b.translation(), b.linvel());
            out.push(([p.x, p.y, p.z], [v.x, v.y, v.z]));
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
