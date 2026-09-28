//! Rapier 2D, at its defaults, one thread (no `parallel` feature), with
//! its counters on for the stages.

use std::collections::VecDeque;

use rapier2d::prelude::*;

use crate::scene::{GRAVITY, Scene, Spec};
use crate::sim::Mark;
use crate::{Dyn, Sim};

pub struct Rapier {
    world: PhysicsWorld,
    label: String,
    sleep: bool,
    turning: bool,
    /// Full continuous collision on every dynamic body (`ccd_enabled`), off
    /// as shipped: Rapier then sweeps fast bodies against fixed colliders
    /// only.
    ccd: bool,
    dynamic: VecDeque<(RigidBodyHandle, Spec)>,
    /// Per stage, milliseconds summed since `reset`.
    time: Vec<(String, f64)>,
}

impl Rapier {
    /// `iterations` is `num_solver_iterations`, 4 by default.
    pub fn new(scene: &Scene, iterations: usize, sleep: bool, turning: bool, ccd: bool) -> Rapier {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0.0, GRAVITY);
        let defaults = world.integration_parameters.num_solver_iterations;
        world.integration_parameters.num_solver_iterations = iterations;
        world.physics_pipeline.counters.enable();
        let mut label = if iterations == defaults { "Rapier".to_string() } else { format!("Rapier, {iterations} iterations") };
        if sleep {
            label += ", sleeping";
        }
        if ccd {
            label += ", CCD";
        }
        let mut r = Rapier { world, label, sleep, turning, ccd, dynamic: VecDeque::new(), time: Vec::new() };
        for s in scene.build() {
            r.add(&s);
        }
        r
    }

    fn add(&mut self, s: &Spec) {
        let shape = if s.circle { ColliderBuilder::ball(s.hx) } else { ColliderBuilder::cuboid(s.hx, s.hy) };
        // Mixed as //engine/std/physics mixes them: the least friction, the
        // greatest restitution. Rapier averages both by default.
        let collider = shape
            .friction(s.friction)
            .restitution(s.restitution)
            .friction_combine_rule(CoefficientCombineRule::Min)
            .restitution_combine_rule(CoefficientCombineRule::Max);
        let at = Vector::new(s.x, s.y);
        if !s.dynamic {
            self.world.insert(RigidBodyBuilder::fixed().translation(at).rotation(s.angle), collider);
            return;
        }
        let body = RigidBodyBuilder::dynamic()
            .translation(at)
            .rotation(s.angle)
            .linvel(Vector::new(s.vx, s.vy))
            .gravity_scale(s.gravity_scale)
            .ccd_enabled(self.ccd)
            .can_sleep(self.sleep);
        let body = if self.turning { body.angvel(s.w) } else { body.lock_rotations() };
        // The scene's mass (1 but where a scene sets one) whatever the
        // shape, as in the engine; turning, the inertia follows from the
        // shape at that mass, as the engine's does.
        let (h, _) = self.world.insert(body, collider.mass(s.mass));
        self.dynamic.push_back((h, *s));
    }
}

impl Sim for Rapier {
    fn label(&self) -> String {
        self.label.clone()
    }

    fn step(&mut self, n: u32) {
        for _ in 0..n {
            self.world.step();
            let c = &self.world.physics_pipeline.counters;
            let stages = [
                // Rapier updates its tree at the end of a step, for the
                // positions just solved ("final"), and finds pairs from it at
                // the start of the next.
                ("broadphase", c.cd.broad_phase_time.time_ms() + c.cd.final_broad_phase_time.time_ms()),
                ("narrowphase", c.cd.narrow_phase_time.time_ms()),
                ("solver", c.stages.solver_time.time_ms()),
                ("rp:collision detection", c.stages.collision_detection_time.time_ms()),
                ("rp:broadphase: pairs", c.cd.broad_phase_time.time_ms()),
                ("rp:broadphase: final", c.cd.final_broad_phase_time.time_ms()),
                ("rp:island construction", c.stages.island_construction_time.time_ms()),
                ("rp:constraints collection", c.stages.island_constraints_collection_time.time_ms()),
                ("rp:solver: velocity assembly", c.solver.velocity_assembly_time.time_ms()),
                ("rp:solver: velocity resolution", c.solver.velocity_resolution_time.time_ms()),
                ("rp:solver: velocity update", c.solver.velocity_update_time.time_ms()),
                ("rp:solver: writeback", c.solver.velocity_writeback_time.time_ms()),
                ("rp:update", c.stages.update_time.time_ms()),
                ("rp:user changes", c.stages.user_changes.time_ms()),
                ("rp:ccd", c.stages.ccd_time.time_ms()),
                ("engine step", c.step_time.time_ms()),
            ];
            if self.time.is_empty() {
                self.time = stages.iter().map(|(k, _)| (k.to_string(), 0.0)).collect();
            }
            for ((_, sum), (_, ms)) in self.time.iter_mut().zip(stages) {
                *sum += ms;
            }
        }
    }

    fn rain(&mut self, arrivals: &[Spec], alive: usize) {
        for s in arrivals {
            self.add(s);
            if self.dynamic.len() > alive {
                let (h, _) = self.dynamic.pop_front().unwrap();
                self.world.remove_body(h);
            }
        }
    }

    fn bodies(&self) -> Vec<Dyn> {
        self.dynamic
            .iter()
            .map(|(h, s)| {
                let b = &self.world.bodies[*h];
                let (p, v) = (b.translation(), b.linvel());
                Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: p.x, y: p.y, vx: v.x, vy: v.y, angle: b.rotation().angle(), w: b.angvel() }
            })
            .collect()
    }

    fn reset(&mut self) {
        self.time.iter_mut().for_each(|(_, t)| *t = 0.0);
    }

    fn stages(&self) -> Vec<(String, f64)> {
        self.time.iter().map(|(k, ms)| (k.clone(), ms * 1e3)).collect()
    }

    fn contacts(&self) -> usize {
        self.world.narrow_phase.contact_pairs().filter(|p| p.has_any_active_contact()).count()
    }

    /// Every contact pair's manifold points, on the first collider's
    /// surface.
    fn marks(&self) -> Vec<Mark> {
        let mut out = Vec::new();
        for pair in self.world.narrow_phase.contact_pairs() {
            let pose = self.world.colliders[pair.collider1].position();
            for m in pair.manifolds() {
                let n = m.data.normal;
                for p in &m.points {
                    let at = pose.transform_point(p.local_p1);
                    out.push(Mark { x: at.x, y: at.y, nx: n.x, ny: n.y, estimated: false, speculative: p.dist > 0.0 });
                }
            }
        }
        out
    }

    fn sleeping(&self) -> Vec<bool> {
        self.dynamic.iter().map(|(h, _)| self.world.bodies[*h].is_sleeping()).collect()
    }

    fn native(&self) -> String {
        let c = &self.world.physics_pipeline.counters;
        format!(
            "contact pairs {} (touching {}), constraints {}, active bodies {}",
            c.cd.ncontact_pairs,
            self.contacts(),
            c.solver.nconstraints,
            self.world.islands.num_active_bodies()
        )
    }
}
