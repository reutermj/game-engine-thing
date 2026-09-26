//! Our own step: `//engine/std/physics3d`, translation-only bodies in the
//! ECS (3D spatial storage for the broadphase, contacts as entities in an
//! ordered table), run on the ECS harness, one thread.

use std::collections::HashMap;
use std::time::Instant;

use engine_ecs::harness::Schedule;
use engine_ecs::{Build, Entity, World};
use physics3d::{AngularVelocity, Body, Collider, Manifold, Position, Quat, Rotation, TIMINGS, Timings, Vec3, Velocity, dynamic, fixed};

use crate::{Backend, Config, Iters, Shape, Spec, State};

pub struct Ours {
    world: World,
    schedule: Schedule,
    /// Dynamic bodies, in the order they were added.
    dynamic: Vec<Entity>,
    last: Timings,
    total_us: f64,
    rotate: bool,
}

/// Runs ours with a variant of its step, named as `physics3d::Tuning::parse`
/// reads it.
pub fn tune(variant: &str) {
    *physics3d::TUNING.lock().unwrap() = Some(physics3d::Tuning::parse(variant).unwrap_or_else(|e| panic!("--tune: {e}")));
}

impl Ours {
    pub fn new(config: &Config) -> Ours {
        // One setting: the soft step has its own substeps, whatever the
        // harness asks of the others.
        let _ = matches!(config.iters, Iters::Default | Iters::Eight);
        let world = World::new();
        let schedule = physics3d::step(&world);
        *TIMINGS.lock().unwrap() = Timings::default();
        Ours { world, schedule, dynamic: Vec::new(), last: Timings::default(), total_us: 0.0, rotate: config.rotate }
    }
}

impl Backend for Ours {
    fn name(&self) -> String {
        "ours".into()
    }

    fn solver(&self) -> String {
        use physics3d::solver::{RELAX_ITERATIONS, STIFFNESS, SUBSTEPS};
        format!(
            "soft step, {SUBSTEPS} substeps ({} Hz, 1 solve + {RELAX_ITERATIONS} relax each), speculative margin {}",
            STIFFNESS * SUBSTEPS as f32 * 60.0,
            physics3d::narrow::MARGIN
        )
    }

    fn add(&mut self, bodies: &[Spec]) -> Vec<u32> {
        let mut m = self.world.between_frames(Build::default()).unwrap();
        bodies
            .iter()
            .map(|s| {
                let at = Vec3::new(s.pos[0], s.pos[1], s.pos[2]);
                let c = match s.shape {
                    Shape::Sphere(r) => Collider::sphere(r),
                    Shape::Box([x, y, z]) => Collider::cuboid(Vec3::new(x, y, z)),
                };
                let e = if s.fixed {
                    let (p, q, r, c, mut body, st) = fixed(at, Quat::IDENTITY, c);
                    (body.friction, body.restitution) = (crate::FRICTION, crate::RESTITUTION);
                    m.spawn((p, q, r, c, body, st))
                } else {
                    let inv_mass = 1.0 / crate::MASS;
                    let mut body = if self.rotate { Body::solid(inv_mass, &c) } else { Body::new(inv_mass) };
                    (body.friction, body.restitution) = (crate::FRICTION, crate::RESTITUTION);
                    let (p, q, r, c, body, _, w) = dynamic(at, Quat::IDENTITY, c, body);
                    let e = m.spawn((p, q, r, c, body, Velocity { x: s.vel[0], y: s.vel[1], z: s.vel[2] }, w));
                    self.dynamic.push(e);
                    e
                };
                e.index
            })
            .collect()
    }

    fn step(&mut self, dt: f32) {
        // The harness runs every system at 1/60 s, the bench's step too.
        assert!((dt - crate::DT).abs() < 1e-9, "the ECS harness steps at 1/60 s");
        *TIMINGS.lock().unwrap() = Timings::default();
        let t = Instant::now();
        self.schedule.run_sequential(&self.world);
        self.total_us = t.elapsed().as_secs_f64() * 1e6;
        self.last = *TIMINGS.lock().unwrap();
    }

    fn state(&self, out: &mut Vec<State>) {
        let at: HashMap<Entity, Position> = self.world.values::<Position>().unwrap().into_iter().collect();
        let v: HashMap<Entity, Velocity> = self.world.values::<Velocity>().unwrap().into_iter().collect();
        let q: HashMap<Entity, Rotation> = self.world.values::<Rotation>().unwrap().into_iter().collect();
        let w: HashMap<Entity, AngularVelocity> = self.world.values::<AngularVelocity>().unwrap().into_iter().collect();
        out.clear();
        out.extend(self.dynamic.iter().map(|e| {
            let (p, v, q, w) = (at[e], v[e], q[e], w[e]);
            State { pos: [p.x, p.y, p.z], vel: [v.x, v.y, v.z], rot: [q.x, q.y, q.z, q.w], ang: [w.x, w.y, w.z] }
        }));
    }

    fn touching(&self) -> usize {
        // Speculative contacts (apart, negative depth) aren't touching.
        self.world.values::<Manifold>().map_or(0, |m| m.iter().filter(|(_, m)| m.deepest() >= 0.0).count())
    }

    fn stages(&self) -> Vec<(&'static str, f64)> {
        let t = &self.last;
        let us = |ns: u64| ns as f64 / 1e3;
        let inside = [t.gravity, t.gather, t.broadphase, t.narrowphase, t.merge, t.solve_gather, t.solver, t.write_back];
        vec![
            ("gather", us(t.gravity + t.gather)),
            ("broadphase", us(t.broadphase)),
            ("narrowphase", us(t.narrowphase)),
            ("merge contacts", us(t.merge)),
            ("solve: copy in/out", us(t.solve_gather + t.write_back)),
            ("solver", us(t.solver)),
            ("re-sorts (outside systems)", self.total_us - us(inside.iter().sum())),
            ("pairs", t.pairs as f64),
            ("contacts", t.contacts as f64),
            ("points", t.points as f64),
            ("warm-started %", 100.0 * t.matched as f64 / t.kept.max(1) as f64),
        ]
    }
}
