//! Our own step: `//engine/std/physics3d`, translation-only bodies in the
//! ECS (3D spatial storage for the broadphase, contacts as entities in an
//! ordered table), run on the ECS harness, one thread.

use std::collections::HashMap;
use std::time::Instant;

use engine_ecs::harness::Schedule;
use engine_ecs::{Build, Entity, World};
use physics3d::{Body, Collider, Manifold, Position, Static, TIMINGS, Timings, Vec3, Velocity};

use crate::{Backend, Config, Iters, Shape, Spec};

pub struct Ours {
    world: World,
    schedule: Schedule,
    /// Dynamic bodies, in the order they were added.
    dynamic: Vec<Entity>,
    last: Timings,
    total_us: f64,
}

impl Ours {
    pub fn new(config: &Config) -> Ours {
        // One setting: 8 velocity and 8 position iterations, which is what
        // both of the harness's modes ask for.
        let _ = matches!(config.iters, Iters::Default | Iters::Eight);
        let world = World::new();
        let schedule = physics3d::step(&world);
        *TIMINGS.lock().unwrap() = Timings::default();
        Ours { world, schedule, dynamic: Vec::new(), last: Timings::default(), total_us: 0.0 }
    }
}

impl Backend for Ours {
    fn name(&self) -> String {
        "ours".into()
    }

    fn solver(&self) -> String {
        format!(
            "sequential impulses, {} velocity + {} split-impulse position iterations, speculative margin {}",
            physics3d::solver::ITERATIONS,
            physics3d::solver::ITERATIONS,
            physics3d::narrow::MARGIN
        )
    }

    fn add(&mut self, bodies: &[Spec]) -> Vec<u32> {
        let mut m = self.world.between_frames(Build::default()).unwrap();
        bodies
            .iter()
            .map(|s| {
                let at = Position { x: s.pos[0], y: s.pos[1], z: s.pos[2] };
                let c = match s.shape {
                    Shape::Sphere(r) => Collider::sphere(r),
                    Shape::Box([x, y, z]) => Collider::cuboid(Vec3::new(x, y, z)),
                };
                let mut body = Body::new(if s.fixed { 0.0 } else { 1.0 / crate::MASS });
                (body.friction, body.restitution) = (crate::FRICTION, crate::RESTITUTION);
                let e = if s.fixed {
                    m.spawn((at, c, body, Static {}))
                } else {
                    let e = m.spawn((at, c, body, Velocity { x: s.vel[0], y: s.vel[1], z: s.vel[2] }));
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

    fn state(&self, out: &mut Vec<([f32; 3], [f32; 3])>) {
        let at: HashMap<Entity, Position> = self.world.values::<Position>().unwrap().into_iter().collect();
        let v: HashMap<Entity, Velocity> = self.world.values::<Velocity>().unwrap().into_iter().collect();
        out.clear();
        out.extend(self.dynamic.iter().map(|e| {
            let (p, v) = (at[e], v[e]);
            ([p.x, p.y, p.z], [v.x, v.y, v.z])
        }));
    }

    fn touching(&self) -> usize {
        // Speculative contacts (apart, negative depth) aren't touching.
        self.world.values::<Manifold>().map_or(0, |m| m.iter().filter(|(_, m)| m.depth >= 0.0).count())
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
        ]
    }
}
