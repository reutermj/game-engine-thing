//! Where a 3D pile's step goes, by stage, in the ECS:
//! `./bazel run -c opt //engine/std/physics3d:stages -- [n] [sphere|box]`.
//! The comparison with other engines is //bench/physics3d; this is the
//! breakdown behind it.

use std::time::Instant;

use engine_ecs::{Build, World};
use physics3d::{Body, Collider, Position, Static, TIMINGS, Timings, Vec3, Velocity, step};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let n: usize = args.get(1).and_then(|a| a.parse().ok()).unwrap_or(10_000);
    let boxes = args.get(2).is_some_and(|a| a == "box");
    // About 12 layers deep, in a square box.
    let side = ((n as f32 / 12.0).sqrt().ceil() as usize).max(4);
    let half = side as f32 * 0.5;
    let w = World::new();
    let s = step(&w);
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let mut wall = |at: Vec3, h: Vec3| {
            m.spawn((Position { x: at.x, y: at.y, z: at.z }, Collider::cuboid(h), Body::new(0.0), Static {}));
        };
        wall(Vec3::new(0.0, -0.5, 0.0), Vec3::new(half + 1.0, 0.5, half + 1.0));
        let tall = 40.0;
        wall(Vec3::new(half + 0.5, tall, 0.0), Vec3::new(0.5, tall, half + 1.0));
        wall(Vec3::new(-half - 0.5, tall, 0.0), Vec3::new(0.5, tall, half + 1.0));
        wall(Vec3::new(0.0, tall, half + 0.5), Vec3::new(half + 1.0, tall, 0.5));
        wall(Vec3::new(0.0, tall, -half - 0.5), Vec3::new(half + 1.0, tall, 0.5));
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut jitter = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.3
        };
        let per = side - 1;
        for k in 0..n {
            let (i, j, l) = (k % per, (k / per) % per, k / (per * per));
            let at = Vec3::new(
                (i as f32 - per as f32 / 2.0 + 0.5) * 1.05 + jitter(),
                1.0 + l as f32 * 1.1,
                (j as f32 - per as f32 / 2.0 + 0.5) * 1.05 + jitter(),
            );
            let c = if boxes { Collider::cuboid(Vec3::splat(0.5)) } else { Collider::sphere(0.5) };
            m.spawn((Position { x: at.x, y: at.y, z: at.z }, c, Body::new(1.0), Velocity::default()));
        }
    }
    println!("{n} {}, {side} wide: µs per step by stage (ECS, one thread)", if boxes { "boxes" } else { "spheres" });
    println!("| steps | frame | gravity | gather | broadphase | narrowphase | merge | solve: gather | solver | write back | outside systems | pairs | contacts |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let mut done = 0;
    for (from, to) in [(0, 60), (60, 300), (300, 400), (400, 900), (900, 1000)] {
        while done < from {
            s.run_sequential(&w);
            done += 1;
        }
        *TIMINGS.lock().unwrap() = Timings::default();
        let t = Instant::now();
        while done < to {
            s.run_sequential(&w);
            done += 1;
        }
        let frame = t.elapsed().as_secs_f64() * 1e6 / (to - from) as f64;
        let t = *TIMINGS.lock().unwrap();
        let us = |ns: u64| ns as f64 / 1e3 / (to - from) as f64;
        let inside = us(t.gravity + t.gather + t.broadphase + t.narrowphase + t.merge + t.solve_gather + t.solver + t.write_back);
        println!(
            "| {from}-{to} | {frame:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {} | {} |",
            us(t.gravity),
            us(t.gather),
            us(t.broadphase),
            us(t.narrowphase),
            us(t.merge),
            us(t.solve_gather),
            us(t.solver),
            us(t.write_back),
            frame - inside,
            t.pairs,
            t.contacts
        );
    }
}
