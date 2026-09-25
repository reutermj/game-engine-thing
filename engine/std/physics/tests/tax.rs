//! What physics in the ECS costs over the same step on plain arrays:
//! `./bazel run -c opt //engine/std/physics:tax`.
//!
//! A pile runs in the engine (the physics mod, on the lockstep bootstrap)
//! until the frame to measure. Its state is then copied out (bodies, and
//! contacts with their impulses) into arrays, and both run the same frames:
//! the mod in the world, and the same step here, with the same narrowphase
//! and solver, contacts in the same order, and bodies as indices instead of
//! entities. They must end bit for bit the same, or the comparison is of two
//! different computations. Then each stage is timed on both.
//!
//! With `-- parallel`, the same at 1 to 16 threads instead: see `tax_par.rs`.

#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;
#[path = "tax_par.rs"]
mod par;
#[path = "pool.rs"]
mod pool;

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use engine_ecs::{Entity, World};
use engine_loader::engine::Engine;
use physics::{
    Body, Collider, ContactPair, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Placed, Position, Response, STATIC,
    Shape, Vec2, Velocity,
};
use solver::{Constraint, SolverBody};

const DT: f32 = 1.0 / 60.0;

/// One contact as the arrays keep it: by body index, `a < b`.
#[derive(Clone, Copy)]
struct Cached {
    a: u32,
    b: u32,
    normal: Vec2,
    depth: f32,
    friction: f32,
    restitution: f32,
    jn: f32,
    jt: f32,
    pressed: bool,
    was_pressed: bool,
}

/// The pile as arrays, in entity order: index `i` is the `i`th entity with
/// a collider, so index order is entity order and pairs sort the same.
struct Arrays {
    entity: Vec<Entity>,
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    collider: Vec<Collider>,
    /// `Body::fixed()` for colliders without a body, as the mod treats them.
    body: Vec<Body>,
    /// Indices of the bodies with a `Body`: what the solver moves.
    moving: Vec<u32>,
    gravity: Vec2,
    contacts: Vec<Cached>,
    /// Indices by the left edge of their boxes, kept across steps: the
    /// broadphase re-sorts it by insertion, nearly free once sorted.
    by_x: Vec<u32>,
}

#[derive(Default, Clone, Copy)]
struct Stages {
    /// The broadphase again, sorting afresh: not part of the step.
    fresh_sweep: f64,
    gravity: f64,
    broadphase: f64,
    narrowphase: f64,
    merge: f64,
    solve_gather: f64,
    solver: f64,
    write_back: f64,
}

impl Arrays {
    fn snapshot(w: &World) -> Arrays {
        let pos: HashMap<Entity, Position> = w.values::<Position>().unwrap().into_iter().collect();
        let vel: HashMap<Entity, Velocity> = w.values::<Velocity>().unwrap().into_iter().collect();
        let body: HashMap<Entity, Body> = w.values::<Body>().unwrap().into_iter().collect();
        let mut colliders = w.values::<Collider>().unwrap();
        colliders.sort_by_key(|(e, _)| *e);
        let entity: Vec<Entity> = colliders.iter().map(|(e, _)| *e).collect();
        let index: HashMap<Entity, u32> = entity.iter().enumerate().map(|(i, e)| (*e, i as u32)).collect();
        let at = |e: &Entity| pos[e];
        let gravity = w.values::<Gravity>().unwrap().first().map_or(Vec2::ZERO, |(_, g)| Vec2::new(g.x, g.y));
        let manifolds: HashMap<Entity, Manifold> = w.values::<Manifold>().unwrap().into_iter().collect();
        let responses: HashMap<Entity, Response> = w.values::<Response>().unwrap().into_iter().collect();
        let impulses: HashMap<Entity, Impulse> = w.values::<Impulse>().unwrap().into_iter().collect();
        let mut contacts: Vec<Cached> = w
            .values::<ContactPair>()
            .unwrap()
            .into_iter()
            .map(|(e, p)| {
                let (m, r, j) = (manifolds[&e], responses[&e], impulses[&e]);
                Cached {
                    a: index[&p.a],
                    b: index[&p.b],
                    normal: Vec2::new(m.nx, m.ny),
                    depth: m.depth,
                    friction: r.friction,
                    restitution: r.restitution,
                    jn: j.normal,
                    jt: j.tangent,
                    pressed: m.pressed,
                    was_pressed: m.was_pressed,
                }
            })
            .collect();
        contacts.sort_by_key(|c| (c.a, c.b));
        let mut a = Arrays {
            pos: entity.iter().map(|e| Vec2::new(at(e).x, at(e).y)).collect(),
            vel: entity.iter().map(|e| vel.get(e).map_or(Vec2::ZERO, |v| Vec2::new(v.x, v.y))).collect(),
            collider: colliders.iter().map(|(_, c)| *c).collect(),
            body: entity.iter().map(|e| body.get(e).copied().unwrap_or_else(Body::fixed)).collect(),
            moving: entity.iter().enumerate().filter(|(_, e)| body.contains_key(e)).map(|(i, _)| i as u32).collect(),
            by_x: (0..entity.len() as u32).collect(),
            entity,
            gravity,
            contacts,
        };
        a.sort_by_x();
        a
    }

    fn placed(&self, i: usize) -> Placed {
        Placed { shape: Shape::of(&self.collider[i]), at: self.pos[i] }
    }

    fn sort_by_x(&mut self) {
        let min_x: Vec<f32> = (0..self.pos.len()).map(|i| self.placed(i).aabb().min.x).collect();
        // Insertion sort: bodies move a little each step, so this is linear
        // once the order is established.
        for k in 1..self.by_x.len() {
            let mut j = k;
            while j > 0 && min_x[self.by_x[j - 1] as usize] > min_x[self.by_x[j] as usize] {
                self.by_x.swap(j - 1, j);
                j -= 1;
            }
        }
    }

    /// One step, as the physics mod takes it.
    fn step(&mut self, t: &mut Stages) {
        let start = Instant::now();
        for &i in &self.moving {
            let (b, v) = (&self.body[i as usize], &mut self.vel[i as usize]);
            if b.kind == DYNAMIC {
                v.x += self.gravity.x * b.gravity_scale * DT;
                v.y += self.gravity.y * b.gravity_scale * DT;
            }
        }
        let gravity = Instant::now();

        // Sweep and prune along x over boxes grown by the margin, as
        // `near_pairs` grows them.
        self.sort_by_x();
        let boxes: Vec<physics::Aabb> = (0..self.pos.len())
            .map(|i| {
                let b = self.placed(i).aabb();
                let m = Vec2::new(narrow::MARGIN, narrow::MARGIN);
                physics::Aabb { min: b.min - m, max: b.max + m }
            })
            .collect();
        let mut pairs: Vec<(u32, u32)> = Vec::new();
        for (k, &i) in self.by_x.iter().enumerate() {
            let a = boxes[i as usize];
            for &j in &self.by_x[k + 1..] {
                let b = boxes[j as usize];
                if b.min.x > a.max.x {
                    break;
                }
                if a.min.y <= b.max.y && b.min.y <= a.max.y {
                    pairs.push((i.min(j), i.max(j)));
                }
            }
        }
        pairs.sort_unstable();
        let broadphase = Instant::now();
        let pair_count = pairs.len();

        let mut found: Vec<Cached> = Vec::new();
        for (i, j) in pairs {
            let (a, b) = (i as usize, j as usize);
            let (ca, cb, ba, bb) = (&self.collider[a], &self.collider[b], &self.body[a], &self.body[b]);
            let meets = ca.mask & cb.layer != 0 && cb.mask & ca.layer != 0;
            if !meets || ca.sensor || cb.sensor || (ba.kind != DYNAMIC && bb.kind != DYNAMIC) {
                continue;
            }
            let Some(m) = narrow::collide(&self.placed(a), &self.placed(b), self.vel[b] - self.vel[a]) else { continue };
            found.push(Cached {
                a: i,
                b: j,
                normal: m.normal,
                depth: m.depth,
                friction: ba.friction.min(bb.friction),
                restitution: ba.restitution.max(bb.restitution),
                jn: 0.0,
                jt: 0.0,
                pressed: false,
                was_pressed: false,
            });
        }
        let narrowphase = Instant::now();

        let mut k = 0;
        for f in &mut found {
            while k < self.contacts.len() && (self.contacts[k].a, self.contacts[k].b) < (f.a, f.b) {
                k += 1;
            }
            if let Some(old) = self.contacts.get(k).filter(|c| (c.a, c.b) == (f.a, f.b)) {
                (f.jn, f.jt, f.was_pressed) = (old.jn, old.jt, old.pressed);
            }
        }
        self.contacts = found;
        let merge = Instant::now();

        // Statics all stand for one immovable body at the end, as in the mod.
        let mut slot = vec![u32::MAX; self.pos.len()];
        let mut bodies: Vec<SolverBody> = Vec::with_capacity(self.moving.len() + 1);
        for (s, &i) in self.moving.iter().enumerate() {
            let b = &self.body[i as usize];
            let inv_mass = if b.kind == DYNAMIC { b.inv_mass } else { 0.0 };
            bodies.push(SolverBody { v: self.vel[i as usize], inv_mass, pseudo: Vec2::ZERO });
            slot[i as usize] = s as u32;
        }
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        let at = |i: u32| if slot[i as usize] == u32::MAX { still } else { slot[i as usize] };
        let mut constraints: Vec<Constraint> = self
            .contacts
            .iter()
            .map(|c| Constraint {
                a: at(c.a),
                b: at(c.b),
                normal: c.normal,
                depth: c.depth,
                friction: c.friction,
                restitution: c.restitution,
                jn: c.jn,
                jt: c.jt,
                speed: 0.0,
            })
            .collect();
        let solve_gather = Instant::now();
        solver::solve(&mut bodies, &mut constraints, DT);
        let solved = Instant::now();

        for (s, &i) in self.moving.iter().enumerate() {
            let (b, body) = (bodies[s], self.body[i as usize]);
            if body.kind == STATIC {
                continue;
            }
            self.vel[i as usize] = b.v;
            let step = if body.kind == KINEMATIC { b.v } else { b.v + b.pseudo };
            self.pos[i as usize] = Vec2::new(self.pos[i as usize].x + step.x * DT, self.pos[i as usize].y + step.y * DT);
        }
        for (c, k) in self.contacts.iter_mut().zip(&constraints) {
            (c.jn, c.jt) = (k.jn, k.jt);
            c.pressed = k.jn > 0.0 || c.depth >= 0.0;
        }
        let done = Instant::now();

        // The same sweep with no order kept from the last step, over this
        // step's boxes: what the persistent order is worth. Timed only.
        let fresh_start = Instant::now();
        let mut by_x: Vec<u32> = (0..boxes.len() as u32).collect();
        by_x.sort_unstable_by(|&i, &j| boxes[i as usize].min.x.total_cmp(&boxes[j as usize].min.x));
        let mut fresh: Vec<(u32, u32)> = Vec::new();
        for (k, &i) in by_x.iter().enumerate() {
            let a = boxes[i as usize];
            for &j in &by_x[k + 1..] {
                let b = boxes[j as usize];
                if b.min.x > a.max.x {
                    break;
                }
                if a.min.y <= b.max.y && b.min.y <= a.max.y {
                    fresh.push((i.min(j), i.max(j)));
                }
            }
        }
        fresh.sort_unstable();
        t.fresh_sweep += fresh_start.elapsed().as_secs_f64() * 1e6;
        assert_eq!(fresh.len(), pair_count);

        let us = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1e6;
        t.gravity += us(start, gravity);
        t.broadphase += us(gravity, broadphase);
        t.narrowphase += us(broadphase, narrowphase);
        t.merge += us(narrowphase, merge);
        t.solve_gather += us(merge, solve_gather);
        t.solver += us(solve_gather, solved);
        t.write_back += us(solved, done);
    }
}

/// Bodies, and how wide the box is they're dropped in. A box whose rows
/// hold an odd number of bodies (40 and 400 wide) stacks them in columns
/// that alternate circles and boxes and never touch their neighbors: without
/// rotation a circle on a box stays put, so each body rests on one contact
/// and each column is an island. One body more or less a row (41 and 401
/// wide) packs them into one pile, with half again the contacts: the pile
/// a solver is up against (2026-09-24, found by the parallel solver's
/// work).
const PILES: [(u32, f32); 4] = [(1000, 40.0), (1000, 41.0), (10000, 400.0), (10000, 401.0)];

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

fn main() {
    let manifest = engine_control::read_manifest(&std::env::var("PILE").unwrap()).unwrap();
    if std::env::args().any(|a| a == "parallel") {
        par::run(&manifest);
        return;
    }
    const FRAMES: u32 = 60;
    println!("µs per step, {FRAMES} steps, -c opt, one thread; ECS / arrays\n");
    println!("| bodies | box | scene | contacts | frame | gravity | gather | broadphase | narrowphase | merge | solve: gather | solver | write back | outside systems |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (n, width) in PILES {
        // Settled is still creeping (every body 1e-4 to 1e-2 a step); at rest
        // is still bit for bit, which the 10 000 are by about step 3000.
        for (scene, warmup) in [("falling", 1u32), ("settled", 400), ("at rest", 4000)] {
            let dir = std::env::temp_dir().join(format!("physics-tax-{}-{n}-{}", std::process::id(), scene.replace(' ', "-")));
            let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
            e.load_batch(&manifest.mods).expect("loading the pile");
            e.send("pile", &format!("widen {width}")).unwrap();
            e.send("pile", &format!("drop {n}")).unwrap();
            e.send("lockstep", &format!("step {warmup}")).unwrap();

            let mut arrays = Arrays::snapshot(e.world());
            let mut t = Stages::default();
            let start = Instant::now();
            for _ in 0..FRAMES {
                arrays.step(&mut t);
            }
            let array_frame = (start.elapsed().as_secs_f64() * 1e6 - t.fresh_sweep) / FRAMES as f64;

            e.send("physics", "reset_timings").unwrap();
            let start = Instant::now();
            e.send("lockstep", &format!("step {FRAMES}")).unwrap();
            let ecs_frame = start.elapsed().as_secs_f64() * 1e6 / FRAMES as f64;
            let (stats, stages) = (e.send("physics", "stats").unwrap(), e.send("physics", "stages").unwrap());

            // The same computation, or the numbers mean nothing.
            let ecs: HashMap<Entity, Position> = e.world().values::<Position>().unwrap().into_iter().collect();
            let differ = arrays
                .entity
                .iter()
                .zip(&arrays.pos)
                .filter(|(e, p)| ecs[e].x.to_bits() != p.x.to_bits() || ecs[e].y.to_bits() != p.y.to_bits())
                .count();
            assert_eq!(differ, 0, "{n} {scene}: {differ} bodies ended elsewhere than the arrays put them");
            assert_eq!(field(&stats, "contacts") as usize, arrays.contacts.len(), "{n} {scene}: contacts");

            let f = FRAMES as f64;
            let per_step = stats.split("us/step").nth(1).expect("timings in stats");
            let systems = field(per_step, "gravity") + field(per_step, "contacts") + field(per_step, "solve");
            let pair = |ecs: f64, arr: f64| format!("{ecs:.0} / {arr:.0}");
            println!("  (the arrays' sweep, sorting afresh each step: {:.0} µs)", t.fresh_sweep / f);
            println!(
                "| {n} | {width} | {scene} | {} | {} | {} | {} / – | {} | {} | {} | {} | {} | {} | {:.0} |",
                arrays.contacts.len(),
                pair(ecs_frame, array_frame),
                pair(field(per_step, "gravity"), t.gravity / f),
                field(&stages, "gather").round(),
                pair(field(&stages, "broadphase"), t.broadphase / f),
                pair(field(&stages, "narrowphase"), t.narrowphase / f),
                pair(field(&stages, "merge"), t.merge / f),
                pair(field(&stages, "solve_gather"), t.solve_gather / f),
                pair(field(&stages, "solver"), t.solver / f),
                pair(field(&stages, "write_back"), t.write_back / f),
                ecs_frame - systems,
            );
            drop(e);
            let _ = std::fs::remove_dir_all(dir);
        }
    }
    sleeping(&manifest, FRAMES);
}

/// Sleeping, which changes the simulation, so the ECS alone: the pile with
/// `Sleep` on, from when all of it is asleep, against the same pile awake at
/// the same step.
fn sleeping(manifest: &engine_control::Manifest, frames: u32) {
    const SPEED: f32 = 0.05;
    const TIME: f32 = 0.5;
    println!("\nSleeping (speed {SPEED}, {TIME} s), ECS only: µs per step, asleep / awake at the same step\n");
    println!("| bodies | asleep at step | frame | gravity | gather | broadphase | narrowphase | merge | solve: gather | solver | write back | outside systems | deepest overlap |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (n, width) in [(1000u32, 40.0f32), (10000, 400.0)] {
        let run = |sleep: bool, until: Option<u32>| {
            let dir = std::env::temp_dir().join(format!("physics-tax-{}-{n}-sleep-{sleep}", std::process::id()));
            let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
            e.load_batch(&manifest.mods).expect("loading the pile");
            e.send("pile", &format!("widen {width}")).unwrap();
            e.send("pile", &format!("drop {n}")).unwrap();
            if sleep {
                e.send("pile", &format!("sleep {SPEED} {TIME}")).unwrap();
            }
            let mut steps = 0;
            match until {
                Some(s) => {
                    e.send("lockstep", &format!("step {s}")).unwrap();
                    steps = s;
                }
                None => {
                    while field(&e.send("physics", "sleeping").unwrap(), "asleep") < n as f64 && steps < 6000 {
                        e.send("lockstep", "step 10").unwrap();
                        steps += 10;
                    }
                }
            }
            e.send("physics", "reset_timings").unwrap();
            let start = Instant::now();
            e.send("lockstep", &format!("step {frames}")).unwrap();
            let frame = start.elapsed().as_secs_f64() * 1e6 / frames as f64;
            let (stats, stages) = (e.send("physics", "stats").unwrap(), e.send("physics", "stages").unwrap());
            let per_step = stats.split("us/step").nth(1).expect("timings in stats").to_string();
            let systems = field(&per_step, "gravity") + field(&per_step, "contacts") + field(&per_step, "solve");
            let pile = e.send("pile", "stats").unwrap();
            drop(e);
            let _ = std::fs::remove_dir_all(dir);
            (steps, frame, stages, frame - systems, field(&pile, "deepest"))
        };
        let (at, frame, stages, outside, deepest) = run(true, None);
        let (_, frame_awake, stages_awake, outside_awake, deepest_awake) = run(false, Some(at));
        let pair = |k: &str| format!("{:.0} / {:.0}", field(&stages, k), field(&stages_awake, k));
        println!(
            "| {n} | {at} | {frame:.0} / {frame_awake:.0} | {} | {} | {} | {} | {} | {} | {} | {} | {outside:.0} / {outside_awake:.0} | {deepest:.3} / {deepest_awake:.3} |",
            pair("gravity"),
            pair("gather"),
            pair("broadphase"),
            pair("narrowphase"),
            pair("merge"),
            pair("solve_gather"),
            pair("solver"),
            pair("write_back"),
        );
    }
}
