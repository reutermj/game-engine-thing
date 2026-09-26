//! Our side: the physics mod in the engine, and the same step on plain
//! arrays (`tests/arrays.rs`, which `:tax` checks against the mod bit for
//! bit), each on the comparison's scenes.

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use engine_ecs::Entity;
use engine_loader::engine::Engine;
use physics::{Body, Collider, DYNAMIC, Manifold, Position, Vec2, Velocity};

use crate::arrays::{Arrays, Stages};
use crate::scene::{GRAVITY, Scene, Spec};
use crate::{Dyn, Sim};

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

pub struct Ecs {
    engine: Box<Engine>,
    dir: PathBuf,
    label: String,
    /// Wall time in the frames stepped since `reset`, in µs.
    wall: f64,
    /// Rain steps the bench has asked for, which the scene mod's system
    /// must have run as many of.
    ticks: u32,
}

impl Ecs {
    pub fn new(manifest: &engine_control::Manifest, scene: &Scene, sleep: bool) -> Ecs {
        static RUN: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let run = RUN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("physics-compare-{}-{run}", std::process::id()));
        let engine = Engine::new(manifest.bootstrap.clone(), dir.clone());
        engine.load_batch(&manifest.mods).expect("loading the scene");
        // The mod reads the scene back from its text.
        assert_eq!(Scene::parse(&scene.text()), Some(*scene));
        engine.send("scene", &format!("build {}", scene.text())).unwrap();
        let mut label = "ours (ECS)".to_string();
        if sleep {
            engine.send("scene", "sleep default").unwrap();
            label += ", sleeping";
        }
        Ecs { engine, dir, label, wall: 0.0, ticks: 0 }
    }
}

impl Drop for Ecs {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

impl Sim for Ecs {
    fn label(&self) -> String {
        self.label.clone()
    }

    fn step(&mut self, n: u32) {
        let start = Instant::now();
        self.engine.send("lockstep", &format!("step {n}")).unwrap();
        self.wall += start.elapsed().as_secs_f64() * 1e6;
    }

    /// Nothing to do: the scene mod's `rain` system makes the same
    /// arrivals from the same code in the step, and removes the oldest the
    /// same way, so its cost is in the step, where a game's would be.
    fn rain(&mut self, arrivals: &[Spec], _: usize) {
        assert!(!arrivals.is_empty());
        self.ticks += 1;
    }

    fn bodies(&self) -> Vec<Dyn> {
        if self.ticks > 0 {
            let said = self.engine.send("scene", "drops").unwrap();
            assert_eq!(field(&said, "tick") as u32, self.ticks, "the scene mod rained as often as the others");
        }
        let w = self.engine.world();
        let pos: HashMap<Entity, Position> = w.values::<Position>().unwrap().into_iter().collect();
        let vel: HashMap<Entity, Velocity> = w.values::<Velocity>().unwrap().into_iter().collect();
        let colliders: HashMap<Entity, Collider> = w.values::<Collider>().unwrap().into_iter().collect();
        let mut bodies: Vec<(Entity, Body)> = w.values::<Body>().unwrap().into_iter().filter(|(_, b)| b.kind == DYNAMIC).collect();
        bodies.sort_by_key(|(e, _)| *e);
        bodies
            .iter()
            .map(|(e, _)| {
                let (p, v, c) = (pos[e], vel[e], colliders[e]);
                Dyn { circle: c.shape == physics::CIRCLE, hx: c.hx, hy: c.hy, x: p.x, y: p.y, vx: v.x, vy: v.y, angle: 0.0 }
            })
            .collect()
    }

    fn reset(&mut self) {
        self.engine.send("physics", "reset_timings").unwrap();
        self.wall = 0.0;
    }

    /// Physics's own timings, which it keeps per system and stage, as
    /// totals; what the frame spends outside the systems is mostly the
    /// spatial re-sort and the contacts' at apply nodes (physics.md, "What
    /// the ECS costs").
    fn stages(&self) -> Vec<(String, f64)> {
        let stats = self.engine.send("physics", "stats").unwrap();
        let stages = self.engine.send("physics", "stages").unwrap();
        let steps = field(&stats, "steps");
        let per_step = stats.split("us/step").nth(1).expect("timings in stats");
        let systems = (field(per_step, "gravity") + field(per_step, "contacts") + field(per_step, "solve")) * steps;
        let mut v = vec![
            ("broadphase".to_string(), field(&stages, "broadphase") * steps),
            ("narrowphase".to_string(), field(&stages, "narrowphase") * steps),
            ("solver".to_string(), field(&stages, "solver") * steps),
        ];
        for k in ["gravity", "gather", "broadphase", "near", "narrowphase", "merge", "solve_gather", "solver", "write_back"] {
            v.push((format!("ours:{k}"), field(&stages, k) * steps));
        }
        v.push(("ours:outside the systems".to_string(), self.wall - systems));
        v.push(("engine step".to_string(), self.wall));
        v
    }

    /// Contacts pressed this step (touching or pushed on): the rest are
    /// speculative, within the margin.
    fn contacts(&self) -> usize {
        self.engine.world().values::<Manifold>().unwrap().iter().filter(|(_, m)| m.pressed).count()
    }

    fn native(&self) -> String {
        let stats = self.engine.send("physics", "stats").unwrap();
        format!("contacts {} (pressed {})", field(&stats, "contacts"), self.contacts())
    }
}

pub struct Flat {
    arrays: Arrays,
    solve: crate::variants::Boxed,
    label: String,
    t: Stages,
    wall: f64,
    /// Where the dynamic bodies start: statics come first.
    statics: usize,
}

fn collider(s: &Spec) -> Collider {
    if s.circle { Collider::circle(s.hx) } else { Collider::rect(s.hx, s.hy) }
}

fn body(s: &Spec) -> Body {
    if s.dynamic { Body { friction: s.friction, restitution: s.restitution, ..Body::default() } } else { Body::fixed() }
}

impl Flat {
    pub fn new(scene: &Scene, solve: crate::variants::Boxed, label: &str) -> Flat {
        let specs = scene.build();
        let statics = specs.iter().take_while(|s| !s.dynamic).count();
        assert!(specs[statics..].iter().all(|s| s.dynamic), "statics first");
        let mut arrays = Arrays::of(
            specs.iter().map(|s| Vec2::new(s.x, s.y)).collect(),
            specs.iter().map(collider).collect(),
            specs.iter().map(body).collect(),
            (statics as u32..specs.len() as u32).collect(),
            Vec2::new(0.0, GRAVITY),
        );
        for (v, s) in arrays.vel.iter_mut().zip(&specs) {
            *v = Vec2::new(s.vx, s.vy);
        }
        Flat { arrays, solve, label: label.to_string(), t: Stages::default(), wall: 0.0, statics }
    }
}

impl Sim for Flat {
    fn label(&self) -> String {
        self.label.clone()
    }

    fn step(&mut self, n: u32) {
        let start = Instant::now();
        for _ in 0..n {
            self.arrays.step(&mut self.t, &*self.solve);
        }
        self.wall += start.elapsed().as_secs_f64() * 1e6;
    }

    /// Appended, since index order is the order of arrival, and the oldest
    /// removed in one pass, every index after them moved down: contacts
    /// keep their pair order, so the merge still finds last step's.
    fn rain(&mut self, arrivals: &[Spec], alive: usize) {
        let a = &mut self.arrays;
        for s in arrivals {
            let i = a.pos.len() as u32;
            a.entity.push(Entity { index: a.entity.last().map_or(0, |e| e.index + 1), generation: 0 });
            a.pos.push(Vec2::new(s.x, s.y));
            a.vel.push(Vec2::new(s.vx, s.vy));
            a.collider.push(collider(s));
            a.body.push(body(s));
            a.moving.push(i);
            a.by_x.push(i);
        }
        let gone = a.moving.len().saturating_sub(alive);
        if gone == 0 {
            return;
        }
        let (from, to) = (self.statics as u32, (self.statics + gone) as u32);
        let keep = |i: u32| i < from || i >= to;
        let moved = |i: u32| if i >= to { i - gone as u32 } else { i };
        let range = self.statics..self.statics + gone;
        a.entity.drain(range.clone());
        a.pos.drain(range.clone());
        a.vel.drain(range.clone());
        a.collider.drain(range.clone());
        a.body.drain(range);
        a.moving = a.moving.iter().copied().filter(|&i| keep(i)).map(moved).collect();
        a.by_x = a.by_x.iter().copied().filter(|&i| keep(i)).map(moved).collect();
        a.contacts.retain(|c| keep(c.a) && keep(c.b));
        for c in &mut a.contacts {
            (c.a, c.b) = (moved(c.a), moved(c.b));
        }
    }

    fn bodies(&self) -> Vec<Dyn> {
        let a = &self.arrays;
        a.moving
            .iter()
            .map(|&i| {
                let (c, p, v) = (&a.collider[i as usize], a.pos[i as usize], a.vel[i as usize]);
                Dyn { circle: c.shape == physics::CIRCLE, hx: c.hx, hy: c.hy, x: p.x, y: p.y, vx: v.x, vy: v.y, angle: 0.0 }
            })
            .collect()
    }

    fn reset(&mut self) {
        self.t = Stages::default();
        self.wall = 0.0;
    }

    fn stages(&self) -> Vec<(String, f64)> {
        let t = &self.t;
        vec![
            ("broadphase".to_string(), t.broadphase),
            ("narrowphase".to_string(), t.narrowphase),
            ("solver".to_string(), t.solver),
            ("ours:gravity".to_string(), t.gravity),
            ("ours:broadphase".to_string(), t.broadphase),
            ("ours:narrowphase".to_string(), t.narrowphase),
            ("ours:merge".to_string(), t.merge),
            ("ours:solve_gather".to_string(), t.solve_gather),
            ("ours:solver".to_string(), t.solver),
            ("ours:write_back".to_string(), t.write_back),
            ("engine step".to_string(), self.wall),
        ]
    }

    fn contacts(&self) -> usize {
        self.arrays.contacts.iter().filter(|c| c.pressed).count()
    }

    fn native(&self) -> String {
        format!("contacts {} (pressed {})", self.arrays.contacts.len(), self.contacts())
    }
}
