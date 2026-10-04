//! Our side: the physics mod in the engine, and the same step on plain
//! arrays (`tests/arrays.rs`, which `:tax` checks against the mod bit for
//! bit), each on the comparison's scenes.

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use engine_ecs::Entity;
use engine_loader::engine::Engine;
use physics2d::{Asleep, Body, Collider, ContactPair, ContactPoints, DYNAMIC, Manifold, Position, Rot, Rotation, Spin, Vec2, Velocity};

use crate::arrays::{Arrays, Stages};
use crate::scene::{DT, Scene, Spec};
use crate::sim::Mark;
use crate::{Dyn, Sim};

/// Where a contact without points touches, for the view: on the face of
/// the smaller of its two shapes, toward the other, which is right for a
/// body on a floor or a wall and near enough between equals.
fn estimate(a: (Vec2, &Collider), b: (Vec2, &Collider), n: Vec2, speculative: bool) -> Mark {
    let ext = |c: &Collider| if c.shape == physics2d::CIRCLE { c.hx } else { n.x.abs() * c.hx + n.y.abs() * c.hy };
    let p = if a.1.reach() <= b.1.reach() { a.0 + n * ext(a.1) } else { b.0 - n * ext(b.1) };
    Mark { x: p.x, y: p.y, nx: n.x, ny: n.y, estimated: true, speculative }
}

/// A contact's points as `ContactPoints` has them, each at its arm from `a`.
fn points(at_a: Vec2, cp: &ContactPoints, count: u8, n: Vec2, speculative: bool) -> impl Iterator<Item = Mark> + '_ {
    (0..count as usize).map(move |k| {
        let p = at_a + cp.anchors(k).0;
        Mark { x: p.x, y: p.y, nx: n.x, ny: n.y, estimated: false, speculative }
    })
}

/// `TURN=2`: where rotation is locked, ours still gives every dynamic body a
/// `Rotation` (and no `Spin`), as a lock by a flag or by infinite inertia
/// would leave every body carrying one: what that costs a world where
/// nothing turns (physics.md, "The rotation lock").
fn oriented() -> bool {
    std::env::var("TURN").is_ok_and(|t| t == "2")
}

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
    pub fn new(manifest: &engine_control::Manifest, scene: &Scene, sleep: bool, turning: bool) -> Ecs {
        static RUN: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let run = RUN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("physics-compare-{}-{run}", std::process::id()));
        let engine = Engine::new(manifest.bootstrap.clone(), dir.clone());
        // The scene game less its thread host, and the process's one pool in
        // its place: a harness makes thousands of engines, and each load of
        // the host costs a TLS key (`engine_threads::shared`). The tests and
        // the baseline run on it; the comparison times ours `alone`, as the
        // other engines run.
        let mods: Vec<_> = manifest.mods.iter().filter(|(name, _)| name != engine_threads::MOD_NAME).cloned().collect();
        engine.load_batch(&mods).expect("loading the scene");
        engine.world().set_executor(engine_threads::shared().map(|p| p as std::sync::Arc<dyn engine_ecs::Executor>));
        // The mod reads the scene back from its text.
        assert_eq!(Scene::parse(&scene.text()), Some(*scene));
        // Its phase is fixed-rate: only the arrays step at another.
        assert_eq!(scene.dt(), DT, "the mod steps at 60 Hz, not as {} asks", scene.text());
        engine.send("scene", &format!("build {}", scene.text())).unwrap();
        let mut label = "ours (ECS)".to_string();
        if turning {
            engine.send("scene", "turn").unwrap();
        } else if oriented() {
            engine.send("scene", "orient").unwrap();
            label += ", oriented";
        }
        if sleep {
            engine.send("scene", "sleep default").unwrap();
            label += ", sleeping";
        }
        Ecs { engine, dir, label, wall: 0.0, ticks: 0 }
    }

    /// On the threads of `executor` in place of the process's pool
    /// (`engine_threads::shared`, which the quality tests and the baseline
    /// run on, as `ENGINE_THREADS` sets it).
    #[allow(dead_code)] // The comparison sets it; the quality tests don't.
    pub fn on_threads(mut self, executor: std::sync::Arc<dyn engine_ecs::Executor>) -> Ecs {
        self.label += &format!(", {} threads", executor.threads());
        self.engine.world().set_executor(Some(executor));
        self
    }

    /// On the frame's thread alone: no pool, as the other engines are
    /// timed.
    #[allow(dead_code)] // The comparison's; the quality tests run on the game's threads.
    pub fn alone(self) -> Ecs {
        self.engine.world().set_executor(None);
        self
    }
}

impl Ecs {
    /// The engine, for a bench or test that sets the world's switches or
    /// reads physics's stages itself (`step_bench.rs`, `quality_test.rs`).
    #[allow(dead_code)]
    pub fn engine(&self) -> &Engine {
        &self.engine
    }

    /// The dynamic bodies, in the order they came.
    fn dynamic(&self) -> Vec<Entity> {
        let w = self.engine.world();
        let mut bodies: Vec<Entity> = w.values::<Body>().unwrap().into_iter().filter(|(_, b)| b.kind == DYNAMIC).map(|(e, _)| e).collect();
        bodies.sort();
        bodies
    }

    /// Bodies physics has asleep: its count, which its tests check against
    /// the world's sleeping tables.
    #[allow(dead_code)] // The quality tests read it; the comparison doesn't.
    pub fn asleep(&self) -> usize {
        field(&self.engine.send("physics2d", "sleeping").unwrap(), "asleep") as usize
    }

    /// Physics's substeps, set as a game sets them: a `Tuning` in the world.
    #[allow(dead_code)] // The quality tests set it; the comparison doesn't.
    pub fn substeps(&mut self, n: u32) {
        self.engine.send("scene", &format!("substeps {n}")).unwrap();
        self.label += &format!(", {n} substeps");
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
        let rotations: HashMap<Entity, Rotation> = w.values::<Rotation>().unwrap_or_default().into_iter().collect();
        let spins: HashMap<Entity, Spin> = w.values::<Spin>().unwrap_or_default().into_iter().collect();
        self.dynamic()
            .iter()
            .map(|e| {
                let (p, v, c) = (pos[e], vel[e], colliders[e]);
                let angle = rotations.get(e).map_or(0.0, |q| q.angle());
                let w = spins.get(e).map_or(0.0, |s| s.w);
                Dyn { circle: c.shape == physics2d::CIRCLE, hx: c.hx, hy: c.hy, x: p.x, y: p.y, vx: v.x, vy: v.y, angle, w }
            })
            .collect()
    }

    fn reset(&mut self) {
        self.engine.send("physics2d", "reset_timings").unwrap();
        self.wall = 0.0;
    }

    /// Physics's own timings, which it keeps per system and stage, as
    /// totals; what the frame spends outside the systems is mostly the
    /// spatial re-sort and the contacts' at apply nodes (physics.md, "What
    /// the ECS costs").
    fn stages(&self) -> Vec<(String, f64)> {
        let stats = self.engine.send("physics2d", "stats").unwrap();
        let stages = self.engine.send("physics2d", "stages").unwrap();
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
        let stats = self.engine.send("physics2d", "stats").unwrap();
        format!("contacts {} (pressed {})", field(&stats, "contacts"), self.contacts())
    }

    /// Every contact in the world, its points from its `ContactPoints`
    /// where either end turns, else estimated from its normal.
    fn marks(&self) -> Vec<Mark> {
        let w = self.engine.world();
        let pos: HashMap<Entity, Position> = w.values::<Position>().unwrap().into_iter().collect();
        let colliders: HashMap<Entity, Collider> = w.values::<Collider>().unwrap().into_iter().collect();
        let manifolds: HashMap<Entity, Manifold> = w.values::<Manifold>().unwrap_or_default().into_iter().collect();
        let cps: HashMap<Entity, ContactPoints> = w.values::<ContactPoints>().unwrap_or_default().into_iter().collect();
        let at = |e: &Entity| Vec2::new(pos[e].x, pos[e].y);
        let mut out = Vec::new();
        for (c, pair) in w.values::<ContactPair>().unwrap_or_default() {
            let m = &manifolds[&c];
            let n = Vec2::new(m.nx, m.ny);
            if m.points > 0 {
                out.extend(points(at(&pair.a), &cps[&c], m.points, n, !m.pressed));
            } else {
                out.push(estimate((at(&pair.a), &colliders[&pair.a]), (at(&pair.b), &colliders[&pair.b]), n, !m.pressed));
            }
        }
        out
    }

    fn sleeping(&self) -> Vec<bool> {
        let asleep: HashMap<Entity, Asleep> = self.engine.world().values::<Asleep>().unwrap_or_default().into_iter().collect();
        self.dynamic().iter().map(|e| asleep.contains_key(e)).collect()
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
    /// Whether the bodies turn, and so rain's drops do.
    turning: bool,
}

fn collider(s: &Spec) -> Collider {
    if s.circle { Collider::circle(s.hx) } else { Collider::rect(s.hx, s.hy) }
}

fn body(s: &Spec) -> Body {
    let material = |b: Body| Body { friction: s.friction, restitution: s.restitution, ..b };
    if s.dynamic {
        material(Body { inv_mass: 1.0 / s.mass, gravity_scale: s.gravity_scale, bullet: s.bullet, ..Body::default() })
    } else if s.kinematic {
        material(Body::kinematic())
    } else {
        material(Body::fixed())
    }
}

impl Flat {
    /// Ours on `scene`: the variant `spec`, or with "" ours as built, at
    /// the substeps the scene sets (a `rot` variant's `sub`), if it does.
    pub fn ours(scene: &Scene, turning: bool, spec: &str, label: &str) -> Flat {
        let spec = match (scene.substeps(), spec) {
            (None, _) => spec.to_string(),
            (Some(n), "") => format!("rot/sub={n}"),
            (Some(n), s) if s.starts_with("rot") => format!("{s}/sub={n}"),
            (Some(_), s) => panic!("{s} has no substeps to set, as {} asks", scene.text()),
        };
        if spec.is_empty() {
            Flat::new(scene, turning, Box::new(crate::solver::solve_points), label)
        } else {
            Flat::variant(scene, turning, &spec, label)
        }
    }

    /// One of the variants of `variants.rs` (`arrays:<spec>`): a solver, and
    /// how its points are warm-started.
    pub fn variant(scene: &Scene, turning: bool, spec: &str, label: &str) -> Flat {
        let v = crate::variants::parse(spec);
        let mut flat = Flat::new(scene, turning, v.solve, label);
        (flat.arrays.warm, flat.arrays.deepest) = (v.warm, v.deepest);
        flat
    }

    pub fn new(scene: &Scene, turning: bool, solve: crate::variants::Boxed, label: &str) -> Flat {
        let specs = scene.build();
        let statics = specs.iter().take_while(|s| !s.dynamic).count();
        assert!(specs[statics..].iter().all(|s| s.dynamic), "statics first");
        let mut arrays = Arrays::of(
            specs.iter().map(|s| Vec2::new(s.x, s.y)).collect(),
            specs.iter().map(collider).collect(),
            specs.iter().map(body).collect(),
            (0..specs.len() as u32).filter(|&i| specs[i as usize].dynamic || specs[i as usize].kinematic).collect(),
            Vec2::new(0.0, scene.gravity()),
        );
        arrays.dt = scene.dt();
        for (v, s) in arrays.vel.iter_mut().zip(&specs) {
            *v = Vec2::new(s.vx, s.vy);
        }
        let mut label = label.to_string();
        if turning {
            arrays = arrays.turning();
            for (i, s) in specs.iter().enumerate() {
                arrays.spin[i] = arrays.spin[i].map(|_| s.w);
            }
        } else if oriented() {
            for (q, b) in arrays.rot.iter_mut().zip(&arrays.body) {
                if b.kind == DYNAMIC {
                    *q = Some(Rot::IDENTITY);
                }
            }
            label += ", oriented";
        }
        // A turned static is a ramp, and a turned body starts so.
        for (i, s) in specs.iter().enumerate() {
            if s.angle != 0.0 {
                arrays.rot[i] = Some(Rot::from_angle(s.angle));
            }
        }
        Flat { arrays, solve, label, t: Stages::default(), wall: 0.0, statics, turning }
    }
}

impl Sim for Flat {
    fn label(&self) -> String {
        self.label.clone()
    }

    fn step(&mut self, n: u32) {
        let start = Instant::now();
        for _ in 0..n {
            self.arrays.step(&mut self.t, crate::arrays::WithPoints(&*self.solve));
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
            a.rot.push((self.turning || oriented()).then_some(Rot::IDENTITY));
            a.spin.push(self.turning.then_some(0.0));
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
        a.rot.drain(range.clone());
        a.spin.drain(range.clone());
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
        // The kinematic ones aren't the scene's bodies (as in the mod).
        a.moving
            .iter()
            .filter(|&&i| a.body[i as usize].kind == DYNAMIC)
            .map(|&i| {
                let (c, p, v) = (&a.collider[i as usize], a.pos[i as usize], a.vel[i as usize]);
                let (angle, w) = (a.rot[i as usize].map_or(0.0, |q| q.angle()), a.spin[i as usize].unwrap_or(0.0));
                Dyn { circle: c.shape == physics2d::CIRCLE, hx: c.hx, hy: c.hy, x: p.x, y: p.y, vx: v.x, vy: v.y, angle, w }
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

    fn marks(&self) -> Vec<Mark> {
        let a = &self.arrays;
        let mut out = Vec::new();
        for c in &a.contacts {
            let (i, j) = (c.a as usize, c.b as usize);
            if c.points > 0 {
                let p = &a.points[c.points as usize - 1];
                out.extend(points(a.pos[i], &p.cp, p.count, c.normal, !c.pressed));
            } else {
                out.push(estimate((a.pos[i], &a.collider[i]), (a.pos[j], &a.collider[j]), c.normal, !c.pressed));
            }
        }
        out
    }
}
