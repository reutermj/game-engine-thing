//! Our own step: the physics3d mod in the engine (on the lockstep
//! bootstrap, its systems on the threads `Config::threads` says: one where
//! the bench times it), on the scene mod pile3d, which builds each scene
//! from the same code (`scenes.rs`) the other engines are given theirs by:
//! its statics at `build`, and each step's arrivals from a system in that
//! step, where a game's spawns would be. So its step is the engine's frame,
//! with what the ECS costs around the systems (the re-sorts at apply nodes,
//! the schedule) in it, as `//engine/std/physics2d:tax` measures the 2D mod.

use std::cell::RefCell;
use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use engine_ecs::Entity;
use engine_loader::engine::Engine;
use physics3d::{AngularVelocity, Manifold, Position, Rotation, Tuning, Velocity};

use crate::scenes::Scene;
use crate::{Backend, Config, Spec, State, Threads};

pub struct Ours {
    engine: Box<Engine>,
    dir: PathBuf,
    /// Dynamic bodies, in the order the scene adds them: looked up after a
    /// step, in `state`, not in the step the harness times.
    dynamic: RefCell<Vec<Entity>>,
    /// The last frame's wall time, in µs.
    total_us: f64,
    config: Config,
}

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f64 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

impl Ours {
    pub fn new(config: &Config) -> Ours {
        static RUN: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
        let run = RUN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let manifest = engine_control::read_manifest(env!("PILE3D")).unwrap();
        let dir = std::env::temp_dir().join(format!("physics3d-bench-{}-{run}", std::process::id()));
        let engine = Engine::new(manifest.bootstrap.clone(), dir.clone());
        // pile3d's game less its thread host, and the process's one pool in
        // its place: a harness makes thousands of engines, and each load of
        // the host costs a TLS key (`engine_threads::shared`).
        let mods: Vec<_> = manifest.mods.iter().filter(|(name, _)| name != engine_threads::MOD_NAME).cloned().collect();
        engine.load_batch(&mods).expect("loading pile3d");
        let executor: Option<std::sync::Arc<dyn engine_ecs::Executor>> = match config.threads {
            Threads::Shared => engine_threads::shared().map(|p| p as _),
            Threads::One => None,
            Threads::Pool(n) => Some(std::sync::Arc::new(engine_threads::Pool::new(
                &engine_threads::OneCcd,
                &engine_threads::Settings { threads: Some(n), ..Default::default() },
            ))),
        };
        engine.world().set_executor(executor);
        // A bounce's substeps, as a game sets them: in the world's Tuning.
        let tune = match (config.tune, config.substeps) {
            (t, 0) => t.to_string(),
            ("", n) => format!("sub={n}"),
            (t, n) => format!("{t},sub={n}"),
        };
        let ours =
            Ours { engine, dir, dynamic: RefCell::default(), total_us: 0.0, config: Config { tune: tune.clone().leak(), ..*config } };
        if !tune.is_empty() {
            ours.send("pile3d", &format!("tune {tune}"));
        }
        // One setting for iterations: the soft step has its own substeps,
        // whatever the harness asks of the others.
        if !config.rotate {
            ours.send("pile3d", "lock");
        }
        ours
    }

    /// How many threads the world's executor has: `None` for no executor,
    /// the calling thread alone.
    pub fn threads(&self) -> Option<usize> {
        self.engine.world().executor_threads()
    }

    fn send(&self, to: &str, message: &str) -> String {
        self.engine.send(to, message).unwrap_or_else(|e| panic!("{to} {message:?}: {e}"))
    }

    /// The bodies pile3d has added since the last look.
    fn arrived(&self) {
        let mut dynamic = self.dynamic.borrow_mut();
        let said = self.send("pile3d", &format!("bodies {}", dynamic.len()));
        dynamic.extend(said.split_whitespace().map(|e| {
            let (index, generation) = e.split_once(':').expect("index:generation");
            Entity { index: index.parse().unwrap(), generation: generation.parse().unwrap() }
        }));
    }
}

impl Drop for Ours {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

impl Backend for Ours {
    fn name(&self) -> String {
        match self.config.threads {
            Threads::Pool(n) => format!("ours, {n} threads"),
            Threads::Shared | Threads::One => "ours".into(),
        }
    }

    fn solver(&self) -> String {
        let t = Tuning::parse(self.config.tune).unwrap_or_else(|e| panic!("--tune: {e}"));
        let variant = if self.config.tune.is_empty() { String::new() } else { format!(", tuned {}", self.config.tune) };
        format!(
            "physics3d mod in the engine: soft step, {} substeps ({} Hz, 1 solve + {} relax each){variant}",
            t.substeps,
            t.stiffness * t.substeps as f32 * 60.0,
            t.relax
        )
    }

    /// Built by pile3d from the scene's kind and size, which make the same
    /// scene every time (checked: the scene it was given is the one it
    /// would build).
    fn builds(&mut self, scene: &Scene) -> bool {
        let same = crate::scenes::build(scene.kind, scene.n);
        assert!(same.statics.len() == scene.statics.len() && same.spawn.len() == scene.spawn.len(), "a scene pile3d can't build");
        self.send("pile3d", &format!("build {} {}", scene.kind.name(), scene.n));
        true
    }

    fn add(&mut self, _: &[Spec]) -> Vec<u32> {
        unreachable!("pile3d adds the bodies: see `load`")
    }

    /// One frame, and nothing else in the time the harness takes of it:
    /// the timings are read and reset after, in `stages`.
    fn step(&mut self, dt: f32) {
        // Lockstep steps at 1/60 s, the bench's step too.
        assert!((dt - crate::DT).abs() < 1e-9, "lockstep steps at 1/60 s");
        let t = Instant::now();
        self.send("lockstep", "step 1");
        self.total_us = t.elapsed().as_secs_f64() * 1e6;
    }

    fn state(&self, out: &mut Vec<State>) {
        self.arrived();
        let w = self.engine.world();
        let at: HashMap<Entity, Position> = w.values::<Position>().unwrap().into_iter().collect();
        let v: HashMap<Entity, Velocity> = w.values::<Velocity>().unwrap().into_iter().collect();
        let q: HashMap<Entity, Rotation> = w.values::<Rotation>().unwrap().into_iter().collect();
        let w: HashMap<Entity, AngularVelocity> = w.values::<AngularVelocity>().unwrap().into_iter().collect();
        out.clear();
        out.extend(self.dynamic.borrow().iter().map(|e| {
            let (p, v, q, w) = (at[e], v[e], q[e], w[e]);
            State { pos: [p.x, p.y, p.z], vel: [v.x, v.y, v.z], rot: [q.x, q.y, q.z, q.w], ang: [w.x, w.y, w.z] }
        }));
    }

    fn touching(&self) -> usize {
        // Speculative contacts (apart, negative depth) aren't touching.
        self.engine.world().values::<Manifold>().map_or(0, |m| m.iter().filter(|(_, m)| m.deepest() >= 0.0).count())
    }

    /// physics3d's own timings of the last step, which it keeps per stage;
    /// reset for the next.
    fn stages(&self) -> Vec<(&'static str, f64)> {
        let t = self.send("physics3d", "stages");
        self.send("physics3d", "reset_timings");
        let f = |k: &str| field(&t, k);
        let inside: f64 =
            ["gravity", "gather", "broadphase", "narrowphase", "merge", "solve_gather", "solver", "write_back"].iter().map(|k| f(k)).sum();
        vec![
            ("gather", f("gravity") + f("gather")),
            ("broadphase", f("broadphase")),
            ("narrowphase", f("narrowphase")),
            ("merge contacts", f("merge")),
            ("solve: copy in/out", f("solve_gather") + f("write_back")),
            ("solver", f("solver")),
            // The re-sorts at apply nodes, and the rest of the frame.
            ("outside systems", self.total_us - inside),
            ("pairs", f("pairs")),
            ("contacts", f("contacts")),
            ("points", f("points")),
            ("warm-started %", 100.0 * f("matched") / f("kept").max(1.0)),
        ]
    }
}
