//! The pile3d game in an engine, for tests: its messages, steps, and the
//! world's values by entity.

use std::path::PathBuf;

use engine_ecs::{Component, Entity};
use engine_loader::engine::Engine;
use physics3d::{AngularVelocity, Position, Quat, Rotation, Vec3, Velocity};

pub struct Sim {
    pub engine: Box<Engine>,
}

fn v3(v: Vec3) -> String {
    format!("{} {} {}", v.x, v.y, v.z)
}

impl Sim {
    /// The game loaded, in a staging directory of the test's own.
    pub fn new(test: &str) -> Sim {
        let manifest = engine_control::read_manifest(&std::env::var("PILE3D").unwrap()).unwrap();
        let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(test);
        let engine = Engine::new(manifest.bootstrap.clone(), dir);
        engine.load_batch(&manifest.mods).expect("loading the pile");
        Sim { engine }
    }

    pub fn send(&self, message: &str) -> String {
        self.engine.send("pile3d", message).unwrap_or_else(|err| panic!("pile3d {message:?}: {err}"))
    }

    pub fn run(&self, steps: u32) {
        self.engine.send("lockstep", &format!("step {steps}")).unwrap();
    }

    /// A body, `shape` as pile3d's `body` takes it (`box 0.5 0.5 0.5`),
    /// turned by `turn` (an axis and an angle), moving at `v` and `w`.
    pub fn body(&self, shape: &str, at: Vec3, turn: (Vec3, f32), v: Vec3, w: Vec3) -> Entity {
        let (axis, angle) = turn;
        // No turn at all for none: an axis of zero has no direction.
        let turn = if angle == 0.0 { String::new() } else { format!(" turn {} {angle}", v3(axis)) };
        let said = self.send(&format!("body {shape} at {}{turn} v {} w {}", v3(at), v3(v), v3(w)));
        let (index, generation) = said.strip_prefix("body ").and_then(|e| e.split_once(':')).expect("body <index>:<generation>");
        Entity { index: index.parse().unwrap(), generation: generation.parse().unwrap() }
    }

    pub fn value<T: Component + Clone>(&self, e: Entity) -> T {
        let all = self.engine.world().values::<T>().unwrap_or_default();
        all.into_iter().find(|(x, _)| *x == e).unwrap_or_else(|| panic!("{e:?} has no {}", T::NAME)).1
    }

    pub fn at(&self, e: Entity) -> Vec3 {
        self.value::<Position>(e).at()
    }

    pub fn rot(&self, e: Entity) -> Quat {
        self.value::<Rotation>(e).quat()
    }

    pub fn v(&self, e: Entity) -> Vec3 {
        let v: Velocity = self.value(e);
        Vec3::new(v.x, v.y, v.z)
    }

    pub fn w(&self, e: Entity) -> Vec3 {
        let w: AngularVelocity = self.value(e);
        Vec3::new(w.x, w.y, w.z)
    }

    /// How nearly one of the body's axes points up: 1 when it lies on a face.
    pub fn flat(&self, e: Entity) -> f32 {
        self.rot(e).matrix().cols.iter().map(|c| c.y.abs()).fold(0.0, f32::max)
    }
}
