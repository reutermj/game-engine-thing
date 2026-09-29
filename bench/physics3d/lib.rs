//! A comparison harness for 3D rigid bodies: the same scenes and the same
//! quality measures over several engines, each single-threaded, with
//! rotations locked (as the translation-only step had them) or free, so a new
//! solver can be judged against established ones on identical input.
//!
//! An engine plugs in as a [`Backend`] and is named in [`BACKENDS`] and
//! [`make_backend`]; everything else (the scenes, the timing, the metrics)
//! reads only what the trait returns, so every engine is measured by the same
//! code. The world is fixed across engines: y up, gravity (0, -9.81, 0), a
//! 1/60 s step, mass 1, friction 0.5, restitution 0.

// The baseline's format and comparison, 2D's.
#[path = "../../engine/std/physics/compare/baseline.rs"]
pub mod baseline;
pub mod behave;
pub mod bounces;
pub mod box3d;
mod ffi;
pub mod jolt;
pub mod measure;
pub mod ours;
pub mod rapier;
pub mod record;
pub mod runs;
// The scene mod builds the same scenes in the engine, from this file.
#[path = "../../engine/std/physics3d/tests/scenes.rs"]
pub mod scenes;

pub use scenes::{Shape, Spec};

pub const GRAVITY: [f32; 3] = [0.0, -9.81, 0.0];
pub const DT: f32 = 1.0 / 60.0;
pub const FRICTION: f32 = 0.5;
pub const RESTITUTION: f32 = 0.0;
/// Every dynamic body's mass, whatever its size.
pub const MASS: f32 = 1.0;

/// How hard each engine's solver works.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Iters {
    /// Each engine's own defaults (see [`Backend::solver`]).
    Default,
    /// 8 wherever the engine has a single knob for it: Rapier's
    /// num_solver_iterations, Jolt's velocity and position steps, Box3D's
    /// substeps. Not the same work in each: see the report.
    Eight,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub iters: Iters,
    /// Off by default: sleeping would make "settled" measure how soon an engine
    /// stops simulating instead of what simulating costs.
    pub sleep: bool,
    /// An upper bound on bodies in the world, for engines that preallocate.
    pub max_bodies: u32,
    /// Bodies turn; off, every engine locks their rotation.
    pub rotate: bool,
    /// A variant of ours (`physics3d::Tuning::parse`), or "" for the
    /// defaults.
    pub tune: &'static str,
    /// y is up: `scenes::EARTH` but for a bounce (`Scene::gravity`).
    pub gravity: [f32; 3],
    /// The substeps a bounce sets in every engine that has them (ours,
    /// Rapier's solver iterations, Box3D's), or 0 for each engine's own.
    pub substeps: u32,
}

impl Config {
    /// Each engine's defaults on `scene`, as `runs` makes them: its gravity
    /// and substeps, turning or not, and ours tuned as `tune` says.
    pub fn of(scene: &scenes::Scene, rotate: bool, tune: &'static str) -> Config {
        let max_bodies = (scene.spawn.iter().map(Vec::len).sum::<usize>() + scene.statics.len() + 16) as u32;
        Config { iters: Iters::Default, sleep: false, max_bodies, rotate, tune, gravity: scene.gravity, substeps: scene.substeps }
    }
}

/// A dynamic body as the harness reads it back.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    pub pos: [f32; 3],
    pub vel: [f32; 3],
    /// A unit quaternion, (x, y, z, w).
    pub rot: [f32; 4],
    pub ang: [f32; 3],
}

/// One engine behind the harness.
pub trait Backend {
    fn name(&self) -> String;
    /// Builds the scene itself, if it can, and says so: then the harness
    /// adds nothing. Ours does, since a game spawns its own bodies.
    fn builds(&mut self, _: &scenes::Scene) -> bool {
        false
    }
    /// The solver settings in effect, for the report.
    fn solver(&self) -> String;
    /// Adds bodies, callable at any point in a run; returns one handle per
    /// spec. The harness only relies on the add order, not on the handles.
    fn add(&mut self, bodies: &[Spec]) -> Vec<u32>;
    fn step(&mut self, dt: f32);
    /// Overwrites `out` with the state of every dynamic body, in the order
    /// they were added.
    fn state(&self, out: &mut Vec<State>);
    /// The engine's own count of body pairs in contact after the last step.
    fn touching(&self) -> usize;
    /// Per-stage times of the last step in microseconds, where the engine
    /// exposes them; empty otherwise.
    fn stages(&self) -> Vec<(&'static str, f64)>;
}

pub const BACKENDS: &[&str] = &["ours", "rapier", "jolt", "box3d"];

pub fn make_backend(name: &str, config: &Config) -> Option<Box<dyn Backend>> {
    Some(match name {
        "ours" => Box::new(ours::Ours::new(config)),
        "rapier" => Box::new(rapier::Rapier::new(config)),
        "jolt" => Box::new(jolt::Jolt::new(config)),
        "box3d" => Box::new(box3d::Box3d::new(config)),
        _ => return None,
    })
}
