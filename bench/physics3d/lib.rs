//! A comparison harness for translation-only 3D rigid bodies: the same scenes
//! and the same quality measures over several engines, each single-threaded
//! with rotations locked, so a new solver can be judged against established
//! ones on identical input.
//!
//! An engine plugs in as a [`Backend`] and is named in [`BACKENDS`] and
//! [`make_backend`]; everything else (the scenes, the timing, the metrics)
//! reads only what the trait returns, so every engine is measured by the same
//! code. The world is fixed across engines: y up, gravity (0, -9.81, 0), a
//! 1/60 s step, mass 1, friction 0.5, restitution 0.

pub mod box3d;
mod ffi;
pub mod jolt;
pub mod measure;
pub mod ours;
pub mod rapier;
pub mod scenes;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Sphere(f32),
    /// Half-extents, axis-aligned for good since rotation is locked.
    Box([f32; 3]),
}

impl Shape {
    /// The radius of a sphere around the shape: what the harness's pair grid
    /// is sized by.
    pub fn bounding_radius(&self) -> f32 {
        match *self {
            Shape::Sphere(r) => r,
            Shape::Box([x, y, z]) => (x * x + y * y + z * z).sqrt(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Spec {
    pub shape: Shape,
    pub pos: [f32; 3],
    /// Ignored for a fixed body.
    pub vel: [f32; 3],
    pub fixed: bool,
}

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
}

/// One engine behind the harness.
pub trait Backend {
    fn name(&self) -> String;
    /// The solver settings in effect, for the report.
    fn solver(&self) -> String;
    /// Adds bodies, callable at any point in a run; returns one handle per
    /// spec. The harness only relies on the add order, not on the handles.
    fn add(&mut self, bodies: &[Spec]) -> Vec<u32>;
    fn step(&mut self, dt: f32);
    /// Overwrites `out` with (position, velocity) of every dynamic body, in the
    /// order they were added.
    fn state(&self, out: &mut Vec<([f32; 3], [f32; 3])>);
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
