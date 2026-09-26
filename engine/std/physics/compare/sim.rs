//! What every engine is read through: its dynamic bodies, and a step.
//! Shared by the comparison and the quality tests (`quality_test.rs`).

use crate::scene;

/// A dynamic body as every engine reports it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dyn {
    pub circle: bool,
    pub hx: f32,
    pub hy: f32,
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    /// Radians: 0 always where rotation is locked, or a body turned that
    /// should have been locked.
    pub angle: f32,
    /// Radians a second.
    pub w: f32,
}

impl Dyn {
    /// How fast it moves: its center, or its edge if that's faster, as
    /// sleeping goes by in ours and Box2D.
    pub fn speed(&self) -> f32 {
        let reach = if self.circle { self.hx } else { self.hx.hypot(self.hy) };
        (self.vx * self.vx + self.vy * self.vy).sqrt().max(self.w.abs() * reach)
    }
}

/// One engine running one scene.
pub trait Sim {
    fn label(&self) -> String;
    fn step(&mut self, n: u32);
    /// Adds a rain step's arrivals and removes the oldest past `alive`.
    fn rain(&mut self, arrivals: &[scene::Spec], alive: usize);
    /// The dynamic bodies, in the order they came.
    fn bodies(&self) -> Vec<Dyn>;
    fn reset(&mut self);
    /// µs spent since `reset`: `broadphase`, `narrowphase` and `solver`
    /// first, as each engine's stages best map to them, then its own
    /// stages under its prefix, and `engine step`, the step as timed.
    fn stages(&self) -> Vec<(String, f64)>;
    /// Contacts the engine is solving that touch.
    fn contacts(&self) -> usize;
    /// Its own counts, for the notes.
    fn native(&self) -> String;
}
