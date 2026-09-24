//! The walker enemy: a one-tile box that paces back and forth.

use engine_api::component;

/// Tiles per second.
pub const WALK_SPEED: f32 = 3.0;
/// Upward speed the player bounces off a stomped walker with.
pub const STOMP_BOUNCE: f32 = 12.0;

component! {
    /// Marks a walker. Its one-tile box is physics's.
    #[derive(Debug, Default, Copy)]
    pub struct Walker: "walkers::Walker" {}
}
