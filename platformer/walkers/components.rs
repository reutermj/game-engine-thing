//! The walker enemy: a one-tile box that paces back and forth.

use engine_api::component;

/// Tiles per second.
pub const WALK_SPEED: f32 = 3.0;
/// Upward speed the player bounces off a stomped walker with.
pub const STOMP_BOUNCE: f32 = 12.0;

component! {
    #[derive(Debug, Default)]
    pub struct Walker: "walkers::Walker" {
        /// Top-left corner of its one-tile box.
        pub x: f32,
        pub y: f32,
        pub vx: f32,
    }
}
