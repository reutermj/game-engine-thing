//! The components `physics` owns and other mods may use.

use engine_api::component;

component! {
    /// Units per second.
    #[derive(Debug, Default)]
    pub struct Velocity: "physics::Velocity" {
        pub x: f32,
        pub y: f32,
    }
}
