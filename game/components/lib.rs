//! The game's components. Every mod that uses one links this crate, and the
//! world matches them by name, so changing a type here and reloading one mod
//! migrates that component's values for all of them (see
//! docs/architecture/ecs.md).

use engine_api::component;

component! {
    #[derive(Debug, Default)]
    pub struct Position: "game::Position" {
        pub x: f32,
        pub y: f32,
    }
}

component! {
    /// Units per second.
    #[derive(Debug, Default)]
    pub struct Velocity: "game::Velocity" {
        pub x: f32,
        pub y: f32,
    }
}
