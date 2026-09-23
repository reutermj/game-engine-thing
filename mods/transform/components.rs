//! Where things are. Changing a type here changes the interface of
//! `transform`, so every mod that depends on it has to be reloaded with it:
//! `./bazel run //game:reload`.

use engine_api::component;

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Position: "transform::Position" {
        pub x: f32,
        pub y: f32,
    }
}
