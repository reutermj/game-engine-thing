//! The platformer's world. Units are tiles, with y growing downward, which is
//! how the level is written and how `platformer_text` draws it. A tile at
//! `(x, y)` covers `[x, x + 1) × [y, y + 1)`.

use engine_api::component;

/// Tiles per second squared.
pub const GRAVITY: f32 = 40.0;
/// Upward speed of a jump, in tiles per second: about four tiles high.
pub const JUMP_SPEED: f32 = 18.0;
pub const RUN_SPEED: f32 = 7.0;
pub const MAX_FALL: f32 = 30.0;
/// The player's box, anchored at its top-left corner.
pub const PLAYER_WIDTH: f32 = 0.8;
pub const PLAYER_HEIGHT: f32 = 0.95;

/// `Tile::kind` values.
pub const SOLID: u8 = 0;
pub const SPIKE: u8 = 1;
pub const GOAL: u8 = 2;

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Tile: "platformer::Tile" {
        pub x: i32,
        pub y: i32,
        pub kind: u8,
    }
}

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Coin: "platformer::Coin" {
        pub x: i32,
        pub y: i32,
    }
}

component! {
    /// The level's size and where the player starts. One per level.
    #[derive(Debug, Default, Copy)]
    pub struct LevelInfo: "platformer::LevelInfo" {
        pub width: i32,
        pub height: i32,
        pub spawn_x: f32,
        pub spawn_y: f32,
    }
}

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Player: "platformer::Player" {
        pub x: f32,
        pub y: f32,
        pub vx: f32,
        pub vy: f32,
        pub on_ground: bool,
        pub coins: u32,
        pub deaths: u32,
        /// Set once the player touches the goal.
        pub won: bool,
        /// Set by anything that kills the player (an enemy); the rules
        /// respawn the player on the next frame.
        pub hurt: bool,
    }
}

component! {
    /// What the player is asking for, on the player's entity. Set by whoever
    /// controls the player; read by the rules each frame.
    #[derive(Debug, Default, Copy)]
    pub struct Input: "platformer::Input" {
        /// -1 runs left, 1 right, 0 stands. Held until changed.
        pub dir: f32,
        /// A jump request, consumed by the next frame: it jumps if the player
        /// is on the ground then, and is dropped otherwise.
        pub jump: bool,
    }
}
