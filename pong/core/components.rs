//! Pong's world. Units are text cells: the court is `WIDTH` columns by
//! `HEIGHT` rows, with y growing downward, which is how `pong_text` draws it.

use engine_api::{component, event};

pub const WIDTH: f32 = 40.0;
pub const HEIGHT: f32 = 20.0;
pub const PADDLE_HEIGHT: f32 = 4.0;
/// Columns of the paddles' faces: the ball bounces when it reaches them.
pub const LEFT_FACE: f32 = 2.0;
pub const RIGHT_FACE: f32 = WIDTH - 2.0;
/// Cells per second.
pub const PADDLE_SPEED: f32 = 16.0;
pub const SERVE_SPEED: f32 = 16.0;

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Ball: "pong::Ball" {
        pub x: f32,
        pub y: f32,
        pub vx: f32,
        pub vy: f32,
    }
}

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Paddle: "pong::Paddle" {
        /// The paddle's face column (`LEFT_FACE` or `RIGHT_FACE`).
        pub face: f32,
        /// The paddle's center.
        pub y: f32,
        /// -1 moves up at full speed, 1 down, 0 stays. Set by whoever
        /// controls the paddle.
        pub intent: f32,
    }
}

component! {
    /// Marks the paddle `pong_text` controls: the left one.
    #[derive(Debug, Default, Copy)]
    pub struct Player: "pong::Player" {}
}

component! {
    /// Marks the paddle `pong_ai` controls: the right one.
    #[derive(Debug, Default, Copy)]
    pub struct Opponent: "pong::Opponent" {}
}

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Score: "pong::Score" {
        pub left: u32,
        pub right: u32,
        /// Serves so far, which picks each serve's angle.
        pub serves: u32,
    }
}

event! {
    /// Sets the player's paddle moving: -1 up, 1 down, 0 still. It keeps
    /// moving until the next one. Sent by `pong_text`, applied in the
    /// `input` phase.
    #[derive(Debug, Default, Copy)]
    pub struct Steer: "pong::Steer" {
        pub intent: f32,
    }
}
