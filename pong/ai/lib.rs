//! Plays the right paddle: follows the ball while it's coming, and drifts back
//! to the middle while it's going away. Slower than the ball can get, so it
//! can be beaten with angled hits.

use engine_api::{Cx, Mod, Query, Systems, export_mod};
use pong::{Ball, HEIGHT, Opponent, Paddle};

/// Fraction of full paddle speed the AI uses.
const EFFORT: f32 = 0.8;
/// How far off the ball it tolerates before moving, in cells.
const SLACK: f32 = 0.5;

engine_api::mod_state! {
    #[derive(Default)]
    struct Ai {}
}

impl Ai {
    fn think(&mut self, _: &mut (), _: &mut Cx, mut balls: Query<&Ball>, mut paddles: Query<(&Opponent, &mut Paddle)>) {
        let Some(ball) = balls.single(|_, b| *b) else { return };
        let target = if ball.vx > 0.0 { ball.y } else { HEIGHT / 2.0 };
        paddles.for_each(|_, (_, paddle)| {
            let off = target - paddle.y;
            paddle.intent = if off.abs() < SLACK { 0.0 } else { off.signum() * EFFORT };
        });
    }
}

impl Mod for Ai {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        // In `update`, so the paddle moves on this frame's decision in
        // `simulate`, not the last frame's.
        s.add("think", Self::think);
    }
}

export_mod!(Ai);
