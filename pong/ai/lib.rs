//! Plays the right paddle: follows the ball while it's coming, and drifts back
//! to the middle while it's going away. Slower than the ball can get, so it
//! can be beaten with angled hits.

use engine_api::{Cx, Mod, Status, export_mod};
use pong::{Ball, HEIGHT, Opponent, Paddle};

/// Fraction of full paddle speed the AI uses.
const EFFORT: f32 = 0.8;
/// How far off the ball it tolerates before moving, in cells.
const SLACK: f32 = 0.5;

engine_api::mod_state! {
    #[derive(Default)]
    struct Ai {}
}

impl Mod for Ai {
    type Transient = ();

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        let mut world = cx.world();
        let Some(ball) = world.query::<Ball>().next().map(|(_, b)| *b) else {
            return Status::OK;
        };
        let target = if ball.vx > 0.0 { ball.y } else { HEIGHT / 2.0 };
        for (_, _, paddle) in world.query2::<Opponent, Paddle>() {
            let off = target - paddle.y;
            paddle.intent = if off.abs() < SLACK { 0.0 } else { off.signum() * EFFORT };
        }
        Status::OK
    }
}

export_mod!(Ai);
