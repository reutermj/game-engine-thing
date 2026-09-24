//! Plays the right paddle: follows the ball while it's coming, and drifts back
//! to the middle while it's going away. Slower than the ball can get, so it
//! can be beaten with angled hits.

use engine_api::{Cx, Mod, Query, Systems, With, export_mod};
use physics::{Position, Velocity};
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
    fn think(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut balls: Query<(&Position, &Velocity), With<Ball>>,
        mut paddles: Query<(&mut Paddle, &Position), With<Opponent>>,
    ) {
        let Some((y, vx)) = balls.single(|_, (p, v)| (p.y, v.x)) else { return };
        let target = if vx > 0.0 { y } else { HEIGHT / 2.0 };
        paddles.for_each(|_, (mut paddle, p)| {
            let off = target - p.y;
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
