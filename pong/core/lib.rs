//! Pong's rules as a system: moves the paddles by their intent, moves and
//! bounces the ball, and scores.
//!
//! Setup is keyed on the world, not on this mod's state: if there's no ball,
//! the court is set up. A reload, or a state reset, then never sets up a second
//! court.

use clock::Clock;
use engine_api::{Cx, Mod, Status, World, export_mod};
use pong::{
    Ball, HEIGHT, LEFT_FACE, Opponent, PADDLE_HEIGHT, PADDLE_SPEED, Paddle, Player, RIGHT_FACE,
    SERVE_SPEED, Score, WIDTH,
};

/// Vertical speed of each serve, as a fraction of `SERVE_SPEED`, in turn.
/// Fixed rather than random so a game is reproducible from its inputs.
const SERVE_ANGLES: [f32; 5] = [0.35, -0.6, 0.15, 0.8, -0.3];
/// How much a hit off-center adds to vertical speed, per cell off center.
const SPIN: f32 = 3.0;
/// Speed gained on every paddle hit, capped.
const SPEEDUP: f32 = 1.05;
const MAX_SPEED: f32 = 40.0;

engine_api::mod_state! {
    #[derive(Default)]
    struct Core {}
}

fn set_up(world: &mut World) {
    if world.query::<Ball>().next().is_some() {
        return;
    }
    let score = Score::default();
    let ball = world.spawn();
    // The first serve goes to the player, on the left.
    world.insert(ball, serve(&score, -1.0));
    let e = world.spawn();
    world.insert(e, score);
    for (face, player) in [(LEFT_FACE, true), (RIGHT_FACE, false)] {
        let e = world.spawn();
        world.insert(e, Paddle { face, y: HEIGHT / 2.0, intent: 0.0 });
        if player {
            world.insert(e, Player {});
        } else {
            world.insert(e, Opponent {});
        }
    }
}

/// A ball at the center, heading toward `direction` (-1 left, 1 right).
fn serve(score: &Score, direction: f32) -> Ball {
    let angle = SERVE_ANGLES[score.serves as usize % SERVE_ANGLES.len()];
    Ball { x: WIDTH / 2.0, y: HEIGHT / 2.0, vx: direction * SERVE_SPEED, vy: angle * SERVE_SPEED }
}

/// Bounces the ball off `paddle` if it just crossed the paddle's face within
/// reach of it.
fn hit(ball: &mut Ball, before_x: f32, paddle: &Paddle) {
    let face = paddle.face;
    let crossed = (before_x - face) * (ball.x - face) <= 0.0 && before_x != face;
    let toward = if face < WIDTH / 2.0 { ball.vx < 0.0 } else { ball.vx > 0.0 };
    let reach = PADDLE_HEIGHT / 2.0 + 0.5;
    if crossed && toward && (ball.y - paddle.y).abs() <= reach {
        ball.x = 2.0 * face - ball.x;
        ball.vx = (-ball.vx * SPEEDUP).clamp(-MAX_SPEED, MAX_SPEED);
        ball.vy = (ball.vy + (ball.y - paddle.y) * SPIN).clamp(-MAX_SPEED, MAX_SPEED);
    }
}

impl Mod for Core {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        set_up(&mut cx.world());
    }

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        let mut world = cx.world();
        let Some(Clock { dt, .. }) = clock::now(&mut world) else {
            return Status::OK;
        };

        let mut paddles = Vec::new();
        for (_, paddle) in world.query::<Paddle>() {
            let half = PADDLE_HEIGHT / 2.0;
            paddle.y = (paddle.y + paddle.intent.clamp(-1.0, 1.0) * PADDLE_SPEED * dt).clamp(half, HEIGHT - half);
            paddles.push(*paddle);
        }
        let Some((score_entity, mut score)) = world.query::<Score>().next().map(|(e, s)| (e, *s)) else {
            return Status::OK;
        };

        let mut scored = false;
        for (_, ball) in world.query::<Ball>() {
            let before_x = ball.x;
            ball.x += ball.vx * dt;
            ball.y += ball.vy * dt;
            if ball.y < 0.0 {
                ball.y = -ball.y;
                ball.vy = -ball.vy;
            } else if ball.y > HEIGHT {
                ball.y = 2.0 * HEIGHT - ball.y;
                ball.vy = -ball.vy;
            }
            for paddle in &paddles {
                hit(ball, before_x, paddle);
            }
            // Past a paddle and out: a point, and a serve to whoever lost it.
            let lost_by = if ball.x < 0.0 {
                score.right += 1;
                -1.0
            } else if ball.x > WIDTH {
                score.left += 1;
                1.0
            } else {
                continue;
            };
            score.serves += 1;
            *ball = serve(&score, lost_by);
            scored = true;
        }
        if scored {
            world.insert(score_entity, score);
        }
        Status::OK
    }
}

export_mod!(Core);

