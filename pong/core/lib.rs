//! Pong's rules, over physics: `steer` applies the player's input (in
//! `input`); `play` moves the paddles by their intent (in `simulate`);
//! physics bounces the ball off walls and paddles; and `rebound` (in
//! `late`) adds pong's own rules to what physics reported: spin and speed
//! on a paddle hit, and a point and a serve when the ball reaches a goal
//! line.
//!
//! Setup is keyed on the world, not on this mod's state: if there's no ball,
//! the court is set up. A reload, or a state reset, then never sets up a second
//! court.

use engine_api::{Cx, Dt, Entity, EventReader, Mod, Query, Systems, With, Without, WorldMut, export_mod, phase};
use physics::{Body, Collider, Contact, Position, Trigger, Velocity};
use pong::{
    BALL_RADIUS, Ball, Goal, HEIGHT, LEFT_FACE, Opponent, PADDLE_HEIGHT, PADDLE_SPEED, Paddle, Player, RIGHT_FACE,
    SERVE_SPEED, Score, Steer, WIDTH,
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

/// A box by its edges.
fn wall(world: &mut WorldMut, left: f32, top: f32, right: f32, bottom: f32, collider: Collider) -> Entity {
    let at = Position { x: (left + right) / 2.0, y: (top + bottom) / 2.0 };
    world.spawn((at, Collider { hx: (right - left) / 2.0, hy: (bottom - top) / 2.0, ..collider }))
}

impl Core {
    fn set_up(&mut self, world: &mut WorldMut) {
        if world.single::<&Ball, ()>(|_, _| ()).is_some() {
            return;
        }
        let score = Score::default();
        let ball = Body { restitution: 1.0, friction: 0.0, gravity_scale: 0.0, ..Body::default() };
        // The first serve goes to the player, on the left.
        let (at, v) = serve(&score, -1.0);
        world.spawn((Ball {}, at, v, ball, Collider::circle(BALL_RADIUS)));
        world.spawn((score,));
        let r = BALL_RADIUS;
        // Top and bottom, and the goal lines behind the paddles, far enough
        // out that nothing gets around them.
        wall(world, -10.0, -10.0, WIDTH + 10.0, -r, Collider::default());
        wall(world, -10.0, HEIGHT + r, WIDTH + 10.0, HEIGHT + 10.0, Collider::default());
        let goal = Collider::default().sensor();
        let left = wall(world, -10.0, -10.0, -r, HEIGHT + 10.0, goal);
        world.insert(left, Goal { side: -1.0 });
        let right = wall(world, WIDTH + r, -10.0, WIDTH + 10.0, HEIGHT + 10.0, goal);
        world.insert(right, Goal { side: 1.0 });
        // One cell deep, and a radius longer at each end than they're drawn,
        // so the ball's center reaches as far as the drawing does.
        let paddle = |face: f32| {
            let x = if face < WIDTH / 2.0 { face - r - 0.5 } else { face + r + 0.5 };
            let collider = Collider::rect(0.5, PADDLE_HEIGHT / 2.0 + r);
            (Paddle { face, intent: 0.0 }, Position { x, y: HEIGHT / 2.0 }, Velocity::default(), Body::kinematic(), collider)
        };
        let (p, at, v, b, c) = paddle(LEFT_FACE);
        world.spawn((p, at, v, b, c, Player {}));
        let (p, at, v, b, c) = paddle(RIGHT_FACE);
        world.spawn((p, at, v, b, c, Opponent {}));
    }
}

/// A ball at the center, heading toward `direction` (-1 left, 1 right).
fn serve(score: &Score, direction: f32) -> (Position, Velocity) {
    let angle = SERVE_ANGLES[score.serves as usize % SERVE_ANGLES.len()];
    (Position { x: WIDTH / 2.0, y: HEIGHT / 2.0 }, Velocity { x: direction * SERVE_SPEED, y: angle * SERVE_SPEED })
}

impl Core {
    fn steer(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut steers: EventReader<Steer>,
        mut players: Query<&mut Paddle, With<Player>>,
    ) {
        let Some(intent) = steers.read().last().map(|s| s.intent) else { return };
        players.for_each(|_, mut paddle| paddle.intent = intent);
    }

    /// Paddles move by their intent, and stop at the court's edges: a
    /// kinematic body goes where its velocity takes it, walls or not.
    fn play(&mut self, _: &mut (), _: &mut Cx, dt: Dt, mut paddles: Query<(&Paddle, &Position, &mut Velocity)>) {
        let dt = *dt;
        let half = PADDLE_HEIGHT / 2.0;
        paddles.for_each(|_, (paddle, p, mut v)| {
            let to = (p.y + paddle.intent.clamp(-1.0, 1.0) * PADDLE_SPEED * dt).clamp(half, HEIGHT - half);
            v.y = (to - p.y) / dt;
        });
    }

    /// Pong's rules on top of physics's bounce: a paddle hit speeds the ball
    /// up and spins it by how far off center it struck, and a ball at a goal
    /// line is a point, and a serve to whoever lost it.
    fn rebound(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut contacts: EventReader<Contact>,
        mut triggers: EventReader<Trigger>,
        // Not the ball, which `balls` moves.
        mut paddles: Query<&Position, (With<Paddle>, Without<Ball>)>,
        mut balls: Query<(&mut Position, &mut Velocity), With<Ball>>,
        mut scores: Query<&mut Score>,
        mut goals: Query<&Goal>,
    ) {
        for c in contacts.read() {
            // Physics sends each pair in entity order: either may be the ball.
            for (ball, other) in [(c.a, c.b), (c.b, c.a)] {
                let Some(paddle_y) = paddles.with(other, |_, p| p.y) else { continue };
                balls.with(ball, |_, (p, mut v)| {
                    v.x = (v.x * SPEEDUP).clamp(-MAX_SPEED, MAX_SPEED);
                    v.y = (v.y + (p.y - paddle_y) * SPIN).clamp(-MAX_SPEED, MAX_SPEED);
                });
            }
        }
        for t in triggers.read() {
            let Some(lost_by) = goals.with(t.sensor, |_, g| g.side) else { continue };
            if balls.with(t.other, |_, _| ()).is_none() {
                continue;
            }
            scores.for_each(|_, mut score| {
                if lost_by < 0.0 {
                    score.right += 1;
                } else {
                    score.left += 1;
                }
                score.serves += 1;
                let score = *score;
                balls.with(t.other, |_, (mut p, mut v)| (*p, *v) = serve(&score, lost_by));
            });
        }
    }
}

impl Mod for Core {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("steer", Self::steer).phase(phase::INPUT);
        s.add("play", Self::play).phase(phase::SIMULATE);
        s.add("rebound", Self::rebound).phase(phase::LATE);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        self.set_up(&mut cx.world());
    }
}

export_mod!(Core);
