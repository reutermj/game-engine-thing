//! Pong's text interface, over messages: `modctl send pong_text <command>`.
//! It runs nothing on its own; with the lockstep bootstrap, time only moves
//! with `modctl send lockstep step <frames>`.
//!
//!   up | down | stay   set the left paddle moving (it keeps moving until changed)
//!   show               draw the court
//!   state              the numbers behind the drawing

use clock::Clock;
use engine_api::{Cx, Mod, World, export_mod};
use pong::{Ball, HEIGHT, PADDLE_HEIGHT, Paddle, Player, Score, WIDTH};

const HELP: &str = "commands: up | down | stay | show | state";

engine_api::mod_state! {
    #[derive(Default)]
    struct Text {}
}

fn set_intent(world: &mut World, intent: f32) -> Result<(), String> {
    let mut found = false;
    for (_, _, paddle) in world.query2::<Player, Paddle>() {
        paddle.intent = intent;
        found = true;
    }
    if found { Ok(()) } else { Err("no player paddle: is pong loaded?".into()) }
}

struct Snapshot {
    frame: u64,
    ball: Ball,
    paddles: Vec<Paddle>,
    score: Score,
}

fn snapshot(world: &mut World) -> Result<Snapshot, String> {
    let frame = clock::now(world).map_or(0, |c: Clock| c.frame);
    let ball = world.query::<Ball>().next().map(|(_, b)| *b).ok_or("no ball: is pong loaded?")?;
    let paddles = world.query::<Paddle>().map(|(_, p)| *p).collect();
    let score = world.query::<Score>().next().map(|(_, s)| *s).unwrap_or_default();
    Ok(Snapshot { frame, ball, paddles, score })
}

fn draw(s: &Snapshot) -> String {
    let (w, h) = (WIDTH as usize, HEIGHT as usize);
    let mut grid = vec![vec![' '; w]; h];
    for p in &s.paddles {
        // Drawn in the column in front of the face, where the ball bounces.
        let col = if p.face < WIDTH / 2.0 { p.face - 1.0 } else { p.face };
        for (row, line) in grid.iter_mut().enumerate() {
            if ((row as f32 + 0.5) - p.y).abs() < PADDLE_HEIGHT / 2.0 {
                line[col as usize] = '#';
            }
        }
    }
    let (bx, by) = (s.ball.x.clamp(0.0, WIDTH - 1.0) as usize, s.ball.y.clamp(0.0, HEIGHT - 1.0) as usize);
    grid[by][bx] = 'o';

    let border = format!("+{}+", "-".repeat(w));
    let mut out = format!("frame {}   you {} : {} ai\n{border}\n", s.frame, s.score.left, s.score.right);
    for line in grid {
        out += &format!("|{}|\n", line.into_iter().collect::<String>());
    }
    out + &border
}

fn describe(s: &Snapshot) -> String {
    let mut out = format!(
        "frame {}\nball x {:.2} y {:.2} vx {:.2} vy {:.2}\n",
        s.frame, s.ball.x, s.ball.y, s.ball.vx, s.ball.vy
    );
    for p in &s.paddles {
        let side = if p.face < WIDTH / 2.0 { "you" } else { "ai" };
        out += &format!("{side} paddle face {:.0} y {:.2} intent {:.1}\n", p.face, p.y, p.intent);
    }
    out + &format!("score you {} ai {}", s.score.left, s.score.right)
}

impl Mod for Text {
    type Transient = ();

    fn step(&mut self, _: &mut (), _cx: &mut Cx) -> engine_api::Status {
        engine_api::Status::OK
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut world = cx.world();
        match message.trim() {
            "up" => set_intent(&mut world, -1.0).map(|()| "moving up".into()),
            "down" => set_intent(&mut world, 1.0).map(|()| "moving down".into()),
            "stay" => set_intent(&mut world, 0.0).map(|()| "staying".into()),
            "show" => snapshot(&mut world).map(|s| draw(&s)),
            "state" => snapshot(&mut world).map(|s| describe(&s)),
            "" | "help" => Ok(HELP.into()),
            other => Err(format!("unknown command {other:?}; {HELP}")),
        }
    }
}

export_mod!(Text);
