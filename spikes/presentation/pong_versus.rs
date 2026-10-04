//! SPIKE (get-3hd.1): `pong_versus`, pong for two agents taking turns
//! (AGENT_VERSUS.md). The windowed lockstep bootstrap keeps the turn
//! barrier and publishes a `spike_turns::Turn`; this mod is the game's side
//! of it:
//!
//! - `steer` (in `update`) sets both paddles' `intent` from the turn's
//!   actions every frame, as `pong_ai` sets the right one: so the game runs
//!   without `pong_ai`, and pong's own code doesn't change. The left
//!   paddle could have gone through `pong::Steer` as `pong_text`'s does,
//!   but there's no such event for the right one, and one path for both
//!   sides keeps them symmetric: an action takes effect on the turn's first
//!   frame either way.
//! - `decide` (in `late`, after pong's `rebound` scores) spawns an
//!   `Outcome` when a side reaches the point limit, which the bootstrap
//!   stops on.
//! - Messages `state` and `show`, which name the sides `left` and `right`
//!   so both players read the same text; `first-to <n>|off`.

use clock::Clock;
use engine_api::{Cx, Mod, Query, Spawner, Systems, WorldMut, export_mod, phase};
use physics2d::{Position, Velocity};
use pong::{Ball, HEIGHT, PADDLE_HEIGHT, Paddle, Score, WIDTH};
use spike_turns::{Outcome, SIDES, Turn, TurnInfo};

/// Points that win, unless `first-to` says otherwise.
const FIRST_TO: u32 = 5;
const HELP: &str = "commands: state | show | first-to <points>|off (submit moves to lockstep: turn <left|right> <up|down|stay>)";

engine_api::mod_state! {
    #[derive(Default)]
    struct Versus {
        /// Points that win; 0 means `FIRST_TO`.
        first_to: u32,
        /// No point limit.
        endless: bool,
        /// The `Outcome` is spawned: once is enough.
        decided: bool,
    }
}

impl Versus {
    fn limit(&self) -> Option<u32> {
        match (self.endless, self.first_to) {
            (true, _) => None,
            (false, 0) => Some(FIRST_TO),
            (false, n) => Some(n),
        }
    }

    fn steer(&mut self, _: &mut (), _: &mut Cx, mut turns: Query<&Turn>, mut paddles: Query<&mut Paddle>) {
        let Some(intents) = turns.single(|_, t| t.now.intents) else { return };
        paddles.for_each(|_, mut paddle| paddle.intent = if paddle.face < WIDTH / 2.0 { intents[0] } else { intents[1] });
    }

    fn decide(&mut self, _: &mut (), _: &mut Cx, mut scores: Query<&Score>, outcomes: Spawner<(Outcome,)>) {
        let (Some(limit), false) = (self.limit(), self.decided) else { return };
        let Some(score) = scores.single(|_, s| *s) else { return };
        let winner = if score.left >= limit {
            0
        } else if score.right >= limit {
            1
        } else {
            return;
        };
        outcomes.spawn((Outcome { winner },));
        self.decided = true;
    }
}

struct Snapshot {
    frame: u64,
    ball: (f32, f32, f32, f32),
    /// Face, y and intent, left first.
    paddles: Vec<(f32, f32, f32)>,
    score: Score,
    turn: Option<TurnInfo>,
    outcome: Option<Outcome>,
}

fn snapshot(world: &mut WorldMut) -> Result<Snapshot, String> {
    let frame = clock::now(world).map_or(0, |c: Clock| c.frame);
    let ball = world.single::<(&Ball, &Position, &Velocity), _>(|_, (_, p, v)| (p.x, p.y, v.x, v.y)).ok_or("no ball: is pong loaded?")?;
    let mut paddles = Vec::new();
    world.for_each::<(&Paddle, &Position)>(|_, (p, at)| paddles.push((p.face, at.y, p.intent)));
    paddles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let score = world.single::<&Score, _>(|_, s| *s).unwrap_or_default();
    Ok(Snapshot { frame, ball, paddles, score, turn: spike_turns::turn(world), outcome: spike_turns::outcome(world) })
}

fn turn_line(s: &Snapshot, limit: Option<u32>) -> String {
    let limit = limit.map_or("no point limit".into(), |n| format!("first to {n}"));
    if let Some(o) = s.outcome {
        let (w, l) = if o.winner == 0 { (s.score.left, s.score.right) } else { (s.score.right, s.score.left) };
        return format!("game over: {} wins {w} to {l} ({limit})", SIDES[o.winner as usize]);
    }
    let Some(t) = s.turn else { return format!("turns are off: start with SPIKE_TURNS ({limit})") };
    if t.playing {
        return format!("turn {} playing, frame {} of {} ({limit})", t.turn, t.played, t.frames);
    }
    let names = |want: bool| -> String {
        let v: Vec<&str> = SIDES.iter().zip(t.submitted).filter(|(_, s)| *s == want).map(|(n, _)| *n).collect();
        if v.is_empty() { "nobody".into() } else { v.join(" and ") }
    };
    format!("turn {} open, {} frames: submitted {}, waiting for {} ({limit})", t.turn, t.frames, names(true), names(false))
}

fn describe(s: &Snapshot, limit: Option<u32>) -> String {
    let (x, y, vx, vy) = s.ball;
    let mut out = format!("{}\nframe {}\nball x {x:.2} y {y:.2} vx {vx:.2} vy {vy:.2}\n", turn_line(s, limit), s.frame);
    let first = s.turn.is_some_and(|t| t.turn == 1 && !t.playing);
    for (i, &(face, y, intent)) in s.paddles.iter().enumerate() {
        let side = SIDES.get(i).copied().unwrap_or("?");
        let last = if first { "none" } else { spike_turns::action(intent) };
        out += &format!("{side} paddle face {face:.0} y {y:.2} last {last}\n");
    }
    out + &format!("score left {} right {}", s.score.left, s.score.right)
}

/// As `pong_text`'s drawing, with the sides named.
fn draw(s: &Snapshot, limit: Option<u32>) -> String {
    let (w, h) = (WIDTH as usize, HEIGHT as usize);
    let mut grid = vec![vec![' '; w]; h];
    for &(face, y, _) in &s.paddles {
        let col = if face < WIDTH / 2.0 { face - 1.0 } else { face };
        for (row, line) in grid.iter_mut().enumerate() {
            if ((row as f32 + 0.5) - y).abs() < PADDLE_HEIGHT / 2.0 {
                line[col as usize] = '#';
            }
        }
    }
    let (bx, by) = (s.ball.0.clamp(0.0, WIDTH - 1.0) as usize, s.ball.1.clamp(0.0, HEIGHT - 1.0) as usize);
    grid[by][bx] = 'o';
    let border = format!("+{}+", "-".repeat(w));
    let mut out = format!("{}\nframe {}   left {} : {} right\n{border}\n", turn_line(s, limit), s.frame, s.score.left, s.score.right);
    for line in grid {
        out += &format!("|{}|\n", line.into_iter().collect::<String>());
    }
    out + &border
}

impl Mod for Versus {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        // In `update`, as `pong_ai`'s `think`: before pong's `play` moves
        // the paddles in `simulate`, so a turn's actions move them on its
        // first frame.
        s.add("steer", Self::steer);
        s.add("decide", Self::decide).phase(phase::LATE).after("pong::rebound");
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let limit = self.limit();
        let mut words = message.split_whitespace();
        match (words.next(), words.next(), words.next()) {
            (Some("state"), None, None) => snapshot(&mut cx.world()).map(|s| describe(&s, limit)),
            (Some("show"), None, None) => snapshot(&mut cx.world()).map(|s| draw(&s, limit)),
            (Some("first-to"), Some("off"), None) => {
                self.endless = true;
                Ok("no point limit".into())
            }
            (Some("first-to"), Some(n), None) => {
                let n: u32 = n.parse().ok().filter(|&n| n > 0).ok_or("first-to takes a number of points, or off")?;
                (self.endless, self.first_to) = (false, n);
                Ok(format!("first to {n} points wins"))
            }
            (None, ..) | (Some("help"), None, None) => Ok(HELP.into()),
            _ => Err(format!("unknown command {:?}; {HELP}", message.trim())),
        }
    }
}

export_mod!(Versus);
