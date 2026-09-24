//! The platformer's text interface, over messages:
//! `modctl send platformer_text <command>`. With the lockstep bootstrap, time
//! only moves with `modctl send lockstep step <frames>`. Moves are sent as
//! `Run` and `Jump` events, which the rules apply at the start of the next
//! frame.
//!
//!   left | right | stop   run (held until changed)
//!   jump                  jump on the next frame, if standing on something
//!   show                  draw the level
//!   state                 the numbers behind the drawing

use clock::Clock;
use engine_api::{Cx, Mod, WorldMut, export_mod};
use platformer::{
    Coin, GOAL, Input, Jump, LevelInfo, PLAYER_HEIGHT, PLAYER_WIDTH, Player, Run, SOLID, SPIKE, Tile,
};
use walkers::Walker;

const HELP: &str = "commands: left | right | stop | jump | show | state";

engine_api::mod_state! {
    #[derive(Default)]
    struct Text {}
}

/// Sends `event`, if there's a player to act on it.
fn send(cx: &mut Cx, event: impl engine_api::Event) -> Result<(), String> {
    if cx.world().single::<&Player, ()>(|_, _| ()).is_none() {
        return Err("no player yet: is a level loaded? try `step`".into());
    }
    cx.send_event(event);
    Ok(())
}

struct Snapshot {
    frame: u64,
    info: LevelInfo,
    player: Player,
    input: Input,
    tiles: Vec<Tile>,
    coins: Vec<Coin>,
    walkers: Vec<Walker>,
}

/// Every `T` in the world, copied out.
fn all<T: engine_api::Component + Copy>(world: &mut WorldMut) -> Vec<T> {
    let mut out = Vec::new();
    world.for_each::<&T>(|_, t| out.push(*t));
    out
}

fn snapshot(world: &mut WorldMut) -> Result<Snapshot, String> {
    let frame = clock::now(world).map_or(0, |c: Clock| c.frame);
    let info = world.single::<&LevelInfo, _>(|_, i| *i).ok_or("no level loaded")?;
    let (input, player) =
        world.single::<(&Input, &Player), _>(|_, (i, p)| (*i, *p)).ok_or("no player yet: try `step`")?;
    Ok(Snapshot {
        frame,
        info,
        player,
        input,
        tiles: all(world),
        coins: all(world),
        walkers: all(world),
    })
}

fn header(s: &Snapshot) -> String {
    let won = if s.player.won { "   YOU WIN" } else { "" };
    format!(
        "frame {}   coins {} ({} left)   deaths {}{won}",
        s.frame,
        s.player.coins,
        s.coins.len(),
        s.player.deaths
    )
}

fn draw(s: &Snapshot) -> String {
    let (w, h) = (s.info.width as usize, s.info.height as usize);
    let mut grid = vec![vec!['.'; w]; h];
    let mut put = |x: i32, y: i32, c: char| {
        if (0..w as i32).contains(&x) && (0..h as i32).contains(&y) {
            grid[y as usize][x as usize] = c;
        }
    };
    for t in &s.tiles {
        let c = match t.kind {
            SOLID => '#',
            SPIKE => '^',
            GOAL => 'G',
            _ => '?',
        };
        put(t.x, t.y, c);
    }
    for c in &s.coins {
        put(c.x, c.y, 'C');
    }
    for walker in &s.walkers {
        put((walker.x + 0.5).floor() as i32, (walker.y + 0.5).floor() as i32, 'E');
    }
    let p = &s.player;
    put((p.x + PLAYER_WIDTH / 2.0).floor() as i32, (p.y + PLAYER_HEIGHT / 2.0).floor() as i32, '@');

    let mut out = header(s) + "\n";
    for row in grid {
        out.extend(row);
        out.push('\n');
    }
    out.trim_end().to_string()
}

fn describe(s: &Snapshot) -> String {
    let p = &s.player;
    let mut out = header(s) + "\n";
    out += &format!(
        "player x {:.2} y {:.2} vx {:.2} vy {:.2} on_ground {}\ninput dir {} jump {}\n",
        p.x, p.y, p.vx, p.vy, p.on_ground, s.input.dir, s.input.jump
    );
    for w in &s.walkers {
        out += &format!("walker x {:.2} y {:.2} vx {:.2}\n", w.x, w.y, w.vx);
    }
    for c in &s.coins {
        out += &format!("coin at {} {}\n", c.x, c.y);
    }
    out.trim_end().to_string()
}

impl Mod for Text {
    type Transient = ();

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        match message.trim() {
            "left" => send(cx, Run { dir: -1.0 }).map(|()| "running left".into()),
            "right" => send(cx, Run { dir: 1.0 }).map(|()| "running right".into()),
            "stop" => send(cx, Run { dir: 0.0 }).map(|()| "stopped".into()),
            "jump" => send(cx, Jump {}).map(|()| "jumping next frame".into()),
            "show" => snapshot(&mut cx.world()).map(|s| draw(&s)),
            "state" => snapshot(&mut cx.world()).map(|s| describe(&s)),
            "" | "help" => Ok(HELP.into()),
            other => Err(format!("unknown command {other:?}; {HELP}")),
        }
    }
}

export_mod!(Text);
