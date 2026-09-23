//! The platformer's text interface, over messages:
//! `modctl send platformer_text <command>`. With the lockstep bootstrap, time
//! only moves with `modctl send lockstep step <frames>`.
//!
//!   left | right | stop   run (held until changed)
//!   jump                  jump on the next frame, if standing on something
//!   show                  draw the level
//!   state                 the numbers behind the drawing

use clock::Clock;
use engine_api::{Cx, Mod, World, export_mod};
use platformer::{
    Coin, GOAL, Input, LevelInfo, PLAYER_HEIGHT, PLAYER_WIDTH, Player, SOLID, SPIKE, Tile,
};
use walkers::Walker;

const HELP: &str = "commands: left | right | stop | jump | show | state";

engine_api::mod_state! {
    #[derive(Default)]
    struct Text {}
}

fn with_input(world: &mut World, f: impl FnOnce(&mut Input)) -> Result<(), String> {
    let input = world.query2::<Input, Player>().next().map(|(_, input, _)| input);
    input.map(f).ok_or_else(|| "no player yet: is a level loaded? try `step`".into())
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

fn snapshot(world: &mut World) -> Result<Snapshot, String> {
    let frame = clock::now(world).map_or(0, |c: Clock| c.frame);
    let info = world.query::<LevelInfo>().next().map(|(_, i)| *i).ok_or("no level loaded")?;
    let (input, player) = world
        .query2::<Input, Player>()
        .next()
        .map(|(_, i, p)| (*i, *p))
        .ok_or("no player yet: try `step`")?;
    Ok(Snapshot {
        frame,
        info,
        player,
        input,
        tiles: world.query::<Tile>().map(|(_, t)| *t).collect(),
        coins: world.query::<Coin>().map(|(_, c)| *c).collect(),
        walkers: world.query::<Walker>().map(|(_, w)| *w).collect(),
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
        let mut world = cx.world();
        match message.trim() {
            "left" => with_input(&mut world, |i| i.dir = -1.0).map(|()| "running left".into()),
            "right" => with_input(&mut world, |i| i.dir = 1.0).map(|()| "running right".into()),
            "stop" => with_input(&mut world, |i| i.dir = 0.0).map(|()| "stopped".into()),
            "jump" => with_input(&mut world, |i| i.jump = true).map(|()| "jumping next frame".into()),
            "show" => snapshot(&mut world).map(|s| draw(&s)),
            "state" => snapshot(&mut world).map(|s| describe(&s)),
            "" | "help" => Ok(HELP.into()),
            other => Err(format!("unknown command {other:?}; {HELP}")),
        }
    }
}

export_mod!(Text);
