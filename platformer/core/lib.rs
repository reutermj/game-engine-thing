//! The platformer's rules, as systems: `steer` turns `Run` and `Jump` events
//! into the player's `Input` (in `input`), and `play` runs and jumps the
//! player by it, collides it with the level's tiles, collects coins, and
//! kills and respawns it on spikes or a fall (in `simulate`); `take_hits`
//! applies the `Hurt` and `Bounce` events other mods (enemies) send (in
//! `late`).
//!
//! The player is spawned once a level exists (a `LevelInfo`) and there's no
//! player yet, so this mod doesn't care whether it loads before or after the
//! level, or reloads.

use std::collections::HashMap;

use clock::Clock;
use engine_api::{Cx, Despawns, EventReader, Mod, Query, Spawner, Systems, export_mod, phase};
use platformer::{
    Bounce, Coin, GOAL, GRAVITY, Hurt, Input, JUMP_SPEED, Jump, LevelInfo, MAX_FALL, PLAYER_HEIGHT, PLAYER_WIDTH,
    Player, RUN_SPEED, Run, SOLID, SPIKE, Tile,
};

/// Keeps a box that is flush against a tile from counting as inside it.
const EPSILON: f32 = 1e-4;

engine_api::mod_state! {
    #[derive(Default)]
    struct Core {}
}

/// The level's tiles by cell, rebuilt each frame. Rebuilding is cheap at this
/// size, and it means a level reload needs no notification.
struct Tiles {
    kinds: HashMap<(i32, i32), u8>,
    width: i32,
}

impl Tiles {
    fn read(tiles: &mut Query<&Tile>, width: i32) -> Tiles {
        let mut kinds = HashMap::new();
        tiles.for_each(|_, t| {
            kinds.insert((t.x, t.y), t.kind);
        });
        Tiles { kinds, width }
    }

    /// Solid tiles, and the level's left and right edges.
    fn solid(&self, x: i32, y: i32) -> bool {
        x < 0 || x >= self.width || self.kinds.get(&(x, y)) == Some(&SOLID)
    }

    /// The cells the box `(x, y, w, h)` overlaps.
    fn cells(x: f32, y: f32, w: f32, h: f32) -> impl Iterator<Item = (i32, i32)> {
        let (x0, x1) = (x.floor() as i32, (x + w - EPSILON).floor() as i32);
        let (y0, y1) = (y.floor() as i32, (y + h - EPSILON).floor() as i32);
        (y0..=y1).flat_map(move |cy| (x0..=x1).map(move |cx| (cx, cy)))
    }

    fn touches(&self, p: &Player, kind: u8) -> bool {
        Self::cells(p.x, p.y, PLAYER_WIDTH, PLAYER_HEIGHT).any(|c| self.kinds.get(&c) == Some(&kind))
    }
}

/// Moves the player one frame: horizontally then vertically, each resolved
/// against solid tiles separately. Speeds stay under a tile per frame, so one
/// correction per axis is enough.
fn integrate(p: &mut Player, input: &mut Input, tiles: &Tiles, dt: f32) {
    p.vx = input.dir.clamp(-1.0, 1.0) * RUN_SPEED;
    if std::mem::take(&mut input.jump) && p.on_ground {
        p.vy = -JUMP_SPEED;
    }
    p.vy = (p.vy + GRAVITY * dt).min(MAX_FALL);

    p.x += p.vx * dt;
    let blocked = Tiles::cells(p.x, p.y, PLAYER_WIDTH, PLAYER_HEIGHT).find(|&(cx, cy)| tiles.solid(cx, cy));
    if let Some((cx, _)) = blocked {
        p.x = if p.vx > 0.0 { cx as f32 - PLAYER_WIDTH } else { cx as f32 + 1.0 };
        p.vx = 0.0;
    }

    p.y += p.vy * dt;
    p.on_ground = false;
    let blocked = Tiles::cells(p.x, p.y, PLAYER_WIDTH, PLAYER_HEIGHT).find(|&(cx, cy)| tiles.solid(cx, cy));
    if let Some((_, cy)) = blocked {
        if p.vy > 0.0 {
            p.y = cy as f32 - PLAYER_HEIGHT;
            p.on_ground = true;
        } else {
            p.y = cy as f32 + 1.0;
        }
        p.vy = 0.0;
    }
}

fn respawn(p: &mut Player, info: &LevelInfo) {
    (p.x, p.y, p.vx, p.vy) = (info.spawn_x, info.spawn_y, 0.0, 0.0);
    p.deaths += 1;
}

impl Core {
    fn steer(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut runs: EventReader<Run>,
        mut jumps: EventReader<Jump>,
        mut inputs: Query<(&mut Input, &Player)>,
    ) {
        let dir = runs.read().last().map(|r| r.dir);
        let jump = !jumps.read().is_empty();
        inputs.for_each(|_, (input, _)| {
            if let Some(dir) = dir {
                input.dir = dir;
            }
            input.jump |= jump;
        });
    }

    #[allow(clippy::too_many_arguments)]
    fn play(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut clocks: Query<&Clock>,
        mut levels: Query<&LevelInfo>,
        mut players: Query<(&mut Input, &mut Player)>,
        mut tiles: Query<&Tile>,
        mut coins: Query<&Coin, (), Despawns>,
        spawner: Spawner<(Player, Input)>,
    ) {
        let Some(dt) = clocks.single(|_, c| c.dt) else { return };
        let Some(info) = levels.single(|_, i| *i) else { return };
        let tiles = Tiles::read(&mut tiles, info.width);
        let played = players.single(|_, (input, p)| {
            integrate(p, input, &tiles, dt);

            if tiles.touches(p, SPIKE) || p.y > info.height as f32 + 2.0 {
                respawn(p, &info);
            }
            if tiles.touches(p, GOAL) {
                p.won = true;
            }

            let cells: Vec<(i32, i32)> = Tiles::cells(p.x, p.y, PLAYER_WIDTH, PLAYER_HEIGHT).collect();
            coins.for_each(|coin, c| {
                if cells.contains(&(c.x, c.y)) {
                    coin.despawn();
                    p.coins += 1;
                }
            });
        });
        if played.is_none() {
            // Seen by the systems after this one: this frame doesn't move it.
            spawner.spawn((Player { x: info.spawn_x, y: info.spawn_y, ..Default::default() }, Input::default()));
        }
    }

    /// Applies what other mods did to the player this frame. In `late`, so
    /// it sees every `simulate` system's events the same frame.
    fn take_hits(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut hurts: EventReader<Hurt>,
        mut bounces: EventReader<Bounce>,
        mut levels: Query<&LevelInfo>,
        mut players: Query<&mut Player>,
    ) {
        let bounce = bounces.read().last().map(|b| b.speed);
        let hurt = !hurts.read().is_empty();
        let Some(info) = levels.single(|_, i| *i) else { return };
        players.for_each(|_, p| {
            if let Some(speed) = bounce {
                p.vy = -speed;
            }
            if hurt {
                respawn(p, &info);
            }
        });
    }
}

impl Mod for Core {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("steer", Self::steer).phase(phase::INPUT);
        s.add("play", Self::play).phase(phase::SIMULATE);
        s.add("take_hits", Self::take_hits).phase(phase::LATE);
    }
}

export_mod!(Core);
