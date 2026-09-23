//! The platformer's rules as a system: runs and jumps the player by its
//! `Input`, collides it with the level's tiles, collects coins, and kills and
//! respawns it on spikes or a fall. Also provides `platformer::Rules`, so other
//! mods (enemies) can kill or bounce the player.
//!
//! The player is spawned once a level exists (a `LevelInfo`) and there's no
//! player yet, so this mod doesn't care whether it loads before or after the
//! level, or reloads.

use std::collections::HashMap;

use clock::Clock;
use engine_api::{Cx, Mod, Systems, World, export_mod, phase};
use platformer::{
    Coin, GOAL, GRAVITY, Input, JUMP_SPEED, LevelInfo, MAX_FALL, PLAYER_HEIGHT, PLAYER_WIDTH, Player,
    RUN_SPEED, SOLID, SPIKE, Tile,
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
    fn read(world: &mut World, width: i32) -> Tiles {
        let kinds = world.query::<Tile>().map(|(_, t)| ((t.x, t.y), t.kind)).collect();
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

/// Changes the player, if there is one.
fn with_player(cx: &mut Cx, change: impl FnOnce(&mut Player, &LevelInfo)) {
    let mut world = cx.world();
    let Some(info) = world.query::<LevelInfo>().next().map(|(_, i)| *i) else { return };
    if let Some((_, p)) = world.query::<Player>().next() {
        change(p, &info);
    }
}

impl Core {
    fn play(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        let Some(Clock { dt, .. }) = clock::now(&mut world) else {
            return;
        };
        let Some(info) = world.query::<LevelInfo>().next().map(|(_, i)| *i) else {
            return;
        };
        let player = world.query2::<Input, Player>().next().map(|(e, i, p)| (e, *i, *p));
        let Some((entity, mut input, mut p)) = player else {
            let e = world.spawn();
            world.insert(e, Player { x: info.spawn_x, y: info.spawn_y, ..Default::default() });
            world.insert(e, Input::default());
            return;
        };

        let tiles = Tiles::read(&mut world, info.width);
        integrate(&mut p, &mut input, &tiles, dt);

        if tiles.touches(&p, SPIKE) || p.y > info.height as f32 + 2.0 {
            respawn(&mut p, &info);
        }
        if tiles.touches(&p, GOAL) {
            p.won = true;
        }

        // Collected first and despawned after: a query can't be open while
        // the world changes shape.
        let cells: Vec<(i32, i32)> = Tiles::cells(p.x, p.y, PLAYER_WIDTH, PLAYER_HEIGHT).collect();
        let collected: Vec<_> =
            world.query::<Coin>().filter(|(_, c)| cells.contains(&(c.x, c.y))).map(|(e, _)| e).collect();
        for coin in collected {
            world.despawn(coin);
            p.coins += 1;
        }

        world.insert(entity, input);
        world.insert(entity, p);
    }
}

impl Mod for Core {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("play", Self::play).phase(phase::SIMULATE).exclusive();
    }

}

impl platformer::Rules for Core {
    fn hurt(&mut self, _: &mut (), cx: &mut Cx) {
        with_player(cx, respawn);
    }

    fn bounce(&mut self, _: &mut (), cx: &mut Cx, speed: f32) {
        with_player(cx, |p, _| p.vy = -speed);
    }
}

export_mod!(Core, provides = [platformer::Rules]);
