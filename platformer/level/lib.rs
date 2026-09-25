//! The level, and the mod that puts it in the world.
//!
//! The map is `map.txt`, compiled into this mod, so editing the level is an
//! implementation change: `./bazel run //platformer/level` swaps it in without
//! touching any other mod. On load the mod compares the map with the one it last built; if
//! it changed, it replaces every entity it made and moves the player to the
//! new start, and if not (a reload for a code change) it leaves the level
//! alone, so collected coins stay collected.
//!
//! What it made is in the world, not this mod's state: the level is the
//! entity holding its `LevelInfo`, and everything built from the map is
//! `ChildOf` it.

use engine_api::{ChildOf, Cx, Entity, Mod, WorldMut, export_mod};
use physics::{Body, Collider, Position, Touching, Velocity};
use platformer::{Coin, GOAL, LevelInfo, PLAYER, PLAYER_HEIGHT, PLAYER_WIDTH, Player, SENSORS, SOLID, SPIKE, TILES, Tile, WALKERS};
use walkers::Walker;

/// The map: `#` solid, `^` spikes, `G` goal, `C` coin, `E` walker, `P` player
/// start, `.` empty. The test builds of this mod swap in another map
/// (`test_alt_map`) or change only the code (`test_alt_code`); see BUILD.bazel.
#[cfg(not(feature = "test_alt_map"))]
const MAP_TEXT: &str = include_str!("map.txt");
#[cfg(feature = "test_alt_map")]
const MAP_TEXT: &str = include_str!("test_alt_map.txt");

#[cfg(not(feature = "test_alt_code"))]
const BUILT: &str = "built";
#[cfg(feature = "test_alt_code")]
const BUILT: &str = "built (alt code)";

fn map() -> Vec<&'static str> {
    MAP_TEXT.lines().collect()
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Level {
        /// Hash of the map these entities were built from; 0 before the first build.
        built: u64,
    }
}

fn map_hash() -> u64 {
    // FNV-1a: only ever compared with an earlier build of this mod, but a
    // `DefaultHasher` isn't promised to agree across Rust versions.
    MAP_TEXT.bytes().fold(0xcbf29ce484222325, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

impl Level {
    fn build(&mut self, world: &mut WorldMut) -> Result<LevelInfo, String> {
        let map = map();
        let width = map.first().ok_or("the map is empty")?.len();
        if let Some(row) = map.iter().position(|r| r.len() != width) {
            return Err(format!("row {row} is {} wide, not {width}", map[row].len()));
        }
        let mut spawn = None;
        // The level itself, its info filled in once the map is read.
        let level = world.spawn((LevelInfo::default(),));
        let of = ChildOf { parent: level };
        for (y, row) in map.iter().enumerate() {
            for (x, c) in row.chars().enumerate() {
                let (x, y) = (x as i32, y as i32);
                // Everything is one cell, centered in it.
                let at = Position { x: x as f32 + 0.5, y: y as f32 + 0.5 };
                let solid = Collider::rect(0.5, 0.5).on(TILES, u32::MAX);
                let sensor = Collider::rect(0.5, 0.5).on(SENSORS, PLAYER).sensor();
                match c {
                    '#' => world.spawn((Tile { x, y, kind: SOLID }, at, solid, of)),
                    '^' => world.spawn((Tile { x, y, kind: SPIKE }, at, sensor, of)),
                    'G' => world.spawn((Tile { x, y, kind: GOAL }, at, sensor, of)),
                    'C' => world.spawn((Coin { x, y }, at, sensor, of)),
                    'E' => world.spawn((
                        Walker {},
                        at,
                        Velocity::default(),
                        Body { friction: 0.0, ..Body::default() },
                        Collider::rect(0.5, 0.5).on(WALKERS, TILES).sensing(PLAYER),
                        Touching::default(),
                        of,
                    )),
                    'P' => {
                        // Standing on the floor of its cell.
                        spawn = Some((x as f32 + 0.1, y as f32 + 1.0 - platformer::PLAYER_HEIGHT));
                        continue;
                    }
                    '.' => continue,
                    other => return Err(format!("unknown map character {other:?} at ({x}, {y})")),
                };
            }
        }
        let (spawn_x, spawn_y) = spawn.ok_or("the map has no P")?;
        let info = LevelInfo { width: width as i32, height: map.len() as i32, spawn_x, spawn_y };
        // The level's sides are walls, however high the player jumps.
        let (w, h) = (width as f32, map.len() as f32);
        for x in [-0.5, w + 0.5] {
            world.spawn((Position { x, y: h / 2.0 }, Collider::rect(0.5, h * 4.0).on(TILES, u32::MAX), of));
        }
        world.insert(level, info);
        Ok(info)
    }

    /// Despawns every level and what's left of what was built from it (a
    /// collected coin or a stomped walker is gone already).
    fn clear(&mut self, world: &mut WorldMut) {
        let mut levels: Vec<Entity> = Vec::new();
        world.for_each::<&LevelInfo>(|e, _| levels.push(e));
        let mut built: Vec<Entity> = Vec::new();
        world.for_each::<&ChildOf>(|e, of| {
            if levels.contains(&of.parent) {
                built.push(e);
            }
        });
        for e in built.into_iter().chain(levels) {
            world.despawn(e);
        }
    }
}

impl Mod for Level {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        let hash = map_hash();
        if hash == self.built {
            return;
        }
        let mut world = cx.world();
        self.clear(&mut world);
        let result = self.build(&mut world);
        let info = match result {
            Ok(info) => info,
            Err(e) => {
                self.clear(&mut world);
                self.built = 0;
                cx.log(format!("not loading the map: {e}"));
                return;
            }
        };
        // A player from the previous map may be inside a wall of this one.
        let start = Position { x: info.spawn_x + PLAYER_WIDTH / 2.0, y: info.spawn_y + PLAYER_HEIGHT / 2.0 };
        world.for_each::<(&Player, &mut Position, &mut Velocity)>(|_, (_, mut p, mut v)| (*p, *v) = (start, Velocity::default()));
        self.built = hash;
        cx.log(format!("{BUILT} a {}x{} level", info.width, info.height));
    }

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        self.clear(&mut cx.world());
    }
}

export_mod!(Level);
