//! The level, and the mod that puts it in the world.
//!
//! The map is `map.txt`, compiled into this mod, so editing the level is an
//! implementation change: `./bazel run //platformer/level` swaps it in without
//! touching any other mod. On load the mod compares the map with the one it last built; if
//! it changed, it replaces every entity it made and moves the player to the
//! new start, and if not (a reload for a code change) it leaves the level
//! alone, so collected coins stay collected.

use engine_api::{Cx, Entity, Mod, World, export_mod};
use platformer::{Coin, GOAL, LevelInfo, Player, SOLID, SPIKE, Tile};
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
        entities: Vec<Entity>,
    }
}

fn map_hash() -> u64 {
    // FNV-1a: only ever compared with an earlier build of this mod, but a
    // `DefaultHasher` isn't promised to agree across Rust versions.
    MAP_TEXT.bytes().fold(0xcbf29ce484222325, |h, b| (h ^ b as u64).wrapping_mul(0x100000001b3))
}

impl Level {
    fn build(&mut self, world: &mut World) -> Result<LevelInfo, String> {
        let map = map();
        let width = map.first().ok_or("the map is empty")?.len();
        if let Some(row) = map.iter().position(|r| r.len() != width) {
            return Err(format!("row {row} is {} wide, not {width}", map[row].len()));
        }
        let mut spawn = None;
        for (y, row) in map.iter().enumerate() {
            for (x, c) in row.chars().enumerate() {
                let (x, y) = (x as i32, y as i32);
                let e = world.spawn();
                match c {
                    '#' => world.insert(e, Tile { x, y, kind: SOLID }),
                    '^' => world.insert(e, Tile { x, y, kind: SPIKE }),
                    'G' => world.insert(e, Tile { x, y, kind: GOAL }),
                    'C' => world.insert(e, Coin { x, y }),
                    'E' => world.insert(e, Walker { x: x as f32, y: y as f32, vx: 0.0 }),
                    'P' => {
                        // Standing on the floor of its cell.
                        spawn = Some((x as f32 + 0.1, y as f32 + 1.0 - platformer::PLAYER_HEIGHT));
                        world.despawn(e)
                    }
                    '.' => world.despawn(e),
                    other => return Err(format!("unknown map character {other:?} at ({x}, {y})")),
                };
                self.entities.push(e);
            }
        }
        let (spawn_x, spawn_y) = spawn.ok_or("the map has no P")?;
        let info = LevelInfo { width: width as i32, height: map.len() as i32, spawn_x, spawn_y };
        let e = world.spawn();
        world.insert(e, info);
        self.entities.push(e);
        Ok(info)
    }

    fn clear(&mut self, world: &mut World) {
        for e in self.entities.drain(..) {
            // Stale handles (a collected coin, a stomped walker) are no-ops.
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
        for (_, p) in world.query::<Player>() {
            *p = Player { x: info.spawn_x, y: info.spawn_y, vx: 0.0, vy: 0.0, ..*p };
        }
        self.built = hash;
        cx.log(format!("{BUILT} a {}x{} level", info.width, info.height));
    }

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        self.clear(&mut cx.world());
    }
}

export_mod!(Level);
