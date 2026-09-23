//! Spawns a few moving entities, once. They live in the world, so reloading
//! this mod (or any other) leaves them where they are.

use engine_api::{Cx, Entity, Mod, Status, export_mod};
use physics::Velocity;
use transform::Position;

#[derive(Default)]
struct Spawner {
    /// Empty until the first load. Mod state survives reloads, so a reload
    /// sees these and doesn't spawn again.
    spawned: Vec<Entity>,
}

impl Mod for Spawner {
    fn load(&mut self, cx: &mut Cx) {
        if !self.spawned.is_empty() {
            return;
        }
        let mut world = cx.world();
        for (i, vx) in [1.0, 2.0, 3.0].into_iter().enumerate() {
            let e = world.spawn();
            world.insert(e, Position { x: 0.0, y: i as f32 });
            world.insert(e, Velocity { x: vx, y: 0.0 });
            self.spawned.push(e);
        }
        let n = self.spawned.len();
        cx.log(format!("spawned {n} entities"));
    }

    fn step(&mut self, _cx: &mut Cx) -> Status {
        Status::OK
    }

    /// Only on unload for good (or a state reset): the entities are this mod's
    /// to clean up.
    fn close(&mut self, cx: &mut Cx) {
        let mut world = cx.world();
        for e in self.spawned.drain(..) {
            world.despawn(e);
        }
        cx.log("despawned its entities");
    }
}

export_mod!(Spawner);
