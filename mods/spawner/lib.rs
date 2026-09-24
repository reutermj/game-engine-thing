//! Builds a walled box, drops bodies into it, and once a second tosses one
//! back up, so something is always moving. The bodies live in the world, so
//! reloading this mod (or physics, or any other) leaves them where they are.

use engine_api::{Cx, Entity, Mod, Query, Systems, With, export_mod};
use physics::{Body, Collider, Gravity, Position, Velocity};

const WIDTH: f32 = 20.0;
const FLOOR: f32 = 12.0;
const BODIES: usize = 24;

engine_api::mod_state! {
    #[derive(Default)]
    struct Spawner {
        /// Empty until the first load. Mod state survives reloads, so a reload
        /// sees these and doesn't spawn again.
        spawned: Vec<Entity>,
        frames: u64,
    }
}

impl Spawner {
    fn toss(&mut self, _: &mut (), _: &mut Cx, mut bodies: Query<&mut Velocity, With<Body>>) {
        self.frames += 1;
        if self.frames % 60 != 0 || self.spawned.len() < BODIES {
            return;
        }
        // The walls and gravity come first, and aren't bodies.
        let bodies_from = self.spawned.len() - BODIES;
        let e = self.spawned[bodies_from + (self.frames / 60) as usize % BODIES];
        bodies.with(e, |_, v| (v.x, v.y) = (3.0, -14.0));
    }
}

impl Mod for Spawner {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("toss", Self::toss);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        if !self.spawned.is_empty() {
            return;
        }
        let mut world = cx.world();
        self.spawned.push(world.spawn((Gravity { x: 0.0, y: 20.0 },)));
        for (x, y, hx, hy) in [(WIDTH / 2.0, FLOOR + 0.5, WIDTH / 2.0, 0.5), (-0.5, 0.0, 0.5, FLOOR), (WIDTH + 0.5, 0.0, 0.5, FLOOR)] {
            self.spawned.push(world.spawn((Position { x, y }, Collider::rect(hx, hy))));
        }
        for i in 0..BODIES {
            let at = Position { x: 1.0 + (i % 12) as f32 * 1.5, y: 1.0 + (i / 12) as f32 * 1.5 };
            let collider = if i % 2 == 0 { Collider::circle(0.5) } else { Collider::rect(0.5, 0.5) };
            let body = Body { restitution: 0.6, friction: 0.3, ..Body::default() };
            self.spawned.push(world.spawn((at, Velocity { x: 2.0 - (i % 5) as f32, y: 0.0 }, body, collider)));
        }
        let n = self.spawned.len();
        cx.log(format!("spawned {n} entities"));
    }

    /// Only on unload for good (or a state reset): the entities are this mod's
    /// to clean up.
    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        for e in self.spawned.drain(..) {
            world.despawn(e);
        }
        cx.log("despawned its entities");
    }
}

export_mod!(Spawner);
