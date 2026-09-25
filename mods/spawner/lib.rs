//! Builds a walled box, drops bodies into it, and once a second tosses one
//! back up, so something is always moving. The bodies live in the world, so
//! reloading this mod (or physics, or any other) leaves them where they are.

use engine_api::{ChildOf, Cx, Entity, Mod, Query, Systems, With, component, children_of, export_mod};
use physics::{Body, Collider, Gravity, Position, Velocity};

const WIDTH: f32 = 20.0;
const FLOOR: f32 = 12.0;
const BODIES: usize = 24;

component! {
    /// The box this mod builds: everything in it is `ChildOf` this entity,
    /// so the world, not this mod's state, says what the mod made.
    #[derive(Debug, Default, Copy)]
    pub struct Scene: "spawner::Scene" {}
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Spawner {
        frames: u64,
    }
}

/// The box, if it's been built.
fn scene(world: &mut engine_api::WorldMut) -> Option<Entity> {
    world.single::<&Scene, _>(|e, _| e)
}

impl Spawner {
    fn toss(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut scenes: Query<(), With<Scene>>,
        mut bodies: Query<(&ChildOf, &mut Velocity), With<Body>>,
    ) {
        self.frames += 1;
        if !self.frames.is_multiple_of(60) {
            return;
        }
        let Some(scene) = scenes.single(|row, ()| row.entity()) else { return };
        // One of the box's bodies (its children, the walls skipped by
        // `With<Body>`), a different one each time. Bodies are kept in
        // spatial order, not by parent, so this scans rather than seeks,
        // and which is `k`th changes as they move.
        let k = (self.frames / 60) as usize % BODIES;
        let mut i = 0;
        bodies.in_keys::<ChildOf>(children_of(scene), |_, (_, mut v)| {
            if i == k {
                (v.x, v.y) = (3.0, -14.0);
            }
            i += 1;
        });
    }
}

impl Mod for Spawner {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("toss", Self::toss);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        if scene(&mut world).is_some() {
            return;
        }
        let of = ChildOf { parent: world.spawn((Scene {},)) };
        world.spawn((Gravity { x: 0.0, y: 20.0 }, of));
        for (x, y, hx, hy) in [(WIDTH / 2.0, FLOOR + 0.5, WIDTH / 2.0, 0.5), (-0.5, 0.0, 0.5, FLOOR), (WIDTH + 0.5, 0.0, 0.5, FLOOR)] {
            world.spawn((Position { x, y }, Collider::rect(hx, hy), of));
        }
        for i in 0..BODIES {
            let at = Position { x: 1.0 + (i % 12) as f32 * 1.5, y: 1.0 + (i / 12) as f32 * 1.5 };
            let collider = if i % 2 == 0 { Collider::circle(0.5) } else { Collider::rect(0.5, 0.5) };
            let body = Body { restitution: 0.6, friction: 0.3, ..Body::default() };
            world.spawn((at, Velocity { x: 2.0 - (i % 5) as f32, y: 0.0 }, body, collider, of));
        }
        cx.log(format!("spawned a box of {BODIES} bodies"));
    }

    /// Only on unload for good (or a state reset): the entities are this mod's
    /// to clean up.
    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        let Some(scene) = scene(&mut world) else { return };
        let mut made = Vec::new();
        world.for_each::<&ChildOf>(|e, of| {
            if of.parent == scene {
                made.push(e);
            }
        });
        for e in made.into_iter().chain([scene]) {
            world.despawn(e);
        }
        cx.log("despawned its entities");
    }
}

export_mod!(Spawner);
