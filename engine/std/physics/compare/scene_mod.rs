//! The comparison's scenes in the engine, built by the same code
//! (`scene.rs`) the other engines are, and run by the physics mod:
//!
//!   build <scene>  spawn a scene's bodies (`Scene::parse`: pile <n> <width>,
//!              columns <n> <width>, pyramid <base>, rain <n> <width>); a
//!              rain scene then rains every step, from `rain`, a system
//!   sleep default  leave sleeping to physics's default, which is on; the
//!              scenes have it off, as the other engines do unless asked
//!
//! Rain is a system rather than a message the bench sends each step, since
//! that is how a game spawns and despawns: through a `Spawner` and a
//! query's rows, applied at the system's apply node, not by `WorldMut`
//! one entity at a time.

// Shared with the bench, which uses more of it than the mod.
#[allow(dead_code)]
#[path = "scene.rs"]
mod scene;

use engine_api::{Cx, Despawns, Entity, Mod, Query, Spawner, Systems, WorldMut, export_mod};
use physics::{Body, Collider, Gravity, Position, Sleep, Velocity};
use scene::{GRAVITY, Scene, Spec};

engine_api::mod_state! {
    #[derive(Default)]
    struct Scenes {
        /// The rain scene's numbers (`n`, `width`), once built.
        rain: Vec<f32>,
        tick: u32,
        /// Raindrops alive, oldest first from `head`, a ring once full.
        drops: Vec<Entity>,
        head: u32,
    }
}

type Dynamic = (Position, Velocity, Body, Collider);

fn collider(s: &Spec) -> Collider {
    if s.circle { Collider::circle(s.hx) } else { Collider::rect(s.hx, s.hy) }
}

fn dynamic(s: &Spec) -> Dynamic {
    let body = Body { friction: s.friction, restitution: s.restitution, ..Body::default() };
    (Position { x: s.x, y: s.y }, Velocity { x: s.vx, y: s.vy }, body, collider(s))
}

fn spawn(world: &mut WorldMut, s: &Spec) -> Entity {
    if !s.dynamic {
        // A collider without a body is static with `Body::fixed()`'s
        // friction and restitution, which is what `scene::wall` gives the
        // other engines.
        return world.spawn((Position { x: s.x, y: s.y }, collider(s)));
    }
    world.spawn(dynamic(s))
}

impl Scenes {
    /// The next step's arrivals, and the oldest removed once more than the
    /// scene's n are alive.
    fn rain(&mut self, _: &mut (), _: &mut Cx, spawner: Spawner<Dynamic>, mut drops: Query<&Body, (), Despawns>) {
        let &[n, width] = self.rain.as_slice() else { return };
        let scene = Scene::Rain { n: n as u32, width };
        for s in scene.rain(self.tick) {
            let e = spawner.spawn(dynamic(&s));
            if self.drops.len() < n as usize {
                self.drops.push(e);
            } else {
                let old = std::mem::replace(&mut self.drops[self.head as usize], e);
                drops.with(old, |row, _| row.despawn());
                self.head = (self.head + 1) % n as u32;
            }
        }
        self.tick += 1;
    }
}

impl Mod for Scenes {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("rain", Self::rain);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        if self.tick > 0 || !self.drops.is_empty() {
            return;
        }
        let mut world = cx.world();
        world.spawn((Gravity { x: 0.0, y: GRAVITY },));
        world.spawn((Sleep::OFF,));
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut world = cx.world();
        match message.split_once(' ') {
            Some(("build", text)) => {
                let scene = Scene::parse(text).ok_or_else(|| format!("no scene {text:?}"))?;
                let specs = scene.build();
                for s in &specs {
                    spawn(&mut world, s);
                }
                if let Scene::Rain { n, width } = scene {
                    self.rain = vec![n as f32, width];
                }
                Ok(format!("built {} bodies", specs.len()))
            }
            Some(("sleep", "default")) => {
                let mut was = Vec::new();
                world.for_each::<&Sleep>(|e, _| was.push(e));
                was.into_iter().for_each(|e| world.despawn(e));
                Ok("sleeping by default".into())
            }
            None if message.trim() == "drops" => Ok(format!("drops {} tick {}", self.drops.len(), self.tick)),
            _ => Err("commands: build <scene> | sleep default | drops".into()),
        }
    }
}

export_mod!(Scenes);
