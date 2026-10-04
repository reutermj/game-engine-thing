//! The comparison's scenes in the engine, built by the same code
//! (`scene.rs`) the other engines are, and run by the physics mod:
//!
//!   build <scene>  spawn a scene's bodies (`Scene::parse`: pile <n> <width>,
//!              columns <n> <width>, pyramid <base>, rain <n> <width>); a
//!              rain scene then rains every step, from `rain`, a system
//!   sleep default  leave sleeping to physics's default, which is on; the
//!              scenes have it off, as the other engines do unless asked
//!   turn       every dynamic body turns (a `Rotation` and a `Spin`), and
//!              every raindrop from then on
//!   orient     every dynamic body has a `Rotation` and no `Spin`: faces a
//!              way, and doesn't turn
//!   substeps <n>  physics's `Tuning` in the world, at n substeps
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
use physics2d::{Body, Collider, Gravity, Position, Rotation, Sleep, Spin, Tuning, Velocity};
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
        /// Whether bodies turn, and so what rains does.
        turning: bool,
    }
}

type Dynamic = (Position, Velocity, Body, Collider);
type Turning = (Position, Velocity, Body, Collider, Rotation, Spin);

fn collider(s: &Spec) -> Collider {
    if s.circle { Collider::circle(s.hx) } else { Collider::rect(s.hx, s.hy) }
}

fn dynamic(s: &Spec) -> Dynamic {
    let body = Body {
        friction: s.friction,
        restitution: s.restitution,
        inv_mass: 1.0 / s.mass,
        gravity_scale: s.gravity_scale,
        ..Body::default()
    };
    (Position { x: s.x, y: s.y }, Velocity { x: s.vx, y: s.vy }, body, collider(s))
}

fn spawn(world: &mut WorldMut, s: &Spec) -> Entity {
    let e = if s.dynamic {
        world.spawn(dynamic(s))
    } else if s.kinematic {
        let body = Body { friction: s.friction, restitution: s.restitution, ..Body::kinematic() };
        world.spawn((Position { x: s.x, y: s.y }, Velocity { x: s.vx, y: s.vy }, body, collider(s)))
    } else if s.friction == Body::fixed().friction && s.restitution == Body::fixed().restitution {
        // A collider without a body is static with `Body::fixed()`'s
        // friction and restitution, which is what `scene::wall` gives the
        // other engines.
        world.spawn((Position { x: s.x, y: s.y }, collider(s)))
    } else {
        let body = Body { friction: s.friction, restitution: s.restitution, ..Body::fixed() };
        world.spawn((Position { x: s.x, y: s.y }, collider(s), body))
    };
    // A turned static is a ramp; a turned body starts so, and one spinning
    // turns from the start (`turn` gives the rest theirs).
    if s.angle != 0.0 {
        world.insert(e, Rotation::from_angle(s.angle));
    }
    if s.dynamic && s.w != 0.0 {
        world.insert(e, Spin { w: s.w });
    }
    e
}

impl Scenes {
    /// The next step's arrivals, and the oldest removed once more than the
    /// scene's n are alive.
    fn rain(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        (spawner, turners): (Spawner<Dynamic>, Spawner<Turning>),
        mut drops: Query<&Body, (), Despawns>,
    ) {
        let &[n, width] = self.rain.as_slice() else { return };
        let scene = Scene::Rain { n: n as u32, width };
        for s in scene.rain(self.tick) {
            let e = if self.turning {
                let (p, v, b, c) = dynamic(&s);
                turners.spawn((p, v, b, c, Rotation::default(), Spin::default()))
            } else {
                spawner.spawn(dynamic(&s))
            };
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
                // A bounce's own gravity and substeps, as a game sets them.
                if scene.gravity() != GRAVITY {
                    let mut was = Vec::new();
                    world.for_each::<&Gravity>(|e, _| was.push(e));
                    was.into_iter().for_each(|e| world.despawn(e));
                    world.spawn((Gravity { x: 0.0, y: scene.gravity() },));
                }
                if let Some(n) = scene.substeps() {
                    world.spawn((Tuning { substeps: n },));
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
            Some(("substeps", n)) => {
                let n: u32 = n.trim().parse().map_err(|e| format!("substeps {n:?}: {e}"))?;
                let mut was = Vec::new();
                world.for_each::<&Tuning>(|e, _| was.push(e));
                was.into_iter().for_each(|e| world.despawn(e));
                world.spawn((Tuning { substeps: n },));
                Ok(format!("{n} substeps"))
            }
            None if message.trim() == "drops" => Ok(format!("drops {} tick {}", self.drops.len(), self.tick)),
            None if message.trim() == "orient" => {
                let mut bodies = Vec::new();
                world.for_each::<&Body>(|e, _| bodies.push(e));
                bodies.iter().for_each(|&e| world.insert(e, Rotation::default()));
                Ok(format!("{} bodies face a way", bodies.len()))
            }
            None if message.trim() == "turn" => {
                self.turning = true;
                let (mut bodies, mut turned, mut spinning) =
                    (Vec::new(), std::collections::HashSet::new(), std::collections::HashSet::new());
                world.for_each::<&Body>(|e, b| {
                    if b.kind == physics2d::DYNAMIC {
                        bodies.push(e)
                    }
                });
                world.for_each::<&Rotation>(|e, _| {
                    turned.insert(e);
                });
                world.for_each::<&Spin>(|e, _| {
                    spinning.insert(e);
                });
                // Those the scene started turned or spinning keep it.
                for &e in &bodies {
                    if !turned.contains(&e) {
                        world.insert(e, Rotation::default());
                    }
                    if !spinning.contains(&e) {
                        world.insert(e, Spin::default());
                    }
                }
                Ok(format!("{} bodies turn", bodies.len()))
            }
            _ => Err("commands: build <scene> | sleep default | turn | orient | substeps <n> | drops".into()),
        }
    }
}

export_mod!(Scenes);
