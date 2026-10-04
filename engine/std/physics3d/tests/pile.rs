//! 3D scenes for the physics3d mod: the comparison's (the same code,
//! `scenes.rs`, builds them in the other engines), and single bodies for
//! the tests.
//!
//!   build <scene> <n>  a scene of scenes.rs (spheres, boxes, planks, rain)
//!              of n bodies: its statics now, and its bodies as the scene
//!              adds them, each step's before that step, from `arrive`, a
//!              system; they turn unless `lock` was sent first
//!   lock       bodies from now on don't turn (no inertia)
//!   tune <variant>  set physics3d's `Tuning` (`Tuning::parse`), which
//!              the step reads from the world
//!   floor      a static floor, its top at y = 0, 100 across
//!   fixed <x> <y> <z> <hx> <hy> <hz>  a static box
//!   body sphere <r> | box <hx> <hy> <hz>, then at <x> <y> <z>, and
//!              optionally turn <ax> <ay> <az> <radians>, v <x> <y> <z>,
//!              w <x> <y> <z>, e <restitution>: a body of mass 1, solid
//!              (turning) unless locked; says its entity,
//!              `index:generation`
//!   bodies <from>  the entities `build` has added, from the from-th, in
//!              the order the scene adds them: the bench reads its bodies
//!              back in that order
//!   stats      how many bodies, how many at rest, the fastest, the deepest
//!              contact, the mean height, how many left the scene's bounds,
//!              and how many steps have added bodies
//!
//! Arrivals are a system rather than messages the bench sends each step,
//! as in the 2D comparison's scene mod: that is how a game spawns, through
//! a `Spawner` applied at the system's apply node.

// Shared with the bench, which uses more of it than the mod.
#[allow(dead_code)]
#[path = "scenes.rs"]
mod scenes;

use engine_api::{Cx, Entity, Mod, Spawner, Systems, WorldMut, export_mod, phase};
use physics3d::{
    AngularVelocity, Body, Collider, ContactPair, Gravity, Manifold, Position, Quat, Rotation, Static, Tuning, Vec3, Velocity, dynamic,
    fixed,
};
use scenes::{Kind, Scene, Shape};

/// Below this speed, at its fastest point, a body counts as at rest.
const REST: f32 = 0.05;

engine_api::mod_state! {
    #[derive(Default)]
    struct Pile {
        /// Gravity spawned: once, not again at a reload.
        started: bool,
        locked: bool,
        /// The scene `build` made, as its `Kind`'s place in `KINDS` and
        /// its n, once built: its bodies are regenerated from them (the
        /// same every time) by each build.
        kind: Vec<u8>,
        n: u64,
        /// Steps `arrive` has run since the build.
        tick: u32,
        /// What `build` has added, in the order the scene adds them.
        bodies: Vec<Entity>,
    }
}

use scenes::KINDS;

fn collider(s: Shape) -> Collider {
    match s {
        Shape::Sphere(r) => Collider::sphere(r),
        Shape::Box([x, y, z]) => Collider::cuboid(Vec3::new(x, y, z)),
    }
}

/// A moving body of mass 1, friction 0.5 and no restitution (the
/// comparison's): solid, or not turning when locked.
fn moving(at: Vec3, rot: Quat, c: Collider, locked: bool) -> Moving {
    let body = if locked { Body::new(1.0) } else { Body::solid(1.0, &c) };
    dynamic(at, rot, c, body)
}

fn vec(v: [f32; 3]) -> Vec3 {
    Vec3::new(v[0], v[1], v[2])
}

fn quat(q: [f32; 4]) -> Quat {
    Quat { v: Vec3::new(q[0], q[1], q[2]), w: q[3] }
}

/// A scene's body as the scene sets it: its turn, material and mass.
fn spec_body(b: &scenes::Spec, locked: bool) -> Moving {
    let (p, q, c, body, _, w) = moving(vec(b.pos), quat(b.rot), collider(b.shape), locked);
    let inv = 1.0 / b.mass;
    let body =
        Body { inv_mass: inv, ix: body.ix * inv, iy: body.iy * inv, iz: body.iz * inv, friction: b.friction, restitution: b.restitution };
    (p, q, c, body, Velocity { x: b.vel[0], y: b.vel[1], z: b.vel[2] }, w)
}

/// A scene's static, turned and of its material.
fn spec_static(b: &scenes::Spec) -> Still {
    let (p, q, c, body, s) = fixed(vec(b.pos), quat(b.rot), collider(b.shape));
    (p, q, c, Body { friction: b.friction, restitution: b.restitution, ..body }, s)
}

type Moving = (Position, Rotation, Collider, Body, Velocity, AngularVelocity);
type Still = (Position, Rotation, Collider, Body, Static);

impl Pile {
    fn scene(&self) -> Option<Scene> {
        let k = *self.kind.first()?;
        Some(scenes::build(KINDS[k as usize], self.n as usize))
    }

    /// The scene's bodies for this step, spawned before it.
    fn arrive(&mut self, scene: &mut Option<Scene>, _: &mut Cx, (movers, still): (Spawner<Moving>, Spawner<Still>)) {
        let Some(s) = scene else {
            return;
        };
        if let Some(batch) = s.spawn.get(self.tick as usize) {
            for b in batch {
                if b.fixed {
                    still.spawn(spec_static(b));
                    continue;
                }
                self.bodies.push(movers.spawn(spec_body(b, self.locked)));
            }
        }
        self.tick += 1;
    }

    fn build(&mut self, scene: &mut Option<Scene>, world: &mut WorldMut, what: &str) -> Result<String, String> {
        let mut words = what.split_whitespace();
        let (Some(kind), Some(n)) = (words.next(), words.next()) else {
            return Err("build <scene> <n>".into());
        };
        let kind = Kind::parse(kind).ok_or_else(|| format!("no scene {kind:?}"))?;
        let n: u64 = n.parse().map_err(|e| format!("{n:?}: {e}"))?;
        if !self.kind.is_empty() {
            return Err("a scene is built already".into());
        }
        let k = KINDS.iter().position(|&k| k == kind).expect("every kind is in KINDS");
        (self.kind, self.n, self.tick) = (vec![k as u8], n, 0);
        let s = self.scene().expect("just built");
        for b in &s.statics {
            world.spawn(spec_static(b));
        }
        // A bounce's own gravity, as a game sets it.
        if s.gravity != scenes::EARTH {
            let mut was = Vec::new();
            world.for_each::<&Gravity>(|e, _| was.push(e));
            was.into_iter().for_each(|e| world.despawn(e));
            let [x, y, z] = s.gravity;
            world.spawn((Gravity { x, y, z },));
        }
        let statics = s.statics.len();
        *scene = Some(s);
        Ok(format!("built {} {n}: {statics} statics", kind.name()))
    }

    fn body(&self, world: &mut WorldMut, what: &str) -> Result<String, String> {
        let words: Vec<&str> = what.split_whitespace().collect();
        let nums = |from: usize, n: usize| -> Result<Vec<f32>, String> {
            let w = words.get(from..from + n).ok_or_else(|| format!("{what:?}: too few numbers"))?;
            w.iter().map(|x| x.parse::<f32>().map_err(|e| format!("{x:?}: {e}"))).collect()
        };
        let (c, mut at) = match words.first() {
            Some(&"sphere") => (Collider::sphere(nums(1, 1)?[0]), 2),
            Some(&"box") => {
                let h = nums(1, 3)?;
                (Collider::cuboid(Vec3::new(h[0], h[1], h[2])), 4)
            }
            _ => return Err("body sphere <r> | box <hx> <hy> <hz> ...".into()),
        };
        let (mut pos, mut rot, mut v, mut w, mut e) = (Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO, Vec3::ZERO, 0.0);
        while at < words.len() {
            match words[at] {
                "e" => {
                    e = nums(at + 1, 1)?[0];
                    at += 2;
                }
                "at" | "v" | "w" => {
                    let x = nums(at + 1, 3)?;
                    let x = Vec3::new(x[0], x[1], x[2]);
                    match words[at] {
                        "at" => pos = x,
                        "v" => v = x,
                        _ => w = x,
                    }
                    at += 4;
                }
                "turn" => {
                    let x = nums(at + 1, 4)?;
                    rot = Quat::axis_angle(Vec3::new(x[0], x[1], x[2]).normalize(), x[3]);
                    at += 5;
                }
                other => return Err(format!("{other:?}: at, turn, v, w or e")),
            }
        }
        let (p, q, c, body, _, _) = moving(pos, rot, c, self.locked);
        let body = Body { restitution: e, ..body };
        let e = world.spawn((p, q, c, body, Velocity { x: v.x, y: v.y, z: v.z }, AngularVelocity { x: w.x, y: w.y, z: w.z }));
        Ok(format!("body {}:{}", e.index, e.generation))
    }

    fn stats(&self, scene: &Option<Scene>, world: &mut WorldMut) -> String {
        let mut reach = std::collections::HashMap::new();
        world.for_each::<&Collider>(|e, c| {
            reach.insert(e, c.reach());
        });
        let mut bodies = Vec::new();
        world.for_each::<(&Position, &Velocity, &AngularVelocity)>(|e, (p, v, w)| {
            let turning = Vec3::new(w.x, w.y, w.z).len() * reach.get(&e).copied().unwrap_or(0.0);
            bodies.push((p.at(), Vec3::new(v.x, v.y, v.z).len() + turning));
        });
        let mut deepest = 0.0f32;
        world.for_each::<(&ContactPair, &Manifold)>(|_, (_, m)| deepest = deepest.max(m.deepest()));
        let n = bodies.len().max(1) as f32;
        let resting = bodies.iter().filter(|(_, s)| *s < REST).count();
        let fastest = bodies.iter().map(|(_, s)| *s).fold(0.0, f32::max);
        let mean_y = bodies.iter().map(|(p, _)| p.y).sum::<f32>() / n;
        let escaped = scene.as_ref().map_or(0, |s| {
            let (lo, hi) = s.bounds;
            let out = |p: &Vec3| p.x < lo[0] || p.x > hi[0] || p.y < lo[1] || p.y > hi[1] || p.z < lo[2] || p.z > hi[2];
            bodies.iter().filter(|(p, _)| out(p)).count()
        });
        format!(
            "bodies {} resting {resting} fastest {fastest:.3} deepest {deepest:.4} mean_y {mean_y:.3} escaped {escaped} arrived {}",
            bodies.len(),
            self.tick
        )
    }
}

impl Mod for Pile {
    /// The scene built, regenerated from the state by each build: plain
    /// data too big to carry, and the same every time.
    type Transient = Option<Scene>;

    fn systems(s: &mut Systems<Self>) {
        // In `simulate`, which runs once per step at physics3d's rate and
        // before physics3d's own phase: a step's arrivals are there for it.
        s.add("arrive", Self::arrive).phase(phase::SIMULATE);
    }

    fn load(&mut self, scene: &mut Option<Scene>, cx: &mut Cx) {
        *scene = self.scene();
        if !self.started {
            cx.world().spawn((Gravity::EARTH,));
            self.started = true;
        }
    }

    fn message(&mut self, scene: &mut Option<Scene>, cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut world = cx.world();
        let (word, rest) = message.trim().split_once(' ').unwrap_or((message.trim(), ""));
        match word {
            "build" => self.build(scene, &mut world, rest),
            "lock" => {
                self.locked = true;
                Ok("bodies from now on don't turn".into())
            }
            "tune" => {
                let t = Tuning::parse(rest)?;
                let mut was = Vec::new();
                world.for_each::<&Tuning>(|e, _| was.push(e));
                was.into_iter().for_each(|e| world.despawn(e));
                world.spawn((t,));
                Ok(format!("tuned {t:?}"))
            }
            "floor" => {
                world.spawn(fixed(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY, Collider::cuboid(Vec3::new(50.0, 0.5, 50.0))));
                Ok("floor".into())
            }
            "fixed" => {
                let v: Vec<f32> = rest.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                let &[x, y, z, hx, hy, hz] = v.as_slice() else {
                    return Err("fixed <x> <y> <z> <hx> <hy> <hz>".into());
                };
                world.spawn(fixed(Vec3::new(x, y, z), Quat::IDENTITY, Collider::cuboid(Vec3::new(hx, hy, hz))));
                Ok("fixed".into())
            }
            "body" => self.body(&mut world, rest),
            "bodies" => {
                let from: usize = rest.trim().parse().map_err(|e| format!("{rest:?}: {e}"))?;
                let ids: Vec<String> = self.bodies.iter().skip(from).map(|e| format!("{}:{}", e.index, e.generation)).collect();
                Ok(ids.join(" "))
            }
            "stats" => Ok(self.stats(scene, &mut world)),
            _ => Err(
                "commands: build <scene> <n> | lock | tune <variant> | floor | fixed <x y z hx hy hz> | body ... | bodies <from> | stats"
                    .into(),
            ),
        }
    }
}

export_mod!(Pile);
