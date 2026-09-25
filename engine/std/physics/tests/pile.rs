//! The stress demo: a walled box that bodies are dropped into.
//!
//!   widen <w>  rebuild the box's walls w wide, for piles bigger than ~1100
//!   drop <n>   drop n bodies, circles and boxes in turn, in rows from the floor up
//!   sleep <speed> <time>  turn on sleeping (see `physics::Sleep`), which
//!              the pile has off, for the benchmarks
//!   sleep off  turn it off again
//!   sleep default  leave it to physics's default, which is on
//!   kick <vx> <vy> [newest | naps]  set the velocity of the first body
//!              dropped (or one of the island last to fall asleep, or the
//!              bodies `nap` spawned), as a game would a sleeping body's
//!   despawn    despawn the first body dropped, at the bottom
//!   despawn nap  that, and spawn a body asleep (with `Asleep`) above the box
//!   nap later  spawn an entity asleep above the box that isn't a body yet
//!   nap body   make those bodies
//!   unsleep    remove the first body's `Asleep`, as a game waking it would
//!   resleep    give it one again, as a game putting it to sleep would
//!   post       spawn a static body with a velocity above the box, which the
//!              solve never writes
//!   resleep post  give those an `Asleep`
//!   grow <h>   make the first body dropped a box h across each way
//!   floor off  despawn the floor
//!   floor falls  make the floor a body, which falls
//!   floor swap despawn the floor and spawn a static far above the box
//!   floor <dy> move the floor down by dy
//!   touching   give every body a `Touching`
//!   sensing    make every body sense the others: an `Overlap` each
//!   block <x> <y>  put a static unit box at x, y, or move it there
//!   pusher <x> <y> <vx> <vy>  a kinematic box at x, y moving at vx, vy
//!   stats      how many bodies, how many at rest, the deepest overlap
//!              between two of them, and how many got out of the box
//!   shelves [sensing]  two shelves across the box at `SHELF`, one a body with no
//!              velocity and one a velocity with no body, and a row of
//!              bodies dropped on each: colliders physics gathers apart
//!   lift <vy>  give the shelf that has no velocity one, up at vy

use engine_api::{Cx, Entity, Mod, WorldMut, export_mod};
use physics::{Asleep, Body, Collider, Gravity, Placed, Position, Shape, Sleep, Touching, Vec2, Velocity};

pub const WIDTH: f32 = 40.0;
pub const HEIGHT: f32 = 30.0;
/// Where `shelves` puts its shelves' tops.
pub const SHELF: f32 = 10.0;
const RADIUS: f32 = 0.45;
/// Below this speed a body counts as at rest.
const REST: f32 = 0.1;
/// Where `despawn nap` spawns its sleeping body, above the box and clear
/// of it, and the island it names: far from physics's own numbering.
pub const NAP: f32 = -5.0;
const NAP_ISLAND: u32 = 1 << 30;

engine_api::mod_state! {
    #[derive(Default)]
    struct Pile {
        walls: Vec<Entity>,
        dropped: u32,
        /// 0 until widened: `WIDTH`.
        width: f32,
        /// The static box `block` places, once it has.
        block: Vec<Entity>,
        /// The shelves `shelves` made: the kinematic one first.
        shelves: Vec<Entity>,
        /// What `despawn nap` and `nap later` spawned.
        naps: Vec<Entity>,
        later: Vec<Entity>,
        /// What `post` spawned.
        posts: Vec<Entity>,
    }
}

/// Replaces the `Sleep` entity with `to`, or with none: physics's default.
fn set_sleep(world: &mut WorldMut, to: Option<Sleep>) {
    let mut was = Vec::new();
    world.for_each::<&Sleep>(|e, _| was.push(e));
    was.into_iter().for_each(|e| world.despawn(e));
    if let Some(to) = to {
        world.spawn((to,));
    }
}

/// The first body dropped: the least entity with a body and a velocity.
fn first_body(world: &mut WorldMut) -> Option<Entity> {
    let mut first = None;
    world.for_each::<(&Velocity, &Body)>(|e, _| first = Some(first.map_or(e, |f: Entity| f.min(e))));
    first
}

fn wall(world: &mut WorldMut, cx: f32, cy: f32, hx: f32, hy: f32) -> Entity {
    world.spawn((Position { x: cx, y: cy }, Collider::rect(hx, hy)))
}

impl Pile {
    fn width(&self) -> f32 {
        if self.width > 0.0 { self.width } else { WIDTH }
    }

    fn build_walls(&mut self, world: &mut WorldMut) {
        let (w, h) = (self.width(), HEIGHT);
        self.walls = vec![
            wall(world, w / 2.0, h + 0.5, w / 2.0 + 1.0, 0.5),
            wall(world, -0.5, h / 2.0, 0.5, h),
            wall(world, w + 0.5, h / 2.0, 0.5, h),
        ];
    }

    fn drop_bodies(&mut self, world: &mut WorldMut, n: u32) {
        // Rows from the floor up, a little apart so each falls a little, and
        // jittered so the pile doesn't stand in perfect columns. The jitter
        // is a function of the index, so a drop is the same on every run.
        // The walls reach half the box's height above it, room for about
        // 1100 bodies.
        let per_row = ((self.width() - 2.0) / 1.2) as u32;
        for k in self.dropped..self.dropped + n {
            let (col, row) = (k % per_row, k / per_row);
            let jitter = ((k * 7919) % 100) as f32 / 100.0 * 0.2 - 0.1;
            let at = Position { x: 1.5 + col as f32 * 1.2 + jitter, y: HEIGHT - 1.0 - row as f32 * 1.2 };
            let collider = if k % 2 == 0 { Collider::circle(RADIUS) } else { Collider::rect(RADIUS, RADIUS) };
            let body = Body { friction: 0.4, restitution: 0.1, ..Body::default() };
            world.spawn((at, Velocity::default(), body, collider));
        }
        self.dropped += n;
    }
}

fn stats(world: &mut WorldMut, width: f32) -> String {
    let mut bodies: Vec<(Placed, f32)> = Vec::new();
    world.for_each::<(&Position, &Velocity, &Collider)>(|_, (p, v, c)| {
        bodies.push((Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y) }, Vec2::new(v.x, v.y).len()));
    });
    let resting = bodies.iter().filter(|(_, speed)| *speed < REST).count();
    let escaped = bodies
        .iter()
        .filter(|(b, _)| b.at.x < 0.0 || b.at.x > width || b.at.y < -HEIGHT / 2.0 || b.at.y > HEIGHT)
        .count();
    // Quadratic, and fine for a message.
    let mut deepest = 0.0f32;
    for (i, (a, _)) in bodies.iter().enumerate() {
        for (b, _) in &bodies[i + 1..] {
            deepest = deepest.max(depth(a, b));
        }
    }
    let fastest = bodies.iter().map(|(_, speed)| *speed).fold(0.0, f32::max);
    let mean = bodies.iter().map(|(_, speed)| *speed).sum::<f32>() / bodies.len().max(1) as f32;
    format!(
        "bodies {} resting {resting} deepest {deepest:.3} escaped {escaped} fastest {fastest:.3} mean {mean:.3}",
        bodies.len()
    )
}

/// How far two of the pile's shapes overlap, or 0.
fn depth(a: &Placed, b: &Placed) -> f32 {
    let d = b.at - a.at;
    let (ha, hb) = (a.shape.half_extents(), b.shape.half_extents());
    match (a.shape, b.shape) {
        (Shape::Circle(ra), Shape::Circle(rb)) => (ra + rb - d.len()).max(0.0),
        // A circle against a box by the box's point nearest its center: by
        // the circle's bounding box, a circle resting on a box's corner (as
        // they do in a real pile, not in columns) read as 0.12 deep.
        (Shape::Circle(r), Shape::Box(h)) | (Shape::Box(h), Shape::Circle(r)) => {
            let d = d.abs();
            let outside = Vec2::new((d.x - h.x).max(0.0), (d.y - h.y).max(0.0));
            if outside == Vec2::ZERO { r + (h.x - d.x).min(h.y - d.y) } else { (r - outside.len()).max(0.0) }
        }
        _ => (ha.x + hb.x - d.x.abs()).min(ha.y + hb.y - d.y.abs()).max(0.0),
    }
}

impl Mod for Pile {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        if !self.walls.is_empty() {
            return;
        }
        let mut world = cx.world();
        world.spawn((Gravity { x: 0.0, y: 20.0 },));
        // The benchmarks' scene, which `:tax` checks bit for bit against a
        // step that doesn't sleep: off unless asked.
        world.spawn((Sleep::OFF,));
        self.build_walls(&mut world);
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut world = cx.world();
        match message.split_once(' ') {
            Some(("widen", w)) => {
                self.width = w.trim().parse().map_err(|e| format!("{w:?}: {e}"))?;
                for e in std::mem::take(&mut self.walls) {
                    world.despawn(e);
                }
                self.build_walls(&mut world);
                Ok(format!("{} wide", self.width))
            }
            Some(("drop", n)) => {
                let n = n.trim().parse().map_err(|e| format!("{n:?}: {e}"))?;
                self.drop_bodies(&mut world, n);
                Ok(format!("dropped {n}"))
            }
            Some(("sleep", "off")) => {
                set_sleep(&mut world, Some(Sleep::OFF));
                Ok("sleeping off".into())
            }
            Some(("sleep", "default")) => {
                set_sleep(&mut world, None);
                Ok("sleeping by default".into())
            }
            Some(("grow", h)) => {
                let h: f32 = h.trim().parse().map_err(|e| format!("{h:?}: {e}"))?;
                let mut first = None;
                world.for_each::<(&Velocity, &Body)>(|e, _| first = Some(first.map_or(e, |f: Entity| f.min(e))));
                world.with_mut::<Collider, _>(first.ok_or("nothing to grow")?, |c| (c.hx, c.hy) = (h, h));
                Ok("grown".into())
            }
            Some(("kick", args)) => {
                let mut args = args.split_whitespace().map(|a| a.parse::<f32>().map_err(|e| format!("{a:?}: {e}")));
                let (Some(x), Some(y)) = (args.next(), args.next()) else { return Err("kick <vx> <vy> [newest]".into()) };
                let (x, y) = (x?, y?);
                let kicked = if message.ends_with("naps") {
                    for &e in &self.naps {
                        world.with_mut::<Velocity, _>(e, |v| *v = Velocity { x, y });
                    }
                    return Ok("kicked the naps".into());
                } else if message.ends_with("newest") {
                    let mut newest: Option<(u32, Entity)> = None;
                    world.for_each::<(&Asleep, &Velocity)>(|e, (a, _)| newest = newest.max(Some((a.island, e))));
                    newest.map(|(_, e)| e)
                } else {
                    first_body(&mut world)
                };
                let kicked = kicked.ok_or("nothing to kick")?;
                world.with_mut::<Velocity, _>(kicked, |v| *v = Velocity { x, y });
                Ok(format!("kicked {kicked:?}"))
            }
            None if message.trim() == "despawn" => {
                let mut first = None;
                world.for_each::<(&Velocity, &Body)>(|e, _| first = Some(first.map_or(e, |f: Entity| f.min(e))));
                world.despawn(first.ok_or("nothing to despawn")?);
                Ok("despawned".into())
            }
            // The same step (the same message) as something else, so a check
            // that counts can't see either.
            Some(("despawn", "nap")) => {
                let first = first_body(&mut world).ok_or("nothing to despawn")?;
                world.despawn(first);
                let body = Body { friction: 0.4, restitution: 0.1, ..Body::default() };
                let at = Position { x: self.width() / 2.0, y: NAP };
                self.naps.push(world.spawn((at, Velocity::default(), body, Collider::circle(RADIUS), Asleep { island: NAP_ISLAND })));
                Ok("despawned, and napping".into())
            }
            Some(("nap", "later")) => {
                let at = Position { x: self.width() / 2.0 + 3.0, y: NAP };
                let later = world.spawn((at, Collider::circle(RADIUS), Asleep { island: NAP_ISLAND + 1 }));
                self.naps.push(later);
                self.later.push(later);
                Ok("napping, not yet a body".into())
            }
            Some(("nap", "body")) => {
                let body = Body { friction: 0.4, restitution: 0.1, ..Body::default() };
                for &e in &self.later {
                    world.insert(e, Velocity::default());
                    world.insert(e, body);
                }
                Ok("napping bodies".into())
            }
            None if message.trim() == "unsleep" => {
                let first = first_body(&mut world).ok_or("nothing to wake")?;
                world.remove::<Asleep>(first);
                Ok(format!("woke {first:?}"))
            }
            None if message.trim() == "resleep" => {
                let first = first_body(&mut world).ok_or("nothing to put to sleep")?;
                world.insert(first, Asleep { island: NAP_ISLAND + 2 });
                Ok(format!("put {first:?} to sleep"))
            }
            // A body the solve never writes (a static one with a velocity),
            // so what's new about it once it's given `Asleep` is only that.
            None if message.trim() == "post" => {
                let at = Position { x: self.width() / 2.0 - 3.0, y: NAP };
                self.posts.push(world.spawn((at, Velocity::default(), Body::fixed(), Collider::rect(RADIUS, RADIUS))));
                Ok("post".into())
            }
            Some(("resleep", "post")) => {
                for &e in &self.posts {
                    world.insert(e, Asleep { island: NAP_ISLAND + 3 });
                }
                Ok("posts put to sleep".into())
            }
            Some(("floor", "swap")) => {
                world.despawn(self.walls[0]);
                self.walls[0] = wall(&mut world, self.width() / 2.0, -HEIGHT, 1.0, 0.5);
                Ok("floor swapped".into())
            }
            Some(("lift", vy)) => {
                let vy: f32 = vy.trim().parse().map_err(|e| format!("{vy:?}: {e}"))?;
                let &shelf = self.shelves.first().ok_or("no shelves")?;
                world.insert(shelf, Velocity { x: 0.0, y: -vy });
                Ok("lifting".into())
            }
            Some(("floor", "falls")) => {
                world.insert(self.walls[0], Velocity::default());
                world.insert(self.walls[0], Body { gravity_scale: 1.0, ..Body::default() });
                Ok("floor falls".into())
            }
            Some(("floor", "off")) => {
                world.despawn(self.walls[0]);
                Ok("floor off".into())
            }
            Some(("floor", dy)) => {
                let dy: f32 = dy.trim().parse().map_err(|e| format!("{dy:?}: {e}"))?;
                world.with_mut::<Position, _>(self.walls[0], |p| p.y += dy);
                Ok("floor moved".into())
            }
            Some(("block", args)) => {
                let mut args = args.split_whitespace().map(|a| a.parse::<f32>().map_err(|e| format!("{a:?}: {e}")));
                let (Some(x), Some(y)) = (args.next(), args.next()) else { return Err("block <x> <y>".into()) };
                let at = Position { x: x?, y: y? };
                match self.block.first() {
                    Some(&b) => world.with_mut::<Position, _>(b, |p| *p = at).ok_or("the block is gone")?,
                    None => self.block.push(world.spawn((at, Collider::rect(0.5, 0.5)))),
                }
                Ok("block".into())
            }
            Some(("pusher", args)) => {
                let args: Result<Vec<f32>, String> = args.split_whitespace().map(|a| a.parse::<f32>().map_err(|e| format!("{a:?}: {e}"))).collect();
                let &[x, y, vx, vy] = args?.as_slice() else { return Err("pusher <x> <y> <vx> <vy>".into()) };
                world.spawn((Position { x, y }, Velocity { x: vx, y: vy }, Body::kinematic(), Collider::rect(2.0, 0.5)));
                Ok("pusher".into())
            }
            None if message.trim() == "sensing" => {
                let mut bodies = Vec::new();
                world.for_each::<(&Velocity, &Collider)>(|e, _| bodies.push(e));
                bodies.into_iter().for_each(|e| {
                    world.with_mut::<Collider, _>(e, |c| c.senses = 1);
                });
                Ok("sensing".into())
            }
            None if message.trim() == "touching" => {
                let mut bodies = Vec::new();
                world.for_each::<&Velocity>(|e, _| bodies.push(e));
                bodies.into_iter().for_each(|e| world.insert(e, Touching::default()));
                Ok("touching".into())
            }
            Some(("sleep", args)) => {
                let mut args = args.split_whitespace().map(|a| a.parse::<f32>().map_err(|e| format!("{a:?}: {e}")));
                let (Some(speed), Some(time)) = (args.next(), args.next()) else { return Err("sleep <speed> <time>".into()) };
                set_sleep(&mut world, Some(Sleep { speed: speed?, time: time? }));
                Ok("sleeping on".into())
            }
            None if message.trim() == "stats" => Ok(stats(&mut world, self.width())),
            None | Some(("shelves", "sensing")) if message.starts_with("shelves") => {
                // Sensing, the bodies' every overlap with the shelves (and
                // each other) is an `Overlap` too.
                let senses = if message.ends_with("sensing") { 1 } else { 0 };
                let w = self.width();
                let half = Collider::rect(w / 4.0 - 0.5, 0.25);
                let (left, right) = (Position { x: w / 4.0, y: SHELF + 0.25 }, Position { x: w * 0.75, y: SHELF + 0.25 });
                self.shelves = vec![world.spawn((left, half, Body::kinematic())), world.spawn((right, half, Velocity::default()))];
                let body = Body { friction: 0.4, restitution: 0.1, ..Body::default() };
                for k in 0..(w as u32 - 2) {
                    let at = Position { x: 1.5 + k as f32, y: SHELF - 1.0 };
                    world.spawn((at, Velocity::default(), body, Collider::rect(RADIUS, RADIUS).sensing(senses)));
                }
                Ok("shelved".into())
            }
            _ => Err("commands: widen <w> | drop <n> | sleep <speed> <time> | sleep off | sleep default | kick <vx> <vy> [newest | naps] | grow <h> | despawn [nap] | nap later | nap body | unsleep | resleep [post] | post | floor off | floor falls | floor swap | floor <dy> | lift <vy> | touching | sensing | block <x> <y> | pusher <x> <y> <vx> <vy> | stats | shelves [sensing]".into()),
        }
    }
}

export_mod!(Pile);
