//! The stress demo: a walled box that bodies are dropped into.
//!
//!   drop <n>   drop n bodies, circles and boxes in turn, in rows from the floor up
//!   stats      how many bodies, how many at rest, the deepest overlap
//!              between two of them, and how many got out of the box

use engine_api::{Cx, Entity, Mod, WorldMut, export_mod};
use physics::{Body, Collider, Gravity, Placed, Position, Shape, Vec2, Velocity};

pub const WIDTH: f32 = 40.0;
pub const HEIGHT: f32 = 30.0;
const RADIUS: f32 = 0.45;
/// Below this speed a body counts as at rest.
const REST: f32 = 0.1;

engine_api::mod_state! {
    #[derive(Default)]
    struct Pile {
        walls: Vec<Entity>,
        dropped: u32,
    }
}

fn wall(world: &mut WorldMut, cx: f32, cy: f32, hx: f32, hy: f32) -> Entity {
    world.spawn((Position { x: cx, y: cy }, Collider::rect(hx, hy)))
}

impl Pile {
    fn drop_bodies(&mut self, world: &mut WorldMut, n: u32) {
        // Rows from the floor up, a little apart so each falls a little, and
        // jittered so the pile doesn't stand in perfect columns. The jitter
        // is a function of the index, so a drop is the same on every run.
        // The walls reach half the box's height above it, room for about
        // 1100 bodies.
        let per_row = ((WIDTH - 2.0) / 1.2) as u32;
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

fn stats(world: &mut WorldMut) -> String {
    let mut bodies: Vec<(Placed, f32)> = Vec::new();
    world.for_each::<(&Position, &Velocity, &Collider)>(|_, (p, v, c)| {
        bodies.push((Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y) }, Vec2::new(v.x, v.y).len()));
    });
    let resting = bodies.iter().filter(|(_, speed)| *speed < REST).count();
    let escaped = bodies
        .iter()
        .filter(|(b, _)| b.at.x < 0.0 || b.at.x > WIDTH || b.at.y < -HEIGHT / 2.0 || b.at.y > HEIGHT)
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
        // Boxes, and circles against boxes by their bounding boxes: an
        // overestimate for the circles, which only makes the check stricter.
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
        let (w, h) = (WIDTH, HEIGHT);
        self.walls = vec![
            wall(&mut world, w / 2.0, h + 0.5, w / 2.0 + 1.0, 0.5),
            wall(&mut world, -0.5, h / 2.0, 0.5, h),
            wall(&mut world, w + 0.5, h / 2.0, 0.5, h),
        ];
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut world = cx.world();
        match message.split_once(' ') {
            Some(("drop", n)) => {
                let n = n.trim().parse().map_err(|e| format!("{n:?}: {e}"))?;
                self.drop_bodies(&mut world, n);
                Ok(format!("dropped {n}"))
            }
            None if message.trim() == "stats" => Ok(stats(&mut world)),
            _ => Err("commands: drop <n> | stats".into()),
        }
    }
}

export_mod!(Pile);
