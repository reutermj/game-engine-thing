//! The comparison's scenes, as plain data every engine builds from: the
//! physics mod (through `scene_mod.rs`, which compiles this file too), the
//! step on arrays, Box2D and Rapier. y points down, as the engine's games
//! have it, and every dynamic body has mass 1 and its rotation locked,
//! since //engine/std/physics has no rotation.

/// Down, in units per second squared: the pile's.
pub const GRAVITY: f32 = 20.0;
pub const DT: f32 = 1.0 / 60.0;

/// One body. Statics (walls, floors) first in every scene, so that in the
/// engine, where entities are numbered as spawned, contacts sort the same
/// as on arrays.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spec {
    pub dynamic: bool,
    pub circle: bool,
    pub x: f32,
    pub y: f32,
    /// Half extents; a circle's radius is `hx`.
    pub hx: f32,
    pub hy: f32,
    pub vx: f32,
    pub vy: f32,
    pub friction: f32,
    pub restitution: f32,
}

/// A static box with the engine's defaults for a collider without a
/// body (`Body::fixed()`: friction 0.5, no bounce).
fn wall(x: f32, y: f32, hx: f32, hy: f32) -> Spec {
    Spec { dynamic: false, circle: false, x, y, hx, hy, vx: 0.0, vy: 0.0, friction: 0.5, restitution: 0.0 }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scene {
    /// `n` bodies dropped in rows into a box `width` wide, as the pile of
    /// //engine/std/physics:pile drops them, with every other row shifted
    /// half a body over when `stagger`, so each body lands between two.
    /// Unstaggered it is that pile body for body, which at 41 wide our
    /// engine turns into a pile but Box2D and Rapier leave standing in
    /// columns, one contact a body (friction holds a circle on a circle at
    /// the jitter'''s angles; see physics.md, "Against other engines").
    /// Staggered, it is a real pile in every engine.
    Pile { n: u32, width: f32, stagger: bool },
    /// Unit boxes in a pyramid `base` wide at the bottom, resting flush on
    /// a floor and on each other, as Box2D's pyramid benchmark stands.
    Pyramid { base: u32 },
    /// Circles falling into the pile's box, `n / RAIN_LIFE` a step, each
    /// removed `RAIN_LIFE` steps after it came, so `n` are alive once it's
    /// full and contacts begin and end all the time. Only circles: boxes
    /// that can't turn land flush on boxes and stay, and grow towers up
    /// past where the rain starts.
    Rain { n: u32, width: f32 },
}

pub const HEIGHT: f32 = 30.0;
const RADIUS: f32 = 0.45;
/// Steps a raindrop lives: 8 s, long enough to land (about 1.3 s) and be
/// buried.
pub const RAIN_LIFE: u32 = 480;
/// Where rain appears: inside the box (its walls reach up to -15), above
/// where the heap tops out.
const RAIN_Y: f32 = -10.0;

impl Scene {
    pub fn parse(text: &str) -> Option<Scene> {
        let words: Vec<&str> = text.split_whitespace().collect();
        let num = |i: usize| words.get(i).and_then(|w| w.parse::<f32>().ok());
        match *words.first()? {
            "pile" => Some(Scene::Pile { n: num(1)? as u32, width: num(2)?, stagger: true }),
            "columns" => Some(Scene::Pile { n: num(1)? as u32, width: num(2)?, stagger: false }),
            "pyramid" => Some(Scene::Pyramid { base: num(1)? as u32 }),
            "rain" => Some(Scene::Rain { n: num(1)? as u32, width: num(2)? }),
            _ => None,
        }
    }

    /// What `parse` reads.
    pub fn text(&self) -> String {
        match self {
            Scene::Pile { n, width, stagger: true } => format!("pile {n} {width}"),
            Scene::Pile { n, width, stagger: false } => format!("columns {n} {width}"),
            Scene::Pyramid { base } => format!("pyramid {base}"),
            Scene::Rain { n, width } => format!("rain {n} {width}"),
        }
    }

    /// The bodies at step 0.
    pub fn build(&self) -> Vec<Spec> {
        match *self {
            Scene::Pile { n, width, stagger } => {
                let mut v = pile_walls(width);
                // As tests/pile.rs drops them: rows from the floor up, a
                // little apart, jittered by index so no run differs.
                let per_row = ((width - 2.0) / 1.2) as u32;
                for k in 0..n {
                    let (col, row) = (k % per_row, k / per_row);
                    let jitter = ((k * 7919) % 100) as f32 / 100.0 * 0.2 - 0.1;
                    let shift = if stagger && row % 2 == 1 { 0.6 } else { 0.0 };
                    v.push(drop(k % 2 == 0, 1.5 + col as f32 * 1.2 + jitter + shift, HEIGHT - 1.0 - row as f32 * 1.2, 0.0));
                }
                v
            }
            Scene::Pyramid { base } => {
                let mut v = vec![wall(0.0, 0.5, base as f32 + 10.0, 0.5)];
                for row in 0..base {
                    let count = base - row;
                    for j in 0..count {
                        let x = j as f32 - (count - 1) as f32 / 2.0;
                        let y = -0.5 - row as f32;
                        v.push(Spec {
                            dynamic: true,
                            circle: false,
                            x,
                            y,
                            hx: 0.5,
                            hy: 0.5,
                            vx: 0.0,
                            vy: 0.0,
                            friction: 0.6,
                            restitution: 0.0,
                        });
                    }
                }
                v
            }
            Scene::Rain { width, .. } => pile_walls(width),
        }
    }

    /// Bodies at the start of each rain step: how many come a step.
    pub fn rain_rate(&self) -> u32 {
        match *self {
            Scene::Rain { n, .. } => (n / RAIN_LIFE).max(1),
            _ => 0,
        }
    }

    /// The bodies that arrive before step `tick`. Each step's come in
    /// slots across the box, shifted a third of a slot each step, so a slot
    /// is reused only every third step, by when the last drop in it has
    /// fallen more than a body's height (they start at 20 a second down).
    pub fn rain(&self, tick: u32) -> Vec<Spec> {
        let Scene::Rain { width, .. } = *self else { return Vec::new() };
        let k = self.rain_rate();
        let slot = (width - 3.0) / k as f32;
        (0..k)
            .map(|i| {
                let jitter = (((tick * 7919 + i * 104_729) % 100) as f32 / 100.0 - 0.5) * 0.2;
                let x = 1.5 + (i as f32 + (tick % 3) as f32 / 3.0) * slot + jitter;
                drop(true, x, RAIN_Y, 20.0)
            })
            .collect()
    }

    /// Whether a body at `x, y` has left the scene (through a wall, or
    /// fallen off the pyramid's floor).
    pub fn escaped(&self, x: f32, y: f32) -> bool {
        match *self {
            Scene::Pile { width, .. } | Scene::Rain { width, .. } => x < 0.0 || x > width || y > HEIGHT || y < -HEIGHT / 2.0,
            Scene::Pyramid { base } => y > 0.0 || x.abs() > base as f32 + 10.0,
        }
    }
}

fn pile_walls(width: f32) -> Vec<Spec> {
    vec![
        wall(width / 2.0, HEIGHT + 0.5, width / 2.0 + 1.0, 0.5),
        wall(-0.5, HEIGHT / 2.0, 0.5, HEIGHT),
        wall(width + 0.5, HEIGHT / 2.0, 0.5, HEIGHT),
    ]
}

/// A pile body: a circle or a box 0.9 across, friction 0.4, restitution
/// 0.1, as tests/pile.rs makes them.
fn drop(circle: bool, x: f32, y: f32, vy: f32) -> Spec {
    Spec { dynamic: true, circle, x, y, hx: RADIUS, hy: RADIUS, vx: 0.0, vy, friction: 0.4, restitution: 0.1 }
}
