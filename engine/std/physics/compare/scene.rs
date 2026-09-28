//! The comparison's scenes, as plain data every engine builds from: the
//! physics mod (through `scene_mod.rs`, which compiles this file too), the
//! step on arrays, Box2D and Rapier. y points down, as the engine's games
//! have it. The settling scenes (piles, pyramids, stacks, rain) give every
//! dynamic body mass 1; the behaviour scenes (ramps, bounces, mass ratios,
//! overlap, bullets, structures: physics.md, "Quality beyond settling") set
//! a mass, a starting angle and spin, and a gravity scale where they need
//! one.

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
    /// Radians, as `Rot::from_angle` takes them (y down, so a positive angle
    /// turns +x toward +y, clockwise on screen): a turned static is a ramp.
    pub angle: f32,
    /// Radians a second, at the start.
    pub w: f32,
    pub mass: f32,
    pub gravity_scale: f32,
}

/// What a field a scene doesn't set is: at rest, unturned, mass 1.
pub const SPEC: Spec = Spec {
    dynamic: true,
    circle: false,
    x: 0.0,
    y: 0.0,
    hx: 0.5,
    hy: 0.5,
    vx: 0.0,
    vy: 0.0,
    friction: 0.5,
    restitution: 0.0,
    angle: 0.0,
    w: 0.0,
    mass: 1.0,
    gravity_scale: 1.0,
};

/// A static box with the engine's defaults for a collider without a
/// body (`Body::fixed()`: friction 0.5, no bounce).
fn wall(x: f32, y: f32, hx: f32, hy: f32) -> Spec {
    Spec { dynamic: false, x, y, hx, hy, ..SPEC }
}

/// A unit box resting with its bottom at `y_bottom`.
fn unit_box(x: f32, y_bottom: f32, friction: f32, mass: f32) -> Spec {
    Spec { x, y: y_bottom - 0.5, friction, mass, ..SPEC }
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
    /// Unit boxes `n` high, flush on a floor and on each other, each set off
    /// sideways by up to 0.04 so the column leans a little: whether bodies
    /// that turn stand it, or rock on their corners (Box2D's vertical stack
    /// sample, which it offsets likewise).
    Stack { n: u32 },
    /// Circles falling into the pile's box, `n / RAIN_LIFE` a step, each
    /// removed `RAIN_LIFE` steps after it came, so `n` are alive once it's
    /// full and contacts begin and end all the time. Only circles: boxes
    /// that can't turn land flush on boxes and stay, and grow towers up
    /// past where the rain starts.
    Rain { n: u32, width: f32 },
    /// A body on a static ramp tilted `deg` degrees down to the right, the
    /// same friction `mu` on both: a unit box, or a disc of radius 0.5 that
    /// turns. Below the friction angle the box holds still; above it, it
    /// slides at g (sin θ − μ cos θ); the disc rolls without slipping at
    /// (2/3) g sin θ while μ ≥ tan θ / 3.
    Ramp { deg: f32, mu: f32, circle: bool },
    /// A ball of radius 0.5 dropped from `DROP` above a floor, restitution
    /// `e`, no friction: it rebounds to e² of the height.
    Bounce { e: f32 },
    /// `light` unit boxes of mass 1 stacked on a floor, and a unit box
    /// `ratio` times as heavy on top (Box2D's "HighMassRatio1" has a heavy
    /// box on a pyramid of light ones).
    Ratio { ratio: f32, light: u32 },
    /// Box2D's "HighMassRatio2": a 20-wide box 400 times as heavy on two
    /// unit boxes 18 apart.
    BigOnSmall,
    /// Box2D's "Overlap Recovery": a pyramid of unit boxes `base` wide
    /// spawned `overlap` of a box into each other (0.5 is at half spacing),
    /// its bottom row on the floor: pushed apart at the capped speed.
    Overlap { base: u32, overlap: f32 },
    /// A ball of radius `radius`, no gravity, restitution 1 and no friction
    /// (pong's ball), fired at `speed` at a wall `thick` thick and 10 tall,
    /// from `BULLET_RUN` and `phase` of a step's travel more: where in a
    /// step it reaches the wall, which decides whether a step skips it.
    Bullet { speed: f32, radius: f32, thick: f32, phase: f32 },
    /// Box2D's "Card House" (from PEEL), `rows` storeys, scaled five times
    /// (cards 2 tall and 0.01 thick), each pair leaning `lean` degrees
    /// (Box2D's 25) and friction `mu` (0.7): at the edge of standing, so a
    /// family of them is what's judged (`family.rs`).
    Cards { rows: u32, lean: f32, mu: f32 },
    /// A plank 5 long and 0.2 thick leaning `deg` from upright against a
    /// frictionless wall, on a floor of friction `mu`: it stands while
    /// μ ≥ tan θ / 2 − hx / (2 hy) (`Scene::ladder_mu`). A structure that
    /// stands on friction alone, in place of Box2D's arch, whose blocks are
    /// wedges (polygons), which physics doesn't have.
    Ladder { deg: f32, mu: f32 },
    /// Box2D's "Double Domino": `n` dominoes 0.25 by 1, `spacing` apart
    /// (Box2D's 1), friction `mu` (0.6), the first knocked over as Box2D's
    /// impulse knocks it. One reaches the next while the spacing is under
    /// its height and thickness, 1.25.
    Dominoes { n: u32, spacing: f32, mu: f32 },
    /// `Pyramid` at friction `mu` (the pyramid's is 0.6).
    PyramidAt { base: u32, mu: f32 },
}

/// How far above the floor `Scene::Bounce` drops its ball's bottom.
pub const DROP: f32 = 5.0;
/// Where `Scene::Bullet`'s ball starts: this far left of the wall's face.
pub const BULLET_RUN: f32 = 3.0;
/// The ladder's half extents.
pub const LADDER: (f32, f32) = (0.1, 2.5);
/// The card house's scale over Box2D's sample.
const CARD_SCALE: f32 = 5.0;

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
            "pyramid" => match num(2) {
                Some(mu) => Some(Scene::PyramidAt { base: num(1)? as u32, mu }),
                None => Some(Scene::Pyramid { base: num(1)? as u32 }),
            },
            "stack" => Some(Scene::Stack { n: num(1)? as u32 }),
            "rain" => Some(Scene::Rain { n: num(1)? as u32, width: num(2)? }),
            "ramp" => Some(Scene::Ramp { deg: num(1)?, mu: num(2)?, circle: *words.get(3)? == "disc" }),
            "bounce" => Some(Scene::Bounce { e: num(1)? }),
            "ratio" => Some(Scene::Ratio { ratio: num(1)?, light: num(2)? as u32 }),
            "bigonsmall" => Some(Scene::BigOnSmall),
            "overlap" => Some(Scene::Overlap { base: num(1)? as u32, overlap: num(2)? }),
            "bullet" => Some(Scene::Bullet { speed: num(1)?, radius: num(2)?, thick: num(3)?, phase: num(4)? }),
            "cards" => Some(Scene::Cards { rows: num(1)? as u32, lean: num(2).unwrap_or(25.0), mu: num(3).unwrap_or(0.7) }),
            "ladder" => Some(Scene::Ladder { deg: num(1)?, mu: num(2)? }),
            "dominoes" => Some(Scene::Dominoes { n: num(1)? as u32, spacing: num(2).unwrap_or(1.0), mu: num(3).unwrap_or(0.6) }),
            _ => None,
        }
    }

    /// The least floor friction the ladder stands on: its weight's moment
    /// about its foot corner against the wall's push at its top corner,
    /// which only the floor's friction balances.
    pub fn ladder_mu(deg: f32) -> f32 {
        let (hx, hy) = LADDER;
        let a = deg.to_radians();
        (hy * a.sin() - hx * a.cos()) / (2.0 * hy * a.cos())
    }

    /// Whether this is one of the behaviour scenes (`behave.rs`), which
    /// the settling tables leave out.
    pub fn behaviour(&self) -> bool {
        !matches!(self, Scene::Pile { .. } | Scene::Pyramid { .. } | Scene::Stack { .. } | Scene::Rain { .. })
    }

    /// What `parse` reads.
    pub fn text(&self) -> String {
        match self {
            Scene::Pile { n, width, stagger: true } => format!("pile {n} {width}"),
            Scene::Pile { n, width, stagger: false } => format!("columns {n} {width}"),
            Scene::Pyramid { base } => format!("pyramid {base}"),
            Scene::Stack { n } => format!("stack {n}"),
            Scene::Rain { n, width } => format!("rain {n} {width}"),
            Scene::Ramp { deg, mu, circle } => format!("ramp {deg} {mu} {}", if *circle { "disc" } else { "box" }),
            Scene::Bounce { e } => format!("bounce {e}"),
            Scene::Ratio { ratio, light } => format!("ratio {ratio} {light}"),
            Scene::BigOnSmall => "bigonsmall".to_string(),
            Scene::Overlap { base, overlap } => format!("overlap {base} {overlap}"),
            Scene::Bullet { speed, radius, thick, phase } => format!("bullet {speed} {radius} {thick} {phase}"),
            Scene::Cards { rows, lean, mu } if (*lean, *mu) == (25.0, 0.7) => format!("cards {rows}"),
            Scene::Cards { rows, lean, mu } => format!("cards {rows} {lean} {mu}"),
            Scene::Ladder { deg, mu } => format!("ladder {deg} {mu}"),
            Scene::Dominoes { n, spacing, mu } if (*spacing, *mu) == (1.0, 0.6) => format!("dominoes {n}"),
            Scene::Dominoes { n, spacing, mu } => format!("dominoes {n} {spacing} {mu}"),
            Scene::PyramidAt { base, mu } => format!("pyramid {base} {mu}"),
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
                // Its floor at a collider's default friction, as the
                // pyramid was built before it had a friction of its own.
                let mut v = Scene::PyramidAt { base, mu: 0.6 }.build();
                v[0].friction = SPEC.friction;
                v
            }
            Scene::PyramidAt { base, mu } => {
                let mut v = vec![Spec { friction: mu, ..wall(0.0, 0.5, base as f32 + 10.0, 0.5) }];
                for row in 0..base {
                    let count = base - row;
                    for j in 0..count {
                        let x = j as f32 - (count - 1) as f32 / 2.0;
                        let y = -0.5 - row as f32;
                        v.push(Spec { x, y, friction: mu, ..SPEC });
                    }
                }
                v
            }
            Scene::Stack { n } => {
                let mut v = vec![wall(0.0, 0.5, 10.0, 0.5)];
                for i in 0..n {
                    let x = ((i * 7919) % 9) as f32 / 100.0 - 0.04;
                    v.push(Spec { x, y: -0.5 - i as f32, friction: 0.6, ..SPEC });
                }
                v
            }
            Scene::Rain { width, .. } => pile_walls(width),
            Scene::Ramp { deg, mu, circle } => {
                let a = deg.to_radians();
                // Down the slope, and out of its top face.
                let (t, n) = ((a.cos(), a.sin()), (a.sin(), -a.cos()));
                let ramp = Spec { dynamic: false, x: 0.0, y: 10.0, hx: 20.0, hy: 0.5, friction: mu, angle: a, ..SPEC };
                // The top face's middle, then 12 up the slope, the body's
                // bottom touching it.
                let (sx, sy) = (ramp.x + 0.5 * n.0 - 12.0 * t.0, ramp.y + 0.5 * n.1 - 12.0 * t.1);
                let body = Spec { circle, x: sx + 0.5 * n.0, y: sy + 0.5 * n.1, friction: mu, angle: if circle { 0.0 } else { a }, ..SPEC };
                vec![ramp, body]
            }
            Scene::Bounce { e } => {
                vec![wall(0.0, 0.5, 10.0, 0.5), Spec { circle: true, y: -DROP - 0.5, friction: 0.0, restitution: e, ..SPEC }]
            }
            Scene::Ratio { ratio, light } => {
                let mut v = vec![wall(0.0, 0.5, 10.0, 0.5)];
                for i in 0..light {
                    v.push(unit_box(0.0, -(i as f32), 0.6, 1.0));
                }
                v.push(unit_box(0.0, -(light as f32), 0.6, ratio));
                v
            }
            Scene::BigOnSmall => vec![
                wall(0.0, 0.5, 50.0, 0.5),
                unit_box(-9.0, 0.0, 0.6, 1.0),
                unit_box(9.0, 0.0, 0.6, 1.0),
                // 20 by 20 at the small boxes' density: 400 times as heavy.
                Spec { y: -1.0 - 10.0, hx: 10.0, hy: 10.0, friction: 0.6, mass: 400.0, ..SPEC },
            ],
            Scene::Overlap { base, overlap } => {
                // As the sample places them: rows `fraction` of a box apart
                // each way, the bottom row's bottoms on the floor.
                let mut v = vec![wall(0.0, 0.5, 40.0, 0.5)];
                let fraction = 1.0 - overlap;
                for row in 0..base {
                    let y = -0.5 - row as f32 * fraction;
                    let x0 = fraction * 0.5 * (row as f32 - base as f32);
                    for j in row..base {
                        v.push(Spec { x: x0 + (j - row) as f32 * fraction, y, friction: 0.6, ..SPEC });
                    }
                }
                v
            }
            Scene::Bullet { speed, radius, thick, phase } => vec![
                wall(thick / 2.0, 0.0, thick / 2.0, 5.0),
                Spec {
                    circle: true,
                    x: -BULLET_RUN - radius - phase * speed * DT,
                    hx: radius,
                    hy: radius,
                    vx: speed,
                    friction: 0.0,
                    restitution: 1.0,
                    gravity_scale: 0.0,
                    ..SPEC
                },
            ],
            Scene::Cards { rows, lean, mu } => cards(rows, lean, mu),
            Scene::Ladder { deg, mu } => {
                let (hx, hy) = LADDER;
                let a = deg.to_radians();
                // The wall's face at x = 0; the plank's top right corner
                // against it, its lowest corner on the floor.
                let wall_face = 0.0;
                let x = wall_face - (hx * a.cos() + hy * a.sin());
                let y = -(hx * a.sin() + hy * a.cos());
                vec![
                    Spec { friction: mu, ..wall(0.0, 0.5, 20.0, 0.5) },
                    Spec { friction: 0.0, ..wall(wall_face + 0.5, -10.0, 0.5, 10.0) },
                    Spec { x, y, hx, hy, friction: mu, angle: a, ..SPEC },
                ]
            }
            Scene::Dominoes { n, spacing, mu } => {
                let mut v = vec![Spec { friction: mu, ..wall(0.0, 1.0, 100.0, 1.0) }];
                let x0 = -0.5 * n as f32 * spacing;
                for i in 0..n {
                    let mut d = Spec { x: x0 + i as f32 * spacing, y: -0.5, hx: 0.125, hy: 0.5, friction: mu, ..SPEC };
                    if i == 0 {
                        // Box2D's impulse of 0.2 to the right at the top of
                        // a domino of mass 0.25: 0.8 a second, and turning
                        // over at r × J / I = 0.1 / 0.0221 = 4.52 a second.
                        (d.vx, d.w) = (0.8, 4.518);
                    }
                    v.push(d);
                }
                v
            }
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
            Scene::Pyramid { base } | Scene::PyramidAt { base, .. } => y > 0.0 || x.abs() > base as f32 + 10.0,
            Scene::Stack { .. } => y > 0.0 || x.abs() > 10.0,
            Scene::Ramp { .. } | Scene::Bullet { .. } => false,
            Scene::Bounce { .. } | Scene::Ratio { .. } | Scene::Overlap { .. } | Scene::Ladder { .. } => y > 0.0 || x.abs() > 10.0,
            Scene::BigOnSmall | Scene::Cards { .. } | Scene::Dominoes { .. } => y > 0.0 || x.abs() > 40.0,
        }
    }
}

/// Box2D's card house (`sample_stacking.cpp`, "Card House", from PEEL),
/// y turned down and every length five times: pairs of cards leaning 25°
/// into each other, a flat card across each two pairs, `rows` storeys.
fn cards(rows: u32, lean: f32, mu: f32) -> Vec<Spec> {
    let s = CARD_SCALE;
    let (height, thick) = (0.2 * s, 0.001 * s);
    let card = |x: f32, y: f32, angle: f32| Spec { x, y: -y, hx: thick, hy: height, friction: mu, angle, ..SPEC };
    // The sample's floor is a box 2 deep under y = 0, friction 0.7.
    let mut v = vec![Spec { friction: mu, ..wall(0.0, 2.0 * s, 40.0 * s, 2.0 * s) }];
    let lean = lean.to_radians();
    let (mut z0, mut y, mut nb) = (0.0, height - 0.02 * s, rows);
    while nb > 0 {
        let mut z = z0;
        for i in 0..nb {
            if i != nb - 1 {
                v.push(card(z + 0.25 * s, y + height - 0.015 * s, std::f32::consts::FRAC_PI_2));
            }
            // y up there, down here: the sample's angles change sign.
            v.push(card(z, y, lean));
            z += 0.175 * s;
            v.push(card(z, y, -lean));
            z += 0.175 * s;
        }
        y += 2.0 * height - 0.03 * s;
        z0 += 0.175 * s;
        nb -= 1;
    }
    v
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
    Spec { circle, x, y, hx: RADIUS, hy: RADIUS, vy, friction: 0.4, restitution: 0.1, ..SPEC }
}
