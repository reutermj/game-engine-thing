//! The meet families (get-lye): pong's ball fast into pong's paddle, a
//! kinematic box coming toward it (`paddle`), or a static one (`wall`),
//! over a grid of speeds, angles, the paddle's speeds and where in a step
//! they meet (`Scene::Meet`). The playtest loop found pong's ball inside a
//! paddle by up to half a cell at 35 to 40 cells a second, with no contact
//! found yet at its deepest frame, and inside the walls by up to 0.42
//! (docs/architecture/presentation-spike.md, "The playtest loop"). Each run
//! is measured from positions alone, as `behave.rs` measures, the paddle's
//! from its velocity (`Meet::paddle_at`), so every engine is judged by the
//! same code; whether the engine held a contact is from its own contacts
//! (`Sim::marks`). `MEETS=<names>` (`all`) in the comparison measures every
//! engine on them, `MEET_LONG=1` the long grids, `MEET_RUNS=1` every run.

use crate::behave::Behaviour;
use crate::family::Family;
use crate::scene::{DT, Meet, PADDLE, Scene};
use crate::{Dyn, Sim, quality};

/// Steps a run lasts: to the meeting (3 and a phase) and well past it.
pub const STEPS: u32 = 20;

/// An overlap past this is sinking in: more than a resting contact's slop
/// (the solver's, 0.005), less than any step of these speeds covers.
pub const SLOP: f32 = 0.01;

/// Every meet family, on its short grid (the default suite's) or its long
/// one.
pub fn families(long: bool) -> Vec<Family> {
    let pick = |short: &'static [f32], full: &'static [f32]| if long { full } else { short };
    let speeds = pick(&[20.0, 40.0, 56.6], &[10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 48.0, 56.6]);
    let phases = pick(&[0.0, 0.5], &[0.0, 0.25, 0.5, 0.75]);
    let mut paddle = Vec::new();
    for &speed in speeds {
        for &deg in pick(&[0.0, 30.0, 45.0], &[0.0, 15.0, 30.0, 45.0, 60.0]) {
            for &toward in pick(&[8.0, 16.0], &[4.0, 8.0, 16.0, 24.0]) {
                for &slide in pick(&[0.0, 16.0], &[0.0, 8.0, 16.0]) {
                    for &phase in phases {
                        paddle.push(Scene::Meet(Meet { speed, deg, paddle: toward, slide, phase, bullet: false }));
                    }
                }
            }
        }
    }
    let mut wall = Vec::new();
    for &speed in speeds {
        for &deg in pick(&[0.0, 30.0, 60.0], &[0.0, 15.0, 30.0, 45.0, 60.0, 75.0]) {
            for &phase in phases {
                wall.push(Scene::Meet(Meet { speed, deg, paddle: 0.0, slide: 0.0, phase, bullet: false }));
            }
        }
    }
    // The paddle's grid with the ball a bullet (`Body::bullet`), as a game
    // that wants it stopped by moving bodies too would make it.
    let bullet = paddle.iter().map(|s| if let Scene::Meet(m) = *s { Scene::Meet(Meet { bullet: true, ..m }) } else { *s }).collect();
    vec![Family { name: "paddle", scenes: paddle }, Family { name: "wall", scenes: wall }, Family { name: "bullet", scenes: bullet }]
}

/// One meeting, stepped `STEPS` and read every step:
/// - `deepest`: how far the ball got inside the paddle's box;
/// - `held`: 1 if the engine held a contact for the pair in the step the
///   ball first sank past `SLOP`, or it never did; 0 if it sank in with no
///   contact found, what the playtest loop saw;
/// - `rebound`: the ball's speed away from the paddle along the normal at
///   the end, over its speed toward it (restitution 1: 1 is right);
/// - `through`: 1 if it ended behind the paddle's face.
pub fn meet(sim: &mut dyn Sim, m: &Meet) -> Behaviour {
    let mut b = Behaviour { label: sim.label(), values: Vec::new() };
    let (mut deepest, mut held) = (0.0f32, None);
    let mut ball = sim.bodies()[0];
    for k in 1..=STEPS {
        sim.step(1);
        ball = sim.bodies()[0];
        let (x, y) = m.paddle_at(k as f32 * DT);
        let paddle = Dyn { circle: false, hx: PADDLE.0, hy: PADDLE.1, x, y, vx: 0.0, vy: 0.0, angle: 0.0, w: 0.0 };
        let depth = -quality::gap(&ball, &paddle);
        deepest = deepest.max(depth);
        if depth > SLOP && held.is_none() {
            held = Some(!sim.marks().is_empty());
        }
    }
    let toward = m.speed * m.deg.to_radians().cos() + m.paddle;
    let (centre, _) = m.paddle_at(STEPS as f32 * DT);
    b.put("deepest", deepest as f64);
    b.put("held", held.unwrap_or(true) as u8 as f64);
    b.put("rebound", (-(ball.vx + m.paddle) / toward) as f64);
    // Past the paddle's middle, a push out goes out of its back.
    b.put("through", (ball.x > centre) as u8 as f64);
    b
}

/// A family's statistics over its runs: the deepest of any run and the
/// median run's, how many held a contact as they sank in (or never sank),
/// how many went through, and the least and greatest rebound.
pub fn stats(f: &Family, runs: &[Behaviour]) -> Vec<(&'static str, f64)> {
    let all = |k: &str| runs.iter().map(|r| r.get(k)).collect::<Vec<f64>>();
    let mut deep = all("deepest");
    deep.sort_by(f64::total_cmp);
    let rebound = all("rebound");
    let fold = |init: f64, g: fn(f64, f64) -> f64| rebound.iter().copied().fold(init, g);
    assert_eq!(runs.len(), f.scenes.len());
    vec![
        ("deepest", deep[deep.len() - 1]),
        ("median deepest", deep[deep.len() / 2]),
        ("sank past slop", deep.iter().filter(|&&d| d > SLOP as f64).count() as f64),
        ("held", all("held").iter().sum()),
        ("through", all("through").iter().sum()),
        ("least rebound", fold(f64::INFINITY, f64::min)),
        ("most rebound", fold(0.0, f64::max)),
    ]
}
