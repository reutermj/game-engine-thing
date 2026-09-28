//! Scenes at the edge of stability, judged as families: a grid of one
//! scene across the parameter that decides it (a card house's lean and
//! friction, a ramp's angle about the friction angle, a stack's height),
//! each run a yes or a no (`good`), and the family by the share of yeses.
//! Near its edge a scene goes either way on rounding, in every engine: the
//! 5-storey card house stands or loses two cards as the warm start's last
//! bits change (get-emj.61). So, as chaotic piles are judged by medians
//! over sizes, these are judged by a share over a grid, against the share
//! the references meet on exactly that grid (physics-testing.md, "Families
//! at the edge"). `:behaviour_test` runs the short grids,
//! `:behaviour_long_test` the long ones; `FAMILIES=<names>` (`all`) in the
//! comparison measures every engine on them (`FAMILY_LONG=1`, the long).

use crate::behave::Behaviour;
use crate::scene::{DT, Scene};

/// One family: its grid, short or long.
pub struct Family {
    pub name: &'static str,
    pub scenes: Vec<Scene>,
}

/// Every family, on its short grid (the default suite's) or its long one.
pub fn families(long: bool) -> Vec<Family> {
    let pick = |short: Vec<Scene>, full: Vec<Scene>| if long { full } else { short };
    vec![
        Family {
            name: "cards",
            scenes: pick(cards(&[4, 5], &[24.0, 25.0, 26.0], &[0.7, 0.8]), cards(&[4, 5, 6], &LEANS, &[0.65, 0.7, 0.75, 0.8, 0.9])),
        },
        Family {
            name: "ramp",
            scenes: pick(
                ramps(&[0.5], &[-0.25, -0.1, 0.1, 0.25]),
                ramps(&[0.3, 0.5, 0.7], &[-2.0, -1.0, -0.5, -0.25, -0.1, 0.1, 0.25, 0.5, 1.0, 2.0]),
            ),
        },
        Family {
            name: "ladder",
            scenes: pick(
                ladders(&[30.0], &[-0.01, -0.005, 0.005, 0.01]),
                ladders(&[20.0, 30.0, 40.0], &[-0.05, -0.02, -0.01, -0.005, 0.005, 0.01, 0.02, 0.05]),
            ),
        },
        Family {
            name: "dominoes",
            scenes: pick(dominoes(&[1.0, 1.1, 1.2, 1.3], &[0.6]), dominoes(&[0.8, 0.9, 1.0, 1.1, 1.15, 1.2, 1.3, 1.4], &[0.3, 0.6])),
        },
        Family { name: "stacks", scenes: pick(stacks(&[18, 22]), stacks(&[12, 14, 16, 18, 19, 20, 21, 22, 23, 24, 26, 28])) },
        Family {
            name: "ratios",
            scenes: pick(
                ratios(&[100.0, 300.0], &[3, 5]),
                ratios(&[10.0, 20.0, 30.0, 50.0, 70.0, 100.0, 150.0, 200.0, 300.0, 500.0, 1000.0], &[1, 2, 3, 5]),
            ),
        },
        Family { name: "pyramids", scenes: pick(pyramids(&[0.0, 0.1]), pyramids(&[0.0, 0.02, 0.05, 0.1, 0.2, 0.3, 0.6])) },
    ]
}

/// The long grid's leans: Box2D's 25° and 2° either way.
const LEANS: [f32; 9] = [23.0, 23.5, 24.0, 24.5, 25.0, 25.5, 26.0, 26.5, 27.0];

fn cards(rows: &[u32], leans: &[f32], mus: &[f32]) -> Vec<Scene> {
    let mut v = Vec::new();
    for &rows in rows {
        for &lean in leans {
            v.extend(mus.iter().map(|&mu| Scene::Cards { rows, lean, mu }));
        }
    }
    v
}

/// A box on a ramp `off` degrees from the friction angle of each `mu`.
fn ramps(mus: &[f32], offs: &[f32]) -> Vec<Scene> {
    let mut v = Vec::new();
    for &mu in mus {
        let edge = mu.atan().to_degrees();
        v.extend(offs.iter().map(|off| Scene::Ramp { deg: edge + off, mu, circle: false }));
    }
    v
}

/// The ladder at each lean, on friction `off` from what it needs.
fn ladders(degs: &[f32], offs: &[f32]) -> Vec<Scene> {
    let mut v = Vec::new();
    for &deg in degs {
        v.extend(offs.iter().map(|off| Scene::Ladder { deg, mu: Scene::ladder_mu(deg) + off }));
    }
    v
}

/// Ten dominoes at each spacing and friction.
fn dominoes(spacings: &[f32], mus: &[f32]) -> Vec<Scene> {
    let mut v = Vec::new();
    for &spacing in spacings {
        v.extend(mus.iter().map(|&mu| Scene::Dominoes { n: DOMINOES, spacing, mu }));
    }
    v
}

const DOMINOES: u32 = 10;
/// A domino reaches the next while they're closer than its height and its
/// thickness: 1 + 0.25.
const DOMINO_REACH: f32 = 1.25;

fn stacks(heights: &[u32]) -> Vec<Scene> {
    heights.iter().map(|&n| Scene::Stack { n }).collect()
}

/// A heavy box on `light` unit boxes, at each mass ratio.
fn ratios(ratios: &[f32], lights: &[u32]) -> Vec<Scene> {
    let mut v = Vec::new();
    for &light in lights {
        v.extend(ratios.iter().map(|&ratio| Scene::Ratio { ratio, light }));
    }
    v
}

/// A pyramid 20 wide at each friction.
fn pyramids(mus: &[f32]) -> Vec<Scene> {
    mus.iter().map(|&mu| Scene::PyramidAt { base: 20, mu }).collect()
}

/// Whether a run did what the scene should: a house, a stack, a pyramid or
/// a heavy box stands; a box on a ramp holds below the friction angle and
/// slides above it (at least half the distance g (sin θ − μ cos θ) takes it
/// in the run's 2 s); the ladder stands on friction above what it needs
/// and slides below; dominoes all fall while one reaches the next, and
/// only the first once none does.
pub fn good(scene: &Scene, b: &Behaviour) -> bool {
    match *scene {
        Scene::Cards { .. } => b.get("fallen") == 0.0,
        Scene::Ramp { .. } => {
            let a = b.get("expected a");
            let t = crate::behave::steps(scene) as f64 * DT as f64;
            if a <= 0.0 { b.get("crept") < 0.05 } else { b.get("crept") > 0.25 * a * t * t }
        }
        Scene::Ladder { mu, .. } => (b.get("stands") == 1.0) == (mu as f64 > b.get("needs mu")),
        Scene::Dominoes { n, spacing, .. } => {
            let all = if spacing < DOMINO_REACH { n } else { 1 };
            b.get("toppled") == all as f64
        }
        Scene::Stack { .. } | Scene::Ratio { .. } | Scene::PyramidAt { .. } => b.get("stands") == 1.0,
        _ => panic!("{} is in no family", scene.text()),
    }
}

/// Yes or no on each of a family's runs, and how many yeses.
pub fn share(f: &Family, runs: &[Behaviour]) -> (usize, String) {
    let marks: String = f.scenes.iter().zip(runs).map(|(s, b)| if good(s, b) { 'S' } else { '.' }).collect();
    (marks.matches('S').count(), marks)
}
