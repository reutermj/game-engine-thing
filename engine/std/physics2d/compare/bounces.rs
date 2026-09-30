//! The bounce families: one bounce (`Scene::Hit`) over a grid of what can
//! decide it, so that restitution is judged on the whole of its behaviour,
//! not on one drop (get-emj.56: a ball dropped 5 gained 4.8% of its height
//! a bounce, which the drops at e ≤ 0.5 hid). Each family is a grid, each
//! run measured by `behave::hit` (or `series`), and the family by
//! statistics over its runs (`stats`), which `:behaviour_test` bounds by
//! the analytic answer where there is one (a bounce returns e of the impact
//! speed along the normal, never more energy than that, and nothing below
//! `BOUNCE_THRESHOLD`) and by Box2D's and Rapier's statistics on exactly the
//! same grid otherwise (physics-testing.md, "Families at the edge", for why
//! a family). `BOUNCES=<names>` (`all`) in the comparison measures every
//! engine on them, `BOUNCE_LONG=1` the long grids, `BOUNCE_RUNS=1` every
//! run (physics.md, "Bounces").
//!
//! - `drops`: a ball, a box landing flat and a box landing on a corner,
//!   onto a floor, at restitution 0 to 1, impact speeds from under the
//!   threshold to fast, gravity none (pong's), the comparison's and four
//!   times it, and where in a step it meets.
//! - `rates`: a ball and a box at 60 and 30 Hz, 5 and 6 substeps: whether
//!   an error scales with the step.
//! - `oblique`: angled impacts, friction off and on: along the normal e,
//!   along the floor what friction takes, or nothing.
//! - `pairs`: a ball on a ball, equal and three times as heavy, and on a
//!   box, both free: e of the closing speed, and momentum kept.
//! - `series`: 20 s of bounces: a lossless ball never rises, and one of
//!   restitution e keeps e² of its height a bounce.

use physics_testkit::bounces::Bounce;

use crate::behave::Behaviour;
use crate::family::Family;
use crate::scene::{HIT, Hit, Scene, Target};
use crate::solver::BOUNCE_THRESHOLD;

/// Every bounce family, on its short grid (the default suite's) or its
/// long one.
pub fn families(long: bool) -> Vec<Family> {
    let pick = |short: &'static [f32], full: &'static [f32]| if long { full } else { short };
    let es_fine: &[f32] = &[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
    let quarters: &[f32] = &[0.0, 0.25, 0.5, 0.75];
    let floor = [Target::Circle, Target::Box, Target::Corner];
    let flat = [Target::Circle, Target::Box];
    let mut v = Vec::new();

    let mut drops = Vec::new();
    for target in floor {
        for &e in pick(&[0.0, 0.5, 1.0], es_fine) {
            for &speed in pick(&[0.8, 1.5, 5.0, 14.0], &[0.5, 0.8, 0.95, 1.1, 1.3, 1.6, 2.0, 3.0, 5.0, 7.0, 10.0, 14.0, 20.0, 28.0, 40.0]) {
                for &g in pick(&[0.0, 20.0, 80.0], &[0.0, 10.0, 20.0, 40.0, 80.0]) {
                    for &phase in pick(&[0.0, 0.5], quarters) {
                        drops.push(Hit { target, e, v: speed, g, phase, ..HIT });
                    }
                }
            }
        }
    }
    v.push(("drops", drops));

    let mut rates = Vec::new();
    for target in flat {
        for &e in pick(&[0.5, 1.0], &[0.25, 0.5, 0.75, 1.0]) {
            for &speed in pick(&[3.0, 14.0], &[2.0, 3.0, 5.0, 14.0, 28.0]) {
                for g in [20.0, 80.0] {
                    for hz in [60.0, 30.0] {
                        for sub in [5, 6] {
                            for &phase in pick(&[0.0, 0.5], quarters) {
                                rates.push(Hit { target, e, v: speed, g, hz, sub, phase, ..HIT });
                            }
                        }
                    }
                }
            }
        }
    }
    v.push(("rates", rates));

    let mut oblique = Vec::new();
    for target in flat {
        for e in [0.5, 1.0] {
            for &deg in pick(&[30.0, 60.0], &[15.0, 30.0, 45.0, 60.0, 75.0]) {
                for &mu in pick(&[0.0, 0.3, 1.0], &[0.0, 0.1, 0.3, 0.6, 1.0]) {
                    for g in [0.0, 20.0] {
                        for &speed in pick(&[5.0, 14.0], &[3.0, 5.0, 14.0, 28.0]) {
                            for &phase in pick(&[0.0], &[0.0, 0.5]) {
                                let along = speed * f32::to_radians(deg).tan();
                                oblique.push(Hit { target, e, v: speed, g, mu, along, phase, ..HIT });
                            }
                        }
                    }
                }
            }
        }
    }
    v.push(("oblique", oblique));

    let mut pairs = Vec::new();
    let bodies: &[(Target, f32)] = if long {
        &[(Target::Balls, 1.0), (Target::Balls, 3.0), (Target::Balls, 10.0), (Target::BallBox, 1.0), (Target::BallBox, 3.0)]
    } else {
        &[(Target::Balls, 1.0), (Target::Balls, 3.0), (Target::BallBox, 1.0)]
    };
    for &(target, ratio) in bodies {
        for &e in pick(&[0.0, 0.5, 1.0], &[0.0, 0.25, 0.5, 0.75, 1.0]) {
            for &speed in pick(&[2.0, 14.0], &[1.5, 3.0, 7.0, 14.0, 28.0]) {
                for g in [0.0, 20.0] {
                    for &phase in pick(&[0.0, 0.5], quarters) {
                        pairs.push(Hit { target, ratio, e, v: speed, g, phase, ..HIT });
                    }
                }
            }
        }
    }
    v.push(("pairs", pairs));

    let mut series = Vec::new();
    for target in flat {
        for e in [0.5, 0.75, 0.9, 1.0] {
            for g in [20.0, 80.0] {
                for &speed in pick(&[14.0], &[7.0, 14.0, 28.0]) {
                    for &hz in pick(&[60.0], &[60.0, 30.0]) {
                        series.push(Hit { target, e, v: speed, g, hz, secs: 20.0, ..HIT });
                    }
                }
            }
        }
    }
    v.push(("series", series));

    v.into_iter().map(|(name, hits)| Family { name, scenes: hits.into_iter().map(Scene::Hit).collect() }).collect()
}

fn hit(s: &Scene) -> Hit {
    match *s {
        Scene::Hit(h) => h,
        _ => panic!("{} is no bounce", s.text()),
    }
}

/// What the test kit's statistics read of a 2D bounce: a ball is round,
/// and only a box landing on a corner tips.
impl Bounce for Hit {
    fn e(&self) -> f32 {
        self.e
    }
    fn speed(&self) -> f32 {
        self.v
    }
    fn mu(&self) -> f32 {
        self.mu
    }
    fn round(&self) -> bool {
        self.target == Target::Circle
    }
    fn flat(&self) -> bool {
        self.target == Target::Box
    }
    fn tips(&self) -> bool {
        self.target == Target::Corner
    }
}

/// A family's statistics over its runs, by name: what the tests bound and
/// the baseline records (`physics_testkit::bounces::stats` has each), with
/// our solver's threshold.
pub fn stats(f: &Family, runs: &[Behaviour]) -> Vec<(&'static str, f64)> {
    let hits: Vec<Hit> = f.scenes.iter().map(hit).collect();
    physics_testkit::bounces::stats(f.name, &hits, runs, BOUNCE_THRESHOLD)
}

/// A statistic by name.
#[allow(unused_imports)] // The tests read statistics by name; the comparison and the baseline take them all.
pub use physics_testkit::bounces::stat;
