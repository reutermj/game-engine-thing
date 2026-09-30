//! The bounce families in 3D, as 2D's (//engine/std/physics2d/compare,
//! `bounces.rs`, which says why a family): a sphere, a cube landing flat, on
//! an edge and on a corner, two free spheres, angled impacts and 20 s of
//! bounces, over restitution, impact speed, gravity (none, the
//! comparison's 9.81 and 40), where in a step it meets, and 5 and 6
//! substeps. Not the step: physics3d is a mod, stepped at the simulation's
//! fixed rate, so 30 Hz is 2D's alone (its arrays step at any rate). Each
//! scene is a `Kind::Hit`, the bounce packed into its n; `--bounces` in the
//! bench measures every engine on them (`--long`, the long grids).

use physics_testkit::bounces::Bounce;

use crate::behave::Behaviour;
use crate::scenes::{HIT, Hit, Target};

/// One family: its grid.
pub struct Family {
    pub name: &'static str,
    pub hits: Vec<Hit>,
}

/// Every family, on its short grid (the default suite's) or its long one:
/// about 1500 runs, since each of ours is an engine of its own, and a
/// process maps each engine's mods anew (2300 exhausted it).
pub fn families(long: bool) -> Vec<Family> {
    let pick = |short: &'static [f32], full: &'static [f32]| if long { full } else { short };
    let halves: &[f32] = &[0.0, 0.5];
    let mut v = Vec::new();

    let mut drops = Vec::new();
    for target in [Target::Sphere, Target::Cube, Target::Edge, Target::Corner] {
        for &e in pick(&[0.0, 0.5, 1.0], &[0.0, 0.5, 1.0]) {
            for &speed in pick(&[0.8, 3.0, 9.9], &[0.5, 0.8, 0.9, 1.1, 1.5, 2.0, 3.0, 5.0, 9.9, 14.0, 20.0]) {
                for &g in pick(&[0.0, 9.81, 40.0], &[0.0, 9.81, 20.0, 40.0]) {
                    for &phase in pick(&[0.0], halves) {
                        drops.push(Hit { target, e, v: speed, g, phase, ..HIT });
                    }
                }
            }
        }
    }
    v.push(Family { name: "drops", hits: drops });

    let mut rates = Vec::new();
    for target in [Target::Sphere, Target::Cube] {
        for e in [0.5, 1.0] {
            for &speed in pick(&[3.0, 9.9], &[2.0, 3.0, 5.0, 9.9, 20.0]) {
                for g in [9.81, 40.0] {
                    for sub in [5, 6] {
                        for &phase in pick(&[0.0], halves) {
                            rates.push(Hit { target, e, v: speed, g, sub, phase, ..HIT });
                        }
                    }
                }
            }
        }
    }
    v.push(Family { name: "rates", hits: rates });

    let mut oblique = Vec::new();
    for target in [Target::Sphere, Target::Cube] {
        for e in [0.5, 1.0] {
            for &deg in pick(&[30.0, 60.0], &[15.0, 45.0, 75.0]) {
                for mu in [0.0, 0.3, 1.0] {
                    for g in [0.0, 9.81] {
                        for &speed in pick(&[5.0], &[5.0, 9.9]) {
                            // Along the floor to a tenth, as a bounce packs it.
                            let along = (speed * f32::to_radians(deg).tan() * 10.0).round() / 10.0;
                            oblique.push(Hit { target, e, v: speed, g, mu, along, ..HIT });
                        }
                    }
                }
            }
        }
    }
    v.push(Family { name: "oblique", hits: oblique });

    let mut pairs = Vec::new();
    for ratio in [1.0, 3.0] {
        for e in [0.0, 0.5, 1.0] {
            for &speed in pick(&[2.0, 9.9], &[1.5, 3.0, 9.9, 20.0]) {
                for g in [0.0, 9.81] {
                    for &phase in pick(&[0.0], halves) {
                        pairs.push(Hit { target: Target::Spheres, ratio, e, v: speed, g, phase, ..HIT });
                    }
                }
            }
        }
    }
    v.push(Family { name: "pairs", hits: pairs });

    let mut series = Vec::new();
    for target in [Target::Sphere, Target::Cube] {
        for e in [0.5, 0.75, 0.9, 1.0] {
            for g in [9.81, 40.0] {
                for &speed in pick(&[9.9], &[5.0, 9.9, 20.0]) {
                    series.push(Hit { target, e, v: speed, g, series: true, ..HIT });
                }
            }
        }
    }
    v.push(Family { name: "series", hits: series });
    v
}

/// What the test kit's statistics read of a 3D bounce: a sphere is round,
/// and a cube landing on an edge or a corner tips.
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
        self.target == Target::Sphere
    }
    fn flat(&self) -> bool {
        self.target == Target::Cube
    }
    fn tips(&self) -> bool {
        matches!(self.target, Target::Edge | Target::Corner)
    }
}

/// A family's statistics over its runs, by the names and rules of 2D's
/// (`physics_testkit::bounces::stats`), at physics3d's threshold.
pub fn stats(f: &Family, runs: &[Behaviour]) -> Vec<(&'static str, f64)> {
    physics_testkit::bounces::stats(f.name, &f.hits, runs, crate::behave::BOUNCE_THRESHOLD)
}

/// A statistic by name.
#[allow(unused_imports)] // The tests read statistics by name; the bench and the baseline take them all.
pub use physics_testkit::bounces::stat;
