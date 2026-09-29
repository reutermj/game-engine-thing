//! The bounce families in 3D, as 2D's (//engine/std/physics/compare,
//! `bounces.rs`, which says why a family): a sphere, a cube landing flat, on
//! an edge and on a corner, two free spheres, angled impacts and 20 s of
//! bounces, over restitution, impact speed, gravity (none, the
//! comparison's 9.81 and 40), where in a step it meets, and 5 and 6
//! substeps. Not the step: physics3d is a mod, stepped at the simulation's
//! fixed rate, so 30 Hz is 2D's alone (its arrays step at any rate). Each
//! scene is a `Kind::Hit`, the bounce packed into its n; `--bounces` in the
//! bench measures every engine on them (`--long`, the long grids).

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

fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn most(v: impl Iterator<Item = f64>) -> f64 {
    v.fold(f64::NEG_INFINITY, f64::max)
}

/// A family's statistics over its runs, by the names and rules of 2D's
/// `bounces::stats`; square on here is a sphere, or a cube landing flat
/// without friction.
pub fn stats(f: &Family, runs: &[Behaviour]) -> Vec<(&'static str, f64)> {
    let both = |keep: &dyn Fn(&Hit, &Behaviour) -> bool, value: &dyn Fn(&Behaviour) -> f64| -> Vec<f64> {
        f.hits.iter().zip(runs).filter(|(h, b)| keep(h, b)).map(|(_, b)| value(b)).collect()
    };
    let mut v = Vec::new();
    if f.name == "series" {
        v.push(("rise worst", most(both(&|h, _| h.e >= 1.0, &|b| b.get("rise most")).into_iter())));
        // A box that tips as it lands turns in flight, trading its turn for
        // height at the next bounce: each apex against the last is a
        // round body's.
        let round = |h: &Hit| h.target == Target::Sphere;
        v.push(("decay worst", most(both(&|h, _| round(h), &|b| b.get("decay most")).into_iter())));
        v.push(("decay median", median(both(&|h, _| round(h) && h.e < 1.0, &|b| b.get("decay median")))));
        return v;
    }
    if f.name == "pairs" {
        v.push(("gain worst", most(both(&|_, _| true, &|b| b.get("gain")).into_iter())));
        v.push(("momentum worst", most(both(&|_, _| true, &|b| b.get("momentum")).into_iter())));
        return v;
    }
    let threshold = crate::behave::BOUNCE_THRESHOLD;
    // A box landing on a corner tips as it bounces, and its energy then
    // turns between height, speed and spin: bounded apart.
    let tips = |h: &Hit| matches!(h.target, Target::Edge | Target::Corner);
    v.push(("excess worst", most(both(&|h, _| !tips(h), &|b| b.get("excess")).into_iter())));
    if f.hits.iter().any(tips) {
        v.push(("excess tipping", most(both(&|h, _| tips(h), &|b| b.get("excess")).into_iter())));
    }
    let square = |h: &Hit, _: &Behaviour| h.target == Target::Sphere || (h.target == Target::Cube && h.mu == 0.0);
    let bounces = |h: &Hit, b: &Behaviour| square(h, b) && h.v > threshold && h.e > 0.0;
    let gains = both(&bounces, &|b| b.get("gain"));
    v.push(("gain worst", most(both(&square, &|b| b.get("gain")).into_iter())));
    v.push(("gain median", median(gains.clone())));
    v.push(("loss worst", most(gains.iter().map(|g| -g))));
    v.push(("bounced below", both(&|h, b| square(h, b) && h.v < threshold && b.get("bounced") == 1.0, &|_| 1.0).len() as f64));
    let flat = |h: &Hit, b: &Behaviour| bounces(h, b) && h.e >= 0.25 && b.get("bounced") == 0.0;
    v.push(("flat above", both(&flat, &|_| 1.0).len() as f64));
    if f.name == "oblique" {
        v.push(("slip worst", most(both(&|h, _| h.mu == 0.0, &|b| (b.get("tangent") - 1.0).abs()).into_iter())));
        v.push(("tangent most", most(both(&|h, _| h.mu > 0.0, &|b| b.get("tangent")).into_iter())));
        v.push(("tangent median", median(both(&|h, _| h.mu > 0.0, &|b| b.get("tangent")))));
    }
    v
}

/// A statistic by name.
#[allow(dead_code)] // The tests read statistics by name; the bench and the baseline take them all.
pub fn stat(stats: &[(&'static str, f64)], name: &str) -> f64 {
    stats.iter().find(|(k, _)| *k == name).unwrap_or_else(|| panic!("no {name} in {stats:?}")).1
}
