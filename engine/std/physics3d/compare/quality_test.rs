//! Quality as a test, in 3D: how soon piles of turning boxes come to rest
//! in the physics3d mod, how deep they sink, what energy is left, and
//! whether stacks stand, measured by the comparison's own code
//! (`measure.rs`) on its scenes (`scenes.rs`), against bounds derived from
//! what Rapier 3D 0.36 and Box3D 0.1 meet on the same scenes (Jolt 5.6
//! recorded beside them). The bounds are set as 2D's are
//! (`//engine/std/physics2d/compare:quality_test`; physics.md, "Quality as a
//! test"): rest from the references' medians over the sizes, since
//! settling is chaotic; depth at half the shallower reference's. The values
//! are from
//!
//!     ./bazel run --config=bench //engine/std/physics3d/compare:bench -- boxes 200,300,400,500 all --rotate --runs=1
//!
//! `TUNE=<variant>` in the environment (`physics3d::Tuning::parse`) runs
//! ours tuned so, which is how the bounds were checked to fail on the
//! variants they should. The 1000- and 10 000-body scenes are
//! `:quality_long_test` (manual).

use std::sync::Arc;

use physics_testkit::Broken;
use physics3d_compare::measure::Run;
use physics3d_compare::record::{SIZES, STILL};
use physics3d_compare::scenes::Kind;
use physics3d_compare::{baseline, record, runs};

/// Our step on `kind` at each of `sizes`, in threads of their own, each
/// printed so a failing run shows every number.
/// Each run once in the binary (`runs.rs`), which the baseline reads too.
fn runs(kind: Kind, sizes: &[usize], rotate: bool) -> Vec<Arc<Run>> {
    let runs = runs::par(sizes, |&n| runs::ours(kind, n, rotate));
    for (n, r) in sizes.iter().zip(&runs) {
        let q = &r.quality;
        println!(
            "{} {n}{}: at rest from {:?}; deepest {:.4} at end, {:.4} during, mean {:.4} during; energy {:.1e} a body; {:.2} partners a body, {:.2} not columns; tilt {:.1}°, top moved {:.3}, {} escaped",
            kind.name(),
            if rotate { ", turning" } else { "" },
            r.settled_at,
            q.pen_max,
            r.pen_max_during,
            r.pen_mean_during,
            q.kinetic_energy / q.bodies.max(1) as f64,
            q.contacts_per_body,
            q.not_columns,
            q.tilt,
            r.top_moved,
            q.escaped,
        );
    }
    runs
}

/// What a pile must meet over its sizes: 2D's `PileBounds`, the same rules
/// from Rapier's and Box3D's values. `partners` and `not_columns` say it is
/// a pile (docs/lore: a staggered drop lattice still stacks locked spheres
/// in columns): columns are about two partners a body, and 0 not columns.
struct PileBounds {
    rest_worst: usize,
    rest_median: usize,
    deepest_end: f32,
    deepest_during: f32,
    mean_during: f32,
    energy_end: f64,
    partners: f64,
    not_columns: f64,
}

fn piles_meet(kind: Kind, sizes: &[usize], rotate: bool, b: &PileBounds) {
    let runs = runs(kind, sizes, rotate);
    let mut broken = Broken::default();
    for (n, r) in sizes.iter().zip(&runs) {
        let (q, name) = (&r.quality, format!("{} {n}", kind.name()));
        let rest = r.settled_at.unwrap_or(usize::MAX);
        broken.check(q.contacts_per_body >= b.partners && q.not_columns >= b.not_columns, || format!("{name}: not a pile: {q:?}"));
        broken.check(q.escaped == 0, || format!("{name}: {} escaped", q.escaped));
        broken.check(rest <= b.rest_worst, || format!("{name}: at rest from {:?}, bound {}", r.settled_at, b.rest_worst));
        broken.check(q.pen_max <= b.deepest_end, || format!("{name}: {} deep at the end, bound {}", q.pen_max, b.deepest_end));
        broken.check(r.pen_max_during <= b.deepest_during, || {
            format!("{name}: {} deep while settling, bound {}", r.pen_max_during, b.deepest_during)
        });
        broken.check(r.pen_mean_during <= b.mean_during, || {
            format!("{name}: {} deep on average, bound {}", r.pen_mean_during, b.mean_during)
        });
        let (energy, bound) = (q.kinetic_energy / q.bodies as f64, b.energy_end.max(STILL));
        broken.check(energy <= bound, || format!("{name}: energy {energy:e} a body at the end, bound {bound:e}"));
    }
    let mut rests: Vec<usize> = runs.iter().map(|r| r.settled_at.unwrap_or(usize::MAX)).collect();
    rests.sort();
    let m = rests[rests.len() / 2];
    broken.check(m <= b.rest_median, || format!("at rest from {rests:?}: median {m}, bound {}", b.rest_median));
    broken.assert();
}

/// Turning cubes (1000 steps, 2026-09-26). At rest from: Rapier 128, 265,
/// 170, 203 (median 203); Box3D 117, 262, 203, 219 (219); Jolt 240, 835,
/// 274, never; ours 127, 189, 200, 160 (142, 177, 187, 200 before
/// recycling, 2026-09-27). Deepest at the end: Rapier up to 0.016, Box3D
/// 0.014, Jolt 0.020 (its slop); ours 0.0055. While settling:
/// 0.116 and 0.088 (ours 0.075), mean 0.029 and 0.029 (ours 0.007).
/// Partners a body 2.57-3.43 and 0.88-0.99 not columns everywhere.
#[test]
fn piles_of_turning_boxes_rest_as_soon_as_rapier_and_box3d_do() {
    let b = PileBounds {
        rest_worst: 2 * 219,
        rest_median: 273,
        deepest_end: 0.5 * 0.0139,
        deepest_during: 1.25 * 0.1162,
        mean_during: 0.0291,
        energy_end: 10.0 * BOXES_TURNING_ENERGY,
        partners: 2.0,
        not_columns: 0.8,
    };
    piles_meet(Kind::BoxPile, &SIZES, true, &b);
}

/// Locked cubes. At rest from: Rapier 65, 90, never, 127 (median 127);
/// Box3D 59, 90, never, 250 (250); both breathe at 400 (docs/lore: a locked
/// box pile breathes forever under soft contacts at 4 iterations); Jolt 47,
/// 68, 103, 83; ours 42, 55, 80, 66. Deepest at the end: 0.0021 and 0.0022
/// (Jolt 0.020); ours 0.0006. While settling 0.077 and 0.085 (ours 0.038),
/// mean 0.029 and 0.027 (ours 0.005).
#[test]
fn piles_of_locked_boxes_rest_as_soon_as_rapier_and_box3d_do() {
    let b = PileBounds {
        rest_worst: 2 * 250,
        rest_median: 312,
        deepest_end: 0.5 * 0.0021,
        deepest_during: 1.25 * 0.0848,
        mean_during: 0.0268,
        energy_end: 10.0 * BOXES_LOCKED_ENERGY,
        partners: 2.0,
        not_columns: 0.8,
    };
    piles_meet(Kind::BoxPile, &SIZES, false, &b);
}

/// Turning planks (0.5 by 0.125 by 0.25). At rest from: Rapier 212, 255,
/// 364, 267 (median 267); Box3D 823, 209, 254, 302 (302); Jolt 187, 421,
/// 526, 992; ours 303, 257, 249, 287 (341, 239, 325, 354 before recycling).
/// Deepest at the end: Rapier up to 0.031, Box3D 0.040; ours 0.0085. While settling 0.27 and 0.29 (ours 0.30), mean 0.016
/// and 0.019 (ours 0.014).
#[test]
fn piles_of_turning_planks_rest_as_soon_as_rapier_and_box3d_do() {
    let b = PileBounds {
        rest_worst: 2 * 302,
        rest_median: 377,
        deepest_end: 0.5 * 0.0307,
        deepest_during: 1.25 * 0.2921,
        mean_during: 0.0164,
        // How still they are once at rest is the next test's.
        energy_end: f64::INFINITY,
        partners: 2.5,
        not_columns: 0.8,
    };
    piles_meet(Kind::PlankPile, &SIZES, true, &b);
}

/// The same planks at rest are as still as Rapier's and Box3D's: theirs
/// under 1.0e-11 a body at step 1000, ours 3.7e-13 to 5.6e-13 (get-emj.43:
/// 1.4e-7, 1.9e-10, 2.1e-8 and 3.0e-13 before recycling, some planks
/// rocking at about 5e-4 on points that flickered between features).
#[test]
fn piles_of_turning_planks_are_as_still_at_rest_as_rapier_and_box3d() {
    let b = PileBounds {
        rest_worst: usize::MAX,
        rest_median: usize::MAX,
        deepest_end: f32::INFINITY,
        deepest_during: f32::INFINITY,
        mean_during: f32::INFINITY,
        energy_end: 10.0 * PLANKS_TURNING_ENERGY,
        partners: 2.5,
        not_columns: 0.8,
    };
    piles_meet(Kind::PlankPile, &SIZES, true, &b);
}

/// A stack of `n` turning cubes against Rapier's and Box3D's values on it:
/// their later rest twice over, half their smaller top move and depth, and
/// the smaller tilt or 1°. Stacks stand, and don't vary with rounding.
fn stands(n: usize, rest: (usize, usize), top: (f32, f32), deepest: (f32, f32), tilt: (f32, f32), energy: (f64, f64)) {
    let r = &runs(Kind::Stack, &[n], true)[0];
    let q = &r.quality;
    let mut broken = Broken::default();
    let bound = 2 * rest.0.max(rest.1);
    broken.check(q.escaped == 0, || format!("stack {n}: {} escaped", q.escaped));
    broken.check(r.settled_at.is_some_and(|s| s <= bound), || format!("stack {n}: at rest from {:?}, bound {bound}", r.settled_at));
    let bound = 0.5 * top.0.min(top.1);
    broken.check(r.top_moved <= bound, || format!("stack {n}: its top moved {}, bound {bound}", r.top_moved));
    let bound = 0.5 * deepest.0.min(deepest.1);
    broken.check(q.pen_max <= bound, || format!("stack {n}: {} deep at the end, bound {bound}", q.pen_max));
    let bound = tilt.0.min(tilt.1).max(1.0);
    broken.check(q.tilt <= bound, || format!("stack {n}: a box leans {}°, bound {bound}", q.tilt));
    let (e, bound) = (q.kinetic_energy / n as f64, (10.0 * energy.0.max(energy.1)).max(STILL));
    broken.check(e <= bound, || format!("stack {n}: energy {e:e} a body at the end, bound {bound:e}"));
    broken.assert();
}

/// 10, 15 and 20 cubes high, each set off by up to 0.04 (1000 steps,
/// 2026-09-26; the arguments are Rapier's and Box3D's values). Ours: at rest
/// from 2, 13, 18; the top 0.007, 0.017, 0.032 lower; 0.0012-0.0026 deep;
/// leaning at most 0.1°. Jolt topples the 20 (8 cubes out).
#[test]
fn stacks_of_turning_boxes_stand_as_in_rapier_and_box3d() {
    std::thread::scope(|s| {
        s.spawn(|| stands(10, (11, 12), (0.028, 0.028), (0.0047, 0.0047), (0.1, 0.1), (6.6e-11, 1.0e-12)));
        s.spawn(|| stands(15, (19, 19), (0.058, 0.058), (0.0075, 0.0075), (0.1, 0.1), (1.2e-6, 9.7e-7)));
        s.spawn(|| stands(20, (27, 27), (0.121, 0.124), (0.0106, 0.0106), (0.3, 0.3), (2.4e-5, 1.0e-5)));
    });
}

/// Five high: Rapier and Box3D rest from step 1 and 1, the top 0.006
/// lower; ours from 0, the top 0.002 lower, 2.7e-10 a body (get-emj.42:
/// until static contacts were softened, never, the column circling on the
/// floor's corners at 4.3 Hz, its top 0.026 lower).
#[test]
fn a_five_high_stack_of_turning_boxes_comes_to_rest() {
    stands(5, (1, 1), (0.006, 0.006), (0.0022, 0.0022), (0.0, 0.0), (3.7e-10, 1.9e-11));
}

#[cfg(feature = "long")]
mod quality_long;

/// Kinetic energy a body at the end: the references' worst over the sizes
/// they came to rest on (a pile that breathes isn't at rest; the bounds are
/// ten times these, as in 2D). Turning cubes: Rapier up to 1.7e-9, Box3D
/// 4.2e-11 (ours 3.1e-11). Locked: 4.6e-10 and 1.4e-10 (ours 1.1e-11).
/// Turning planks: 1.0e-11 and 1.5e-12 (ours up to 5.6e-13).
const BOXES_TURNING_ENERGY: f64 = 1.7e-9;
const BOXES_LOCKED_ENERGY: f64 = 4.6e-10;
const PLANKS_TURNING_ENERGY: f64 = 1.0e-11;

/// The default suite's baseline, group `quality`: every value `record.rs`
/// takes from these tests' runs, each within its band of `baseline.txt`,
/// better or worse, all of them listed where any moved.
#[test]
fn baseline() {
    baseline::assert_holds(record::DEFAULT, &["quality"], &record::quality(false), record::WRITE);
}

/// The long suite's, from `:quality_long_test`'s runs (`baseline_long.txt`).
#[cfg(feature = "long")]
#[test]
fn baseline_long() {
    baseline::assert_holds(record::LONG, &["quality"], &record::quality(true), record::WRITE_LONG);
}
