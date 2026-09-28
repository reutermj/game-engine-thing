//! The quality tests at 10 000 bodies (`:quality_long_test`, manual: about
//! a minute and a half at -c opt, and much longer without). The bounds are
//! set by the same rules as the small scenes', from the references on these
//! scenes.

use super::*;
use record::{BIG_PILES, LONG_STEPS};

/// At rest from (2500 steps, 2026-09-26): Box2D 200, 190, 200; Rapier 210,
/// 200, 190; ours 200, 230, 300. Energy at the end: 1.5e-10 and 3.1e-8
/// (ours 2.4e-9). Deepest at the end: Box2D up to 0.058,
/// Rapier 0.060, ours 0.016. While settling: 0.38 and 0.40 (ours 0.31),
/// mean 0.061 and 0.057 (ours 0.026). Contacts a body 1.56-1.59 (ours
/// 1.39-1.40), islands 2-9 (ours 17).
#[test]
fn big_real_piles_rest_as_soon_as_box2d_and_rapier_do() {
    let b = PileBounds {
        rest_worst: 2 * 200,
        rest_median: 250,
        first_rest: None,
        deepest_end: 0.5 * 0.0576,
        deepest_during: 1.25 * 0.3973,
        mean_during: 0.0574,
        energy_end: ENERGY_LOCKED,
        contacts_per_body: 1.2,
        islands: 30,
    };
    piles_meet(&BIG_PILES, record::BIG_WIDTH, false, LONG_STEPS, &b);
}

/// Turning: no engine stays at rest, a body now and then moving over 0.05
/// again long after the pile came to rest (which sleeping would take in its
/// stride). At rest from: Box2D 1550, 1760, 2300 (median 1760); Rapier
/// 2490, 2270, never (2490); ours 350, 320, 1870 (before get-emj.48, 330,
/// 350, 1620; before get-emj.61's default, 370, 310, 360). So rest from is
/// bounded by the references' medians themselves (the worst of ours by the
/// later, the median by the earlier), and the first look at rest by the
/// rules: Box2D 330, 470, 280 (median 330), Rapier 240, 430, 520 (430);
/// ours 350, 320, 400. Deepest at the end: Box2D up to 0.126, Rapier
/// 0.095, ours 0.024. While settling: 0.47 and 0.45 (ours 0.43), mean
/// 0.052 and 0.052 (ours 0.018). Energy at the end: 4.1e-8 and 6.7e-7
/// (ours 1.2e-8). Contacts a body 2.02-2.07, 1-4 islands (ours 1.95-1.97,
/// 3-5).
#[test]
fn big_real_piles_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = PileBounds {
        rest_worst: 2490,
        rest_median: 1760,
        first_rest: Some((2 * 430, 537)),
        deepest_end: 0.5 * 0.0950,
        deepest_during: 1.25 * 0.4650,
        mean_during: 0.0521,
        energy_end: ENERGY_TURNING,
        contacts_per_body: 1.6,
        islands: 30,
    };
    piles_meet(&BIG_PILES, record::BIG_WIDTH, true, LONG_STEPS, &b);
}

/// The pyramid 100 wide (5050 boxes): ours at rest from 150, its top 0.27
/// lower, 0.006 deep; turning, 780, 0.26, 0.006, leaning 0.4° (by level,
/// before get-emj.61's default: 450, 0.25, 0.006, 0.3°).
#[test]
fn a_big_pyramid_stands_as_in_box2d_and_rapier() {
    let locked = from_refs((70, 280), (0.834, 0.834), (0.0188, 0.0188), (0.0, 0.0), (4.1e-11, 1.9e-10));
    stand(&[(Scene::Pyramid { base: 100 }, locked)], false, LONG_STEPS);
    let turning = from_refs((160, 1100), (1.462, 1.545), (0.0408, 0.0350), (1.5, 1.8), (5.9e-10, 8.2e-8));
    stand(&[(Scene::Pyramid { base: 100 }, turning)], true, LONG_STEPS);
}

/// A turning pyramid 50 wide (1275 boxes) stands in the default's colors,
/// and in the colors of its contacts shuffled afresh each step and taken
/// row by row from the top down (`rot/order`), by the references' bounds
/// (`pyramid_50_refs`). While each step restarted its points from the
/// substeps' average impulse, only pair order stood: shuffled, it rested
/// from 680, and from the top down from 660 (get-emj.48). Ours now: 120,
/// 90, 90 (by level, before get-emj.61's default: 150, 230, 180).
#[test]
fn a_pyramid_50_wide_that_turns_stands_whatever_order_its_contacts_are_solved_in() {
    pyramid_stands_in(50, pyramid_50_refs, &["rot", "rot/order=2", "rot/order=3"]);
}

/// The pyramid 100 wide turning, graph-colored as Box2D colors
/// (`rot/colored=4`), the order threads share, by the same bounds: at rest
/// from 780, the top 0.26 lower, leaning 0.4°. Colored is the default, so
/// this is the turning half of `a_big_pyramid_stands_as_in_box2d_and_rapier`
/// again, kept so that a `SOLVER` run still sees its colored pyramid. With
/// both impulses carried from the last substep it rested from 1500; while
/// each step restarted its points from the substeps' average impulse it
/// never rested, its top 5.06 lower and a box leaning 37° (get-emj.48;
/// physics.md, "The solver's speed").
#[test]
fn a_big_pyramid_that_turns_stands_when_its_contacts_are_colored() {
    let scene = Scene::Pyramid { base: 100 };
    let b = from_refs((160, 1100), (1.462, 1.545), (0.0408, 0.0350), (1.5, 1.8), (5.9e-10, 8.2e-8));
    let run = runs::settled(scene, true, LONG_STEPS, &runs::with_solver("rot/colored=4"));
    print_runs(&[scene], true, std::slice::from_ref(&run));
    stand_runs(&[(scene, b)], &[run]);
}

/// Kinetic energy a body at the end: ten times the references' worst.
const ENERGY_LOCKED: f64 = 10.0 * 3.1e-8;
const ENERGY_TURNING: f64 = 10.0 * 6.7e-7;

// The wider families (`record::WIDE_PILES`, `MIXED`, `PYRAMID_FAMILIES`):
// more data before tuning, each family bounded by the rules on the
// references' statistics over exactly its grid (physics-testing.md,
// "Wider families"), measured with `SETTLE=<steps> SCENES=...` in the
// comparison, 2026-09-28.

/// One reference's statistics over a family's grid, what the rules take:
/// the median of its rests from and of its first looks at rest, and the
/// worst depth at the end, while settling and on average while settling,
/// and energy a body at the end of the runs that came to rest.
struct Refs {
    rest: u32,
    first: u32,
    depth_end: f32,
    during: f32,
    mean_during: f64,
    energy: f64,
}

/// The rules (physics.md, "Quality as a test") on Box2D's and Rapier's
/// statistics: the worst rest within twice the later median, the median
/// within a quarter over it; half the shallower worst depth at rest, a
/// quarter over the worst while settling, the shallower mean; ten times
/// the worse energy. `first`: the same rules on the first look at rest,
/// and rest from bounded by the medians themselves, where the references
/// don't stay at rest (the big turning piles).
fn by_the_rules(b: Refs, r: Refs, first: bool, contacts: f64, islands: usize) -> PileBounds {
    let later = b.rest.max(r.rest);
    let (rest_worst, rest_median, first_rest) = if first {
        let later_first = b.first.max(r.first);
        (later, b.rest.min(r.rest), Some((2 * later_first, later_first * 5 / 4)))
    } else {
        (2 * later, later * 5 / 4, None)
    };
    PileBounds {
        rest_worst,
        rest_median,
        first_rest,
        deepest_end: 0.5 * b.depth_end.min(r.depth_end),
        deepest_during: 1.25 * b.during.max(r.during),
        mean_during: b.mean_during.min(r.mean_during),
        energy_end: 10.0 * b.energy.max(r.energy),
        contacts_per_body: contacts,
        islands,
    }
}

fn wide(width: f32) -> (&'static [u32], u32) {
    let &(_, sizes, steps) = record::WIDE_PILES.iter().find(|(w, _, _)| *w == width).expect("a wide family");
    (sizes, steps)
}

fn wide_meet(width: f32, turning: bool, b: Refs, r: Refs, first: bool) {
    let (sizes, steps) = wide(width);
    let (contacts, islands) = if turning { (1.6, 30) } else { (1.2, 40) };
    piles_meet(sizes, width, turning, steps, &by_the_rules(b, r, first, contacts, islands));
}

/// 21 wide, 150-450: at rest from, Box2D 140-190 (median 160), Rapier
/// 150-210 (180); ours 120-320 (200).
#[test]
fn narrow_piles_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 160, first: 160, depth_end: 0.0464, during: 0.329, mean_during: 0.0641, energy: 1.2e-10 };
    let r = Refs { rest: 180, first: 180, depth_end: 0.0464, during: 0.312, mean_during: 0.0662, energy: 1.9e-9 };
    wide_meet(21.0, false, b, r, false);
}

/// 41 wide, 300-1200 by 100 (the default's sizes and between): Box2D
/// 130-260 (170), Rapier 130-220 (160); ours 100-240 (210, the bound 212).
#[test]
fn piles_at_every_hundred_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 170, first: 170, depth_end: 0.0666, during: 0.363, mean_during: 0.0613, energy: 1.8e-10 };
    let r = Refs { rest: 160, first: 160, depth_end: 0.0565, during: 0.320, mean_during: 0.0579, energy: 4.3e-9 };
    wide_meet(41.0, false, b, r, false);
}

/// 81 wide, 800-2400: Box2D 130-250 (190), Rapier 150-250 (220); ours
/// 110, 170, 170, 340, 160, 1380, never, 1150, 260: at 1800-2200 first at
/// rest by 290, then moving again (get-emj.63).
#[test]
#[ignore = "get-emj.63: locked piles 81 wide move again after resting, where Box2D and Rapier don't"]
fn wider_piles_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 190, first: 190, depth_end: 0.0682, during: 0.385, mean_during: 0.0633, energy: 3.0e-10 };
    let r = Refs { rest: 220, first: 210, depth_end: 0.0734, during: 0.378, mean_during: 0.0622, energy: 2.6e-9 };
    wide_meet(81.0, false, b, r, false);
}

/// 161 wide, 2000-4800: Box2D 140-230 (200), Rapier 140-260 (180); ours
/// 140-290, and 410 at 4800, past twice the later median (get-emj.63).
#[test]
#[ignore = "get-emj.63: the locked pile of 4800, 161 wide, rests late"]
fn piles_161_wide_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 200, first: 200, depth_end: 0.0750, during: 0.407, mean_during: 0.0613, energy: 3.2e-10 };
    let r = Refs { rest: 180, first: 180, depth_end: 0.0651, during: 0.366, mean_during: 0.0591, energy: 6.4e-9 };
    wide_meet(161.0, false, b, r, false);
}

/// 401 wide, 5000-12000 (the big piles' sizes and around them): Box2D
/// 150-230 (190), Rapier 140-210 (190); ours 150-340 (230).
#[test]
fn big_piles_at_eight_sizes_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 190, first: 190, depth_end: 0.0628, during: 0.378, mean_during: 0.0612, energy: 3.0e-9 };
    let r = Refs { rest: 190, first: 180, depth_end: 0.0617, during: 0.397, mean_during: 0.0574, energy: 3.4e-8 };
    wide_meet(401.0, false, b, r, false);
}

/// Turning, 21 wide: Box2D 190-410 (230), Rapier 160-310 (190); ours
/// 150-350 (210), but the 450 keeps 2.9e-6 a body at the end, the bound
/// 2.5e-7 (get-emj.63).
#[test]
#[ignore = "get-emj.63: a narrow turning pile keeps energy at rest"]
fn narrow_piles_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 230, first: 200, depth_end: 0.1014, during: 0.357, mean_during: 0.0459, energy: 2.5e-8 };
    let r = Refs { rest: 190, first: 190, depth_end: 0.0911, during: 0.376, mean_during: 0.0429, energy: 6.0e-9 };
    wide_meet(21.0, true, b, r, false);
}

/// Turning, 41 wide, 300-1200 by 100: Box2D 140-700 (210), Rapier 160-400
/// (250); ours 140-300 (210).
#[test]
fn piles_at_every_hundred_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 210, first: 190, depth_end: 0.1363, during: 0.528, mean_during: 0.0558, energy: 2.1e-6 };
    let r = Refs { rest: 250, first: 240, depth_end: 0.1004, during: 0.484, mean_during: 0.0550, energy: 2.5e-7 };
    wide_meet(41.0, true, b, r, false);
}

/// Turning, 81 wide: Box2D 150-970 and twice never (290), Rapier 200-750
/// (260); ours 270-650 (370, the bound 362; 590 and 650 at 800 and 1000)
/// (get-emj.63).
#[test]
#[ignore = "get-emj.63: turning piles 81 wide rest later than Box2D and Rapier"]
fn wider_piles_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 290, first: 260, depth_end: 0.1039, during: 0.416, mean_during: 0.0547, energy: 4.3e-8 };
    let r = Refs { rest: 260, first: 240, depth_end: 0.1109, during: 0.449, mean_during: 0.0536, energy: 2.3e-7 };
    wide_meet(81.0, true, b, r, false);
}

/// Turning, 161 wide: Box2D 230-390 (350), Rapier 290-1470 (780); ours
/// 180-420, but 4800 never at rest in 1500 steps (get-emj.63).
#[test]
#[ignore = "get-emj.63: the turning pile of 4800, 161 wide, never comes to rest"]
fn piles_161_wide_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 350, first: 260, depth_end: 0.1250, during: 0.507, mean_during: 0.0530, energy: 7.2e-7 };
    let r = Refs { rest: 780, first: 290, depth_end: 0.1098, during: 0.446, mean_during: 0.0531, energy: 1.2e-7 };
    wide_meet(161.0, true, b, r, false);
}

/// Turning, 401 wide, 5000-12000, where no engine stays at rest: at rest
/// from, Box2D 380-2360 (median 1590), Rapier 1760-never (2450); ours
/// 250, 250, 360, 2150, 350, 320, 1870, 690 (360). First at rest, Box2D
/// 400, Rapier 340 (medians); ours 320. So the 11 000's 1870 (get-emj.62)
/// is one of two late among eight, and the family rests sooner than
/// either reference's: bounded as `big_real_piles_that_turn...` is.
#[test]
fn big_piles_at_eight_sizes_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = Refs { rest: 1590, first: 400, depth_end: 0.1363, during: 0.481, mean_during: 0.0526, energy: 6.0e-8 };
    let r = Refs { rest: 2450, first: 340, depth_end: 0.1046, during: 0.489, mean_during: 0.0527, energy: 6.7e-7 };
    wide_meet(401.0, true, b, r, true);
}

/// Mixed shapes and materials (`Scene::Mixed`), 400-1200, locked and
/// turning. Locked: Box2D 170-290 (210), Rapier 160-230 (200); ours
/// 160-380 (250). Turning: Box2D 180-360 (290), Rapier 170-310 and one
/// never (270); ours 200-480 (260). Both sink about four times as deep
/// as ours (0.118 and 0.116 at the end, ours 0.027 and 0.020).
#[test]
fn mixed_piles_rest_as_soon_as_box2d_and_rapier_do() {
    let scenes: Vec<Scene> = record::MIXED.iter().map(|&n| Scene::Mixed { n, width: record::PILE_WIDTH }).collect();
    let b = Refs { rest: 210, first: 210, depth_end: 0.1180, during: 0.531, mean_during: 0.0456, energy: 1.4e-10 };
    let r = Refs { rest: 200, first: 200, depth_end: 0.1164, during: 0.595, mean_during: 0.0449, energy: 7.1e-9 };
    scenes_meet(&scenes, false, STEPS, &by_the_rules(b, r, false, 1.2, 30));
    let b = Refs { rest: 290, first: 290, depth_end: 0.0915, during: 0.526, mean_during: 0.0556, energy: 1.7e-7 };
    let r = Refs { rest: 270, first: 270, depth_end: 0.1124, during: 0.584, mean_during: 0.0567, energy: 4.0e-8 };
    scenes_meet(&scenes, true, STEPS, &by_the_rules(b, r, false, 1.6, 30));
}

/// One reference's statistics over a pyramid family: the medians of its
/// rests, of how far the top moved, of its depth at the end and of its
/// energy over the last `settle::TAIL` steps, and the worst lean.
struct PyramidRefs {
    rest: u32,
    top: f32,
    depth: f32,
    tilt: f32,
    energy: f64,
}

/// A pyramid family (`record::PYRAMID_FAMILIES`[`k`]) by the stand rules
/// on the medians over its widths: rest within twice the later
/// reference's, the top's move and the depth within half the smaller's,
/// the worst lean within the smaller's (or 1°), energy within ten times
/// the larger's, and all of them standing (no box past 5°, nothing out),
/// as all of the references' do.
fn pyramids_meet(k: usize, turning: bool, b: PyramidRefs, r: PyramidRefs) {
    let (bases, steps) = record::PYRAMID_FAMILIES[k];
    let scenes: Vec<Scene> = bases.iter().map(|&base| Scene::Pyramid { base }).collect();
    let runs = settle_all(&scenes, turning, steps);
    let mid = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let rest = mid(runs.iter().map(|r| r.rest_from.map_or(f64::INFINITY, f64::from)).collect());
    let top = mid(runs.iter().map(|r| r.top_moved.unwrap_or(f32::INFINITY) as f64).collect());
    let depth = mid(runs.iter().map(|r| r.end.max_depth as f64).collect());
    let energy = mid(runs.iter().map(|r| r.energy_tail).collect());
    let tilt = runs.iter().map(|r| r.end.tilt).fold(0.0, f32::max);
    let stand = runs.iter().filter(|r| r.end.tilt < 5.0 && r.end.escaped == 0).count();
    let name = format!("pyramids {:?}{}", bases, if turning { ", turning" } else { "" });
    let mut broken = Broken::default();
    let bound = 2.0 * b.rest.max(r.rest) as f64;
    broken.check(rest <= bound, || format!("{name}: median at rest from {rest}, bound {bound}"));
    let bound = 0.5 * b.top.min(r.top) as f64;
    broken.check(top <= bound, || format!("{name}: median top moved {top}, bound {bound}"));
    let bound = 0.5 * b.depth.min(r.depth) as f64;
    broken.check(depth <= bound, || format!("{name}: median depth {depth}, bound {bound}"));
    let bound = b.tilt.min(r.tilt).max(1.0);
    broken.check(tilt <= bound, || format!("{name}: a box leans {tilt}°, bound {bound}"));
    let bound = (10.0 * b.energy.max(r.energy)).max(STILL);
    broken.check(energy <= bound, || format!("{name}: median energy {energy:e} a body in the tail, bound {bound:e}"));
    broken.check(stand == runs.len(), || format!("{name}: {stand} of {} stand", runs.len()));
    broken.assert();
}

/// Pyramids 30-60 wide and 70-120 (`record::PYRAMID_FAMILIES`), locked:
/// ours at rest from 20-80 (median 50) and 80-200 (150); Box2D 70-90 (80)
/// and 70-110 (80); Rapier 160-260 (210) and 140-never (280). The tops
/// sink 0.054 and 0.267 (medians) where both references' sink 0.170 and
/// 0.834.
#[test]
fn pyramids_at_more_widths_stand_as_in_box2d_and_rapier() {
    let b = PyramidRefs { rest: 80, top: 0.170, depth: 0.0084, tilt: 0.0, energy: 7.3e-12 };
    let r = PyramidRefs { rest: 210, top: 0.170, depth: 0.0084, tilt: 0.0, energy: 6.4e-7 };
    pyramids_meet(0, false, b, r);
    let b = PyramidRefs { rest: 80, top: 0.834, depth: 0.0188, tilt: 0.0, energy: 4.1e-11 };
    let r = PyramidRefs { rest: 280, top: 0.834, depth: 0.0188, tilt: 0.0, energy: 2.0e-10 };
    pyramids_meet(1, false, b, r);
}

/// Turning: ours at rest from 40-190 (median 110) and 290-1200 (780),
/// growing about 10 steps a unit of width, so the 5050's 780
/// (get-emj.62) is in line with its neighbours; Box2D 50-90 (60) and
/// 90-310 (160); Rapier 80-330 (160) and 450-1650 (1100) (get-emj.64:
/// four to five times Box2D's, within the bound). The tops sink 0.051
/// and 0.258 where Box2D's sink 0.298 and 1.462.
#[test]
fn pyramids_that_turn_at_more_widths_stand_as_in_box2d_and_rapier() {
    let b = PyramidRefs { rest: 60, top: 0.298, depth: 0.0184, tilt: 0.8, energy: 4.1e-11 };
    let r = PyramidRefs { rest: 160, top: 0.306, depth: 0.0156, tilt: 1.1, energy: 1.0e-8 };
    pyramids_meet(0, true, b, r);
    let b = PyramidRefs { rest: 160, top: 1.462, depth: 0.0408, tilt: 1.8, energy: 6.1e-10 };
    let r = PyramidRefs { rest: 1100, top: 1.545, depth: 0.0350, tilt: 1.9, energy: 8.2e-8 };
    pyramids_meet(1, true, b, r);
}
