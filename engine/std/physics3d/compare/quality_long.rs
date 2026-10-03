//! The 3D quality tests at 1000 and 10 000 bodies (`:quality_long_test`,
//! manual: minutes at -c opt). One pile a size, since each is minutes in
//! every engine: the bounds are the same rules on one run, so they are
//! looser against chaos than the small scenes' over four sizes. The values
//! are single runs of the comparison (1000 steps at 1000, 1500 at 10 000,
//! 2026-09-26).

use super::*;

/// Turning cubes. 1000: at rest from Rapier 304, Box3D 299, Jolt 502, ours
/// 180; deepest at the end 0.022, 0.023 (ours 0.007); while settling 0.10,
/// 0.11 (ours 0.11), mean 0.026 and 0.026 (ours 0.007); energy 3.1e-9 and
/// 1.6e-10 (ours 3.5e-11). 10 000: at rest from 716, 713, 1413, ours 374;
/// deepest 0.042, 0.043 (ours 0.013); while settling 0.13, 0.14 (ours
/// 0.15), mean 0.032, 0.032 (ours 0.018); energy 9.2e-6 and 1.4e-6 (ours
/// 9.4e-10). Since recycling and softer static contacts (2026-09-27):
/// ours at rest from 182 and 468, 0.007 and 0.013 deep.
#[test]
fn big_piles_of_turning_boxes_rest_as_soon_as_rapier_and_box3d_do() {
    let (at_1000, at_10000) = big_turning_boxes();
    std::thread::scope(|s| {
        s.spawn(|| piles_meet(Kind::BoxPile, &[1000], true, &at_1000));
        s.spawn(|| piles_meet(Kind::BoxPile, &[10000], true, &at_10000));
    });
}

/// A known miss of the test above (`KNOWN`): colored, the pile of 10 000
/// lands deeper than Box3D's and a quarter; at three other seeds 0.207,
/// 0.157, 0.178 (by level 0.158 at this seed, then 0.167, 0.178, 0.152):
/// the order's, missed at three seeds of four.
#[test]
#[ignore = "get-emj.96: colored, turning boxes 10000 0.175 deep while settling, bound 0.172"]
fn known_miss_big_turning_boxes_land_no_deeper_than_box3d_and_a_quarter() {
    known_miss(Kind::BoxPile, &[10000], true, &big_turning_boxes().1, Bound::DeepestDuring(10000));
}

fn big_turning_boxes() -> (PileBounds, PileBounds) {
    let at_1000 = PileBounds {
        rest_worst: 2 * 304,
        rest_median: 380,
        deepest_end: 0.5 * 0.0224,
        deepest_during: 1.25 * 0.1059,
        mean_during: 0.0256,
        energy_end: 10.0 * 3.1e-9,
        partners: 2.0,
        not_columns: 0.8,
    };
    let at_10000 = PileBounds {
        rest_worst: 2 * 716,
        rest_median: 895,
        deepest_end: 0.5 * 0.0422,
        deepest_during: 1.25 * 0.1377,
        mean_during: 0.0319,
        energy_end: 10.0 * 9.2e-6,
        partners: 2.0,
        not_columns: 0.8,
    };
    (at_1000, at_10000)
}

/// Turning planks, until recycling 3D's weak spot. 1000: at rest from
/// Rapier 278, Box3D 262, Jolt 840, ours 408 (324 since, 2026-09-27); deepest at the end 0.030 and 0.034 (ours 0.010);
/// while settling 0.28 and 0.27 (ours 0.36), mean 0.019 and 0.017 (ours
/// 0.013); energy 5.6e-12 and 3.4e-13 (ours 1.0e-9). 10 000: at rest from
/// 392, 369, Jolt never; ours never (6 planks still at up to 0.34 at step
/// 1500; 335 since, energy 6.1e-10); deepest 0.076 and 0.078 (ours 0.024); while settling 0.51 and
/// 0.46 (ours 0.50), mean 0.025 and 0.025 (ours 0.014); energy 3.7e-11 and
/// 9.7e-13 (ours 2.0e-6).
#[test]
fn big_piles_of_turning_planks_rest_as_soon_as_rapier_and_box3d_do() {
    let (at_1000, at_10000) = big_turning_planks();
    std::thread::scope(|s| {
        s.spawn(|| piles_meet(Kind::PlankPile, &[1000], true, &at_1000));
        s.spawn(|| piles_meet(Kind::PlankPile, &[10000], true, &at_10000));
    });
}

/// A known miss of the test above (`KNOWN`): colored, the pile of 10 000
/// rests later than the later reference and a quarter (its median, of
/// one); at three other seeds 421, 1349, 1191 (by level 335 at this seed,
/// then 1152, 523, 362): missed at three seeds of four, by level at two,
/// so the bound sits inside both orders' spread.
#[test]
#[ignore = "get-emj.96: colored, turning planks 10000 at rest from 767, bound 490"]
fn known_miss_big_turning_planks_rest_as_soon_as_the_later_reference_and_a_quarter() {
    known_miss(Kind::PlankPile, &[10000], true, &big_turning_planks().1, Bound::RestMedian);
}

fn big_turning_planks() -> (PileBounds, PileBounds) {
    let at_1000 = PileBounds {
        rest_worst: 2 * 278,
        rest_median: 347,
        deepest_end: 0.5 * 0.0296,
        deepest_during: 1.25 * 0.2814,
        mean_during: 0.0165,
        energy_end: 10.0 * 5.6e-12,
        partners: 2.5,
        not_columns: 0.8,
    };
    let at_10000 = PileBounds {
        rest_worst: 2 * 392,
        rest_median: 490,
        deepest_end: 0.5 * 0.0763,
        deepest_during: 1.25 * 0.5108,
        mean_during: 0.0245,
        energy_end: 10.0 * 3.7e-11,
        partners: 2.5,
        not_columns: 0.8,
    };
    (at_1000, at_10000)
}

// The wider families (`record::WIDE`): more sizes, each its own seeded
// drop, bounded by the rules on Rapier's and Box3D's statistics over
// exactly the same sizes (Jolt's recorded beside them), measured with
// `bench -- <kind> <n> rapier,box3d,jolt [--rotate] --runs=1`, 2026-09-28
// (physics-testing.md, "Wider families").

/// One reference over a family: its median rest (`None` where most of its
/// runs never rest), and the worst depth at the end, while settling and
/// on average while settling, and energy a body, the worst of the runs at
/// rest (of all, where most breathe; `None` where none rest).
struct Refs {
    rest: Option<usize>,
    depth_end: f32,
    during: f32,
    mean_during: f32,
    energy: Option<f64>,
}

/// 2D's rules (`by_the_rules` there) on Rapier's and Box3D's statistics;
/// no bound on rest, or on energy, where a reference gives none.
fn by_the_rules(r: Refs, b: Refs, partners: f64) -> PileBounds {
    let later = match (r.rest, b.rest) {
        (Some(x), Some(y)) => Some(x.max(y)),
        _ => None,
    };
    let energy = match (r.energy, b.energy) {
        (Some(x), Some(y)) => 10.0 * x.max(y),
        _ => f64::INFINITY,
    };
    PileBounds {
        rest_worst: later.map_or(usize::MAX, |l| 2 * l),
        rest_median: later.map_or(usize::MAX, |l| l * 5 / 4),
        deepest_end: 0.5 * r.depth_end.min(b.depth_end),
        deepest_during: 1.25 * r.during.max(b.during),
        mean_during: r.mean_during.min(b.mean_during),
        energy_end: energy,
        partners,
        not_columns: 0.8,
    }
}

fn wide(k: usize) -> (Kind, bool, &'static [usize]) {
    record::WIDE[k]
}

fn wide_meet(k: usize, r: Refs, b: Refs, partners: f64) {
    let (kind, rotate, sizes) = wide(k);
    piles_meet(kind, sizes, rotate, &by_the_rules(r, b, partners));
}

/// Turning cubes, 200-1000 by 100: at rest from, Rapier 128-364 (median
/// 220), Box3D 117-710 (215), Jolt 240-never (568); ours 127-281 (189).
#[test]
fn piles_of_turning_boxes_at_nine_sizes_rest_as_soon_as_rapier_and_box3d_do() {
    let r = Refs { rest: Some(220), depth_end: 0.0224, during: 0.116, mean_during: 0.0293, energy: Some(3.1e-9) };
    let b = Refs { rest: Some(215), depth_end: 0.0226, during: 0.106, mean_during: 0.0291, energy: Some(3.4e-8) };
    wide_meet(0, r, b, 2.0);
}

/// Locked cubes, 200-1000: Rapier and Box3D rest at 200 and 300 (Box3D
/// 500 too) and breathe at the other sizes (docs/lore: a locked box pile
/// breathes under soft contacts at 4 iterations), so rest has no bound,
/// and energy's is from all their runs, 3.6e-2 and 3.7e-2 a body at 700;
/// Jolt rests 47-149; ours 42-92, but breathes at 700 too, at 1.5e-2.
#[test]
fn piles_of_locked_boxes_at_nine_sizes_sink_no_deeper_than_in_rapier_and_box3d() {
    let r = Refs { rest: None, depth_end: 0.0052, during: 0.111, mean_during: 0.0286, energy: Some(3.6e-2) };
    let b = Refs { rest: None, depth_end: 0.0049, during: 0.088, mean_during: 0.0268, energy: Some(3.7e-2) };
    wide_meet(1, r, b, 2.0);
}

/// Turning planks, 200-1000: Rapier 212-364 (267), Box3D 209-823 (262),
/// Jolt 187-never (907); ours 238-397 (288).
#[test]
fn piles_of_turning_planks_at_nine_sizes_rest_as_soon_as_rapier_and_box3d_do() {
    let r = Refs { rest: Some(267), depth_end: 0.0340, during: 0.335, mean_during: 0.0185, energy: Some(2.2e-11) };
    let b = Refs { rest: Some(262), depth_end: 0.0528, during: 0.356, mean_during: 0.0201, energy: Some(1.5e-12) };
    wide_meet(2, r, b, 2.5);
}

/// Mixed spheres and boxes (`Kind::Mixed`), turning, 200-1000: no engine
/// comes to rest in 1000 steps, the spheres rolling on (docs/lore), so
/// only depth is bounded: at the end Rapier up to 0.052, Box3D 0.111,
/// Jolt 0.057, ours 0.014.
#[test]
fn mixed_piles_that_turn_sink_no_deeper_than_in_rapier_and_box3d() {
    let r = Refs { rest: None, depth_end: 0.0519, during: 0.235, mean_during: 0.0120, energy: None };
    let b = Refs { rest: None, depth_end: 0.1111, during: 0.235, mean_during: 0.0126, energy: None };
    wide_meet(3, r, b, 2.0);
}

/// Mixed, locked: Rapier 187-354 (285), Box3D 232-347 (274), Jolt
/// 185-475 (308); ours 237-425 (297).
#[test]
fn mixed_piles_rest_as_soon_as_rapier_and_box3d_do() {
    let r = Refs { rest: Some(285), depth_end: 0.0355, during: 0.249, mean_during: 0.0224, energy: Some(1.1e-10) };
    let b = Refs { rest: Some(274), depth_end: 0.0293, during: 0.208, mean_during: 0.0226, energy: Some(8.1e-12) };
    wide_meet(4, r, b, 2.0);
}

/// Turning cubes, 2000-5000 by 1000 (1500 steps): Rapier 214-711 (288),
/// Box3D 206-439 (292), Jolt 528-never; ours 314, 505, 402, 298 (402,
/// the bound 365) (get-emj.65).
#[test]
#[ignore = "get-emj.65: turning cubes 2000-5000 rest later than Rapier and Box3D"]
fn bigger_piles_of_turning_boxes_rest_as_soon_as_rapier_and_box3d_do() {
    let r = Refs { rest: Some(288), depth_end: 0.0299, during: 0.099, mean_during: 0.0246, energy: Some(4.2e-9) };
    let b = Refs { rest: Some(292), depth_end: 0.0284, during: 0.109, mean_during: 0.0253, energy: Some(2.6e-10) };
    wide_meet(5, r, b, 2.0);
}

/// Turning planks, 2000-5000: Rapier 270-381 (376), Box3D 291-340 (314),
/// Jolt 1474-never; ours 320-546 (417).
#[test]
fn bigger_piles_of_turning_planks_rest_as_soon_as_rapier_and_box3d_do() {
    let (r, b) = bigger_turning_planks();
    wide_meet(6, r, b, 2.5);
}

fn bigger_turning_planks() -> (Refs, Refs) {
    let r = Refs { rest: Some(376), depth_end: 0.0685, during: 0.411, mean_during: 0.0227, energy: Some(4.6e-11) };
    let b = Refs { rest: Some(314), depth_end: 0.1258, during: 0.410, mean_during: 0.0221, energy: Some(1.2e-12) };
    (r, b)
}

/// Known misses of the test above (`KNOWN`): colored, the family's median
/// rest (316, 432, 559, 1327: 559) and the pile of 4000's (1327). At three
/// other seeds the medians 378, 771, 387 and the latest 1405, 989, 434;
/// by level 417 at this seed, then 401, 1416, 365, and the latest 546,
/// 1249, never, 376. The median missed at two seeds of four (by level
/// one), a pile past 752 at three (by level two): chaotic in both orders.
#[test]
#[ignore = "get-emj.96: colored, turning planks 2000-5000 at rest from a median 559, bound 470"]
fn known_miss_bigger_turning_planks_rest_as_soon_as_rapier_and_box3d_by_median() {
    let ((kind, rotate, sizes), (r, b)) = (wide(6), bigger_turning_planks());
    known_miss(kind, sizes, rotate, &by_the_rules(r, b, 2.5), Bound::RestMedian);
}

#[test]
#[ignore = "get-emj.96: colored, turning planks 4000 at rest from 1327, bound 752"]
fn known_miss_turning_planks_4000_rest_within_twice_the_later_reference() {
    let ((kind, rotate, sizes), (r, b)) = (wide(6), bigger_turning_planks());
    known_miss(kind, sizes, rotate, &by_the_rules(r, b, 2.5), Bound::Rest(4000));
}
