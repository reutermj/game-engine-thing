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
/// 9.4e-10).
#[test]
fn big_piles_of_turning_boxes_rest_as_soon_as_rapier_and_box3d_do() {
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
    std::thread::scope(|s| {
        s.spawn(|| piles_meet(Kind::BoxPile, &[1000], true, &at_1000));
        s.spawn(|| piles_meet(Kind::BoxPile, &[10000], true, &at_10000));
    });
}

/// Turning planks, 3D's weak spot. 1000: at rest from Rapier 278, Box3D
/// 262, Jolt 840, ours 408; deepest at the end 0.030 and 0.034 (ours 0.010);
/// while settling 0.28 and 0.27 (ours 0.36), mean 0.019 and 0.017 (ours
/// 0.013); energy 5.6e-12 and 3.4e-13 (ours 1.0e-9). 10 000: at rest from
/// 392, 369, Jolt never; ours never (6 planks still at up to 0.34 at step
/// 1500); deepest 0.076 and 0.078 (ours 0.024); while settling 0.51 and
/// 0.46 (ours 0.50), mean 0.025 and 0.025 (ours 0.014); energy 3.7e-11 and
/// 9.7e-13 (ours 2.0e-6).
#[test]
#[ignore = "known failure, get-emj.43: 1000 turning planks rest at 408 where Rapier's rest at 278, and 10 000 never do where theirs rest by 392"]
fn big_piles_of_turning_planks_rest_as_soon_as_rapier_and_box3d_do() {
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
    std::thread::scope(|s| {
        s.spawn(|| piles_meet(Kind::PlankPile, &[1000], true, &at_1000));
        s.spawn(|| piles_meet(Kind::PlankPile, &[10000], true, &at_10000));
    });
}
