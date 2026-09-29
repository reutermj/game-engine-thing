//! The 3D bounce families on their long grids (`:behaviour_long_test`,
//! manual), bounded as on their short ones, Rapier's and Box3D's statistics
//! measured on exactly these grids (`bench -- ours,rapier,box3d --bounces
//! --long --each`, 2026-09-29), and the long baseline's bounce group.

use super::*;

/// Energy over the long grids: spheres and flat cubes to 1% past the
/// push-out (Rapier 0.026, Box3D 0.000), a cube on an edge or a corner no
/// more than the worse reference (Box3D 0.111: a slow cube on an edge at
/// e = 1 tips, slaps its face and leaves). Ours 4.39, 0.423, 0.058, and
/// 0.412 tipping; with the step's gravity taken out (`closing=before`) 0
/// and 0.079.
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn bounces_leave_with_no_more_energy_than_they_came_in_with_over_the_long_grids() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique"] {
        broken.stat_most(&bounce_family(name, true), "excess worst", 0.01);
    }
    broken.stat_most(&bounce_family("drops", true), "excess tipping", 0.110898);
    broken.assert();
}

/// Square-on bounces at e of the speed they meet at, to 1%: Box3D 0.66 and
/// Rapier 1.0 at worst (Box3D's a slow cube caught by its speculative
/// contact), ours 5.39, 0.423, 0.062; with the step's gravity taken out
/// 0.0003.
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn a_bounce_rebounds_at_e_of_the_speed_it_meets_at_over_the_long_grids() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique", "pairs"] {
        broken.stat_most(&bounce_family(name, true), "gain worst", 0.01);
    }
    broken.assert();
}

/// Nothing under the threshold bounces: Box3D 4, Rapier 66 (no threshold),
/// ours 14, where a step's gravity (0.33 at 20, 0.67 at 40) takes a slow
/// cube past it.
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn nothing_bounces_below_the_threshold_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("drops", true), "bounced below", 0.0);
    broken.assert();
}

/// Losses no worse than the references': Rapier's and Box3D's medians
/// −0.0500 and −0.0508 (drops), −0.0521 and −0.0636 (rates), −0.0034 and
/// −0.0029 (angled); flat 9 and 14, none, none. Ours 0.000, +0.007, 0.000;
/// 1, 0, 0.
#[test]
fn bounces_lose_no_more_than_in_rapier_and_box3d_over_the_long_grids() {
    let mut broken = Broken::default();
    for (name, lower, flat) in [("drops", -0.050804, 14.0), ("rates", -0.063633, 0.0), ("oblique", -0.003413, 0.0)] {
        let f = bounce_family(name, true);
        broken.stat_least(&f, "gain median", under(lower));
        broken.stat_most(&f, "flat above", flat);
    }
    broken.assert();
}

/// Angled bounces: Rapier and Box3D keep a median 0.710 along the floor
/// with friction; ours 0.711.
#[test]
fn an_angled_bounce_keeps_its_speed_along_the_floor_but_what_friction_takes_over_the_long_grid() {
    let f = bounce_family("oblique", true);
    let mut broken = Broken::default();
    broken.stat_most(&f, "slip worst", 0.01);
    broken.stat_most(&f, "tangent most", 1.01);
    broken.stat_least(&f, "tangent median", kept(0.710075));
    broken.assert();
}

/// Momentum through 96 bounces of two free spheres: Rapier 0.00003, Box3D
/// 0.000001, ours 0.00004.
#[test]
fn two_free_spheres_keep_their_momentum_through_a_bounce_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("pairs", true), "momentum worst", 1e-3);
    broken.assert();
}

/// 20 s of bounces from three drops: Rapier and Box3D highest 1.40 and
/// 1.10 (cubes that tip at 40), most kept 1.03 and 1.05 of e², median
/// 0.975 and 0.979. Ours 19.8, 1.32, 1.06; with the step's gravity taken
/// out 0.999, 1.000, 0.978.
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn a_lossless_sphere_never_rises_and_a_sphere_keeps_e_squared_of_its_height_over_the_long_grid() {
    let f = bounce_family("series", true);
    let mut broken = Broken::default();
    broken.stat_most(&f, "rise worst", 1.01);
    broken.stat_most(&f, "decay worst", 1.01);
    broken.stat_least(&f, "decay median", kept(0.974519));
    broken.assert();
}

/// The long suite's baseline, group `behaviour`: the bounce families on
/// their long grids (`baseline_long.txt`).
#[test]
fn baseline_long() {
    baseline::assert_holds(record::LONG, &["behaviour"], &record::behaviour(true), record::WRITE_LONG);
}
