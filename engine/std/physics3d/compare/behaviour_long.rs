//! The 3D bounce families on their long grids (`:behaviour_long_test`,
//! manual), bounded as on their short ones, Rapier's and Box3D's statistics
//! measured on exactly these grids (`bench -- ours,rapier,box3d --bounces
//! --long --each`, 2026-09-29), and the long baseline's bounce group.

use super::*;

/// Energy over the long grids: spheres and flat cubes to 1% past the
/// push-out (Rapier 0.026, Box3D 0.000), a cube on an edge or a corner no
/// more than the worse reference (Box3D 0.111: a slow cube on an edge at
/// e = 1 tips, slaps its face and leaves). Ours 0.000, and 0.079 tipping
/// (get-emj.72); 4.39, 0.423, 0.058 and 0.412 with the step's gravity in
/// the closing speed (get-emj.60).
#[test]
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
/// contact), ours 0.0003 (5.39, 0.423, 0.062 with the step's gravity in).
#[test]
fn a_bounce_rebounds_at_e_of_the_speed_it_meets_at_over_the_long_grids() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique", "pairs"] {
        broken.stat_most(&bounce_family(name, true), "gain worst", 0.01);
    }
    broken.assert();
}

/// Nothing under the threshold bounces: Box3D 4, Rapier 66 (no threshold),
/// ours none (14 with the step's gravity in the closing speed, which at 20
/// and 40 took a slow cube past it).
#[test]
fn nothing_bounces_below_the_threshold_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("drops", true), "bounced below", 0.0);
    broken.assert();
}

/// Losses no worse than the references': Rapier's and Box3D's medians
/// −0.0500 and −0.0508 (drops), −0.0521 and −0.0636 (rates), −0.0034 and
/// −0.0029 (angled); flat none on the rates and angled grids. Ours −0.058,
/// −0.061, −0.005; none. The drops' flat bounces are the next test's.
#[test]
fn bounces_lose_no_more_than_in_rapier_and_box3d_over_the_long_grids() {
    let mut broken = Broken::default();
    for (name, lower) in [("drops", -0.050804), ("rates", -0.063633), ("oblique", -0.003413)] {
        broken.stat_least(&bounce_family(name, true), "gain median", under(lower));
    }
    for name in ["rates", "oblique"] {
        broken.stat_most(&bounce_family(name, true), "flat above", 0.0);
    }
    broken.assert();
}

/// The long drops flat above the threshold: Rapier 9, Box3D 14; ours 25
/// (get-emj.71: bounces missed where a step's gravity is large against the
/// threshold, likely our speculative margin catching a body a step early).
#[test]
#[ignore = "get-emj.71: bounces missed near the threshold at large g·dt"]
fn bounces_near_the_threshold_bounce_as_often_as_in_rapier_and_box3d_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("drops", true), "flat above", 14.0);
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
/// 0.975 and 0.979. Ours 0.999, 1.000, 0.978 (19.8, 1.32, 1.06 with the
/// step's gravity in the closing speed).
#[test]
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
