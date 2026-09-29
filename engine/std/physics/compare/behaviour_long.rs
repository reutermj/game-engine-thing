//! The edge-of-stability families on their long grids
//! (`:behaviour_long_test`, manual, meant for -c opt), each bounded by the
//! less reliable reference's count on exactly that grid (`family_meets`),
//! measured with `FAMILIES=all FAMILY_LONG=1` in the comparison
//! (2026-09-28).

use super::*;

/// Card houses of 4, 5 and 6 storeys, leaning 23-27° by halves, friction
/// 0.65-0.9 (135): Box2D stands 104, Rapier 70, ours 105 (87 with both
/// impulses carried from the last substep).
#[test]
fn card_houses_stand_as_often_as_in_box2d_and_rapier_over_the_long_grid() {
    family_meets("cards", true, (104, 70));
}

/// A box 0.1-2° either side of the friction angle of 0.3, 0.5 and 0.7
/// (30): all three hold or slide as they should, every time.
#[test]
fn a_box_near_the_friction_angle_holds_or_slides_as_it_should_over_the_long_grid() {
    family_meets("ramp", true, (30, 30));
}

/// The ladder at 20°, 30° and 40° on friction 0.005-0.05 either side of
/// what it needs (24): all three right every time.
#[test]
fn a_ladder_near_its_standing_friction_stands_or_slides_as_it_should_over_the_long_grid() {
    family_meets("ladder", true, (24, 24));
}

/// Ten dominoes 0.8-1.4 apart at friction 0.3 and 0.6 (16): Box2D 13,
/// Rapier 14, ours 13 (the wave dies at 1.15 and 1.2).
#[test]
fn dominoes_near_their_reach_topple_as_often_as_in_box2d_and_rapier_over_the_long_grid() {
    family_meets("dominoes", true, (13, 14));
}

/// Stacks 12-28 high, turning (12): Rapier stands 10 (to 24), Box2D none
/// (it sways 12 over), ours 6 (to 20).
#[test]
fn stacks_near_their_buckling_height_stand_as_often_as_in_box2d_and_rapier_over_the_long_grid() {
    family_meets("stacks", true, (0, 10));
}

/// Heavy boxes 10-1000 times the mass on 1, 2, 3 and 5 unit boxes (44):
/// Box2D stands 21, Rapier 28, ours 34.
#[test]
fn heavy_boxes_on_light_ones_stand_as_often_as_in_box2d_and_rapier_over_the_long_grid() {
    family_meets("ratios", true, (21, 28));
}

/// A pyramid 20 wide at friction 0-0.6 (7): every engine stands all but
/// the frictionless one.
#[test]
fn pyramids_across_friction_stand_as_often_as_in_box2d_and_rapier_over_the_long_grid() {
    family_meets("pyramids", true, (6, 6));
}

// The wider families (`family::wide`, long suite only), each on exactly
// the grid its references were measured on (`FAMILIES=<name>
// FAMILY_LONG=1` in the comparison, 2026-09-28).

/// Card houses of 3 to 7 storeys, leaning 23-27° by degrees, friction
/// 0.6-0.9 (100): Box2D stands 72, Rapier 48, ours 69. By storeys, 3 to
/// 7, of 20 each: Box2D 19, 16, 15, 14, 8; Rapier 18, 12, 12, 5, 1; ours
/// 17, 15, 14, 13, 10.
#[test]
fn card_houses_stand_as_often_as_in_box2d_and_rapier_over_the_wide_grid() {
    family_meets("cards_wide", true, (72, 48));
}

/// Ten dominoes 0.8-1.45 apart by twentieths, skipping the reach (1.25),
/// at friction 0.3 and 0.6 (30): Box2D 25, Rapier 26, ours 25. The wave
/// dies at 1.15 on friction 0.3 and at 1.2 and 1.22 in all three, but for
/// Rapier's at 1.2 on 0.6.
#[test]
fn dominoes_near_their_reach_topple_as_often_as_in_box2d_and_rapier_over_the_fine_grid() {
    family_meets("dominoes_fine", true, (25, 26));
}

/// Stacks of every height from 12 to 30, turning (19): Rapier stands 14
/// (to 25), Box2D none, ours 9 (to 20), as on the long grid.
#[test]
fn stacks_near_their_buckling_height_stand_as_often_as_in_box2d_and_rapier_over_the_fine_grid() {
    family_meets("stacks_fine", true, (0, 14));
}

/// Heavy boxes 10-1000 times the mass on 1 to 6 unit boxes (84): Box2D
/// stands 33, Rapier 49, ours 60.
#[test]
fn heavy_boxes_on_light_ones_stand_as_often_as_in_box2d_and_rapier_over_the_fine_grid() {
    family_meets("ratios_fine", true, (33, 49));
}

/// A pyramid 20 wide at friction 0 to 0.8, finest near 0 (12): every
/// engine stands all but the frictionless one.
#[test]
fn pyramids_across_friction_stand_as_often_as_in_box2d_and_rapier_over_the_fine_grid() {
    family_meets("pyramids_fine", true, (11, 11));
}

// The bounce families on their long grids (`bounces.rs`), bounded as on
// their short ones (`behaviour_test.rs`), the references' statistics
// measured on exactly these grids (`BOUNCES=all BOUNCE_LONG=1`,
// 2026-09-29).

/// Every bounce of 9900 drops, 1280 at other rates and substeps and 1600
/// angled ones leaves with no more energy than it came in with (1%) past
/// the push-out's lift, nor does a box landing on a corner. Box2D and
/// Rapier: 0 but for corners, 0.091, which tip flat and throw themselves
/// up; ours 0.000 and 0.000 (12.4, 2.05, 0.127 and 0.148 with the step's
/// gravity in the closing speed, get-emj.56).
#[test]
fn bounces_leave_with_no_more_energy_than_they_came_in_with_over_the_long_grids() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique"] {
        broken.stat_most(&bounce_family(name, true), "excess worst", 0.01);
    }
    broken.stat_most(&bounce_family("drops", true), "excess tipping", 0.01);
    broken.assert();
}

/// Square-on bounces at e of the speed they meet at, to 1%, and free
/// pairs at e of their closing speed. Box2D 0.0008 at most; Rapier 1.0
/// (under the threshold). Ours 0.090 (drops) and 0.032 (rates), slow
/// bounces at low e that a speculative contact catches short of the floor
/// (get-emj.69); 13.4 and 2.48 with the step's gravity in the closing
/// speed (get-emj.56).
#[test]
#[ignore = "get-emj.69: a bounce caught short of the floor"]
fn a_bounce_rebounds_at_e_of_the_speed_it_meets_at_over_the_long_grids() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique", "pairs"] {
        broken.stat_most(&bounce_family(name, true), "gain worst", 0.01);
    }
    broken.assert();
}

/// Nothing under the threshold bounces: Box2D none, Rapier 535 (no
/// threshold), ours none (278 with the step's gravity in the closing
/// speed).
#[test]
fn nothing_bounces_below_the_threshold_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("drops", true), "bounced below", 0.0);
    broken.assert();
}

/// Losses no worse than the references': Box2D and Rapier medians −0.0633
/// and −0.0648 (drops), −0.132 and −0.134 (rates), −0.0009 and −0.0007
/// (angled); flat 534 and 365 (drops), 0 and 0 (angled). Ours −0.018,
/// −0.070, −0.000; 502, 0. The rates grid's flat bounces are the next
/// test's.
#[test]
fn bounces_lose_no_more_than_in_box2d_and_rapier_over_the_long_grids() {
    let mut broken = Broken::default();
    for (name, lower) in [("drops", -0.064799), ("rates", -0.133928), ("oblique", -0.000906)] {
        broken.stat_least(&bounce_family(name, true), "gain median", under(lower));
    }
    for (name, flat) in [("drops", 534.0), ("oblique", 0.0)] {
        broken.stat_most(&bounce_family(name, true), "flat above", flat);
    }
    broken.assert();
}

/// The rates grid's bounces flat above the threshold: Box2D 202, Rapier
/// 199; ours 207 (get-emj.71).
#[test]
#[ignore = "get-emj.71: bounces missed near the threshold at large g·dt"]
fn bounces_near_the_threshold_bounce_as_often_as_in_box2d_and_rapier_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("rates", true), "flat above", 202.0);
    broken.assert();
}

/// Angled bounces over the long grid: Box2D and Rapier keep a median 0.667
/// and 0.664 of their speed along the floor with friction, the most 0.973;
/// ours 0.681 and 0.975.
#[test]
fn an_angled_bounce_keeps_its_speed_along_the_floor_but_what_friction_takes_over_the_long_grid() {
    let f = bounce_family("oblique", true);
    let mut broken = Broken::default();
    broken.stat_most(&f, "slip worst", 0.01);
    broken.stat_most(&f, "tangent most", 1.01);
    broken.stat_least(&f, "tangent median", kept(0.664435));
    broken.assert();
}

/// Momentum through 1000 bounces of two free bodies: Box2D and Rapier lose
/// 0.00018, ours 0.0001.
#[test]
fn two_free_bodies_keep_their_momentum_through_a_bounce_over_the_long_grid() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("pairs", true), "momentum worst", 1e-3);
    broken.assert();
}

/// 20 s of bounces at three drops, gravity 20 and 80, 60 and 30 Hz.
/// Box2D and Rapier: highest 10.8 (at 30 Hz and gravity 80, where a ball
/// passes the surface in a free step, and bounces at the speed it gained
/// under it), most kept 1.52 and 1.53 of e², median 0.962. Ours 0.998,
/// 1.08 (a bounce caught short of the floor, get-emj.69), median 0.963;
/// 42.6, 1.88 and 1.22 with the step's gravity in the closing speed
/// (get-emj.56).
#[test]
#[ignore = "get-emj.69: a bounce caught short of the floor"]
fn a_lossless_ball_never_rises_and_a_ball_keeps_e_squared_of_its_height_over_the_long_grid() {
    let f = bounce_family("series", true);
    let mut broken = Broken::default();
    broken.stat_most(&f, "rise worst", 1.01);
    broken.stat_most(&f, "decay worst", 1.01);
    broken.stat_least(&f, "decay median", kept(0.961659));
    broken.assert();
}
