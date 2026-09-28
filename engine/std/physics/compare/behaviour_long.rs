//! The edge-of-stability families on their long grids
//! (`:behaviour_long_test`, manual, meant for -c opt), each bounded by the
//! less reliable reference's count on exactly that grid (`family_meets`),
//! measured with `FAMILIES=all FAMILY_LONG=1` in the comparison
//! (2026-09-28).

use super::*;

/// Card houses of 4, 5 and 6 storeys, leaning 23-27° by halves, friction
/// 0.65-0.9 (135): Box2D stands 104, Rapier 70, ours 87.
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
