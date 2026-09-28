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
