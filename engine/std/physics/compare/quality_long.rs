//! The quality tests at 10 000 bodies (`:quality_long_test`, manual: about
//! a minute and a half at -c opt, and much longer without). The bounds are
//! set by the same rules as the small scenes', from the references on these
//! scenes.

use super::*;

/// Long enough to see the references stay at rest, or not: at this size
/// their turning piles don't (below).
const LONG_STEPS: u32 = 2500;

/// The comparison's big piles, 401 wide, at three sizes.
const BIG_PILES: [u32; 3] = [9000, 10000, 11000];

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
    piles_meet(&BIG_PILES, 401.0, false, LONG_STEPS, &b);
}

/// Turning: no engine stays at rest, a body now and then moving over 0.05
/// again long after the pile came to rest (which sleeping would take in its
/// stride). At rest from: Box2D 1550, 1760, 2300 (median 1760); Rapier
/// 2490, 2270, never (2490); ours 330, 350, 1620. So rest from is bounded
/// by the references' medians themselves (the worst of ours by the later,
/// the median by the earlier), and the first look at rest by the rules:
/// Box2D 330, 470, 280 (median 330), Rapier 240, 430, 520 (430); ours 330,
/// 350, 450. Deepest at the end: Box2D up to 0.126, Rapier 0.095, ours
/// 0.027. While settling: 0.47 and 0.45 (ours 0.41), mean 0.052 and 0.052
/// (ours 0.018). Energy at the end: 4.1e-8 and 6.7e-7 (ours 5.3e-8).
/// Contacts a body 2.02-2.07, 1-4 islands (ours 1.95-1.97, 3-5).
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
    piles_meet(&BIG_PILES, 401.0, true, LONG_STEPS, &b);
}

/// The pyramid 100 wide (5050 boxes): ours at rest from 150, its top 0.27
/// lower, 0.006 deep; turning, 440, 0.26, 0.006, leaning 0.3°.
#[test]
fn a_big_pyramid_stands_as_in_box2d_and_rapier() {
    let locked = from_refs((70, 280), (0.834, 0.834), (0.0188, 0.0188), (0.0, 0.0), (4.1e-11, 1.9e-10));
    stand(&[(Scene::Pyramid { base: 100 }, locked)], false, LONG_STEPS);
    let turning = from_refs((160, 1100), (1.462, 1.545), (0.0408, 0.0350), (1.5, 1.8), (5.9e-10, 8.2e-8));
    stand(&[(Scene::Pyramid { base: 100 }, turning)], true, LONG_STEPS);
}

/// Kinetic energy a body at the end: ten times the references' worst.
const ENERGY_LOCKED: f64 = 10.0 * 3.1e-8;
const ENERGY_TURNING: f64 = 10.0 * 6.7e-7;
