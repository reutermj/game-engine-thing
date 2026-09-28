//! Quality beyond settling: what a player sees a body do, on the behaviour
//! scenes (`behave.rs`): a box holding or sliding on a ramp, a disc rolling,
//! a ball bouncing, a heavy box on light ones, boxes spawned into each
//! other, a ball fired at a wall, a card house, a ladder and a row of
//! dominoes. Each bound comes from a hand calculation or from what Box2D
//! v3.1.1 and Rapier 2D 0.36 do on the same scene, recorded beside it and
//! measured with
//!
//!     BEHAVE=1 VARIANTS=rapier:ccd ./bazel run -c opt //engine/std/physics/compare
//!
//! (runbook 005; physics.md, "Quality beyond settling", has the tables and
//! says how each bound was set). Ours runs as the step on arrays, bit for
//! bit the mod's on these scenes too
//! (`the_mod_is_the_arrays_on_the_behaviour_scenes`), bodies turning;
//! `SOLVER=<variant>` runs one of `variants.rs` instead, as in
//! `:quality_test`.
//!
//! A test that finds a real problem stays, ignored, naming its bead: run
//! them with `--test_arg=--include-ignored`. Every value these tests bound,
//! the ignored ones' too, is also held to the baseline, both ways
//! (`baseline` below; `record.rs`; physics-testing.md, "The baseline"),
//! from the same runs (`runs.rs`).

#[allow(dead_code)] // `Arrays::snapshot`, which only `:tax` uses.
#[path = "../tests/arrays.rs"]
mod arrays;
mod baseline;
mod behave;
#[cfg(feature = "long")]
mod behaviour_long;
#[allow(dead_code)] // The timings and rain, which only the comparison reads.
mod ecs;
mod family;
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
mod quality;
#[allow(dead_code)] // The quality group's, which `:quality_test` checks.
mod record;
#[allow(dead_code)]
mod runs;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod settle;
#[allow(dead_code)]
mod sim;
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)] // The constants and tests only the experiments use.
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
mod variants;

pub use sim::{Dyn, Sim};

use behave::Behaviour;
use scene::Scene;

/// Each scene run by ours on arrays (or the variant `SOLVER` names),
/// bodies turning, in threads of their own, and printed as the comparison
/// prints it, so a failing run shows every number.
fn run(scenes: &[Scene]) -> Vec<Behaviour> {
    let runs = runs::par(scenes, |&scene| runs::behaved(scene));
    for (scene, r) in scenes.iter().zip(&runs) {
        let values: Vec<String> = r.values.iter().map(|(k, v)| format!("{k} {v:.4}")).collect();
        println!("{}, {}: {}", scene.text(), r.label, values.join(", "));
    }
    runs
}

/// Every bound a run broke, not the first, so a failure (or a planted bug)
/// says everything it changed.
#[derive(Default)]
struct Broken(Vec<String>);

impl Broken {
    fn check(&mut self, scene: &Scene, ok: bool, what: impl FnOnce() -> String) {
        let matrix = std::env::var_os("MATRIX").is_some();
        if !ok || matrix {
            let line = format!("{}: {}", scene.text(), what());
            if matrix {
                // Every bound's value, met or not, for the decision matrix
                // (physics.md, "Why colors let the pyramid fall").
                println!("CHECK {} | {} | {line}", std::thread::current().name().unwrap_or("?"), if ok { "ok" } else { "FAIL" });
            }
            if !ok {
                self.0.push(line);
            }
        }
    }

    /// `name` at most `bound`.
    fn most(&mut self, scene: &Scene, r: &Behaviour, name: &str, bound: f64) {
        let v = r.get(name);
        self.check(scene, v <= bound, || format!("{name} {v}, bound {bound}"));
    }

    /// `name` at least `bound`.
    fn least(&mut self, scene: &Scene, r: &Behaviour, name: &str, bound: f64) {
        let v = r.get(name);
        self.check(scene, v >= bound, || format!("{name} {v}, at least {bound}"));
    }

    fn assert(self) {
        assert!(self.0.is_empty(), "{} bounds broken:\n{}", self.0.len(), self.0.join("\n"));
    }
}

/// Measured over expected, less one.
fn off(r: &Behaviour) -> f64 {
    r.get("a") / r.get("expected a") - 1.0
}

/// A unit box on a ramp 20° steep, friction 0.6 (tan 20° = 0.36): it holds.
/// In 2 s, Box2D creeps 0.00008 down the slope, Rapier 0.0003; ours 0.0001.
/// The bound is three times Rapier's.
#[test]
fn a_box_below_the_friction_angle_holds_still() {
    let scene = Scene::Ramp { deg: 20.0, mu: 0.6, circle: false };
    let r = &run(&[scene])[0];
    let mut broken = Broken::default();
    broken.most(&scene, r, "crept", 3.0 * 0.0003);
    broken.assert();
}

/// A unit box on a ramp 30° steep, friction 0.2: it slides at g (sin θ −
/// μ cos θ) = 6.536, measured from its speed at steps 30 and 90. Box2D is
/// 0.17% fast, Rapier and ours exact to four digits; the bound is 1%.
#[test]
fn a_box_above_the_friction_angle_slides_at_g_sin_less_mu_g_cos() {
    let scene = Scene::Ramp { deg: 30.0, mu: 0.2, circle: false };
    let r = &run(&[scene])[0];
    let mut broken = Broken::default();
    broken.check(&scene, off(r).abs() <= 0.01, || format!("a {} against {}", r.get("a"), r.get("expected a")));
    broken.assert();
}

/// A disc on a ramp 30° steep, friction 0.6, above tan θ / 3 = 0.19: it
/// rolls without slipping at (2/3) g sin θ = 6.667. Box2D is 0.39% slow,
/// Rapier 0.49%, ours 0.21%; the bound is 1%. Its contact point slips at
/// 0.052 a second on average in both references, ours 0.073; the bound is
/// twice theirs. And friction 0.1, below it: the disc slips, and slides at
/// g (sin θ − μ cos θ) = 8.268 (Box2D 0.20% fast, Rapier 0.07%, ours 0.10%;
/// slipping at 4.8 a second in all three).
#[test]
fn a_disc_rolls_at_two_thirds_g_sin_and_slips_below_a_third_of_tan() {
    let (rolls, slips) = (Scene::Ramp { deg: 30.0, mu: 0.6, circle: true }, Scene::Ramp { deg: 30.0, mu: 0.1, circle: true });
    let runs = run(&[rolls, slips]);
    let mut broken = Broken::default();
    for (scene, r) in [rolls, slips].iter().zip(&runs) {
        broken.check(scene, off(r).abs() <= 0.01, || format!("a {} against {}", r.get("a"), r.get("expected a")));
    }
    broken.most(&rolls, &runs[0], "slip", 2.0 * 0.0522);
    broken.least(&slips, &runs[1], "slip", 0.9 * 4.805);
    broken.assert();
}

/// The first rebound of a ball dropped 5 onto a floor, as a share of the
/// drop, against e²: no engine reaches it at low restitution (the soft
/// contact takes some of the bounce), so the floor is a quarter further
/// below e² than the lower reference; and no rebound may pass e² by more
/// than 1%, which is energy from nowhere. Box2D and Rapier: 0.0376 at
/// e = 0.25 (e² = 0.0625), 0.2287 at 0.5 (0.25); ours 0.0437, 0.2442.
#[test]
fn a_ball_rebounds_to_about_e_squared_of_its_drop() {
    let cases = [(0.25, 0.0376), (0.5, 0.2287)];
    let scenes: Vec<Scene> = cases.iter().map(|&(e, _)| Scene::Bounce { e }).collect();
    let runs = run(&scenes);
    let mut broken = Broken::default();
    for ((scene, r), (e, lowest)) in scenes.iter().zip(&runs).zip(cases) {
        let e2 = (e * e) as f64;
        broken.least(scene, r, "first apex", e2 - 1.25 * (e2 - lowest));
        broken.most(scene, r, "first apex", 1.01 * e2);
    }
    broken.assert();
}

/// The same above e = 0.5, and a lossless ball (e = 1, no friction) over 20
/// s: Box2D and Rapier rebound to 0.548 at 0.75 (e² = 0.5625) and 0.996 at
/// 1, and the lossless ball keeps 0.92 of its height after 14 bounces. Ours
/// rebounds to 0.579 and 1.048, and the lossless ball climbs to 1.64 of its
/// drop in 12: each bounce gives back the speed it came in with plus the
/// step's gravity, which `integrate_velocities` added before the contact
/// was found (get-emj.56).
#[test]
#[ignore = "get-emj.56: a bounce gives back a step's gravity more than it came in with"]
fn a_ball_never_rebounds_higher_than_e_squared() {
    let scenes = [Scene::Bounce { e: 0.75 }, Scene::Bounce { e: 1.0 }];
    let runs = run(&scenes);
    let mut broken = Broken::default();
    broken.most(&scenes[0], &runs[0], "first apex", 1.01 * 0.5625);
    broken.most(&scenes[1], &runs[1], "most apex", 1.01);
    broken.least(&scenes[1], &runs[1], "last apex", 0.9 * 0.9194);
    broken.assert();
}

/// A heavy box on a light one, 10, 100 and 1000 times as heavy; one 100
/// times as heavy on a column of five; and Box2D's 20-wide box 400 times
/// as heavy on two unit boxes. Box2D and Rapier stand the first two and
/// the wide box, sinking the top 0.0097, 0.088 and 0.176, 0.0077, 0.071
/// and 0.141 deep; at 1000 to 1 both crush the light box flat (the top
/// 1.0 lower, 0.92-0.93 deep); on the column of five, Box2D topples it and
/// Rapier crushes one box (0.67 lower). Ours stands every one: the top
/// 0.0015, 0.014, 0.138, 0.107 and 0.028 lower, 0.0012, 0.011, 0.115, 0.024
/// and 0.023 deep. The bounds: half the smaller reference's sinking and
/// depth where they stand, and where they crush it, half a box lower and
/// a quarter of one deep.
#[test]
fn heavy_boxes_stand_on_light_ones_sinking_less_than_in_box2d_and_rapier() {
    // (scene, top sank, deepest at the end)
    let cases = [
        (Scene::Ratio { ratio: 10.0, light: 1 }, 0.5 * 0.00967, 0.5 * 0.00774),
        (Scene::Ratio { ratio: 100.0, light: 1 }, 0.5 * 0.0882, 0.5 * 0.0711),
        (Scene::Ratio { ratio: 1000.0, light: 1 }, 0.5, 0.25),
        (Scene::Ratio { ratio: 100.0, light: 5 }, 0.5 * 0.667, 0.25),
        (Scene::BigOnSmall, 0.5 * 0.1764, 0.5 * 0.1411),
    ];
    let scenes: Vec<Scene> = cases.iter().map(|c| c.0).collect();
    let runs = run(&scenes);
    let mut broken = Broken::default();
    for ((scene, sank, deepest), r) in cases.iter().zip(&runs) {
        broken.least(scene, r, "stands", 1.0);
        broken.most(scene, r, "escaped", 0.0);
        broken.most(scene, r, "top sank", *sank);
        broken.most(scene, r, "deepest at end", *deepest);
    }
    broken.assert();
}

/// The same come to rest: Box2D and Rapier at 3 and 2 steps at 10 to 1,
/// 23 and 23 at 100, 41 and 29 at 1000 (crushed), 253 and 68 on the column
/// of five, 34 and 33 under the wide box; the bound is twice the later.
/// Ours rests at 2, 24, 199, 48 and 69: at 1000 to 1 and under the wide
/// box past the bound, the heavy box and what it stands on jittering
/// (get-emj.57).
#[test]
#[ignore = "get-emj.57: a heavy box on light ones jitters for ever"]
fn heavy_boxes_on_light_ones_come_to_rest_as_soon_as_in_box2d_and_rapier() {
    let cases = [
        (Scene::Ratio { ratio: 10.0, light: 1 }, 3.0),
        (Scene::Ratio { ratio: 100.0, light: 1 }, 23.0),
        (Scene::Ratio { ratio: 1000.0, light: 1 }, 41.0),
        (Scene::Ratio { ratio: 100.0, light: 5 }, 253.0),
        (Scene::BigOnSmall, 34.0),
    ];
    let scenes: Vec<Scene> = cases.iter().map(|c| c.0).collect();
    let runs = run(&scenes);
    let mut broken = Broken::default();
    for ((scene, rest), r) in cases.iter().zip(&runs) {
        broken.most(scene, r, "at rest from", 2.0 * rest);
    }
    broken.assert();
}

/// A box 1000 times as heavy on a column of five: every engine crushes the
/// column (the top 5 lower), Box2D and Rapier with nothing faster than 4.1
/// a second in the last second and nothing out of the scene. Ours throws
/// four light boxes out through the floor, at up to 83 a second
/// (get-emj.58): with the default's tangent averaged over the substeps
/// (`solver::Carry::Normal`), as with both averaged; with both carried
/// from the last substep (`rot/carry=0`) nothing escapes. The bound is
/// twice the references' fastest.
#[test]
#[ignore = "get-emj.58: a heavy box crushing a column throws light boxes through the floor"]
fn a_heavy_box_crushing_a_column_throws_nothing_through_the_floor() {
    let scene = Scene::Ratio { ratio: 1000.0, light: 5 };
    let r = &run(&[scene])[0];
    let mut broken = Broken::default();
    broken.most(&scene, r, "escaped", 0.0);
    broken.most(&scene, r, "jitter", 2.0 * 4.131);
    broken.assert();
}

/// Box2D's overlap recovery: a pyramid 4 wide spawned a quarter and half a
/// box into each other, and one 10 wide at half: pushed apart at the capped
/// push-out speed (3 a second in all three), not flung. Box2D and Rapier:
/// fastest 3.04 and 3.47, 3.57 and 4.39, 16.8 and 16.5; apart (nothing
/// deeper than 0.01) at 67 and 46, 56 and 57, 75 and 101; at rest at 55
/// and 48, 54 and 58, 198 and 241; the 10-wide one topples in both (its top
/// 5.1 and 3.0 from where it rests, two boxes out in Box2D). Ours: 3.0,
/// 3.1, 11.8 fast; apart at 20, 33, 50; at rest at 24, 37, 130; its top
/// within 0.012 of where it rests. The bounds: a quarter over the smaller
/// fastest, twice the later step apart and at rest, and half the smaller
/// reference's top off where they stand (0.1 where they topple).
#[test]
fn overlapping_boxes_separate_at_the_capped_speed_without_exploding() {
    // (scene, fastest, apart, at rest, top off)
    let cases = [
        (Scene::Overlap { base: 4, overlap: 0.25 }, 3.039, 67.0, 55.0, 0.00315),
        (Scene::Overlap { base: 4, overlap: 0.5 }, 3.568, 57.0, 58.0, 0.0034),
        (Scene::Overlap { base: 10, overlap: 0.5 }, 16.46, 101.0, 241.0, 0.2),
    ];
    let scenes: Vec<Scene> = cases.iter().map(|c| c.0).collect();
    let runs = run(&scenes);
    let mut broken = Broken::default();
    for ((scene, fastest, apart, rest, top), r) in cases.iter().zip(&runs) {
        broken.most(scene, r, "peak speed", 1.25 * fastest);
        broken.most(scene, r, "separated at", 2.0 * apart);
        broken.most(scene, r, "at rest from", 2.0 * rest);
        let t = r.get("top off").abs();
        broken.check(scene, t <= 0.5 * top, || format!("top off {t}, bound {}", 0.5 * top));
        broken.most(scene, r, "escaped", 0.0);
    }
    broken.assert();
}

/// Bullet scenes at each speed, each at the four phases.
fn bullets(thick: f32, speeds: &[f32]) -> Vec<Scene> {
    speeds.iter().flat_map(|&speed| behave::BULLET_PHASES.map(|phase| Scene::Bullet { speed, radius: 0.25, thick, phase })).collect()
}

/// Ours has no continuous collision (physics.md, "Open questions"): a ball
/// is stopped only if some step leaves it within the speculative margin
/// (`narrow::MARGIN`, 0.05) of the wall or short of its middle, so it
/// bounces for certain while a step is at most the margin, its radius and
/// half the wall: 0.35, 21 a second, at radius 0.25 through a wall 0.1
/// thick; 0.8, 48 a second, through pong's paddle, a unit thick, which is
/// 20% over pong's fastest along the court (40). Past it, whether it
/// tunnels depends on where in a step it meets the wall. Pinned both ways,
/// every phase bouncing up to the limit and some tunnelling just past it,
/// so the limit moving is seen: continuous collision would move it, and
/// this test with it. Box2D (continuous on) and Rapier (as shipped: it
/// sweeps fast bodies against fixed colliders) bounce at every speed to
/// 400, the next test.
#[test]
fn a_ball_bounces_off_a_wall_while_a_step_is_within_the_margin_its_radius_and_half_the_wall() {
    let limit = |thick: f32| (narrow::MARGIN + 0.25 + thick / 2.0) / scene::DT;
    assert!((limit(0.1) - 21.0).abs() < 0.01 && (limit(1.0) - 48.0).abs() < 0.01, "the limits the speeds are chosen about");
    let mut broken = Broken::default();
    for (thick, below, past) in [(0.1, [10.0, 20.0, 21.0], 25.0), (1.0, [30.0, 40.0, 48.0], 50.0)] {
        let scenes = bullets(thick, &below);
        for (scene, r) in scenes.iter().zip(run(&scenes)) {
            broken.most(scene, &r, "through", 0.0);
            broken.least(scene, &r, "rebound", 0.99);
        }
        let scenes = bullets(thick, &[past]);
        let through: f64 = run(&scenes).iter().map(|r| r.get("through")).sum();
        broken.check(&scenes[0], through > 0.0, || format!("no phase tunnelled at {past}, just past the limit"));
    }
    broken.assert();
}

/// Past that limit ours tunnels at some phases, from 25 a second through
/// the thin wall and 50 through the paddle, at every phase through the
/// thin wall from 160; Box2D and Rapier never, at any speed to 400
/// (get-emj.59).
#[test]
#[ignore = "get-emj.59: no continuous collision: a ball tunnels once a step passes margin, radius and half the wall"]
fn a_ball_never_tunnels_through_a_wall_as_in_box2d_and_rapier() {
    let mut broken = Broken::default();
    for thick in [0.1, 1.0] {
        let scenes = bullets(thick, &behave::BULLET_SPEEDS);
        for (scene, r) in scenes.iter().zip(run(&scenes)) {
            broken.most(scene, &r, "through", 0.0);
        }
    }
    broken.assert();
}

/// A family (`family.rs`) on its short or long grid, whose runs did what
/// they should as often as the less reliable reference's did on exactly
/// that grid: `refs`, Box2D's and Rapier's counts (measured with
/// `FAMILIES=<name>` in the comparison, 2026-09-28). Near an edge of
/// stability a single run goes either way on rounding, in every engine, so
/// the share is what's bounded, as a pile's rest is by the median over its
/// sizes (physics-testing.md, "Families at the edge").
fn family_meets(name: &str, long: bool, refs: (usize, usize)) {
    let f = family::families(long).into_iter().find(|f| f.name == name).unwrap_or_else(|| panic!("no family {name}"));
    let runs = run(&f.scenes);
    let (yes, marks) = family::share(&f, &runs);
    let bound = refs.0.min(refs.1);
    println!("family {name}: {} {yes} of {} {marks}, bound {bound} (Box2D {}, Rapier {})", runs[0].label, runs.len(), refs.0, refs.1);
    let mut broken = Broken::default();
    broken.check(&f.scenes[0], yes >= bound, || format!("family {name}: {yes} of {}, at least {bound}", runs.len()));
    broken.assert();
}

/// Box2D's card house at 4 and 5 storeys, leaning 24-26°, friction 0.7
/// and 0.8 (12 houses): Box2D stands 10, Rapier 7, ours 11 (8 with both
/// impulses carried from the last substep); the 5-storey house at Box2D's
/// own 25° and 0.7 is one Rapier drops two cards of, and ours did too with
/// the last substep's tangent impulse carried (get-emj.61). Friction
/// halved, ours stands none.
#[test]
fn card_houses_stand_as_often_as_in_box2d_and_rapier() {
    family_meets("cards", false, (10, 7));
}

/// A box on a ramp 0.1° and 0.25° either side of friction 0.5's angle
/// holds below it and slides above it, in all three: every engine's
/// Coulomb friction on one contact is exact.
#[test]
fn a_box_near_the_friction_angle_holds_or_slides_as_it_should() {
    family_meets("ramp", false, (4, 4));
}

/// The ladder at 30° on friction 0.005 and 0.01 either side of what it
/// needs (0.269) stands or slides as it should, in all three.
#[test]
fn a_ladder_near_its_standing_friction_stands_or_slides_as_it_should() {
    family_meets("ladder", false, (4, 4));
}

/// Ten dominoes 1, 1.1, 1.2 and 1.3 apart (one reaches the next under
/// 1.25): all fall where each reaches the next and only the first at 1.3,
/// but at 1.2 the wave dies in Box2D and in ours, not in Rapier.
#[test]
fn dominoes_near_their_reach_topple_as_often_as_in_box2d_and_rapier() {
    family_meets("dominoes", false, (3, 4));
}

/// Stacks 18 and 22 high, turning, at the default substeps, over 10 s:
/// Rapier stands both, Box2D neither (it sways a stack of 12 over); ours
/// the 18. A record more than a bound, since Box2D's zero is its floor.
#[test]
fn stacks_near_their_buckling_height_stand_as_often_as_in_box2d_and_rapier() {
    family_meets("stacks", false, (0, 2));
}

/// A box 100 and 300 times as heavy on 3 and 5 unit boxes: Box2D stands
/// none, Rapier one, ours two.
#[test]
fn heavy_boxes_on_light_ones_stand_as_often_as_in_box2d_and_rapier() {
    family_meets("ratios", false, (0, 1));
}

/// A pyramid 20 wide at friction 0 and 0.1: every engine stands it at 0.1
/// and none at 0 (it needs friction only to hold what rounding sets
/// sliding).
#[test]
fn pyramids_across_friction_stand_as_often_as_in_box2d_and_rapier() {
    family_meets("pyramids", false, (1, 1));
}

/// A plank leaning 30° on a frictionless wall stands on the floor's
/// friction above tan θ / 2 − hx / (2 hy) = 0.269, and slides below: at
/// 0.4 and 0.3 no engine moves it more than 0.0004 (ours 0.00005); at 0.24
/// and 0.2 all three slide it 1.30-1.35 and lay it down. The bounds: ten
/// times Box2D's creep where it stands, and a unit where it slides.
#[test]
fn a_ladder_stands_on_friction_above_the_hand_calculation_and_slides_below() {
    let scenes: Vec<Scene> = [0.4, 0.3, 0.24, 0.2].map(|mu| Scene::Ladder { deg: 30.0, mu }).to_vec();
    let runs = run(&scenes);
    let mut broken = Broken::default();
    for (scene, r) in scenes.iter().zip(&runs) {
        let Scene::Ladder { mu, deg } = *scene else { unreachable!() };
        if mu >= Scene::ladder_mu(deg) {
            broken.most(scene, r, "slid", 10.0 * 0.0004);
        } else {
            broken.least(scene, r, "slid", 1.0);
        }
    }
    broken.assert();
}

/// Box2D's 15 dominoes, the first knocked over: every one falls, in order,
/// the wave running at 2.667 dominoes a second in Box2D and 2.736 in
/// Rapier, the last lying flat (90°), at rest from 434 and 443. Ours: all
/// 15 in order at 2.60, flat, at rest from 447. The bounds: every one in
/// order, the wave within 10% of the references' mean, the last down past
/// 80°, and twice the later rest.
#[test]
fn dominoes_topple_in_order_as_in_box2d_and_rapier() {
    let scene = Scene::Dominoes { n: 15, spacing: 1.0, mu: 0.6 };
    let r = &run(&[scene])[0];
    let mut broken = Broken::default();
    broken.least(&scene, r, "toppled", 15.0);
    broken.least(&scene, r, "in order", 1.0);
    let (wave, mean) = (r.get("wave"), (2.667 + 2.736) / 2.0);
    broken.check(&scene, (wave / mean - 1.0).abs() <= 0.1, || format!("wave {wave}, the references' {mean}"));
    broken.least(&scene, r, "last lean", 80.0);
    broken.most(&scene, r, "at rest from", 2.0 * 443.0);
    broken.assert();
}

/// Every bound here is on the arrays, so it must be the mod's step on these
/// scenes too, whose turned statics (ramps), statics with a material of
/// their own, starting spins and masses reach the mod by other paths than
/// the settling scenes': every body where the mod has it, bit for bit.
#[test]
fn the_mod_is_the_arrays_on_the_behaviour_scenes() {
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    let scenes = [
        Scene::Ramp { deg: 30.0, mu: 0.6, circle: true },
        Scene::Bounce { e: 0.5 },
        Scene::Ratio { ratio: 100.0, light: 1 },
        Scene::Ladder { deg: 30.0, mu: 0.2 },
        Scene::Dominoes { n: 15, spacing: 1.0, mu: 0.6 },
    ];
    let bits = |b: &Dyn| [b.x, b.y, b.vx, b.vy, b.angle, b.w].map(f32::to_bits);
    for scene in scenes {
        let (mut m, mut a) = (ecs::Ecs::new(&manifest, &scene, false, true), runs::ours(&scene, true, ""));
        m.step(120);
        a.step(120);
        let (mb, ab) = (m.bodies(), a.bodies());
        assert_eq!(mb.len(), ab.len());
        let differ = mb.iter().zip(&ab).filter(|(x, y)| bits(x) != bits(y)).count();
        assert_eq!(differ, 0, "{}: {differ} bodies differ from the arrays", scene.text());
    }
}

/// The default suite's baseline, group `behaviour`: every value
/// `record.rs` takes from these tests' scenes and families (the ignored
/// tests' scenes too, so that fixing what they found shows), each within
/// its band of `baseline.txt`, all of them listed where any moved.
#[test]
fn baseline() {
    baseline::assert_holds(record::DEFAULT, &["behaviour"], &record::behaviour(false), record::WRITE);
}

/// The long suite's: the families on their long grids (`baseline_long.txt`).
#[cfg(feature = "long")]
#[test]
fn baseline_long() {
    baseline::assert_holds(record::LONG, &["behaviour"], &record::behaviour(true), record::WRITE_LONG);
}
