//! Quality as a test: how soon our 2D step brings a scene to rest, how deep
//! it sinks while it does and once it has, how much energy is left, whether
//! stacks and pyramids stand, and that sleeping then engages, on the
//! comparison's scenes (`scene.rs`), measured by its code (`settle.rs`,
//! `quality.rs`), against bounds derived from what Box2D v3.1.1 and Rapier
//! 0.36 meet on the same scenes. The references aren't linked here: their
//! values are recorded beside each bound, measured with
//!
//!     ENGINES=arrays,box2d,rapier SETTLE=700 SCENES=... ./bazel run -c opt //engine/std/physics/compare
//!
//! (runbook 005), and physics.md, "Quality as a test", says how each bound
//! was set from them and how to refresh them.
//!
//! Ours runs as the step on arrays (`tests/arrays.rs`), bit for bit the
//! mod's (`the_mod_is_the_arrays_bit_for_bit` holds that here), so another
//! solver can be put in its place: `SOLVER=<spec>` in the environment runs
//! one of `variants.rs` instead (`--test_env=SOLVER=split`, the split
//! impulse this step replaced), which is how the bounds were checked to
//! fail on the solvers they should.
//!
//! Settling is chaotic (docs/lore: a pile's step to rest moves by hundreds
//! with rounding alone), so piles run at five sizes and are bounded on the
//! worst of them and their median, never on one run. Pyramids and stacks
//! stand and don't vary so: one scene each. The 10 000-body scenes are
//! `:quality_long_test` (manual), with `--features long`.

#[allow(dead_code)] // `Arrays::snapshot`, which only `:tax` uses.
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)] // The timings and rain, which only the comparison reads.
mod ecs;
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
mod quality;
#[cfg(feature = "long")]
mod quality_long;
#[allow(dead_code)]
mod scene;
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

use scene::Scene;
use settle::Settling;

/// Our step on arrays, or the variant `SOLVER` names.
fn ours(scene: &Scene, turning: bool) -> ecs::Flat {
    match std::env::var("SOLVER") {
        Ok(spec) if !spec.is_empty() => ecs::Flat::variant(scene, turning, &spec, &format!("ours ({spec})")),
        _ => ecs::Flat::new(scene, turning, Box::new(solver::solve_points), "ours"),
    }
}

/// Steps every scene is run: past the latest bound on rest (a turning
/// pile's, 500), so "at rest from" means it stayed so for a while. The
/// references were measured over the same steps.
const STEPS: u32 = 700;

/// Each scene settled, in threads of their own (the test's time is its
/// slowest scene's), and printed as the comparison prints it, so a failing
/// run shows every number, not just the one that failed.
fn settle_all(scenes: &[Scene], turning: bool, steps: u32) -> Vec<Settling> {
    let runs: Vec<Settling> = std::thread::scope(|s| {
        let threads: Vec<_> =
            scenes.iter().map(|&scene| s.spawn(move || settle::settle(&mut ours(&scene, turning), &scene, turning, steps))).collect();
        threads.into_iter().map(|t| t.join().expect("a scene panicked")).collect()
    });
    print_runs(scenes, turning, &runs);
    runs
}

fn print_runs(scenes: &[Scene], turning: bool, runs: &[Settling]) {
    let at = |s: Option<u32>| s.map_or("never".to_string(), |s| s.to_string());
    for (scene, r) in scenes.iter().zip(runs) {
        let q = &r.end;
        let top = r.top_moved.map_or(String::new(), |t| format!(", top moved {t:.4}"));
        println!(
            "{}{}: {}, at rest {} / from {}; deepest {:.4} at end, {:.4} during, mean {:.4} during; energy {:.1e}; {:.2} contacts a body, {} islands; tilt {:.1}°, {} escaped{top}",
            scene.text(),
            if turning { ", turning" } else { "" },
            r.label,
            at(r.first_rest),
            at(r.rest_from),
            q.max_depth,
            r.deepest_during,
            r.mean_during,
            q.energy,
            q.contacts_per_body,
            q.islands,
            q.tilt,
            q.escaped,
        );
    }
}

fn median(mut v: Vec<u32>) -> u32 {
    v.sort();
    v[v.len() / 2]
}

/// What a family of piles (one scene at several sizes) must meet, each
/// bound set from the references' values on the same piles (physics.md,
/// "Quality as a test", has how, and the values):
struct PileBounds {
    /// Steps to rest (the look from which every body stays under 0.05),
    /// the worst of the sizes: twice the references' typical rest, the later
    /// of each engine's median over the sizes. Medians, since each engine's
    /// rest moves by 100-200 steps from size to size with rounding alone
    /// (docs/lore), theirs as much as ours; twice, since ours does too.
    rest_worst: u32,
    /// And the median of ours: a quarter over theirs.
    rest_median: u32,
    /// The same rules on the first look at rest, worst and median, where
    /// the references don't stay at rest (the 10 000 turning piles, in
    /// `quality_long.rs`), so their rest from says little.
    first_rest: Option<(u32, u32)>,
    /// The deepest overlap once at rest: half the shallower reference's
    /// worst. Stiffer contacts than theirs (5 substeps at 75 Hz against 4
    /// at 30) are a design choice (physics.md, "Settling"), and a bound at
    /// their depth would let it go unnoticed.
    deepest_end: f32,
    /// The deepest at any look, landing included: a quarter over the
    /// references' worst, since each lands as deep (push-out capped at 3
    /// u/s in all three).
    deepest_during: f32,
    /// The greatest mean overlap at any look: the shallower reference's
    /// worst.
    mean_during: f64,
    /// Kinetic energy a body at the end: ten times the references' worst.
    energy_end: f64,
    /// That it is a pile, not columns (docs/lore: a pile 41 wide stands in
    /// columns): columns are 1.0 contacts a body, an island a column.
    contacts_per_body: f64,
    islands: usize,
}

/// Every bound a run broke: all of them, not the first, so a failure (or
/// a planted bug) says everything it changed.
#[derive(Default)]
struct Broken(Vec<String>);

impl Broken {
    fn check(&mut self, ok: bool, what: impl FnOnce() -> String) {
        if !ok {
            self.0.push(what());
        }
    }

    fn assert(self) {
        assert!(self.0.is_empty(), "{} bounds broken:\n{}", self.0.len(), self.0.join("\n"));
    }
}

/// The look a run stayed at rest from, or never (past any bound).
fn rest(r: &Settling) -> u32 {
    r.rest_from.unwrap_or(u32::MAX)
}

fn piles_meet(sizes: &[u32], width: f32, turning: bool, steps: u32, b: &PileBounds) {
    let scenes: Vec<Scene> = sizes.iter().map(|&n| Scene::Pile { n, width, stagger: true }).collect();
    let runs = settle_all(&scenes, turning, steps);
    let mut broken = Broken::default();
    for (scene, r) in scenes.iter().zip(&runs) {
        let (q, name) = (&r.end, scene.text());
        broken.check(q.contacts_per_body >= b.contacts_per_body && q.islands <= b.islands, || format!("{name}: not a pile: {q:?}"));
        broken.check(q.escaped == 0, || format!("{name}: {} escaped", q.escaped));
        broken.check(rest(r) <= b.rest_worst, || format!("{name}: at rest from {:?}, bound {}", r.rest_from, b.rest_worst));
        broken.check(q.max_depth <= b.deepest_end, || format!("{name}: {} deep at rest, bound {}", q.max_depth, b.deepest_end));
        broken.check(r.deepest_during <= b.deepest_during, || {
            format!("{name}: {} deep while settling, bound {}", r.deepest_during, b.deepest_during)
        });
        broken.check(r.mean_during <= b.mean_during, || {
            format!("{name}: {} deep on average while settling, bound {}", r.mean_during, b.mean_during)
        });
        let energy = b.energy_end.max(STILL);
        broken.check(q.energy <= energy, || format!("{name}: energy {:e} a body at the end, bound {energy:e}", q.energy));
    }
    if let Some((worst, median_bound)) = b.first_rest {
        let firsts: Vec<u32> = runs.iter().map(|r| r.first_rest.unwrap_or(u32::MAX)).collect();
        let m = median(firsts.clone());
        broken.check(firsts.iter().all(|&f| f <= worst), || format!("first at rest {firsts:?}, bound {worst}"));
        broken.check(m <= median_bound, || format!("first at rest {firsts:?}: median {m}, bound {median_bound}"));
    }
    let rests: Vec<u32> = runs.iter().map(rest).collect();
    let m = median(rests.clone());
    broken.check(m <= b.rest_median, || format!("at rest from {rests:?}: median {m}, bound {}", b.rest_median));
    broken.assert();
}

/// What a pyramid or a stack must meet. These stand, and don't move with
/// rounding as a pile does (docs/lore), so each is one scene, and its bounds
/// come from the references on that scene alone:
struct StandBounds {
    /// Twice the later reference's rest.
    rest: u32,
    /// How far the top box sank or slid, and the deepest overlap at the
    /// end: half the smaller reference's (the stiffness `PileBounds` says).
    top_moved: f32,
    deepest_end: f32,
    /// The most any box leans, in degrees: the smaller reference's, or 1°.
    tilt: f32,
    /// Kinetic energy a body, the most at any look over the last
    /// `settle::TAIL` steps: ten times the larger reference's. Not at the
    /// end alone: a tall stack sways, in Rapier as in ours, and its energy
    /// at one step says where in the swing it was (Rapier's 20-high turning
    /// stack has 1.5e-7 at step 700 and 2.8e-4 twenty steps before).
    energy_tail: f64,
}

/// Energies under this are rounding, not motion: every body slower than
/// about 1e-4, a five-hundredth of the sleep threshold. Where ten times a
/// reference's energy is below it (a pyramid at rest, 1e-13 to 1e-9 a
/// body in every engine), the bound is this.
const STILL: f64 = 1e-8;

fn stand(cases: &[(Scene, StandBounds)], turning: bool, steps: u32) {
    let scenes: Vec<Scene> = cases.iter().map(|(s, _)| *s).collect();
    stand_runs(cases, &settle_all(&scenes, turning, steps));
}

/// Each run against its case's bounds.
fn stand_runs(cases: &[(Scene, StandBounds)], runs: &[Settling]) {
    let mut broken = Broken::default();
    for (r, (scene, b)) in runs.iter().zip(cases) {
        let (q, name) = (&r.end, scene.text());
        broken.check(q.escaped == 0, || format!("{name}: {} escaped", q.escaped));
        broken.check(rest(r) <= b.rest, || format!("{name}: at rest from {:?}, bound {}", r.rest_from, b.rest));
        let top = r.top_moved.expect("a stack or a pyramid");
        broken.check(top <= b.top_moved, || format!("{name}: its top moved {top}, bound {}", b.top_moved));
        broken.check(q.max_depth <= b.deepest_end, || format!("{name}: {} deep at rest, bound {}", q.max_depth, b.deepest_end));
        broken.check(q.tilt <= b.tilt, || format!("{name}: a box leans {}°, bound {}", q.tilt, b.tilt));
        let energy = b.energy_tail.max(STILL);
        broken.check(r.energy_tail <= energy, || {
            format!("{name}: energy {:e} a body in the last {} steps, bound {energy:e}", r.energy_tail, settle::TAIL)
        });
    }
    broken.assert();
}

/// The piles: the comparison's, 41 wide, staggered, rotation locked.
const PILES: [u32; 5] = [400, 600, 800, 1000, 1200];

/// At rest from, over `PILES` (700 steps, 2026-09-26): Box2D 140, 140,
/// 260, 260, 200 (median 200); Rapier 170, 130, 130, 160, 220 (160); ours
/// 100, 140, 210, 230, 240. Deepest at the end: Box2D up to 0.067, Rapier
/// 0.057; ours 0.014. While settling: deepest 0.36 and 0.32 (ours 0.31),
/// mean 0.061 and 0.058 (ours 0.028). Energy at the end: 1.5e-10 and
/// 4.3e-9 (ours 5.4e-9). Contacts a body 1.44-1.58 and 1-5 islands in both
/// (ours 1.34-1.40, 2).
#[test]
fn real_piles_rest_as_soon_as_box2d_and_rapier_do() {
    let b = PileBounds {
        rest_worst: 2 * 200,
        rest_median: 250,
        first_rest: None,
        deepest_end: 0.5 * 0.0565,
        deepest_during: 1.25 * 0.3592,
        mean_during: 0.0579,
        energy_end: 10.0 * 4.3e-9,
        contacts_per_body: 1.2,
        islands: 5,
    };
    piles_meet(&PILES, 41.0, false, STEPS, &b);
}

/// The same piles, turning: at rest from, Box2D 170, 160, 210, 240, 250
/// (median 210); Rapier 160, 400, 250, 230, 310 (250); ours 230, 220, 400,
/// 440, 290. Deepest at the end: Box2D up to 0.136, Rapier 0.099; ours
/// 0.024. While settling: deepest 0.53 and 0.48 (ours 0.44), mean 0.056
/// and 0.055 (ours 0.020). Energy at the end: 1.3e-8 and 3.3e-8 (ours
/// 1.6e-7). Contacts a body 1.90-2.09, 1-3 islands (ours 1.89-1.95, 1-2).
#[test]
fn real_piles_that_turn_rest_as_soon_as_box2d_and_rapier_do() {
    let b = PileBounds {
        rest_worst: 2 * 250,
        rest_median: 312,
        first_rest: None,
        deepest_end: 0.5 * 0.0985,
        deepest_during: 1.25 * 0.5281,
        mean_during: 0.0552,
        energy_end: 10.0 * 3.3e-8,
        contacts_per_body: 1.6,
        islands: 5,
    };
    piles_meet(&PILES, 41.0, true, STEPS, &b);
}

/// `StandBounds` from the references on one scene: the later rest, the
/// smaller top move, depth and tilt, the larger energy, of Box2D's and
/// Rapier's (in that order in each pair).
fn from_refs(rest: (u32, u32), top: (f32, f32), deepest: (f32, f32), tilt: (f32, f32), energy: (f64, f64)) -> StandBounds {
    StandBounds {
        rest: 2 * rest.0.max(rest.1),
        top_moved: 0.5 * top.0.min(top.1),
        deepest_end: 0.5 * deepest.0.min(deepest.1),
        tilt: tilt.0.min(tilt.1).max(1.0),
        energy_tail: 10.0 * energy.0.max(energy.1),
    }
}

/// Pyramids of unit boxes 15, 20 and 25 wide (120, 210 and 325 boxes),
/// rotation locked. The references' values (700 steps, 2026-09-26; energy
/// the most over the last 200, 2026-09-27) are the arguments; ours: at rest from 10, 10, 20, the top 0.006, 0.011, 0.017
/// lower, 0.0009-0.0015 deep.
#[test]
fn pyramids_stand_as_in_box2d_and_rapier() {
    stand(
        &[
            (Scene::Pyramid { base: 15 }, from_refs((30, 30), (0.019, 0.019), (0.0027, 0.0027), (0.0, 0.0), (7.4e-11, 3.3e-10))),
            (Scene::Pyramid { base: 20 }, from_refs((40, 50), (0.034, 0.034), (0.0036, 0.0036), (0.0, 0.0), (2.4e-13, 4.9e-12))),
            (Scene::Pyramid { base: 25 }, from_refs((60, 100), (0.053, 0.053), (0.0046, 0.0046), (0.0, 0.0), (2.7e-13, 3.2e-12))),
        ],
        false,
        STEPS,
    );
}

/// The same, turning: ours at rest from 10, 20, 30, the top 0.006, 0.011,
/// 0.017 lower, 0.0008-0.0014 deep, leaning at most 0.1°.
#[test]
fn pyramids_that_turn_stand_as_in_box2d_and_rapier() {
    stand(
        &[
            (Scene::Pyramid { base: 15 }, from_refs((20, 30), (0.034, 0.035), (0.0052, 0.0051), (0.1, 0.2), (7.0e-11, 6.1e-10))),
            (Scene::Pyramid { base: 20 }, from_refs((30, 50), (0.060, 0.061), (0.0071, 0.0067), (0.2, 0.3), (1.4e-10, 1.5e-9))),
            (Scene::Pyramid { base: 25 }, from_refs((40, 60), (0.094, 0.094), (0.0091, 0.0085), (0.2, 0.4), (1.3e-10, 2.8e-9))),
        ],
        true,
        STEPS,
    );
}

/// Unit boxes stacked 10 and 20 high, locked: ours at rest from 10 and 30,
/// the top 0.008 and 0.035 lower.
#[test]
fn stacks_stand_as_in_box2d_and_rapier() {
    stand(
        &[
            (Scene::Stack { n: 10 }, from_refs((20, 20), (0.026, 0.026), (0.0051, 0.0051), (0.0, 0.0), (5.3e-11, 5.6e-10))),
            (Scene::Stack { n: 20 }, from_refs((50, 50), (0.108, 0.108), (0.0107, 0.0107), (0.0, 0.0), (2.4e-12, 1.1e-11))),
        ],
        false,
        STEPS,
    );
}

/// Ten boxes high, turning: ours at rest from 10, the top 0.011 lower,
/// leaning 0.1°. The references' energy is the most over the last 200
/// steps (5.8e-7 and 1.8e-7; at step 700, 6.5e-9 and 8.7e-9).
#[test]
fn a_stack_that_turns_stands_as_in_box2d_and_rapier() {
    stand(&[(Scene::Stack { n: 10 }, from_refs((100, 50), (0.112, 0.071), (0.0129, 0.0120), (0.8, 0.3), (5.8e-7, 1.8e-7)))], true, STEPS);
}

/// Twenty boxes high, turning, at six substeps, which a game that stacks
/// picks by writing physics's `Tuning` (the mod in the engine, since the
/// arrays have no world to read it from). Box2D topples it (7 boxes out of
/// the box); Rapier rests from 220, its top 0.27 lower, and still sways,
/// at up to 2.8e-4 a body over the last 200 steps (1.5e-7 at step 700,
/// where its swing turned). At the default five ours sways as much, up to
/// 5.6e-4, but slower to die, and rests only from 580, which is why a
/// stacking game picks six (from 60): its contacts are stiffer, and the
/// column further from buckling (physics.md, "Still at rest"; get-emj.41).
#[test]
fn a_twenty_high_stack_that_turns_rests_as_soon_as_rapiers_at_six_substeps() {
    let b = StandBounds { rest: 2 * 220, top_moved: 0.5 * 0.266, deepest_end: 0.5 * 0.0265, tilt: 1.0, energy_tail: 10.0 * 2.8e-4 };
    let scene = Scene::Stack { n: 20 };
    let mut m = mod_in_engine(&scene, true, false);
    m.substeps(6);
    let run = settle::settle(&mut m, &scene, true, STEPS);
    print_runs(&[scene], true, std::slice::from_ref(&run));
    stand_runs(&[(scene, b)], &[run]);
}

/// A `Tuning` in the world is what the mod's step solves by: at six
/// substeps, the mod is the arrays at six bit for bit, and not the arrays
/// at the default five.
#[test]
fn the_mod_solves_at_the_substeps_its_world_sets() {
    let scene = Scene::Pile { n: 400, width: 41.0, stagger: true };
    let six = solver::Params::of(&physics::Tuning { substeps: 6 });
    assert_eq!(six.substeps, 6);
    let mut m = mod_in_engine(&scene, true, false);
    m.substeps(6);
    let mut at_six =
        ecs::Flat::new(&scene, true, Box::new(move |b, s, c, p, dt| solver::solve_with(&six, (b, s), c, p, dt)), "ours at six");
    let mut at_five = ecs::Flat::new(&scene, true, Box::new(solver::solve_points), "ours");
    m.step(100);
    at_six.step(100);
    at_five.step(100);
    let bits = |b: &Dyn| [b.x, b.y, b.vx, b.vy, b.angle, b.w].map(f32::to_bits);
    let differ = |x: &[Dyn], y: &[Dyn]| x.iter().zip(y).filter(|(x, y)| bits(x) != bits(y)).count();
    let mb = m.bodies();
    assert_eq!(differ(&mb, &at_six.bodies()), 0, "the mod isn't the arrays at six substeps");
    assert!(differ(&mb, &at_five.bodies()) > 0, "six substeps solved as five");
}

fn mod_in_engine(scene: &Scene, turning: bool, sleep: bool) -> ecs::Ecs {
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    ecs::Ecs::new(&manifest, scene, sleep, turning)
}

/// Every bound here is on the arrays, so it must be the mod's step: on a
/// pile of each kind, every body where the mod has it, bit for bit, well
/// into settling. (The comparison checks the same on its scenes, but only
/// when it is run.)
#[test]
fn the_mod_is_the_arrays_bit_for_bit() {
    let scene = Scene::Pile { n: 400, width: 41.0, stagger: true };
    std::thread::scope(|s| {
        for turning in [false, true] {
            s.spawn(move || {
                let (mut m, mut a) =
                    (mod_in_engine(&scene, turning, false), ecs::Flat::new(&scene, turning, Box::new(solver::solve_points), "ours"));
                m.step(150);
                a.step(150);
                let bits = |b: &Dyn| [b.x, b.y, b.vx, b.vy, b.angle, b.w].map(f32::to_bits);
                let (mb, ab) = (m.bodies(), a.bodies());
                assert_eq!(mb.len(), ab.len());
                let differ = mb.iter().zip(&ab).filter(|(x, y)| bits(x) != bits(y)).count();
                assert_eq!(differ, 0, "turning {turning}: {differ} bodies differ from the arrays");
            });
        }
    });
}

/// Once a pile is at rest, sleeping takes every body within its half
/// second (30 steps; `Sleep::DEFAULT`): by the rest bound of its kind of
/// pile and that. A pile that crept would never sleep, which is what the
/// comparison found of the split impulse (physics.md, "Settling"). The mod,
/// since the arrays have no sleeping; one size each, since each is a whole
/// engine.
#[test]
fn sleeping_takes_every_body_of_a_pile_at_rest() {
    std::thread::scope(|s| {
        for (turning, rest) in [(false, 2 * 200), (true, 2 * 250)] {
            s.spawn(move || {
                let scene = Scene::Pile { n: 1000, width: 41.0, stagger: true };
                let mut m = mod_in_engine(&scene, turning, true);
                let limit = rest + 30 + settle::EVERY;
                let mut step = 0;
                while m.asleep() < 1000 {
                    assert!(step < limit, "turning {turning}: {} of 1000 asleep at step {step}", m.asleep());
                    m.step(settle::EVERY);
                    step += settle::EVERY;
                }
                println!("pile 1000{}: all asleep at step {step}", if turning { ", turning" } else { "" });
            });
        }
    });
}
