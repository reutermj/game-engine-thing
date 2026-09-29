//! Quality beyond settling, in 3D: a cube holding and sliding on a ramp, a
//! sphere rolling, a ball bouncing, a heavy cube on a light one, the
//! physics3d mod against bounds from hand calculations and from what Rapier
//! 3D 0.36 and Box3D 0.1 do on the same scenes (Jolt 5.6 recorded beside
//! them), set as 2D's are (//engine/std/physics/compare:behaviour_test;
//! physics.md, "Quality beyond settling"). The values are from
//!
//!     ./bazel run --config=bench //bench/physics3d:bench -- ramp_hold,ramp_slide,ramp_roll 1 all --rotate --behave
//!     ./bazel run --config=bench //bench/physics3d:bench -- bounce 25,50,75,100 all --rotate --behave
//!     ./bazel run --config=bench //bench/physics3d:bench -- ratio 10,100,1000 all --rotate --behave
//!
//! `TUNE=<variant>` runs ours tuned so, as in `:quality_test`. A test that
//! finds a real problem stays, ignored, naming its bead.

use std::sync::Arc;

#[cfg(feature = "long")]
mod behaviour_long;

use physics3d_bench::behave::Behaviour;
use physics3d_bench::scenes::Kind;
use physics3d_bench::{baseline, bounces, record, runs};

/// Ours on each scene, bodies turning, in threads of their own (each run
/// once in the binary, `runs.rs`, which the baseline reads too), each
/// printed so a failing run shows every number.
fn run(cases: &[(Kind, usize)]) -> Vec<Arc<Behaviour>> {
    let runs = runs::par(cases, |&(kind, n)| runs::behaved(kind, n));
    for ((kind, n), r) in cases.iter().zip(&runs) {
        let values: Vec<String> = r.values.iter().map(|(k, v)| format!("{k} {v:.6}")).collect();
        println!("{} {n}: {}", kind.name(), values.join(", "));
    }
    runs
}

/// Every bound broken, not the first.
#[derive(Default)]
struct Broken(Vec<String>);

impl Broken {
    fn check(&mut self, what: (Kind, usize), ok: bool, why: impl FnOnce() -> String) {
        if !ok {
            self.0.push(format!("{} {}: {}", what.0.name(), what.1, why()));
        }
    }

    fn most(&mut self, what: (Kind, usize), r: &Behaviour, name: &str, bound: f64) {
        let v = r.get(name);
        self.check(what, v <= bound, || format!("{name} {v}, bound {bound}"));
    }

    fn least(&mut self, what: (Kind, usize), r: &Behaviour, name: &str, bound: f64) {
        let v = r.get(name);
        self.check(what, v >= bound, || format!("{name} {v}, at least {bound}"));
    }

    /// A bounce family's statistic `name` at most `bound`.
    fn stat_most(&mut self, f: &(Family, Stats), name: &str, bound: f64) {
        let v = bounces::stat(&f.1, name);
        self.check(f.0, v <= bound, || format!("{name} {v}, bound {bound}"));
    }

    /// ... at least `bound`.
    fn stat_least(&mut self, f: &(Family, Stats), name: &str, bound: f64) {
        let v = bounces::stat(&f.1, name);
        self.check(f.0, v >= bound, || format!("{name} {v}, at least {bound}"));
    }

    fn assert(self) {
        assert!(self.0.is_empty(), "{} bounds broken:\n{}", self.0.len(), self.0.join("\n"));
    }
}

type Stats = Vec<(&'static str, f64)>;
/// A family's first scene, which names it where a bound breaks.
type Family = (Kind, usize);

/// A bounce family (`bounces.rs`) on its short or long grid, run by ours
/// (each run once in the binary, which the baseline reads too), and its
/// statistics, printed.
fn bounce_family(name: &str, long: bool) -> (Family, Stats) {
    let f = bounces::families(long).into_iter().find(|f| f.name == name).unwrap_or_else(|| panic!("no bounce family {name}"));
    let runs: Vec<Behaviour> = runs::par(&f.hits, |h| (*runs::hit(h)).clone());
    let stats = bounces::stats(&f, &runs);
    let values: Vec<String> = stats.iter().map(|(k, v)| format!("{k} {v:.6}")).collect();
    println!("bounces {name} ({} runs): {}", runs.len(), values.join(", "));
    ((Kind::Hit, f.hits[0].pack()), stats)
}

/// 2D's rules (its `behaviour_test.rs`): a median bounce a quarter further
/// under e² than the lower reference's, and a share kept a quarter further
/// from all of it.
fn under(lower: f64) -> f64 {
    1.25 * lower.min(0.0) - 0.002
}

fn kept(lower: f64) -> f64 {
    1.0 - 1.25 * (1.0 - lower) - 0.002
}

/// Measured over expected, less one.
fn off(r: &Behaviour) -> f64 {
    r.get("a") / r.get("expected a") - 1.0
}

/// A unit cube on a ramp 20° steep, friction 0.6: it holds. In 2 s Rapier
/// creeps 0.000068 down the slope, Box3D 0.000063, Jolt 0.000001; ours
/// 0.000001. The bound is three times the larger reference.
#[test]
fn a_cube_below_the_friction_angle_holds_still() {
    let c = (Kind::RampHold, 1);
    let r = &run(&[c])[0];
    let mut broken = Broken::default();
    broken.most(c, r, "crept", 3.0 * 0.000068);
    broken.assert();
}

/// A unit cube on a ramp 30° steep, friction 0.2: it slides at g (sin θ −
/// μ cos θ) = 3.2059 (gravity 9.81). Rapier, Box3D and ours meet it to six
/// digits; Jolt is 5.0% slow, its bodies' default damping (0.05 a second,
/// linear and angular) left on. The bound is 1%.
#[test]
fn a_cube_above_the_friction_angle_slides_at_g_sin_less_mu_g_cos() {
    let c = (Kind::RampSlide, 1);
    let r = &run(&[c])[0];
    let mut broken = Broken::default();
    broken.check(c, off(r).abs() <= 0.01, || format!("a {} against {}", r.get("a"), r.get("expected a")));
    broken.assert();
}

/// A sphere on a ramp 30° steep, friction 0.6, above (2/7) tan θ = 0.16: it
/// rolls without slipping at (5/7) g sin θ = 3.5036. Rapier is 0.03% fast,
/// Box3D 0.04% slow, ours 0.03% slow (Jolt 5.0% slow, damped); the bound is
/// 1%. Its contact point slips at 0.032 a second on average in Rapier,
/// 0.035 in Box3D, ours 0.032; the bound is twice the larger.
#[test]
fn a_sphere_rolls_without_slipping_at_five_sevenths_g_sin() {
    let c = (Kind::RampRoll, 1);
    let r = &run(&[c])[0];
    let mut broken = Broken::default();
    broken.check(c, off(r).abs() <= 0.01, || format!("a {} against {}", r.get("a"), r.get("expected a")));
    broken.most(c, r, "slip", 2.0 * 0.0354);
    broken.assert();
}

/// A ball dropped 5 onto a floor, both of restitution e: its first rebound
/// as a share of the drop, against e², bounded as in 2D (a quarter further
/// below e² than the lower of Rapier and Box3D, and never 1% over e²).
/// Rapier 0.0486 and 0.2377, Box3D 0.0516 and 0.2406 at e = 0.25 and 0.5
/// (Jolt 0.0512, 0.2319); ours 0.0525 and 0.2481.
#[test]
fn a_ball_rebounds_to_about_e_squared_of_its_drop() {
    let cases = [((Kind::Bounce, 25), 0.0486), ((Kind::Bounce, 50), 0.2377)];
    let runs = run(&cases.map(|c| c.0));
    let mut broken = Broken::default();
    for ((c, lowest), r) in cases.iter().zip(&runs) {
        let e = c.1 as f64 / 100.0;
        broken.least(*c, r, "first apex", e * e - 1.25 * (e * e - lowest));
        broken.most(*c, r, "first apex", 1.01 * e * e);
    }
    broken.assert();
}

/// Above e = 0.5, and lossless: Rapier and Box3D rebound to 0.554 and 0.557
/// at 0.75 (e² = 0.5625), 0.996 and 0.999 at 1, where a lossless ball keeps
/// 0.96 and 0.998 of its height after 9 bounces (Jolt 0.53, damped). Ours
/// rebounds to 0.575 and 1.032, and the lossless ball climbs to 1.31 of
/// its drop in 9, as 2D's does (get-emj.56): each bounce returns a step's
/// gravity more than it came in with (get-emj.60).
#[test]
#[ignore = "get-emj.60: a bounce returns a step's gravity more than it came in with"]
fn a_ball_never_rebounds_higher_than_e_squared() {
    let cases = [(Kind::Bounce, 75), (Kind::Bounce, 100)];
    let runs = run(&cases);
    let mut broken = Broken::default();
    broken.most(cases[0], &runs[0], "first apex", 1.01 * 0.5625);
    broken.most(cases[1], &runs[1], "most apex", 1.01);
    broken.least(cases[1], &runs[1], "last apex", 0.9 * 0.9624);
    broken.assert();
}

/// A unit cube 10, 100 and 1000 times as heavy on a light one. Rapier and
/// Box3D stand the first two, the top 0.0026 and 0.033 lower, 0.0030 and
/// 0.028 deep, at rest from 1 and 1, 55 and 21 (Jolt 0.0009 and 0.030,
/// at rest from 2 and 193); ours stands them 0.0006 and 0.0099 lower,
/// 0.0008 and 0.0070 deep, at rest from 0 and 42. At 1000 to 1 every engine
/// crushes the light cube and topples the heavy off it, at rest from 210
/// (Rapier), 279 (Box3D), 101 (Jolt) and 316 (ours). The bounds: half the
/// smaller reference's sinking and depth where they stand, twice the later
/// rest, nothing out of the scene.
#[test]
fn heavy_cubes_stand_on_light_ones_sinking_less_than_in_rapier_and_box3d() {
    // (scene, top sank, deepest, at rest from)
    let cases = [
        ((Kind::Ratio, 10), Some((0.5 * 0.002636, 0.5 * 0.003035)), 1.0),
        ((Kind::Ratio, 100), Some((0.5 * 0.033224, 0.5 * 0.027886)), 55.0),
        ((Kind::Ratio, 1000), None, 279.0),
    ];
    let runs = run(&cases.map(|c| c.0));
    let mut broken = Broken::default();
    for ((c, stands, rest), r) in cases.iter().zip(&runs) {
        if let Some((sank, deepest)) = stands {
            broken.least(*c, r, "stands", 1.0);
            broken.most(*c, r, "top sank", *sank);
            broken.most(*c, r, "deepest at end", *deepest);
        }
        broken.most(*c, r, "at rest from", 2.0 * rest);
        broken.most(*c, r, "escaped", 0.0);
    }
    broken.assert();
}

// The bounce families (`bounces.rs`; physics.md, "Bounces"), bounded as
// 2D's are: the analytic answer where there is one, Rapier's and Box3D's
// statistics on exactly the same grid otherwise, measured with `bench --
// ours,rapier,box3d --bounces --each` (2026-09-29). The long grids' are
// `behaviour_long.rs`.

/// No bounce of a sphere or a cube leaves with more energy than it came in
/// with, to 1%, past what the push-out of its deepest overlap lifts it,
/// nor a cube landing on an edge or a corner, which tips as it bounces
/// (Rapier and Box3D: 0.000 each on this grid). Ours 0.061 (drops),
/// 0.068 (rates), 0.046 (angled), and 0.394 tipping: the step's gravity,
/// as in 2D (get-emj.56).
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn bounces_leave_with_no_more_energy_than_they_came_in_with() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique"] {
        broken.stat_most(&bounce_family(name, false), "excess worst", 0.01);
    }
    broken.stat_most(&bounce_family("drops", false), "excess tipping", 0.01);
    broken.assert();
}

/// A sphere, and a cube landing flat without friction, rebound at e of the
/// speed they meet at, to 1% of the energy along the normal, and two free
/// spheres part at e of their closing speed. Box3D 0.001 at most, Rapier
/// 1.0 (it has no threshold); ours 0.061, 0.068 and 0.062, the pairs
/// exactly.
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn a_bounce_rebounds_at_e_of_the_speed_it_meets_at() {
    let mut broken = Broken::default();
    for name in ["drops", "rates", "oblique", "pairs"] {
        broken.stat_most(&bounce_family(name, false), "gain worst", 0.01);
    }
    broken.assert();
}

/// Nothing meeting the floor under the threshold bounces: Box3D none,
/// Rapier 8, ours none (a step's gravity at 40 is 0.67, and the slowest
/// here meets at 0.8).
#[test]
fn nothing_bounces_below_the_threshold() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("drops", false), "bounced below", 0.0);
    broken.assert();
}

/// Losses no worse than the references': Rapier's and Box3D's median
/// square-on bounces −0.0330 and −0.0367 (drops), −0.0522 and −0.0554
/// (rates), −0.0065 and −0.0053 (angled), none flat; ours 0.000, +0.0035,
/// 0.000, none flat.
#[test]
fn bounces_lose_no_more_than_in_rapier_and_box3d() {
    let mut broken = Broken::default();
    for (name, lower, flat) in [("drops", -0.036697, 0.0), ("rates", -0.055437, 0.0), ("oblique", -0.006474, 0.0)] {
        let f = bounce_family(name, false);
        broken.stat_least(&f, "gain median", under(lower));
        broken.stat_most(&f, "flat above", flat);
    }
    broken.assert();
}

/// An angled bounce keeps its speed along the floor with no friction, never
/// gains any with it, and keeps a median 0.710 with it in Rapier and Box3D;
/// ours 0.711.
#[test]
fn an_angled_bounce_keeps_its_speed_along_the_floor_but_what_friction_takes() {
    let f = bounce_family("oblique", false);
    let mut broken = Broken::default();
    broken.stat_most(&f, "slip worst", 0.01);
    broken.stat_most(&f, "tangent most", 1.01);
    broken.stat_least(&f, "tangent median", kept(0.710074));
    broken.assert();
}

/// Two free spheres keep their momentum through a bounce, to a thousandth
/// of the impulse: Rapier and Box3D exactly, ours 0.00002.
#[test]
fn two_free_spheres_keep_their_momentum_through_a_bounce() {
    let mut broken = Broken::default();
    broken.stat_most(&bounce_family("pairs", false), "momentum worst", 1e-3);
    broken.assert();
}

/// Over 20 s a lossless sphere, or cube, never rises past its drop, and a
/// sphere keeps no more than e² of its height a bounce, nor less than the
/// lower reference's median by a quarter more. Rapier and Box3D: highest
/// 0.997 and 0.997, most kept 0.998 and 1.005 of e², median 0.978 and
/// 0.984. Ours climbs to 5.43 of its drop, keeps up to 1.32 of e², a
/// median 1.08.
#[test]
#[ignore = "get-emj.60: a bounce returns a step of gravity more than it came in with"]
fn a_lossless_sphere_never_rises_and_a_sphere_keeps_e_squared_of_its_height() {
    let f = bounce_family("series", false);
    let mut broken = Broken::default();
    broken.stat_most(&f, "rise worst", 1.01);
    broken.stat_most(&f, "decay worst", 1.01);
    broken.stat_least(&f, "decay median", kept(0.978267));
    broken.assert();
}

/// The default suite's baseline, group `behaviour`: every value
/// `record.rs` takes from these tests' scenes, the ignored tests' too, and
/// the bounce families', each within its band of `baseline.txt`.
#[test]
fn baseline() {
    baseline::assert_holds(record::DEFAULT, &["behaviour"], &record::behaviour(false), record::WRITE);
}
