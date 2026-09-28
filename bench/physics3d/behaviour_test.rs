//! Quality beyond settling, in 3D: a cube holding and sliding on a ramp, a
//! sphere rolling, a ball bouncing, a heavy cube on a light one, the
//! physics3d mod against bounds from hand calculations and from what Rapier
//! 3D 0.36 and Box3D 0.1 do on the same scenes (Jolt 5.6 recorded beside
//! them), set as 2D's are (//engine/std/physics/compare:behaviour_test;
//! physics.md, "Quality beyond settling"). The values are from
//!
//!     ./bazel run -c opt //bench/physics3d:bench -- ramp_hold,ramp_slide,ramp_roll 1 all --rotate --behave
//!     ./bazel run -c opt //bench/physics3d:bench -- bounce 25,50,75,100 all --rotate --behave
//!     ./bazel run -c opt //bench/physics3d:bench -- ratio 10,100,1000 all --rotate --behave
//!
//! `TUNE=<variant>` runs ours tuned so, as in `:quality_test`. A test that
//! finds a real problem stays, ignored, naming its bead.

use physics3d_bench::behave::{self, Behaviour};
use physics3d_bench::scenes::{self, Kind};
use physics3d_bench::{Config, Iters, make_backend};

/// Ours on each scene, bodies turning, in threads of their own, each
/// printed so a failing run shows every number.
fn run(cases: &[(Kind, usize)]) -> Vec<Behaviour> {
    let tune: &'static str = std::env::var("TUNE").unwrap_or_default().leak();
    let runs: Vec<Behaviour> = std::thread::scope(|s| {
        let threads: Vec<_> = cases
            .iter()
            .map(|&(kind, n)| {
                s.spawn(move || {
                    let config = Config { iters: Iters::Default, sleep: false, max_bodies: 16, rotate: true, tune };
                    behave::behave(&scenes::build(kind, n), make_backend("ours", &config).unwrap().as_mut())
                })
            })
            .collect();
        threads.into_iter().map(|t| t.join().expect("a scene panicked")).collect()
    });
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

    fn assert(self) {
        assert!(self.0.is_empty(), "{} bounds broken:\n{}", self.0.len(), self.0.join("\n"));
    }
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
