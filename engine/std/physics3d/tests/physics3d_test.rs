//! The physics3d mod in a running engine, on pile3d's scenes: what
//! rotation has to get right that a translation-only step couldn't get
//! wrong, and a pile settling with the 3D spatial order intact.

mod sim;

use std::f32::consts::FRAC_PI_2;

use engine_ecs::Entity;
use physics3d::{Collider, ContactPair, Manifold, Position, Rotation, Vec3, Velocity};
use sim::Sim;

const UNIT: &str = "box 0.5 0.5 0.5";
const NONE: (Vec3, f32) = (Vec3::ZERO, 0.0);

/// A floor, its top at y = 0. P3_TUNE names a variant of the step
/// (`physics3d::Tuning::parse`), for trying one against these tests.
fn floor(test: &str) -> Sim {
    let s = Sim::new(test);
    s.send("floor");
    if let Ok(t) = std::env::var("P3_TUNE") {
        s.send(&format!("tune {t}"));
    }
    s
}

#[test]
fn a_box_rests_on_its_face() {
    let s = floor("rests");
    let b = s.body(UNIT, Vec3::new(0.0, 0.5, 0.0), NONE, Vec3::ZERO, Vec3::ZERO);
    s.run(300);
    assert!(s.v(b).len() < 1e-3 && s.w(b).len() < 1e-3, "moving: {:?} {:?}", s.v(b), s.w(b));
    assert!((s.at(b) - Vec3::new(0.0, 0.5, 0.0)).len() < 0.005, "at {:?}", s.at(b));
    assert!(s.flat(b) > 0.99999, "tilted: {:?}", s.rot(b));
}

#[test]
fn a_box_tumbles_off_an_edge_and_comes_to_rest_on_a_face() {
    let s = floor("tumbles");
    // A ledge 2 high whose edge is at x = 0; the box's center is past it.
    s.send("fixed -2 1 0 2 1 2");
    let b = s.body(UNIT, Vec3::new(0.15, 2.5, 0.0), NONE, Vec3::ZERO, Vec3::ZERO);
    s.run(40);
    let tipping = s.w(b);
    assert!(tipping.z < -0.5, "it tips over the edge, turning about z: {tipping:?}");
    s.run(600);
    assert!(s.v(b).len() < 1e-3 && s.w(b).len() < 1e-3, "moving: {:?} {:?}", s.v(b), s.w(b));
    let at = s.at(b);
    assert!(at.x > 0.5 && (at.y - 0.5).abs() < 0.01, "on the floor past the ledge: {at:?}");
    assert!(s.flat(b) > 0.9999, "on a face, not an edge: {:?}", s.rot(b));
}

#[test]
fn a_sliding_sphere_starts_rolling_at_five_sevenths_of_its_speed() {
    // A solid sphere sliding without turning: friction slows it and spins
    // it up until it rolls, at 5/7 of the speed it slid at.
    let s = floor("slides");
    let r = 0.5;
    let b = s.body("sphere 0.5", Vec3::new(0.0, r, 0.0), NONE, Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO);
    s.run(120);
    let (v, w) = (s.v(b), s.w(b));
    assert!((v.x - 5.0 * 5.0 / 7.0).abs() < 0.02, "rolls at {v:?}");
    assert!((w.z + v.x / r).abs() < 0.02, "without slipping: {w:?} at {v:?}");
}

#[test]
fn a_spinning_sphere_rolls_off_at_two_sevenths_of_its_spin() {
    // Backspin in place: friction turns spin into rolling, v = 2/7 r w.
    let s = floor("spins");
    let r = 0.5;
    let b = s.body("sphere 0.5", Vec3::new(0.0, r, 0.0), NONE, Vec3::ZERO, Vec3::new(0.0, 0.0, -10.0));
    s.run(120);
    let (v, w) = (s.v(b), s.w(b));
    assert!((v.x - 2.0 / 7.0 * r * 10.0).abs() < 0.02, "rolls at {v:?}");
    assert!((w.z + v.x / r).abs() < 0.02, "without slipping: {w:?} at {v:?}");
}

#[test]
fn a_spinning_box_stops_by_friction() {
    // Twisting on its face: only twist friction can stop it.
    let s = floor("twists");
    let b = s.body(UNIT, Vec3::new(0.0, 0.5, 0.0), NONE, Vec3::ZERO, Vec3::new(0.0, 5.0, 0.0));
    s.run(120);
    assert!(s.w(b).len() < 1e-3 && s.v(b).len() < 1e-3, "still turning: {:?} {:?}", s.w(b), s.v(b));
    assert!(s.flat(b) > 0.99999, "tilted: {:?}", s.rot(b));
}

#[test]
fn a_box_stack_stands() {
    let s = floor("stack");
    let boxes: Vec<Entity> = (0..10).map(|k| s.body(UNIT, Vec3::new(0.0, 0.5 + k as f32, 0.0), NONE, Vec3::ZERO, Vec3::ZERO)).collect();
    s.run(600);
    for (k, &b) in boxes.iter().enumerate() {
        let at = s.at(b);
        assert!(s.v(b).len() < 1e-3 && s.w(b).len() < 1e-3, "box {k} moving: {:?} {:?}", s.v(b), s.w(b));
        assert!(at.x.abs() < 0.01 && at.z.abs() < 0.01, "box {k} slid to {at:?}");
        assert!((at.y - (0.5 + k as f32)).abs() < 0.05, "box {k} sank to {at:?}");
        assert!(s.flat(b) > 0.99999, "box {k} tilted: {:?}", s.rot(b));
    }
}

#[test]
fn bodies_dropped_turning_land_on_the_floor() {
    let s = floor("dropped");
    let mut all = Vec::new();
    for k in 0..40 {
        let f = k as f32;
        let shape = if k % 2 == 0 { UNIT } else { "sphere 0.5" };
        let at = Vec3::new((k % 8) as f32 * 2.0 - 8.0, 15.0, (k / 8) as f32 * 2.0 - 5.0);
        let turn = (Vec3::new(f.sin(), 1.0, f.cos()), f * 0.7 + 0.1);
        all.push((k, s.body(shape, at, turn, Vec3::ZERO, Vec3::new(f.cos() * 3.0, 1.0, f.sin() * 3.0))));
    }
    s.run(200);
    for (k, b) in &all {
        assert!(s.at(*b).y > 0.3, "body {k} fell through, at {:?}", s.at(*b));
    }
}

/// A body is bounded as it is turned, and re-bounded as it turns: a plank
/// standing on end rests on it, and one tipping as it falls meets the floor
/// with its low end. Bounded as if it lay flat, a plank is paired with the
/// floor only once its center is within its thin side of it, so it sinks
/// that far first.
#[test]
fn a_planks_bounds_follow_its_turn() {
    let s = floor("planks");
    let (plank, shape) = (Collider::cuboid(Vec3::new(1.0, 0.1, 0.25)), "box 1 0.1 0.25");
    let standing = s.body(shape, Vec3::new(0.0, 1.0, 0.0), (Vec3::Z, FRAC_PI_2), Vec3::ZERO, Vec3::ZERO);
    let tipping = s.body(shape, Vec3::new(5.0, 1.5, 0.0), NONE, Vec3::ZERO, Vec3::new(0.0, 0.0, 4.0));
    let mut lowest = f32::MAX;
    for _ in 0..120 {
        s.run(1);
        lowest = lowest.min(s.at(tipping).y - plank.turned_half(s.rot(tipping)).y);
    }
    assert!(lowest > -0.03, "the tipping plank went {lowest} into the floor");
    assert!((s.at(standing).y - 1.0).abs() < 0.02, "the plank on end sank to {:?}", s.at(standing));
    assert!(s.w(tipping).len() < 4.0, "it landed: {:?}", s.w(tipping));
}

/// A pile of turning boxes dropped into a walled box settles: at rest,
/// above the floor, inside the walls, barely overlapping, touching several
/// neighbors each (a pile, not columns), with the 3D spatial order intact.
#[test]
fn a_pile_settles_in_the_world() {
    const HALF: f32 = 3.0;
    let s = Sim::new("pile");
    s.send(&format!("fixed 0 -0.5 0 {} 0.5 {}", HALF + 1.0, HALF + 1.0));
    for (x, z) in [(1.0f32, 0.0f32), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
        let hx = if x != 0.0 { 0.5 } else { HALF + 1.0 };
        let hz = if z != 0.0 { 0.5 } else { HALF + 1.0 };
        s.send(&format!("fixed {} 5 {} {hx} 5 {hz}", x * (HALF + 0.5), z * (HALF + 0.5)));
    }
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut jitter = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.4
    };
    let (n, per) = (200, 5);
    for k in 0..n {
        let (i, j, l) = (k % per, (k / per) % per, k / (per * per));
        let at = Vec3::new(i as f32 * 1.1 - 2.2 + jitter(), 1.0 + l as f32 * 1.2, j as f32 * 1.1 - 2.2 + jitter());
        s.body(UNIT, at, NONE, Vec3::ZERO, Vec3::ZERO);
    }
    s.run(600);
    let w = s.engine.world();
    let speeds = w.values::<Velocity>().unwrap();
    assert_eq!(speeds.len(), n);
    let fastest = speeds.iter().map(|(_, v)| Vec3::new(v.x, v.y, v.z).len()).fold(0.0, f32::max);
    assert!(fastest < 0.05, "still moving at {fastest}");
    let moving: std::collections::HashSet<_> = speeds.iter().map(|(e, _)| *e).collect();
    for (e, p) in w.values::<Position>().unwrap().iter().filter(|(e, _)| moving.contains(e)) {
        assert!(p.y > 0.4 && p.x.abs() < HALF && p.z.abs() < HALF, "{e:?} at {p:?}");
    }
    let manifolds = w.values::<Manifold>().unwrap();
    let deepest = manifolds.iter().map(|(_, m)| m.deepest()).fold(0.0, f32::max);
    assert!(deepest < 0.02, "a contact {deepest} deep");
    // A pile: bodies rest on several others. Columns would be about two
    // contacts a body, the one below and the one above (4.1 measured here).
    let touching = manifolds.iter().filter(|(_, m)| m.deepest() > -0.01).count();
    let per_body = 2.0 * touching as f32 / n as f32;
    assert!(per_body > 3.0, "{touching} touching contacts for {n} bodies: {per_body} a body");
    assert_eq!(w.values::<ContactPair>().unwrap().len(), manifolds.len());
    let turned = w.values::<Rotation>().unwrap().iter().filter(|(e, q)| moving.contains(e) && q.quat().v.len() > 1e-3).count();
    assert!(turned > n / 2, "{turned} of the boxes turned");
    for t in w.tables() {
        let Some(spatial) = &t.spatial else {
            continue;
        };
        let order = spatial.pages.read().unwrap();
        assert_eq!(order.dims(), 3);
        order.check(&t.rows.read().unwrap(), 2.0, 1.0).unwrap();
    }
    // What physics3d says of it agrees with the world.
    let stats = s.engine.send("physics3d", "stats").unwrap();
    assert!(stats.starts_with(&format!("steps 600 contacts {} ", manifolds.len())), "{stats}");
}

/// The step reads its settings from the world, both systems that have
/// any: a box on the floor, run at the defaults, with fewer substeps (the
/// solve's), and without warm starting (the contacts').
#[test]
fn a_tuning_in_the_world_is_the_steps() {
    let run = |test: &str, tune: Option<&str>| {
        let s = floor(test);
        if let Some(t) = tune {
            s.send(&format!("tune {t}"));
        }
        let b = s.body(UNIT, Vec3::new(0.0, 0.6, 0.0), (Vec3::Y, 0.3), Vec3::new(0.5, 0.0, 0.0), Vec3::ZERO);
        s.run(30);
        let stages = s.engine.send("physics3d", "stages").unwrap();
        let count = |k: &str| stages.split_whitespace().skip_while(|w| *w != k).nth(1).unwrap().parse::<u64>().unwrap();
        (s.at(b), count("kept"), count("matched"))
    };
    let (at, kept, matched) = run("tuned_default", None);
    assert!(kept > 0 && matched == kept, "warm-started {matched} of {kept}");
    let (fewer, _, _) = run("tuned_substeps", Some("sub=2"));
    assert_ne!(fewer, at, "two substeps a step moved it as five do");
    let (_, kept, matched) = run("tuned_cold", Some("warm=cold"));
    assert!(kept > 0 && matched == 0, "warm-started {matched} of {kept} cold");
}

/// What the last step's `stages` says of `key`.
fn stage(s: &Sim, key: &str) -> u64 {
    let stages = s.engine.send("physics3d", "stages").unwrap();
    stages.split_whitespace().skip_while(|w| *w != key).nth(1).unwrap().parse().unwrap()
}

/// Recycling (`Tuning::recycle`): a box settling onto the floor from a
/// tilt keeps its contact from step to step, its ids and so its impulses,
/// with its points carried where the narrowphase finds them; one sliding
/// faster than the recycling distance a step is found again every step.
#[test]
fn a_settling_box_keeps_its_contact_and_a_sliding_one_is_found_again() {
    let (s, found) = (floor("recycles"), floor("recycles_off"));
    found.send("tune recycle=0");
    let tilt = (Vec3::new(1.0, 0.0, 0.3).normalize(), 0.02);
    let at = Vec3::new(0.0, 0.51, 0.0);
    let (b, c) = (s.body(UNIT, at, tilt, Vec3::ZERO, Vec3::ZERO), found.body(UNIT, at, tilt, Vec3::ZERO, Vec3::ZERO));
    let manifold = |s: &Sim| s.engine.world().values::<Manifold>().unwrap()[0].1;
    let mut carried = 0;
    for step in 0..30 {
        s.run(1);
        found.run(1);
        carried += stage(&s, "recycled");
        assert_eq!(stage(&found, "recycled"), 0);
        assert_eq!(stage(&s, "matched"), stage(&s, "kept"), "step {step}: every point warm-started");
        let (kept, fresh) = (manifold(&s), manifold(&found));
        for k in 0..kept.count as usize {
            let (ra, depth) = kept.point(k);
            let near = (0..fresh.count as usize).map(|j| fresh.point(j)).find(|(rb, _)| (*rb - ra).len() < 2e-3);
            let (_, fresh_depth) = near.unwrap_or_else(|| panic!("step {step}: no point found near {ra:?}: {kept:?} against {fresh:?}"));
            assert!((depth - fresh_depth).abs() < 2e-4, "step {step}, point {k}: {depth} deep carried, {fresh_depth} found");
        }
    }
    assert!(carried >= 25, "carried {carried} of 30 steps");
    assert!((s.at(b) - found.at(c)).len() < 1e-3, "{:?} and {:?}", s.at(b), found.at(c));

    let slide = floor("slides");
    slide.body(UNIT, Vec3::new(0.0, 0.5, 0.0), NONE, Vec3::new(4.0, 0.0, 0.0), Vec3::ZERO);
    slide.run(10);
    assert_eq!(stage(&slide, "recycled"), 0, "a box moving 0.067 a step is found again");
}
