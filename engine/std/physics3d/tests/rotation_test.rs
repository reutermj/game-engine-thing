//! Bodies that turn, in the world: what rotation has to get right that a
//! translation-only step couldn't get wrong.

use engine_ecs::{Build, Entity, World};
use physics3d::{AngularVelocity, Body, Collider, Position, Quat, Rotation, Vec3, Velocity, dynamic, fixed, step};

struct Sim {
    world: World,
}

impl Sim {
    /// A floor, its top at y = 0. P3_TUNE names a variant of the step
    /// (physics3d::Tuning::parse), for trying one against these tests.
    fn new() -> Sim {
        if let Ok(t) = std::env::var("P3_TUNE") {
            *physics3d::TUNING.lock().unwrap() = Some(physics3d::Tuning::parse(&t).unwrap());
        }
        let world = World::new();
        drop(step(&world));
        let s = Sim { world };
        s.fixed(Vec3::new(0.0, -0.5, 0.0), Vec3::new(50.0, 0.5, 50.0));
        s
    }

    fn fixed(&self, at: Vec3, half: Vec3) {
        let mut m = self.world.between_frames(Build::default()).unwrap();
        m.spawn(fixed(at, Quat::IDENTITY, Collider::cuboid(half)));
    }

    fn body(&self, at: Vec3, rot: Quat, c: Collider, v: Vec3, w: Vec3) -> Entity {
        let mut m = self.world.between_frames(Build::default()).unwrap();
        let (p, q, r, c, b, _, _) = dynamic(at, rot, c, Body::solid(1.0, &c));
        m.spawn((p, q, r, c, b, Velocity { x: v.x, y: v.y, z: v.z }, AngularVelocity { x: w.x, y: w.y, z: w.z }))
    }

    fn run(&self, steps: usize) {
        let s = step(&self.world);
        for _ in 0..steps {
            s.run_sequential(&self.world);
        }
    }

    fn at(&self, e: Entity) -> Vec3 {
        self.world.values::<Position>().unwrap().into_iter().find(|(x, _)| *x == e).unwrap().1.at()
    }

    fn rot(&self, e: Entity) -> Quat {
        self.world.values::<Rotation>().unwrap().into_iter().find(|(x, _)| *x == e).unwrap().1.quat()
    }

    fn v(&self, e: Entity) -> Vec3 {
        let v = self.world.values::<Velocity>().unwrap().into_iter().find(|(x, _)| *x == e).unwrap().1;
        Vec3::new(v.x, v.y, v.z)
    }

    fn w(&self, e: Entity) -> Vec3 {
        let w = self.world.values::<AngularVelocity>().unwrap().into_iter().find(|(x, _)| *x == e).unwrap().1;
        Vec3::new(w.x, w.y, w.z)
    }

    /// How nearly one of the body's axes points up: 1 when it lies on a face.
    fn flat(&self, e: Entity) -> f32 {
        let m = self.rot(e).matrix();
        m.cols.iter().map(|c| c.y.abs()).fold(0.0, f32::max)
    }
}

const UNIT: Vec3 = Vec3::splat(0.5);

#[test]
fn a_box_rests_on_its_face() {
    let s = Sim::new();
    let b = s.body(Vec3::new(0.0, 0.5, 0.0), Quat::IDENTITY, Collider::cuboid(UNIT), Vec3::ZERO, Vec3::ZERO);
    s.run(300);
    assert!(s.v(b).len() < 1e-3 && s.w(b).len() < 1e-3, "moving: {:?} {:?}", s.v(b), s.w(b));
    assert!((s.at(b) - Vec3::new(0.0, 0.5, 0.0)).len() < 0.005, "at {:?}", s.at(b));
    assert!(s.flat(b) > 0.99999, "tilted: {:?}", s.rot(b));
}

#[test]
fn a_box_tumbles_off_an_edge_and_comes_to_rest_on_a_face() {
    let s = Sim::new();
    // A ledge 2 high whose edge is at x = 0; the box's center is past it.
    s.fixed(Vec3::new(-2.0, 1.0, 0.0), Vec3::new(2.0, 1.0, 2.0));
    let b = s.body(Vec3::new(0.15, 2.5, 0.0), Quat::IDENTITY, Collider::cuboid(UNIT), Vec3::ZERO, Vec3::ZERO);
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
    let s = Sim::new();
    let r = 0.5;
    let b = s.body(Vec3::new(0.0, r, 0.0), Quat::IDENTITY, Collider::sphere(r), Vec3::new(5.0, 0.0, 0.0), Vec3::ZERO);
    s.run(120);
    let (v, w) = (s.v(b), s.w(b));
    assert!((v.x - 5.0 * 5.0 / 7.0).abs() < 0.02, "rolls at {v:?}");
    assert!((w.z + v.x / r).abs() < 0.02, "without slipping: {w:?} at {v:?}");
}

#[test]
fn a_spinning_sphere_rolls_off_at_two_sevenths_of_its_spin() {
    // Backspin in place: friction turns spin into rolling, v = 2/7 r w.
    let s = Sim::new();
    let r = 0.5;
    let b = s.body(Vec3::new(0.0, r, 0.0), Quat::IDENTITY, Collider::sphere(r), Vec3::ZERO, Vec3::new(0.0, 0.0, -10.0));
    s.run(120);
    let (v, w) = (s.v(b), s.w(b));
    assert!((v.x - 2.0 / 7.0 * r * 10.0).abs() < 0.02, "rolls at {v:?}");
    assert!((w.z + v.x / r).abs() < 0.02, "without slipping: {w:?} at {v:?}");
}

#[test]
fn a_spinning_box_stops_by_friction() {
    // Twisting on its face: only twist friction can stop it.
    let s = Sim::new();
    let b = s.body(Vec3::new(0.0, 0.5, 0.0), Quat::IDENTITY, Collider::cuboid(UNIT), Vec3::ZERO, Vec3::new(0.0, 5.0, 0.0));
    s.run(120);
    assert!(s.w(b).len() < 1e-3 && s.v(b).len() < 1e-3, "still turning: {:?} {:?}", s.w(b), s.v(b));
    assert!(s.flat(b) > 0.99999, "tilted: {:?}", s.rot(b));
}

#[test]
fn a_box_stack_stands() {
    let s = Sim::new();
    let boxes: Vec<Entity> = (0..10)
        .map(|k| s.body(Vec3::new(0.0, 0.5 + k as f32, 0.0), Quat::IDENTITY, Collider::cuboid(UNIT), Vec3::ZERO, Vec3::ZERO))
        .collect();
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
    let s = Sim::new();
    let mut all = Vec::new();
    for k in 0..40 {
        let f = k as f32;
        let rot = Quat::axis_angle(Vec3::new(f.sin(), 1.0, f.cos()), f * 0.7);
        let c = if k % 2 == 0 { Collider::cuboid(UNIT) } else { Collider::sphere(0.5) };
        let at = Vec3::new((k % 8) as f32 * 2.0 - 8.0, 15.0, (k / 8) as f32 * 2.0 - 5.0);
        all.push((k, s.body(at, rot, c, Vec3::ZERO, Vec3::new(f.cos() * 3.0, 1.0, f.sin() * 3.0))));
    }
    s.run(200);
    for (k, b) in &all {
        assert!(s.at(*b).y > 0.3, "body {k} fell through, at {:?}", s.at(*b));
    }
}
