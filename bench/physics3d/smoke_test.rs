//! Every backend links, runs a small pile to rest and keeps it above the
//! floor, measured by the harness's own geometry. Small enough for fastbuild.

use physics3d_bench::scenes::{self, Kind};
use physics3d_bench::{BACKENDS, Config, Iters, make_backend, measure};

fn settles(kind: Kind) {
    let mut scene = scenes::build(kind, 20);
    scene.steps = 240;
    for name in BACKENDS {
        let config = Config { iters: Iters::Default, sleep: false, max_bodies: 64 };
        let mut backend = make_backend(name, &config).unwrap();
        let run = measure::run(&scene, backend.as_mut());
        let q = &run.quality;
        assert_eq!(q.bodies, 20, "{name}");
        assert_eq!(q.escaped, 0, "{name}: {q:?}");
        assert!(q.max_speed < 0.1, "{name} still moving: {q:?}");
        assert!(q.pen_max < 0.05, "{name} penetrates: {q:?}");
        // Resting on something: nothing floats, and something reached the floor.
        assert_eq!(q.histogram[0], 0, "{name}: {q:?}");
        assert!(run.native_touching > 0, "{name} reports no contacts");
    }
}

#[test]
fn spheres_settle() {
    settles(Kind::SpherePile);
}

#[test]
fn boxes_settle() {
    settles(Kind::BoxPile);
}

#[test]
fn rain_lands() {
    let mut scene = scenes::build(Kind::Rain, 20);
    scene.steps = scene.spawn.len() + 200;
    for name in BACKENDS {
        let config = Config { iters: Iters::Default, sleep: false, max_bodies: 64 };
        let mut backend = make_backend(name, &config).unwrap();
        let q = measure::run(&scene, backend.as_mut()).quality;
        assert_eq!((q.bodies, q.escaped), (20, 0), "{name}: {q:?}");
        assert!(q.mean_height < 1.5, "{name} has not landed: {q:?}");
    }
}
