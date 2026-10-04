//! Every backend links, runs a small pile to rest and keeps it above the
//! floor, measured by the harness's own geometry. Small enough for fastbuild.

use physics3d_compare::ours::Ours;
use physics3d_compare::scenes::{self, Kind};
use physics3d_compare::{BACKENDS, Backend, Config, Iters, State, Threads, make_backend, measure};

fn settles(kind: Kind, rotate: bool) {
    let mut scene = scenes::build(kind, 20);
    scene.steps = 240;
    for name in BACKENDS {
        let config = Config {
            iters: Iters::Default,
            sleep: false,
            max_bodies: 64,
            rotate,
            tune: "",
            gravity: physics3d_compare::scenes::EARTH,
            substeps: 0,
            threads: Threads::One,
        };
        let mut backend = make_backend(name, &config).unwrap();
        let run = measure::run(&scene, backend.as_mut());
        let q = &run.quality;
        assert_eq!(q.bodies, 20, "{name}");
        assert_eq!(q.escaped, 0, "{name}: {q:?}");
        // Turning spheres roll on in every engine (none has rolling
        // resistance on): only that they have slowed.
        let rest = if rotate && kind == Kind::SpherePile { 5.0 } else { 0.1 };
        assert!(q.max_speed < rest, "{name} still moving: {q:?}");
        assert!(q.pen_max < 0.05, "{name} penetrates: {q:?}");
        // Resting on something: nothing floats, and something reached the floor.
        assert_eq!(q.histogram[0], 0, "{name}: {q:?}");
        assert!(run.native_touching > 0, "{name} reports no contacts");
    }
}

#[test]
fn spheres_settle() {
    settles(Kind::SpherePile, false);
    settles(Kind::SpherePile, true);
}

#[test]
fn boxes_settle() {
    settles(Kind::BoxPile, false);
    settles(Kind::BoxPile, true);
}

#[test]
fn rain_lands() {
    for rotate in [false, true] {
        rain(rotate);
    }
}

fn rain(rotate: bool) {
    let mut scene = scenes::build(Kind::Rain, 20);
    scene.steps = scene.spawn.len() + 200;
    for name in BACKENDS {
        let config = Config {
            iters: Iters::Default,
            sleep: false,
            max_bodies: 64,
            rotate,
            tune: "",
            gravity: physics3d_compare::scenes::EARTH,
            substeps: 0,
            threads: Threads::One,
        };
        let mut backend = make_backend(name, &config).unwrap();
        let q = measure::run(&scene, backend.as_mut()).quality;
        assert_eq!((q.bodies, q.escaped), (20, 0), "{name}: {q:?}");
        assert!(q.mean_height < 1.5, "{name} has not landed: {q:?}");
    }
}

/// Ours where `Config::threads` puts it: alone where the bench times every
/// engine, on a pool of its own for `--threads`. Its step is the same bits
/// on each, so only the world's executor can tell them apart.
#[test]
fn ours_runs_where_its_config_says() {
    let mut scene = scenes::build(Kind::BoxPile, 20);
    scene.steps = 120;
    let config = |threads| Config {
        iters: Iters::Default,
        sleep: false,
        max_bodies: 64,
        rotate: true,
        tune: "",
        gravity: physics3d_compare::scenes::EARTH,
        substeps: 0,
        threads,
    };
    let mut ends: Vec<Vec<State>> = Vec::new();
    for (threads, executor, name) in [(Threads::One, None, "ours"), (Threads::Pool(2), Some(2), "ours, 2 threads")] {
        let mut ours = Ours::new(&config(threads));
        assert_eq!(ours.threads(), executor, "{threads:?}");
        assert_eq!(ours.name(), name);
        measure::run(&scene, &mut ours);
        let mut end = Vec::new();
        ours.state(&mut end);
        ends.push(end);
    }
    assert_eq!(format!("{:?}", ends[0]), format!("{:?}", ends[1]), "ours alone and on two threads part");
}
