//! Integration tests: real mod libraries, loaded and reloaded by a real
//! `Engine`, with no process or socket. Every reload goes through the same
//! staging, `dlopen` and state handoff as `./bazel run //mods/<name>`.
//!
//! Each reload loads a *different* build of the mod (`counter_v1` then
//! `counter_v2`, both named `counter`) and asserts on which build's code ran.
//! Reloading the same build would pass even if reloading silently did
//! nothing.

use std::path::PathBuf;

use engine_api::Status;
use engine_loader::engine::Engine;
use runfiles::Runfiles;
use test_probe::Probe;

/// The library built by the `engine_mod` target whose rlocation is in `$var`.
fn lib(var: &str) -> PathBuf {
    let rlocation = std::env::var(var).unwrap_or_else(|_| panic!("${var} is not set"));
    Runfiles::create()
        .expect("runfiles")
        .rlocation(&rlocation)
        .unwrap_or_else(|| panic!("{rlocation} is not in runfiles"))
}

/// An engine driven by the pacing-free `driver` mod. Each test stages into its
/// own directory: the test harness runs tests on parallel threads, and staged
/// names only differ per engine, not per process.
fn engine(test: &str) -> Box<Engine> {
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").expect("TEST_TMPDIR")).join(test);
    let engine = Engine::new(Some("driver".into()), dir);
    load(&engine, "driver", "DRIVER");
    engine
}

fn load(engine: &Engine, name: &str, var: &str) -> String {
    engine
        .load(name, &lib(var))
        .unwrap_or_else(|e| panic!("loading {var} as {name}: {e}"))
}

fn step(engine: &Engine, frames: usize) {
    for _ in 0..frames {
        assert_eq!(engine.step_bootstrap(), Some(Status::OK), "{}", engine.list());
    }
}

fn probes(engine: &Engine) -> Vec<Probe> {
    let world = engine.world().borrow();
    let values = world.values::<Probe>().expect("test::Probe is registered with this layout");
    values.into_iter().map(|(_, probe)| probe).collect()
}

/// The single probe the counter mod maintains.
fn probe(engine: &Engine) -> Probe {
    match probes(engine).as_slice() {
        [probe] => *probe,
        other => panic!("expected exactly one probe, found {other:?}"),
    }
}

#[test]
fn reload_keeps_state_and_runs_the_new_code() {
    let e = engine("reload_keeps_state");
    load(&e, "counter", "COUNTER_V1");
    step(&e, 3);
    assert_eq!((probe(&e).value, probe(&e).build), (3, 1));

    let msg = load(&e, "counter", "COUNTER_V2");
    assert_eq!(msg, "reloaded counter (generation 1)");
    step(&e, 2);
    // 3 from v1's state, then two of v2's steps of 100.
    assert_eq!((probe(&e).value, probe(&e).build), (203, 2));
}

#[test]
fn every_load_maps_a_fresh_copy_of_the_library() {
    // Reloading the *same* file: without staging, dlopen would hand back the
    // image already mapped and the mod's statics would carry over.
    let e = engine("fresh_copy");
    load(&e, "counter", "COUNTER_V1");
    assert_eq!(probe(&e).loads_seen_by_statics, 1);
    load(&e, "counter", "COUNTER_V1");
    assert_eq!(probe(&e).loads_seen_by_statics, 1, "statics survived the reload");
}

#[test]
fn incompatible_state_layout_resets_state_through_the_old_build() {
    let e = engine("state_reset");
    load(&e, "counter", "COUNTER_V1");
    step(&e, 3);

    let msg = load(&e, "counter", "COUNTER_V3");
    assert!(msg.contains("state was reset"), "{msg}");
    step(&e, 1);
    // One probe, not two: v1's `close` ran and despawned the probe its state
    // tracked before v3 started from `Default`.
    assert_eq!((probe(&e).value, probe(&e).build), (1000, 3));
}

#[test]
fn a_bad_build_leaves_the_old_build_running() {
    let e = engine("bad_build");
    load(&e, "counter", "COUNTER_V1");
    step(&e, 2);

    let garbage = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join("garbage.so");
    std::fs::write(&garbage, "not an ELF file").unwrap();
    let cases = [
        (garbage, "dlopen failed"),
        (lib("NO_ENTRY"), "engine_mod_info"),
        (lib("WRONG_API"), "mod API"),
    ];
    for (path, expected) in cases {
        let err = e.load("counter", &path).expect_err("a bad build must be rejected");
        assert!(err.contains(expected), "{} gave {err:?}, expected it to mention {expected:?}", path.display());
    }

    step(&e, 2);
    assert_eq!((probe(&e).value, probe(&e).build), (4, 1));
}

#[test]
fn a_panicking_mod_is_disabled_until_reloaded() {
    let e = engine("panic");
    load(&e, "counter", "COUNTER_V1");
    step(&e, 2);

    load(&e, "counter", "COUNTER_PANIC");
    step(&e, 3);
    assert_eq!(probe(&e).value, 2, "a panicking step must not change state");
    assert!(e.list().contains("counter gen 1 [failed]"), "{}", e.list());

    load(&e, "counter", "COUNTER_V1");
    assert!(!e.list().contains("[failed]"), "{}", e.list());
    step(&e, 1);
    assert_eq!((probe(&e).value, probe(&e).build), (3, 1));
}

#[test]
fn unload_closes_the_mod() {
    let e = engine("unload");
    load(&e, "counter", "COUNTER_V1");
    assert_eq!(probes(&e).len(), 1);

    assert_eq!(e.unload("counter").as_deref(), Ok("unloaded counter"));
    assert_eq!(probes(&e).len(), 0, "close should have despawned the probe");
    assert!(e.unload("counter").is_err());
}

mod migration {
    use engine_api::component;

    // Mirrors `test::Pos` as built by mover_v2.
    component! {
        #[derive(Debug, Default, PartialEq)]
        pub struct Pos: "test::Pos" {
            pub y: f64,
            pub x: f32,
            pub z: f32,
        }
    }

    #[test]
    fn a_newer_build_migrates_component_values_to_its_layout() {
        let e = super::engine("migration");
        super::load(&e, "mover", "MOVER_V1");
        super::load(&e, "mover", "MOVER_V2");

        let world = e.world().borrow();
        let values = world.values::<Pos>().expect("test::Pos should now have mover_v2's layout");
        let values: Vec<Pos> = values.into_iter().map(|(_, pos)| pos).collect();
        // One entity, spawned by v1 and never respawned: its fields carried
        // over by name, `y` widened, and `z` from v2's `Default`.
        assert_eq!(values, [Pos { y: 2.5, x: 1.5, z: 7.0 }]);
    }
}
