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
fn a_new_build_at_the_same_path_is_loaded_fresh() {
    // What Bazel does to bazel-bin: a new build replaces the file at the path
    // the engine already loaded from. Without staging, dlopen would hand back
    // the image already mapped, v1's code would keep running and its statics
    // would carry over.
    let e = engine("same_path");
    let path = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join("libcounter.so");
    std::fs::copy(lib("COUNTER_V1"), &path).unwrap();
    e.load("counter", &path).unwrap();
    assert_eq!(probe(&e).loads_seen_by_statics, 1);

    // Replaced, not overwritten, as Bazel does (and the copy is read-only).
    std::fs::remove_file(&path).unwrap();
    std::fs::copy(lib("COUNTER_V2"), &path).unwrap();
    assert_eq!(e.load("counter", &path).as_deref(), Ok("reloaded counter (generation 1)"));
    step(&e, 1);
    assert_eq!(probe(&e).build, 2, "v1's image was reused");
    assert_eq!(probe(&e).loads_seen_by_statics, 1, "statics survived the reload");
}

#[test]
fn an_unchanged_build_is_not_reloaded() {
    let e = engine("unchanged");
    load(&e, "counter", "COUNTER_V1");
    step(&e, 2);
    assert_eq!(load(&e, "counter", "COUNTER_V1"), "counter unchanged");
    assert!(e.list().contains("counter gen 0"), "{}", e.list());
    assert_eq!(probe(&e).value, 2);
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

/// Mods that depend on another mod's interface: `user` on `base`.
mod deps {
    use super::{engine, lib, load};

    #[test]
    fn a_mod_needs_its_dependencies_loaded() {
        let e = engine("needs_deps");
        let err = e.load("user", &lib("USER_V1")).unwrap_err();
        assert_eq!(err, "user depends on base, which isn't loaded");
    }

    #[test]
    fn an_implementation_change_reloads_alone() {
        let e = engine("impl_change");
        load(&e, "base", "BASE_V1");
        load(&e, "user", "USER_V1");
        // Same interface as v1, so user is still built against the running one.
        assert_eq!(load(&e, "base", "BASE_V1B"), "reloaded base (generation 1)");
        assert!(e.list().contains("user gen 0"), "{}", e.list());
    }

    #[test]
    fn an_interface_change_that_would_strand_a_dependent_is_rejected() {
        let e = engine("strand");
        load(&e, "base", "BASE_V1");
        load(&e, "user", "USER_V1");
        let err = e.load("base", &lib("BASE_V2")).unwrap_err();
        assert!(
            err.contains("base's interface changed, and user was built against the old one"),
            "{err}"
        );
        assert!(e.list().contains("base gen 0"), "nothing may change:\n{}", e.list());
    }

    #[test]
    fn the_rejection_names_the_game_reload_target() {
        let e = engine("hint");
        e.set_reload_hint(Some("//game:reload".into()));
        load(&e, "base", "BASE_V1");
        load(&e, "user", "USER_V1");
        let err = e.load("base", &lib("BASE_V2")).unwrap_err();
        assert!(err.contains("`./bazel run //game:reload`"), "{err}");
    }

    #[test]
    fn a_dependent_built_against_another_interface_is_rejected() {
        let e = engine("wrong_interface");
        load(&e, "base", "BASE_V1");
        let err = e.load("user", &lib("USER_V2")).unwrap_err();
        assert_eq!(err, "user was built against a different interface of base than the running one");
    }

    #[test]
    fn a_batch_reloads_a_dependency_with_its_dependents() {
        let e = engine("batch");
        load(&e, "base", "BASE_V1");
        load(&e, "user", "USER_V1");
        // Listed dependent-first: the engine orders the batch itself.
        let msg = e.load_batch(&[("user".into(), lib("USER_V2")), ("base".into(), lib("BASE_V2"))]);
        assert_eq!(msg.as_deref(), Ok("reloaded base (generation 1); reloaded user (generation 1)"));
    }

    #[test]
    fn a_batch_with_one_bad_build_changes_nothing() {
        let e = engine("atomic");
        load(&e, "base", "BASE_V1");
        load(&e, "user", "USER_V1");
        let garbage = std::path::PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join("bad.so");
        std::fs::write(&garbage, "not an ELF file").unwrap();
        let err = e
            .load_batch(&[
                ("base".into(), lib("BASE_V2")),
                ("user".into(), lib("USER_V2")),
                ("broken".into(), garbage),
            ])
            .unwrap_err();
        assert!(err.contains("broken: dlopen failed"), "{err}");
        let list = e.list();
        assert!(list.contains("base gen 0") && list.contains("user gen 0") && !list.contains("broken"), "{list}");
    }

    #[test]
    fn a_batch_loads_dependencies_first_and_skips_unchanged_builds() {
        let e = engine("order");
        let batch = [("user".into(), lib("USER_V1")), ("base".into(), lib("BASE_V1"))];
        assert_eq!(e.load_batch(&batch).as_deref(), Ok("loaded base; loaded user"));
        assert_eq!(e.load_batch(&batch).as_deref(), Ok("user unchanged; base unchanged"));
    }

    #[test]
    fn a_mod_with_dependents_cannot_be_unloaded() {
        let e = engine("unload_deps");
        load(&e, "base", "BASE_V1");
        load(&e, "user", "USER_V1");
        assert_eq!(e.unload("base").unwrap_err(), "base is needed by user; unload them first");
        assert!(e.unload("user").is_ok());
        assert!(e.unload("base").is_ok());
    }
}
