//! Integration tests: real mod libraries, loaded and reloaded by a real
//! `Engine`, with no process or socket. Every reload goes through the same
//! staging, `dlopen` and state handoff as `./bazel run //mods/<name>`.
//!
//! Each reload loads a *different* build of the mod (`counter_v1` then
//! `counter_v2`, both named `counter`) and asserts on which build's code ran.
//! Reloading the same build would pass even if reloading silently did
//! nothing.

use std::path::PathBuf;

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

/// An engine with no bootstrap: tests step it frame by frame themselves.
/// Each test stages into its own directory: the test harness runs tests on
/// parallel threads, and staged names only differ per engine, not per process.
fn engine(test: &str) -> Box<Engine> {
    engine_with(test, None)
}

fn engine_with(test: &str, bootstrap: Option<&str>) -> Box<Engine> {
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").expect("TEST_TMPDIR")).join(test);
    Engine::new(bootstrap.map(Into::into), dir)
}

fn load(engine: &Engine, name: &str, var: &str) -> String {
    engine
        .load(name, &lib(var))
        .unwrap_or_else(|e| panic!("loading {var} as {name}: {e}"))
}

fn step(engine: &Engine, frames: usize) {
    for _ in 0..frames {
        engine.step_all();
    }
}

fn probes(engine: &Engine) -> Vec<Probe> {
    let world = engine.world();
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
        #[derive(Debug, Default, PartialEq, Copy)]
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

        let world = e.world();
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

/// Text sent to a mod with `modctl send`.
mod messages {
    use super::{engine, load, probe, step};

    #[test]
    fn a_mod_replies_to_a_message_and_can_act_on_it() {
        let e = engine("messages");
        load(&e, "counter", "COUNTER_V1");
        step(&e, 2);
        assert_eq!(e.send("counter", "get").as_deref(), Ok("2"));
        assert_eq!(e.send("counter", "add 40").as_deref(), Ok("42"));
        assert_eq!(probe(&e).value, 42, "the handler's change reached the world");
    }

    #[test]
    fn a_declined_message_is_an_error_carrying_the_mods_reason() {
        let e = engine("declined");
        load(&e, "counter", "COUNTER_V1");
        assert_eq!(e.send("counter", "jump").unwrap_err(), "counter doesn't understand \"jump\"");
        assert!(e.send("counter", "add x").unwrap_err().contains("\"x\""));
        // Declining isn't failing: the mod keeps running.
        assert!(!e.list().contains("[failed]"), "{}", e.list());
        step(&e, 1);
        assert_eq!(probe(&e).value, 1);
    }

    #[test]
    fn a_mod_without_a_handler_or_not_loaded_refuses() {
        let e = engine("no_handler");
        load(&e, "clock", "CLOCK");
        assert_eq!(e.send("clock", "hi").unwrap_err(), "clock doesn't take messages");
        assert_eq!(e.send("nobody", "hi").unwrap_err(), "nobody is not loaded");
    }

    #[test]
    fn a_message_handler_can_step_the_other_mods() {
        // What the lockstep bootstrap does, delivered directly, as a test
        // driving an Engine does, rather than through its pump.
        let e = engine("handler_steps");
        load(&e, "counter", "COUNTER_V1");
        load(&e, "clock", "CLOCK");
        load(&e, "lockstep", "LOCKSTEP");
        assert_eq!(e.send("lockstep", "step 5").as_deref(), Ok("frame 5"));
        assert_eq!(probe(&e).value, 5);
        assert!(e.send("lockstep", "step many").is_err());
    }
}

/// Components holding heap data (`Vec<String>`), across real builds: the
/// buffers are allocated by one library's code and grown, migrated and freed
/// by another's.
mod heap {
    use super::{engine, lib, load, step};

    mod v1 {
        engine_api::component! {
            #[derive(Default)]
            pub struct Bag: "test::Bag" {
                pub words: Vec<String>,
            }
        }
    }

    mod v3 {
        engine_api::component! {
            #[derive(Default)]
            pub struct Bag: "test::Bag" {
                pub note: String,
                pub words: Vec<String>,
            }
        }
    }

    fn words(e: &engine_loader::engine::Engine) -> Vec<String> {
        let world = e.world();
        let bags = world.values::<v1::Bag>().expect("test::Bag with v1's layout");
        assert_eq!(bags.len(), 1);
        bags.into_iter().next().unwrap().1.words
    }

    #[test]
    fn heap_data_written_by_one_build_is_grown_by_the_next() {
        let e = engine("heap_reload");
        load(&e, "bag", "BAG_V1");
        step(&e, 2);
        assert_eq!(load(&e, "bag", "BAG_V2"), "reloaded bag (generation 1)");
        step(&e, 2);
        assert_eq!(words(&e), ["v1-1", "v1-2", "v2-3", "v2-4"]);
    }

    #[test]
    fn heap_data_outlives_its_mod_and_is_freed_with_its_code() {
        let e = engine("heap_unload");
        load(&e, "bag", "BAG_V1");
        step(&e, 2);
        e.unload("bag").unwrap();
        // v1 is gone, but the world kept its library for the values' sake.
        assert_eq!(words(&e), ["v1-1", "v1-2"]);
        // Dropping the engine drops the values with v1's code; if the library
        // had been unmapped, this would crash the test binary.
        drop(e);

        // And a later build picks the values up where v1 left them.
        let e = engine("heap_unload_then_load");
        load(&e, "bag", "BAG_V1");
        step(&e, 1);
        e.unload("bag").unwrap();
        load(&e, "bag", "BAG_V2");
        step(&e, 1);
        assert_eq!(words(&e), ["v1-1", "v2-1"]);
    }

    #[test]
    fn heap_fields_migrate_between_real_builds() {
        let e = engine("heap_migrate");
        load(&e, "bag", "BAG_V1");
        step(&e, 2);
        load(&e, "bag", "BAG_V3");
        step(&e, 1);
        let world = e.world();
        let bags = world.values::<v3::Bag>().expect("test::Bag with v3's layout");
        let bag = &bags[0].1;
        assert_eq!(bag.words, ["v1-1", "v1-2", "v3-3"], "the Vec<String> moved intact");
        assert_eq!(bag.note, "3 words");
    }

    #[test]
    fn a_build_from_another_rustc_is_refused() {
        let e = engine("rustc");
        // The same library, claiming a different compiler in its `.comment`.
        let mut bytes = std::fs::read(lib("BAG_V1")).unwrap();
        let at = bytes.windows(14).position(|w| w == b"rustc version ").expect("rustc's .comment line");
        bytes[at + 14..at + 18].copy_from_slice(b"0.0.");
        let path = std::path::PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join("other_rustc.so");
        std::fs::write(&path, bytes).unwrap();
        let err = e.load("bag", &path).unwrap_err();
        assert!(err.starts_with("bag was built by rustc version 0.0."), "{err}");
        assert!(err.contains("restart the engine"), "{err}");
    }
}

/// State that points into its own build, carried across a reload: get-y5t.5.
mod state_pointing_into_its_build {
    use super::{engine, load, probe, step};

    #[test]
    fn done_right_the_closure_is_rebuilt_and_the_state_kept() {
        let e = engine("greeter_right");
        load(&e, "greeter", "GREETER_RIGHT_V1");
        step(&e, 2);
        assert_eq!(e.send("greeter", "hi").as_deref(), Ok("hello! (2 greetings, word hello)"));
        assert_eq!(load(&e, "greeter", "GREETER_RIGHT_V2"), "reloaded greeter (generation 1)");
        step(&e, 1);
        // v2's closure (rebuilt in its load), v1's count.
        assert_eq!(e.send("greeter", "hi").as_deref(), Ok("bonjour! (3 greetings, word bonjour)"));
    }

    #[test]
    fn done_wrong_the_state_is_reset_instead_of_crashing() {
        let e = engine("greeter_wrong");
        load(&e, "greeter", "GREETER_V1");
        step(&e, 2);
        let reply = load(&e, "greeter", "GREETER_V2");
        assert_eq!(reply, "reloaded greeter (generation 1, state held pointers into the old build, so it was reset)");
        step(&e, 1);
        assert_eq!(e.send("greeter", "hi").as_deref(), Ok("bonjour! (1 greetings, word bonjour)"));
    }

    #[test]
    fn a_state_that_gains_a_field_is_migrated_not_reset() {
        let e = engine("state_migrate");
        load(&e, "counter", "COUNTER_V1");
        step(&e, 3);
        let reply = load(&e, "counter", "COUNTER_V4");
        assert_eq!(
            reply,
            "reloaded counter (generation 1, state migrated: kept total, kept probe, added grown (default))"
        );
        step(&e, 1);
        // v1's total and probe, then one of v4's steps of 10.
        assert_eq!((probe(&e).value, probe(&e).build), (13, 5));
    }
}

/// Calls between mods: `caller` calls `calc`'s service (get-y5t.1).
mod calls {
    use super::{engine, lib, load};

    fn with_calc(test: &str) -> Box<engine_loader::engine::Engine> {
        let e = engine(test);
        load(&e, "calc", "CALC_V1");
        load(&e, "caller", "CALLER");
        e
    }

    fn call(e: &engine_loader::engine::Engine, message: &str) -> Result<String, String> {
        e.send("caller", message)
    }

    #[test]
    fn a_call_runs_in_the_provider_with_its_state() {
        let e = with_calc("call");
        assert_eq!(call(&e, "apply 5").as_deref(), Ok("6"));
        // One call from caller's load, one just now: counted in calc's state.
        assert_eq!(call(&e, "describe").as_deref(), Ok("calc v1 (2 calls)"));
        assert_eq!(call(&e, "at_load").as_deref(), Ok("101"), "a mod's load can call the mods it depends on");
    }

    #[test]
    fn borrowed_arguments_and_owned_returns_cross() {
        let e = with_calc("call_borrow");
        assert_eq!(call(&e, "greet world").as_deref(), Ok("hello, world"));
    }

    #[test]
    fn a_provider_reloaded_between_calls_is_called_in_its_new_build() {
        let e = with_calc("call_reload");
        assert_eq!(call(&e, "apply 5").as_deref(), Ok("6"));
        // Same interface, so caller isn't stranded and isn't reloaded.
        assert_eq!(load(&e, "calc", "CALC_V2"), "reloaded calc (generation 1)");
        assert_eq!(call(&e, "apply 5").as_deref(), Ok("10"));
        assert_eq!(call(&e, "describe").as_deref(), Ok("calc v2 (3 calls)"), "the provider's state carried over");
        assert!(e.list().contains("caller gen 0"), "{}", e.list());
    }

    #[test]
    fn a_panicking_provider_fails_the_call_and_is_marked_failed() {
        let e = with_calc("call_panic");
        assert_eq!(call(&e, "boom"), Err("calc::Calc::boom: its provider panicked".into()));
        assert!(e.list().contains("calc gen 0 [failed]"), "{}", e.list());
        assert_eq!(call(&e, "apply 5"), Err("calc::Calc::apply: its provider has failed; reload it".into()));
        // The caller carried on, and a reload of the provider brings it back.
        load(&e, "calc", "CALC_V2");
        assert_eq!(call(&e, "apply 5").as_deref(), Ok("10"));
    }

    #[test]
    fn a_call_back_into_a_running_mod_is_refused() {
        let e = with_calc("call_reentrant");
        assert_eq!(
            call(&e, "recurse").as_deref(),
            Ok("calc::Calc::apply: its provider is already running (a call back into a running mod)")
        );
        // Refusing it didn't leave calc marked running or failed.
        assert_eq!(call(&e, "apply 5").as_deref(), Ok("6"));
    }

    #[test]
    fn a_service_has_one_provider() {
        let e = with_calc("call_two_providers");
        let err = e.load("calc2", &lib("CALC_V2")).unwrap_err();
        assert_eq!(err, "calc2 and calc both provide calc::Calc");
    }
}

/// Resident mods: loaded once, never swapped (get-y5t.7). `vault` is resident
/// and owns a thread; `teller` calls it.
mod resident {
    use std::time::{Duration, Instant};

    use super::{engine, lib, load};

    fn with_vault(test: &str) -> Box<engine_loader::engine::Engine> {
        let e = engine(test);
        load(&e, "vault", "VAULT_V1");
        load(&e, "teller", "TELLER");
        e
    }

    fn ask(e: &engine_loader::engine::Engine, what: &str) -> String {
        e.send("teller", what).unwrap_or_else(|err| panic!("{what}: {err}"))
    }

    fn ticks(e: &engine_loader::engine::Engine) -> u64 {
        ask(e, "ticks").parse().unwrap()
    }

    /// Waits for vault's thread to count past `past`.
    fn ticks_past(e: &engine_loader::engine::Engine, past: u64) -> u64 {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let now = ticks(e);
            if now > past {
                return now;
            }
            assert!(Instant::now() < deadline, "vault's thread stopped at {now}");
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn a_new_build_of_a_resident_mod_is_refused() {
        let e = with_vault("resident_refuse");
        let err = e.load("vault", &lib("VAULT_V2")).unwrap_err();
        assert_eq!(err, "vault is resident: restart the engine to load its new build");
        assert!(e.list().contains("vault gen 0 [resident]"), "{}", e.list());
        assert_eq!(ask(&e, "build"), "v1");
        // Loading the build it already has is fine.
        assert_eq!(load(&e, "vault", "VAULT_V1"), "vault unchanged");
    }

    #[test]
    fn a_game_reload_keeps_the_resident_build_and_reloads_the_rest() {
        let e = with_vault("resident_batch");
        let reply = e.load_batch(&[("vault".into(), lib("VAULT_V2")), ("counter".into(), lib("COUNTER_V1"))]);
        assert_eq!(
            reply.as_deref(),
            Ok("loaded counter; vault is resident: restart the engine to load its new build")
        );
        assert_eq!(ask(&e, "build"), "v1");
    }

    #[test]
    fn its_transient_part_and_thread_last_the_session() {
        let e = with_vault("resident_thread");
        let first = ticks_past(&e, 0);
        // Other mods coming and going don't touch it.
        load(&e, "counter", "COUNTER_V1");
        load(&e, "counter", "COUNTER_V2");
        assert_eq!(ask(&e, "made"), "1", "one transient part for the session");
        ticks_past(&e, first);
    }

    /// Threads in this process whose name is `name`.
    fn threads_named(name: &str) -> usize {
        std::fs::read_dir("/proc/self/task")
            .unwrap()
            .filter_map(|t| std::fs::read_to_string(t.ok()?.path().join("comm")).ok())
            .filter(|comm| comm.trim_end() == name)
            .count()
    }

    #[test]
    fn dropping_the_engine_stops_a_resident_mods_thread() {
        // Its own name, so no other test's vault thread is counted.
        let e = engine("resident_shutdown");
        load(&e, "vault_solo", "VAULT_V1");
        // A thread names itself once it starts, and its /proc entry can
        // linger briefly after it has been joined, so both checks wait.
        let wait_for = |count: usize, what: &str| {
            let deadline = Instant::now() + Duration::from_secs(10);
            while threads_named("tick-vault_solo") != count {
                assert!(Instant::now() < deadline, "{what}");
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        wait_for(1, "vault's thread never started");
        // Dropping the engine closes vault, whose transient part stops and
        // joins the thread; otherwise it would keep running this library's
        // code after the engine is gone.
        drop(e);
        wait_for(0, "vault's thread outlived the engine");
    }

    #[test]
    fn a_resident_mod_cant_be_unloaded() {
        let e = with_vault("resident_unload");
        e.unload("teller").unwrap();
        assert_eq!(e.unload("vault").unwrap_err(), "vault is resident: it unloads when the engine exits");
    }

    #[test]
    fn a_resident_mod_may_depend_only_on_resident_mods() {
        // user_resident's dependency, base, is loaded here as the reloadable
        // base_v1: same interface, so only residency is wrong.
        let e = engine("resident_on_reloadable");
        load(&e, "base", "BASE_V1");
        let err = e.load("user", &lib("USER_RESIDENT")).unwrap_err();
        assert_eq!(err, "user is resident, so everything it depends on must be too; base isn't");

        let e = engine("resident_on_resident");
        load(&e, "base", "BASE_RESIDENT");
        assert_eq!(load(&e, "user", "USER_RESIDENT"), "loaded user");
    }
}

/// The bootstrap owning the loop (docs/architecture/overview.md, "Who runs
/// the loop"): the loader serves requests only when the bootstrap pumps it,
/// and only when nothing but resident mods is running.
mod pumping {
    use std::sync::mpsc::{self, Sender};
    use std::time::{Duration, Instant};

    use engine_api::Status;
    use engine_control::Request;
    use engine_loader::engine::Pending;

    use super::{engine, engine_with, lib, load, probe};

    /// Sends `request` as the control socket's thread would, and waits for the reply.
    fn request(requests: &Sender<Pending>, request: Request) -> String {
        let (tx, rx) = mpsc::channel();
        let reply = Box::new(move |reply: String| {
            let _ = tx.send(reply);
        });
        requests.send(Pending { text: request.encode(), reply }).expect("the engine is gone");
        rx.recv_timeout(Duration::from_secs(10)).expect("no reply: is the bootstrap pumping?").trim_end().to_string()
    }

    fn send(requests: &Sender<Pending>, name: &str, message: &str) -> String {
        request(requests, Request::Send { name: name.into(), message: message.into() })
    }

    /// Asks until the reply is `want`, as frames go by.
    fn until(requests: &Sender<Pending>, name: &str, message: &str, want: impl Fn(&str) -> bool) -> String {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let reply = send(requests, name, message);
            if want(&reply) || Instant::now() > deadline {
                return reply;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn an_event_loop_bootstrap_serves_requests_between_frames() {
        let e = engine_with("event_loop", Some("fake_os"));
        load(&e, "fake_os", "FAKE_OS");
        let requests = e.requests();
        let (v1, v2, fake_os) = (lib("COUNTER_V1"), lib("COUNTER_V2"), lib("FAKE_OS"));

        // Plays the other end of the control socket while this thread runs
        // the session. Whatever happens, it ends with a quit, so a failing
        // check can't leave the session running forever.
        let client = std::thread::spawn(move || {
            let checks = std::panic::catch_unwind(|| {
                let load = |name: &str, path: &std::path::Path| {
                    request(&requests, Request::Load { name: name.into(), path: path.into() })
                };
                assert_eq!(send(&requests, "fake_os", "size"), "ok 640x480");
                assert_eq!(load("counter", &v1), "ok loaded counter");
                let total = until(&requests, "counter", "get", |r| r != "ok 0");
                assert!(total.starts_with("ok "), "{total}");

                // A reload, applied between two frames while the loop runs.
                assert_eq!(load("counter", &v2), "ok reloaded counter (generation 1)");
                let total = |r: &str| r.strip_prefix("ok ").and_then(|n| n.parse::<u64>().ok());
                let after = until(&requests, "counter", "get", |r| total(r).is_some_and(|n| n >= 100));
                assert!(total(&after).is_some_and(|n| n >= 100), "v2 never stepped: {after}");

                // The bootstrap itself is resident: never swapped or unloaded.
                assert_eq!(load("fake_os", &fake_os), "ok fake_os unchanged");
                assert_eq!(
                    request(&requests, Request::Unload { name: "fake_os".into() }),
                    "err fake_os is resident: it unloads when the engine exits"
                );

                // Messages to the running bootstrap go to its pump handler,
                // and can feed its event loop.
                assert_eq!(send(&requests, "fake_os", "key a"), "ok queued");
                assert_eq!(send(&requests, "fake_os", "key b"), "ok queued");
                assert_eq!(until(&requests, "fake_os", "keys", |r| r == "ok ab"), "ok ab");
                assert_eq!(send(&requests, "counter", "pump"), "ok Refused");
            });
            assert_eq!(request(&requests, Request::Quit), "ok quitting");
            if let Err(panic) = checks {
                std::panic::resume_unwind(panic);
            }
        });

        assert_eq!(e.run_bootstrap(), Ok(Status::QUIT));
        client.join().expect("the client's checks failed");
        assert_eq!(probe(&e).build, 2, "the reload took effect while the loop ran");
        assert!(probe(&e).value > 100, "{:?}", probe(&e));
    }

    #[test]
    fn a_mod_that_isnt_resident_cant_pump() {
        let e = engine("pump_not_resident");
        load(&e, "counter", "COUNTER_V1");
        assert_eq!(e.send("counter", "pump").as_deref(), Ok("Refused"));
    }

    #[test]
    fn a_resident_mod_can_pump_only_at_the_top_of_the_stack() {
        // Delivering a message holds the mod list, so this isn't a safe point.
        let e = engine("pump_during_delivery");
        load(&e, "fake_os", "FAKE_OS");
        assert_eq!(e.send("fake_os", "pump").as_deref(), Ok("Refused"));
    }

    #[test]
    fn the_bootstrap_must_be_resident() {
        let e = engine_with("bootstrap_not_resident", Some("fake_os"));
        load(&e, "fake_os", "FAKE_OS_RELOADABLE");
        let err = e.run_bootstrap().unwrap_err();
        assert!(err.contains("must be resident"), "{err}");
    }

    #[test]
    fn the_bootstrap_must_be_one() {
        let e = engine_with("bootstrap_not_one", Some("counter"));
        let err = e.load("counter", &lib("COUNTER_V1")).unwrap_err();
        assert_eq!(
            err,
            "counter is the game's bootstrap, but isn't one: implement Bootstrap and export_mod!(.., bootstrap)"
        );
    }

    #[test]
    fn requests_wait_for_the_next_pump() {
        // With no bootstrap, `Engine::pump` stands in for one.
        let e = engine("pump_queue");
        let requests = e.requests();
        let (tx, rx) = mpsc::channel();
        let reply = Box::new(move |reply: String| tx.send(reply).unwrap());
        requests.send(Pending { text: Request::List.encode(), reply }).unwrap();
        assert!(rx.try_recv().is_err(), "served before a pump");
        assert_eq!(e.pump(Duration::ZERO), engine_api::Pumped::Continue);
        assert!(rx.try_recv().unwrap().starts_with("ok 0 mod(s) loaded"));
    }
}

/// Systems, phases, access, structural changes and events:
/// docs/architecture/scheduling.md and storage.md.
mod scheduling {
    use test_probe::Trace;

    use super::{engine, lib, load, step};

    fn trace(e: &engine_loader::engine::Engine) -> Vec<String> {
        let world = e.world();
        let traces = world.values::<Trace>().expect("test::Trace is registered with this layout");
        traces.into_iter().flat_map(|(_, t)| t.lines).collect()
    }

    /// The trace from the frames run since `from` lines.
    fn since(e: &engine_loader::engine::Engine, from: usize) -> Vec<String> {
        trace(e)[from..].to_vec()
    }

    #[test]
    fn a_frame_runs_systems_by_phase_then_constraints() {
        let e = engine("sched_order");
        load(&e, "a", "TRACER_A");
        load(&e, "b", "TRACER_B");
        step(&e, 1);
        // Load order alone would run a's systems first.
        assert_eq!(trace(&e), ["a::input", "b::update", "a::update", "b::simulate", "a::late"]);
        assert_eq!(
            e.schedule().unwrap(),
            "input: a::input\nupdate: b::update, a::update\nsimulate (60 Hz): b::simulate\nlate: a::late"
        );
    }

    #[test]
    fn a_load_that_would_order_systems_in_a_cycle_is_refused() {
        let e = engine("sched_cycle");
        load(&e, "a", "TRACER_A");
        load(&e, "b", "TRACER_B");
        let err = e.load("b", &lib("TRACER_B_CYCLE")).unwrap_err();
        assert_eq!(err, "the systems a::update, b::update are ordered in a cycle");
        // The running b is untouched.
        assert!(e.list().contains("b gen 0"), "{}", e.list());
        step(&e, 1);
        assert_eq!(trace(&e).len(), 5);
    }

    #[test]
    fn a_mod_without_systems_runs_nothing_each_frame() {
        let e = engine("sched_none");
        load(&e, "mover", "MOVER_V1");
        load(&e, "a", "TRACER_A");
        assert_eq!(e.schedule().unwrap(), "input: a::input\nupdate: a::update\nlate: a::late");
    }

    #[test]
    fn reaching_for_the_whole_world_from_a_system_fails_the_mod() {
        let e = engine("sched_world");
        load(&e, "sneak", "SNEAK_WORLD");
        step(&e, 1);
        assert!(e.list().contains("sneak gen 0 [failed]"), "{}", e.list());
        // What it did through its parameters before reaching stands, and
        // its next frames don't run.
        assert_eq!(trace(&e), ["sneak ran"]);
        step(&e, 2);
        assert_eq!(trace(&e).len(), 1);
    }

    #[test]
    fn an_insert_is_seen_by_the_systems_after_the_one_that_made_it() {
        let e = engine("sched_insert");
        load(&e, "builder", "BUILDER");
        step(&e, 1);
        assert_eq!(trace(&e), ["saw []", "inserted", "build saw []", "saw [7]"]);
        assert!(e.world().summary().contains("test::Probe x1"), "{}", e.world().summary());
    }

    #[test]
    fn each_reader_sees_each_event_once_from_the_apply_that_publishes_it() {
        let e = engine("sched_events");
        load(&e, "pinger", "PINGER");
        load(&e, "listener", "LISTENER_V1");
        step(&e, 3);
        // Ping n is sent in frame n's simulate, and published when
        // `pinger::ping` returns: `mid` and `late`, after it, see it that
        // frame; `early`, before it, the next.
        let frames: Vec<Vec<String>> = trace(&e).chunks(3).map(|c| c.to_vec()).collect();
        assert_eq!(frames[0], ["v1 saw []", "v1 saw [1]", "v1 saw [1]"]);
        assert_eq!(frames[1], ["v1 saw [1]", "v1 saw [2]", "v1 saw [2]"]);
        assert_eq!(frames[2], ["v1 saw [2]", "v1 saw [3]", "v1 saw [3]"]);
    }

    #[test]
    fn an_event_lives_until_the_end_of_the_next_frame() {
        let e = engine("sched_expire");
        load(&e, "pinger", "PINGER");
        step(&e, 3);
        // Pings 1 and 2 have expired; 3 was sent last frame, so it lasts
        // through this one.
        load(&e, "listener", "LISTENER_V1");
        step(&e, 1);
        assert_eq!(trace(&e), ["v1 saw [3]", "v1 saw [3, 4]", "v1 saw [3, 4]"]);
    }

    #[test]
    fn a_reloaded_reader_picks_up_where_it_left_off() {
        let e = engine("sched_cursor");
        load(&e, "pinger", "PINGER");
        load(&e, "listener", "LISTENER_V1");
        step(&e, 1);
        load(&e, "listener", "LISTENER_V2");
        step(&e, 1);
        // Cursors are per system: `early` hadn't seen ping 1; `mid` and
        // `late` had, and don't see it again though it's still alive.
        assert_eq!(since(&e, 3), ["v2 saw [1]", "v2 saw [2]", "v2 saw [2]"]);
    }

    #[test]
    fn an_event_sent_between_frames_is_seen_at_the_start_of_the_next() {
        let e = engine("sched_message_event");
        load(&e, "pinger", "PINGER");
        load(&e, "listener", "LISTENER_V1");
        e.send("pinger", "ping 99").unwrap();
        step(&e, 1);
        assert_eq!(trace(&e), ["v1 saw [99]", "v1 saw [99, 1]", "v1 saw [99, 1]"]);
    }
}

/// Fixed-rate phases: docs/architecture/scheduling.md, "Fixed rates".
mod rates {
    use test_probe::Trace;

    use super::{engine, lib, load};

    fn trace(e: &engine_loader::engine::Engine) -> Vec<String> {
        e.world().values::<Trace>().unwrap().into_iter().flat_map(|(_, t)| t.lines).collect()
    }

    fn count(lines: &[String], what: &str) -> usize {
        lines.iter().filter(|l| l.contains(what)).count()
    }

    /// Runs one frame covering `seconds`, as a bootstrap would.
    fn frame(e: &engine_loader::engine::Engine, seconds: f32) {
        e.set_frame_time(seconds);
        e.step_all();
    }

    #[test]
    fn each_phase_runs_at_its_rate_with_its_step() {
        let e = engine("rates_each");
        load(&e, "rates", "RATES_V1");
        for _ in 0..6 {
            frame(&e, 1.0 / 60.0);
        }
        let t = trace(&e);
        assert_eq!((count(&t, " frame "), count(&t, " 60 "), count(&t, " 10 ")), (6, 6, 1), "{t:?}");
        assert!(t.contains(&"v1 60 0.0167".to_string()) && t.contains(&"v1 10 0.1000".to_string()), "{t:?}");
        // The ten-hertz step comes on the sixth frame, after its sixty.
        assert_eq!(t[t.len() - 1], "v1 10 0.1000");
    }

    #[test]
    fn a_long_frame_takes_several_steps_and_a_short_one_may_take_none() {
        let e = engine("rates_long_short");
        load(&e, "rates", "RATES_V1");
        frame(&e, 3.0 / 60.0);
        assert_eq!(count(&trace(&e), " 60 "), 3, "three steps' worth");
        frame(&e, 0.5 / 60.0);
        assert_eq!(count(&trace(&e), " 60 "), 3, "half a step: none yet");
        frame(&e, 0.5 / 60.0);
        assert_eq!(count(&trace(&e), " 60 "), 4, "the other half");
        assert_eq!(count(&trace(&e), " frame "), 3, "once a frame, whatever its length");
    }

    #[test]
    fn a_stall_takes_at_most_eight_steps_and_drops_the_rest() {
        let e = engine("rates_stall");
        load(&e, "rates", "RATES_V1");
        frame(&e, 1.0);
        assert_eq!(count(&trace(&e), " 60 "), 8);
        frame(&e, 1.0 / 60.0);
        assert_eq!(count(&trace(&e), " 60 "), 9, "the backlog is gone, not paid over the next frames");
    }

    #[test]
    fn time_toward_a_step_survives_a_reload() {
        let e = engine("rates_reload");
        load(&e, "rates", "RATES_V1");
        for _ in 0..3 {
            frame(&e, 1.0 / 60.0);
        }
        e.load("rates", &lib("RATES_V2")).unwrap();
        for _ in 0..3 {
            frame(&e, 1.0 / 60.0);
        }
        let t = trace(&e);
        assert_eq!(count(&t, "v1 10"), 0);
        assert_eq!(count(&t, "v2 10"), 1, "three frames before the reload and three after make a step: {t:?}");
    }
}

/// Scheduling policy as a mod: docs/architecture/scheduling.md, "Who schedules".
mod schedulers {
    use test_probe::Trace;

    use super::{engine_with, load};

    fn trace(e: &engine_loader::engine::Engine) -> Vec<String> {
        let world = e.world();
        world.values::<Trace>().unwrap().into_iter().flat_map(|(_, t)| t.lines).collect()
    }

    /// The tracer mods under the lockstep bootstrap, whose `step` runs a
    /// frame through `cx.run_frame()`, as every bootstrap does.
    fn game(test: &str, scheduler: Option<&str>) -> Box<engine_loader::engine::Engine> {
        let e = engine_with(test, Some("lockstep"));
        load(&e, "clock", "CLOCK");
        load(&e, "lockstep", "LOCKSTEP");
        if let Some(var) = scheduler {
            load(&e, "scheduler", var);
        }
        load(&e, "a", "TRACER_A");
        load(&e, "b", "TRACER_B");
        e
    }

    fn frame(e: &engine_loader::engine::Engine) {
        e.send("lockstep", "step").unwrap();
    }

    const FRAME: [&str; 5] = ["a::input", "b::update", "a::update", "b::simulate", "a::late"];

    #[test]
    fn a_scheduler_mod_runs_the_frame_the_loader_would() {
        let e = game("sched_mod", Some("SCHEDULER_V1"));
        frame(&e);
        let mut expected = vec!["scheduled by v1".to_string()];
        expected.extend(FRAME.map(String::from));
        assert_eq!(trace(&e), expected);

        let without = game("sched_loader", None);
        frame(&without);
        assert_eq!(trace(&without), FRAME);
    }

    #[test]
    fn the_default_scheduler_runs_the_same_frame() {
        let e = engine_with("sched_sequential", Some("lockstep"));
        load(&e, "clock", "CLOCK");
        load(&e, "lockstep", "LOCKSTEP");
        load(&e, "sequential", "SEQUENTIAL");
        load(&e, "a", "TRACER_A");
        load(&e, "b", "TRACER_B");
        frame(&e);
        assert_eq!(trace(&e), FRAME);
        assert!(e.list().contains("sequential gen 0"), "{}", e.list());
    }

    #[test]
    fn a_scheduler_hot_reloads_between_frames() {
        let e = game("sched_reload", Some("SCHEDULER_V1"));
        frame(&e);
        assert_eq!(load(&e, "scheduler", "SCHEDULER_V2"), "reloaded scheduler (generation 1)");
        frame(&e);
        assert_eq!(trace(&e)[6], "scheduled by v2");
    }

    #[test]
    fn a_panicking_scheduler_ends_its_frame_and_the_loader_takes_over() {
        let e = game("sched_panic", Some("SCHEDULER_PANIC"));
        // The first node ran before the panic; the rest of the frame didn't.
        assert!(e.send("lockstep", "step").is_ok());
        assert_eq!(trace(&e), ["scheduled by panic", "a::input"]);
        assert!(e.list().contains("scheduler gen 0 [failed]"), "{}", e.list());
        // The frame was closed as the panic unwound, so the next one opens,
        // on the loader's own scheduler.
        frame(&e);
        assert_eq!(trace(&e)[2..], FRAME);
    }

    #[test]
    fn a_frame_inside_a_frame_is_refused() {
        let e = engine_with("sched_nested", None);
        load(&e, "sneak", "SNEAK_NESTED");
        e.step_all();
        assert_eq!(trace(&e), ["sneak ran", "sneak got Status(-1)"]);
    }
}
