//! The world across builds: components installed, taken over and migrated
//! by newer builds; events published and read by systems; and the world
//! between frames.

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Build, Entity, EventReader, EventWriter, Query, Structural, World, component, event};

fn build(name: &str, loaded_at: u64) -> Build {
    Build { name: name.into(), loaded_at, keepalive: None }
}

/// Two builds' layouts of one component, `test::Pos`, as two mods (or two
/// builds of one) would declare it.
mod v1 {
    engine_ecs::component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        pub struct Pos: "test::Pos" {
            pub x: f32,
            pub y: f32,
            pub w: u32,
        }
    }
}

mod v2 {
    engine_ecs::component! {
        /// Reordered, `y` widened, `w` dropped, `z` added with a default.
        #[derive(Debug, PartialEq, Copy)]
        pub struct Pos: "test::Pos" {
            pub y: f64,
            pub x: f32,
            pub z: f32,
        }
    }
    impl Default for Pos {
        fn default() -> Self {
            Pos { y: 0.0, x: 0.0, z: 7.0 }
        }
    }
}

mod v1_bumped {
    engine_ecs::component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        pub struct Pos: "test::Pos", version = 1 {
            pub x: f32,
            pub y: f32,
            pub w: u32,
        }
    }
}

mod sparse_pos {
    engine_ecs::component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        pub struct Pos: "test::Pos", storage = sparse {
            pub x: f32,
            pub y: f32,
            pub w: u32,
        }
    }
}

component! {
    #[derive(Debug, Default, PartialEq)]
    struct Name: "test::Name" { text: String }
}

/// A world with a few `v1::Pos` entities, some in a second table.
fn world() -> (World, Vec<Entity>) {
    let w = World::new();
    let es = {
        let mut m = w.between_frames(build("a", 1)).unwrap();
        let mut es: Vec<Entity> = (0..3).map(|i| m.spawn((v1::Pos { x: i as f32, y: 1.0, w: 9 },))).collect();
        es.push(m.spawn((v1::Pos { x: 5.0, y: 2.0, w: 9 }, Name { text: "named".into() })));
        es
    };
    (w, es)
}

#[test]
fn a_newer_layout_migrates_values_in_every_table() {
    let (w, es) = world();
    let report = w.install(&engine_ecs::ComponentDesc::of::<v2::Pos>(), &build("b", 2)).unwrap().unwrap();
    assert!(report.starts_with("migrated 4 value(s) of test::Pos to b's layout"), "{report}");
    let values = w.values::<v2::Pos>().expect("installed as v2");
    let named = values.iter().find(|(e, _)| *e == es[3]).unwrap().1;
    assert_eq!(named, v2::Pos { y: 2.0, x: 5.0, z: 7.0 });
    // The other components of a migrated row are untouched.
    assert_eq!(w.values::<Name>().unwrap(), [(es[3], Name { text: "named".into() })]);
    assert!(w.values::<v1::Pos>().is_none(), "v1's layout is gone");
}

#[test]
fn a_version_bump_resets_values_to_the_new_default() {
    let (w, es) = world();
    let report = w.install(&engine_ecs::ComponentDesc::of::<v1_bumped::Pos>(), &build("b", 2)).unwrap().unwrap();
    assert!(report.contains("its version changed; reset 4 value(s)"), "{report}");
    let values = w.values::<v1_bumped::Pos>().unwrap();
    assert_eq!(values.len(), 4, "entities keep the component");
    assert!(values.iter().all(|(_, p)| *p == v1_bumped::Pos::default()));
    assert!(w.entities.is_alive(es[0]));
}

#[test]
fn an_older_build_with_another_layout_is_refused() {
    let (w, _) = world();
    w.install(&engine_ecs::ComponentDesc::of::<v2::Pos>(), &build("b", 5)).unwrap();
    let err = w.install(&engine_ecs::ComponentDesc::of::<v1::Pos>(), &build("old", 3)).unwrap_err();
    assert_eq!(err, "old was built with an older layout of test::Pos; reload it with the rest of the game");
}

#[test]
fn a_component_cannot_change_storage_without_a_restart() {
    let (w, _) = world();
    let err = w.install(&engine_ecs::ComponentDesc::of::<sparse_pos::Pos>(), &build("b", 2)).unwrap_err();
    assert!(err.contains("restart the engine to change a component's storage"), "{err}");
}

#[test]
#[should_panic(expected = "another layout of test::Pos than the one installed")]
fn a_system_built_against_another_layout_fails_when_it_runs() {
    let (w, _) = world();
    fn read(_: &mut Cx, mut q: Query<&v1::Pos>) {
        q.for_each(|_, _| {});
    }
    let s = Schedule { systems: vec![read.system(&w, "read")] };
    w.install(&engine_ecs::ComponentDesc::of::<v2::Pos>(), &build("b", 9)).unwrap();
    s.run_sequential(&w);
}

#[test]
fn the_world_between_frames_is_refused_in_one() {
    let w = World::new();
    w.begin_frame();
    assert!(w.between_frames(Build::default()).is_err());
    w.end_frame();
    assert!(w.between_frames(Build::default()).is_ok());
}

#[test]
fn a_system_that_panics_holding_a_guard_leaves_it_usable() {
    let (w, _) = world();
    fn boom(_: &mut Cx, mut q: Query<&mut v1::Pos>) {
        q.for_each(|_, p| p.w = 0);
        panic!("the system fails holding its guard");
    }
    let s = Schedule { systems: vec![boom.system(&w, "boom")] };
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| s.run_sequential(&w))).is_err());
    // Poisoned, which isn't contention: its writes stand, and the next
    // frame's systems take the guard.
    fn read(_: &mut Cx, mut q: Query<&v1::Pos>) {
        q.for_each(|_, p| assert_eq!(p.w, 0));
    }
    Schedule { systems: vec![read.system(&w, "read")] }.run_sequential(&w);
    assert!(w.values::<v1::Pos>().unwrap().iter().all(|(_, p)| p.w == 0));
}

component! {
    #[derive(Debug, Default, PartialEq)]
    struct Slot: "test::Slot", storage = sparse { n: u32 }
}

#[test]
fn a_reused_index_does_not_inherit_a_dead_entitys_sparse_entry() {
    let w = World::new();
    let c = w.between_frames(Build::default()).unwrap().id::<Slot>();
    let old = w.entities.reserve();
    let mut s = Structural::new(&w);
    s.spawn_empty(old);
    s.insert_id(old, c, Slot { n: 1 });
    s.despawn(old);
    drop(s);

    // The same index, a new generation; the set still has `old`'s entry.
    let new = w.entities.reserve();
    assert_eq!((new.index, new.generation), (old.index, old.generation + 1));
    let mut s = Structural::new(&w);
    s.lock_sparse_with(c, false);
    s.spawn_empty(new);
    s.insert_id(new, c, Slot { n: 2 });
    drop(s);

    let set = w.sparse_set(c).read().unwrap();
    assert_eq!(set.get::<Slot>(new), Some(&Slot { n: 2 }));
    assert_eq!(set.get::<Slot>(old), None, "the dead entity's entry is gone");
}

mod events {
    use std::sync::Mutex;

    use super::*;

    event! {
        #[derive(Debug, Default, PartialEq, Copy)]
        pub struct Ping: "test::Ping" { pub n: u64 }
    }

    static SEEN: Mutex<Vec<String>> = Mutex::new(Vec::new());

    fn seen(e: impl Into<String>) {
        SEEN.lock().unwrap().push(e.into());
    }

    fn early(_: &mut Cx, mut pings: EventReader<Ping>) {
        let ns: Vec<u64> = pings.read().iter().map(|p| p.n).collect();
        seen(format!("early {ns:?}"));
    }

    fn late(_: &mut Cx, mut pings: EventReader<Ping>) {
        let ns: Vec<u64> = pings.read().iter().map(|p| p.n).collect();
        seen(format!("late {ns:?}"));
    }

    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

    fn ping(_: &mut Cx, out: EventWriter<Ping>) {
        out.send(Ping { n: NEXT.fetch_add(1, std::sync::atomic::Ordering::SeqCst) });
    }

    // One test: the statics are shared.
    #[test]
    fn events_reach_later_readers_this_frame_earlier_ones_next_and_expire() {
        let w = World::new();
        let s = Schedule { systems: vec![early.system(&w, "early"), ping.system(&w, "ping"), late.system(&w, "late")] };
        for _ in 0..3 {
            s.run_sequential(&w);
        }
        assert_eq!(
            *SEEN.lock().unwrap(),
            ["early []", "late [1]", "early [1]", "late [2]", "early [2]", "late [3]"],
            "each reader sees each event once: late the same frame, early the next"
        );

        // Sent between frames, from a message handler: every reader sees it
        // next frame.
        SEEN.lock().unwrap().clear();
        w.between_frames(Build::default()).unwrap().send_event(Ping { n: 99 });
        s.run_sequential(&w);
        assert_eq!(*SEEN.lock().unwrap(), ["early [3, 99]", "late [99, 4]"]);

        // An event lives until the end of the frame after the one it was
        // published for: a reader that starts late sees only the recent ones.
        // 99 was published for frame 4, like ping 4, so both last through 5.
        SEEN.lock().unwrap().clear();
        fn newcomer(_: &mut Cx, mut pings: EventReader<Ping>) {
            let ns: Vec<u64> = pings.read().iter().map(|p| p.n).collect();
            seen(format!("newcomer {ns:?}"));
        }
        let t = Schedule { systems: vec![newcomer.system(&w, "newcomer")] };
        t.run_sequential(&w);
        assert_eq!(*SEEN.lock().unwrap(), ["newcomer [99, 4]"]);
    }
}

/// A world's values are dropped by their build's code, so the keepalive
/// mapping it must go after them: a library unmapped first crashes the drop
/// (as dropping an engine did, when `World` declared its components first).
mod drop_order {
    use std::sync::{Arc, Mutex};

    use super::*;

    static DROPS: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());

    /// Stands in for a build's library: records when it would be unmapped.
    struct Library(&'static str);
    impl Drop for Library {
        fn drop(&mut self) {
            DROPS.lock().unwrap().push(self.0);
        }
    }

    macro_rules! noisy {
        ($name:ident, $id:literal, $storage:expr, $label:literal) => {
            #[derive(Default)]
            struct $name;
            impl Drop for $name {
                fn drop(&mut self) {
                    DROPS.lock().unwrap().push($label);
                }
            }
            // SAFETY: no fields, so no schema to describe.
            unsafe impl engine_ecs::Component for $name {
                const NAME: &'static str = $id;
                const STORAGE: engine_ecs::Storage = $storage;
            }
        };
    }
    noisy!(InTable, "test::InTable", engine_ecs::Storage::Table, "table value");
    noisy!(InSparse, "test::InSparse", engine_ecs::Storage::Sparse, "sparse value");
    noisy!(Queued, "test::Queued", engine_ecs::Storage::Table, "event");
    unsafe impl engine_ecs::Event for Queued {}

    fn build(name: &'static str) -> Build {
        Build { name: name.into(), loaded_at: 1, keepalive: Some(Arc::new(Library(name))) }
    }

    #[test]
    fn values_drop_before_the_code_that_drops_them_is_unmapped() {
        let w = World::new();
        w.between_frames(build("table lib")).unwrap().spawn((InTable,));
        w.between_frames(build("sparse lib")).unwrap().spawn((InSparse,));
        w.between_frames(build("event lib")).unwrap().send_event(Queued);
        DROPS.lock().unwrap().clear();
        drop(w);
        let drops = DROPS.lock().unwrap().clone();
        for (value, library) in [("table value", "table lib"), ("sparse value", "sparse lib"), ("event", "event lib")] {
            let at = |what| drops.iter().position(|&d| d == what).unwrap_or_else(|| panic!("{what} never dropped: {drops:?}"));
            assert!(at(value) < at(library), "{library} went before its {value}: {drops:?}");
        }
    }
}
