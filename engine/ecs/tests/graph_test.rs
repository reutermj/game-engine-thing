//! The scheduler and the system API, on the walkthrough's scenario (see
//! game.rs) and small cases of their own.
//!
//! `readiness` drives `Schedule::blockers` by hand, node by node, so whether
//! two nodes may run at once is asserted without depending on thread
//! timing. `rows` covers structural changes through query rows.
//! `equivalence` runs whole frames on threads and compares the world with
//! the sequential run's.

use engine_ecs::World;
use engine_ecs::harness::{FrameState, Schedule, State};
use ecs_game::{Burning, Options, SparseBurning, TableBurning};

fn setup<B: Burning>(anywhere: bool) -> (World, Schedule) {
    let w = ecs_game::world::<B>();
    ecs_game::populate(&w, 2000, 200);
    let s = ecs_game::schedule::<B>(&w, Options { anywhere });
    (w, s)
}

fn index(s: &Schedule, fs: &FrameState, name: &str) -> usize {
    fs.nodes.iter().position(|&n| s.node_name(n) == name).unwrap_or_else(|| panic!("no node {name}"))
}

/// Runs node `name` for real, as the executor would, and marks it done.
fn step(w: &World, s: &Schedule, fs: &mut FrameState, name: &str) {
    let i = index(s, fs, name);
    s.step(w, fs, i);
}

/// What node `name` waits for, now.
fn blockers(w: &World, s: &Schedule, fs: &FrameState, name: &str) -> Vec<String> {
    s.blockers(w, fs, index(s, fs, name))
}

fn running(s: &Schedule, fs: &mut FrameState, name: &str) {
    let i = index(s, fs, name);
    s.mark(fs, i, State::Running);
}

fn has(list: &[String], name: &str) -> bool {
    list.iter().any(|x| x == name)
}

mod readiness {
    use super::*;

    #[test]
    fn at_the_start_what_overlaps_nothing_earlier_is_ready() {
        let (w, s) = setup::<SparseBurning>(false);
        let fs = s.frame();
        assert!(blockers(&w, &s, &fs, "ignite").is_empty());
        // physics writes the Position ignite reads, in the same tables.
        assert!(has(&blockers(&w, &s, &fs, "physics"), "ignite"));
        // ui's labels are in tables nothing before it touches.
        assert_eq!(blockers(&w, &s, &fs, "ui"), Vec::<String>::new());
    }

    #[test]
    fn a_sparse_insert_applies_while_physics_runs() {
        let (w, s) = setup::<SparseBurning>(false);
        let mut fs = s.frame();
        step(&w, &s, &mut fs, "ignite");
        assert!(blockers(&w, &s, &fs, "apply(ignite)").is_empty());
        running(&s, &mut fs, "apply(ignite)");
        // The apply touches only Burning's sparse set: physics needn't wait
        // for it (it waits for the walkers' spawns and deaths, earlier).
        assert!(!has(&blockers(&w, &s, &fs, "physics"), "apply(ignite)"));
        // burn reads Burning, and comes after ignite: it must.
        assert_eq!(blockers(&w, &s, &fs, "burn"), ["apply(ignite)"]);
    }

    #[test]
    fn a_table_insert_makes_later_systems_on_those_tables_wait() {
        let (w, s) = setup::<TableBurning>(false);
        let mut fs = s.frame();
        step(&w, &s, &mut fs, "ignite");
        running(&s, &mut fs, "apply(ignite)");
        // Walkers move between tables, which physics walks.
        assert!(has(&blockers(&w, &s, &fs, "physics"), "apply(ignite)"));
        assert!(!has(&blockers(&w, &s, &fs, "ui"), "apply(ignite)"));
    }

    #[test]
    fn a_query_bounds_its_rows_changes_before_the_system_runs() {
        let (w, s) = setup::<TableBurning>(false);
        let fs = s.frame();
        // ignite adds Burning only through walkers' rows, so labels' tables
        // are out of reach from the start.
        assert_eq!(blockers(&w, &s, &fs, "ui"), Vec::<String>::new());
    }

    #[test]
    fn a_query_matching_everything_holds_up_later_table_systems_until_it_runs() {
        let (w, s) = setup::<TableBurning>(true);
        let fs = s.frame();
        assert!(has(&blockers(&w, &s, &fs, "ui"), "apply(ignite)"));
    }

    #[test]
    fn a_despawn_waits_for_a_system_that_only_iterates_a_sparse_set() {
        use engine_ecs::harness::{Cx, IntoSystem};
        use engine_ecs::{Despawns, Query};
        let w = ecs_game::world::<SparseBurning>();
        ecs_game::populate(&w, 10, 0);
        // watch touches no table, only Burning's set, and sees who's alive;
        // kill, after it, despawns. Only liveness links them.
        fn watch(_: &mut Cx, mut fires: Query<&SparseBurning>) {
            fires.for_each(|_, _| {});
        }
        fn kill(_: &mut Cx, mut living: Query<&ecs_game::Health, (), Despawns>) {
            living.for_each(|row, _| row.despawn());
        }
        let s = Schedule { systems: vec![watch.system(&w, "watch"), kill.system(&w, "kill")] };
        let mut fs = s.frame();
        step(&w, &s, &mut fs, "kill");
        assert_eq!(blockers(&w, &s, &fs, "apply(kill)"), ["watch"]);
    }

    #[test]
    fn a_despawn_waits_for_earlier_readers_of_the_tables() {
        let (w, s) = setup::<SparseBurning>(false);
        // Someone for reap to reap.
        let health = w.id("game::Health").unwrap();
        let victim = w.tables().find(|t| t.components.contains(&health)).unwrap().rows.read().unwrap()[0][0];
        w.between_frames(Default::default()).unwrap().insert(victim, ecs_game::Health { hp: 0.0 });

        let mut fs = s.frame();
        step(&w, &s, &mut fs, "ignite");
        step(&w, &s, &mut fs, "apply(ignite)");
        step(&w, &s, &mut fs, "reap");
        assert_eq!(blockers(&w, &s, &fs, "apply(reap)"), ["burn"]);
    }
}

mod rows {
    use engine_ecs::harness::{Cx, IntoSystem, SystemDecl};
    use engine_ecs::{Adds, Entity, Query, Removes, With, Without, component};

    use super::*;

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Frozen: "Frozen" { n: u32 }
    }

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Wet: "Wet" { n: u32 }
    }

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Mark: "Mark" {}
    }

    fn world() -> (World, Vec<Entity>) {
        let w = World::new();
        let es = {
            let mut m = w.between_frames(Default::default()).unwrap();
            m.id::<Wet>();
            m.id::<Mark>();
            (0..4).map(|n| m.spawn((Frozen { n },))).collect()
        };
        (w, es)
    }

    fn shape(w: &World, e: Entity) -> Vec<String> {
        let at = w.entities.location(e).expect("alive");
        let mut names: Vec<String> = w.table(at.table).components.iter().map(|&c| w.name(c).to_string()).collect();
        // By name: ids, and so table order, depend on interning order.
        names.sort();
        names
    }

    fn run(w: &World, systems: Vec<SystemDecl>) {
        Schedule { systems }.run_sequential(w);
    }

    #[test]
    fn a_row_removes_and_adds_in_the_order_it_asked() {
        let (w, es) = world();
        fn thaw(_: &mut Cx, mut ice: Query<&Frozen, (), (Removes<(Frozen, Mark)>, Adds<(Wet, Mark)>)>) {
            ice.for_each(|row, f| match f.n {
                // Thaws: loses Frozen, gains Wet.
                0 => {
                    row.remove::<Frozen>();
                    row.insert(Wet { n: 0 });
                }
                // Gains Mark and loses it again: the last change wins.
                1 => {
                    row.insert(Mark {});
                    row.remove::<Mark>();
                    row.insert(Wet { n: 1 });
                }
                // Only adds.
                2 => row.insert(Mark {}),
                _ => {}
            });
        }
        run(&w, vec![thaw.system(&w, "thaw")]);
        assert_eq!(shape(&w, es[0]), ["Wet"]);
        assert_eq!(shape(&w, es[1]), ["Frozen", "Wet"]);
        assert_eq!(shape(&w, es[2]), ["Frozen", "Mark"]);
        assert_eq!(shape(&w, es[3]), ["Frozen"]);
    }

    #[test]
    #[should_panic(expected = "Wet isn't in this query's Adds")]
    fn a_change_the_query_did_not_declare_is_refused() {
        let (w, _) = world();
        fn sneaky(_: &mut Cx, mut ice: Query<&Frozen, (), Adds<Mark>>) {
            ice.for_each(|row, _| row.insert(Wet { n: 9 }));
        }
        run(&w, vec![sneaky.system(&w, "sneaky")]);
    }

    #[test]
    fn a_system_does_not_see_its_own_changes() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let (w, _) = world();
        static SEEN: AtomicUsize = AtomicUsize::new(usize::MAX);
        fn mark(_: &mut Cx, mut ice: Query<&Frozen, Without<Mark>, Adds<Mark>>, mut marked: Query<&Mark>) {
            ice.for_each(|row, _| row.insert(Mark {}));
            let mut n = 0;
            marked.for_each(|_, _| n += 1);
            SEEN.store(n, Ordering::SeqCst);
        }
        run(&w, vec![mark.system(&w, "mark")]);
        assert_eq!(SEEN.load(Ordering::SeqCst), 0, "the marks land after the system");
        let mark = w.id("Mark").unwrap();
        let marked: usize = w
            .tables()
            .filter(|t| t.components.contains(&mark))
            .map(|t| t.rows.read().unwrap().iter().map(Vec::len).sum::<usize>())
            .sum();
        assert_eq!(marked, 4);
    }

    #[test]
    fn a_query_reaches_entities_it_did_not_iterate_to() {
        let (w, es) = world();
        static TARGET: std::sync::Mutex<Option<Entity>> = std::sync::Mutex::new(None);
        *TARGET.lock().unwrap() = Some(es[3]);
        fn tag(_: &mut Cx, mut ice: Query<(), (), Adds<Mark>>) {
            let e = TARGET.lock().unwrap().unwrap();
            ice.get(e).expect("it matches").insert(Mark {});
        }
        run(&w, vec![tag.system(&w, "tag")]);
        assert_eq!(shape(&w, es[3]), ["Frozen", "Mark"]);
        assert_eq!(shape(&w, es[2]), ["Frozen"]);
    }

    #[test]
    fn queries_kept_apart_by_their_filters_may_share_a_written_component() {
        let (w, es) = world();
        w.between_frames(Default::default()).unwrap().insert(es[0], Wet { n: 7 });
        // The wet one takes the dry ones' numbers: one query writes Frozen,
        // the other reads it, on tables no entity can be in both of.
        fn copy(_: &mut Cx, mut wet: Query<&mut Frozen, With<Wet>>, mut dry: Query<&Frozen, Without<Wet>>) {
            let mut sum = 0;
            dry.for_each(|_, f| sum += f.n);
            wet.for_each(|_, f| f.n = sum);
        }
        run(&w, vec![copy.system(&w, "copy")]);
        let values = w.values::<Frozen>().unwrap();
        assert_eq!(values.iter().find(|(e, _)| *e == es[0]).unwrap().1, Frozen { n: 1 + 2 + 3 });
    }

    #[test]
    #[should_panic(expected = "two queries access Frozen and one writes it")]
    fn filters_that_could_both_match_an_entity_keep_them_apart_from_nothing() {
        let (w, _) = world();
        fn both(_: &mut Cx, _a: Query<&mut Frozen, With<Wet>>, _b: Query<&Frozen, With<Mark>>) {}
        both.system(&w, "both");
    }

    #[test]
    #[should_panic(expected = "two queries access Sparse and one writes it")]
    fn filters_dont_keep_apart_two_queries_of_a_sparse_component() {
        component! {
            #[derive(Debug, Default, PartialEq)]
            struct Sparse: "Sparse", storage = sparse { n: u32 }
        }
        let (w, _) = world();
        fn both(_: &mut Cx, _a: Query<&mut Sparse, With<Wet>>, _b: Query<&Sparse, Without<Wet>>) {}
        both.system(&w, "both");
    }

    #[test]
    #[should_panic(expected = "two queries access Frozen and one writes it")]
    fn a_system_whose_queries_conflict_is_refused() {
        let (w, _) = world();
        fn both(_: &mut Cx, _a: Query<&mut Frozen>, _b: Query<&Frozen>) {}
        both.system(&w, "both");
    }
}

/// Parameters made of parameters: a tuple of them, which is how a crate
/// builds its own (physics's `Spatial`). The rules see the members.
mod groups {
    use engine_ecs::harness::{Cx, IntoSystem};
    use engine_ecs::{Adds, Query, Without, component};

    use super::*;

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Frozen: "Frozen" { n: u32 }
    }

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Mark: "Mark" {}
    }

    fn world() -> World {
        let w = World::new();
        let mut m = w.between_frames(Default::default()).unwrap();
        m.id::<Mark>();
        for n in 0..4 {
            m.spawn((Frozen { n },));
        }
        w
    }

    fn marked(w: &World) -> usize {
        let mark = w.id("Mark").unwrap();
        w.tables()
            .filter(|t| t.components.contains(&mark))
            .map(|t| t.rows.read().unwrap().iter().map(Vec::len).sum::<usize>())
            .sum()
    }

    #[test]
    fn a_group_works_like_its_members_and_its_changes_are_applied() {
        let w = world();
        fn mark(_: &mut Cx, (mut ice, mut seen): (Query<&Frozen, Without<Mark>, Adds<Mark>>, Query<&Frozen>)) {
            ice.for_each(|row, _| row.insert(Mark {}));
            let mut n = 0;
            seen.for_each(|_, _| n += 1);
            assert_eq!(n, 4);
        }
        Schedule { systems: vec![mark.system(&w, "mark")] }.run_sequential(&w);
        assert_eq!(marked(&w), 4, "the group's changing query got an apply node");
    }

    #[test]
    #[should_panic(expected = "two queries access Frozen and one writes it")]
    fn a_group_cannot_hide_a_conflict_with_another_parameter() {
        let w = world();
        fn both(_: &mut Cx, _g: (Query<&mut Frozen>,), _b: Query<&Frozen>) {}
        both.system(&w, "both");
    }

    #[test]
    fn a_groups_write_orders_it_before_a_later_reader() {
        let w = world();
        fn write(_: &mut Cx, _g: (Query<&Mark>, Query<&mut Frozen>)) {}
        fn read(_: &mut Cx, _q: Query<&Frozen>) {}
        let s = Schedule { systems: vec![write.system(&w, "write"), read.system(&w, "read")] };
        let fs = s.frame();
        assert_eq!(blockers(&w, &s, &fs, "read"), ["write"]);
    }
}

mod equivalence {
    use super::*;

    fn frames<B: Burning>(anywhere: bool, threads: Option<usize>, n: usize) -> Vec<String> {
        let (w, s) = setup::<B>(anywhere);
        for _ in 0..n {
            match threads {
                None => {
                    s.run_sequential(&w);
                }
                Some(t) => {
                    s.run_parallel(&w, t);
                }
            }
        }
        ecs_game::snapshot::<B>(&w)
    }

    /// Enough busy work that nodes overlap in time; without it, each
    /// finishes before the next starts and a parallel bug can't show.
    fn same_world<B: Burning>(anywhere: bool) {
        ecs_game::set_work(30);
        let reference = frames::<B>(anywhere, None, 30);
        for run in 0..5 {
            assert_eq!(frames::<B>(anywhere, Some(8), 30), reference, "run {run}");
        }
    }

    #[test]
    fn sparse_burning_in_parallel_matches_sequential() {
        same_world::<SparseBurning>(false);
    }

    #[test]
    fn table_burning_in_parallel_matches_sequential() {
        same_world::<TableBurning>(false);
    }

    #[test]
    fn table_burning_added_anywhere_in_parallel_matches_sequential() {
        same_world::<TableBurning>(true);
    }

    #[test]
    fn the_scenario_changes_the_world() {
        // Guards the others: a frame that did nothing would match trivially.
        let (w, s) = setup::<SparseBurning>(false);
        let before = ecs_game::snapshot::<SparseBurning>(&w);
        for _ in 0..30 {
            s.run_sequential(&w);
        }
        let after = ecs_game::snapshot::<SparseBurning>(&w);
        assert_ne!(before, after);
        assert!(after.iter().any(|l| l.contains("SparseBurning")), "something is burning");
        // Walkers burn for three frames, catch fire again, and die at 0.
        assert!(after.len() < before.len() + 60, "some walkers died");
        assert!(after.iter().any(|l| l.contains("Position { x: 1.5,")), "walkers spawned");
    }
}
