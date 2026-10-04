//! Flows (flows.rs) and parallel shapes (shape.rs): edges in the frame's
//! graph, the plan check and its errors, ownership at run time, recycling
//! and the store's bins, and what each shape does on one thread.

use std::any::Any;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use engine_ecs::flows::{PlanStep, check_plan};
use engine_ecs::harness::{Cx, IntoSystem, Schedule, SystemDecl};
use engine_ecs::{
    Build, Colored, Coloring, Executor, Make, ParMap, ParamDecl, Pass, Passes, Reduce, Scoped, See, ShapeKind, Stage, States, Take, World,
    flow,
};

flow! {
    pub struct Numbers: "test::Numbers" {
        pub values: Vec<u64>,
    }
}

flow! {
    pub struct Sums: "test::Sums" {
        pub total: u64,
    }
}

// Another type under `Numbers`' name, as a second mod declaring it apart
// would make.
flow! {
    pub struct Impostor: "test::Numbers" {
        pub values: Vec<u32>,
    }
}

fn make(n: u64) -> impl Fn(&mut Cx, Make<Numbers>) + Send + Sync + 'static {
    move |_: &mut Cx, mut out: Make<Numbers>| out.values.extend(1..=n)
}

type Log = Arc<Mutex<Vec<String>>>;

fn see(log: &Log, name: &'static str) -> impl Fn(&mut Cx, See<Numbers>) + Send + Sync + 'static {
    let log = log.clone();
    move |_: &mut Cx, numbers: See<Numbers>| log.lock().unwrap().push(format!("{name} {}", numbers.values.iter().sum::<u64>()))
}

fn double() -> impl Fn(&mut Cx, Pass<Numbers>) + Send + Sync + 'static {
    |_: &mut Cx, mut numbers: Pass<Numbers>| numbers.values.iter_mut().for_each(|v| *v *= 2)
}

fn take(log: &Log) -> impl Fn(&mut Cx, Take<Numbers>) + Send + Sync + 'static {
    let log = log.clone();
    move |_: &mut Cx, numbers: Take<Numbers>| {
        log.lock().unwrap().push(format!("take {} cap {}", numbers.values.len(), numbers.values.capacity()))
    }
}

fn schedule(systems: Vec<SystemDecl>) -> Schedule {
    Schedule { systems }
}

fn panic_text(run: impl FnOnce()) -> String {
    let err = std::panic::catch_unwind(std::panic::AssertUnwindSafe(run)).unwrap_err();
    match err.downcast::<String>() {
        Ok(s) => *s,
        Err(e) => e.downcast::<&str>().map(|s| s.to_string()).unwrap_or_default(),
    }
}

#[test]
fn a_flow_passes_from_its_maker_through_its_stages_to_its_taker() {
    let w = World::new();
    let log = Log::default();
    let s = schedule(vec![
        make(4).system(&w, "make"),
        see(&log, "a").system(&w, "a"),
        double().system(&w, "double"),
        see(&log, "b").system(&w, "b"),
        take(&log).system(&w, "take"),
    ]);
    assert_eq!(s.check_flows(), Ok(()));
    s.run_sequential(&w);
    assert_eq!(*log.lock().unwrap(), ["a 10", "b 20", "take 4 cap 4"]);
}

#[test]
fn the_graph_orders_a_flows_uses_and_lets_its_readers_run_together() {
    let w = World::new();
    let log = Log::default();
    let s = schedule(vec![
        make(4).system(&w, "make"),
        see(&log, "a").system(&w, "a"),
        see(&log, "b").system(&w, "b"),
        double().system(&w, "double"),
        take(&log).system(&w, "take"),
    ]);
    let fs = s.frame();
    // No apply nodes: a flow's use changes nothing in the world.
    assert_eq!(fs.nodes.len(), 5);
    let index = |name: &str| (0..fs.nodes.len()).find(|&i| s.node_name(fs.nodes[i]) == name).unwrap();
    // A reader waits for the maker; two readers don't wait for each other;
    // an editor waits for both readers.
    assert_eq!(s.blockers(&w, &fs, index("a")), ["make"]);
    assert_eq!(s.blockers(&w, &fs, index("b")), ["make"]);
    assert_eq!(s.blockers(&w, &fs, index("double")), ["make", "a", "b"]);
    assert_eq!(s.blockers(&w, &fs, index("take")), ["make", "a", "b", "double"]);
    // On threads, every frame is the sequential one.
    for _ in 0..50 {
        log.lock().unwrap().clear();
        s.run_parallel(&w, 4);
        let mut got = log.lock().unwrap().clone();
        got[..2].sort();
        assert_eq!(got, ["a 10", "b 10", "take 4 cap 4"]);
    }
}

#[test]
fn two_flows_in_one_system_are_two_edges() {
    let w = World::new();
    let sum = |_: &mut Cx, numbers: See<Numbers>, mut out: Make<Sums>| out.total = numbers.values.iter().sum();
    let got = Arc::new(AtomicU32::new(0));
    let g = got.clone();
    let read = move |_: &mut Cx, s: Take<Sums>| g.store(s.total as u32, Ordering::Relaxed);
    let s = schedule(vec![make(10).system(&w, "make"), sum.system(&w, "sum"), read.system(&w, "read")]);
    assert_eq!(s.check_flows(), Ok(()));
    s.run_sequential(&w);
    assert_eq!(got.load(Ordering::Relaxed), 55);
}

// ---- The plan check ----

fn refused(systems: Vec<SystemDecl>) -> String {
    schedule(systems).check_flows().unwrap_err()
}

#[test]
fn the_plan_check_refuses_a_flow_used_before_its_made_and_names_the_fix() {
    let w = World::new();
    let log = Log::default();
    assert_eq!(
        refused(vec![see(&log, "a").system(&w, "m::a"), make(1).system(&w, "m::make")]),
        "`See<test::Numbers>` in `m::a` runs before `Make<test::Numbers>` in `m::make`; add `.after(\"m::make\")` to `m::a`"
    );
    assert_eq!(refused(vec![double().system(&w, "m::d")]), "`Pass<test::Numbers>` in `m::d`: nothing loaded makes test::Numbers");
}

#[test]
fn the_plan_check_refuses_a_second_maker() {
    let w = World::new();
    assert_eq!(
        refused(vec![make(1).system(&w, "m::make"), make(2).system(&w, "m::again")]),
        "`Make<test::Numbers>` in `m::again` and `Make<test::Numbers>` in `m::make`: a flow has one maker; a stage that changes it \
         takes `Pass<test::Numbers>`"
    );
}

#[test]
fn the_plan_check_refuses_a_use_after_the_take() {
    let w = World::new();
    let log = Log::default();
    assert_eq!(
        refused(vec![make(1).system(&w, "m::make"), take(&log).system(&w, "m::t"), see(&log, "late").system(&w, "m::late")]),
        "`See<test::Numbers>` in `m::late` runs after `Take<test::Numbers>` in `m::t` took it; add `.before(\"m::t\")` to `m::late`"
    );
    assert!(
        refused(vec![make(1).system(&w, "m::make"), take(&log).system(&w, "m::t"), double().system(&w, "m::d")])
            .starts_with("`Pass<test::Numbers>` in `m::d` runs after `Take<test::Numbers>` in `m::t` took it")
    );
    assert_eq!(
        refused(vec![make(1).system(&w, "m::make"), take(&log).system(&w, "m::t1"), take(&log).system(&w, "m::t2")]),
        "`Take<test::Numbers>` in `m::t2` runs after `Take<test::Numbers>` in `m::t1` took it: a flow has one taker; make one of \
         them a `Pass` that runs before the other"
    );
}

#[test]
fn the_plan_check_allows_a_flow_made_again_after_its_taken_or_never_read() {
    let w = World::new();
    let log = Log::default();
    let s = schedule(vec![make(1).system(&w, "make")]);
    assert_eq!(s.check_flows(), Ok(()));
    let s =
        schedule(vec![make(1).system(&w, "make"), take(&log).system(&w, "t"), make(2).system(&w, "again"), see(&log, "a").system(&w, "a")]);
    assert_eq!(s.check_flows(), Ok(()));
    s.run_sequential(&w);
    assert_eq!(*log.lock().unwrap(), ["take 1 cap 4", "a 3"]);
}

fn step<'a>(system: &'a str, phase: &'a str, group: Option<&'a str>, decl: &'a SystemDecl) -> PlanStep<'a> {
    PlanStep { system, phase, group, params: &decl.params }
}

#[test]
fn the_plan_check_names_the_phase_to_move_to_across_phases() {
    let w = World::new();
    let log = Log::default();
    let (early, maker, taker, late) =
        (see(&log, "a").system(&w, "a"), make(1).system(&w, "m"), take(&log).system(&w, "t"), see(&log, "b").system(&w, "b"));
    assert_eq!(
        check_plan(&[step("v::view", "update", None, &early), step("p::make", "late", None, &maker)]).unwrap_err(),
        "`See<test::Numbers>` in `v::view` (phase update) runs before `Make<test::Numbers>` in `p::make` (phase late); move \
         `v::view` to phase late, with `.after(\"p::make\")`"
    );
    assert_eq!(
        check_plan(&[
            step("p::make", "update", None, &maker),
            step("p::sink", "update", None, &taker),
            step("v::view", "late", None, &late)
        ])
        .unwrap_err(),
        "`See<test::Numbers>` in `v::view` (phase late) runs after `Take<test::Numbers>` in `p::sink` (phase update) took it; move \
         `v::view` to phase update, with `.before(\"p::sink\")`"
    );
    // Once-a-frame phases are one group: a flow made in update is seen in
    // late.
    assert_eq!(check_plan(&[step("p::make", "update", None, &maker), step("v::view", "late", None, &late)]), Ok(()));
}

#[test]
fn the_plan_check_refuses_a_flow_across_groups() {
    let w = World::new();
    let log = Log::default();
    let (maker, view) = (make(1).system(&w, "m"), see(&log, "a").system(&w, "a"));
    let sim = Some("simulate at 60 Hz");
    assert_eq!(
        check_plan(&[step("p::make", "simulate", sim, &maker), step("r::draw", "render", None, &view)]).unwrap_err(),
        "`See<test::Numbers>` in `r::draw` (once a frame) and `Make<test::Numbers>` in `p::make` (simulate at 60 Hz): a flow lives \
         one step of its group, so its uses can't cross groups; use it in one group, or carry the value across in a component or an \
         event"
    );
    assert!(
        check_plan(&[step("p::make", "simulate", sim, &maker), step("a::think", "ai", Some("ai at 10 Hz"), &view)])
            .unwrap_err()
            .contains("(ai at 10 Hz) and `Make<test::Numbers>` in `p::make` (simulate at 60 Hz)")
    );
    assert_eq!(check_plan(&[step("p::make", "simulate", sim, &maker), step("p::look", "simulate", sim, &view)]), Ok(()));
}

#[test]
fn the_plan_check_refuses_two_types_under_one_name() {
    let w = World::new();
    let other = |_: &mut Cx, _: See<Impostor>| {};
    let message = refused(vec![make(1).system(&w, "a::make"), other.system(&w, "b::view")]);
    assert!(
        message.starts_with(
            "test::Numbers is declared as two types: `flow_test::Numbers` by `a::make` and `flow_test::Impostor` by `b::view`"
        ),
        "{message}"
    );
    // Unchecked, the first wrong use fails, naming the flow.
    let s = schedule(vec![make(1).system(&w, "make"), other.system(&w, "view")]);
    assert!(
        panic_text(|| {
            s.run_sequential(&w);
        })
        .contains("test::Numbers: declared as two types")
    );
}

#[test]
fn a_system_uses_a_flow_once() {
    let w = World::new();
    let both = |_: &mut Cx, _: See<Numbers>, _: Make<Numbers>| {};
    let message = panic_text(|| {
        both.system(&w, "both");
    });
    assert!(message.contains("both: uses the flow test::Numbers twice"), "{message}");
}

// ---- Run time ----

#[test]
fn a_plan_run_without_its_check_fails_where_a_flow_is_missing() {
    let w = World::new();
    let log = Log::default();
    let s = schedule(vec![see(&log, "early").system(&w, "early"), make(1).system(&w, "make")]);
    let message = panic_text(|| {
        s.run_sequential(&w);
    });
    assert!(message.contains("test::Numbers: seen with none made this frame"), "{message}");
}

#[test]
fn last_frames_flow_is_never_this_frames() {
    let w = World::new();
    let log = Log::default();
    schedule(vec![make(3).system(&w, "make")]).run_sequential(&w);
    // Between frames, it's in the bin, not a value.
    assert_eq!(w.flow_holds("test::Numbers"), (false, true));
    let late = schedule(vec![see(&log, "late").system(&w, "late")]);
    assert!(
        panic_text(|| {
            late.run_sequential(&w);
        })
        .contains("seen with none made this frame")
    );
}

/// The (len, capacity) each `Make` starts from, frame by frame.
fn starts(w: &World, then: Vec<SystemDecl>, frames: usize) -> Vec<(usize, usize)> {
    let seen: Arc<Mutex<Vec<(usize, usize)>>> = Arc::default();
    let at = seen.clone();
    let maker = move |_: &mut Cx, mut out: Make<Numbers>| {
        at.lock().unwrap().push((out.values.len(), out.values.capacity()));
        out.values.extend(0..100);
    };
    let s = schedule(std::iter::once(maker.system(w, "make")).chain(then).collect());
    for _ in 0..frames {
        s.run_sequential(w);
    }
    seen.lock().unwrap().clone()
}

#[test]
fn a_makers_allocations_come_back_from_last_frames_taker() {
    let w = World::new();
    let got = starts(&w, vec![take(&Log::default()).system(&w, "take")], 3);
    assert_eq!(got[0], (0, 0));
    assert!(got[1..].iter().all(|&(len, cap)| len == 0 && cap >= 100), "{got:?}");
}

#[test]
fn a_flow_nothing_takes_is_recycled_by_the_next_maker() {
    let w = World::new();
    let got = starts(&w, vec![see(&Log::default(), "a").system(&w, "a")], 3);
    assert_eq!(got[0], (0, 0));
    assert!(got[1..].iter().all(|&(len, cap)| len == 0 && cap >= 100), "{got:?}");
}

#[test]
fn without_recycling_every_maker_starts_from_nothing() {
    let w = World::new();
    w.set_flow_recycling(false);
    let got = starts(&w, vec![take(&Log::default()).system(&w, "take")], 3);
    assert!(got.iter().all(|&s| s == (0, 0)), "{got:?}");
    assert_eq!(w.flow_holds("test::Numbers"), (false, false));
    // Nor from an earlier step's value, which nothing took.
    let again = |_: &mut Cx, out: Make<Numbers>| assert_eq!(out.values.capacity(), 0);
    schedule(vec![make(5).system(&w, "make"), again.system(&w, "make, step 2")]).run_sequential(&w);
}

#[test]
fn a_later_steps_maker_starts_from_an_earlier_steps_value() {
    // A fixed-rate group's two steps in one frame, as the node list repeats
    // them: the second `Make` finds the first's value, which nothing took.
    let w = World::new();
    let caps: Arc<Mutex<Vec<(usize, usize)>>> = Arc::default();
    let at = caps.clone();
    let maker = move |_: &mut Cx, mut out: Make<Numbers>| {
        at.lock().unwrap().push((out.values.len(), out.values.capacity()));
        out.values.extend(0..10);
    };
    let log = Log::default();
    let s = schedule(vec![
        maker.clone().system(&w, "make"),
        see(&log, "a").system(&w, "a"),
        maker.system(&w, "make, step 2"),
        see(&log, "b").system(&w, "b"),
    ]);
    s.run_sequential(&w);
    let caps = caps.lock().unwrap().clone();
    assert_eq!(caps[0], (0, 0));
    assert!(caps[1].0 == 0 && caps[1].1 >= 10, "{caps:?}");
    assert_eq!(*log.lock().unwrap(), ["a 45", "b 45"]);
}

#[test]
fn take_into_inner_keeps_the_value_and_its_allocation() {
    let w = World::new();
    let kept: Arc<Mutex<Option<Numbers>>> = Arc::default();
    let k = kept.clone();
    let keeper = move |_: &mut Cx, n: Take<Numbers>| *k.lock().unwrap() = Some(n.into_inner());
    let got = starts(&w, vec![keeper.system(&w, "keep")], 2);
    assert_eq!(got, [(0, 0), (0, 0)]);
    assert_eq!(kept.lock().unwrap().as_ref().map(|n| n.values.len()), Some(100));
}

#[test]
fn installing_a_build_drops_the_bin_and_keeps_the_build_mapped() {
    let w = World::new();
    starts(&w, vec![take(&Log::default()).system(&w, "take")], 1);
    assert_eq!(w.flow_holds("test::Numbers"), (false, true));
    let v1: Arc<dyn Any + Send + Sync> = Arc::new(1u8);
    w.install_flow("test::Numbers", &Build { name: "maker".into(), loaded_at: 1, keepalive: Some(v1.clone()) });
    assert_eq!(w.flow_holds("test::Numbers"), (false, false));
    assert_eq!(w.flow_keepalives("test::Numbers"), ["maker"]);
    assert_eq!(Arc::strong_count(&v1), 2);
    // A newer build of the same mod lets go of the older.
    let v2: Arc<dyn Any + Send + Sync> = Arc::new(2u8);
    w.install_flow("test::Numbers", &Build { name: "maker".into(), loaded_at: 2, keepalive: Some(v2.clone()) });
    assert_eq!((Arc::strong_count(&v1), Arc::strong_count(&v2)), (1, 2));
    // The next maker starts from nothing.
    let got = starts(&w, vec![take(&Log::default()).system(&w, "take")], 2);
    assert_eq!(got[0], (0, 0));
    assert!(got[1].1 >= 100);
}

// ---- Shapes ----

#[test]
fn shapes_are_declared_and_touch_nothing() {
    let w = World::new();
    let s = schedule(vec![(|_: &mut Cx, _: ParMap| {}).system(&w, "map"), (|_: &mut Cx, _: Reduce, _: Passes| {}).system(&w, "both")]);
    assert!(matches!(s.systems[0].params[..], [ParamDecl::Shape(ShapeKind::Map)]));
    assert!(matches!(s.systems[1].params[..], [ParamDecl::Shape(ShapeKind::Reduce), ParamDecl::Shape(ShapeKind::Passes)]));
    let fs = s.frame();
    assert_eq!(fs.nodes.len(), 2, "no apply nodes");
    assert!(s.blockers(&w, &fs, 1).is_empty(), "two shapes don't order systems");
}

/// Runs `f` with each shape, as a system would get them.
fn with_shapes(f: impl Fn(&ParMap, &Reduce, &Passes) + Send + Sync + 'static) {
    let w = World::new();
    let s = schedule(vec![(move |_: &mut Cx, m: ParMap, r: Reduce, p: Passes| f(&m, &r, &p)).system(&w, "shapes")]);
    s.run_sequential(&w);
}

#[test]
fn par_map_keeps_the_items_order_and_the_outputs_allocation() {
    with_shapes(|map, _, _| {
        let xs: Vec<u32> = (0..1000).collect();
        let mut out = Vec::with_capacity(4000);
        let before = out.as_ptr();
        map.map_into(&xs, 16, &mut out, |i, x| i as u32 * 2 + x);
        assert_eq!(out, (0..1000).map(|x| x * 3).collect::<Vec<_>>());
        assert_eq!(out.as_ptr(), before);
        map.map_into(&xs[..10], 1, &mut out, |_, x| *x);
        assert_eq!(out, (0..10).collect::<Vec<_>>());
        let mut ys = vec![0usize; 100];
        map.for_each_mut(&mut ys, 8, |i, y| *y = i * i);
        assert!(ys.iter().enumerate().all(|(i, y)| *y == i * i));
    });
}

#[test]
fn a_reduction_folds_fixed_chunks_in_the_items_order() {
    with_shapes(|_, reduce, _| {
        let xs: Vec<u32> = (0..10).collect();
        // A fold that isn't commutative shows the order and the chunks.
        let got = reduce.reduce(&xs, 4, |c| format!("{c:?}"), |a, b| format!("{a}+{b}"));
        assert_eq!(got.as_deref(), Some("[0, 1, 2, 3]+[4, 5, 6, 7]+[8, 9]"));
        assert_eq!(reduce.reduce(&[] as &[u32], 4, |c| c.len(), |a, b| a + b), None);
        // A float sum is the chunks' partial sums added left to right.
        let fs: Vec<f32> = (0..1003).map(|i| (i as f32 * 0.37).sin() * 1e3).collect();
        let sum = reduce.reduce(&fs, 256, |c| c.iter().sum::<f32>(), |a, b| a + b).unwrap();
        let by_hand = fs.chunks(256).map(|c| c.iter().sum::<f32>()).fold(None, |a: Option<f32>, b| Some(a.map_or(b, |a| a + b)));
        assert_eq!(sum.to_bits(), by_hand.unwrap().to_bits());
    });
}

#[test]
fn passes_run_stages_in_order_the_overflow_first_then_each_color() {
    with_shapes(|_, _, passes| {
        // An overflow of 2, an empty color 0, then colors of 3 and 1.
        let layout = Colored { overflow: 2, colors: vec![0, 3, 1] };
        let mut items: Vec<u32> = (0..6).collect();
        let calls = Mutex::new(Vec::new());
        let program = [Stage::All('f'), Stage::Items('a'), Stage::Each('e', 4), Stage::Each('z', 0), Stage::Items('b')];
        let mut none: [f32; 0] = [];
        passes.run(
            &layout,
            &mut items,
            &mut none,
            &program,
            |k, at, block, _| {
                block.iter_mut().for_each(|x| *x += 10);
                calls.lock().unwrap().push(format!("{k}{at}{:?}", block.iter().map(|x| x % 10).collect::<Vec<_>>()));
            },
            |k, r, _| calls.lock().unwrap().push(format!("{k}{r:?}")),
        );
        let want = ["f0[0, 1, 2, 3, 4, 5]", "a0[0, 1]", "a2[2, 3, 4]", "a5[5]", "e0..4", "b0[0, 1]", "b2[2, 3, 4]", "b5[5]"];
        assert_eq!(*calls.lock().unwrap(), want);
        assert_eq!(items, [30, 31, 32, 33, 34, 35]);
    });
}

/// `serial` keeps a program on the system's thread, its states plain,
/// where the world's executor would share it out; without it, the same
/// program runs in blocks across the threads, and a kernel's panic reaches
/// the system.
#[test]
fn passes_run_across_the_worlds_threads_unless_serial() {
    let w = World::new();
    w.set_executor(Some(Arc::new(Scoped(4))));
    let s = schedule(vec![
        (|_: &mut Cx, passes: Passes| {
            let layout = Colored { overflow: 1, colors: vec![400, 3] };
            let mut items: Vec<u32> = (0..404).collect();
            let mut states = vec![0.0f32; 4];
            let program = [Stage::Items(()), Stage::Each((), 4)];
            let mut calls = |passes: Passes| {
                let views = Mutex::new(Vec::new());
                let threads = passes.threads();
                passes.run(
                    &layout,
                    &mut items.clone(),
                    &mut states,
                    &program,
                    |_, _, block, s| views.lock().unwrap().push((block.len(), matches!(s, States::Plain(_)))),
                    |_, _, _| {},
                );
                (threads, views.into_inner().unwrap())
            };
            let (threads, serial) = calls(passes.clone().serial(true));
            assert_eq!(threads, 1);
            assert_eq!(serial, [(1, true), (400, true), (3, true)], "a block a color, plain");
            let (threads, across) = calls(passes.clone());
            assert_eq!(threads, 4);
            assert!(across.len() > 3 && across.iter().all(|(_, plain)| !plain), "blocks of colors, shared: {across:?}");
            let text = panic_text(|| {
                passes.run(
                    &layout,
                    &mut items,
                    &mut states,
                    &program,
                    |_, _, block, _| assert!(!block.contains(&200), "item 200"),
                    |_, _, _| {},
                )
            });
            assert_eq!(text, "item 200");
        })
        .system(&w, "passes"),
    ]);
    s.run_sequential(&w);
}

/// `ParMap::for_each_mut` across the world's threads: every item once, each
/// given its own index (so the results are in the items' order), in blocks
/// the calling thread joins in, on executors whose threads come at once,
/// late, or one after another, as `Passes`' are tested; and a call's panic
/// reaches the system.
#[test]
fn a_map_runs_across_the_worlds_threads_every_item_once() {
    let mut executors: Vec<Option<Arc<dyn Executor>>> = vec![None];
    for n in [1, 2, 3, 4, 8] {
        executors.push(Some(Arc::new(Scoped(n))));
        executors.push(Some(Arc::new(Late(n))));
        executors.push(Some(Arc::new(OneByOne(n))));
    }
    for executor in executors {
        let threads = executor.as_ref().map_or(1, |e| e.threads());
        let w = World::new();
        w.set_executor(executor);
        let s = schedule(vec![
            (move |_: &mut Cx, map: ParMap| {
                assert_eq!(map.threads(), threads);
                let n = 1001;
                let mut items: Vec<u64> = (0..n).map(|i| i * 7).collect();
                let calls: Vec<AtomicU32> = (0..n).map(|_| AtomicU32::new(0)).collect();
                let on = Mutex::new(Vec::new());
                map.for_each_mut(&mut items, 8, |i, x| {
                    calls[i].fetch_add(1, Ordering::Relaxed);
                    *x = *x * 3 + i as u64;
                    on.lock().unwrap().push(std::thread::current().id());
                    // An item takes a while, so threads that come late find
                    // blocks taken, and the calling thread's share shows.
                    let t = std::time::Instant::now();
                    while t.elapsed() < std::time::Duration::from_micros(2) {
                        std::hint::spin_loop();
                    }
                });
                assert!(calls.iter().all(|c| c.load(Ordering::Relaxed) == 1), "{threads} threads: every item once");
                assert!(items.iter().enumerate().all(|(i, x)| *x == i as u64 * 22), "{threads} threads: each item its own index");
                let me = std::thread::current().id();
                assert!(on.into_inner().unwrap().contains(&me), "{threads} threads: the calling thread takes blocks");
                // Fewer items than a block's least are one block, here.
                let mut few = [0u8; 3];
                map.for_each_mut(&mut few, 8, |i, x| {
                    assert_eq!(std::thread::current().id(), me, "one block runs on the calling thread");
                    *x = i as u8 + 1;
                });
                assert_eq!(few, [1, 2, 3]);
                let text = panic_text(|| map.for_each_mut(&mut items, 8, |i, _| assert!(i != 700, "item 700")));
                assert_eq!(text, "item 700");
            })
            .system(&w, "map"),
        ]);
        s.run_sequential(&w);
    }
}

/// `ParMap::map_into` and `Reduce::reduce` across the world's threads, as
/// `for_each_mut` is tested: every item (and chunk) mapped once, results
/// in the items' order and a fold in the chunks', the calling thread among
/// the takers, on executors whose threads come at once, late, or one after
/// another; and a call's panic reaches the system.
#[test]
fn a_map_into_and_a_reduction_run_across_the_worlds_threads() {
    let mut executors: Vec<Option<Arc<dyn Executor>>> = vec![None];
    for n in [1, 2, 3, 4, 8] {
        executors.push(Some(Arc::new(Scoped(n))));
        executors.push(Some(Arc::new(Late(n))));
        executors.push(Some(Arc::new(OneByOne(n))));
    }
    // An item takes a while, so threads that come late find blocks taken,
    // and the calling thread's share shows.
    let work = || {
        let t = std::time::Instant::now();
        while t.elapsed() < std::time::Duration::from_micros(2) {
            std::hint::spin_loop();
        }
    };
    for executor in executors {
        let threads = executor.as_ref().map_or(1, |e| e.threads());
        let w = World::new();
        w.set_executor(executor);
        let s = schedule(vec![
            (move |_: &mut Cx, map: ParMap, reduce: Reduce| {
                assert_eq!((map.threads(), reduce.threads()), (threads, threads));
                let me = std::thread::current().id();
                let n = 1001;
                let xs: Vec<u64> = (0..n).map(|i| i * 7).collect();
                let calls: Vec<AtomicU32> = (0..n).map(|_| AtomicU32::new(0)).collect();
                let on = Mutex::new(Vec::new());
                let mut out = Vec::with_capacity(2000);
                let before = out.as_ptr();
                map.map_into(&xs, 8, &mut out, |i, x| {
                    calls[i].fetch_add(1, Ordering::Relaxed);
                    on.lock().unwrap().push(std::thread::current().id());
                    work();
                    x * 3 + i as u64
                });
                assert!(calls.iter().all(|c| c.load(Ordering::Relaxed) == 1), "{threads} threads: every item once");
                assert!(out.iter().enumerate().all(|(i, x)| *x == i as u64 * 22), "{threads} threads: results in the items' order");
                assert_eq!(out.len(), n as usize);
                assert_eq!(out.as_ptr(), before, "{threads} threads: the output's allocation kept");
                assert!(on.into_inner().unwrap().contains(&me), "{threads} threads: the calling thread takes blocks");
                let text = panic_text(|| map.map_into(&xs, 8, &mut Vec::new(), |i, _| assert!(i != 700, "item 700")));
                assert_eq!(text, "item 700");

                // Chunks of 7, the last of 0..1001 short: each mapped once,
                // folded left to right, which a fold that isn't commutative
                // shows (a list of each chunk's first item).
                let mapped: Vec<AtomicU32> = (0..xs.len().div_ceil(7)).map(|_| AtomicU32::new(0)).collect();
                let on = Mutex::new(Vec::new());
                let got = reduce.reduce(
                    &xs,
                    7,
                    |c| {
                        mapped[(c[0] / 7 / 7) as usize].fetch_add(1, Ordering::Relaxed);
                        on.lock().unwrap().push(std::thread::current().id());
                        work();
                        vec![(c[0], c.len())]
                    },
                    |mut a, b| {
                        a.extend(b);
                        a
                    },
                );
                assert!(mapped.iter().all(|c| c.load(Ordering::Relaxed) == 1), "{threads} threads: every chunk once");
                let want: Vec<(u64, usize)> = xs.chunks(7).map(|c| (c[0], c.len())).collect();
                assert_eq!(got, Some(want), "{threads} threads: the chunks folded in order");
                assert!(on.into_inner().unwrap().contains(&me), "{threads} threads: the calling thread maps chunks");
                assert_eq!(reduce.reduce(&[] as &[u64], 7, |c| c.len(), |a, b| a + b), None);
                let text = panic_text(|| assert!(reduce.reduce(&xs, 7, |c| assert!(c[0] != 700 * 7, "chunk 100"), |_, _| ()).is_some()));
                assert_eq!(text, "chunk 100");
            })
            .system(&w, "map"),
        ]);
        s.run_sequential(&w);
    }
}

/// Threads that start one at a time, each later than the last: a stage's
/// blocks taken by whoever is there.
struct Late(usize);

impl Executor for Late {
    fn threads(&self) -> usize {
        self.0
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        std::thread::scope(|s| {
            for k in 1..tasks {
                s.spawn(move || {
                    std::thread::sleep(std::time::Duration::from_micros(50 * k as u64));
                    f(k)
                });
            }
            f(0);
        });
    }
}

/// Tasks one after another on the calling thread, the last first: one
/// thread does every stage alone, and the others come once it's done.
struct OneByOne(usize);

impl Executor for OneByOne {
    fn threads(&self) -> usize {
        self.0
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        (0..tasks).rev().for_each(f);
    }
}

/// A ring of springs with chords: an edge pulls its two ends' values
/// together, Gauss-Seidel, so the result depends on the order edges run in.
fn graph(n: u32) -> (Vec<(u32, u32)>, Vec<bool>) {
    let mut edges: Vec<(u32, u32)> = (0..n).map(|i| (i, (i + 1) % n)).collect();
    edges.extend((0..n).step_by(3).map(|i| (i, (i * 7 + 5) % n)).filter(|(a, b)| a != b));
    // Every fifth state fixed: edges at it don't count it as shared.
    let moves = (0..n).map(|i| i % 5 != 0).collect();
    (edges, moves)
}

#[test]
fn a_coloring_keeps_moving_states_apart_and_packs_in_edge_order() {
    let (edges, moves) = graph(200);
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, true, &mut Vec::new());
    let mut place = Vec::new();
    let layout = coloring.pack(4, &mut place);
    for (i, &(a, b)) in edges.iter().enumerate() {
        for (j, &(c, d)) in edges.iter().enumerate().skip(i + 1) {
            let shared = [a, b].iter().any(|x| moves[*x as usize] && (*x == c || *x == d));
            assert!(!(shared && coloring.of[i] == coloring.of[j]), "edges {i} and {j} share a moving state in one color");
        }
        if !moves[a as usize] || !moves[b as usize] {
            assert_ne!(coloring.of[i], 0, "an edge at a fixed state isn't in the first color");
        }
    }
    let mut last = vec![None; layout.colors.len()];
    for (i, p) in place.iter().enumerate() {
        let Some((item, lane)) = *p else {
            assert_eq!(coloring.of[i], engine_ecs::shape::UNSOLVED);
            continue;
        };
        let k = coloring.of[i] as usize;
        if let Some(prev) = last[k] {
            assert!((item, lane) > prev);
        }
        last[k] = Some((item, lane));
    }
    assert_eq!(layout.items(), coloring.count.iter().map(|c| c.div_ceil(4)).sum::<usize>());
    // `seat` is `pack` the other way round, every lane it fills an edge's
    // place, every other lane empty.
    let mut seats = Vec::new();
    assert_eq!(coloring.seat(4, &mut seats), layout);
    assert_eq!(seats.len(), layout.items() * 4);
    let mut want = vec![engine_ecs::shape::EMPTY; seats.len()];
    for (i, p) in place.iter().enumerate() {
        if let Some((item, lane)) = *p {
            want[item as usize * 4 + lane as usize] = i as u32;
        }
    }
    assert_eq!(seats, want);
}

/// `seat` with an overflow: past the colors, one edge an item, lane 0.
#[test]
fn edges_past_the_colors_are_seated_one_an_item_first() {
    // A star: every edge at state 0, which moves, so the 65th overflows.
    let edges: Vec<(u32, u32)> = (1..=70).map(|i| (0, i)).collect();
    let moves = vec![true; 71];
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, true, &mut Vec::new());
    assert_eq!(coloring.overflow, 6);
    let (mut place, mut seats) = (Vec::new(), Vec::new());
    let layout = coloring.pack(4, &mut place);
    assert_eq!(coloring.seat(4, &mut seats), layout);
    for (i, p) in place.iter().enumerate() {
        let (item, lane) = p.expect("every edge solved");
        assert_eq!(seats[item as usize * 4 + lane as usize], i as u32);
    }
    let empty = engine_ecs::shape::EMPTY;
    assert_eq!(seats[..24].chunks(4).map(|s| s[1..] == [empty; 3]).filter(|e| *e).count(), 6, "an overflowed edge alone");
}

#[test]
fn colored_passes_relax_the_edges_color_by_color() {
    let (edges, moves) = graph(300);
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, true, &mut Vec::new());
    let mut place = Vec::new();
    let layout = coloring.pack(4, &mut place);
    let mut packed: Vec<Vec<(u32, u32)>> = vec![Vec::new(); layout.items()];
    for (i, p) in place.iter().enumerate() {
        if let Some((item, _)) = p {
            packed[*item as usize].push(edges[i]);
        }
    }
    // The items are filled by the program's first stage, from their seats:
    // an item filled twice relaxes its edges twice, one missed none, and a
    // stage let start before the fill is done finds items empty.
    let mut seats = Vec::new();
    coloring.seat(4, &mut seats);
    let fill = {
        let edges = edges.clone();
        move |at: usize, block: &mut [Vec<(u32, u32)>]| {
            for (j, item) in block.iter_mut().enumerate() {
                let lanes = &seats[(at + j) * 4..(at + j + 1) * 4];
                item.extend(lanes.iter().filter(|e| **e != engine_ecs::shape::EMPTY).map(|e| edges[*e as usize]));
            }
            // A block takes a while, as a real fill's does, so a stage let
            // start before the fill is done overlaps it, and shows: without
            // it the fill was over before any thread reached the next.
            let t = std::time::Instant::now();
            while t.elapsed() < std::time::Duration::from_micros(20) {
                std::hint::spin_loop();
            }
        }
    };
    let relax = move |x: &mut States<'_, f32>, (a, b): (u32, u32), k: f32| {
        let (a, b) = (a as usize, b as usize);
        let d = (x.get(b) - x.get(a)) * k;
        if moves[a] {
            x.set(a, x.get(a) + d);
        }
        if moves[b] {
            x.set(b, x.get(b) - d);
        }
    };
    // The reference: edges one at a time, in color order, edge order within
    // a color, then every state scaled.
    let start: Vec<f32> = (0..300).map(|i| (i as f32 * 0.37).sin()).collect();
    let mut want = start.clone();
    let mut order: Vec<usize> = (0..edges.len()).filter(|&i| coloring.of[i] != engine_ecs::shape::UNSOLVED).collect();
    order.sort_by_key(|&i| coloring.of[i]);
    for &i in &order {
        relax(&mut States::Plain(&mut want), edges[i], 0.5);
    }
    want.iter_mut().for_each(|x| *x *= 0.99);
    // Plain, as on one thread, and shared, as on several; then across
    // threads, which share them, on executors whose threads come at once,
    // late, or one after another.
    let mut runs: Vec<(bool, Option<Arc<dyn Executor>>)> = vec![(false, None), (true, None)];
    for n in [1, 2, 3, 4, 8] {
        runs.push((false, Some(Arc::new(Scoped(n)))));
        runs.push((false, Some(Arc::new(Late(n)))));
        runs.push((false, Some(Arc::new(OneByOne(n)))));
    }
    for (shared, executor) in runs {
        let threads = executor.as_ref().map_or(1, |e| e.threads());
        let shared = shared || threads > 1;
        let (layout, want, relax, fill, packed) = (layout.clone(), want.clone(), relax.clone(), fill.clone(), packed.clone());
        let items = Mutex::new(vec![Vec::new(); layout.items()]);
        let states = Mutex::new(start.clone());
        let views = Mutex::new(Vec::new());
        let w = World::new();
        w.set_shapes_shared(shared && threads == 1);
        w.set_executor(executor);
        let s = schedule(vec![
            (move |_: &mut Cx, passes: Passes| {
                let seen = |s: &States<'_, f32>| views.lock().unwrap().push(matches!(s, States::Shared(_)));
                passes.run(
                    &layout,
                    &mut items.lock().unwrap(),
                    &mut states.lock().unwrap(),
                    &[Stage::All(None), Stage::Items(Some(0.5f32)), Stage::Each(Some(0.99f32), 300)],
                    |k, at, block, mut s| {
                        seen(&s);
                        match k {
                            None => fill(at, block),
                            Some(k) => block.iter().flatten().for_each(|&e| relax(&mut s, e, k)),
                        }
                    },
                    |k, r, mut s| {
                        seen(&s);
                        let k = k.expect("a stage over states scales");
                        r.for_each(|i| s.set(i, s.get(i) * k))
                    },
                );
                assert!(*items.lock().unwrap() == packed, "{threads} threads: every item filled once");
                let got = states.lock().unwrap();
                assert!(
                    got.iter().zip(&want).all(|(a, b)| a.to_bits() == b.to_bits()),
                    "shared {shared}, {threads} threads: the colors' order"
                );
                assert!(views.lock().unwrap().iter().all(|v| *v == shared), "shared {shared}: every kernel saw the states so");
                assert!(threads == 1 || views.lock().unwrap().len() > layout.colors.len() + 1, "{threads} threads: blocks, not colors");
            })
            .system(&w, "relax"),
        ]);
        s.run_sequential(&w);
    }
}
