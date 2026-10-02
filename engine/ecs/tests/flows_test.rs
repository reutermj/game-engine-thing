//! The flows spike's mechanism (tests/flows.rs): edges ordered by the
//! frame's graph, the plan check, ownership at run time, recycling, and the
//! parallel shapes' results at any thread count.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use engine_ecs::harness::{Cx, IntoSystem, Schedule, SystemDecl};
use engine_ecs::{Scoped, Workers, World};
use flows::{Access, Colored, Coloring, Make, Pass, See, Stage, Take, accesses, check, flow};

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

fn world() -> World {
    let w = World::new();
    flows::reset(&w);
    w
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

#[test]
fn a_flow_passes_from_its_maker_through_its_stages_to_its_taker() {
    let w = world();
    let log = Log::default();
    let s = schedule(vec![
        make(4).system(&w, "make"),
        see(&log, "a").system(&w, "a"),
        double().system(&w, "double"),
        see(&log, "b").system(&w, "b"),
        take(&log).system(&w, "take"),
    ]);
    assert_eq!(check(&w, &s), Ok(vec![]));
    s.run_sequential(&w);
    assert_eq!(*log.lock().unwrap(), ["a 10", "b 20", "take 4 cap 4"]);
}

#[test]
fn the_graph_orders_a_flows_uses_and_lets_its_readers_run_together() {
    let w = world();
    let log = Log::default();
    let s = schedule(vec![
        make(4).system(&w, "make"),
        see(&log, "a").system(&w, "a"),
        see(&log, "b").system(&w, "b"),
        double().system(&w, "double"),
        take(&log).system(&w, "take"),
    ]);
    let fs = s.frame();
    let index = |name: &str| (0..fs.nodes.len()).find(|&i| s.node_name(fs.nodes[i]) == name).unwrap();
    // A reader waits for the maker and its (empty) apply node; two readers
    // don't wait for each other; an editor waits for both readers.
    assert_eq!(s.blockers(&w, &fs, index("a")), ["make", "apply(make)"]);
    assert_eq!(s.blockers(&w, &fs, index("b")), ["make", "apply(make)"]);
    assert_eq!(s.blockers(&w, &fs, index("double")), ["make", "apply(make)", "a", "b"]);
    assert!(s.blockers(&w, &fs, index("take")).contains(&"double".to_string()));
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
fn the_plan_check_refuses_a_flow_used_out_of_turn() {
    let w = world();
    let log = Log::default();
    let refused = |systems: Vec<SystemDecl>| check(&w, &schedule(systems)).unwrap_err();
    assert!(refused(vec![see(&log, "a").system(&w, "a"), make(1).system(&w, "make")]).contains("See by a before anything makes it"));
    assert!(refused(vec![make(1).system(&w, "make"), make(2).system(&w, "again")]).contains("made by make and again by again"));
    assert!(
        refused(vec![make(1).system(&w, "make"), take(&log).system(&w, "t1"), take(&log).system(&w, "t2")])
            .contains("Take by t2 after t1 took it")
    );
    assert!(
        refused(vec![make(1).system(&w, "make"), take(&log).system(&w, "t"), see(&log, "late").system(&w, "late")])
            .contains("See by late after t took it")
    );
    assert!(
        refused(vec![make(1).system(&w, "make"), take(&log).system(&w, "t"), double().system(&w, "d")])
            .contains("Pass by d after t took it")
    );
    // Made and never read is allowed, and noted: the next frame's maker
    // recycles it.
    let s = schedule(vec![make(1).system(&w, "make")]);
    assert_eq!(check(&w, &s), Ok(vec!["test::Numbers: made by make, read by nothing".to_string()]));
    // Made again after it's taken is a new value, which is fine.
    let s =
        schedule(vec![make(1).system(&w, "make"), take(&log).system(&w, "t"), make(2).system(&w, "again"), see(&log, "a").system(&w, "a")]);
    assert_eq!(check(&w, &s), Ok(vec![]));
}

#[test]
fn a_system_that_reads_a_flow_cannot_also_make_it() {
    let w = world();
    let both = |_: &mut Cx, _: See<Numbers>, _: Make<Numbers>| {};
    let refused = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        both.system(&w, "both");
    }));
    let message = *refused.unwrap_err().downcast::<String>().unwrap();
    assert!(message.contains("reads and writes one event type"), "{message}");
}

#[test]
fn a_plan_run_without_its_check_fails_where_a_flow_is_missing() {
    let w = world();
    let log = Log::default();
    let s = schedule(vec![see(&log, "early").system(&w, "early"), make(1).system(&w, "make")]);
    let failed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| s.run_sequential(&w)));
    let message = *failed.unwrap_err().downcast::<String>().unwrap();
    assert!(message.contains("seen with none made this frame"), "{message}");
}

#[test]
fn last_frames_flow_is_never_this_frames() {
    let w = world();
    let log = Log::default();
    // Made in one frame, seen alone in the next: refused at fetch.
    schedule(vec![make(3).system(&w, "make")]).run_sequential(&w);
    let late = schedule(vec![see(&log, "late").system(&w, "late")]);
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| late.run_sequential(&w))).is_err());
}

#[test]
fn a_makers_allocations_come_back_from_last_frames_taker() {
    for recycling in [true, false] {
        let w = world();
        flows::set_recycling(&w, recycling);
        let seen: Arc<Mutex<Vec<(usize, usize)>>> = Arc::default();
        let at = seen.clone();
        let maker = move |_: &mut Cx, mut out: Make<Numbers>| {
            // What it starts from: last frame's allocation, emptied.
            at.lock().unwrap().push((out.values.len(), out.values.capacity()));
            out.values.extend(0..100);
        };
        let s = schedule(vec![maker.system(&w, "make"), take(&Log::default()).system(&w, "take")]);
        for _ in 0..3 {
            s.run_sequential(&w);
        }
        let seen = seen.lock().unwrap().clone();
        if recycling {
            assert_eq!(seen[0], (0, 0));
            assert!(seen[1..].iter().all(|&(len, cap)| len == 0 && cap >= 100), "{seen:?}");
        } else {
            assert!(seen.iter().all(|&s| s == (0, 0)), "{seen:?}");
        }
    }
}

#[test]
fn a_flow_nothing_takes_is_recycled_by_the_next_maker() {
    let w = world();
    let caps: Arc<Mutex<Vec<usize>>> = Arc::default();
    let at = caps.clone();
    let maker = move |_: &mut Cx, mut out: Make<Numbers>| {
        at.lock().unwrap().push(out.values.capacity());
        assert!(out.values.is_empty(), "emptied");
        out.values.extend(0..50);
    };
    let s = schedule(vec![maker.system(&w, "make"), see(&Log::default(), "a").system(&w, "a")]);
    for _ in 0..2 {
        s.run_sequential(&w);
    }
    assert!(caps.lock().unwrap()[1] >= 50);
}

#[test]
fn two_flows_in_one_system_are_two_edges() {
    let w = world();
    let sum = |_: &mut Cx, numbers: See<Numbers>, mut out: Make<Sums>| out.total = numbers.values.iter().sum();
    let got = Arc::new(AtomicU32::new(0));
    let g = got.clone();
    let read = move |_: &mut Cx, s: Take<Sums>| g.store(s.total as u32, Ordering::Relaxed);
    let s = schedule(vec![make(10).system(&w, "make"), sum.system(&w, "sum"), read.system(&w, "read")]);
    let kinds: Vec<Vec<Access>> = s.systems.iter().map(|d| accesses(&d.params).into_iter().map(|(_, a)| a).collect()).collect();
    assert_eq!(kinds, [vec![Access::Make], vec![Access::See, Access::Make], vec![Access::Take]]);
    // Numbers seen and Sums taken: nothing left unread.
    assert_eq!(check(&w, &s), Ok(vec![]));
    s.run_sequential(&w);
    assert_eq!(got.load(Ordering::Relaxed), 55);
}

// ---- Parallel shapes ----

fn on(threads: usize) -> Workers {
    Workers::new(Some(Arc::new(Scoped(threads))))
}

/// A ring of springs with chords: an edge pulls its two ends' values
/// together, Gauss-Seidel, the result depending on the order edges run in.
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
    // Packed: within a color, in edge order, `width` to an item.
    let mut last = vec![None; layout.colors.len()];
    for (i, p) in place.iter().enumerate() {
        // Unsolved: both ends fixed.
        let Some((item, lane)) = *p else {
            assert_eq!(coloring.of[i], flows::UNSOLVED);
            continue;
        };
        let k = coloring.of[i] as usize;
        if let Some((li, ll)) = last[k] {
            assert!((item, lane) > (li, ll));
        }
        last[k] = Some((item, lane));
    }
    assert_eq!(layout.items(), coloring.count.iter().map(|c| c.div_ceil(4)).sum::<usize>());
}

/// The springs relaxed `passes` times through `Colored::passes`, each item
/// `width` edges, states as atomic bits: the values, as bits.
fn relax(threads: usize, width: usize) -> Vec<u32> {
    let (edges, moves) = graph(300);
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, true, &mut Vec::new());
    let mut place = Vec::new();
    let layout = coloring.pack(width, &mut place);
    let mut items: Vec<Vec<(u32, u32)>> = vec![Vec::new(); layout.items()];
    for (i, p) in place.iter().enumerate() {
        if let Some((item, _)) = p {
            items[*item as usize].push(edges[i]);
        }
    }
    let states: Vec<AtomicU32> = (0..300).map(|i| AtomicU32::new((i as f32 * 0.37).sin().to_bits())).collect();
    let get = |s: &[AtomicU32], i: u32| f32::from_bits(s[i as usize].load(Ordering::Relaxed));
    let put = |s: &[AtomicU32], i: u32, x: f32| s[i as usize].store(x.to_bits(), Ordering::Relaxed);
    let program = [Stage::Items(0.5f32), Stage::Each(0.99, 300), Stage::Items(0.25)];
    layout.passes(
        &on(threads),
        &mut items,
        &states[..],
        &program,
        |k, item, s| {
            for &(a, b) in item.iter() {
                let (x, y) = (get(s, a), get(s, b));
                let d = (y - x) * k;
                if moves[a as usize] {
                    put(s, a, x + d);
                }
                if moves[b as usize] {
                    put(s, b, y - d);
                }
            }
        },
        |k, r, s| r.for_each(|i| put(s, i as u32, get(s, i as u32) * k)),
    );
    states.iter().map(|s| s.load(Ordering::Relaxed)).collect()
}

#[test]
fn colored_passes_are_the_same_at_any_thread_count_and_width() {
    let one = relax(1, 1);
    for threads in [1, 2, 3, 8] {
        for width in [1, 4] {
            assert!(relax(threads, width) == one, "{threads} threads, {width} wide");
        }
    }
}

#[test]
fn a_kernels_panic_reaches_the_caller_and_ends_the_waits() {
    let (edges, moves) = graph(100);
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, false, &mut Vec::new());
    let layout = coloring.pack(1, &mut Vec::new());
    let mut items = vec![0u32; layout.items()];
    for (i, x) in items.iter_mut().enumerate() {
        *x = i as u32;
    }
    let stop = items.len() as u32 / 2;
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        layout.passes(
            &on(4),
            &mut items,
            &(),
            &[Stage::Items(()), Stage::Items(())],
            |_, x, _| assert!(*x != stop, "item {x}"),
            |_, _, _| {},
        )
    }));
    assert!(run.is_err());
}

#[test]
fn a_reduction_is_the_same_at_any_thread_count() {
    let xs: Vec<f32> = (0..100_003).map(|i| (i as f32 * 0.001).sin() * 1e3).collect();
    let sum = |w: &Workers| flows::reduce(w, &xs, 256, |c| c.iter().sum::<f32>(), |a, b| a + b).unwrap();
    let one = sum(&Workers::default());
    for t in [2, 3, 8] {
        assert_eq!(sum(&on(t)).to_bits(), one.to_bits(), "{t} threads");
    }
}

#[test]
fn par_map_and_par_for_each_mut_keep_the_items_order() {
    for t in [1, 3, 8] {
        let w = on(t);
        let xs: Vec<u32> = (0..10_000).collect();
        assert_eq!(flows::par_map(&w, &xs, 16, |i, x| (i as u32) * 2 + x), (0..10_000).map(|x| x * 3).collect::<Vec<_>>());
        let mut ys = vec![0usize; 1000];
        flows::par_for_each_mut(&w, &mut ys, 8, |i, y| *y = i * i);
        assert!(ys.iter().enumerate().all(|(i, y)| *y == i * i));
    }
}

#[test]
fn colored_edges_hand_each_edge_its_two_states() {
    // A path, so the first state is only ever an `a` and the last a `b`.
    let edges = [(0u32, 1u32), (1, 2), (2, 3)];
    let moves = vec![true; 4];
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, false, &mut Vec::new());
    let mut place = Vec::new();
    let layout: Colored = coloring.pack(1, &mut place);
    let mut items = vec![(0u32, 0u32); layout.items()];
    for (i, p) in place.iter().enumerate() {
        items[p.unwrap().0 as usize] = edges[i];
    }
    let states: Vec<AtomicU32> = (0..4).map(|_| AtomicU32::new(0)).collect();
    layout.edges(
        &on(2),
        &mut items,
        &states,
        &[Stage::Items(())],
        |e| *e,
        |_, _, a, b| {
            a.fetch_add(10, Ordering::Relaxed);
            b.fetch_add(100, Ordering::Relaxed);
        },
        |_, _, _| {},
    );
    let got: Vec<u32> = states.iter().map(|s| s.load(Ordering::Relaxed)).collect();
    assert_eq!(got, [10, 110, 110, 100]);
}

#[test]
fn a_color_starts_only_once_the_one_before_is_done() {
    // A star's spokes and a ring around it: the spokes all share the hub,
    // so each is its own color, and the ring's edges fill the first colors.
    let n = 64u32;
    let mut edges: Vec<(u32, u32)> = (1..=n).map(|i| (0, i)).collect();
    edges.extend((1..=n).map(|i| (i, i % n + 1)));
    let moves = vec![true; n as usize + 1];
    let mut coloring = Coloring::default();
    coloring.greedy(edges.len(), |i| edges[i], &moves, false, &mut Vec::new());
    let mut place = Vec::new();
    let layout = coloring.pack(1, &mut place);
    // Each item is its color.
    let mut items = vec![0u32; layout.items()];
    for (i, p) in place.iter().enumerate() {
        items[p.unwrap().0 as usize] = coloring.of[i];
    }
    let finished: Vec<AtomicU32> = layout.colors.iter().map(|_| AtomicU32::new(0)).collect();
    layout.passes(
        &on(4),
        &mut items,
        &(),
        &[Stage::Items(())],
        |_, &mut k, _| {
            let k = k as usize;
            // The first color slow, so a thread that ran ahead would find
            // it unfinished.
            if k == 0 {
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            for (j, f) in finished.iter().enumerate().take(k) {
                assert_eq!(f.load(Ordering::Acquire) as usize, layout.colors[j], "color {k} ran before color {j} was done");
            }
            finished[k].fetch_add(1, Ordering::AcqRel);
        },
        |_, _, _| {},
    );
}
