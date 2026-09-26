//! What a structural change costs, by part, per change: sparse inserts and
//! removes against table ones, at 100, 1000 and 10 000 changes a system
//! run, out of 20 000 rows. `./bazel run -c opt //engine/ecs:change_bench`.
//!
//! Two ways: a harness frame, timing the system (which logs the changes)
//! and its apply node (footprint, guards, applying) against the same frame
//! making no changes; and the apply node's parts one at a time, on a log
//! of a boxed closure per insert, as `Row::insert` built it for every
//! component before sparse changes were logged as runs (get-znt.18), and
//! still does for table components.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Instant;

use engine_ecs::graph::exact;
use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::query::Change;
use engine_ecs::{Adds, Build, Component, Entity, Query, Removes, Structural, World, component};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Pos: "bench::Pos" { pub x: f32, pub y: f32 }
}

component! {
    /// Sleep's `Still`, as the prototype had it.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag: "bench::Tag", storage = sparse { pub since: u64 }
}

component! {
    /// The same, stored in tables: an insert or remove moves the row.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Mark: "bench::Mark" { pub since: u64 }
}

component! {
    /// A second sparse component, for changes that alternate between two.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag2: "bench::Tag2", storage = sparse { pub since: u64 }
}

const ROWS: u32 = 20_000;
const ROUNDS: usize = 41;

/// What the systems do this frame: 0 nothing, 1 insert, 2 remove; and to
/// which entities, looked up by `get`, as physics changes the bodies its
/// step chose. A walk of every row would bury a hundred changes' cost in
/// the walk's noise.
static MODE: AtomicU32 = AtomicU32::new(0);
static CHOSEN: Mutex<Vec<Entity>> = Mutex::new(Vec::new());

fn sparse(_: &mut Cx, mut q: Query<&Pos, (), (Adds<Tag>, Removes<Tag>)>) {
    let mode = MODE.load(Ordering::Relaxed);
    for &e in CHOSEN.lock().unwrap().iter() {
        let Some(row) = q.get(e) else { continue };
        match mode {
            1 => row.insert(Tag { since: 7 }),
            2 => row.remove::<Tag>(),
            _ => {}
        }
    }
}

fn table(_: &mut Cx, mut q: Query<&Pos, (), (Adds<Mark>, Removes<Mark>)>) {
    let mode = MODE.load(Ordering::Relaxed);
    for &e in CHOSEN.lock().unwrap().iter() {
        let Some(row) = q.get(e) else { continue };
        match mode {
            1 => row.insert(Mark { since: 7 }),
            2 => row.remove::<Mark>(),
            _ => {}
        }
    }
}

/// Two changes an entity, alternating between two sparse components: runs
/// of one change each, if only consecutive changes join a run.
fn two_sparse(_: &mut Cx, mut q: Query<&Pos, (), (Adds<(Tag, Tag2)>, Removes<(Tag, Tag2)>)>) {
    let mode = MODE.load(Ordering::Relaxed);
    for &e in CHOSEN.lock().unwrap().iter() {
        let Some(row) = q.get(e) else { continue };
        match mode {
            1 => {
                row.insert(Tag { since: 7 });
                row.insert(Tag2 { since: 7 });
            }
            2 => {
                row.remove::<Tag>();
                row.remove::<Tag2>();
            }
            _ => {}
        }
    }
}

/// A sparse change and a table one an entity, alternating, as a woken body
/// loses its `Asleep` (table) and its `Still` (sparse): the table change's
/// cost is the table rows', so this is for the sparse one's.
fn sparse_and_table(_: &mut Cx, mut q: Query<&Pos, (), (Adds<(Tag, Mark)>, Removes<(Tag, Mark)>)>) {
    let mode = MODE.load(Ordering::Relaxed);
    for &e in CHOSEN.lock().unwrap().iter() {
        let Some(row) = q.get(e) else { continue };
        match mode {
            1 => {
                row.insert(Tag { since: 7 });
                row.insert(Mark { since: 7 });
            }
            2 => {
                row.remove::<Tag>();
                row.remove::<Mark>();
            }
            _ => {}
        }
    }
}

fn median(mut v: Vec<u128>) -> u128 {
    v.sort_unstable();
    v[v.len() / 2]
}

/// One frame of a one-system schedule: its system's time and its apply's,
/// in ns.
fn frame(w: &World, s: &Schedule) -> (u128, u128) {
    let mut fs = s.frame();
    w.begin_frame();
    let t = Instant::now();
    s.step(w, &mut fs, 0);
    let run = t.elapsed().as_nanos();
    let t = Instant::now();
    s.step(w, &mut fs, 1);
    let apply = t.elapsed().as_nanos();
    w.end_frame();
    (run, apply)
}

/// Per change, in ns: the system's logging and the apply node, for inserts
/// and then removes, over the same frame making none.
fn frames(w: &World, s: &Schedule, chosen: &[Entity]) -> [f64; 5] {
    *CHOSEN.lock().unwrap() = chosen.to_vec();
    let (mut base, mut ins, mut rem) = (Vec::new(), Vec::new(), Vec::new());
    for _ in 0..ROUNDS {
        for (mode, out) in [(0, &mut base), (1, &mut ins), (2, &mut rem)] {
            MODE.store(mode, Ordering::Relaxed);
            out.push(frame(w, s));
        }
    }
    let run = |v: &Vec<(u128, u128)>| median(v.iter().map(|x| x.0).collect()) as f64;
    let apply = |v: &Vec<(u128, u128)>| median(v.iter().map(|x| x.1).collect()) as f64;
    let n = chosen.len() as f64;
    [
        (run(&ins) - run(&base)) / n,
        (apply(&ins) - apply(&base)) / n,
        (run(&rem) - run(&base)) / n,
        (apply(&rem) - apply(&base)) / n,
        run(&base) / n,
    ]
}

/// The apply node's parts, one at a time, per change in ns: building the
/// log of a boxed closure each, dropping one unapplied,
/// the exact footprint, taking the guards (a sparse set's purge of dead
/// entries), applying, and dropping the `Structural`; for inserts, then
/// removes of the same entities.
fn parts<T: Component + Copy>(w: &World, chosen: &[Entity], value: T) -> [[f64; 6]; 2] {
    let c = w.id(T::NAME).expect("installed");
    let sparse = w.storage(c) == engine_ecs::Storage::Sparse;
    let n = chosen.len() as f64;
    let mut out = [[Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new()], Default::default()];
    for _ in 0..ROUNDS {
        for (k, insert) in [(0, true), (1, false)] {
            let make = || -> Vec<Change> {
                let mut log = Vec::new();
                for &e in chosen {
                    log.push(if insert {
                        Change::Insert { e, c, apply: Box::new(move |s| s.insert_id(e, c, value)) }
                    } else {
                        Change::Remove { e, c }
                    });
                }
                log
            };
            let t = Instant::now();
            let log = make();
            out[k][0].push(t.elapsed().as_nanos());
            let t = Instant::now();
            drop(make());
            out[k][1].push(t.elapsed().as_nanos());
            let t = Instant::now();
            let fp = exact(w, &log);
            out[k][2].push(t.elapsed().as_nanos());
            let t = Instant::now();
            let mut st = Structural::new(w);
            if sparse {
                st.lock_sparse(c);
            } else {
                for shape in &fp.tables {
                    st.lock_table(w.table_for(&shape.lower));
                }
            }
            out[k][3].push(t.elapsed().as_nanos());
            let t = Instant::now();
            for change in log {
                change.apply(&mut st);
            }
            out[k][4].push(t.elapsed().as_nanos());
            let t = Instant::now();
            drop(st);
            out[k][5].push(t.elapsed().as_nanos());
        }
    }
    out.map(|k| k.map(|v| median(v) as f64 / n))
}

fn main() {
    let w = World::new();
    let schedules = [
        ("sparse", Schedule { systems: vec![sparse.system(&w, "sparse")] }),
        ("table", Schedule { systems: vec![table.system(&w, "table")] }),
    ];
    let mut entities = Vec::new();
    {
        let mut between = w.between_frames(Build::default()).unwrap();
        for i in 0..ROWS {
            entities.push(between.spawn((Pos { x: i as f32, y: 0.0 },)));
        }
    }
    println!("{ROWS} rows; ns per change, medians of {ROUNDS} frames");
    println!();
    println!("Whole frames, over the same frame making no changes:");
    println!("| storage | changes | insert: system | insert: apply | remove: system | remove: apply | the system making none |");
    println!("|---|---|---|---|---|---|---|");
    for (name, s) in &schedules {
        for n in [100, 1000, 10_000] {
            let chosen: Vec<Entity> = entities.iter().copied().step_by((ROWS / n) as usize).collect();
            let [a, b, c, d, e] = frames(&w, s, &chosen);
            println!("| {name} | {n} | {a:.1} | {b:.1} | {c:.1} | {d:.1} | {e:.1} |");
        }
    }
    // Per change: two an entity. The table change's moves are most of the
    // second's cost; what's compared is how the sparse changes log.
    let alternating = [
        ("two sparse, alternating", Schedule { systems: vec![two_sparse.system(&w, "two_sparse")] }),
        ("sparse and table, alternating", Schedule { systems: vec![sparse_and_table.system(&w, "sparse_and_table")] }),
    ];
    for (name, s) in &alternating {
        for n in [100, 1000] {
            let chosen: Vec<Entity> = entities.iter().copied().step_by((ROWS / n) as usize).collect();
            let [a, b, c, d, _] = frames(&w, s, &chosen).map(|x| x / 2.0);
            println!("| {name} | {} | {a:.1} | {b:.1} | {c:.1} | {d:.1} |", 2 * n);
        }
    }
    // The set a settling pile's `Still` is: thousands of entries, a few
    // hundred of them changing a step, and entities dying every step (its
    // contacts). Each apply that takes the set walks it to purge the dead.
    {
        let mut between = w.between_frames(Build::default()).unwrap();
        for &e in entities.iter().skip(1).step_by(2) {
            between.insert(e, Tag { since: 1 });
        }
        let dying = between.spawn((Pos::default(),));
        between.despawn(dying);
    }
    for n in [100, 1000] {
        let chosen: Vec<Entity> = entities.iter().copied().step_by((ROWS / n) as usize).collect();
        let [a, b, c, d, _] = frames(&w, &schedules[0].1, &chosen);
        println!("| sparse, beside 10 000 in the set | {n} | {a:.1} | {b:.1} | {c:.1} | {d:.1} |");
    }
    println!();
    println!("The floor: pushing a log entry's 40 bytes onto a `Vec` grown from empty, and onto one with room:");
    for n in [100usize, 1000, 10_000] {
        let (mut grown, mut room) = (Vec::new(), Vec::new());
        let mut kept: Vec<[u64; 5]> = Vec::with_capacity(n);
        for _ in 0..ROUNDS {
            let t = Instant::now();
            let mut v: Vec<[u64; 5]> = Vec::new();
            for i in 0..n as u64 {
                v.push([i; 5]);
            }
            std::hint::black_box(&v);
            grown.push(t.elapsed().as_nanos());
            kept.clear();
            let t = Instant::now();
            for i in 0..n as u64 {
                kept.push([i; 5]);
            }
            std::hint::black_box(&kept);
            room.push(t.elapsed().as_nanos());
        }
        println!("  {n}: {:.1} and {:.1} ns", median(grown) as f64 / n as f64, median(room) as f64 / n as f64);
    }
    println!();
    println!("The apply's parts, on a log of a boxed closure per insert (as sparse inserts were logged before runs):");
    println!("| storage | changes | change | log | dropping it unapplied | exact footprint | guards | apply | `Structural` dropped |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for n in [100u32, 1000, 10_000] {
        let every = (ROWS / n) as usize;
        let chosen: Vec<Entity> = entities.iter().copied().step_by(every).collect();
        for (name, [ins, rem]) in [("sparse", parts(&w, &chosen, Tag { since: 7 })), ("table", parts(&w, &chosen, Mark { since: 7 }))] {
            for (what, p) in [("insert", ins), ("remove", rem)] {
                println!(
                    "| {name} | {n} | {what} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} |",
                    p[0],
                    p[1] - p[0],
                    p[2],
                    p[3],
                    p[4],
                    p[5]
                );
            }
        }
    }
}
