//! Page walks (`for_each_page`, `for_each_ordered_page`) and the walk
//! `for_each` takes for table-only queries, on the harness: they see every
//! row brute force says the query matches, with its own values, in the
//! order `for_each` and `for_each_ordered` do; writes through a page stamp
//! exactly the rows they say, as the spatial re-sort reads them back; rows
//! from a page change the world; sparse terms and filters are refused; a
//! walk for what changed sees exactly the rows written since.

use std::collections::HashSet;
use std::sync::{Arc, Mutex};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Despawns, Entity, Executor, OrderKey, Query, Scoped, SpatialKey, With, Without, Workers, World, WorldMut, component};

component! {
    /// A spatial key, so some rows are in pages of a dozen or so.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "test::At", order = spatial { pub x: f32, pub y: f32 }
}

impl SpatialKey for At {
    type Extent = At;
    fn bounds(&self, _: Option<&At>) -> Bounds {
        Bounds::around([self.x, self.y], [0.1, 0.1])
    }
}

component! {
    /// Every entity's own value, so a walk that hands out another row's
    /// shows.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Val: "test::Val" { pub n: u64 }
}

component! {
    /// Moves an entity to another table.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag: "test::Tag" {}
}

component! {
    /// An ordered key with ties, so the entity tie-break matters.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Rank: "test::Rank", order = key { pub n: u32 }
}

impl OrderKey for Rank {
    fn key(&self) -> u128 {
        self.n as u128
    }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Mark: "test::Mark", storage = sparse {}
}

/// `native` rows or rounds, or `miri` under Miri, where each costs a
/// thousand times more. The test that needs rows enough to split an
/// ordered walk across threads is skipped there instead.
const fn sized(native: usize, miri: usize) -> usize {
    if cfg!(miri) { miri } else { native }
}

fn lcg(s: &mut u64) -> u32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 33) as u32
}

/// Tests share the statics, so run one at a time.
static SERIAL: Mutex<()> = Mutex::new(());
static SEEN: Mutex<Vec<(Entity, u64)>> = Mutex::new(Vec::new());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

fn run<P>(w: &World, system: impl IntoSystem<P>, name: &str) {
    Schedule { systems: vec![system.system(w, name)] }.run_sequential(w);
}

fn take_seen() -> Vec<(Entity, u64)> {
    std::mem::take(&mut *SEEN.lock().unwrap())
}

/// Entities with `Val` in five tables (spatial or not, ranked or not,
/// tagged or not), some despawned after, so pages have holes and tables
/// uneven ends.
fn populate(w: &World, seed: &mut u64) -> Vec<Entity> {
    let mut m = w.between_frames(Build::default()).unwrap();
    let mut live = Vec::new();
    for k in 0..sized(700, 120) as u64 {
        let v = Val { n: k * 7 + 1 };
        let (x, y) = ((lcg(seed) % 400) as f32 / 10.0, (lcg(seed) % 300) as f32 / 10.0);
        let rank = Rank { n: lcg(seed) % 20 };
        let e = match lcg(seed) % 5 {
            0 => m.spawn((v, At { x, y })),
            1 => m.spawn((v, rank)),
            2 => m.spawn((v, rank, Tag {})),
            3 => m.spawn((v, rank, At { x, y })),
            _ => m.spawn((v,)),
        };
        live.push(e);
    }
    for i in (0..live.len()).rev() {
        if lcg(seed).is_multiple_of(5) {
            m.despawn(live.swap_remove(i));
        }
    }
    live
}

/// Every `Val`, by entity.
fn brute(w: &World) -> Vec<(Entity, u64)> {
    let mut all: Vec<(Entity, u64)> = w.values::<Val>().unwrap().into_iter().map(|(e, v)| (e, v.n)).collect();
    all.sort();
    all
}

fn sorted(mut v: Vec<(Entity, u64)>) -> Vec<(Entity, u64)> {
    v.sort();
    v
}

#[test]
fn a_page_walk_sees_what_for_each_does_in_its_order() {
    let _s = serial();
    let w = World::new();
    populate(&w, &mut 3);
    fn paged(_: &mut Cx, mut q: Query<&Val>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each_page(|page, v| {
            assert_eq!(page.rows(), 0..v.len(), "a page walk hands out whole pages");
            assert_eq!(page.entities().len(), v.len());
            assert!(!v.is_empty(), "empty pages are skipped");
            seen.extend(page.rows().map(|r| (page.entity(r), v[r].n)));
        });
    }
    fn each(_: &mut Cx, mut q: Query<&Val>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each(|row, v| seen.push((row.entity(), v.n)));
    }
    run(&w, paged, "paged");
    let by_page = take_seen();
    run(&w, each, "each");
    assert_eq!(by_page, take_seen(), "the same rows in the same order");
    assert_eq!(sorted(by_page), brute(&w), "every row, with its own value");
}

#[test]
fn for_each_over_tables_agrees_with_brute_force_with_or_without_a_sparse_filter() {
    let _s = serial();
    let w = World::new();
    let live = populate(&w, &mut 4);
    let marked: HashSet<Entity> = live.iter().step_by(3).copied().collect();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for &e in &marked {
            m.insert(e, Mark {});
        }
    }
    fn all(_: &mut Cx, mut q: Query<&Val>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each(|row, v| seen.push((row.entity(), v.n)));
    }
    fn unmarked(_: &mut Cx, mut q: Query<&Val, Without<Mark>>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each(|row, v| seen.push((row.entity(), v.n)));
    }
    run(&w, all, "all");
    assert_eq!(sorted(take_seen()), brute(&w));
    run(&w, unmarked, "unmarked");
    let want: Vec<(Entity, u64)> = brute(&w).into_iter().filter(|(e, _)| !marked.contains(e)).collect();
    assert_eq!(sorted(take_seen()), want, "the sparse filter still filters");
}

#[test]
fn an_ordered_page_walk_is_in_key_order_across_tables() {
    let _s = serial();
    let w = World::new();
    populate(&w, &mut 5);
    let ranked = w.tables().filter(|t| t.ordered.is_some() && !t.is_empty()).count();
    assert!(ranked >= 2, "rows of one key in two tables, which only a merge orders");
    static RUNS: Mutex<Vec<usize>> = Mutex::new(Vec::new());
    fn paged(_: &mut Cx, mut q: Query<(&Val, &Rank)>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each_ordered_page(|page, (v, rank)| {
            assert!(page.rows().end <= v.len() && !page.rows().is_empty(), "a run is rows of its page");
            RUNS.lock().unwrap().push(page.rows().len());
            seen.extend(page.rows().map(|r| (page.entity(r), rank[r].n as u64)));
        });
    }
    fn each(_: &mut Cx, mut q: Query<(&Val, &Rank)>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each_ordered(|row, (_, rank)| seen.push((row.entity(), rank.n as u64)));
    }
    run(&w, paged, "paged");
    let by_run = take_seen();
    run(&w, each, "each");
    assert_eq!(by_run, take_seen(), "the order for_each_ordered walks");
    // By key, then entity, over the tables kept by key: a table with a
    // spatial key too is in spatial order, and comes after.
    let spatial: HashSet<Entity> = w.values::<At>().unwrap().into_iter().map(|(e, _)| e).collect();
    let mut want: Vec<(u64, Entity)> =
        w.values::<Rank>().unwrap().into_iter().filter(|(e, _)| !spatial.contains(e)).map(|(e, r)| (r.n as u64, e)).collect();
    want.sort();
    assert!(by_run.len() > want.len(), "and rows of a table that isn't kept by key");
    let got: Vec<(u64, Entity)> = by_run.iter().take(want.len()).map(|&(e, k)| (k, e)).collect();
    assert_eq!(got, want, "the ordered tables first, merged by key");
    let runs = std::mem::take(&mut *RUNS.lock().unwrap());
    assert!(runs.iter().any(|&n| n > 1), "runs are longer than a row");
}

/// Rows a spatial table's last re-sort recomputed boxes for: rows whose
/// key's tick says they were written.
fn rebounded(w: &World) -> usize {
    w.tables().filter_map(|t| t.spatial.as_ref()).map(|s| s.pages.read().unwrap().rebounded).sum()
}

fn spatial_rows(w: &World) -> usize {
    w.tables().filter(|t| t.spatial.is_some()).map(|t| t.len()).sum()
}

#[test]
fn writes_through_a_page_stamp_the_rows_they_say() {
    let _s = serial();
    let w = World::new();
    populate(&w, &mut 6);
    static EVERY: Mutex<usize> = Mutex::new(7);
    // Reads every row through the write term, and writes every 7th.
    fn set_some(_: &mut Cx, mut q: Query<&mut At>) {
        let every = *EVERY.lock().unwrap();
        let mut i = 0;
        q.for_each_page(|page, mut at| {
            for r in page.rows() {
                if i % every == 0 {
                    let a = at[r];
                    at.set(r, At { x: a.x, y: a.y + 0.5 });
                }
                assert!(at.as_slice()[r].x >= 0.0);
                i += 1;
            }
        });
    }
    fn mut_some(_: &mut Cx, mut q: Query<&mut At>) {
        let every = *EVERY.lock().unwrap();
        let mut i = 0;
        q.for_each_page(|page, mut at| {
            for r in page.rows() {
                let mut a = at.get_mut(r);
                if i % every == 0 {
                    a.y -= 0.5;
                } else {
                    assert!(a.x >= 0.0, "a read through `Mut` isn't a write");
                }
                i += 1;
            }
        });
    }
    fn all(_: &mut Cx, mut q: Query<&mut At>) {
        q.for_each_page(|_, mut at| {
            let _ = at.write_all();
        });
    }
    let n = spatial_rows(&w);
    let want = n.div_ceil(7);
    run(&w, set_some, "set_some");
    assert_eq!(rebounded(&w), want, "set stamps its row");
    run(&w, mut_some, "mut_some");
    assert_eq!(rebounded(&w), want, "get_mut stamps the rows written through it");
    run(&w, all, "all");
    assert_eq!(rebounded(&w), n, "write_all stamps every row, changed or not");
}

#[test]
fn rows_from_a_page_change_the_world() {
    let _s = serial();
    let w = World::new();
    let live = populate(&w, &mut 7);
    fn reap(_: &mut Cx, mut q: Query<&Val, (), Despawns>) {
        q.for_each_page(|page, v| {
            for r in page.rows().filter(|&r| v[r].n % 3 == 0) {
                page.row(r).despawn();
            }
        });
    }
    let want: Vec<(Entity, u64)> = brute(&w).into_iter().filter(|(_, n)| n % 3 != 0).collect();
    assert!(want.len() < live.len(), "some rows go");
    run(&w, reap, "reap");
    assert_eq!(brute(&w), want);
}

#[test]
#[should_panic(expected = "filters by a sparse component")]
fn a_page_walk_refuses_a_sparse_filter() {
    let _s = serial();
    let w = World::new();
    populate(&w, &mut 8);
    fn walk(_: &mut Cx, mut q: Query<&Val, Without<Mark>>) {
        q.for_each_page(|_, _| {});
    }
    run(&w, walk, "walk");
}

#[test]
#[should_panic(expected = "is sparse, and a page walk is over table components")]
fn a_page_walk_refuses_a_sparse_term() {
    let _s = serial();
    let w = World::new();
    let live = populate(&w, &mut 9);
    w.between_frames(Build::default()).unwrap().insert(live[0], Mark {});
    fn walk(_: &mut Cx, mut q: Query<(&Val, &Mark)>) {
        q.for_each_page(|_, _| {});
    }
    run(&w, walk, "walk");
}


component! {
    /// Moves an entity to a table nothing has walked.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Moved: "test::Moved" {}
}

static SINCE: Mutex<u32> = Mutex::new(0);
static WROTE: Mutex<Vec<Entity>> = Mutex::new(Vec::new());
static CHANGED: Mutex<Vec<Entity>> = Mutex::new(Vec::new());

/// Change detection (`Query::for_each_written`): exactly the rows written
/// after a `now`, by a system through `Mut` or between frames, in either
/// term, wherever they've moved since; nothing once they're older than the
/// `now`. The writes between frames are to tables the system didn't walk,
/// whose pages then say they're written only if the write told them.
#[test]
fn a_walk_for_what_changed_sees_exactly_the_rows_written_since() {
    let _s = serial();
    let w = World::new();
    let live = populate(&w, &mut 9);
    fn mark(_: &mut Cx, q: Query<&Val>) {
        *SINCE.lock().unwrap() = q.now();
    }
    // Visits every ranked row mutably, and writes every 13th.
    fn write_some(_: &mut Cx, mut q: Query<&mut Val, With<Rank>>) {
        let mut wrote = WROTE.lock().unwrap();
        q.for_each(|row, mut v| {
            if v.n % 13 == 0 {
                v.n += 13 * 7;
                wrote.push(row.entity());
            } else {
                assert!(v.n > 0, "a read through `Mut` isn't a write");
            }
        });
    }
    // Two terms, either written: tables with `At`, and the rest.
    fn changed(_: &mut Cx, mut q: Query<(&Val, &At)>, mut vals: Query<&Val, Without<At>>) {
        let since = *SINCE.lock().unwrap();
        let mut seen = CHANGED.lock().unwrap();
        q.for_each_written(since, |row, _| seen.push(row.entity()));
        vals.for_each_written(since, |row, _| seen.push(row.entity()));
    }
    let changed_now = || {
        run(&w, changed, "changed");
        let mut seen = std::mem::take(&mut *CHANGED.lock().unwrap());
        seen.sort();
        seen
    };
    run(&w, mark, "mark");
    assert_eq!(changed_now(), [], "nothing written since");
    run(&w, write_some, "write_some");
    let mut want = std::mem::take(&mut *WROTE.lock().unwrap());
    assert!(want.len() > sized(20, 2));
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        // Written rows moved to a table no one walked keep their ticks.
        for &e in want.iter().step_by(3) {
            m.insert(e, Moved {});
        }
        // And unranked rows (in tables `write_some` didn't walk) written
        // between frames, some their `Val`, some their `At`.
        let unranked: Vec<Entity> = live.iter().copied().filter(|&e| m.get::<Rank>(e).is_none()).step_by(10).collect();
        for (i, e) in unranked.into_iter().enumerate() {
            let wrote = if i % 2 == 0 { m.with_mut::<Val, _>(e, |v| v.n += 1) } else { m.with_mut::<At, _>(e, |a| a.x += 0.0) };
            if wrote.is_some() {
                want.push(e);
            }
        }
    }
    want.sort();
    assert_eq!(changed_now(), want);
    run(&w, mark, "mark");
    assert_eq!(changed_now(), [], "all of it older than the `now`");
}

/// Per query of `changed`: whether rows arrived in its tables, and left.
static MOVED: Mutex<[bool; 4]> = Mutex::new([false; 4]);

/// Rows new to a query are written as they arrive, spawned or given a
/// term by an insert, so a walk for what changed sees them; rows that left
/// a query's tables, despawned or moved out, have nothing left to see.
/// `arrived_since` and `left_since` say whether any did, for each query
/// only its own tables'. Rows that arrive by a removal keep their ticks,
/// so only `arrived_since` sees them.
#[test]
fn rows_arriving_are_written_and_rows_leaving_are_seen_to_have_left() {
    let _s = serial();
    let w = World::new();
    let live = populate(&w, &mut 13);
    fn mark(_: &mut Cx, q: Query<&Val>) {
        *SINCE.lock().unwrap() = q.now();
    }
    fn changed(_: &mut Cx, mut q: Query<(&Val, &At)>, mut vals: Query<&Val, Without<At>>) {
        let since = *SINCE.lock().unwrap();
        let mut seen = CHANGED.lock().unwrap();
        q.for_each_written(since, |row, _| seen.push(row.entity()));
        vals.for_each_written(since, |row, _| seen.push(row.entity()));
        *MOVED.lock().unwrap() = [q.arrived_since(since), vals.arrived_since(since), q.left_since(since), vals.left_since(since)];
    }
    let changed_now = || {
        run(&w, changed, "changed");
        let mut seen = std::mem::take(&mut *CHANGED.lock().unwrap());
        seen.sort();
        (seen, *MOVED.lock().unwrap())
    };
    let between = |f: &mut dyn FnMut(&mut WorldMut<'_>) -> Vec<Entity>| {
        run(&w, mark, "mark");
        let mut m = w.between_frames(Build::default()).unwrap();
        let mut want = f(&mut m);
        want.sort();
        want
    };
    let (with_at, without_at): (Vec<Entity>, Vec<Entity>) = {
        let m = w.between_frames(Build::default()).unwrap();
        live.iter().partition(|&&e| m.get::<At>(e).is_some())
    };

    let want = between(&mut |_| Vec::new());
    assert_eq!(changed_now(), (want, [false; 4]), "nothing arrived or left");
    let want = between(&mut |m| vec![m.spawn((Val { n: 1 }, At { x: 3.0, y: 4.0 })), m.spawn((Val { n: 2 },))]);
    assert_eq!(changed_now(), (want, [true, true, false, false]), "spawned: arrived, written, and nothing left");
    let want = between(&mut |m| {
        m.insert(without_at[0], At { x: 1.0, y: 1.0 });
        vec![without_at[0]]
    });
    assert_eq!(changed_now(), (want, [true, false, false, true]), "given an `At`: arrived in one query's tables, left the other's");
    let want = between(&mut |m| {
        m.despawn(with_at[0]);
        Vec::new()
    });
    assert_eq!(changed_now(), (want, [false, false, true, false]), "despawned: left, and nothing to see");
    let want = between(&mut |m| {
        m.remove::<At>(with_at[1]);
        Vec::new()
    });
    assert_eq!(changed_now(), (want, [false, true, true, false]), "an `At` removed: left one query's tables, and arrived unwritten in the other's");
    let want = between(&mut |_| Vec::new());
    assert_eq!(changed_now(), (want, [false; 4]), "all of it older than the `now`");
}

/// Executors for the parallel walks: one thread, then threads spawned per
/// run, so tasks really run at once and finish in any order.
fn executors() -> Vec<Workers> {
    let mut all = vec![Workers::default()];
    all.extend([2, 3, 8].map(|n| Workers::new(Some(Arc::new(Scoped(n)) as Arc<dyn Executor>))));
    all
}

static WORKERS: Mutex<Option<Workers>> = Mutex::new(None);
type Chunks = Vec<(std::ops::Range<usize>, Vec<(Entity, u64)>)>;
static CHUNKS: Mutex<Chunks> = Mutex::new(Vec::new());

fn workers() -> Workers {
    WORKERS.lock().unwrap().clone().expect("an executor")
}

/// `par_for_each_page` is `for_each_page` in chunks: each chunk's rows are
/// the walk's next ones, in order, and the range `make` got says which;
/// with threads there's more than one. The ordered walk is
/// `for_each_ordered_page`'s, over an ordered table and one that isn't.
#[test]
#[cfg_attr(miri, ignore = "needs rows enough to split the ordered walk; too slow interpreted")]
fn a_parallel_page_walk_is_the_page_walk_in_chunks() {
    let _s = serial();
    let w = World::new();
    // The ranked table in spatial order made first, so walking in table
    // order isn't walking the ordered table first.
    w.between_frames(Build::default()).unwrap().spawn((Val { n: 1 }, Rank { n: 3 }, At { x: 1.0, y: 1.0 }));
    // Enough rows that the smaller, ordered walk splits too.
    for mut seed in [10, 20, 30] {
        populate(&w, &mut seed);
    }
    fn split(_: &mut Cx, mut q: Query<&Val>) {
        let chunks = q.par_for_each_page(&workers(), |r| (r, Vec::new()), |(_, seen), page, v| {
            seen.extend(page.rows().map(|r| (page.entity(r), v[r].n)));
        });
        *CHUNKS.lock().unwrap() = chunks;
    }
    fn split_ordered(_: &mut Cx, mut q: Query<(&Val, &Rank), Without<Tag>>) {
        let chunks = q.par_for_each_ordered_page(&workers(), |r| (r, Vec::new()), |(_, seen), page, (_, rank)| {
            seen.extend(page.rows().map(|r| (page.entity(r), rank[r].n as u64)));
        });
        *CHUNKS.lock().unwrap() = chunks;
    }
    fn paged(_: &mut Cx, mut q: Query<&Val>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each_page(|page, v| seen.extend(page.rows().map(|r| (page.entity(r), v[r].n))));
    }
    fn paged_ordered(_: &mut Cx, mut q: Query<(&Val, &Rank), Without<Tag>>) {
        let mut seen = SEEN.lock().unwrap();
        q.for_each_ordered_page(|page, (_, rank)| seen.extend(page.rows().map(|r| (page.entity(r), rank[r].n as u64))));
    }
    run(&w, paged, "paged");
    let whole = take_seen();
    run(&w, paged_ordered, "paged_ordered");
    let ordered = take_seen();
    let ranked = w.tables().filter(|t| t.ordered.is_some() && !t.is_empty()).count();
    assert!(ranked >= 2 && ordered.len() < whole.len(), "tables ordered or not, and some the ordered walk leaves out");
    for (i, workers) in executors().into_iter().enumerate() {
        *WORKERS.lock().unwrap() = Some(workers.clone());
        for (ordered_walk, want) in [(false, &whole), (true, &ordered)] {
            if ordered_walk {
                run(&w, split_ordered, "split_ordered");
            } else {
                run(&w, split, "split");
            }
            let chunks = std::mem::take(&mut *CHUNKS.lock().unwrap());
            assert_eq!(chunks.len() > 1, i > 0, "{} chunks at {} threads", chunks.len(), workers.threads());
            let mut at = 0;
            for (range, seen) in &chunks {
                assert!(!seen.is_empty(), "no chunk is empty");
                assert_eq!(*range, at..at + seen.len(), "a chunk's range is its rows in the walk");
                at = range.end;
            }
            let joined: Vec<(Entity, u64)> = chunks.into_iter().flat_map(|(_, seen)| seen).collect();
            assert_eq!(joined, *want, "the walk's rows, in its order, at {} threads", workers.threads());
        }
    }
}

/// Writes and changes made through a parallel walk are one thread's: the
/// same rows stamped written, and the same log, in the same order, which
/// shows in the ids entities spawned after it get (despawns free them in
/// log order).
#[test]
fn a_parallel_walk_writes_and_changes_what_one_thread_does() {
    let _s = serial();
    static SINCE: Mutex<u32> = Mutex::new(0);
    fn mark(_: &mut Cx, q: Query<&Val>) {
        *SINCE.lock().unwrap() = q.now();
    }
    fn change(_: &mut Cx, mut q: Query<&mut Val, (), Despawns>) {
        q.par_for_each_page(&workers(), |_| (), |_, page, mut v| {
            for r in page.rows() {
                let n = v[r].n;
                if n % 5 == 0 {
                    v.set(r, Val { n: n + 1 });
                } else if n % 11 == 0 {
                    // Read through `Mut`, not written.
                    assert!(v.get_mut(r).n > 0);
                }
                if n % 3 == 0 {
                    page.row(r).despawn();
                }
            }
        });
    }
    fn written(_: &mut Cx, mut q: Query<&Val>) {
        let since = *SINCE.lock().unwrap();
        let mut seen = SEEN.lock().unwrap();
        q.for_each_written(since, |row, v| seen.push((row.entity(), v.n)));
    }
    let mut outcomes = Vec::new();
    for workers in executors() {
        *WORKERS.lock().unwrap() = Some(workers);
        let w = World::new();
        populate(&w, &mut 11);
        run(&w, mark, "mark");
        run(&w, change, "change");
        run(&w, written, "written");
        let wrote = sorted(take_seen());
        assert!(!wrote.is_empty());
        let spawned: Vec<Entity> = {
            let mut m = w.between_frames(Build::default()).unwrap();
            (0..sized(100, 10) as u64).map(|k| m.spawn((Val { n: k },))).collect()
        };
        outcomes.push((brute(&w), wrote, spawned));
    }
    for o in &outcomes[1..] {
        assert!(o.0 == outcomes[0].0, "the same values and rows");
        assert!(o.1 == outcomes[0].1, "the same rows stamped written");
        assert!(o.2 == outcomes[0].2, "the same ids for what's spawned after: the same log");
    }
}

#[test]
#[should_panic(expected = "over one ordered table")]
fn a_parallel_walk_in_key_order_refuses_two_ordered_tables() {
    let _s = serial();
    let w = World::new();
    populate(&w, &mut 12);
    *WORKERS.lock().unwrap() = Some(Workers::new(Some(Arc::new(Scoped(2)))));
    fn walk(_: &mut Cx, mut q: Query<(&Val, &Rank)>) {
        q.par_for_each_ordered_page(&workers(), |_| (), |_, _, _| {});
    }
    run(&w, walk, "walk");
}
