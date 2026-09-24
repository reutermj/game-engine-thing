//! Page walks (`for_each_page`, `for_each_ordered_page`) and the walk
//! `for_each` takes for table-only queries, on the harness: they see every
//! row brute force says the query matches, with its own values, in the
//! order `for_each` and `for_each_ordered` do; writes through a page stamp
//! exactly the rows they say, as the spatial re-sort reads them back; rows
//! from a page change the world; sparse terms and filters are refused.

use std::collections::HashSet;
use std::sync::Mutex;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Despawns, Entity, OrderKey, Query, SpatialKey, Without, World, component};

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
    for k in 0..700u64 {
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
        if lcg(seed) % 5 == 0 {
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
