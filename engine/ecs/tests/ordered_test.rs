//! Ordered tables (engine/ecs/ordered.rs), on the harness: whatever spawns,
//! despawns, changes tables or writes keys, every ordered table stays in
//! key order (ties by entity), each row keeps its values and its location,
//! and walks by key agree with brute force; a system never sees its own
//! rows move and the systems after it do; a re-sort is an apply node that
//! readers of the table wait for.

use std::collections::HashMap;
use std::sync::Mutex;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Entity, OrderKey, Query, SpatialKey, World, component, pair_key, pairs_from};

component! {
    /// An ordered key with many ties, so the entity tie-break matters.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Rank: "test::Rank", order = key { pub n: u32 }
}

impl OrderKey for Rank {
    fn key(&self) -> u128 {
        self.n as u128
    }
}

component! {
    /// An ordered pair, as contacts are.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Link: "test::Link", order = key { pub a: Entity, pub b: Entity }
}

impl OrderKey for Link {
    fn key(&self) -> u128 {
        pair_key(self.a, self.b)
    }
}

component! {
    /// Moves rows between tables when added or removed.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag: "test::Tag" {}
}

component! {
    /// Heap data, so a re-sort that loses, doubles or mixes up values shows.
    #[derive(Debug, Default, PartialEq)]
    pub struct Name: "test::Name" { pub s: String }
}

component! {
    /// A spatial key: a table holding it and `Rank` is in spatial order.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "test::At", order = spatial { pub x: f32 }
}

impl SpatialKey for At {
    type Extent = At;
    fn bounds(&self, _: Option<&At>) -> Bounds {
        Bounds::around([self.x, 0.0], [0.1, 0.1])
    }
}

/// `native` rows or rounds, or `miri` under Miri, where each costs a
/// thousand times more.
const fn sized(native: usize, miri: usize) -> usize {
    if cfg!(miri) { miri } else { native }
}

fn lcg(s: &mut u64) -> u32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 33) as u32
}

/// Tests share the statics, so run one at a time.
static SERIAL: Mutex<()> = Mutex::new(());
static SEEN: Mutex<Vec<Entity>> = Mutex::new(Vec::new());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Every table ordered by `Rank`: its rows in order, their recorded keys
/// their keys now, their locations where they are, and every entity's name
/// the one it was spawned with.
fn check(w: &World, names: &HashMap<Entity, String>) {
    let ranks: HashMap<Entity, Rank> = w.values::<Rank>().unwrap_or_default().into_iter().collect();
    let links: HashMap<Entity, Link> = w.values::<Link>().unwrap_or_default().into_iter().collect();
    let mut ordered = 0;
    for t in w.tables() {
        let Some(o) = &t.ordered else { continue };
        ordered += 1;
        let rows = t.rows.read().unwrap();
        let order = o.order.read().unwrap();
        let key = |p: usize, r: usize| {
            let e = rows[p][r];
            ranks.get(&e).map(Rank::key).or_else(|| links.get(&e).map(Link::key)).expect("a key")
        };
        order.check(&rows, key).unwrap_or_else(|e| panic!("table {:?}: {e}", t.id));
        for (p, page) in rows.iter().enumerate() {
            for (r, e) in page.iter().enumerate() {
                let at = w.entities.location(*e).expect("alive");
                assert_eq!((at.table, at.page, at.row), (t.id, p as u32, r as u32), "{e:?}'s location");
            }
        }
    }
    assert!(ordered > 0, "the test has ordered tables");
    let now: HashMap<Entity, String> = w.values::<Name>().unwrap_or_default().into_iter().map(|(e, n)| (e, n.s)).collect();
    assert_eq!(&now, names, "every entity keeps its own values");
}

/// `Rank` entities in key order, by brute force: key, then entity.
fn brute(w: &World) -> Vec<Entity> {
    let mut all: Vec<(u32, Entity)> = w.values::<Rank>().unwrap_or_default().into_iter().map(|(e, r)| (r.n, e)).collect();
    all.sort();
    all.into_iter().map(|(_, e)| e).collect()
}

fn walk_ordered(_: &mut Cx, mut q: Query<&Rank>) {
    let mut seen = SEEN.lock().unwrap();
    seen.clear();
    q.for_each_ordered(|row, _| seen.push(row.entity()));
}

fn seen() -> Vec<Entity> {
    SEEN.lock().unwrap().clone()
}

#[test]
fn random_changes_keep_every_table_in_order() {
    let _s = serial();
    let w = World::new();
    let mut seed = 1;
    let mut names = HashMap::new();
    let mut live = Vec::new();
    let spawn = |w: &World, seed: &mut u64, names: &mut HashMap<Entity, String>, live: &mut Vec<Entity>| {
        let mut m = w.between_frames(Build::default()).unwrap();
        let rank = Rank { n: lcg(seed) % 50 };
        let e = if lcg(seed).is_multiple_of(3) { m.spawn((rank, Tag {}, Name::default())) } else { m.spawn((rank, Name::default())) };
        let s = format!("{e:?}");
        m.insert(e, Name { s: s.clone() });
        names.insert(e, s);
        live.push(e);
    };
    for _ in 0..sized(600, 80) {
        spawn(&w, &mut seed, &mut names, &mut live);
    }
    let walk = Schedule { systems: vec![walk_ordered.system(&w, "walk")] };
    for round in 0..sized(40, 4) {
        {
            let mut m = w.between_frames(Build::default()).unwrap();
            for _ in 0..sized(30, 10) {
                let i = lcg(&mut seed) as usize % live.len();
                let e = live[i];
                match lcg(&mut seed) % 4 {
                    0 => {
                        m.despawn(e);
                        live.swap_remove(i);
                        names.remove(&e);
                    }
                    1 => m.insert(e, Rank { n: lcg(&mut seed) % 50 }),
                    2 => m.insert(e, Tag {}),
                    _ => m.remove::<Tag>(e),
                }
            }
        }
        // Before any spawn, which would mark the tables for sorting anyway.
        check(&w, &names);
        for _ in 0..sized(20, 5) {
            spawn(&w, &mut seed, &mut names, &mut live);
        }
        check(&w, &names);
        walk.run_sequential(&w);
        assert_eq!(seen(), brute(&w), "round {round}: key order across both tables");
    }
}

/// A freed index is reused, so a new entity can sort before an older one
/// with the same key while being appended after it.
#[test]
fn ties_are_in_entity_order_whatever_the_history() {
    let _s = serial();
    let w = World::new();
    let mut m = w.between_frames(Build::default()).unwrap();
    let first = m.spawn((Rank { n: 7 },));
    let second = m.spawn((Rank { n: 7 },));
    m.despawn(first);
    let reused = m.spawn((Rank { n: 7 },));
    drop(m);
    assert!(reused < second, "the test needs a reused, lesser index");
    Schedule { systems: vec![walk_ordered.system(&w, "walk")] }.run_sequential(&w);
    assert_eq!(seen(), [reused, second]);
    check(&w, &HashMap::new());
}

#[test]
fn a_key_range_is_the_rows_with_those_keys() {
    let _s = serial();
    let w = World::new();
    let mut m = w.between_frames(Build::default()).unwrap();
    let ends: Vec<Entity> = (0..20).map(|_| m.spawn((Tag {},))).collect();
    let mut seed = 2;
    let mut links = Vec::new();
    for _ in 0..sized(400, 60) {
        let (a, b) = (ends[lcg(&mut seed) as usize % 20], ends[lcg(&mut seed) as usize % 20]);
        links.push((m.spawn((Link { a, b },)), a, b));
    }
    drop(m);
    static FROM: Mutex<Vec<Entity>> = Mutex::new(Vec::new());
    for &a in &ends {
        let find = move |_: &mut Cx, mut q: Query<&Link>| {
            let mut out = FROM.lock().unwrap();
            out.clear();
            q.in_keys::<Link>(pairs_from(a), |row, _| out.push(row.entity()));
        };
        Schedule { systems: vec![find.system(&w, "find")] }.run_sequential(&w);
        let mut want: Vec<(Entity, Entity)> = links.iter().filter(|l| l.1 == a).map(|l| (l.2, l.0)).collect();
        want.sort();
        let want: Vec<Entity> = want.into_iter().map(|(_, e)| e).collect();
        assert_eq!(*FROM.lock().unwrap(), want, "links from {a:?}, in order of the other end");
    }
}

/// Rows in a table kept in another order (spatial, or another key's) are
/// found by a scan:
/// the same rows as a seek would find, in that table's order.
#[test]
fn a_key_range_over_a_table_in_another_order_is_scanned() {
    let _s = serial();
    let w = World::new();
    let mut m = w.between_frames(Build::default()).unwrap();
    let mut seed = 3;
    // First, so `Link` is the older component and orders the table the two
    // share: `Rank` is in it, but not in `Rank`'s order.
    let end = m.spawn((Tag {},));
    for i in 0..sized(100, 20) as u32 {
        m.spawn((Link { a: end, b: Entity { index: 1000 - i, generation: 0 } }, Rank { n: lcg(&mut seed) % 40 }));
    }
    for i in 0..sized(300, 40) {
        let rank = Rank { n: lcg(&mut seed) % 40 };
        if i % 2 == 0 {
            m.spawn((rank, At { x: lcg(&mut seed) as f32 % 100.0 }));
        } else {
            m.spawn((rank,));
        }
    }
    drop(m);
    assert!(w.tables().any(|t| t.spatial.is_some() && t.ordered.is_none()), "one table is spatial, not ordered");
    let (link, rank) = (w.id("test::Link").unwrap(), w.id("test::Rank").unwrap());
    let by_link = |t: &&engine_ecs::world::Table| t.ordered.as_ref().is_some_and(|o| o.key == link);
    assert!(w.tables().filter(by_link).any(|t| t.components.contains(&rank)), "one is ordered by another key");
    static FOUND: Mutex<Vec<Entity>> = Mutex::new(Vec::new());
    fn find(_: &mut Cx, mut q: Query<&Rank>) {
        let mut out = FOUND.lock().unwrap();
        out.clear();
        q.in_keys::<Rank>(10..=19, |row, _| out.push(row.entity()));
    }
    Schedule { systems: vec![find.system(&w, "find")] }.run_sequential(&w);
    let mut found = FOUND.lock().unwrap().clone();
    found.sort();
    let mut want: Vec<Entity> = w.values::<Rank>().unwrap().into_iter().filter(|(_, r)| (10..20).contains(&r.n)).map(|(e, _)| e).collect();
    want.sort();
    assert_eq!(found, want);
}

#[test]
#[should_panic(expected = "in_keys::<test::Rank> is for a query that reads test::Rank")]
fn a_key_range_needs_the_query_to_read_the_key() {
    let _s = serial();
    let w = World::new();
    w.between_frames(Build::default()).unwrap().spawn((Rank { n: 1 }, Tag {}));
    fn find(_: &mut Cx, mut q: Query<&Tag>) {
        q.in_keys::<Rank>(0..=5, |_, _| {});
    }
    Schedule { systems: vec![find.system(&w, "find")] }.run_sequential(&w);
}

/// Reverses every key; `SEEN` gets the order it walked in.
fn reverse(_: &mut Cx, mut q: Query<&mut Rank>) {
    let mut seen = SEEN.lock().unwrap();
    seen.clear();
    q.for_each(|row, mut r| {
        seen.push(row.entity());
        r.n = 1000 - r.n;
    });
}

#[test]
fn a_system_does_not_see_its_own_rows_move_and_the_next_does() {
    let _s = serial();
    let w = World::new();
    let mut m = w.between_frames(Build::default()).unwrap();
    for n in 0..sized(300, 60) as u32 {
        m.spawn((Rank { n: n % 97 },));
    }
    drop(m);
    let before = brute(&w);
    static AFTER: Mutex<Vec<Entity>> = Mutex::new(Vec::new());
    fn after(_: &mut Cx, mut q: Query<&Rank>) {
        let mut out = AFTER.lock().unwrap();
        out.clear();
        q.for_each(|row, _| out.push(row.entity()));
    }
    let s = Schedule { systems: vec![reverse.system(&w, "reverse"), after.system(&w, "after")] };
    s.run_sequential(&w);
    assert_eq!(seen(), before, "the writer walked the order it started with");
    assert_eq!(*AFTER.lock().unwrap(), brute(&w), "the next system walks the new order");
    assert_ne!(brute(&w), before);
}

#[test]
fn a_re_sort_is_an_apply_node_that_readers_of_the_table_wait_for() {
    let _s = serial();
    let w = World::new();
    w.between_frames(Build::default()).unwrap().spawn((Rank { n: 1 }, Tag {}));
    // Reads only tags, but tags live in rows the re-sort moves.
    fn tags(_: &mut Cx, mut q: Query<&Tag>) {
        q.for_each(|_, _| {});
    }
    let s = Schedule { systems: vec![reverse.system(&w, "reverse"), tags.system(&w, "tags")] };
    let fs = s.frame();
    let i = fs.nodes.iter().position(|&n| s.node_name(n) == "tags").unwrap();
    assert_eq!(s.blockers(&w, &fs, i), ["apply(reverse)"]);
}

/// Rows keyed at the last re-sort, over every ordered table.
fn keyed(w: &World) -> usize {
    w.tables().filter_map(|t| t.ordered.as_ref()).map(|o| o.order.read().unwrap().keyed).sum()
}

/// Visits every rank mutably and writes one: a writer over a table whose
/// keys mostly don't change.
fn bump_one(_: &mut Cx, mut q: Query<&mut Rank>) {
    let mut first = true;
    q.for_each(|_, mut r| {
        if std::mem::take(&mut first) {
            r.n += 100;
        }
    });
}

#[test]
fn a_re_sort_keys_only_the_rows_new_or_written_since() {
    let _s = serial();
    let w = World::new();
    let mut seed = 4;
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for _ in 0..sized(200, 40) {
            m.spawn((Rank { n: lcg(&mut seed) % 50 },));
        }
    }
    // Each spawn between frames is a re-sort of its own.
    assert_eq!(keyed(&w), 1, "a spawn keys only its own row");
    Schedule { systems: vec![bump_one.system(&w, "bump")] }.run_sequential(&w);
    assert_eq!(keyed(&w), 1, "one row was written");
    w.between_frames(Build::default()).unwrap().spawn((Rank { n: 3 },));
    assert_eq!(keyed(&w), 1, "one row is new");
    check(&w, &HashMap::new());
    let walk = Schedule { systems: vec![walk_ordered.system(&w, "walk")] };
    walk.run_sequential(&w);
    assert_eq!(seen(), brute(&w));
}

component! {
    /// `Rank` as a newer build keys it: the same layout, the other way up.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct RankDown: "test::Rank", order = key { pub n: u32 }
}

impl OrderKey for RankDown {
    fn key(&self) -> u128 {
        (u32::MAX - self.n) as u128
    }
}

/// Keys written nowhere can still change: a newer build's glue may key the
/// same values otherwise, so the next re-sort keys every row again.
#[test]
fn a_newer_build_of_the_key_keys_every_row_again() {
    use engine_ecs::ComponentDesc;
    let _s = serial();
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for n in 0..40 {
            m.spawn((Rank { n },));
        }
    }
    let newer = Build { name: "newer".into(), loaded_at: 1, keepalive: None };
    w.install(&ComponentDesc::of::<RankDown>(), &newer).unwrap();
    // A new row marks the table for its next re-sort.
    w.between_frames(Build::default()).unwrap().spawn((Rank { n: 40 },));
    assert_eq!(keyed(&w), 41);
    let t = w.tables().find(|t| t.ordered.is_some()).unwrap();
    let ranks: HashMap<Entity, Rank> = w.values::<Rank>().unwrap().into_iter().collect();
    let order: Vec<u32> = t.rows.read().unwrap().iter().flatten().map(|e| ranks[e].n).collect();
    assert_eq!(order, (0..=40).rev().collect::<Vec<u32>>(), "in the newer build's order");
}

#[test]
fn whether_a_component_is_ordered_is_fixed_like_its_storage() {
    use engine_ecs::ComponentDesc;
    let w = World::new();
    w.between_frames(Build::default()).unwrap().spawn((Rank { n: 1 },));
    let mut plain = ComponentDesc::of::<Rank>();
    plain.order = None;
    let err = w.install(&plain, &Build::default()).unwrap_err();
    assert!(err.contains("ordered key"), "{err}");
}
