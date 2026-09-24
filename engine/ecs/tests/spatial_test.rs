//! Spatial tables (docs/architecture/spatial-storage.md), on the harness:
//! region queries and pairs agree with brute force whatever has moved,
//! spawned or changed tables; a system never sees its own rows move and
//! the systems after it do; a re-sort is an apply node the scheduler orders
//! readers of the table after; parallel frames equal sequential ones.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

use engine_ecs::harness::{Cx, IntoSystem, Schedule, SystemDecl};
use engine_ecs::{Bounds, Build, ComponentDesc, Entity, Query, SpatialKey, With, Without, World, component};

component! {
    /// A spatial key: its tables are kept in spatial order.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "test::At", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    /// `At`'s extent.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size: "test::Size" { pub hx: f32, pub hy: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag: "test::Tag" { pub n: u32 }
}

/// Without a size, a point with a little reach.
const POINT: f32 = 0.1;

impl SpatialKey for At {
    type Extent = Size;
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        let (hx, hy) = size.map_or((POINT, POINT), |s| (s.hx, s.hy));
        Bounds::around([self.x, self.y], [hx, hy])
    }
}

/// `At::BIG`'s default: a row with a larger half extent goes in a big page.
const BIG: f32 = 2.0;

fn lcg(s: &mut u64) -> f32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 40) as f32 / (1u64 << 24) as f32
}

/// Every entity with a position, and its box, by brute force.
fn boxes(w: &World) -> Vec<(Entity, Bounds)> {
    let sizes: std::collections::HashMap<Entity, Size> = w.values::<Size>().unwrap_or_default().into_iter().collect();
    let mut out: Vec<(Entity, Bounds)> =
        w.values::<At>().unwrap_or_default().into_iter().map(|(e, at)| (e, at.bounds(sizes.get(&e)))).collect();
    out.sort_by_key(|(e, _)| *e);
    out
}

fn brute_region(w: &World, region: &Bounds) -> Vec<Entity> {
    boxes(w).into_iter().filter(|(_, b)| b.overlaps(region)).map(|(e, _)| e).collect()
}

fn brute_pairs(w: &World, grow: f32) -> Vec<(Entity, Entity)> {
    let all = boxes(w);
    let mut out = Vec::new();
    for (i, (a, ba)) in all.iter().enumerate() {
        for (b, bb) in &all[i + 1..] {
            if ba.grown(grow).overlaps(&bb.grown(grow)) {
                out.push((*a.min(b), *a.max(b)));
            }
        }
    }
    out.sort_unstable();
    out
}

/// The order's invariants, and that each row's stored box is its box now.
fn check(w: &World) {
    let now: std::collections::HashMap<Entity, Bounds> = boxes(w).into_iter().collect();
    for t in w.tables() {
        let Some(spatial) = &t.spatial else { continue };
        let rows = t.rows.read().unwrap();
        let pages = spatial.pages.read().unwrap();
        pages.check(&rows, BIG, 1.0).unwrap_or_else(|e| panic!("table {:?}: {e}", t.id));
        for (p, page) in rows.iter().enumerate() {
            for (r, e) in page.iter().enumerate() {
                assert_eq!(pages.row_bounds(p, r), now[e], "{e:?}'s stored box");
                let at = w.entities.location(*e).expect("alive");
                assert_eq!((at.table, at.page, at.row), (t.id, p as u32, r as u32), "{e:?}'s location");
            }
        }
    }
}

/// Spawns `n` entities over a 50 by 50 square: most with a size, a few of
/// them big, some tagged, some bare points.
fn populate(w: &World, n: usize, seed: &mut u64) -> Vec<Entity> {
    let mut m = w.between_frames(Build::default()).unwrap();
    (0..n)
        .map(|i| {
            let at = At { x: lcg(seed) * 50.0, y: lcg(seed) * 50.0 };
            // Every twentieth is big (and tagged: `i % 5 == 1`).
            let size = if i % 20 == 1 { Size { hx: 3.0, hy: 0.5 } } else { Size { hx: 0.1 + lcg(seed) * 0.5, hy: 0.1 + lcg(seed) * 0.5 } };
            match i % 5 {
                0 => m.spawn((at,)),
                1 => m.spawn((at, size, Tag { n: i as u32 })),
                _ => m.spawn((at, size)),
            }
        })
        .collect()
}

fn region(seed: &mut u64) -> Bounds {
    let (x, y, r) = (lcg(seed) * 50.0, lcg(seed) * 50.0, 1.0 + lcg(seed) * 4.0);
    Bounds::around([x, y], [r, r])
}

static FOUND: Mutex<Vec<Vec<Entity>>> = Mutex::new(Vec::new());
static PAIRS: Mutex<Vec<(Entity, Entity)>> = Mutex::new(Vec::new());
static REGIONS: Mutex<Vec<Bounds>> = Mutex::new(Vec::new());
/// Tests share the statics, so run one at a time.
static SERIAL: Mutex<()> = Mutex::new(());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Answers `REGIONS` into `FOUND`, and pairs within 0.05 into `PAIRS`.
fn probe(_: &mut Cx, mut q: Query<&At>) {
    let regions = REGIONS.lock().unwrap().clone();
    let mut found = Vec::new();
    for r in regions {
        let mut hits = Vec::new();
        q.in_region(r, |row, _| hits.push(row.entity()));
        hits.sort_unstable();
        found.push(hits);
    }
    *FOUND.lock().unwrap() = found;
    *PAIRS.lock().unwrap() = q.near_pairs(0.05);
}

static FRAME: AtomicU32 = AtomicU32::new(0);

/// Moves every position a little, and every seventh a long way.
fn mover(_: &mut Cx, mut q: Query<&mut At>) {
    let frame = FRAME.fetch_add(1, Ordering::SeqCst) as u64;
    let mut s = frame + 99;
    q.for_each(|row, mut at| {
        let jump = (row.entity().index as u64 + frame) % 7 == 0;
        let step = if jump { 20.0 } else { 0.6 };
        at.x = (at.x + (lcg(&mut s) - 0.5) * step).rem_euclid(50.0);
        at.y = (at.y + (lcg(&mut s) - 0.5) * step).rem_euclid(50.0);
    });
}

fn agrees(w: &World, seed: &mut u64) {
    let regions: Vec<Bounds> = (0..40).map(|_| region(seed)).collect();
    *REGIONS.lock().unwrap() = regions.clone();
    Schedule { systems: vec![probe.system(w, "probe")] }.run_sequential(w);
    let found = FOUND.lock().unwrap().clone();
    for (r, got) in regions.iter().zip(&found) {
        assert_eq!(*got, brute_region(w, r), "region {r:?}");
    }
    assert_eq!(*PAIRS.lock().unwrap(), brute_pairs(w, 0.05));
    check(w);
}

#[test]
fn regions_and_pairs_agree_with_brute_force() {
    let _s = serial();
    let w = World::new();
    let mut seed = 1;
    populate(&w, 600, &mut seed);
    check(&w);
    agrees(&w, &mut seed);
}

#[test]
fn moving_keeps_the_order_and_the_systems_after_see_it() {
    let _s = serial();
    let w = World::new();
    let mut seed = 2;
    populate(&w, 400, &mut seed);
    let s = Schedule { systems: vec![mover.system(&w, "mover"), probe.system(&w, "probe")] };
    for _ in 0..30 {
        let regions: Vec<Bounds> = (0..20).map(|_| region(&mut seed)).collect();
        *REGIONS.lock().unwrap() = regions.clone();
        s.run_sequential(&w);
        // `probe` ran after `mover`'s apply: what it found is where things
        // are now.
        let found = FOUND.lock().unwrap().clone();
        for (r, got) in regions.iter().zip(&found) {
            assert_eq!(*got, brute_region(&w, r));
        }
        assert_eq!(*PAIRS.lock().unwrap(), brute_pairs(&w, 0.05));
        check(&w);
    }
}

#[test]
fn everything_moving_at_once_keeps_the_order() {
    let _s = serial();
    let w = World::new();
    let mut seed = 7;
    populate(&w, 500, &mut seed);
    // Every row leaves its page in one step: full pages of rows that belong
    // elsewhere, split while they're still there.
    fn shift(_: &mut Cx, mut q: Query<&mut At>) {
        q.for_each(|_, mut at| (at.x, at.y) = ((at.x + 25.0) % 50.0, (at.y * 0.5 + 13.0) % 50.0));
    }
    let s = Schedule { systems: vec![shift.system(&w, "shift")] };
    for _ in 0..6 {
        s.run_sequential(&w);
        check(&w);
        agrees(&w, &mut seed);
    }
}

/// Many rows to a cell, so pages fill with rows of one key: the splits
/// around a key half a page shares, which a spread-out table never makes.
#[test]
fn crowded_cells_keep_the_order() {
    let _s = serial();
    let w = World::new();
    let mut seed = 11;
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for _ in 0..600 {
            let at = At { x: lcg(&mut seed) * 5.0, y: lcg(&mut seed) * 5.0 };
            m.spawn((at, Size { hx: 0.1 + lcg(&mut seed) * 0.2, hy: 0.1 }));
        }
    }
    check(&w);
    static FRAME: AtomicU32 = AtomicU32::new(0);
    // Every row a step within the square, a different one each frame.
    fn jitter(_: &mut Cx, mut q: Query<&mut At>) {
        let mut seed = FRAME.fetch_add(1, Ordering::Relaxed) as u64;
        q.for_each(|_, mut at| {
            let (dx, dy) = (lcg(&mut seed) - 0.5, lcg(&mut seed) - 0.5);
            (at.x, at.y) = ((at.x + dx).clamp(0.0, 5.0), (at.y + dy).clamp(0.0, 5.0));
        });
    }
    let s = Schedule { systems: vec![jitter.system(&w, "jitter")] };
    for _ in 0..20 {
        s.run_sequential(&w);
        check(&w);
        agrees(&w, &mut seed);
    }
}

#[test]
fn a_system_does_not_see_its_own_rows_move() {
    let _s = serial();
    let w = World::new();
    let e = w.between_frames(Build::default()).unwrap().spawn((At { x: 5.0, y: 5.0 },));
    static SEEN: Mutex<Vec<String>> = Mutex::new(Vec::new());
    fn teleport(_: &mut Cx, mut q: Query<&mut At>) {
        q.for_each(|_, mut at| (at.x, at.y) = (40.0, 40.0));
        let (mut there, mut here) = (0, 0);
        q.in_region(Bounds::around([40.0, 40.0], [1.0, 1.0]), |_, _| there += 1);
        q.in_region(Bounds::around([5.0, 5.0], [1.0, 1.0]), |_, _| here += 1);
        SEEN.lock().unwrap().push(format!("teleport: there {there} here {here}"));
    }
    fn after(_: &mut Cx, mut q: Query<&At>) {
        let mut there = 0;
        q.in_region(Bounds::around([40.0, 40.0], [1.0, 1.0]), |_, _| there += 1);
        SEEN.lock().unwrap().push(format!("after: there {there}"));
    }
    Schedule { systems: vec![teleport.system(&w, "teleport"), after.system(&w, "after")] }.run_sequential(&w);
    assert_eq!(*SEEN.lock().unwrap(), ["teleport: there 0 here 1", "after: there 1"]);
    assert_eq!(brute_region(&w, &Bounds::around([40.0, 40.0], [1.0, 1.0])), [e]);
    check(&w);
}

#[test]
fn structural_changes_keep_the_order() {
    let _s = serial();
    let w = World::new();
    let mut seed = 3;
    let mut alive = populate(&w, 300, &mut seed);
    for round in 0..20 {
        {
            let mut m = w.between_frames(Build::default()).unwrap();
            for _ in 0..40 {
                let i = (lcg(&mut seed) * alive.len() as f32) as usize % alive.len();
                let e = alive[i];
                match (lcg(&mut seed) * 6.0) as u32 {
                    // Gains or loses its size: another table.
                    0 => m.insert(e, Size { hx: 0.3, hy: 0.3 }),
                    1 => m.remove::<Size>(e),
                    // Loses its position: out of every spatial table.
                    2 => m.remove::<At>(e),
                    // Gains one back.
                    3 => m.insert(e, At { x: lcg(&mut seed) * 50.0, y: lcg(&mut seed) * 50.0 }),
                    4 => {
                        m.despawn(e);
                        alive.swap_remove(i);
                    }
                    _ => alive.push(m.spawn((At { x: lcg(&mut seed) * 50.0, y: lcg(&mut seed) * 50.0 }, Tag { n: round }))),
                }
            }
        }
        check(&w);
        agrees(&w, &mut seed);
    }
}

#[test]
fn writing_an_extent_re_sorts_too() {
    let _s = serial();
    let w = World::new();
    let e = w.between_frames(Build::default()).unwrap().spawn((At { x: 10.0, y: 10.0 }, Size { hx: 0.2, hy: 0.2 }));
    fn grow(_: &mut Cx, mut q: Query<&mut Size>) {
        q.for_each(|_, mut s| s.hx = 6.0);
    }
    Schedule { systems: vec![grow.system(&w, "grow")] }.run_sequential(&w);
    // Now big, so in a big page; and found from where only its reach gets.
    check(&w);
    assert_eq!(brute_region(&w, &Bounds::around([15.5, 10.0], [0.1, 0.1])), [e]);
    let mut seed = 4;
    agrees(&w, &mut seed);
}

/// The frame's nodes, by name, that `name` waits for at the start.
fn blockers(w: &World, s: &Schedule, name: &str) -> Vec<String> {
    let fs = s.frame();
    let i = fs.nodes.iter().position(|&n| s.node_name(n) == name).unwrap();
    s.blockers(w, &fs, i)
}

#[test]
fn a_re_sort_is_an_apply_node_that_readers_of_the_table_wait_for() {
    let _s = serial();
    let w = World::new();
    let mut seed = 5;
    populate(&w, 50, &mut seed);
    // Reads only tags, but tags live in rows the mover's re-sort moves.
    fn tags(_: &mut Cx, mut q: Query<&Tag>) {
        q.for_each(|_, _| {});
    }
    let s = Schedule { systems: vec![mover.system(&w, "mover"), tags.system(&w, "tags")] };
    assert!(blockers(&w, &s, "tags").iter().any(|b| b == "apply(mover)"), "{:?}", blockers(&w, &s, "tags"));
}

#[test]
fn a_parallel_frame_equals_a_sequential_one() {
    let _s = serial();
    fn tagger(_: &mut Cx, mut q: Query<&mut Tag>) {
        q.for_each(|_, mut t| t.n += 1);
    }
    let run = |threads: Option<usize>| {
        FRAME.store(0, Ordering::SeqCst);
        let w = World::new();
        let mut seed = 6;
        populate(&w, 500, &mut seed);
        let systems: Vec<SystemDecl> =
            vec![mover.system(&w, "mover"), probe.system(&w, "probe"), tagger.system(&w, "tagger")];
        let s = Schedule { systems };
        *REGIONS.lock().unwrap() = (0..20).map(|_| region(&mut seed)).collect();
        let mut seen = Vec::new();
        for _ in 0..20 {
            match threads {
                Some(n) => s.run_parallel(&w, n),
                None => s.run_sequential(&w),
            };
            seen.push((FOUND.lock().unwrap().clone(), PAIRS.lock().unwrap().clone()));
        }
        check(&w);
        (boxes(&w), w.values::<Tag>().unwrap().into_iter().map(|(e, t)| (e, t.n)).collect::<std::collections::BTreeMap<_, _>>(), seen)
    };
    let sequential = run(None);
    for _ in 0..3 {
        assert!(run(Some(4)) == sequential, "a parallel frame differed");
    }
}

#[test]
fn boxes_touching_edge_to_edge_pair() {
    let _s = serial();
    let w = World::new();
    // Unit tiles in a row, each touching the next exactly: what a tile
    // level is made of. Two rows, so pages hold neighbors from both.
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for y in 0..2 {
            for x in 0..40 {
                m.spawn((At { x: x as f32 + 0.5, y: y as f32 * 10.0 + 0.5 }, Size { hx: 0.5, hy: 0.5 }));
            }
        }
    }
    fn pairs(_: &mut Cx, mut q: Query<&At>) {
        *PAIRS.lock().unwrap() = q.near_pairs(0.0);
    }
    Schedule { systems: vec![pairs.system(&w, "pairs")] }.run_sequential(&w);
    let got = PAIRS.lock().unwrap().clone();
    assert_eq!(got.len(), 2 * 39, "every neighbor, edge to edge");
    assert_eq!(got, brute_pairs(&w, 0.0));
}

/// Rows whose boxes the last re-sort of each spatial table recomputed.
fn rebounded(w: &World) -> usize {
    w.tables().filter_map(|t| t.spatial.as_ref()).map(|s| s.pages.read().unwrap().rebounded).sum()
}

#[test]
fn a_re_sort_recomputes_only_the_rows_written_through() {
    let _s = serial();
    let w = World::new();
    let mut seed = 8;
    let es = populate(&w, 300, &mut seed);
    static ONE: Mutex<Option<Entity>> = Mutex::new(None);
    *ONE.lock().unwrap() = Some(es[7]);
    // Visits every row mutably, reads them all, writes one.
    fn nudge(_: &mut Cx, mut q: Query<&mut At>) {
        let one = ONE.lock().unwrap().unwrap();
        let mut sum = 0.0;
        q.for_each(|row, mut at| {
            sum += at.x;
            if row.entity() == one {
                at.x = (at.x + 1.0) % 50.0;
            }
        });
        assert!(sum > 0.0);
    }
    fn look(_: &mut Cx, mut q: Query<&mut At>) {
        q.for_each(|_, at| assert!(at.x >= 0.0));
    }
    Schedule { systems: vec![nudge.system(&w, "nudge")] }.run_sequential(&w);
    assert_eq!(rebounded(&w), 1, "one row was written");
    Schedule { systems: vec![look.system(&w, "look")] }.run_sequential(&w);
    assert_eq!(rebounded(&w), 0, "reading through a write term writes nothing");
    check(&w);
    agrees(&w, &mut seed);
}

#[test]
fn whether_a_component_is_spatial_is_fixed_like_its_storage() {
    component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        struct Flat: "test::At" { x: f32, y: f32 }
    }
    let w = World::new();
    w.between_frames(Build::default()).unwrap().spawn((At::default(),));
    let err = w.intern(&ComponentDesc::of::<Flat>()).unwrap_err();
    assert!(err.contains("spatial order"), "{err}");
}

component! {
    /// Sparse, so a filter on it is checked row by row, not by table.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Mark: "test::Mark", storage = sparse { pub n: u32 }
}

fn brute_pairs_of(w: &World, grow: f32, keep: impl Fn(Entity) -> bool) -> Vec<(Entity, Entity)> {
    brute_pairs(w, grow).into_iter().filter(|&(a, b)| keep(a) && keep(b)).collect()
}

#[test]
fn pairs_through_filters_agree_with_brute_force() {
    let _s = serial();
    let w = World::new();
    let mut seed = 9;
    let es = populate(&w, 600, &mut seed);
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for &e in es.iter().filter(|e| e.index % 3 == 0) {
            m.insert(e, Mark { n: 1 });
        }
    }
    static WITH: Mutex<Vec<(Entity, Entity)>> = Mutex::new(Vec::new());
    static WITHOUT_TAG: Mutex<Vec<(Entity, Entity)>> = Mutex::new(Vec::new());
    fn without(_: &mut Cx, mut q: Query<&At, Without<Mark>>) {
        *PAIRS.lock().unwrap() = q.near_pairs(0.05);
    }
    fn with(_: &mut Cx, mut q: Query<&At, With<Mark>>) {
        *WITH.lock().unwrap() = q.near_pairs(0.05);
    }
    fn without_tag(_: &mut Cx, mut q: Query<&At, Without<Tag>>) {
        *WITHOUT_TAG.lock().unwrap() = q.near_pairs(0.05);
    }
    let s = Schedule {
        systems: vec![
            mover.system(&w, "mover"),
            without.system(&w, "without"),
            with.system(&w, "with"),
            without_tag.system(&w, "without_tag"),
        ],
    };
    let marked: std::collections::HashSet<Entity> = w.values::<Mark>().unwrap().into_iter().map(|(e, _)| e).collect();
    let tagged: std::collections::HashSet<Entity> = w.values::<Tag>().unwrap().into_iter().map(|(e, _)| e).collect();
    for _ in 0..10 {
        s.run_sequential(&w);
        let (unmarked, marked_pairs) = (PAIRS.lock().unwrap().clone(), WITH.lock().unwrap().clone());
        assert!(!unmarked.is_empty() && !marked_pairs.is_empty());
        assert_eq!(unmarked, brute_pairs_of(&w, 0.05, |e| !marked.contains(&e)));
        assert_eq!(marked_pairs, brute_pairs_of(&w, 0.05, |e| marked.contains(&e)));
        assert_eq!(*WITHOUT_TAG.lock().unwrap(), brute_pairs_of(&w, 0.05, |e| !tagged.contains(&e)));
    }
}

#[test]
fn a_dense_pile_pairs_as_brute_force_says() {
    let _s = serial();
    let w = World::new();
    // Touching boxes in rows, as a settled pile is: full pages, each meeting
    // its neighbors on every side, most rows reaching into the next page.
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let mut seed = 10;
        for k in 0..900 {
            let (col, row) = ((k % 45) as f32, (k / 45) as f32);
            let at = At { x: col * 0.9 + lcg(&mut seed) * 0.05, y: row * 0.9 + lcg(&mut seed) * 0.05 };
            m.spawn((at, Size { hx: 0.45, hy: 0.45 }));
        }
        // A floor under all of it, in a big page.
        m.spawn((At { x: 20.0, y: -0.9 }, Size { hx: 21.0, hy: 0.5 }));
    }
    fn jiggle(_: &mut Cx, mut q: Query<&mut At, With<Size>>) {
        let frame = FRAME.fetch_add(1, Ordering::SeqCst) as u64;
        let mut s = frame + 5;
        q.for_each(|_, mut at| {
            at.x += (lcg(&mut s) - 0.5) * 0.04;
            at.y += (lcg(&mut s) - 0.5) * 0.04;
        });
    }
    fn pairs(_: &mut Cx, mut q: Query<&At>) {
        *PAIRS.lock().unwrap() = q.near_pairs(0.05);
    }
    let s = Schedule { systems: vec![jiggle.system(&w, "jiggle"), pairs.system(&w, "pairs")] };
    for _ in 0..10 {
        s.run_sequential(&w);
        let got = PAIRS.lock().unwrap().clone();
        assert!(got.len() > 2000, "{} pairs: not dense", got.len());
        assert_eq!(got, brute_pairs(&w, 0.05));
        check(&w);
    }
}
