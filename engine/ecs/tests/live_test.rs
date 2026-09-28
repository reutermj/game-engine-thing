//! Live proximity relations (`engine_ecs::live`), against `near_pairs`
//! afresh and brute force, in 2D and 3D: at rest, creeping,
//! a few rows flying and everything falling, extents written, rows
//! spawned, despawned (their indices reused) and moved between tables and
//! between the sides. Every frame its pairs are `near_pairs`' exactly, and
//! how it found them is what the motion says it should be. Then what a
//! relation is to the scheduler and to registration: its footprint, two
//! relations on one key kept apart, a declared margin, and what's refused.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::live::{How, LiveStats};
use engine_ecs::{AnyOf, Bounds, Build, Entity, Live, NearSide, Proximity, Query, SpatialKey, With, Without, World, component, near_pairs};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "live::At", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size: "live::Size" { pub hx: f32, pub hy: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At3: "live::At3", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size3: "live::Size3" { pub h: f32 }
}

component! {
    /// Passive: the rows with it are the broadphase's passive side.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag: "live::Tag" { pub n: u32 }
}

component! {
    /// Sparse, which a side mustn't filter by.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Mark: "live::Mark", storage = sparse { pub n: u32 }
}

impl SpatialKey for At {
    type Extent = Size;
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        let (hx, hy) = size.map_or((0.1, 0.1), |s| (s.hx, s.hy));
        Bounds::around([self.x, self.y], [hx, hy])
    }
}

impl SpatialKey<3> for At3 {
    type Extent = Size3;
    fn bounds(&self, s: Option<&Size3>) -> Bounds<3> {
        let h = s.map_or(0.1, |s| s.h);
        Bounds::around([self.x, self.y, self.z], [h, h * 0.8, h * 1.2])
    }
}

const GROW: f32 = 0.05;
/// The script's margin, which its motions are sized against: declared by
/// its relations, over the engine's default.
const MARGIN: f32 = 0.1;

/// The script's relation in 2D: untagged rows active, tagged passive.
struct Near2;
impl Proximity for Near2 {
    type Key = At;
    type Active = (With<At>, Without<Tag>);
    type Passive = With<(At, Tag)>;
    const GROW: f32 = GROW;
    const MARGIN: f32 = MARGIN;
}

/// The same in 3D.
struct Near3;
impl Proximity for Near3 {
    type Key = At3;
    type Active = (With<At3>, Without<Tag>);
    type Passive = With<(At3, Tag)>;
    const GROW: f32 = GROW;
    const MARGIN: f32 = MARGIN;
}

/// `native` rows, or `miri` under Miri, where each costs a thousand times
/// more.
const fn sized(native: usize, miri: usize) -> usize {
    if cfg!(miri) { miri } else { native }
}

/// How far rows are spread in 2D and 3D: sized with them, so Miri's fewer
/// are about as dense.
const PLANE: f32 = if cfg!(miri) { 6.0 } else { 30.0 };
const SPACE: f32 = if cfg!(miri) { 4.0 } else { 12.0 };

fn lcg(s: &mut u64) -> f32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 40) as f32 / (1u64 << 24) as f32
}

/// Every pair whose boxes, grown, meet, but for pairs of two tagged rows.
fn brute<const D: usize>(mut all: Vec<(Entity, Bounds<D>, bool)>) -> Vec<(Entity, Entity)> {
    all.sort_by_key(|x| x.0);
    let mut out = Vec::new();
    for (i, (a, ba, ta)) in all.iter().enumerate() {
        for (b, bb, tb) in &all[i + 1..] {
            if !(*ta && *tb) && ba.grown(GROW).overlaps(&bb.grown(GROW)) {
                out.push((*a, *b));
            }
        }
    }
    out
}

const REST: u32 = 0;
const CREEP: u32 = 1;
const FLY: u32 = 2;
const FALL: u32 = 3;
const RESIZE: u32 = 4;
const SHAKE: u32 = 5;
/// Nothing writes a key, so nothing re-sorts: only what churn does.
const ALONE: u32 = 6;
/// A row in a hundred nudged, within its fat box: few enough that only
/// their candidates are retested.
const NUDGE: u32 = 7;
/// Every row but the nudged a little, so every candidate is retested and
/// none leaves its fat box.
const WOBBLE: u32 = 8;
/// Every row slid along x by `SLID`.
const SLIDE: u32 = 9;
const SLID: f32 = 0.05;

static SCENE: AtomicU32 = AtomicU32::new(REST);
static FRAME: AtomicU32 = AtomicU32::new(0);
/// Tests share the statics, so run one at a time.
static SERIAL: Mutex<()> = Mutex::new(());

/// How far entity `i` moves this frame, if at all: creeping a hair (about
/// a row's margin over tens of frames), a few flying, everything falling
/// or shaken further than any margin.
fn delta<const N: usize>(i: u32) -> Option<[f32; N]> {
    let (scene, frame) = (SCENE.load(Ordering::SeqCst), FRAME.load(Ordering::SeqCst));
    let mut s = (i as u64) << 32 | frame as u64;
    lcg(&mut s);
    let mut by = |r: f32| std::array::from_fn(|_| (lcg(&mut s) - 0.5) * r);
    match scene {
        CREEP => Some(by(0.006)),
        // Back and forth, never further than a margin from where it was.
        WOBBLE if !i.is_multiple_of(100) => Some([0.06; N]),
        NUDGE if i.is_multiple_of(100) => Some(std::array::from_fn(|_| if frame % 2 == 0 { 0.09 } else { -0.09 })),
        FLY if i % 50 == frame % 50 => Some(by(6.0)),
        SHAKE => Some(by(0.5)),
        FALL => Some(std::array::from_fn(|a| if a == 1 { -0.4 } else { 0.0 })),
        SLIDE => Some(std::array::from_fn(|a| if a == 0 { SLID } else { 0.0 })),
        _ => None,
    }
}

/// Whether entity `i`'s extent changes this frame, and by how much.
fn resized(i: u32) -> Option<f32> {
    let frame = FRAME.load(Ordering::SeqCst);
    (SCENE.load(Ordering::SeqCst) == RESIZE && i % 20 == frame % 20).then_some(if i.is_multiple_of(2) { 1.6 } else { 0.6 })
}

type Found = (Vec<(Entity, Entity)>, Vec<(Entity, Entity)>, LiveStats);

static FOUND: Mutex<Option<Found>> = Mutex::new(None);

/// The live pairs, checked, and `near_pairs` afresh over the system's own
/// queries of the relation's sides, into `slot`.
fn found<R: Proximity>(live: &mut Live<'_, R>, act: &impl NearSide, pas: &impl NearSide, slot: &Mutex<Option<Found>>) {
    let got = live.pairs().to_vec();
    live.check().unwrap_or_else(|e| panic!("{}, {:?}: {e}", std::any::type_name::<R>(), live.stats()));
    *slot.lock().unwrap() = Some((got, near_pairs(act, pas, R::GROW), live.stats()));
}

fn move2(_: &mut Cx, mut q: Query<&mut At>) {
    q.for_each(|row, mut at| {
        if let Some([x, y]) = delta(row.entity().index) {
            (at.x, at.y) = (at.x + x, at.y + y);
        }
    });
}

fn resize2(_: &mut Cx, mut q: Query<&mut Size>) {
    q.for_each(|row, mut s| {
        if let Some(k) = resized(row.entity().index) {
            (s.hx, s.hy) = ((s.hx * k).clamp(0.05, 0.8), (s.hy * k).clamp(0.05, 0.8));
        }
    });
}

fn pairs2(_: &mut Cx, (act, pas): (Query<&At, Without<Tag>>, Query<&At, With<Tag>>), mut live: Live<Near2>) {
    found(&mut live, &act, &pas, &FOUND);
}

fn move3(_: &mut Cx, mut q: Query<&mut At3>) {
    q.for_each(|row, mut at| {
        if let Some([x, y, z]) = delta(row.entity().index) {
            (at.x, at.y, at.z) = (at.x + x, at.y + y, at.z + z);
        }
    });
}

fn resize3(_: &mut Cx, mut q: Query<&mut Size3>) {
    q.for_each(|row, mut s| {
        if let Some(k) = resized(row.entity().index) {
            s.h = (s.h * k).clamp(0.05, 0.8);
        }
    });
}

fn pairs3(_: &mut Cx, (act, pas): (Query<&At3, Without<Tag>>, Query<&At3, With<Tag>>), mut live: Live<Near3>) {
    found(&mut live, &act, &pas, &FOUND);
}

/// Every row's box, and whether it's tagged.
fn boxes2(w: &World) -> Vec<(Entity, Bounds, bool)> {
    let sizes: std::collections::HashMap<Entity, Size> = w.values::<Size>().unwrap_or_default().into_iter().collect();
    let tags: std::collections::HashSet<Entity> = w.values::<Tag>().unwrap_or_default().into_iter().map(|(e, _)| e).collect();
    w.values::<At>().unwrap_or_default().into_iter().map(|(e, at)| (e, at.bounds(sizes.get(&e)), tags.contains(&e))).collect()
}

fn boxes3(w: &World) -> Vec<(Entity, Bounds<3>, bool)> {
    let sizes: std::collections::HashMap<Entity, Size3> = w.values::<Size3>().unwrap_or_default().into_iter().collect();
    let tags: std::collections::HashSet<Entity> = w.values::<Tag>().unwrap_or_default().into_iter().map(|(e, _)| e).collect();
    w.values::<At3>().unwrap_or_default().into_iter().map(|(e, at)| (e, at.bounds(sizes.get(&e)), tags.contains(&e))).collect()
}

/// What differs between the dimensions: spawning, a row's extent, boxes,
/// and the systems.
trait Space {
    fn spawn(m: &mut engine_ecs::WorldMut<'_>, seed: &mut u64, i: usize) -> Entity;
    fn unsize(m: &mut engine_ecs::WorldMut<'_>, e: Entity);
    fn size(m: &mut engine_ecs::WorldMut<'_>, e: Entity);
    fn unkey(m: &mut engine_ecs::WorldMut<'_>, e: Entity);
    fn key(m: &mut engine_ecs::WorldMut<'_>, e: Entity);
    fn found_brute(w: &World) -> Vec<(Entity, Entity)>;
    fn schedule(w: &World) -> Schedule;
    fn alone(w: &World) -> Schedule;
}

struct Plane;
struct Space3;

impl Space for Plane {
    fn spawn(m: &mut engine_ecs::WorldMut<'_>, seed: &mut u64, i: usize) -> Entity {
        let at = At { x: lcg(seed) * PLANE, y: lcg(seed) * PLANE };
        // Every twentieth big, a pass of the order of its own.
        let size = if i % 20 == 1 { Size { hx: 3.0, hy: 0.5 } } else { Size { hx: 0.1 + lcg(seed) * 0.5, hy: 0.1 + lcg(seed) * 0.5 } };
        let tag = Tag { n: i as u32 };
        match i % 7 {
            0 => m.spawn((at,)),
            1 => m.spawn((at, tag)),
            2 | 3 => m.spawn((at, size, tag)),
            _ => m.spawn((at, size)),
        }
    }
    fn unsize(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.remove::<Size>(e);
    }
    fn size(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.insert(e, Size { hx: 0.3, hy: 0.4 });
    }
    fn unkey(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.remove::<At>(e);
    }
    fn key(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.insert(e, At { x: PLANE / 2.0, y: PLANE / 2.0 });
    }
    fn found_brute(w: &World) -> Vec<(Entity, Entity)> {
        brute(boxes2(w))
    }
    fn schedule(w: &World) -> Schedule {
        Schedule { systems: vec![move2.system(w, "move"), resize2.system(w, "resize"), pairs2.system(w, "pairs")] }
    }
    fn alone(w: &World) -> Schedule {
        Schedule { systems: vec![pairs2.system(w, "pairs")] }
    }
}

impl Space for Space3 {
    fn spawn(m: &mut engine_ecs::WorldMut<'_>, seed: &mut u64, i: usize) -> Entity {
        let at = At3 { x: lcg(seed) * SPACE, y: lcg(seed) * SPACE, z: lcg(seed) * SPACE };
        let size = Size3 { h: if i % 20 == 1 { 3.0 } else { 0.1 + lcg(seed) * 0.5 } };
        let tag = Tag { n: i as u32 };
        match i % 7 {
            0 => m.spawn((at,)),
            1 => m.spawn((at, tag)),
            2 | 3 => m.spawn((at, size, tag)),
            _ => m.spawn((at, size)),
        }
    }
    fn unsize(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.remove::<Size3>(e);
    }
    fn size(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.insert(e, Size3 { h: 0.35 });
    }
    fn unkey(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.remove::<At3>(e);
    }
    fn key(m: &mut engine_ecs::WorldMut<'_>, e: Entity) {
        m.insert(e, At3 { x: SPACE / 2.0, y: SPACE / 2.0, z: SPACE / 2.0 });
    }
    fn found_brute(w: &World) -> Vec<(Entity, Entity)> {
        brute(boxes3(w))
    }
    fn schedule(w: &World) -> Schedule {
        Schedule { systems: vec![move3.system(w, "move"), resize3.system(w, "resize"), pairs3.system(w, "pairs")] }
    }
    fn alone(w: &World) -> Schedule {
        Schedule { systems: vec![pairs3.system(w, "pairs")] }
    }
}

/// Between frames: a few rows despawned and as many spawned (reusing their
/// indices), some tagged and untagged (moving them between the sides), one
/// losing its extent and one gaining it (moving it between tables).
fn churn<S: Space>(w: &World, es: &mut Vec<Entity>, keyless: &mut Vec<Entity>, seed: &mut u64, only_despawn: bool) {
    let mut m = w.between_frames(Build::default()).unwrap();
    let pick = |es: &Vec<Entity>, seed: &mut u64| (lcg(seed) * es.len() as f32) as usize % es.len();
    if only_despawn {
        // Only despawned, which marks no table for a re-sort.
        for _ in 0..2 {
            let i = pick(es, seed);
            m.despawn(es.swap_remove(i));
        }
        return;
    }
    for _ in 0..3 {
        let i = pick(es, seed);
        m.despawn(es.swap_remove(i));
    }
    // One fewer: an index left free, whose row is gone for good.
    for k in 0..2 {
        es.push(S::spawn(&mut m, seed, 2 + k));
    }
    for _ in 0..2 {
        let e = es[pick(es, seed)];
        if m.get::<Tag>(e).is_some() {
            m.remove::<Tag>(e);
        } else {
            m.insert(e, Tag { n: 0 });
        }
    }
    let e = es[pick(es, seed)];
    S::unsize(&mut m, e);
    // One out of the spatial tables, keyless, and one back.
    let i = pick(es, seed);
    let e = es.swap_remove(i);
    S::unkey(&mut m, e);
    if let Some(e) = keyless.pop() {
        S::key(&mut m, e);
        es.push(e);
    }
    keyless.insert(0, e);
    let e = es[pick(es, seed)];
    S::size(&mut m, e);
}

/// Runs the script: each frame a motion, churn before it or not, and how
/// the relation must have found its pairs (None: either way).
fn run<S: Space>(rows: usize) {
    let _s = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let w = World::new();
    let mut seed = 7;
    let mut es: Vec<Entity> = {
        let mut m = w.between_frames(Build::default()).unwrap();
        (0..rows).map(|i| S::spawn(&mut m, &mut seed, i)).collect()
    };
    let (s, alone) = (S::schedule(&w), S::alone(&w));
    let mut keyless = Vec::new();
    use How::*;
    let mut script: Vec<(u32, bool, Option<How>)> =
        vec![(REST, false, Some(Afresh)), (REST, false, Some(Rebuilt)), (REST, false, Some(Same))];
    // Wobbling retests every candidate; nudging after it retests only the
    // nudged rows', from what wobbling left.
    script.extend([
        (NUDGE, false, Some(Few)),
        (NUDGE, false, Some(Few)),
        (WOBBLE, false, Some(Kept)),
        (NUDGE, false, Some(Few)),
        (NUDGE, false, Some(Few)),
        (NUDGE, false, Some(Few)),
        (NUDGE, false, Some(Few)),
        (NUDGE, false, Some(Few)),
    ]);
    script.extend([(CREEP, false, Some(Kept)); 10]);
    script.push((REST, false, Some(Same)));
    // The first two may take rows that crept to their fat boxes' edge out.
    script.extend([(NUDGE, false, None); 2]);
    script.extend([(NUDGE, false, Some(Few)); 12]);
    script.extend([(FLY, false, Some(Kept)); 4]);
    script.extend([(RESIZE, false, Some(Kept)); 3]);
    script.extend([(REST, true, Some(Kept)); 4]);
    script.extend([(ALONE, true, Some(Kept)); 3]);
    // Too much moving, each call afresh; then, since it keeps on, it waits
    // one call and then two before looking again (`Pairs::skip`).
    script.extend([(FALL, false, Some(Afresh)); 3]);
    script.extend([(REST, false, Some(Afresh)), (REST, false, Some(Afresh)), (REST, false, Some(Rebuilt)), (REST, false, Some(Same))]);
    script.extend([(SHAKE, false, Some(Afresh)), (REST, false, Some(Afresh)), (REST, false, Some(Rebuilt))]);
    script.extend([(CREEP, true, Some(Kept)); 6]);
    script.extend([(REST, false, Some(Same)), (SHAKE, true, None), (CREEP, false, None), (REST, false, None)]);
    let (mut moved, mut nudged) = (0, std::collections::HashSet::new());
    // Under Miri, the start and the frames that churn: what drives the
    // storage's unsafe glue (spawns, despawns, moves between tables).
    if cfg!(miri) {
        let mut frame = 0;
        script.retain(|&(_, churned, _)| {
            frame += 1;
            frame <= 3 || churned
        });
    }
    for (frame, &(scene, churned, how)) in script.iter().enumerate() {
        if churned {
            churn::<S>(&w, &mut es, &mut keyless, &mut seed, scene == ALONE);
        }
        SCENE.store(scene, Ordering::SeqCst);
        FRAME.store(frame as u32, Ordering::SeqCst);
        let _ = if scene == ALONE { alone.run_sequential(&w) } else { s.run_sequential(&w) };
        let (got, fresh, stats) = FOUND.lock().unwrap().take().expect("the pairs system ran");
        // What the script says about how, and how many, is for its full
        // size: under Miri, fewer rows cross the thresholds differently.
        let full = !cfg!(miri);
        assert!(fresh.len() > rows / 3 || !full, "frame {frame}: {} pairs; too sparse to see much", fresh.len());
        assert_eq!(fresh, S::found_brute(&w), "frame {frame}: near_pairs against brute force");
        assert!(got == fresh, "frame {frame} (scene {scene}, {stats:?}): live pairs aren't near_pairs'");
        if let Some(how) = how.filter(|_| full) {
            assert_eq!(stats.how, how, "frame {frame} (scene {scene}): {stats:?}");
        }
        if scene == CREEP {
            moved += stats.moved;
        }
        if scene == NUDGE {
            nudged.insert(got);
        }
    }
    // Some nudge makes a pair or ends one, so retesting a few is seen to.
    if cfg!(miri) {
        return;
    }
    assert!(nudged.len() > 1, "nudging changed no pair");
    // Creeping, few rows leave their fat boxes: that's what keeping is for.
    assert!(moved > 0 && moved < rows, "{moved} rows left their fat boxes creeping, of {rows} over 16 frames");
}

#[test]
fn live_pairs_are_near_pairs_in_2d() {
    run::<Plane>(sized(600, 24));
}

#[test]
fn live_pairs_are_near_pairs_in_3d() {
    run::<Space3>(sized(600, 24));
}

/// The script's relation read the other way (tagged rows active), wider,
/// at the engine's margin: another relation on `At`.
struct Tagged;
impl Proximity for Tagged {
    type Key = At;
    type Active = With<(At, Tag)>;
    type Passive = (With<At>, Without<Tag>);
    const GROW: f32 = 0.2;
}

/// A relation on the 3D key, for the footprint.
struct Far3;
impl Proximity for Far3 {
    type Key = At3;
    type Active = With<At3>;
    type Passive = AnyOf<()>;
    const GROW: f32 = GROW;
}

/// A system's refusal, as it panics from `IntoSystem::system`.
fn refusal(f: impl FnOnce()) -> String {
    let err = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).expect_err("refused");
    err.downcast_ref::<String>().cloned().unwrap_or_default()
}

/// A relation is written by whoever takes it, and reads its key: two
/// systems that take one relation are ordered, one that only reads the key
/// isn't held up by one that takes it, another relation on the same key or
/// another key is apart, and a writer of the key waits for it.
#[test]
fn live_relations_are_a_footprint_the_scheduler_sees() {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        m.spawn((At { x: 1.0, y: 1.0 },));
        m.spawn((At { x: 1.0, y: 1.0 }, Tag { n: 0 }));
        m.spawn((At3 { x: 1.0, y: 1.0, z: 1.0 },));
    }
    fn first(_: &mut Cx, mut live: Live<Near2>) {
        live.pairs();
    }
    fn second(_: &mut Cx, mut live: Live<Near2>) {
        live.pairs();
    }
    fn reader(_: &mut Cx, q: Query<&At>) {
        let _ = near_pairs(&q, &(), GROW);
    }
    fn same_key(_: &mut Cx, mut live: Live<Tagged>) {
        live.pairs();
    }
    fn other_key(_: &mut Cx, mut live: Live<Far3>) {
        live.pairs();
    }
    fn writer(_: &mut Cx, mut q: Query<&mut At, Without<Tag>>) {
        q.for_each(|_, mut at| at.x += 0.0);
    }
    let systems = vec![
        first.system(&w, "first"),
        reader.system(&w, "reader"),
        same_key.system(&w, "same_key"),
        other_key.system(&w, "other_key"),
        second.system(&w, "second"),
        writer.system(&w, "writer"),
    ];
    let s = Schedule { systems };
    let blockers = |name: &str| {
        let fs = s.frame();
        let i = fs.nodes.iter().position(|&n| s.node_name(n) == name).unwrap();
        s.blockers(&w, &fs, i)
    };
    assert_eq!(blockers("second"), ["first"]);
    assert!(blockers("reader").is_empty(), "{:?}", blockers("reader"));
    assert!(blockers("same_key").is_empty(), "{:?}", blockers("same_key"));
    assert!(blockers("other_key").is_empty(), "{:?}", blockers("other_key"));
    // Its writes are the untagged rows', which both relations on At read.
    let written = blockers("writer");
    for taker in ["first", "same_key", "second"] {
        assert!(written.iter().any(|b| b == taker), "{written:?}");
    }
    assert!(!written.iter().any(|b| b == "other_key"), "{written:?}");
    let _ = s.run_sequential(&w);
}

/// What used to go wrong in silence, or mid-frame, is refused when the
/// system is made: a side filtered by a sparse component, a filter that
/// doesn't require the key, a key that isn't spatial, one relation taken
/// twice, and a relation beside a query that writes its key.
#[test]
fn live_relations_refuse_what_they_cant_keep() {
    let w = World::new();
    struct Sparse;
    impl Proximity for Sparse {
        type Key = At;
        type Active = (With<At>, Without<Tag>);
        type Passive = AnyOf<(With<(At, Tag)>, (With<At>, Without<Mark>))>;
        const GROW: f32 = GROW;
    }
    struct Keyless;
    impl Proximity for Keyless {
        type Key = At;
        type Active = AnyOf<(With<At>, Without<Tag>)>;
        type Passive = AnyOf<()>;
        const GROW: f32 = GROW;
    }
    struct Flat;
    impl Proximity for Flat {
        type Key = Size;
        type Active = With<Size>;
        type Passive = AnyOf<()>;
        const GROW: f32 = GROW;
    }
    fn sparse(_: &mut Cx, _: Live<Sparse>) {}
    fn keyless(_: &mut Cx, _: Live<Keyless>) {}
    fn flat(_: &mut Cx, _: Live<Flat>) {}
    fn twice(_: &mut Cx, _: Live<Near2>, _: (Query<&Size>, Live<Near2>)) {}
    fn two_relations(_: &mut Cx, _: Live<Near2>, _: Live<Tagged>) {}
    fn writes_key(_: &mut Cx, _: Live<Near2>, _: Query<&mut At>) {}
    let err = refusal(|| drop(sparse.system(&w, "sparse")));
    assert!(err.contains("Passive filter 1 names live::Mark, which is sparse"), "{err}");
    // The first filter of `AnyOf<(With<At>, Without<Tag>)>`, two filters.
    let err = refusal(|| drop(keyless.system(&w, "keyless")));
    assert!(err.contains("Active filter 1 doesn't require live::At"), "{err}");
    let err = refusal(|| drop(flat.system(&w, "flat")));
    assert!(err.contains("key live::Size isn't a spatial key"), "{err}");
    let err = refusal(|| drop(twice.system(&w, "twice")));
    assert!(err.contains("twice: takes live_test::Near2 twice"), "{err}");
    let err = refusal(|| drop(writes_key.system(&w, "writes_key")));
    assert!(err.contains("access live::At and one writes it"), "{err}");
    // Two relations on one key, each once, are fine.
    drop(two_relations.system(&w, "two_relations"));
}

static FOUND_TAGGED: Mutex<Option<Found>> = Mutex::new(None);

fn tagged(_: &mut Cx, (act, pas): (Query<&At, With<Tag>>, Query<&At, Without<Tag>>), mut live: Live<Tagged>) {
    found(&mut live, &act, &pas, &FOUND_TAGGED);
}

/// Two relations on one key, with other sides, grow and margin, keep a set
/// each: called in turn in a frame, alone on frames of their own, through
/// motion and churn, each answers its own sides as `near_pairs` does and
/// keeps what it keeps. Were they one set, each call would find the other's
/// sides and start over: afresh every time.
#[test]
fn two_relations_on_one_key_are_kept_apart() {
    let _s = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let w = World::new();
    let mut seed = 11;
    let rows = sized(600, 24);
    let mut es: Vec<Entity> = {
        let mut m = w.between_frames(Build::default()).unwrap();
        (0..rows).map(|i| Plane::spawn(&mut m, &mut seed, i)).collect()
    };
    let both = Schedule { systems: vec![move2.system(&w, "move"), pairs2.system(&w, "near"), tagged.system(&w, "tagged")] };
    let (near, tag) = (Schedule { systems: vec![pairs2.system(&w, "near")] }, Schedule { systems: vec![tagged.system(&w, "tagged")] });
    let mut keyless = Vec::new();
    use How::*;
    // Which run, the scene, churn first or not, and how each must answer.
    let script: Vec<(&Schedule, u32, bool, [Option<How>; 2])> = vec![
        (&both, REST, false, [Some(Afresh), Some(Afresh)]),
        (&both, REST, false, [Some(Rebuilt), Some(Rebuilt)]),
        (&both, REST, false, [Some(Same), Some(Same)]),
        (&near, REST, false, [Some(Same), None]),
        (&tag, REST, false, [None, Some(Same)]),
        (&both, CREEP, false, [Some(Kept), Some(Kept)]),
        (&both, CREEP, true, [Some(Kept), Some(Kept)]),
        // Churn the other doesn't see until its next call, which must.
        (&near, REST, true, [Some(Kept), None]),
        (&tag, REST, false, [None, Some(Kept)]),
        (&both, REST, false, [Some(Same), Some(Same)]),
        (&both, FALL, false, [Some(Afresh), Some(Afresh)]),
        (&both, CREEP, true, [None, None]),
    ];
    for (frame, &(s, scene, churned, how)) in script.iter().enumerate() {
        if churned {
            churn::<Plane>(&w, &mut es, &mut keyless, &mut seed, false);
        }
        SCENE.store(scene, Ordering::SeqCst);
        FRAME.store(frame as u32, Ordering::SeqCst);
        let _ = s.run_sequential(&w);
        for (slot, how) in [(&FOUND, how[0]), (&FOUND_TAGGED, how[1])] {
            let Some((got, fresh, stats)) = slot.lock().unwrap().take() else {
                assert!(how.is_none(), "frame {frame}: a relation that should have run didn't");
                continue;
            };
            assert!(!fresh.is_empty() || cfg!(miri), "frame {frame}: no pairs to see");
            assert!(got == fresh, "frame {frame} ({stats:?}): live pairs aren't near_pairs'");
            if let Some(how) = how.filter(|_| !cfg!(miri)) {
                assert_eq!(stats.how, how, "frame {frame}: {stats:?}");
            }
        }
    }
}

/// The script's relation at the engine's margin, which is less than
/// `SLIDE`'s step: its rows leave their fat boxes where `Near2`'s don't.
struct Tight;
impl Proximity for Tight {
    type Key = At;
    type Active = (With<At>, Without<Tag>);
    type Passive = With<(At, Tag)>;
    const GROW: f32 = GROW;
}

static FOUND_TIGHT: Mutex<Option<Found>> = Mutex::new(None);

fn tight(_: &mut Cx, (act, pas): (Query<&At, Without<Tag>>, Query<&At, With<Tag>>), mut live: Live<Tight>) {
    found(&mut live, &act, &pas, &FOUND_TIGHT);
}

/// A relation's declared margin is its fat boxes': every row slid by more
/// than the default and less than `Near2`'s leaves every fat box of the
/// relation at the default, and none of `Near2`'s; and each checks its
/// fat boxes within twice its own margin.
#[test]
fn a_declared_margin_is_the_one_kept() {
    let _s = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    const { assert!(engine_ecs::live::MARGIN < SLID && SLID < MARGIN) };
    let w = World::new();
    let mut seed = 5;
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for i in 0..sized(600, 24) {
            Plane::spawn(&mut m, &mut seed, i);
        }
    }
    let s = Schedule { systems: vec![move2.system(&w, "move"), pairs2.system(&w, "near"), tight.system(&w, "tight")] };
    for (frame, scene) in [REST, REST, SLIDE].into_iter().enumerate() {
        SCENE.store(scene, Ordering::SeqCst);
        FRAME.store(frame as u32, Ordering::SeqCst);
        let _ = s.run_sequential(&w);
    }
    let (wide, narrow) = (FOUND.lock().unwrap().take().unwrap(), FOUND_TIGHT.lock().unwrap().take().unwrap());
    assert!(wide.0 == wide.1 && narrow.0 == narrow.1, "live pairs aren't near_pairs'");
    let rows = w.values::<At>().unwrap().len();
    assert_eq!((wide.2.moved, wide.2.how), (0, How::Kept), "{:?}", wide.2);
    assert_eq!(narrow.2.moved, rows, "{:?}", narrow.2);
}

/// A table with two spatial keys is in the order of one: a relation on the
/// other whose side matches it can't be kept, and says so at the call,
/// naming the relation (registration can't see tables made later).
#[test]
fn a_side_in_another_keys_order_panics_naming_the_relation() {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        // `At` first, so it has the lesser id, whose order a table with both is in.
        m.spawn((At { x: 1.0, y: 1.0 },));
        m.spawn((At { x: 1.0, y: 1.0 }, At3 { x: 1.0, y: 1.0, z: 1.0 }));
    }
    fn far(_: &mut Cx, mut live: Live<Far3>) {
        live.pairs();
    }
    let s = Schedule { systems: vec![far.system(&w, "far")] };
    let err = refusal(|| {
        let _ = s.run_sequential(&w);
    });
    assert!(err.contains("live_test::Far3 is live::At3's, and a table of its sides is in another key's order"), "{err}");
}
