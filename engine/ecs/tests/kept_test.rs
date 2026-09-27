//! A broadphase that keeps its pairs (`engine_ecs::kept`), against
//! `near_pairs` afresh and brute force, in 2D and 3D: at rest, creeping,
//! a few rows flying and everything falling, extents written, rows
//! spawned, despawned (their indices reused) and moved between tables and
//! between the sides. Every frame its pairs are `near_pairs`' exactly, and
//! how it found them is what the motion says it should be.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::kept::{How, KeptStats};
use engine_ecs::{Bounds, Build, Entity, Kept, Query, SpatialKey, With, Without, World, component, near_pairs};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "kept::At", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size: "kept::Size" { pub hx: f32, pub hy: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At3: "kept::At3", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size3: "kept::Size3" { pub h: f32 }
}

component! {
    /// Passive: the rows with it are the broadphase's passive side.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tag: "kept::Tag" { pub n: u32 }
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
const MARGIN: f32 = 0.1;

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
        _ => None,
    }
}

/// Whether entity `i`'s extent changes this frame, and by how much.
fn resized(i: u32) -> Option<f32> {
    let frame = FRAME.load(Ordering::SeqCst);
    (SCENE.load(Ordering::SeqCst) == RESIZE && i % 20 == frame % 20).then_some(if i.is_multiple_of(2) { 1.6 } else { 0.6 })
}

type Found = (Vec<(Entity, Entity)>, Vec<(Entity, Entity)>, KeptStats);

static FOUND: Mutex<Option<Found>> = Mutex::new(None);

/// The kept pairs, and `near_pairs` afresh, between the untagged rows and
/// the tagged ones.
fn found<K>(kept: &mut Kept<'_, K>, act: &impl engine_ecs::NearSide, pas: &impl engine_ecs::NearSide) {
    let got = kept.near_pairs(act, pas, GROW, MARGIN).to_vec();
    kept.check(act, pas).unwrap_or_else(|e| panic!("{:?}: {e}", kept.stats()));
    *FOUND.lock().unwrap() = Some((got, near_pairs(act, pas, GROW), kept.stats()));
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

fn pairs2(_: &mut Cx, (act, pas): (Query<&At, Without<Tag>>, Query<&At, With<Tag>>), mut kept: Kept<At>) {
    found(&mut kept, &act, &pas);
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

fn pairs3(_: &mut Cx, (act, pas): (Query<&At3, Without<Tag>>, Query<&At3, With<Tag>>), mut kept: Kept<At3>) {
    found(&mut kept, &act, &pas);
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
        let at = At { x: lcg(seed) * 30.0, y: lcg(seed) * 30.0 };
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
        m.insert(e, At { x: 15.0, y: 15.0 });
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
        let at = At3 { x: lcg(seed) * 12.0, y: lcg(seed) * 12.0, z: lcg(seed) * 12.0 };
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
        m.insert(e, At3 { x: 6.0, y: 6.0, z: 6.0 });
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
/// the kept broadphase must have found its pairs (None: either way).
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
    for (frame, &(scene, churned, how)) in script.iter().enumerate() {
        if churned {
            churn::<S>(&w, &mut es, &mut keyless, &mut seed, scene == ALONE);
        }
        SCENE.store(scene, Ordering::SeqCst);
        FRAME.store(frame as u32, Ordering::SeqCst);
        let _ = if scene == ALONE { alone.run_sequential(&w) } else { s.run_sequential(&w) };
        let (got, fresh, stats) = FOUND.lock().unwrap().take().expect("the pairs system ran");
        assert!(fresh.len() > rows / 3, "frame {frame}: {} pairs; too sparse to see much", fresh.len());
        assert_eq!(fresh, S::found_brute(&w), "frame {frame}: near_pairs against brute force");
        assert!(got == fresh, "frame {frame} (scene {scene}, {stats:?}): kept pairs aren't near_pairs'");
        if let Some(how) = how {
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
    assert!(nudged.len() > 1, "nudging changed no pair");
    // Creeping, few rows leave their fat boxes: that's what keeping is for.
    assert!(moved > 0 && moved < rows, "{moved} rows left their fat boxes creeping, of {rows} over 16 frames");
}

#[test]
fn kept_pairs_are_near_pairs_in_2d() {
    run::<Plane>(600);
}

#[test]
fn kept_pairs_are_near_pairs_in_3d() {
    run::<Space3>(600);
}

/// Kept pairs are a key's, written by whoever takes them: two systems that
/// take one key's are ordered, one that only reads the key isn't held up by
/// one that takes its pairs, another key's pairs are apart, and a system
/// can't take one key's twice.
#[test]
fn kept_pairs_are_a_footprint_the_scheduler_sees() {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        m.spawn((At { x: 1.0, y: 1.0 },));
        m.spawn((At3 { x: 1.0, y: 1.0, z: 1.0 },));
    }
    fn first(_: &mut Cx, q: Query<&At>, mut kept: Kept<At>) {
        kept.near_pairs(&q, &(), GROW, MARGIN);
    }
    fn second(_: &mut Cx, q: Query<&At>, mut kept: Kept<At>) {
        kept.near_pairs(&q, &(), GROW, MARGIN);
    }
    fn reader(_: &mut Cx, q: Query<&At>) {
        let _ = near_pairs(&q, &(), GROW);
    }
    fn other(_: &mut Cx, q: Query<&At3>, mut kept: Kept<At3>) {
        kept.near_pairs(&q, &(), GROW, MARGIN);
    }
    let s = Schedule {
        systems: vec![first.system(&w, "first"), reader.system(&w, "reader"), other.system(&w, "other"), second.system(&w, "second")],
    };
    let blockers = |name: &str| {
        let fs = s.frame();
        let i = fs.nodes.iter().position(|&n| s.node_name(n) == name).unwrap();
        s.blockers(&w, &fs, i)
    };
    assert_eq!(blockers("second"), ["first"]);
    assert!(blockers("reader").is_empty(), "{:?}", blockers("reader"));
    assert!(blockers("other").is_empty(), "{:?}", blockers("other"));
    let _ = s.run_sequential(&w);
    fn twice(_: &mut Cx, _: Kept<At>, _: Kept<At>) {}
    let err = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| twice.system(&w, "twice"))).err().expect("refused");
    let err = err.downcast_ref::<String>().cloned().unwrap_or_default();
    assert!(err.contains("kept pairs twice"), "{err}");
}
