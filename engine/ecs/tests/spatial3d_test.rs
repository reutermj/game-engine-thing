//! Spatial tables in 3D (docs/architecture/spatial-storage.md, "In 3D"):
//! region queries and pairs agree with brute force after random moves,
//! spawns, despawns and a move of everything at once; the order's own
//! checks hold; and a 2D and a 3D key live in one world, each query seeing
//! only its own dimension's tables.

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Entity, Query, SpatialKey, World, component, near_pairs};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At3: "test::At3", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size3: "test::Size3" { pub h: f32 }
}

component! {
    /// A key in 2D beside the 3D one.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At2: "test::At2", order = spatial { pub x: f32, pub y: f32 }
}

impl SpatialKey<3> for At3 {
    type Extent = Size3;
    fn bounds(&self, s: Option<&Size3>) -> Bounds<3> {
        let h = s.map_or(0.1, |s| s.h);
        Bounds::around([self.x, self.y, self.z], [h, h * 0.8, h * 1.2])
    }
}

impl SpatialKey for At2 {
    type Extent = At2;
    fn bounds(&self, _: Option<&At2>) -> Bounds {
        Bounds::around([self.x, self.y], [0.3, 0.3])
    }
}

fn lcg(s: &mut u64) -> f32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 40) as f32 / (1u64 << 24) as f32
}

fn boxes(w: &World) -> Vec<(Entity, Bounds<3>)> {
    let sizes: std::collections::HashMap<Entity, Size3> = w.values::<Size3>().unwrap_or_default().into_iter().collect();
    let mut out: Vec<_> = w.values::<At3>().unwrap_or_default().into_iter().map(|(e, at)| (e, at.bounds(sizes.get(&e)))).collect();
    out.sort_by_key(|(e, _)| *e);
    out
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

fn check(w: &World) {
    for t in w.tables() {
        let Some(spatial) = &t.spatial else { continue };
        let order = spatial.pages.read().unwrap();
        order.check(&t.rows.read().unwrap(), 2.0, 1.0).unwrap_or_else(|e| panic!("table {:?}: {e}", t.id));
    }
}

/// What the probe system found, and what brute force says after.
static FOUND: std::sync::Mutex<Vec<Vec<Entity>>> = std::sync::Mutex::new(Vec::new());
static PAIRS: std::sync::Mutex<Vec<(Entity, Entity)>> = std::sync::Mutex::new(Vec::new());

fn regions() -> Vec<Bounds<3>> {
    (0..24).map(|i| Bounds::around([(i * 7 % 12) as f32, (i * 5 % 9) as f32, (i * 3 % 10) as f32], [1.5, 1.0, 2.0])).collect()
}

fn probe(_: &mut Cx, mut q: Query<&At3>) {
    let mut found = Vec::new();
    for r in regions() {
        let mut hit = Vec::new();
        q.in_region(r, |row, _| hit.push(row.entity()));
        hit.sort_unstable();
        found.push(hit);
    }
    *FOUND.lock().unwrap() = found;
    *PAIRS.lock().unwrap() = near_pairs(&q, &(), 0.05);
}

fn mover(_: &mut Cx, mut q: Query<&mut At3>) {
    let mut s = 7u64;
    q.for_each(|row, mut at| {
        // A third move each frame, some a long way: across pages and cells.
        if row.entity().index % 3 == (lcg(&mut s) * 3.0) as u32 {
            let far = if lcg(&mut s) < 0.1 { 6.0 } else { 0.4 };
            at.x += (lcg(&mut s) - 0.5) * far;
            at.y += (lcg(&mut s) - 0.5) * far;
            at.z += (lcg(&mut s) - 0.5) * far;
        }
    });
}

fn everything(_: &mut Cx, mut q: Query<&mut At3>) {
    q.for_each(|_, mut at| at.z = -at.z + 3.0);
}

#[test]
fn regions_and_pairs_agree_with_brute_force_in_3d() {
    let w = World::new();
    let s = Schedule { systems: vec![mover.system(&w, "mover"), probe.system(&w, "probe")] };
    let all = Schedule { systems: vec![everything.system(&w, "everything"), probe.system(&w, "probe2")] };
    let mut seed = 1u64;
    let mut spawned = Vec::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for k in 0..900 {
            let at = At3 { x: lcg(&mut seed) * 12.0, y: lcg(&mut seed) * 9.0, z: lcg(&mut seed) * 10.0 };
            // Mostly small, a few big (their own pages), some with no size.
            let e = match k % 30 {
                0 => m.spawn((at, Size3 { h: 3.0 })),
                1..=5 => m.spawn((at,)),
                _ => m.spawn((at, Size3 { h: 0.2 + lcg(&mut seed) * 0.3 })),
            };
            spawned.push(e);
        }
    }
    check(&w);
    for frame in 0..40 {
        let schedule = if frame % 10 == 9 { &all } else { &s };
        schedule.run_sequential(&w);
        check(&w);
        let now = boxes(&w);
        let found = FOUND.lock().unwrap().clone();
        for (r, hit) in regions().iter().zip(found) {
            let want: Vec<Entity> = now.iter().filter(|(_, b)| b.overlaps(r)).map(|(e, _)| *e).collect();
            assert_eq!(hit, want, "frame {frame}, region {r:?}");
        }
        assert_eq!(*PAIRS.lock().unwrap(), brute_pairs(&w, 0.05), "frame {frame}");
        if frame % 7 == 3 {
            let mut m = w.between_frames(Build::default()).unwrap();
            for _ in 0..40 {
                let i = (lcg(&mut seed) * spawned.len() as f32) as usize % spawned.len();
                m.despawn(spawned.swap_remove(i));
            }
            for _ in 0..60 {
                let at = At3 { x: lcg(&mut seed) * 12.0, y: lcg(&mut seed) * 9.0, z: lcg(&mut seed) * 10.0 };
                spawned.push(m.spawn((at, Size3 { h: 0.25 })));
            }
        }
    }
    assert!(w.tables().filter_map(|t| t.spatial.as_ref()).all(|s| s.pages.read().unwrap().dims() == 3));
}

static TWO: std::sync::Mutex<(usize, usize, usize)> = std::sync::Mutex::new((0, 0, 0));

fn both(_: &mut Cx, mut flat: Query<&At2>, mut deep: Query<&At3>) {
    let (mut f, mut d) = (0, 0);
    flat.in_region(Bounds::around([0.0, 0.0], [100.0, 100.0]), |_, _| f += 1);
    deep.in_region(Bounds::around([0.0, 0.0, 0.0], [100.0, 100.0, 100.0]), |_, _| d += 1);
    *TWO.lock().unwrap() = (f, d, near_pairs(&deep, &(), 0.0).len());
}

#[test]
fn a_2d_key_and_a_3d_key_share_a_world() {
    let w = World::new();
    let s = Schedule { systems: vec![both.system(&w, "both")] };
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for i in 0..50 {
            m.spawn((At2 { x: i as f32, y: 0.0 },));
            // Along a line in z, touching their neighbors only.
            m.spawn((At3 { x: 0.0, y: 0.0, z: i as f32 * 0.4 }, Size3 { h: 0.2 }));
        }
    }
    s.run_sequential(&w);
    check(&w);
    assert_eq!(*TWO.lock().unwrap(), (50, 50, 49));
    let dims: Vec<usize> = w.tables().filter_map(|t| t.spatial.as_ref()).map(|s| s.pages.read().unwrap().dims()).collect();
    assert!(dims.contains(&2) && dims.contains(&3), "{dims:?}");
}
