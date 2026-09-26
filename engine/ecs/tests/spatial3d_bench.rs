//! Spatial storage in 3D against 2D, without physics:
//! `./bazel run -c opt //engine/ecs:spatial3d_bench`. A dense pile (touching
//! bodies on a lattice) in 2D and in 3D, a 2D pile kept through a 3D key at
//! z = 0 (what 2D would cost if storage were always 3D), and the re-sort
//! with every row creeping and falling, each in both dimensions.

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Query, SpatialKey, World, component, near_pairs};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At2: "bench3::At2", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At3: "bench3::At3", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Half: "bench3::Half" { pub h: f32 }
}

impl SpatialKey for At2 {
    type Extent = Half;
    #[inline]
    fn bounds(&self, s: Option<&Half>) -> Bounds {
        let h = s.map_or(0.0, |s| s.h);
        Bounds::around([self.x, self.y], [h, h])
    }
}

impl SpatialKey<3> for At3 {
    type Extent = Half;
    #[inline]
    fn bounds(&self, s: Option<&Half>) -> Bounds<3> {
        let h = s.map_or(0.0, |s| s.h);
        Bounds::around([self.x, self.y, self.z], [h, h, h])
    }
}

/// Nanoseconds in near_pairs and in 64 regions, pairs found.
static OUT: Mutex<(u128, u128, usize)> = Mutex::new((0, 0, 0));

fn probe2(_: &mut Cx, mut q: Query<&At2>) {
    let t = Instant::now();
    let pairs = near_pairs(&q, &(), 0.05).len();
    let near = t.elapsed().as_nanos();
    let t = Instant::now();
    let mut hits = 0;
    for i in 0..64 {
        q.in_region(Bounds::around([(i * 7 % 40) as f32, (i * 11 % 30) as f32], [2.0, 2.0]), |_, _| hits += 1);
    }
    let mut out = OUT.lock().unwrap();
    (out.0, out.1, out.2) = (out.0 + near, out.1 + t.elapsed().as_nanos(), pairs);
}

fn probe3(_: &mut Cx, mut q: Query<&At3>) {
    let t = Instant::now();
    let pairs = near_pairs(&q, &(), 0.05).len();
    let near = t.elapsed().as_nanos();
    let t = Instant::now();
    let mut hits = 0;
    for i in 0..64 {
        // Regions of about the same number of rows as the 2D ones: 2D pages
        // at 4x4 cover about as many rows as a 3D box a little over 2 on a side.
        let at = [(i * 7 % 20) as f32, (i * 11 % 20) as f32, (i * 5 % 20) as f32];
        q.in_region(Bounds::around(at, [1.2, 1.2, 1.2]), |_, _| hits += 1);
    }
    let mut out = OUT.lock().unwrap();
    (out.0, out.1, out.2) = (out.0 + near, out.1 + t.elapsed().as_nanos(), pairs);
}

/// `probe2`'s regions, through the 3D key: the same rows found.
fn probe23(_: &mut Cx, mut q: Query<&At3>) {
    let t = Instant::now();
    let pairs = near_pairs(&q, &(), 0.05).len();
    let near = t.elapsed().as_nanos();
    let t = Instant::now();
    let mut hits = 0;
    for i in 0..64 {
        q.in_region(Bounds::around([(i * 7 % 40) as f32, (i * 11 % 30) as f32, 0.0], [2.0, 2.0, 2.0]), |_, _| hits += 1);
    }
    let mut out = OUT.lock().unwrap();
    (out.0, out.1, out.2) = (out.0 + near, out.1 + t.elapsed().as_nanos(), pairs);
}

/// A lattice of touching bodies: `per` a row, 0.9 apart, half extent 0.45.
fn lattice(n: usize, dims: usize) -> Vec<[f32; 3]> {
    let per = if dims == 2 { (n as f32).sqrt().ceil() as usize } else { (n as f32).cbrt().ceil() as usize };
    (0..n)
        .map(|k| {
            let (a, b, c) = (k % per, (k / per) % per, k / (per * per));
            if dims == 2 { [a as f32 * 0.9, b as f32 * 0.9, 0.0] } else { [a as f32 * 0.9, b as f32 * 0.9, c as f32 * 0.9] }
        })
        .collect()
}

fn pages(w: &World) -> String {
    let t = w.tables().find(|t| t.spatial.is_some()).unwrap();
    let (n, extents) = t.spatial.as_ref().unwrap().pages.read().unwrap().page_sizes();
    let rows = t.len();
    format!("{n} pages, {:.1} rows each, mean extent {extents:.2?}", rows as f32 / n as f32)
}

/// The dense pile, `how`: 2 (2D key), 3 (3D key), or 23 (the 2D layout
/// through the 3D key, at z = 0).
fn dense(n: usize, how: usize) {
    let w = World::new();
    let s = match how {
        2 => Schedule { systems: vec![probe2.system(&w, "probe")] },
        3 => Schedule { systems: vec![probe3.system(&w, "probe")] },
        _ => Schedule { systems: vec![probe23.system(&w, "probe")] },
    };
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for p in lattice(n, if how == 3 { 3 } else { 2 }) {
            if how == 2 {
                m.spawn((At2 { x: p[0], y: p[1] }, Half { h: 0.45 }));
            } else {
                m.spawn((At3 { x: p[0], y: p[1], z: p[2] }, Half { h: 0.45 }));
            }
        }
    }
    *OUT.lock().unwrap() = (0, 0, 0);
    let frames = 100;
    for _ in 0..frames {
        s.run_sequential(&w);
    }
    let (near, regions, pairs) = *OUT.lock().unwrap();
    let name = match how {
        2 => "2D",
        3 => "3D",
        _ => "2D via 3D",
    };
    println!(
        "{n:>6} {name:>9} dense: near_pairs {:>7.1} us ({pairs} pairs, {:.2} a row), 64 regions {:>6.1} us; {}",
        near as f64 / frames as f64 / 1e3,
        pairs as f64 / n as f64,
        regions as f64 / frames as f64 / 1e3,
        pages(&w)
    );
}

static WRITING: Mutex<u128> = Mutex::new(0);

/// Every row a hair each frame (a settled pile creeping), or down about a
/// third of a body (falling), wrapping from the floor to the top.
fn mover2(_: &mut Cx, mut q: Query<&mut At2>) {
    let t = Instant::now();
    let fall = FALL.lock().unwrap().0;
    q.for_each(|row, mut at| {
        let i = row.entity().index;
        if fall {
            at.y -= 0.05 + (i * 7919 % 100) as f32 * 0.0025;
            if at.y < -30.0 {
                at.y += 60.0;
            }
        } else {
            at.x += if i % 2 == 0 { 1e-3 } else { -1e-3 };
        }
    });
    *WRITING.lock().unwrap() += t.elapsed().as_nanos();
}

fn mover3(_: &mut Cx, mut q: Query<&mut At3>) {
    let t = Instant::now();
    let fall = FALL.lock().unwrap().0;
    q.for_each(|row, mut at| {
        let i = row.entity().index;
        if fall {
            at.y -= 0.05 + (i * 7919 % 100) as f32 * 0.0025;
            if at.y < -30.0 {
                at.y += 60.0;
            }
        } else {
            at.x += if i % 2 == 0 { 1e-3 } else { -1e-3 };
        }
    });
    *WRITING.lock().unwrap() += t.elapsed().as_nanos();
}

static FALL: Mutex<(bool,)> = Mutex::new((false,));

fn resort(n: usize, dims: usize, fall: bool) {
    FALL.lock().unwrap().0 = fall;
    let w = World::new();
    let s = if dims == 2 {
        Schedule { systems: vec![mover2.system(&w, "mover")] }
    } else {
        Schedule { systems: vec![mover3.system(&w, "mover")] }
    };
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        // Spread out when falling, so rows really cross cells and pages.
        let gap = if fall { 1.2 / 0.9 } else { 1.0 };
        for p in lattice(n, dims) {
            let p = p.map(|v| v * gap);
            if dims == 2 {
                m.spawn((At2 { x: p[0], y: p[1] }, Half { h: 0.45 }));
            } else {
                m.spawn((At3 { x: p[0], y: p[1], z: p[2] }, Half { h: 0.45 }));
            }
        }
    }
    for _ in 0..10 {
        s.run_sequential(&w);
    }
    *WRITING.lock().unwrap() = 0;
    let frames = 200;
    let t = Instant::now();
    for _ in 0..frames {
        s.run_sequential(&w);
    }
    let frame = t.elapsed().as_secs_f64() * 1e6 / frames as f64;
    let writing = *WRITING.lock().unwrap() as f64 / frames as f64 / 1e3;
    println!(
        "{n:>6} {dims}D {:>8}: {frame:>7.1} us/frame, the re-sort {:.1}; {}",
        if fall { "falling" } else { "creeping" },
        frame - writing,
        pages(&w)
    );
}

fn main() {
    println!("rows a page: {}", engine_ecs::spatial::SPATIAL_PAGE_ROWS);
    for n in [1000usize, 10_000] {
        for how in [2, 23, 3] {
            dense(n, how);
        }
    }
    for n in [1000usize, 10_000] {
        for dims in [2, 3] {
            resort(n, dims, false);
            resort(n, dims, true);
        }
    }
}
