//! The re-sort after writers (every row moved a hair, and 1% moved), and
//! `near_pairs` and `in_region` on a pile-like layout, without physics:
//! `./bazel run -c opt //engine/ecs:spatial_bench`.

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Query, SpatialKey, World, component};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "bench::At", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size: "bench::Size" { pub hx: f32, pub hy: f32 }
}

impl SpatialKey for At {
    type Extent = Size;
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        let s = size.copied().unwrap_or_default();
        Bounds::around([self.x, self.y], [s.hx, s.hy])
    }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Vel: "bench::Vel" { pub x: f32, pub y: f32 }
}

static OUT: Mutex<(u128, u128, usize)> = Mutex::new((0, 0, 0));

fn probe(_: &mut Cx, mut q: Query<&At>) {
    let t = Instant::now();
    let pairs = q.near_pairs(0.05);
    let near = t.elapsed().as_nanos();
    let t = Instant::now();
    let mut hits = 0;
    for i in 0..64 {
        let (x, y) = ((i * 7 % 40) as f32, (i * 11 % 30) as f32);
        q.in_region(Bounds::around([x, y], [2.0, 2.0]), |_, _| hits += 1);
    }
    let regions = t.elapsed().as_nanos();
    let mut out = OUT.lock().unwrap();
    out.0 += near;
    out.1 += regions;
    out.2 = pairs.len();
}

/// Visits every position mutably and moves one in a hundred: a blanket
/// writer over mostly static things, which change detection makes pay only
/// for what it moved.
fn sparse_writer(_: &mut Cx, mut q: Query<&mut At>) {
    q.for_each(|row, mut at| {
        if row.entity().index % 100 == 0 {
            at.x += 0.01;
        }
    });
}

fn blanket_writer_frame(n: usize) {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let per_row = (n as f32).sqrt() as usize;
        for k in 0..n {
            m.spawn((At { x: (k % per_row) as f32 * 2.0, y: (k / per_row) as f32 * 2.0 }, Size { hx: 0.45, hy: 0.45 }));
        }
    }
    let s = Schedule { systems: vec![sparse_writer.system(&w, "writer")] };
    let frames = 200;
    let t = Instant::now();
    for _ in 0..frames {
        s.run_sequential(&w);
    }
    let rebounded: usize = w.tables().filter_map(|t| t.spatial.as_ref()).map(|s| s.pages.read().unwrap().rebounded).sum();
    println!(
        "{n:>6} rows, writing 1%: {:>7.1} us/frame, the re-sort re-bounding {rebounded} rows",
        t.elapsed().as_secs_f64() * 1e6 / frames as f64
    );
}

static WRITING: Mutex<u128> = Mutex::new(0);

/// Moves every body a hair, as a settled pile's solver does (every body
/// creeps by 1e-4 to 1e-2 a step, measured 2026-09-24): every row is
/// re-bounded, and almost none changes cell or page.
fn creeper(_: &mut Cx, mut q: Query<(&Size, &mut Vel, &mut At)>) {
    let t = Instant::now();
    q.for_each(|row, (_, mut v, mut at)| {
        let d = if row.entity().index % 2 == 0 { 1e-3 } else { -1e-3 };
        v.x = d;
        at.x += d;
    });
    *WRITING.lock().unwrap() += t.elapsed().as_nanos();
}

fn creeping_frame(n: usize) {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let per_row = (n as f32).sqrt() as usize * 4 / 3;
        for k in 0..n {
            let (col, row) = (k % per_row, k / per_row);
            m.spawn((At { x: 0.5 + col as f32 * 0.9, y: 30.0 - row as f32 * 0.9 }, Size { hx: 0.45, hy: 0.45 }, Vel::default()));
        }
    }
    let s = Schedule { systems: vec![creeper.system(&w, "creeper")] };
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
    println!("{n:>6} rows creeping: {frame:>7.1} us/frame, writing {writing:.1}, the rest (the re-sort) {:.1}", frame - writing);
}

fn main() {
    creeping_frame(1000);
    creeping_frame(10_000);
    blanket_writer_frame(10_000);
    for n in [1000usize, 10_000] {
        let w = World::new();
        {
            let mut m = w.between_frames(Build::default()).unwrap();
            // A settled pile: rows of touching 0.9 bodies, as physics's is.
            let per_row = (n as f32).sqrt() as usize * 4 / 3;
            for k in 0..n {
                let (col, row) = (k % per_row, k / per_row);
                m.spawn((At { x: 0.5 + col as f32 * 0.9, y: 30.0 - row as f32 * 0.9 }, Size { hx: 0.45, hy: 0.45 }));
            }
        }
        let s = Schedule { systems: vec![probe.system(&w, "probe")] };
        *OUT.lock().unwrap() = (0, 0, 0);
        let frames = 200;
        for _ in 0..frames {
            s.run_sequential(&w);
        }
        let (near, regions, pairs) = *OUT.lock().unwrap();
        let t = w.tables().find(|t| t.spatial.is_some()).unwrap();
        let rows = t.rows.read().unwrap();
        let used = rows.iter().filter(|p| !p.is_empty()).count();
        println!(
            "{n:>6} rows: near_pairs {:>7.1} us ({pairs} pairs), 64 regions {:>6.1} us; {used} pages in use of {}",
            near as f64 / frames as f64 / 1e3,
            regions as f64 / frames as f64 / 1e3,
            rows.len()
        );
    }
}
