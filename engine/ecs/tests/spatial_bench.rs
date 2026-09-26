//! The re-sort after writers (every row moved a hair, and 1% moved), and
//! `near_pairs` and `in_region` on a pile-like layout, without physics:
//! `./bazel run -c opt //engine/ecs:spatial_bench`.

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Query, SpatialKey, With, Without, World, component, near_pairs};

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

component! {
    /// A wall: the pile's floor and sides, big rows in a table of their own.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Wall: "bench::Wall" {}
}

component! {
    /// At rest: a body on the passive side of a broadphase, as sleeping
    /// ones are physics's.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Still: "bench::Still" {}
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
    let rebounded: usize = w.tables().filter_map(|t| t.spatial.as_ref()).map(|s| s.pages.read().unwrap().rebounded()).sum();
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

static SIDES: Mutex<[(u128, usize); 3]> = Mutex::new([(0, 0); 3]);

/// The broadphase three ways over one pile with walls: one query over
/// everything; walls passive; and everything passive but the bodies not
/// `Still` (the top rows, falling onto a sleeping pile).
#[allow(clippy::type_complexity)]
fn sides(
    _: &mut Cx,
    mut all: Query<&At>,
    (moving, walls): (Query<&At, Without<(Wall, Still)>>, Query<&At, With<Wall>>),
    still: Query<&At, With<Still>>,
) {
    let mut out = SIDES.lock().unwrap();
    let t = Instant::now();
    let n = all.near_pairs(0.05).len();
    out[0] = (out[0].0 + t.elapsed().as_nanos(), n);
    let t = Instant::now();
    let n = near_pairs(&(&moving, &still), &walls, 0.05).len();
    out[1] = (out[1].0 + t.elapsed().as_nanos(), n);
    let t = Instant::now();
    let n = near_pairs(&moving, &(&walls, &still), 0.05).len();
    out[2] = (out[2].0 + t.elapsed().as_nanos(), n);
}

fn sides_frame(n: usize, awake: usize) {
    let w = World::new();
    let per_row = (n as f32).sqrt() as usize * 4 / 3;
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        for k in 0..n {
            let (col, row) = (k % per_row, k / per_row);
            let at = At { x: 0.5 + col as f32 * 0.9, y: 30.0 - row as f32 * 0.9 };
            if k + awake >= n {
                m.spawn((at, Size { hx: 0.45, hy: 0.45 }));
            } else {
                m.spawn((at, Size { hx: 0.45, hy: 0.45 }, Still {}));
            }
        }
        let width = per_row as f32 * 0.9;
        m.spawn((At { x: width / 2.0, y: 31.0 }, Size { hx: width / 2.0 + 1.0, hy: 0.5 }, Wall {}));
        m.spawn((At { x: -0.5, y: 15.0 }, Size { hx: 0.5, hy: 30.0 }, Wall {}));
        m.spawn((At { x: width + 0.5, y: 15.0 }, Size { hx: 0.5, hy: 30.0 }, Wall {}));
    }
    let s = Schedule { systems: vec![sides.system(&w, "sides")] };
    *SIDES.lock().unwrap() = [(0, 0); 3];
    let frames = 200;
    for _ in 0..frames {
        s.run_sequential(&w);
    }
    let out = *SIDES.lock().unwrap();
    let us = |i: usize| out[i].0 as f64 / frames as f64 / 1e3;
    println!(
        "{n:>6} rows and walls: near_pairs {:>7.1} us ({} pairs); walls passive {:>7.1} us ({}); all but {awake} passive {:>7.1} us ({})",
        us(0),
        out[0].1,
        us(1),
        out[1].1,
        us(2),
        out[2].1
    );
}

/// Moves every body down about as far as a falling pile's do a step (a
/// third of a body at the most), each at its own speed, so rows change
/// cells and pages every frame, as they do falling; a body past the floor
/// goes back to the top.
fn faller(_: &mut Cx, mut q: Query<(&Size, &mut Vel, &mut At)>) {
    let t = Instant::now();
    q.for_each(|row, (_, mut v, mut at)| {
        let i = row.entity().index;
        v.y = 0.05 + (i * 7919 % 100) as f32 * 0.0025;
        at.y += v.y;
        if at.y > 30.0 {
            at.y -= 60.0;
        }
    });
    *WRITING.lock().unwrap() += t.elapsed().as_nanos();
}

fn falling_frame(n: usize) {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let per_row = (n as f32).sqrt() as usize * 4;
        for k in 0..n {
            let (col, row) = (k % per_row, k / per_row);
            m.spawn((At { x: 0.5 + col as f32 * 1.2, y: 30.0 - row as f32 * 1.2 }, Size { hx: 0.45, hy: 0.45 }, Vel::default()));
        }
    }
    let s = Schedule { systems: vec![faller.system(&w, "faller")] };
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
    println!("{n:>6} rows falling: {frame:>7.1} us/frame, writing {writing:.1}, the rest (the re-sort) {:.1}", frame - writing);
}

fn main() {
    for n in [1000usize, 10_000] {
        sides_frame(n, n / 50);
    }
    falling_frame(1000);
    falling_frame(10_000);
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
