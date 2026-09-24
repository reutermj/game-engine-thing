//! What a query costs per row, by how it's walked: `./bazel run -c opt
//! //engine/ecs:query_bench`. The physics mod's gathers and scatters are
//! these loops, over a spatial table like its bodies' (pages of about a
//! dozen rows) and a plain one (pages of 256), against copying from a
//! `Vec`.

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Entity, Query, SpatialKey, Without, World, component};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "bench::At", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size: "bench::Size" { pub hx: f32, pub hy: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Vel: "bench::Vel" { pub x: f32, pub y: f32 }
}

component! {
    /// As big as physics's `Body`.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Mass: "bench::Mass" { pub kind: u8, pub inv: f32, pub a: f32, pub b: f32, pub c: f32 }
}

impl SpatialKey for At {
    type Extent = Size;
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        let s = size.copied().unwrap_or_default();
        Bounds::around([self.x, self.y], [s.hx, s.hy])
    }
}

/// What a solver keeps per body.
#[derive(Clone, Copy, Default)]
struct Solver {
    v: [f32; 2],
    inv: f32,
}

fn solver(m: &Mass, v: &Vel) -> Solver {
    Solver { v: [v.x, v.y], inv: if m.kind == 0 { m.inv } else { 0.0 } }
}

const STAGES: [&str; 9] = [
    "copy from a Vec (the arrays' gather)",
    "for_each, copying out",
    "for_each_page, copying out",
    "for_each, writing each row (Mut)",
    "for_each_page, writing each row (set)",
    "for_each_page, write_all",
    "a map by entity: built, 2 lookups a pair",
    "256-row pages: for_each, copying out",
    "256-row pages: for_each_page, copying out",
];

/// Nanoseconds per stage, summed over frames, and rows or pairs each covers.
static OUT: Mutex<([u128; 9], [usize; 9])> = Mutex::new(([0; 9], [0; 9]));
static SINK: Mutex<f32> = Mutex::new(0.0);

/// Entities to places in a list, by entity index, as physics maps them.
struct Slots(Vec<(u32, u32)>);

impl Slots {
    fn of(entities: &[Entity]) -> Slots {
        let len = entities.iter().map(|e| e.index as usize + 1).max().unwrap_or(0);
        let mut slots = vec![(u32::MAX, u32::MAX); len];
        for (k, e) in entities.iter().enumerate() {
            slots[e.index as usize] = (e.generation, k as u32);
        }
        Slots(slots)
    }

    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }
}

fn record(stage: usize, since: Instant, n: usize) {
    let mut out = OUT.lock().unwrap();
    out.0[stage] += since.elapsed().as_nanos();
    out.1[stage] = n;
}

fn spatial(_: &mut Cx, mut q: Query<(&Mass, &mut Vel, &At)>, mut shapes: Query<&At>) {
    let n = q.len();
    let source: Vec<(Mass, Vel)> = vec![(Mass::default(), Vel::default()); n];

    let s = Instant::now();
    let mut out: Vec<Solver> = Vec::with_capacity(n);
    out.extend(source.iter().map(|(m, v)| solver(m, v)));
    record(0, s, n);

    let s = Instant::now();
    let mut out: Vec<Solver> = Vec::with_capacity(n);
    q.for_each(|_, (m, v, _)| out.push(solver(m, &v)));
    record(1, s, n);

    let s = Instant::now();
    let mut out: Vec<Solver> = Vec::with_capacity(n);
    q.for_each_page(|page, (m, v, _)| out.extend(page.rows().map(|r| solver(&m[r], &v[r]))));
    record(2, s, n);

    let s = Instant::now();
    let mut i = 0;
    q.for_each(|_, (_, mut v, _)| {
        (v.x, v.y) = (out[i].v[0], out[i].v[1]);
        i += 1;
    });
    record(3, s, n);

    let s = Instant::now();
    let mut i = 0;
    q.for_each_page(|page, (_, mut v, _)| {
        for r in page.rows() {
            v.set(r, Vel { x: out[i].v[0], y: out[i].v[1] });
            i += 1;
        }
    });
    record(4, s, n);

    let s = Instant::now();
    let mut i = 0;
    q.for_each_page(|_, (_, mut v, _)| {
        for v in v.write_all() {
            *v = Vel { x: out[i].v[0], y: out[i].v[1] };
            i += 1;
        }
    });
    record(5, s, n);

    let pairs = shapes.near_pairs(0.05);
    let s = Instant::now();
    let mut entities = Vec::with_capacity(n);
    q.for_each_page(|page, _| entities.extend_from_slice(page.entities()));
    let slots = Slots::of(&entities);
    let mut sum = 0u64;
    for (a, b) in &pairs {
        sum += slots.get(*a).unwrap_or(0) as u64 + slots.get(*b).unwrap_or(0) as u64;
    }
    record(6, s, pairs.len());
    *SINK.lock().unwrap() += (sum % 7) as f32 + out.iter().map(|s| s.inv).sum::<f32>();
}

fn plain(_: &mut Cx, mut q: Query<(&Mass, &mut Vel, &Size), Without<At>>) {
    let n = q.len();
    let s = Instant::now();
    let mut out: Vec<Solver> = Vec::with_capacity(n);
    q.for_each(|_, (m, v, _)| out.push(solver(m, &v)));
    record(7, s, n);

    let s = Instant::now();
    let mut out2: Vec<Solver> = Vec::with_capacity(n);
    q.for_each_page(|page, (m, v, _)| out2.extend(page.rows().map(|r| solver(&m[r], &v[r]))));
    record(8, s, n);
    *SINK.lock().unwrap() += (out.len() + out2.len()) as f32;
}

fn main() {
    for n in [1000usize, 10_000] {
        let w = World::new();
        {
            let mut m = w.between_frames(Build::default()).unwrap();
            // A settled pile's layout: rows of touching bodies.
            let per_row = (n as f32).sqrt() as usize * 4 / 3;
            for k in 0..n {
                let (col, row) = (k % per_row, k / per_row);
                let mass = Mass { kind: 0, inv: 1.0, ..Mass::default() };
                let size = Size { hx: 0.45, hy: 0.45 };
                m.spawn((At { x: 0.5 + col as f32 * 0.9, y: 30.0 - row as f32 * 0.9 }, size, Vel::default(), mass));
                m.spawn((size, Vel::default(), mass));
            }
        }
        let s = Schedule { systems: vec![spatial.system(&w, "spatial"), plain.system(&w, "plain")] };
        *OUT.lock().unwrap() = ([0; 9], [0; 9]);
        let frames = 200;
        for _ in 0..frames {
            s.run_sequential(&w);
        }
        let (ns, per) = *OUT.lock().unwrap();
        println!("{n} rows, ns per row (per pair for lookups):");
        for (i, name) in STAGES.iter().enumerate() {
            println!("  {name:<44} {:>6.2}", ns[i] as f64 / frames as f64 / per[i].max(1) as f64);
        }
    }
    let _ = *SINK.lock().unwrap();
}
