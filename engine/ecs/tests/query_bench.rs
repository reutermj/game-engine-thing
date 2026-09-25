//! What a query costs per row, by how it's walked: `./bazel run -c opt
//! //engine/ecs:query_bench`. The physics mod's gathers and scatters are
//! these loops, over a spatial table like its bodies' (pages of about a
//! dozen rows) and a plain one (pages of 256), against copying from a
//! `Vec`.

use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use std::sync::Arc;

use engine_ecs::{Bounds, Build, Entity, Executor, Query, SpatialKey, Without, Workers, World, component};

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

const STAGES: [&str; 18] = [
    "copy from a Vec (the arrays' gather)",
    "for_each, copying out",
    "for_each_page, copying out",
    "for_each, writing each row (Mut)",
    "for_each_page, writing each row (set)",
    "for_each_page, write_all",
    "a map by entity: built, 2 lookups a pair",
    "256-row pages: for_each, copying out",
    "256-row pages: for_each_page, copying out",
    "par_for_each_page, copying out, 16 chunks inline",
    "par_for_each_page, writing each row, 16 inline",
    "the same split into 16 chunks, and nothing done",
    "for_each_page, and nothing done",
    "gravity: for_each and Mut",
    "gravity: par_for_each_page inline, get_mut",
    "gravity: par_for_each inline, Mut",
    "gravity: par_for_each on no executor, Mut",
    "gravity: for_each and Mut, again",
];

/// Claims four threads, and runs every task on the caller: a parallel
/// walk's cost of its own, without another core's.
struct Inline;

impl Executor for Inline {
    fn threads(&self) -> usize {
        4
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        (0..tasks).for_each(f);
    }
}

/// Nanoseconds per stage, summed over frames, and rows or pairs each covers.
static OUT: Mutex<([u128; 18], [usize; 18])> = Mutex::new(([0; 18], [0; 18]));
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

    let workers = Workers::new(Some(Arc::new(Inline)));
    let s = Instant::now();
    let parts = q.par_for_each_page(&workers, |r| Vec::with_capacity(r.len()), |out, page, (m, v, _)| {
        out.extend(page.rows().map(|r| solver(&m[r], &v[r])))
    });
    let mut out: Vec<Solver> = Vec::with_capacity(n);
    parts.into_iter().for_each(|p| out.extend(p));
    record(9, s, n);

    let s = Instant::now();
    let out = &out;
    q.par_for_each_page(&workers, |r| r.start, |i, page, (_, mut v, _)| {
        for r in page.rows() {
            v.set(r, Vel { x: out[*i].v[0], y: out[*i].v[1] });
            *i += 1;
        }
    });
    record(10, s, n);

    let s = Instant::now();
    q.par_for_each_page(&workers, |_| (), |_, _, _| {});
    record(11, s, n);

    let s = Instant::now();
    q.for_each_page(|_, _| {});
    record(12, s, n);

    let s = Instant::now();
    q.for_each(|_, (m, mut v, _)| {
        if m.kind == 0 {
            v.x += m.a * 0.016;
            v.y += m.b * 0.016;
        }
    });
    record(13, s, n);

    let s = Instant::now();
    q.par_for_each_page(&workers, |_| (), |_, page, (m, mut v, _)| {
        for i in page.rows().filter(|&i| m[i].kind == 0) {
            let mut v = v.get_mut(i);
            v.x += m[i].a * 0.016;
            v.y += m[i].b * 0.016;
        }
    });
    record(14, s, n);

    let s = Instant::now();
    q.par_for_each(&workers, |_| (), |_, _, (m, mut v, _)| {
        if m.kind == 0 {
            v.x += m.a * 0.016;
            v.y += m.b * 0.016;
        }
    });
    record(15, s, n);

    let s = Instant::now();
    q.par_for_each(&Workers::default(), |_| (), |_, _, (m, mut v, _)| {
        if m.kind == 0 {
            v.x += m.a * 0.016;
            v.y += m.b * 0.016;
        }
    });
    record(16, s, n);

    let s = Instant::now();
    q.for_each(|_, (m, mut v, _)| {
        if m.kind == 0 {
            v.x += m.a * 0.016;
            v.y += m.b * 0.016;
        }
    });
    record(17, s, n);
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
        *OUT.lock().unwrap() = ([0; 18], [0; 18]);
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
