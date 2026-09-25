//! The step split across threads, stage by stage, in the ECS and on arrays:
//! `./bazel run -c opt //engine/std/physics:tax -- parallel`.
//!
//! Both sides run the same parallel algorithm: work in contiguous chunks
//! (pages of a query, or ranges of an array), a few per thread, whose
//! results are joined in chunk order, so a run at N threads must end bit for
//! bit where the run at one thread does, which is checked (positions,
//! velocities, and every contact with its entity and impulses). The solver
//! itself stays on one thread on both sides: it's measured, not split.
//!
//! The threads are the host's, as they'd be a resident scheduler's: an
//! executor installed in the world between frames, which the physics mod
//! reaches through its systems' `Workers`.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use engine_ecs::par::{carve, even};
use engine_ecs::{Executor, Scoped, Workers};

use super::arrays::{Cached, DT};
use super::pool::Pool;
use super::solver::{Constraint, SolverBody};
use super::*;
use engine_ecs::World;
use physics::{ContactPair, DYNAMIC, Impulse, KINEMATIC, Manifold, STATIC, Vec2, Velocity};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Kind {
    Pool,
    Scoped,
    /// Claims two threads, and runs every task on the caller: the parallel
    /// code's own cost, without another core's.
    Inline,
}

struct Inline;

impl Executor for Inline {
    fn threads(&self) -> usize {
        2
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        (0..tasks).for_each(f);
    }
}

fn executor(kind: Kind, threads: usize) -> Arc<dyn Executor> {
    match kind {
        Kind::Pool => Arc::new(Pool::new(threads)),
        Kind::Scoped => Arc::new(Scoped(threads)),
        Kind::Inline => Arc::new(Inline),
    }
}

/// `slice`, indexed by `idx` (ascending), cut where each of `ranges` of
/// `idx` starts: each piece holds every index its range names, and its
/// first index. How an array addressed through a list of indices is split
/// for tasks with no copy.
fn by_index<'a, T>(slice: &'a mut [T], idx: &[u32], ranges: &[std::ops::Range<usize>]) -> Vec<(&'a mut [T], usize)> {
    let cuts: Vec<usize> = ranges.iter().enumerate().map(|(k, r)| if k == 0 { 0 } else { idx[r.start] as usize }).collect();
    let len = slice.len();
    let lens = cuts.iter().enumerate().map(|(k, &c)| cuts.get(k + 1).copied().unwrap_or(len) - c);
    carve(slice, lens).into_iter().zip(cuts.iter().copied()).collect()
}

/// Pairs split by range of their lesser index, sorted, and joined: the
/// sort `step` makes of them all, in parallel. Lists are made and freed on
/// this thread, as everywhere here: see the physics mod's gathering.
fn sort_pairs(w: &Workers, parts: Vec<Vec<Vec<(u32, u32)>>>) -> Vec<(u32, u32)> {
    let ranges = parts.first().map_or(0, Vec::len);
    let mut parts: Vec<Vec<Option<Vec<(u32, u32)>>>> = parts.into_iter().map(|p| p.into_iter().map(Some).collect()).collect();
    let by_range: Vec<_> = (0..ranges)
        .map(|r| {
            let lists: Vec<Vec<(u32, u32)>> = parts.iter_mut().map(|p| p[r].take().unwrap()).collect();
            let n = lists.iter().map(Vec::len).sum();
            (lists, Vec::with_capacity(n))
        })
        .collect();
    let sorted = w.map_each(by_range, |_, (lists, mut v)| {
        lists.iter().for_each(|l| v.extend_from_slice(l));
        v.sort_unstable();
        (v, lists)
    });
    sorted.into_iter().map(|(v, _)| v).collect::<Vec<_>>().concat()
}

/// `f` over ranges of `0..n` in parallel, each filling a list made here
/// with room for its range, joined in order. With `TASK_ALLOC` set, each
/// task makes its own list instead (filled by one `extend`, so allocated
/// once): docs/lore/memory-a-task-allocates-is-its-threads.md.
fn fill<T: Send + Clone>(w: &Workers, n: usize, min: usize, f: impl Fn(std::ops::Range<usize>, &mut Vec<T>) + Sync) -> Vec<T> {
    static TASK_ALLOC: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let in_task = *TASK_ALLOC.get_or_init(|| std::env::var_os("TASK_ALLOC").is_some());
    let parts: Vec<_> = even(n, w.chunks(n, min)).into_iter().map(|r| (if in_task { Vec::new() } else { Vec::with_capacity(r.len()) }, r)).collect();
    w.map_each(parts, |_, (mut out, r)| {
        f(r, &mut out);
        out
    })
    .concat()
}

impl Arrays {
    /// `step`, each stage split across `w` as the mod splits it, but for the
    /// insertion sort that keeps the sweep's order, which is one pass that
    /// depends on the one before it.
    fn step_par(&mut self, t: &mut Stages, w: &Workers) {
        let start = Instant::now();
        let n = self.pos.len();
        let moving_ranges = even(self.moving.len(), w.chunks(self.moving.len(), 512));
        {
            let (moving, body, g) = (&self.moving, &self.body, self.gravity);
            let pieces = by_index(&mut self.vel, moving, &moving_ranges);
            w.map_each(moving_ranges.iter().cloned().zip(pieces).collect(), |_, (r, (vel, base))| {
                for &i in &moving[r] {
                    let (b, v) = (&body[i as usize], &mut vel[i as usize - base]);
                    if b.kind == DYNAMIC {
                        v.x += g.x * b.gravity_scale * DT;
                        v.y += g.y * b.gravity_scale * DT;
                    }
                }
            });
        }
        let gravity = Instant::now();

        self.sort_by_x();
        let this = &*self;
        let boxes: Vec<physics::Aabb> = fill(w, n, 1024, |r, out| {
            out.extend(r.map(|i| {
                let b = this.placed(i).aabb();
                let m = Vec2::new(narrow::MARGIN, narrow::MARGIN);
                physics::Aabb { min: b.min - m, max: b.max + m }
            }))
        });
        let shift = n.div_ceil(w.threads() * 2).max(1).next_power_of_two().trailing_zeros();
        let ranges = (n >> shift) + 1;
        let (boxes, by_x) = (&boxes, &this.by_x);
        let sweeps: Vec<_> = even(by_x.len(), w.chunks(by_x.len(), 256))
            .into_iter()
            .map(|r| ((0..ranges).map(|_| Vec::with_capacity(r.len() * 3 / ranges + 8)).collect::<Vec<Vec<(u32, u32)>>>(), r))
            .collect();
        let parts = w.map_each(sweeps, |_, (mut out, r)| {
            for k in r {
                let i = by_x[k];
                let a = boxes[i as usize];
                for &j in &by_x[k + 1..] {
                    let b = boxes[j as usize];
                    if b.min.x > a.max.x {
                        break;
                    }
                    if a.min.y <= b.max.y && b.min.y <= a.max.y {
                        let p = (i.min(j), i.max(j));
                        out[(p.0 >> shift) as usize].push(p);
                    }
                }
            }
            out
        });
        let pairs = sort_pairs(w, parts);
        let broadphase = Instant::now();

        let pairs = &pairs;
        let mut found: Vec<Cached> = fill(w, pairs.len(), 256, |r, found| {
                for &(i, j) in &pairs[r] {
                    let (a, b) = (i as usize, j as usize);
                    let (ca, cb, ba, bb) = (&this.collider[a], &this.collider[b], &this.body[a], &this.body[b]);
                    let meets = ca.mask & cb.layer != 0 && cb.mask & ca.layer != 0;
                    if !meets || ca.sensor || cb.sensor || (ba.kind != DYNAMIC && bb.kind != DYNAMIC) {
                        continue;
                    }
                    let Some(m) = narrow::collide(&this.placed(a), &this.placed(b), this.vel[b] - this.vel[a]) else { continue };
                    found.push(Cached {
                        a: i,
                        b: j,
                        normal: m.normal,
                        depth: m.depth,
                        friction: ba.friction.min(bb.friction),
                        restitution: ba.restitution.max(bb.restitution),
                        jn: 0.0,
                        jt: 0.0,
                        pressed: false,
                        was_pressed: false,
                    });
                }
            });
        let narrowphase = Instant::now();

        {
            let old = &self.contacts;
            let ranges = even(found.len(), w.chunks(found.len(), 512));
            let pieces = carve(&mut found, ranges.iter().map(|r| r.len()));
            w.map_each(pieces, |_, found| {
                let Some(first) = found.first() else { return };
                let mut k = old.partition_point(|c| (c.a, c.b) < (first.a, first.b));
                for f in found.iter_mut() {
                    while k < old.len() && (old[k].a, old[k].b) < (f.a, f.b) {
                        k += 1;
                    }
                    if let Some(o) = old.get(k).filter(|c| (c.a, c.b) == (f.a, f.b)) {
                        (f.jn, f.jt, f.was_pressed) = (o.jn, o.jt, o.pressed);
                    }
                }
            });
        }
        self.contacts = found;
        let merge = Instant::now();

        let mut slot = vec![u32::MAX; n];
        {
            let moving = &self.moving;
            let pieces = by_index(&mut slot, moving, &moving_ranges);
            w.map_each(moving_ranges.iter().cloned().zip(pieces).collect(), |_, (r, (slot, base))| {
                for s in r {
                    slot[moving[s] as usize - base] = s as u32;
                }
            });
        }
        let this = &*self;
        let mut bodies: Vec<SolverBody> = fill(w, this.moving.len(), 512, |r, out| {
            out.extend(this.moving[r].iter().map(|&i| {
                let b = &this.body[i as usize];
                let inv_mass = if b.kind == DYNAMIC { b.inv_mass } else { 0.0 };
                SolverBody { v: this.vel[i as usize], inv_mass, pseudo: Vec2::ZERO }
            }))
        });
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        let slot = &slot;
        let at = |i: u32| if slot[i as usize] == u32::MAX { still } else { slot[i as usize] };
        let mut constraints: Vec<Constraint> = fill(w, this.contacts.len(), 512, |r, out| {
            out.extend(this.contacts[r].iter().map(|c| Constraint {
                        a: at(c.a),
                        b: at(c.b),
                        normal: c.normal,
                        depth: c.depth,
                        friction: c.friction,
                        restitution: c.restitution,
                        jn: c.jn,
                        jt: c.jt,
                        speed: 0.0,
            }))
        });
        let solve_gather = Instant::now();
        solver::solve(&mut bodies, &mut constraints, DT);
        let solved = Instant::now();

        {
            let (moving, body, bodies) = (&self.moving, &self.body, &bodies);
            let vel = by_index(&mut self.vel, moving, &moving_ranges);
            let pos = by_index(&mut self.pos, moving, &moving_ranges);
            let tasks: Vec<_> = moving_ranges.iter().cloned().zip(vel).zip(pos).collect();
            w.map_each(tasks, |_, ((r, (vel, base)), (pos, _))| {
                for s in r {
                    let (i, b) = (moving[s] as usize, bodies[s]);
                    if body[i].kind == STATIC {
                        continue;
                    }
                    vel[i - base] = b.v;
                    let step = if body[i].kind == KINEMATIC { b.v } else { b.v + b.pseudo };
                    let p = &mut pos[i - base];
                    *p = Vec2::new(p.x + step.x * DT, p.y + step.y * DT);
                }
            });
            let ranges = even(self.contacts.len(), w.chunks(self.contacts.len(), 512));
            let lens: Vec<usize> = ranges.iter().map(|r| r.len()).collect();
            let tasks: Vec<_> = carve(&mut self.contacts, lens.iter().copied()).into_iter().zip(carve(&mut constraints, lens)).collect();
            w.map_each(tasks, |_, (contacts, constraints)| {
                for (c, k) in contacts.iter_mut().zip(constraints.iter()) {
                    (c.jn, c.jt) = (k.jn, k.jt);
                    c.pressed = k.jn > 0.0 || c.depth >= 0.0;
                }
            });
        }
        let done = Instant::now();

        let us = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1e6;
        t.gravity += us(start, gravity);
        t.broadphase += us(gravity, broadphase);
        t.narrowphase += us(broadphase, narrowphase);
        t.merge += us(narrowphase, merge);
        t.solve_gather += us(merge, solve_gather);
        t.solver += us(solve_gather, solved);
        t.write_back += us(solved, done);
    }

    /// Everything the step leaves, bit for bit.
    fn digest(&self) -> Vec<u32> {
        let mut d: Vec<u32> = self.pos.iter().zip(&self.vel).flat_map(|(p, v)| [p.x, p.y, v.x, v.y].map(f32::to_bits)).collect();
        for c in &self.contacts {
            d.extend([c.a, c.b, c.jn.to_bits(), c.jt.to_bits(), c.pressed as u32]);
        }
        d
    }
}

/// The world's bodies and contacts, bit for bit, entities included: a run
/// at N threads must leave the one at one thread's.
fn world_digest(w: &World) -> Vec<u32> {
    let vel: HashMap<Entity, Velocity> = w.values::<Velocity>().unwrap().into_iter().collect();
    let mut pos = w.values::<Position>().unwrap();
    pos.sort_by_key(|(e, _)| *e);
    let mut d = Vec::new();
    for (e, p) in pos {
        let v = vel.get(&e).copied().unwrap_or_default();
        d.extend([e.index, e.generation, p.x.to_bits(), p.y.to_bits(), v.x.to_bits(), v.y.to_bits()]);
    }
    let impulses: HashMap<Entity, Impulse> = w.values::<Impulse>().unwrap().into_iter().collect();
    let manifolds: HashMap<Entity, Manifold> = w.values::<Manifold>().unwrap().into_iter().collect();
    let mut contacts = w.values::<ContactPair>().unwrap();
    contacts.sort_by_key(|(_, p)| (p.a, p.b));
    for (e, p) in contacts {
        let (j, m) = (impulses[&e], manifolds[&e]);
        d.extend([e.index, e.generation, p.a.index, p.b.index, j.normal.to_bits(), j.tangent.to_bits(), m.pressed as u32, m.depth.to_bits()]);
    }
    d
}

const STAGES: [&str; 11] =
    ["frame", "gravity", "gather", "broadphase", "narrowphase", "merge", "solve_gather", "solver", "write_back", "outside", "near"];

/// One run's µs per step by stage (`STAGES`), ECS and arrays; the arrays
/// have no gathering and nothing outside their stages.
struct Run {
    ecs: [f64; 11],
    arrays: [f64; 11],
    ecs_digest: Vec<u32>,
    arrays_digest: Vec<u32>,
}

/// Jiffies every other process has spent, by process: summed over
/// `/proc`, since the machine's total less this process's counted about a
/// core of this one's own work as someone else's.
fn others() -> std::collections::HashMap<u32, u64> {
    let me = std::process::id();
    let mut out = std::collections::HashMap::new();
    for entry in std::fs::read_dir("/proc").into_iter().flatten().flatten() {
        let Some(pid) = entry.file_name().to_str().and_then(|n| n.parse::<u32>().ok()) else { continue };
        if pid == me {
            continue;
        }
        let Ok(stat) = std::fs::read_to_string(entry.path().join("stat")) else { continue };
        // After the command's closing parenthesis: utime and stime are the
        // 12th and 13th fields.
        let f: Vec<u64> = stat.rsplit(')').next().unwrap_or("").split_whitespace().filter_map(|f| f.parse().ok()).collect();
        out.insert(pid, f.get(10).copied().unwrap_or(0) + f.get(11).copied().unwrap_or(0));
    }
    out
}

/// Cores' worth of other processes' work while `f` ran: another benchmark
/// on the machine (a sibling's) makes the numbers mean nothing.
fn others_during<R>(f: impl FnOnce() -> R) -> (R, f64) {
    let (before, start) = (others(), Instant::now());
    let r = f();
    let after = others();
    let jiffies = start.elapsed().as_secs_f64() * 100.0;
    let spent: u64 = after.iter().map(|(pid, &t)| t.saturating_sub(before.get(pid).copied().unwrap_or(t))).sum();
    (r, spent as f64 / jiffies.max(1.0))
}

/// Cores of other work (a sibling's benchmark) beyond which a run is
/// taken again: the machine's own background (editors, Bazel servers) is
/// a few tenths of one.
const BUSY: f64 = 1.0;

/// `measure`, again until nothing else ran beside it (up to a limit, after
/// which it says so), waiting for the machine to be quiet first.
fn quiet_measure(manifest: &engine_control::Manifest, scene: (u32, f32, u32), threads: usize, kind: Kind, frames: u32) -> Run {
    let mut attempt = 0;
    loop {
        attempt += 1;
        let ((), idle) = others_during(|| std::thread::sleep(Duration::from_millis(200)));
        if idle > BUSY && attempt < 300 {
            std::thread::sleep(Duration::from_secs(2));
            continue;
        }
        let (run, others) = others_during(|| measure(manifest, scene, threads, kind, frames));
        if others <= BUSY || attempt >= 300 {
            if others > BUSY {
                eprintln!("(a run at {threads} threads shared the machine with {others:.1} cores of other work)");
            }
            return run;
        }
    }
}

fn measure(manifest: &engine_control::Manifest, (n, width, warmup): (u32, f32, u32), threads: usize, kind: Kind, frames: u32) -> Run {
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    let k = RUNS.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("physics-tax-par-{}-{k}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), dir.clone());
    e.load_batch(&manifest.mods).expect("loading the pile");
    e.send("pile", &format!("widen {width}")).unwrap();
    e.send("pile", &format!("drop {n}")).unwrap();
    e.send("lockstep", &format!("step {warmup}")).unwrap();

    let exec = (threads > 1 || kind == Kind::Inline).then(|| executor(kind, threads));
    let workers = Workers::new(exec.clone());
    let mut arrays = Arrays::snapshot(e.world());
    let mut t = Stages::default();
    let start = Instant::now();
    for _ in 0..frames {
        if exec.is_some() {
            arrays.step_par(&mut t, &workers);
        } else {
            arrays.step(&mut t, solver::solve);
        }
    }
    let array_frame = (start.elapsed().as_secs_f64() * 1e6 - t.fresh_sweep) / frames as f64;

    e.world().set_executor(exec);
    e.send("physics", "reset_timings").unwrap();
    let start = Instant::now();
    e.send("lockstep", &format!("step {frames}")).unwrap();
    let ecs_frame = start.elapsed().as_secs_f64() * 1e6 / frames as f64;
    let (stats, stages) = (e.send("physics", "stats").unwrap(), e.send("physics", "stages").unwrap());
    let per_step = stats.split("us/step").nth(1).expect("timings in stats");
    let systems = field(per_step, "gravity") + field(per_step, "contacts") + field(per_step, "solve");
    let f = frames as f64;
    let mut ecs = [0.0; 11];
    ecs[0] = ecs_frame;
    for (i, s) in STAGES.iter().enumerate().skip(1).take(8) {
        ecs[i] = field(&stages, s);
    }
    ecs[9] = ecs_frame - systems;
    ecs[10] = field(&stages, "near");
    let arr = [array_frame, t.gravity / f, 0.0, t.broadphase / f, t.narrowphase / f, t.merge / f, t.solve_gather / f, t.solver / f, t.write_back / f, 0.0, 0.0];
    let run = Run { ecs, arrays: arr, ecs_digest: world_digest(e.world()), arrays_digest: arrays.digest() };
    if workers.threads() == 1 {
        // The comparison `main` makes: the same computation both ways.
        let ecs: HashMap<Entity, Position> = e.world().values::<Position>().unwrap().into_iter().collect();
        let differ = arrays.entity.iter().zip(&arrays.pos).filter(|(e, p)| ecs[e].x.to_bits() != p.x.to_bits() || ecs[e].y.to_bits() != p.y.to_bits()).count();
        assert_eq!(differ, 0, "{n}: {differ} bodies ended elsewhere than the arrays put them");
    }
    e.world().set_executor(None);
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
    run
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) / 2.0 }
}

/// What handing work to threads costs by itself: a run of empty tasks, and
/// of tasks of about a microsecond, four per thread.
fn dispatch(threads: &[usize]) {
    println!("\nµs per run of 4 tasks a thread: empty, and each about 1 µs of work (medians of 5 × 2000 runs)\n");
    println!("| threads | pool: empty | pool: 1 µs tasks | scoped: empty | scoped: 1 µs tasks |");
    println!("|---|---|---|---|---|");
    let busy = |_: usize| {
        let mut x = 0u64;
        for i in 0..2000u64 {
            x = std::hint::black_box(x.wrapping_mul(31).wrapping_add(i));
        }
        std::hint::black_box(x);
    };
    for &n in threads.iter().filter(|&&n| n > 1) {
        let mut cells = Vec::new();
        for kind in [Kind::Pool, Kind::Scoped] {
            let w = Workers::new(Some(executor(kind, n)));
            let runs = if kind == Kind::Pool { 2000 } else { 200 };
            for work in [false, true] {
                let mut samples = Vec::new();
                for _ in 0..5 {
                    let start = Instant::now();
                    for _ in 0..runs {
                        if work {
                            w.run(n * 4, busy);
                        } else {
                            w.run(n * 4, |_| {});
                        }
                    }
                    samples.push(start.elapsed().as_secs_f64() * 1e6 / runs as f64);
                }
                cells.push(format!("{:.1}", median(samples)));
            }
        }
        println!("| {n} | {} |", cells.join(" | "));
    }
    let start = Instant::now();
    for _ in 0..200 {
        (0..4).for_each(busy);
    }
    println!("\n(one task of the 1 µs kind: {:.2} µs)", start.elapsed().as_secs_f64() * 1e6 / 800.0);
}

pub fn run(manifest: &engine_control::Manifest) {
    const FRAMES: u32 = 60;
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(3);
    let threads: Vec<usize> = std::env::var("THREADS")
        .ok()
        .map(|t| t.split(',').map(|n| n.parse().unwrap()).collect())
        .unwrap_or_else(|| vec![1, 2, 4, 8, 12, 16]);
    assert_eq!(threads[0], 1, "the first run is the one-thread reference");
    dispatch(&threads);
    // One thread, the parallel code on one thread ("1 inline"), then the
    // pool at each count.
    let mut configs: Vec<(String, usize, Kind)> = vec![("1".into(), 1, Kind::Pool), ("1 inline".into(), 1, Kind::Inline)];
    configs.extend(threads.iter().skip(1).map(|&t| (t.to_string(), t, Kind::Pool)));
    for (width, scene, warmup) in [(400.0f32, "falling", 1u32), (400.0, "settled", 400), (401.0, "falling", 1), (401.0, "settled", 400)] {
        let n = 10000u32;
        // By configuration, by rep.
        let mut runs: Vec<Vec<Run>> = configs.iter().map(|_| Vec::new()).collect();
        for _ in 0..reps {
            for (ti, (name, t, kind)) in configs.iter().enumerate() {
                let run = quiet_measure(manifest, (n, width, warmup), *t, *kind, FRAMES);
                if ti > 0 {
                    let one = &runs[0].last().expect("the reference");
                    assert!(run.ecs_digest == one.ecs_digest, "{scene}: the ECS at {name} ended elsewhere than at one thread");
                    assert!(run.arrays_digest == one.arrays_digest, "{scene}: the arrays at {name} ended elsewhere than at one thread");
                }
                runs[ti].push(run);
            }
        }
        println!("\n{n} in a box {width} wide, {scene}: µs per step (speedup over one thread), ECS / arrays, medians of {reps} runs of {FRAMES} steps; pool");
        println!("(every run at N threads ended bit for bit where one thread did, both sides)\n");
        println!("| stage | {} |", configs.iter().map(|c| c.0.clone()).collect::<Vec<_>>().join(" | "));
        println!("|---|{}", "---|".repeat(configs.len()));
        let med = |ti: usize, s: usize, ecs: bool| median(runs[ti].iter().map(|r| if ecs { r.ecs[s] } else { r.arrays[s] }).collect());
        for (s, name) in STAGES.iter().enumerate() {
            let cells: Vec<String> = (0..configs.len())
                .map(|ti| {
                    let cell = |ecs: bool| {
                        let (one, here) = (med(0, s, ecs), med(ti, s, ecs));
                        if ti == 0 { format!("{here:.0}") } else { format!("{here:.0} ({:.1}×)", one / here) }
                    };
                    let has_arrays = !matches!(*name, "gather" | "outside" | "near");
                    if has_arrays { format!("{} / {}", cell(true), cell(false)) } else { format!("{} / –", cell(true)) }
                })
                .collect();
            println!("| {name} | {} |", cells.join(" | "));
        }
        println!("\nSpread (min–max over the runs), ECS / arrays:\n");
        println!("| stage | {} |", configs.iter().map(|c| c.0.clone()).collect::<Vec<_>>().join(" | "));
        println!("|---|{}", "---|".repeat(configs.len()));
        for (s, name) in STAGES.iter().enumerate() {
            let cells: Vec<String> = (0..configs.len())
                .map(|ti| {
                    let range = |ecs: bool| {
                        let v: Vec<f64> = runs[ti].iter().map(|r| if ecs { r.ecs[s] } else { r.arrays[s] }).collect();
                        format!("{:.0}–{:.0}", v.iter().cloned().fold(f64::INFINITY, f64::min), v.iter().cloned().fold(0.0, f64::max))
                    };
                    format!("{} / {}", range(true), range(false))
                })
                .collect();
            println!("| {name} | {} |", cells.join(" | "));
        }
    }
    // The same step with threads spawned for every run instead of kept.
    let (n, width, warmup) = (10000u32, 401.0f32, 400u32);
    println!("\n{n} settled in a box {width} wide, pool against threads spawned per run (`Scoped`): µs per step, ECS / arrays, medians of {reps}\n");
    println!("| threads | pool: frame | scoped: frame | pool: stages but the solver | scoped: stages but the solver |");
    println!("|---|---|---|---|---|");
    for &t in threads.iter().filter(|&&t| t > 1) {
        let mut cells = Vec::new();
        let mut rest = Vec::new();
        for kind in [Kind::Pool, Kind::Scoped] {
            let runs: Vec<Run> = (0..reps).map(|_| quiet_measure(manifest, (n, width, warmup), t, kind, FRAMES)).collect();
            let m = |f: &dyn Fn(&Run) -> f64| median(runs.iter().map(f).collect());
            cells.push(format!("{:.0} / {:.0}", m(&|r| r.ecs[0]), m(&|r| r.arrays[0])));
            let others = |x: &[f64; 11]| x[1..10].iter().sum::<f64>() - x[7];
            rest.push(format!("{:.0} / {:.0}", m(&|r| others(&r.ecs)), m(&|r| others(&r.arrays))));
        }
        println!("| {t} | {} | {} | {} | {} |", cells[0], cells[1], rest[0], rest[1]);
    }
}
