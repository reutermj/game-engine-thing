//! A live proximity relation's broadphase (`engine_ecs::live`) against
//! `near_pairs` afresh, without physics, on touching boxes on a lattice (as
//! `spatial3d_bench`'s), in 2D and 3D: at rest, creeping as a settled pile
//! does, one row in a hundred flying, and everything falling. The state
//! alone (`LivePairs`, whose margin and threshold the arguments set), and
//! the same through a declared relation, `Live<R>`, at the defaults.
//! `./bazel run --config=bench //engine/ecs:live_bench [-- margin most]`.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::live::{LivePairs, LiveStats};
use engine_ecs::{AnyOf, Bounds, Build, Live, Proximity, Query, SpatialKey, With, Workers, World, component, near_pairs};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At: "bench::At", order = spatial { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct At3: "bench::At3", order = spatial { pub x: f32, pub y: f32, pub z: f32 }
}

impl SpatialKey for At {
    type Extent = At;
    fn bounds(&self, _: Option<&At>) -> Bounds {
        Bounds::around([self.x, self.y], [0.45, 0.45])
    }
}

impl SpatialKey<3> for At3 {
    type Extent = At3;
    fn bounds(&self, _: Option<&At3>) -> Bounds<3> {
        Bounds::around([self.x, self.y, self.z], [0.45, 0.45, 0.45])
    }
}

/// The lattice's pairs, as a relation: every row active.
struct Lattice;
impl Proximity for Lattice {
    type Key = At;
    type Active = With<At>;
    type Passive = AnyOf<()>;
    const GROW: f32 = 0.05;
}

struct Lattice3;
impl Proximity for Lattice3 {
    type Key = At3;
    type Active = With<At3>;
    type Passive = AnyOf<()>;
    const GROW: f32 = 0.05;
}

const REST: u32 = 0;
const CREEP: u32 = 1;
const FLY: u32 = 2;
const FALL: u32 = 3;
const FLY5: u32 = 4;
const FLY20: u32 = 5;
const NAMES: [&str; 6] = ["at rest", "creeping", "1% flying", "falling", "5% flying", "20% flying"];

static SCENE: AtomicU32 = AtomicU32::new(REST);
static FRAME: AtomicU32 = AtomicU32::new(0);

fn lcg(s: &mut u64) -> f32 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*s >> 40) as f32 / (1u64 << 24) as f32
}

/// How far entity `i` moves this frame: creeping by up to 2e-3 an axis (a
/// settled pile creeps 1e-4 to 1e-2), a few flying, all falling 0.3.
fn delta<const N: usize>(i: u32) -> Option<[f32; N]> {
    let (scene, frame) = (SCENE.load(Ordering::Relaxed), FRAME.load(Ordering::Relaxed));
    let mut s = (i as u64) << 32 | frame as u64;
    lcg(&mut s);
    let mut by = |r: f32| std::array::from_fn(|_| (lcg(&mut s) - 0.5) * r);
    match scene {
        CREEP => Some(by(4e-3)),
        FLY if i % 100 == frame % 100 => Some(by(4.0)),
        FLY5 if i % 20 == frame % 20 => Some(by(4.0)),
        FLY20 if i % 5 == frame % 5 => Some(by(4.0)),
        FALL => Some(std::array::from_fn(|a| if a == 1 { -0.3 } else { 0.0 })),
        _ => None,
    }
}

/// Time in each broadphase, summed (the state alone, afresh, through
/// `Live`), and the last call's stats.
static TIMES: Mutex<(u128, u128, u128, usize, Vec<LiveStats>)> = Mutex::new((0, 0, 0, 0, Vec::new()));
static COPY: Mutex<u128> = Mutex::new(0);
static KEPT: Mutex<Option<LivePairs>> = Mutex::new(None);
static HOW: Mutex<(f32, f32)> = Mutex::new((0.05, 0.1));

/// Each way, one after another: the state alone, `Live`'s (another
/// state, over the same pages), and afresh.
fn both<R: Proximity>(now: u32, q: &impl engine_ecs::NearSide, live: &mut Live<'_, R>) {
    let t = Instant::now();
    let l = live.pairs().len();
    let l_ns = t.elapsed().as_nanos();
    let mut kept = KEPT.lock().unwrap();
    let kept = kept.as_mut().expect("made before the frames");
    let t = Instant::now();
    let (margin, most) = *HOW.lock().unwrap();
    let pairs = kept.near_pairs(now, &Workers::default(), (q, &()), (0.05, margin, most));
    let k = t.elapsed().as_nanos();
    let n = pairs.len();
    // What handing the pairs out as a `Vec`, as `near_pairs` does, would add.
    let t = Instant::now();
    std::hint::black_box(pairs.to_vec());
    let copy = t.elapsed().as_nanos();
    let t = Instant::now();
    let m = near_pairs(q, &(), 0.05).len();
    let a = t.elapsed().as_nanos();
    assert_eq!((n, l), (m, m));
    let mut out = TIMES.lock().unwrap();
    (out.0, out.1, out.2, out.3) = (out.0 + k, out.1 + a, out.2 + l_ns, n);
    out.4.push(kept.stats());
    *COPY.lock().unwrap() += copy;
}

fn move2(_: &mut Cx, mut q: Query<&mut At>) {
    q.for_each(|row, mut at| {
        if let Some([x, y]) = delta(row.entity().index) {
            (at.x, at.y) = (at.x + x, at.y + y);
        }
    });
}

fn move3(_: &mut Cx, mut q: Query<&mut At3>) {
    q.for_each(|row, mut at| {
        if let Some([x, y, z]) = delta(row.entity().index) {
            (at.x, at.y, at.z) = (at.x + x, at.y + y, at.z + z);
        }
    });
}

fn pairs2(_: &mut Cx, q: Query<&At>, mut live: Live<Lattice>) {
    both(q.now(), &q, &mut live);
}

fn pairs3(_: &mut Cx, q: Query<&At3>, mut live: Live<Lattice3>) {
    both(q.now(), &q, &mut live);
}

fn frames(dims: usize, n: usize, scene: u32, (margin, most): (f32, f32)) {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let side = (n as f32).powf(1.0 / dims as f32).ceil() as usize;
        for k in 0..n {
            let c = |a: u32| (k / side.pow(a)) % side;
            if dims == 3 {
                m.spawn((At3 { x: c(0) as f32 * 0.9, y: c(1) as f32 * 0.9, z: c(2) as f32 * 0.9 },));
            } else {
                m.spawn((At { x: c(0) as f32 * 0.9, y: c(1) as f32 * 0.9 },));
            }
        }
    }
    let s = if dims == 3 {
        Schedule { systems: vec![move3.system(&w, "move"), pairs3.system(&w, "pairs")] }
    } else {
        Schedule { systems: vec![move2.system(&w, "move"), pairs2.system(&w, "pairs")] }
    };
    *KEPT.lock().unwrap() = Some(LivePairs::default());
    *HOW.lock().unwrap() = (margin, most);
    SCENE.store(scene, Ordering::Relaxed);
    let (warm, timed) = (30, 100);
    for f in 0..warm + timed {
        if f == warm {
            *TIMES.lock().unwrap() = (0, 0, 0, 0, Vec::new());
            *COPY.lock().unwrap() = 0;
        }
        FRAME.store(f, Ordering::Relaxed);
        // `POLLUTE=1`: the rest of a physics step between broadphases,
        // which leaves nothing of the last one in cache.
        if std::env::var_os("POLLUTE").is_some() {
            thrash();
        }
        let _ = s.run_sequential(&w);
    }
    let (k, a, l, pairs, stats) = std::mem::take(&mut *TIMES.lock().unwrap());
    let us = |t: u128| t as f64 / timed as f64 / 1e3;
    let mean = |f: fn(&LiveStats) -> usize| stats.iter().map(f).sum::<usize>() as f64 / stats.len() as f64;
    let mut hows: Vec<String> = Vec::new();
    for how in ["Same", "Kept", "Rebuilt", "Afresh"] {
        let c = stats.iter().filter(|s| format!("{:?}", s.how) == how).count();
        if c > 0 {
            hows.push(format!("{how} {c}"));
        }
    }
    println!(
        "| {dims}D | {n} | {} | {:.1} | {:.1} | {:.1} | {pairs} | {:.0} | {:.1} | {:.0} | {} |",
        NAMES[scene as usize],
        us(k),
        us(l),
        us(a),
        mean(|s| s.walked),
        mean(|s| s.moved),
        mean(|s| s.candidates),
        hows.join(", ")
    );
    println!(
        "    walk / search / test µs: {:.1} {:.1} {:.1}; a copy of the pairs {:.1}",
        mean(|s| s.ns[0] as usize) / 1e3,
        mean(|s| s.ns[1] as usize) / 1e3,
        mean(|s| s.ns[2] as usize) / 1e3,
        us(*COPY.lock().unwrap())
    );
}

fn thrash() {
    static JUNK: Mutex<Vec<u64>> = Mutex::new(Vec::new());
    let mut junk = JUNK.lock().unwrap();
    junk.resize(8 << 20, 0);
    for (i, x) in junk.iter_mut().enumerate() {
        *x = x.wrapping_add(i as u64);
    }
    std::hint::black_box(&*junk);
}

fn main() {
    let args: Vec<f32> = std::env::args().skip(1).map(|a| a.parse().expect("margin most")).collect();
    let (margin, most) =
        (args.first().copied().unwrap_or(engine_ecs::live::MARGIN), args.get(1).copied().unwrap_or(engine_ecs::live::MOST));
    println!("margin {margin}, afresh above {most} moving; µs a frame, 100 frames after 30");
    println!("(kept: the state alone at those; Live: through the parameter, at the defaults)\n");
    println!("| dims | rows | scene | kept | Live | afresh | pairs | walked | moved | candidates | how |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");

    for dims in [2, 3] {
        for n in [1000, 10000] {
            for scene in [REST, CREEP, FLY, FLY5, FLY20, FALL] {
                frames(dims, n, scene, (margin, most));
            }
        }
    }
}
