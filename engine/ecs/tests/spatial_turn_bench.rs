//! How a body's rotation reaches spatial storage, measured five ways on
//! the same rows: `./bazel run --config=bench //engine/ecs:spatial_turn_bench`.
//! A rotated box's bounds depend on its position, its rotation and its
//! shape, where `SpatialKey` once took one key and one extent. The ways:
//!
//! - `aligned`: no rotation at all, the baseline (position key, size extent);
//! - `tuple`: position key, the extent a pair (size, rotation), a row
//!   without a rotation bounded as before;
//! - `pose`: position and rotation (cosine and sine) in one key, every row
//!   carrying a rotation;
//! - `pose angle`: the same with an angle, so bounding takes a sine and a
//!   cosine;
//! - `conservative`: position key, size extent, bounds the circle around
//!   the box, so rotation isn't in the bounds at all and turning never
//!   re-bounds.
//!
//! Each on a lattice of touching boxes (at random angles where they turn),
//! frame by frame: a settled pile that creeps and turns a hair, bodies
//! that only turn, falling and turning, and at rest; and a world where one
//! body in ten turns. Per frame: the writer, the re-sort after it (the rest
//! of the frame), `near_pairs` and the pairs it found. See
//! docs/architecture/spatial-storage.md, "Bounds from several components".

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Bounds, Build, Query, SpatialKey, Without, World, component};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Size: "turn::Size" { pub hx: f32, pub hy: f32 }
}

component! {
    /// A rotation as its cosine and sine, as Box2D's `b2Rot`.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Turn: "turn::Turn" { pub c: f32, pub s: f32 }
}

impl Default for Turn {
    fn default() -> Turn {
        Turn { c: 1.0, s: 0.0 }
    }
}

impl Turn {
    fn of(a: f32) -> Turn {
        Turn { c: a.cos(), s: a.sin() }
    }

    /// Turned by `da`, and normalized: Box2D's `b2IntegrateRotation`.
    #[inline]
    fn by(self, da: f32) -> Turn {
        let (c, s) = (self.c - da * self.s, self.s + da * self.c);
        let k = 1.0 / (c * c + s * s).sqrt();
        Turn { c: c * k, s: s * k }
    }
}

/// The box around `half` turned by `c, s`, at `x, y`.
#[inline]
fn turned(x: f32, y: f32, half: [f32; 2], c: f32, s: f32) -> Bounds {
    let (c, s) = (c.abs(), s.abs());
    Bounds::around([x, y], [c * half[0] + s * half[1], s * half[0] + c * half[1]])
}

fn half(size: Option<&Size>) -> [f32; 2] {
    size.map_or([0.0, 0.0], |s| [s.hx, s.hy])
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Aligned: "turn::Aligned", order = spatial { pub x: f32, pub y: f32 }
}

impl SpatialKey for Aligned {
    type Extent = Size;
    #[inline]
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        Bounds::around([self.x, self.y], half(size))
    }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Tupled: "turn::Tupled", order = spatial { pub x: f32, pub y: f32 }
}

impl SpatialKey for Tupled {
    type Extent = (Size, Turn);
    #[inline]
    fn bounds(&self, (size, turn): (Option<&Size>, Option<&Turn>)) -> Bounds {
        match turn {
            Some(t) => turned(self.x, self.y, half(size), t.c, t.s),
            None => Bounds::around([self.x, self.y], half(size)),
        }
    }
}

component! {
    #[derive(Debug, PartialEq, Copy)]
    pub struct Pose: "turn::Pose", order = spatial { pub x: f32, pub y: f32, pub c: f32, pub s: f32 }
}

impl Default for Pose {
    fn default() -> Pose {
        Pose { x: 0.0, y: 0.0, c: 1.0, s: 0.0 }
    }
}

impl SpatialKey for Pose {
    type Extent = Size;
    #[inline]
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        turned(self.x, self.y, half(size), self.c, self.s)
    }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct PoseAngle: "turn::PoseAngle", order = spatial { pub x: f32, pub y: f32, pub a: f32 }
}

impl SpatialKey for PoseAngle {
    type Extent = Size;
    #[inline]
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        let (s, c) = self.a.sin_cos();
        turned(self.x, self.y, half(size), c, s)
    }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Circled: "turn::Circled", order = spatial { pub x: f32, pub y: f32 }
}

impl SpatialKey for Circled {
    type Extent = Size;
    #[inline]
    fn bounds(&self, size: Option<&Size>) -> Bounds {
        let [hx, hy] = half(size);
        let r = (hx * hx + hy * hy).sqrt();
        Bounds::around([self.x, self.y], [r, r])
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Motion {
    /// Every body moves and turns a hair: a settled pile.
    Creep,
    /// Every body turns, and none moves.
    Turn,
    /// Every body falls about a third of a body and turns.
    Fall,
    /// Nothing is written.
    Rest,
}

impl Motion {
    /// How far body `i` moves (down) and turns this frame: only every
    /// `TURNING`th body turns.
    fn step(self, i: u32) -> (f32, f32, f32) {
        let sign = if i.is_multiple_of(2) { 1.0 } else { -1.0 };
        let turns = if i.is_multiple_of(TURNING.load(Ordering::Relaxed)) { sign } else { 0.0 };
        match self {
            Motion::Creep => (sign * 1e-3, 0.0, turns * 1e-3),
            Motion::Turn => (0.0, 0.0, turns * 1e-2),
            Motion::Fall => (0.0, 0.05 + (i * 7919 % 100) as f32 * 0.0025, turns * 5e-2),
            Motion::Rest => (0.0, 0.0, 0.0),
        }
    }
}

static TURNING: AtomicU32 = AtomicU32::new(1);

/// µs spent writing, and in `near_pairs`, summed over frames; and the pairs.
static OUT: Mutex<(u128, u128, usize)> = Mutex::new((0, 0, 0));
static MOTION: Mutex<Motion> = Mutex::new(Motion::Rest);

fn motion() -> Motion {
    *MOTION.lock().unwrap()
}

fn written(t: Instant) {
    OUT.lock().unwrap().0 += t.elapsed().as_nanos();
}

fn aligned(_: &mut Cx, mut q: Query<&mut Aligned>) {
    let (m, t) = (motion(), Instant::now());
    if m != Motion::Rest && m != Motion::Turn {
        q.for_each(|row, mut p| {
            let (dx, dy, _) = m.step(row.entity().index);
            p.x += dx;
            p.y += dy;
            // Read first: a write through `p` stamps the row, which re-bounds it.
            if p.y > 30.0 {
                p.y -= 60.0;
            }
        });
    }
    written(t);
}

/// Rows with a rotation move and turn; rows without only move.
fn tupled(_: &mut Cx, mut q: Query<(&mut Tupled, &mut Turn)>, mut still: Query<&mut Tupled, Without<Turn>>) {
    let (m, t) = (motion(), Instant::now());
    if m != Motion::Rest {
        q.for_each(|row, (mut p, mut turn)| {
            let (dx, dy, da) = m.step(row.entity().index);
            if dx != 0.0 || dy != 0.0 {
                p.x += dx;
                p.y += dy;
                // Read first: a write through `p` stamps the row, which re-bounds it.
                if p.y > 30.0 {
                    p.y -= 60.0;
                }
            }
            *turn = turn.by(da);
        });
    }
    if m != Motion::Rest && m != Motion::Turn {
        still.for_each(|row, mut p| {
            let (dx, dy, _) = m.step(row.entity().index);
            p.x += dx;
            p.y += dy;
            // Read first: a write through `p` stamps the row, which re-bounds it.
            if p.y > 30.0 {
                p.y -= 60.0;
            }
        });
    }
    written(t);
}

fn posed(_: &mut Cx, mut q: Query<&mut Pose>) {
    let (m, t) = (motion(), Instant::now());
    if m != Motion::Rest {
        q.for_each(|row, mut p| {
            let (dx, dy, da) = m.step(row.entity().index);
            // A row that doesn't turn is written only if it moves, as one
            // without a rotation would be.
            if da != 0.0 {
                let turn = Turn { c: p.c, s: p.s }.by(da);
                *p = Pose { x: p.x + dx, y: p.y + dy, c: turn.c, s: turn.s };
            } else if dx != 0.0 || dy != 0.0 {
                p.x += dx;
                p.y += dy;
            }
            // Read first: a write through `p` stamps the row, which re-bounds it.
            if p.y > 30.0 {
                p.y -= 60.0;
            }
        });
    }
    written(t);
}

fn angled(_: &mut Cx, mut q: Query<&mut PoseAngle>) {
    let (m, t) = (motion(), Instant::now());
    if m != Motion::Rest {
        q.for_each(|row, mut p| {
            let (dx, dy, da) = m.step(row.entity().index);
            if da != 0.0 || dx != 0.0 || dy != 0.0 {
                *p = PoseAngle { x: p.x + dx, y: p.y + dy, a: p.a + da };
            }
            // Read first: a write through `p` stamps the row, which re-bounds it.
            if p.y > 30.0 {
                p.y -= 60.0;
            }
        });
    }
    written(t);
}

fn circled(_: &mut Cx, mut q: Query<(&mut Circled, &mut Turn)>, mut still: Query<&mut Circled, Without<Turn>>) {
    let (m, t) = (motion(), Instant::now());
    if m != Motion::Rest {
        q.for_each(|row, (mut p, mut turn)| {
            let (dx, dy, da) = m.step(row.entity().index);
            if dx != 0.0 || dy != 0.0 {
                p.x += dx;
                p.y += dy;
                // Read first: a write through `p` stamps the row, which re-bounds it.
                if p.y > 30.0 {
                    p.y -= 60.0;
                }
            }
            *turn = turn.by(da);
        });
    }
    if m != Motion::Rest && m != Motion::Turn {
        still.for_each(|row, mut p| {
            let (dx, dy, _) = m.step(row.entity().index);
            p.x += dx;
            p.y += dy;
            // Read first: a write through `p` stamps the row, which re-bounds it.
            if p.y > 30.0 {
                p.y -= 60.0;
            }
        });
    }
    written(t);
}

fn probe<K: SpatialKey + 'static>(_: &mut Cx, q: Query<&K>) {
    let t = Instant::now();
    let pairs = engine_ecs::near_pairs(&q, &(), 0.05);
    let mut out = OUT.lock().unwrap();
    out.1 += t.elapsed().as_nanos();
    out.2 = pairs.len();
}

#[derive(Clone, Copy, PartialEq)]
enum Way {
    Aligned,
    Tuple,
    Pose,
    PoseAngle,
    Conservative,
}

impl Way {
    fn name(self) -> &'static str {
        match self {
            Way::Aligned => "aligned",
            Way::Tuple => "tuple",
            Way::Pose => "pose",
            Way::PoseAngle => "pose angle",
            Way::Conservative => "conservative",
        }
    }
}

/// A lattice of `n` boxes 0.8 wide, a unit apart, so that axis-aligned
/// they are just out of each other's reach (0.2 apart, the margin 0.05 a
/// side) and turned they reach their neighbors; every `turning`th of them
/// at a random angle, with a rotation, the rest axis-aligned without one
/// (in the `pose` ways, at angle 0, since every row there has one).
fn world(way: Way, n: usize, turning: u32) -> World {
    let w = World::new();
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        let per_row = (n as f32).sqrt() as usize * 4 / 3;
        for k in 0..n {
            let (x, y) = (0.5 + (k % per_row) as f32, 30.0 - (k / per_row) as f32);
            let turns = (k as u32).is_multiple_of(turning);
            let a = if turns { ((k * 7919) % 1000) as f32 / 1000.0 * std::f32::consts::FRAC_PI_2 } else { 0.0 };
            let size = Size { hx: 0.4, hy: 0.4 };
            match way {
                Way::Aligned => {
                    m.spawn((Aligned { x, y }, size));
                }
                Way::Tuple if turns => {
                    m.spawn((Tupled { x, y }, size, Turn::of(a)));
                }
                Way::Tuple => {
                    m.spawn((Tupled { x, y }, size));
                }
                Way::Pose => {
                    let t = Turn::of(a);
                    m.spawn((Pose { x, y, c: t.c, s: t.s }, size));
                }
                Way::PoseAngle => {
                    m.spawn((PoseAngle { x, y, a }, size));
                }
                Way::Conservative if turns => {
                    m.spawn((Circled { x, y }, size, Turn::of(a)));
                }
                Way::Conservative => {
                    m.spawn((Circled { x, y }, size));
                }
            }
        }
    }
    w
}

type Measured = (f64, f64, f64, usize);

/// µs per frame: the writer, the re-sort after it, `near_pairs`, and pairs.
fn measure(way: Way, n: usize, turning: u32, m: Motion) -> Measured {
    let w = world(way, n, turning);
    let systems = match way {
        Way::Aligned => vec![aligned.system(&w, "write"), probe::<Aligned>.system(&w, "probe")],
        Way::Tuple => vec![tupled.system(&w, "write"), probe::<Tupled>.system(&w, "probe")],
        Way::Pose => vec![posed.system(&w, "write"), probe::<Pose>.system(&w, "probe")],
        Way::PoseAngle => vec![angled.system(&w, "write"), probe::<PoseAngle>.system(&w, "probe")],
        Way::Conservative => vec![circled.system(&w, "write"), probe::<Circled>.system(&w, "probe")],
    };
    let s = Schedule { systems };
    *MOTION.lock().unwrap() = m;
    TURNING.store(turning, Ordering::Relaxed);
    for _ in 0..10 {
        s.run_sequential(&w);
    }
    *OUT.lock().unwrap() = (0, 0, 0);
    let frames = 200;
    let t = Instant::now();
    for _ in 0..frames {
        s.run_sequential(&w);
    }
    let frame = t.elapsed().as_secs_f64() * 1e6 / frames as f64;
    let (write, near, pairs) = *OUT.lock().unwrap();
    let (write, near) = (write as f64 / frames as f64 / 1e3, near as f64 / frames as f64 / 1e3);
    (write, frame - write - near, near, pairs)
}

fn median(runs: &[Measured], f: fn(&Measured) -> f64) -> f64 {
    let mut v: Vec<f64> = runs.iter().map(f).collect();
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    let ways = [Way::Aligned, Way::Tuple, Way::Pose, Way::PoseAngle, Way::Conservative];
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(3);
    println!("µs per frame, -c opt, one thread, the median of {reps} runs: writer / re-sort / near_pairs (pairs)");
    println!();
    let only = std::env::var("ONLY").ok();
    let cases = [(1, "every body turns"), (10, "one body in ten turns"), (u32::MAX, "no body turns")];
    for (turning, label) in cases.into_iter().filter(|(_, l)| only.as_ref().is_none_or(|o| l.contains(o.as_str()))) {
        for n in [1000usize, 10_000] {
            println!("### {n} bodies, {label}");
            println!();
            println!("| way | creeping | only turning | falling | at rest |");
            println!("|---|---|---|---|---|");
            for way in ways {
                print!("| {} ", way.name());
                for m in [Motion::Creep, Motion::Turn, Motion::Fall, Motion::Rest] {
                    let runs: Vec<Measured> = (0..reps).map(|_| measure(way, n, turning, m)).collect();
                    let (w, r, p) = (median(&runs, |r| r.0), median(&runs, |r| r.1), median(&runs, |r| r.2));
                    print!("| {w:.0} / {r:.0} / {p:.0} ({}) ", runs[0].3);
                }
                println!("|");
            }
            println!();
        }
    }
}
