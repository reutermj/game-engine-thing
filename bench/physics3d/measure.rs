//! Running a scene on a backend, and judging the result from poses alone,
//! with the same geometry for every engine, so no engine is graded by its own
//! notion of contact.

use std::collections::HashMap;
use std::time::Instant;

use crate::scenes::Scene;
use crate::{Backend, DT, Shape, Spec, State};

/// Bodies closer than this count as touching. Above every engine's resting
/// gap, below its speculative margin, so it means "in contact", not "near".
pub const TOUCH_GAP: f32 = 0.01;

/// Below this top speed the scene counts as at rest.
pub const REST_SPEED: f32 = 0.05;

pub const IDENTITY: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

/// `v` turned by the unit quaternion `q` (x, y, z, w).
pub fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let u = [q[0], q[1], q[2]];
    let t = cross(u, v).map(|c| 2.0 * c);
    let c = cross(u, t);
    [v[0] + q[3] * t[0] + c[0], v[1] + q[3] * t[1] + c[1], v[2] + q[3] * t[2] + c[2]]
}

fn conj(q: [f32; 4]) -> [f32; 4] {
    [-q[0], -q[1], -q[2], q[3]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Penetration depth of two shapes turned by `qa` and `qb`; negative is
/// the gap between them. Two boxes not both axis-aligned are measured by
/// their separating axes: the least overlap when every axis overlaps, else
/// the most any axis separates, which is under the true gap only past an
/// edge or a corner, so a pair a hair farther apart there counts as touching.
pub fn depth_turned(a: Shape, pa: [f32; 3], qa: [f32; 4], b: Shape, pb: [f32; 3], qb: [f32; 4]) -> f32 {
    if qa == IDENTITY && qb == IDENTITY {
        return depth(a, pa, b, pb);
    }
    let d = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
    match (a, b) {
        (Shape::Sphere(ra), Shape::Sphere(rb)) => ra + rb - len(d),
        (Shape::Sphere(r), Shape::Box(h)) => sphere_box(r, rotate(conj(qb), d.map(|v| -v)), h),
        (Shape::Box(h), Shape::Sphere(r)) => sphere_box(r, rotate(conj(qa), d), h),
        (Shape::Box(ha), Shape::Box(hb)) => {
            let ax = [0, 1, 2].map(|i| rotate(qa, [(i == 0) as u8 as f32, (i == 1) as u8 as f32, (i == 2) as u8 as f32]));
            let bx = [0, 1, 2].map(|i| rotate(qb, [(i == 0) as u8 as f32, (i == 1) as u8 as f32, (i == 2) as u8 as f32]));
            let mut axes: Vec<[f32; 3]> = ax.iter().chain(bx.iter()).copied().collect();
            for u in ax {
                for v in bx {
                    let c = cross(u, v);
                    let l = len(c);
                    if l > 1e-4 {
                        axes.push(c.map(|x| x / l));
                    }
                }
            }
            let reach = |r: &[[f32; 3]; 3], h: [f32; 3], n: [f32; 3]| (0..3).map(|k| h[k] * dot(r[k], n).abs()).sum::<f32>();
            // The least overlap: the depth if every axis overlaps, else
            // minus the most any axis separates.
            axes.iter().map(|&n| reach(&ax, ha, n) + reach(&bx, hb, n) - dot(d, n).abs()).fold(f32::MAX, f32::min)
        }
    }
}

/// Penetration depth of two axis-aligned shapes; negative is the gap
/// between them.
pub fn depth(a: Shape, pa: [f32; 3], b: Shape, pb: [f32; 3]) -> f32 {
    let d = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
    match (a, b) {
        (Shape::Sphere(ra), Shape::Sphere(rb)) => ra + rb - len(d),
        (Shape::Box(ha), Shape::Box(hb)) => {
            let o = [ha[0] + hb[0] - d[0].abs(), ha[1] + hb[1] - d[1].abs(), ha[2] + hb[2] - d[2].abs()];
            if o.iter().all(|&v| v > 0.0) { o[0].min(o[1]).min(o[2]) } else { -len(o.map(|v| v.min(0.0))) }
        }
        (Shape::Sphere(r), Shape::Box(h)) => sphere_box(r, d.map(|v| -v), h),
        (Shape::Box(h), Shape::Sphere(r)) => sphere_box(r, d, h),
    }
}

/// A sphere of radius r at c relative to a box's centre.
fn sphere_box(r: f32, c: [f32; 3], h: [f32; 3]) -> f32 {
    let q = [c[0].clamp(-h[0], h[0]), c[1].clamp(-h[1], h[1]), c[2].clamp(-h[2], h[2])];
    let out = len([c[0] - q[0], c[1] - q[1], c[2] - q[2]]);
    if out > 0.0 { r - out } else { r + (h[0] - c[0].abs()).min(h[1] - c[1].abs()).min(h[2] - c[2].abs()) }
}

fn len(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// A touching pair: dynamic body a, and b, which is a dynamic body's index
/// when below the dynamic count and a static's index plus that count above.
pub struct Pair {
    pub a: usize,
    pub b: usize,
    pub depth: f32,
}

/// Every touching pair, found through a uniform grid over the dynamic bodies.
pub fn touching_pairs(shapes: &[Shape], pos: &[[f32; 3]], rot: &[[f32; 4]], statics: &[Spec]) -> Vec<Pair> {
    let n = shapes.len();
    let reach = shapes.iter().map(|s| s.bounding_radius()).fold(0.0f32, f32::max);
    let cell = 2.0 * reach + TOUCH_GAP;
    let key = |p: [f32; 3]| [(p[0] / cell).floor() as i32, (p[1] / cell).floor() as i32, (p[2] / cell).floor() as i32];
    let mut grid: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    for (i, &p) in pos.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i);
    }
    let mut pairs = Vec::new();
    for i in 0..n {
        let k = key(pos[i]);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(cell) = grid.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) else { continue };
                    for &j in cell {
                        if j <= i {
                            continue;
                        }
                        let d = depth_turned(shapes[i], pos[i], rot[i], shapes[j], pos[j], rot[j]);
                        if d > -TOUCH_GAP {
                            pairs.push(Pair { a: i, b: j, depth: d });
                        }
                    }
                }
            }
        }
        for (s, st) in statics.iter().enumerate() {
            let d = depth_turned(shapes[i], pos[i], rot[i], st.shape, st.pos, IDENTITY);
            if d > -TOUCH_GAP {
                pairs.push(Pair { a: i, b: n + s, depth: d });
            }
        }
    }
    pairs
}

#[derive(Clone, Debug, Default)]
pub struct Quality {
    pub bodies: usize,
    pub pairs: usize,
    /// Touching partners per body, statics included.
    pub contacts_per_body: f64,
    /// Bodies by touching-partner count: 0, 1, ... 7, and 8 or more.
    pub histogram: [usize; 9],
    /// Of the bodies resting on at least one other dynamic body, the fraction
    /// whose every such support is more than 0.1 off to the side: near 0 for
    /// a pile of columns, near 1 for a real one.
    pub not_columns: f64,
    pub supported: usize,
    pub pen_max: f32,
    /// Over touching pairs, counting a gap as zero.
    pub pen_mean: f32,
    /// Bodies at or above REST_SPEED.
    pub moving: usize,
    pub max_speed: f32,
    pub kinetic_energy: f64,
    pub mean_height: f64,
    pub escaped: usize,
    /// The most any box is turned from lying on a face, in degrees (0 to
    /// about 55): how far a stack has toppled.
    pub tilt: f32,
}

/// How fast any point of the body moves, at most: its speed and its turn
/// at its farthest point, as Box3D judges a body at rest.
pub fn speed(s: &State, shape: Shape) -> f32 {
    len(s.vel) + len(s.ang) * shape.bounding_radius()
}

/// Kinetic energy, moving and turning, of a body of mass `MASS`.
pub fn energy(s: &State, shape: Shape) -> f64 {
    let w = rotate(conj(s.rot), s.ang);
    let i = match shape {
        Shape::Sphere(r) => [0.4 * r * r; 3],
        Shape::Box([x, y, z]) => [(y * y + z * z) / 3.0, (x * x + z * z) / 3.0, (x * x + y * y) / 3.0],
    };
    let turning = (0..3).map(|k| i[k] * w[k] * w[k]).sum::<f32>();
    0.5 * crate::MASS as f64 * (dot(s.vel, s.vel) + turning) as f64
}

pub fn quality(scene: &Scene, shapes: &[Shape], state: &[State]) -> Quality {
    let pos: Vec<[f32; 3]> = state.iter().map(|s| s.pos).collect();
    let rot: Vec<[f32; 4]> = state.iter().map(|s| s.rot).collect();
    let n = pos.len();
    let pairs = touching_pairs(shapes, &pos, &rot, &scene.statics);
    let mut partners = vec![0usize; n];
    // The smallest sideways offset to any dynamic body below, per body.
    let mut below = vec![f32::INFINITY; n];
    for p in &pairs {
        partners[p.a] += 1;
        if p.b < n {
            partners[p.b] += 1;
            for (up, down) in [(p.a, p.b), (p.b, p.a)] {
                if pos[down][1] < pos[up][1] - 0.25 {
                    let off = ((pos[up][0] - pos[down][0]).powi(2) + (pos[up][2] - pos[down][2]).powi(2)).sqrt();
                    below[up] = below[up].min(off);
                }
            }
        }
    }
    let mut q = Quality { bodies: n, pairs: pairs.len(), ..Default::default() };
    for &c in &partners {
        q.histogram[c.min(8)] += 1;
    }
    q.contacts_per_body = partners.iter().sum::<usize>() as f64 / n.max(1) as f64;
    let supported: Vec<f32> = below.iter().copied().filter(|b| b.is_finite()).collect();
    q.supported = supported.len();
    q.not_columns = supported.iter().filter(|&&b| b > 0.1).count() as f64 / supported.len().max(1) as f64;
    q.pen_max = pairs.iter().map(|p| p.depth).fold(0.0, f32::max);
    q.pen_mean = pairs.iter().map(|p| p.depth.max(0.0)).sum::<f32>() / pairs.len().max(1) as f32;
    let (lo, hi) = scene.bounds;
    for (s, &shape) in state.iter().zip(shapes) {
        let p = &s.pos;
        let speed = speed(s, shape);
        q.max_speed = q.max_speed.max(speed);
        q.moving += (speed >= REST_SPEED) as usize;
        q.kinetic_energy += energy(s, shape);
        q.mean_height += p[1] as f64;
        if let Shape::Box(_) = shape {
            let up = (0..3)
                .map(|k| rotate(s.rot, [(k == 0) as u8 as f32, (k == 1) as u8 as f32, (k == 2) as u8 as f32])[1].abs())
                .fold(0.0, f32::max);
            q.tilt = q.tilt.max(up.min(1.0).acos().to_degrees());
        }
        if (0..3).any(|k| p[k] < lo[k] || p[k] > hi[k]) {
            q.escaped += 1;
        }
    }
    q.mean_height /= n.max(1) as f64;
    q
}

pub struct Run {
    pub backend: String,
    pub solver: String,
    /// Wall time of each step() call, in microseconds.
    pub step_us: Vec<f64>,
    /// Mean per-stage time per step over the run, in microseconds.
    pub stages: Vec<(&'static str, f64)>,
    /// The first step from which the top speed stays below REST_SPEED.
    pub settled_at: Option<usize>,
    pub quality: Quality,
    /// The engine's own touching count at the end.
    pub native_touching: usize,
    /// The deepest overlap, and the greatest mean, at any look (every
    /// `LOOK` steps): sinking while it settles, landing included.
    pub pen_max_during: f32,
    pub pen_mean_during: f32,
    /// How far the last body added (a stack's top) is from where it began.
    pub top_moved: f32,
}

/// Steps between looks at overlap during a run: often enough to see a
/// pile sink, rarely enough not to cost more than the step at 10 000.
pub const LOOK: usize = 10;

impl Run {
    pub fn phase_ms(&self, range: &std::ops::Range<usize>) -> f64 {
        let r = range.start.min(self.step_us.len())..range.end.min(self.step_us.len());
        let s = &self.step_us[r.clone()];
        s.iter().sum::<f64>() / s.len().max(1) as f64 / 1000.0
    }
}

pub fn run(scene: &Scene, backend: &mut dyn Backend) -> Run {
    let builds = backend.builds(scene);
    if !builds {
        backend.add(&scene.statics);
    }
    let mut shapes = Vec::with_capacity(scene.n);
    let mut state = Vec::with_capacity(scene.n);
    let mut step_us = Vec::with_capacity(scene.steps);
    let mut stages: Vec<(&'static str, f64)> = Vec::new();
    let mut last_moving = None;
    let (mut pen_max_during, mut pen_mean_during) = (0.0f32, 0.0f32);
    for step in 0..scene.steps {
        if let Some(batch) = scene.spawn.get(step)
            && !batch.is_empty()
        {
            if !builds {
                backend.add(batch);
            }
            shapes.extend(batch.iter().map(|s| s.shape));
        }
        let t = Instant::now();
        backend.step(DT);
        step_us.push(t.elapsed().as_secs_f64() * 1e6);
        for (i, (name, us)) in backend.stages().into_iter().enumerate() {
            if stages.len() <= i {
                stages.push((name, 0.0));
            }
            stages[i].1 += us;
        }
        backend.state(&mut state);
        if state.iter().zip(&shapes).any(|(s, &shape)| speed(s, shape) >= REST_SPEED) {
            last_moving = Some(step);
        }
        if step % LOOK == LOOK - 1 {
            let q = quality(scene, &shapes, &state);
            (pen_max_during, pen_mean_during) = (pen_max_during.max(q.pen_max), pen_mean_during.max(q.pen_mean));
        }
    }
    for s in &mut stages {
        s.1 /= scene.steps as f64;
    }
    let settled_at = match last_moving {
        Some(s) if s + 1 >= scene.steps => None,
        Some(s) => Some(s + 1),
        None => Some(0),
    };
    Run {
        backend: backend.name(),
        solver: backend.solver(),
        step_us,
        stages,
        settled_at,
        quality: quality(scene, &shapes, &state),
        native_touching: backend.touching(),
        pen_max_during,
        pen_mean_during,
        top_moved: match (scene.spawn.iter().flatten().last(), state.last()) {
            (Some(a), Some(b)) => len([b.pos[0] - a.pos[0], b.pos[1] - a.pos[1], b.pos[2] - a.pos[2]]),
            _ => 0.0,
        },
    }
}
