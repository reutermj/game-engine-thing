//! The scenes, as plain data: statics, the bodies present at step 0, the
//! bodies each later step adds, and the phases a run is timed in. Built from
//! a seeded RNG so every engine gets the identical scene.

use std::ops::Range;

use crate::{Shape, Spec};

/// splitmix64: enough randomness for jitter, and no crate to pin.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in [lo, hi).
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let unit = (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32;
        lo + (hi - lo) * unit
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    SpherePile,
    BoxPile,
    Rain,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::SpherePile => "spheres",
            Kind::BoxPile => "boxes",
            Kind::Rain => "rain",
        }
    }

    pub fn parse(s: &str) -> Option<Kind> {
        [Kind::SpherePile, Kind::BoxPile, Kind::Rain].into_iter().find(|k| k.name() == s)
    }
}

pub struct Scene {
    pub kind: Kind,
    pub n: usize,
    pub statics: Vec<Spec>,
    /// Bodies added by step: spawn[k] goes in before step k runs.
    pub spawn: Vec<Vec<Spec>>,
    pub steps: usize,
    /// Timing windows, as step ranges.
    pub phases: Vec<(&'static str, Range<usize>)>,
    /// A dynamic body outside this box at the end has escaped (tunnelled
    /// through a wall or the floor, or fallen off it).
    pub bounds: ([f32; 3], [f32; 3]),
}

pub const RADIUS: f32 = 0.5;
pub const HALF: f32 = 0.5;

pub fn build(kind: Kind, n: usize) -> Scene {
    match kind {
        Kind::SpherePile => pile(kind, n, |_| Shape::Sphere(RADIUS)),
        Kind::BoxPile => pile(kind, n, |_| Shape::Box([HALF; 3])),
        Kind::Rain => rain(n),
    }
}

fn fixed_box(pos: [f32; 3], half: [f32; 3]) -> Spec {
    Spec { shape: Shape::Box(half), pos, vel: [0.0; 3], fixed: true }
}

/// A walled box whose floor holds fewer bodies than are dropped into it, so
/// they stack. The target depth grows with n, 10 layers at 1k and 25 at 10k,
/// so the pile is tall as well as wide at both sizes. Each layer of the drop
/// lattice is shifted by its own random fraction of a cell, and every body
/// jittered, so bodies land resting across several below instead of in
/// columns. (A fixed half-cell stagger on alternate layers was not enough:
/// layers two apart lined up, and locked rotation plus friction lets a sphere
/// balance up to 26 degrees off the top of another, so 60% of the spheres
/// ended up standing on one nearly straight below.)
fn pile(kind: Kind, n: usize, shape: impl Fn(usize) -> Shape) -> Scene {
    let layers = (10.0 * (n as f32 / 1000.0).powf(0.4)).max(2.0);
    let inner = (n as f32 / layers).sqrt().ceil().max(3.0);
    let half_inner = inner / 2.0;

    // Cell 1.25 with jitter under 0.125 keeps unit bodies apart at spawn.
    let cell = 1.25;
    let jitter = 0.1;
    let stagger = cell / 2.0;
    let per_axis = (((inner - 1.0 - 2.0 * jitter - stagger) / cell).floor() as usize + 1).max(1);
    let per_layer = per_axis * per_axis;
    let lattice_layers = n.div_ceil(per_layer);
    let layer_height = 1.1;
    let top = 1.0 + lattice_layers as f32 * layer_height;

    let mut rng = Rng::new(0x5eed ^ n as u64 ^ ((kind as u64) << 32));
    let span = (per_axis - 1) as f32 * cell + stagger;
    let shifts: Vec<(f32, f32)> = (0..lattice_layers).map(|_| (rng.range(0.0, stagger), rng.range(0.0, stagger))).collect();
    let mut bodies = Vec::with_capacity(n);
    for i in 0..n {
        let layer = i / per_layer;
        let (row, col) = ((i % per_layer) / per_axis, i % per_axis);
        let (sx, sz) = shifts[layer];
        let x = -span / 2.0 + col as f32 * cell + sx + rng.range(-jitter, jitter);
        let z = -span / 2.0 + row as f32 * cell + sz + rng.range(-jitter, jitter);
        let y = 1.0 + layer as f32 * layer_height;
        bodies.push(Spec { shape: shape(i), pos: [x, y, z], vel: [0.0; 3], fixed: false });
    }

    let wall_h = top + 2.0;
    let t = 0.5;
    let outer = half_inner + 2.0 * t;
    let statics = vec![
        fixed_box([0.0, -t, 0.0], [outer, t, outer]),
        fixed_box([half_inner + t, wall_h / 2.0, 0.0], [t, wall_h / 2.0, outer]),
        fixed_box([-half_inner - t, wall_h / 2.0, 0.0], [t, wall_h / 2.0, outer]),
        fixed_box([0.0, wall_h / 2.0, half_inner + t], [outer, wall_h / 2.0, t]),
        fixed_box([0.0, wall_h / 2.0, -half_inner - t], [outer, wall_h / 2.0, t]),
    ];

    // Long enough at 10k for a 50 m drop to land and settle.
    let steps = if n <= 2000 { 1000 } else { 1500 };
    Scene {
        kind,
        n,
        statics,
        spawn: vec![bodies],
        steps,
        // Step 0 builds the broad phase from scratch in every engine, so it
        // is in no window.
        phases: vec![("falling", 1..61), ("settling", 300..400), ("settled", steps - 100..steps)],
        bounds: ([-half_inner - 0.1, -0.1, -half_inner - 0.1], [half_inner + 0.1, f32::MAX, half_inner + 0.1]),
    }
}

/// Spheres and boxes, alternating, dropped a batch a step at random spots
/// over a square, onto a floor wide enough to hold the heap without walls.
/// Spots are continuous, not a grid, so bodies land off-centre on each other
/// instead of stacking into columns. A spot within 1.1 (sideways) of anything
/// spawned in the last 30 steps is refused, since a body at rest falls only
/// 1.2 m in 30 steps, so no two spawn overlapping.
fn rain(n: usize) -> Scene {
    let spread = (0.3 * (n as f32).sqrt()).max(5.0);
    let floor = spread + 10.0;
    let height = 15.0;
    let spawn_steps = 300;
    let batch = n.div_ceil(spawn_steps);
    let reserve = 30;
    let clear = 1.1f32;

    let mut rng = Rng::new(0x7a1 ^ n as u64);
    let mut recent: Vec<(usize, f32, f32)> = Vec::new();
    let mut spawn = Vec::new();
    let mut placed = 0;
    let mut step = 0;
    while placed < n {
        recent.retain(|&(at, _, _)| at + reserve > step);
        let mut this = Vec::new();
        let want = batch.min(n - placed);
        let mut tries = 0;
        while this.len() < want && tries < 50 * want {
            tries += 1;
            let (x, z) = (rng.range(-spread, spread), rng.range(-spread, spread));
            if recent.iter().any(|&(_, rx, rz)| (rx - x).powi(2) + (rz - z).powi(2) < clear * clear) {
                continue;
            }
            recent.push((step, x, z));
            let shape = if (placed + this.len()) % 2 == 0 { Shape::Sphere(RADIUS) } else { Shape::Box([HALF; 3]) };
            this.push(Spec { shape, pos: [x, height, z], vel: [0.0; 3], fixed: false });
        }
        placed += this.len();
        spawn.push(this);
        step += 1;
    }
    let last_spawn = spawn.len();
    let steps = last_spawn + 400;
    Scene {
        kind: Kind::Rain,
        n,
        statics: vec![fixed_box([0.0, -0.5, 0.0], [floor, 0.5, floor])],
        spawn,
        steps,
        phases: vec![("raining", 1..last_spawn), ("after rain", last_spawn..steps), ("settled", steps - 100..steps)],
        bounds: ([-floor, -0.1, -floor], [floor, f32::MAX, floor]),
    }
}
