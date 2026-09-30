//! The scenes, as plain data: statics, the bodies present at step 0, the
//! bodies each later step adds, and the phases a run is timed in. Built from
//! a seeded RNG so every engine gets the identical scene. Shared by the
//! comparison (//engine/std/physics3d/compare, which builds them in each engine) and the
//! scene mod `pile3d` (which builds them in ours, in the engine, from this
//! same code).

use std::ops::Range;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Sphere(f32),
    /// Half-extents, along the body's own axes.
    Box([f32; 3]),
}

impl Shape {
    /// The radius of a sphere around the shape: what the harness's pair grid
    /// is sized by.
    pub fn bounding_radius(&self) -> f32 {
        match *self {
            Shape::Sphere(r) => r,
            Shape::Box([x, y, z]) => (x * x + y * y + z * z).sqrt(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Spec {
    pub shape: Shape,
    pub pos: [f32; 3],
    /// Ignored for a fixed body.
    pub vel: [f32; 3],
    pub fixed: bool,
    /// A unit quaternion (x, y, z, w): a turned static is a ramp.
    pub rot: [f32; 4],
    /// The piles and stacks have the comparison's (friction 0.5, no
    /// restitution, mass 1); the behaviour scenes their own, the same on a
    /// body and on what it meets, so every engine's rule for mixing two
    /// (Rapier's mean, Jolt's and ours' geometric mean, Box3D's) gives it.
    pub friction: f32,
    pub restitution: f32,
    /// Ignored for a fixed body.
    pub mass: f32,
}

impl Spec {
    /// Unturned, with the comparison's friction, no restitution, mass 1.
    pub fn new(shape: Shape, pos: [f32; 3], fixed: bool) -> Spec {
        Spec { shape, pos, vel: [0.0; 3], fixed, rot: [0.0, 0.0, 0.0, 1.0], friction: 0.5, restitution: 0.0, mass: 1.0 }
    }
}

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
    /// Boxes a unit long but thin: what turning does to bodies whose
    /// inertia isn't the same about every axis, and whose bounds a sphere
    /// fits badly.
    PlankPile,
    Rain,
    /// Unit cubes stacked `n` high on a floor, each set off by up to 0.04
    /// so the column leans a little: whether turning boxes stand it or rock
    /// on their edges (2D's `Scene::Stack`).
    Stack,
    /// A unit cube on a ramp 20° steep, friction 0.6 on both (tan 20° =
    /// 0.36): it holds. The ramp scenes and the next two are the behaviour
    /// scenes (physics.md, "Quality beyond settling"), whose n says
    /// something else or nothing.
    RampHold,
    /// A unit cube on a ramp 30° steep, friction 0.2: it slides at
    /// g (sin θ − μ cos θ).
    RampSlide,
    /// A sphere of radius 0.5 on a ramp 30° steep, friction 0.6, above
    /// (2/7) tan θ: it rolls without slipping at (5/7) g sin θ.
    RampRoll,
    /// A sphere of radius 0.5 dropped 5 onto a floor, both of restitution n
    /// hundredths and no friction: it rebounds to e² of the drop.
    Bounce,
    /// A unit cube n times as heavy on a light one (mass 1) on a floor.
    Ratio,
    /// The pile's drop with every body its own shape and size, by its
    /// index: a sphere or a box, radius or half extents 0.25 to 0.5 (a
    /// box's three apart). One material, the pile's, since the engines mix
    /// two frictions by different rules (Rapier the mean, the others the
    /// geometric mean), which a pile of mixed materials would measure.
    Mixed,
    /// One bounce of the bounce families (engine/std/physics3d/compare, `bounces.rs`):
    /// n is a `Hit`, packed (`Hit::pack`).
    Hit,
}

/// What a `Kind::Hit` throws, and at what.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// A sphere of radius 0.5 onto the floor.
    Sphere,
    /// A unit cube landing flat: four points.
    Cube,
    /// A unit cube turned `TILT` about z, landing on an edge.
    Edge,
    /// A unit cube turned `TILT` about z and again about x, landing on a
    /// corner.
    Corner,
    /// A sphere onto another, both free, the second `ratio` times as heavy:
    /// the first coming down onto the second coming up.
    Spheres,
}

pub const TARGETS: [Target; 5] = [Target::Sphere, Target::Cube, Target::Edge, Target::Corner, Target::Spheres];

impl Target {
    pub fn name(self) -> &'static str {
        ["sphere", "cube", "edge", "corner", "spheres"][self as usize]
    }

    /// Whether it lands on the static floor, not on a second body.
    pub fn floor(self) -> bool {
        self != Target::Spheres
    }
}

/// How far `Target::Edge`'s and `Corner`'s cubes are turned, each way.
pub const TILT: f32 = std::f32::consts::FRAC_PI_6;

/// y is up: the comparison's gravity, and every scene's but a bounce's.
pub const EARTH: [f32; 3] = [0.0, -9.81, 0.0];

/// A bounce (`Kind::Hit`), as 2D's `Scene::Hit`: a body arriving at the
/// floor, or at a second body, at closing speed `v` along the normal and
/// `along` it the floor's way (x), `phase` of a step after a whole number
/// of steps, restitution `e` and friction `mu` on both, under gravity `g`
/// (down), at `sub` substeps in every engine (0: each its own), over 20 s
/// when `series`, or the one bounce. Packed into a scene's n (`pack`), each
/// field rounded to what the families use: e to hundredths, v, along and
/// mu to tenths, phase to quarters, and g one of `GRAVITIES`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub target: Target,
    pub e: f32,
    pub v: f32,
    pub phase: f32,
    pub g: f32,
    pub along: f32,
    pub mu: f32,
    pub ratio: f32,
    pub sub: u32,
    pub series: bool,
}

/// A `Hit` where a family doesn't say: a lossless sphere dropped from 5
/// onto a frictionless floor, at the comparison's gravity.
pub const HIT: Hit =
    Hit { target: Target::Sphere, e: 1.0, v: 9.9, phase: 0.0, g: 9.81, along: 0.0, mu: 0.0, ratio: 1.0, sub: 0, series: false };

/// Each field's digits in a packed `Hit`, lowest first: 17, as many as a
/// usize holds with room.
const WIDTHS: [u32; 10] = [1, 3, 3, 1, 1, 3, 2, 1, 1, 1];

/// The gravities a `Hit` can have, by their place: none, the comparison's,
/// and others.
pub const GRAVITIES: [f32; 6] = [0.0, 9.81, 40.0, 5.0, 20.0, 80.0];

impl Hit {
    pub fn pack(&self) -> usize {
        let fields = [
            self.target as usize,
            (self.e * 100.0).round() as usize,
            (self.v * 10.0).round() as usize,
            (self.phase * 4.0).round() as usize,
            GRAVITIES.iter().position(|g| *g == self.g).unwrap_or_else(|| panic!("no gravity {} in GRAVITIES", self.g)),
            (self.along * 10.0).round() as usize,
            (self.mu * 10.0).round() as usize,
            self.ratio.round() as usize,
            self.sub as usize,
            self.series as usize,
        ];
        let (mut n, mut scale) = (0usize, 1usize);
        for (f, w) in fields.iter().zip(WIDTHS) {
            assert!(*f < 10usize.pow(w), "a field of {self:?} is too big to pack");
            n += f * scale;
            scale *= 10usize.pow(w);
        }
        n
    }

    pub fn unpack(mut n: usize) -> Hit {
        let mut f = [0usize; 10];
        for (x, w) in f.iter_mut().zip(WIDTHS) {
            *x = n % 10usize.pow(w);
            n /= 10usize.pow(w);
        }
        Hit {
            target: TARGETS[f[0]],
            e: f[1] as f32 / 100.0,
            v: f[2] as f32 / 10.0,
            phase: f[3] as f32 / 4.0,
            g: GRAVITIES[f[4]],
            along: f[5] as f32 / 10.0,
            mu: f[6] as f32 / 10.0,
            ratio: f[7] as f32,
            sub: f[8] as u32,
            series: f[9] != 0,
        }
    }

    /// The flight to the contact, as 2D's `Hit::flight`: the steps it takes,
    /// rounded up, its time, and the speed toward it and the gap at the
    /// start, at the comparison's step.
    pub fn flight(&self, dt: f32) -> (usize, f32, f32, f32) {
        let g = if self.target.floor() { self.g } else { 0.0 };
        let t = if g > 0.0 {
            let most = self.v / g;
            let whole = (most / dt - self.phase).floor().max(0.0);
            ((whole + self.phase) * dt).min(most)
        } else {
            (3.0 + self.phase) * dt
        };
        let u0 = self.v - g * t;
        ((t / dt).ceil() as usize, t, u0, u0 * t + 0.5 * g * t * t)
    }

    /// A one-line name, for the families' tables.
    pub fn text(&self) -> String {
        let mut s = format!("{} e={} v={} g={}", self.target.name(), self.e, self.v, self.g);
        for (k, x, default) in [("phase", self.phase, 0.0), ("along", self.along, 0.0), ("mu", self.mu, 0.0), ("ratio", self.ratio, 1.0)] {
            if x != default {
                s += &format!(" {k}={x}");
            }
        }
        if self.sub > 0 {
            s += &format!(" sub={}", self.sub);
        }
        if self.series {
            s += " series";
        }
        s
    }
}

/// `q` (x, y, z, w) turning `v`.
pub fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let (u, w) = ([q[0], q[1], q[2]], q[3]);
    let cross = |a: [f32; 3], b: [f32; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    let t = cross(u, v).map(|x| 2.0 * x);
    let ut = cross(u, t);
    [0, 1, 2].map(|k| v[k] + w * t[k] + ut[k])
}

/// The rotation turning by `a`, then by `b`.
fn then(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let (x1, y1, z1, w1) = (b[0], b[1], b[2], b[3]);
    let (x2, y2, z2, w2) = (a[0], a[1], a[2], a[3]);
    [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]
}

/// The corners of a cube of half extent `h` turned by `q`, from its centre.
pub fn corners(q: [f32; 4], h: f32) -> [[f32; 3]; 8] {
    std::array::from_fn(|i| rotate(q, [1, 2, 4].map(|bit| if i / bit % 2 == 1 { h } else { -h })))
}

fn hit(n: usize) -> Scene {
    let h = Hit::unpack(n);
    let dt = 1.0 / 60.0;
    let (steps, t, u0, s) = h.flight(dt);
    let material = |spec: Spec| Spec { friction: h.mu, restitution: h.e, ..spec };
    let gravity = [0.0, -h.g, 0.0];
    let len = if h.series { 1200 } else { steps + 45 };
    if !h.target.floor() {
        let m = 1.0 + h.ratio;
        let a = Spec { vel: [0.0, -h.v * h.ratio / m, 0.0], ..Spec::new(Shape::Sphere(RADIUS), [0.0, 2.0 * RADIUS + s, 0.0], false) };
        let b = Spec { vel: [0.0, h.v / m, 0.0], mass: h.ratio, ..Spec::new(Shape::Sphere(RADIUS), [0.0; 3], false) };
        let anywhere = ([-100.0, f32::MIN, -100.0], [100.0, f32::MAX, 100.0]);
        return Scene { gravity, substeps: h.sub, bounds: anywhere, ..one(Kind::Hit, n, vec![], vec![material(a), material(b)], len) };
    }
    let half = |a: f32| [0.0, 0.0, (a / 2.0).sin(), (a / 2.0).cos()];
    let about_x = [(TILT / 2.0).sin(), 0.0, 0.0, (TILT / 2.0).cos()];
    let (shape, rot) = match h.target {
        Target::Sphere => (Shape::Sphere(RADIUS), [0.0, 0.0, 0.0, 1.0]),
        Target::Cube => (Shape::Box([HALF; 3]), [0.0, 0.0, 0.0, 1.0]),
        Target::Edge => (Shape::Box([HALF; 3]), half(TILT)),
        _ => (Shape::Box([HALF; 3]), then(half(TILT), about_x)),
    };
    let reach = match shape {
        Shape::Sphere(r) => r,
        Shape::Box(_) => corners(rot, HALF).iter().map(|c| -c[1]).fold(0.0, f32::max),
    };
    let body = Spec { rot, vel: [h.along, -u0, 0.0], ..Spec::new(shape, [-h.along * t, s + reach, 0.0], false) };
    let floor = Spec { friction: h.mu, restitution: h.e, ..fixed_box([0.0, -0.5, 0.0], [100.0, 0.5, 100.0]) };
    let above = ([-100.0, -0.1, -100.0], [100.0, f32::MAX, 100.0]);
    Scene { gravity, substeps: h.sub, bounds: above, ..one(Kind::Hit, n, vec![floor], vec![material(body)], len) }
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::SpherePile => "spheres",
            Kind::BoxPile => "boxes",
            Kind::PlankPile => "planks",
            Kind::Rain => "rain",
            Kind::Stack => "stack",
            Kind::RampHold => "ramp_hold",
            Kind::RampSlide => "ramp_slide",
            Kind::RampRoll => "ramp_roll",
            Kind::Bounce => "bounce",
            Kind::Ratio => "ratio",
            Kind::Mixed => "mixed",
            Kind::Hit => "hit",
        }
    }

    pub fn parse(s: &str) -> Option<Kind> {
        KINDS.into_iter().find(|k| k.name() == s)
    }

    /// The ramp scenes' slope in degrees and friction.
    pub fn ramp(self) -> Option<(f32, f32)> {
        match self {
            Kind::RampHold => Some((20.0, 0.6)),
            Kind::RampSlide => Some((30.0, 0.2)),
            Kind::RampRoll => Some((30.0, 0.6)),
            _ => None,
        }
    }
}

/// Every kind, in an order that only grows: the scene mod keeps a kind as
/// its place here.
pub const KINDS: [Kind; 12] = [
    Kind::SpherePile,
    Kind::BoxPile,
    Kind::PlankPile,
    Kind::Rain,
    Kind::Stack,
    Kind::RampHold,
    Kind::RampSlide,
    Kind::RampRoll,
    Kind::Bounce,
    Kind::Ratio,
    Kind::Mixed,
    Kind::Hit,
];

/// How far above the floor `Kind::Bounce` drops its ball.
pub const DROP: f32 = 5.0;

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
    /// `EARTH` but for a bounce.
    pub gravity: [f32; 3],
    /// The substeps every engine solves at, where a bounce sets them; 0
    /// for each engine's own.
    pub substeps: u32,
}

pub const RADIUS: f32 = 0.5;
pub const HALF: f32 = 0.5;
pub const PLANK: [f32; 3] = [0.5, 0.125, 0.25];

pub fn build(kind: Kind, n: usize) -> Scene {
    match kind {
        Kind::SpherePile => pile(kind, n, |_| Shape::Sphere(RADIUS)),
        Kind::BoxPile => pile(kind, n, |_| Shape::Box([HALF; 3])),
        Kind::PlankPile => pile(kind, n, |_| Shape::Box(PLANK)),
        Kind::Rain => rain(n),
        Kind::Stack => stack(n),
        Kind::RampHold | Kind::RampSlide | Kind::RampRoll => ramp(kind),
        Kind::Bounce => bounce(n),
        Kind::Ratio => ratio(n),
        Kind::Mixed => pile(kind, n, mixed),
        Kind::Hit => hit(n),
    }
}

/// A mixed pile's body `i`, the same in every engine: its own seeded draw.
fn mixed(i: usize) -> Shape {
    let mut rng = Rng::new(0x3a1d ^ (i as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15));
    if rng.next_u64().is_multiple_of(2) {
        Shape::Sphere(rng.range(0.25, 0.5))
    } else {
        Shape::Box([rng.range(0.25, 0.5), rng.range(0.25, 0.5), rng.range(0.25, 0.5)])
    }
}

fn fixed_box(pos: [f32; 3], half: [f32; 3]) -> Spec {
    Spec::new(Shape::Box(half), pos, true)
}

/// One behaviour scene: its statics and bodies at step 0, `steps` long,
/// timed as a whole; its `n` is what `build` was given, which says which.
fn one(kind: Kind, n: usize, statics: Vec<Spec>, bodies: Vec<Spec>, steps: usize) -> Scene {
    Scene {
        kind,
        n,
        statics,
        spawn: vec![bodies],
        steps,
        phases: vec![("all", 1..steps)],
        bounds: ([-50.0, -0.1, -50.0], [50.0, f32::MAX, 50.0]),
        gravity: EARTH,
        substeps: 0,
    }
}

/// A static ramp turned θ about z, down toward +x (y is up), 40 long and
/// 10 wide, and the body 12 up it from its middle, its bottom touching it.
fn ramp(kind: Kind) -> Scene {
    let (deg, mu) = kind.ramp().expect("a ramp");
    let a = -deg.to_radians();
    let rot = [0.0, 0.0, (a / 2.0).sin(), (a / 2.0).cos()];
    // Down the slope, and out of its top face.
    let (t, n) = ([a.cos(), a.sin(), 0.0], [-a.sin(), a.cos(), 0.0]);
    let centre = [0.0, 20.0, 0.0];
    let ramp = Spec { rot, friction: mu, ..Spec::new(Shape::Box([20.0, 0.5, 5.0]), centre, true) };
    let at = |d: f32, up: f32| [0, 1, 2].map(|k| centre[k] + d * t[k] + up * n[k]);
    let (shape, body_rot) =
        if kind == Kind::RampRoll { (Shape::Sphere(RADIUS), [0.0, 0.0, 0.0, 1.0]) } else { (Shape::Box([HALF; 3]), rot) };
    let body = Spec { rot: body_rot, friction: mu, ..Spec::new(shape, at(-12.0, 0.5 + 0.5), false) };
    Scene { bounds: ([-50.0, -50.0, -50.0], [50.0, f32::MAX, 50.0]), ..one(kind, 1, vec![ramp], vec![body], 120) }
}

fn floor(friction: f32, restitution: f32) -> Spec {
    Spec { friction, restitution, ..fixed_box([0.0, -0.5, 0.0], [10.0, 0.5, 10.0]) }
}

fn bounce(hundredths: usize) -> Scene {
    let e = hundredths as f32 / 100.0;
    let ball = Spec { friction: 0.0, restitution: e, ..Spec::new(Shape::Sphere(RADIUS), [0.0, DROP + RADIUS, 0.0], false) };
    one(Kind::Bounce, hundredths, vec![floor(0.0, e)], vec![ball], if hundredths >= 100 { 1200 } else { 300 })
}

fn ratio(ratio: usize) -> Scene {
    let light = Spec { friction: 0.6, ..Spec::new(Shape::Box([HALF; 3]), [0.0, 0.5, 0.0], false) };
    let heavy = Spec { mass: ratio as f32, ..Spec { pos: [0.0, 1.5, 0.0], ..light } };
    one(Kind::Ratio, ratio, vec![floor(0.6, 0.0)], vec![light, heavy], 600)
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
        bodies.push(Spec::new(shape(i), [x, y, z], false));
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
        gravity: EARTH,
        substeps: 0,
    }
}

/// Spheres and boxes, alternating, dropped a batch a step at random spots
/// over a square, onto a floor wide enough to hold the heap, with low walls
/// round it: turning, spheres roll, in every engine, and without walls about
/// one in twenty rolled off the edge.
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
            this.push(Spec::new(shape, [x, height, z], false));
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
        statics: vec![
            fixed_box([0.0, -0.5, 0.0], [floor, 0.5, floor]),
            fixed_box([floor + 0.5, 1.0, 0.0], [0.5, 1.0, floor + 1.0]),
            fixed_box([-floor - 0.5, 1.0, 0.0], [0.5, 1.0, floor + 1.0]),
            fixed_box([0.0, 1.0, floor + 0.5], [floor + 1.0, 1.0, 0.5]),
            fixed_box([0.0, 1.0, -floor - 0.5], [floor + 1.0, 1.0, 0.5]),
        ],
        spawn,
        steps,
        phases: vec![("raining", 1..last_spawn), ("after rain", last_spawn..steps), ("settled", steps - 100..steps)],
        bounds: ([-floor, -0.1, -floor], [floor, f32::MAX, floor]),
        gravity: EARTH,
        substeps: 0,
    }
}

fn stack(n: usize) -> Scene {
    let off = |i: usize, k: usize| ((i * k) % 9) as f32 / 100.0 - 0.04;
    let bodies = (0..n).map(|i| Spec::new(Shape::Box([HALF; 3]), [off(i, 7919), 0.5 + i as f32, off(i, 104_729)], false)).collect();
    let steps = 1000;
    Scene {
        kind: Kind::Stack,
        n,
        statics: vec![fixed_box([0.0, -0.5, 0.0], [10.0, 0.5, 10.0])],
        spawn: vec![bodies],
        steps,
        phases: vec![("standing", 1..steps)],
        bounds: ([-10.0, -0.1, -10.0], [10.0, f32::MAX, 10.0]),
        gravity: EARTH,
        substeps: 0,
    }
}
