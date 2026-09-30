//! The 3D behaviour scenes (`Kind::RampHold` to `Kind::Ratio`): what a
//! player sees a body do, measured from each engine's states every step,
//! the same for every engine, as 2D's `behave.rs` measures its own
//! (physics.md, "Quality beyond settling"). What `bench -- ... --behave`
//! prints and `:behaviour_test` bounds.

use crate::measure::{self, REST_SPEED};
use crate::scenes::{self, DROP, Hit, Kind, Scene, Shape, Target};
use crate::{Backend, DT, GRAVITY, State};

/// One engine on one scene: named values, in the order they were found.
#[derive(Clone, Debug, Default)]
pub struct Behaviour {
    pub backend: String,
    pub values: Vec<(&'static str, f64)>,
}

impl Behaviour {
    pub fn get(&self, name: &str) -> f64 {
        self.values.iter().find(|(k, _)| *k == name).map(|(_, v)| *v).unwrap_or_else(|| panic!("no {name} in {:?}", self.values))
    }

    fn put(&mut self, name: &'static str, v: f64) {
        self.values.push((name, v));
    }
}

/// Never at rest: past any bound.
pub const NEVER: f64 = f64::INFINITY;

/// Closing speeds below this don't bounce: physics3d's (its solver's
/// `BOUNCE_THRESHOLD`), and Box3D's and Box2D's default.
pub const BOUNCE_THRESHOLD: f32 = 1.0;

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// `backend` on `scene` from the start, every step read.
pub fn behave(scene: &Scene, backend: &mut dyn Backend) -> Behaviour {
    if !backend.builds(scene) {
        backend.add(&scene.statics);
        backend.add(&scene.spawn[0]);
    }
    let shapes: Vec<Shape> = scene.spawn[0].iter().map(|s| s.shape).collect();
    // From the first step's end, not before it: ours spawns the scene's
    // bodies in that step, as a game would, where the others are given them
    // before it.
    let mut t: Vec<Vec<State>> = Vec::with_capacity(scene.steps + 1);
    // A bounce is measured from the step it met in, which may be the first:
    // its start is the scene's, as every engine is given it.
    if scene.kind == Kind::Hit {
        let start = scene.spawn[0].iter().filter(|s| !s.fixed);
        t.push(start.map(|s| State { pos: s.pos, vel: s.vel, rot: s.rot, ang: [0.0; 3] }).collect());
    }
    let mut state = Vec::new();
    for _ in 0..scene.steps {
        backend.step(DT);
        backend.state(&mut state);
        t.push(state.clone());
    }
    let mut b = Behaviour { backend: backend.name(), values: Vec::new() };
    let fastest = |s: &[State]| s.iter().zip(&shapes).map(|(s, &shape)| measure::speed(s, shape)).fold(0.0, f32::max);
    let g = -GRAVITY[1];
    let (first, end) = (&t[0], t.last().unwrap());
    match scene.kind {
        Kind::RampHold | Kind::RampSlide | Kind::RampRoll => {
            let (deg, mu) = scene.kind.ramp().unwrap();
            let a = deg.to_radians();
            // Down the slope, and out of it, as `scenes::ramp` turns it.
            let (tangent, normal) = ([a.cos(), -a.sin(), 0.0], [a.sin(), a.cos(), 0.0]);
            let expected = match scene.kind {
                Kind::RampRoll => 5.0 / 7.0 * g * a.sin(),
                _ => (g * (a.sin() - mu * a.cos())).max(0.0),
            };
            b.put("expected a", expected as f64);
            b.put("a", ((dot(t[90][0].vel, tangent) - dot(t[30][0].vel, tangent)) / (60.0 * DT)) as f64);
            let moved = [0, 1, 2].map(|k| end[0].pos[k] - first[0].pos[k]);
            b.put("crept", dot(moved, tangent).abs() as f64);
            if let Shape::Sphere(r) = shapes[0] {
                // The contact point's speed along the slope: v + ω × (−r n).
                let slip = |s: &State| dot([0, 1, 2].map(|k| s.vel[k] + cross(s.ang, normal.map(|x| -r * x))[k]), tangent).abs();
                b.put("slip", (t[30..90].iter().map(|s| slip(&s[0])).sum::<f32>() / 60.0) as f64);
            }
            let off = t.iter().map(|s| dot(s[0].vel, normal).abs()).fold(0.0, f32::max);
            b.put("off the slope", off as f64);
        }
        Kind::Bounce => {
            let r = match shapes[0] {
                Shape::Sphere(r) => r,
                Shape::Box(_) => unreachable!("a ball"),
            };
            let (mut apexes, mut rising, mut top) = (Vec::new(), false, 0f32);
            for s in &t[1..] {
                let (h, vy) = (s[0].pos[1] - r, s[0].vel[1]);
                if !rising && vy > 0.0 {
                    (rising, top) = (true, h);
                } else if rising {
                    top = top.max(h);
                    if vy < 0.0 {
                        apexes.push(top / DROP);
                        rising = false;
                    }
                }
            }
            let e = scene.n as f32 / 100.0;
            b.put("expected", (e * e) as f64);
            b.put("first apex", apexes.first().copied().unwrap_or(0.0) as f64);
            b.put("bounces", apexes.len() as f64);
            b.put("most apex", apexes.iter().copied().fold(0.0, f32::max) as f64);
            b.put("last apex", apexes.last().copied().unwrap_or(0.0) as f64);
        }
        Kind::Hit => {
            let h = Hit::unpack(scene.n);
            if h.series { series(&mut b, &t, &h, &shapes) } else { hit(&mut b, &t, &h, &shapes) }
        }
        Kind::Ratio => {
            let (a, z) = (first[1], end[1]);
            b.put("top sank", (a.pos[1] - z.pos[1]) as f64);
            b.put("top slid", (z.pos[0] - a.pos[0]).hypot(z.pos[2] - a.pos[2]) as f64);
            b.put("jitter", t[t.len() - 60..].iter().map(|s| fastest(s)).fold(0.0, f32::max) as f64);
            b.put("stands", ((a.pos[1] - z.pos[1]) < 0.5) as u8 as f64);
        }
        k => panic!("{} is not a behaviour scene", k.name()),
    }
    let still = t.iter().rposition(|s| fastest(s) >= REST_SPEED).map_or(0, |k| k + 1);
    b.put("at rest from", if still >= t.len() { NEVER } else { still as f64 });
    let q = measure::quality(scene, &shapes, end);
    b.put("deepest at end", q.pen_max as f64);
    b.put("escaped", q.escaped as f64);
    b
}

/// A body's moment of inertia over its mass, as the engines give a solid
/// one: a sphere's 2 r² / 5, a cube's (every bounce's box is one) h² 2/3.
fn inertia(shape: Shape) -> f32 {
    match shape {
        Shape::Sphere(r) => 0.4 * r * r,
        Shape::Box([x, y, z]) => {
            assert!(x == y && y == z, "a bounce's box is a cube");
            2.0 * x * x / 3.0
        }
    }
}

/// How far below its centre a body reaches, turned as it is, and where
/// across the floor its lowest point (the lowest corners' middle) is from
/// its centre.
fn reach(s: &State, shape: Shape) -> (f32, [f32; 2]) {
    match shape {
        Shape::Sphere(r) => (r, [0.0, 0.0]),
        Shape::Box([h, _, _]) => {
            let c = scenes::corners(s.rot, h);
            let low = c.iter().map(|c| -c[1]).fold(f32::MIN, f32::max);
            let at: Vec<&[f32; 3]> = c.iter().filter(|c| -c[1] > low - 1e-4).collect();
            let n = at.len() as f32;
            (low, [at.iter().map(|c| c[0]).sum::<f32>() / n, at.iter().map(|c| c[2]).sum::<f32>() / n])
        }
    }
}

/// As 2D's `behave::free`: gravity alone changed the velocity, nothing the
/// spin.
fn free(a: &State, z: &State, g: f32) -> bool {
    let tol = 1e-4 * (1.0 + a.vel[1].abs().max(z.vel[1].abs()) + a.vel[0].abs());
    let spun = (0..3).all(|k| (z.ang[k] - a.ang[k]).abs() <= 1e-4 * (1.0 + a.ang[k].abs()));
    (z.vel[1] - a.vel[1] + g * DT).abs() <= tol && (z.vel[0] - a.vel[0]).abs() <= tol && (z.vel[2] - a.vel[2]).abs() <= tol && spun
}

fn dot3(a: [f32; 3]) -> f32 {
    dot(a, a)
}

/// One bounce (`Kind::Hit`), measured as 2D's `behave::hit` measures it
/// (the same values, by the same names), y up.
fn hit(b: &mut Behaviour, t: &[Vec<State>], h: &Hit, shapes: &[Shape]) {
    let g = h.g;
    let n = t[0].len();
    let all_free = |i: usize| (0..n).all(|k| free(&t[i][k], &t[i + 1][k], g));
    let first = (0..t.len() - 1).find(|&i| !all_free(i)).unwrap_or_else(|| panic!("hit {h:?}: nothing met"));
    let apart = |i: usize| if h.target.floor() { t[i][0].vel[1] > 0.0 } else { t[i][0].vel[1] > t[i][1].vel[1] };
    let left = (first + 1..t.len() - 1).find(|&i| all_free(i) && apart(i));
    let after = left.unwrap_or(t.len() - 1);
    let e = if h.v > BOUNCE_THRESHOLD { h.e } else { 0.0 };
    b.put("expected e", e as f64);
    if !h.target.floor() {
        let (a0, b0, a1, b1) = (t[first][0], t[first][1], t[after][0], t[after][1]);
        let (ma, mb) = (1.0, h.ratio);
        let (closing, parting) = (b0.vel[1] - a0.vel[1], a1.vel[1] - b1.vel[1]);
        let rel = parting / closing;
        b.put("bounced", (left.is_some() && rel > 0.01) as u8 as f64);
        b.put("rel", rel as f64);
        b.put("gain", (rel.max(0.0).powi(2) - e * e) as f64);
        let moved = (after - first) as f32 * DT;
        let lost: [f32; 3] = std::array::from_fn(|k| (ma * a1.vel[k] + mb * b1.vel[k]) - (ma * a0.vel[k] + mb * b0.vel[k]));
        let lost = [lost[0], lost[1] + (ma + mb) * g * moved, lost[2]];
        let scale = ma * mb / (ma + mb) * closing;
        b.put("momentum", (dot3(lost).sqrt() / scale) as f64);
        return;
    }
    let shape = shapes[0];
    let (a, z) = (t[first][0], t[after][0]);
    let k = inertia(shape);
    let (low, arm) = reach(&a, shape);
    let gap = a.pos[1] - low;
    let met = a.pos[1] - gap;
    let (vy, uy) = (a.vel[1], z.vel[1]);
    let (normal_in, normal_out) = (0.5 * vy * vy + g * gap, 0.5 * uy * uy + g * (z.pos[1] - met));
    // The whole energy, its height from where it lies flat, as 2D's.
    let flat = match shape {
        Shape::Sphere(r) => r,
        Shape::Box([h, _, _]) => h,
    };
    let whole = |s: &State| 0.5 * dot3(s.vel) + 0.5 * k * dot3(s.ang) + g * (s.pos[1] - flat);
    let energy_in = whole(&a);
    let energy = whole(&z) / energy_in;
    let normal = (normal_out.max(0.0) / normal_in).sqrt();
    let deepest = t.iter().map(|s| reach(&s[0], shape).0 - s[0].pos[1]).fold(0.0, f32::max);
    b.put("bounced", (left.is_some() && uy > 0.05) as u8 as f64);
    b.put("came in", -vy as f64);
    b.put("impact", (2.0 * normal_in).sqrt() as f64);
    b.put("through", (gap < -1e-3) as u8 as f64);
    b.put("energy", energy as f64);
    b.put("normal", normal as f64);
    b.put("excess", (energy - 1.0 - g * deepest / energy_in) as f64);
    let gain = match h.target {
        Target::Edge | Target::Corner => {
            let share = 1.0 / (1.0 + (arm[0] * arm[0] + arm[1] * arm[1]) / k);
            let expected = 1.0 - (1.0 - e * e) * share * normal_in / energy_in;
            b.put("expected", expected as f64);
            energy - expected - g * deepest / energy_in
        }
        _ => {
            b.put("expected", (e * e) as f64);
            normal * normal - e * e - g * deepest / normal_in
        }
    };
    b.put("gain", gain as f64);
    if h.along != 0.0 {
        b.put("tangent", (z.vel[0] / a.vel[0]) as f64);
    }
    b.put("spin", dot3(z.ang).sqrt() as f64);
}

/// Many bounces, as 2D's `behave::series`: apexes by the energy's height.
fn series(b: &mut Behaviour, t: &[Vec<State>], h: &Hit, shapes: &[Shape]) {
    let flat = match shapes[0] {
        Shape::Sphere(r) => r,
        Shape::Box([h, _, _]) => h,
    };
    let height = |s: &State| s.pos[1] - flat + 0.5 * s.vel[1] * s.vel[1] / h.g;
    let drop = height(&t[0][0]);
    let (mut apexes, mut rising, mut top) = (Vec::new(), false, 0f32);
    for st in &t[1..] {
        let s = st[0];
        if !rising && s.vel[1] > 0.0 {
            (rising, top) = (true, height(&s));
        } else if rising {
            if s.vel[1] > 0.0 {
                top = top.max(height(&s));
            }
            if s.vel[1] < 0.0 {
                apexes.push(top / drop);
                rising = false;
            }
        }
    }
    let e2 = h.e * h.e;
    let fast = |a: f32| (2.0 * h.g * a * drop).sqrt() > 2.0;
    let with_drop: Vec<f32> = std::iter::once(1.0).chain(apexes.iter().copied()).collect();
    let mut ratios: Vec<f64> = with_drop.windows(2).filter(|w| fast(w[0]) && fast(w[1])).map(|w| (w[1] / w[0] / e2) as f64).collect();
    b.put("apexes", apexes.len() as f64);
    b.put("rise most", apexes.iter().copied().fold(0.0, f32::max) as f64);
    ratios.sort_by(f64::total_cmp);
    b.put("decay median", ratios.get(ratios.len() / 2).copied().unwrap_or(f64::NAN));
    b.put("decay most", ratios.last().copied().unwrap_or(f64::NAN));
}
