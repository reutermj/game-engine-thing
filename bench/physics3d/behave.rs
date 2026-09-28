//! The 3D behaviour scenes (`Kind::RampHold` to `Kind::Ratio`): what a
//! player sees a body do, measured from each engine's states every step,
//! the same for every engine, as 2D's `behave.rs` measures its own
//! (physics.md, "Quality beyond settling"). What `bench -- ... --behave`
//! prints and `:behaviour_test` bounds.

use crate::measure::{self, REST_SPEED};
use crate::scenes::{DROP, Kind, Scene, Shape};
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
    let mut t: Vec<Vec<State>> = Vec::with_capacity(scene.steps);
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
