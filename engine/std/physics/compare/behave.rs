//! What a player sees a body do, on the behaviour scenes
//! (`Scene::behaviour`: ramps, bounces, mass ratios, overlap, bullets and
//! structures): each scene stepped from the start and read every step, then
//! measured from positions and velocities alone, as `quality.rs` measures
//! settling, so every engine is judged by the same code. What the
//! comparison prints with `BEHAVE=1` and `:behaviour_test` bounds
//! (physics.md, "Quality beyond settling").

use crate::quality;
use crate::scene::{self, DT, GRAVITY, Hit, Scene, Target};
use crate::settle::REST;
use crate::solver;
use crate::{Dyn, Sim};

/// One engine on one scene: named values, in the order they were found.
#[derive(Clone, Debug, Default)]
pub struct Behaviour {
    pub label: String,
    pub values: Vec<(&'static str, f64)>,
}

impl Behaviour {
    #[allow(dead_code)] // The tests read values by name; the comparison prints them all.
    pub fn get(&self, name: &str) -> f64 {
        self.values.iter().find(|(k, _)| *k == name).map(|(_, v)| *v).unwrap_or_else(|| panic!("no {name} in {:?}", self.values))
    }

    fn put(&mut self, name: &'static str, v: f64) {
        self.values.push((name, v));
    }
}

/// Never at rest, or never toppled: past any bound.
pub const NEVER: f64 = f64::INFINITY;

/// Speeds the bullet scenes fire at, a unit a second: from a sixth of a
/// unit a step to Box2D's speed cap (400), closer about where ours starts
/// to tunnel, which is where a step passes the margin, the radius and half
/// the wall (0.35 a step, 21 a second, through a wall 0.1 thick; 0.8, 48,
/// through pong's paddle, a unit thick); 40 is pong's fastest along the
/// court, and 56.6 its fastest diagonal.
pub const BULLET_SPEEDS: [f32; 12] = [10.0, 20.0, 21.0, 25.0, 30.0, 40.0, 48.0, 50.0, 56.6, 80.0, 160.0, 400.0];

/// Where in a step the bullet scenes' ball reaches the wall.
pub const BULLET_PHASES: [f32; 4] = [0.0, 0.25, 0.5, 0.75];

/// The scenes `BEHAVE=1` runs, whose tables set `:behaviour_test`'s bounds.
#[allow(dead_code)] // The comparison's list; each test names its own.
pub fn scenes() -> Vec<Scene> {
    let mut v = vec![
        // Holds (tan 20° = 0.36 < 0.6), slides (0.58 > 0.2), rolls
        // (0.6 ≥ tan 30° / 3 = 0.19), and rolls slipping (0.1 < 0.19).
        Scene::Ramp { deg: 20.0, mu: 0.6, circle: false },
        Scene::Ramp { deg: 30.0, mu: 0.2, circle: false },
        Scene::Ramp { deg: 30.0, mu: 0.6, circle: true },
        Scene::Ramp { deg: 30.0, mu: 0.1, circle: true },
        Scene::Bounce { e: 0.25 },
        Scene::Bounce { e: 0.5 },
        Scene::Bounce { e: 0.75 },
        Scene::Bounce { e: 1.0 },
        Scene::Ratio { ratio: 10.0, light: 1 },
        Scene::Ratio { ratio: 100.0, light: 1 },
        Scene::Ratio { ratio: 1000.0, light: 1 },
        Scene::Ratio { ratio: 100.0, light: 5 },
        Scene::Ratio { ratio: 1000.0, light: 5 },
        Scene::BigOnSmall,
        Scene::Overlap { base: 4, overlap: 0.25 },
        Scene::Overlap { base: 4, overlap: 0.5 },
        Scene::Overlap { base: 10, overlap: 0.5 },
        Scene::Cards { rows: 5, lean: 25.0, mu: 0.7 },
        // Stands above 0.269 (`Scene::ladder_mu`), slides below.
        Scene::Ladder { deg: 30.0, mu: 0.4 },
        Scene::Ladder { deg: 30.0, mu: 0.3 },
        Scene::Ladder { deg: 30.0, mu: 0.24 },
        Scene::Ladder { deg: 30.0, mu: 0.2 },
        Scene::Dominoes { n: 15, spacing: 1.0, mu: 0.6 },
    ];
    // A thin wall, and pong's paddle (a unit thick) and ball (radius 0.25),
    // each met at four points in a step.
    for thick in [0.1, 1.0] {
        for speed in BULLET_SPEEDS {
            v.extend(BULLET_PHASES.iter().map(|&phase| Scene::Bullet { speed, radius: 0.25, thick, phase }));
        }
    }
    v
}

/// Steps a scene runs: long enough for what it shows to be over.
pub fn steps(scene: &Scene) -> u32 {
    match *scene {
        Scene::Ramp { .. } => 120,
        Scene::Bounce { e } if e >= 1.0 => 1200,
        Scene::Bullet { .. } => 60,
        Scene::Hit(h) if h.secs > 0.0 => (h.secs * h.hz) as u32,
        // To the contact, and long enough after for a slow bounce to leave.
        Scene::Hit(h) => h.flight().0 + 45,
        Scene::Cards { .. } | Scene::Dominoes { .. } | Scene::Stack { .. } | Scene::Pyramid { .. } | Scene::PyramidAt { .. } => 600,
        _ => 300,
    }
}

/// Every step's bodies, the start first.
fn run(sim: &mut dyn Sim, n: u32) -> Vec<Vec<Dyn>> {
    let mut t = vec![sim.bodies()];
    for _ in 0..n {
        sim.step(1);
        t.push(sim.bodies());
    }
    t
}

fn fastest(bodies: &[Dyn]) -> f32 {
    bodies.iter().map(Dyn::speed).fold(0.0, f32::max)
}

fn moved(a: &Dyn, b: &Dyn) -> f64 {
    (b.x - a.x).hypot(b.y - a.y) as f64
}

/// `sim` on `scene` from the start, as `steps` says.
pub fn behave(sim: &mut dyn Sim, scene: &Scene, turning: bool) -> Behaviour {
    let t = run(sim, steps(scene));
    let mut b = Behaviour { label: sim.label(), values: Vec::new() };
    let depths: Vec<f32> = t.iter().map(|bodies| quality::measure(scene, bodies, turning).max_depth).collect();
    let (first, end) = (&t[0], t.last().unwrap());
    let q = quality::measure(scene, end, turning);
    match *scene {
        Scene::Ramp { deg, mu, circle } => ramp(&mut b, &t, deg, mu, circle),
        Scene::Bounce { e } => bounce(&mut b, &t, e),
        Scene::Ratio { .. } | Scene::BigOnSmall => {
            let top = first.len() - 1;
            let (a, z) = (first[top], end[top]);
            b.put("top sank", (z.y - a.y) as f64);
            b.put("top slid", (z.x - a.x).abs() as f64);
            // How much a column at rest still moves: every body, the last
            // second.
            let jitter = t[t.len() - 60..].iter().map(|bodies| fastest(bodies)).fold(0.0, f32::max);
            b.put("jitter", jitter as f64);
            b.put("stands", ((z.y - a.y) < 0.5 && (z.x - a.x).abs() < 0.5) as u8 as f64);
        }
        Scene::Overlap { base, .. } => {
            b.put("peak speed", t.iter().map(|bodies| fastest(bodies)).fold(0.0, f32::max) as f64);
            let apart = depths.iter().position(|&d| d < 0.01).map_or(NEVER, |k| k as f64);
            b.put("separated at", apart);
            // The top box against where it rests once apart: a stack of
            // `base` unit boxes.
            let rest_y = -0.5 - (base - 1) as f32;
            b.put("top off", (end[end.len() - 1].y - rest_y) as f64);
        }
        Scene::Bullet { speed, thick, .. } => {
            let z = end[0];
            b.put("through", (z.x > thick) as u8 as f64);
            b.put("rebound", (-z.vx / speed) as f64);
        }
        Scene::Cards { .. } => {
            let most = first.iter().zip(end).map(|(a, z)| moved(a, z)).fold(0.0, f64::max);
            b.put("most moved", most);
            // A card has fallen once it has moved a quarter of its height.
            b.put("fallen", first.iter().zip(end).filter(|(a, z)| moved(a, z) > 0.25 * a.hy as f64).count() as f64);
        }
        Scene::Ladder { deg, .. } => {
            let (a, z) = (first[0], end[0]);
            b.put("slid", (z.x - a.x).abs() as f64);
            b.put("fell", (z.y - a.y) as f64);
            b.put("stands", ((z.x - a.x).abs() < 0.05) as u8 as f64);
            b.put("needs mu", Scene::ladder_mu(deg) as f64);
        }
        Scene::Dominoes { n, .. } => dominoes(&mut b, &t, n),
        Scene::Hit(h) if h.secs > 0.0 => series(&mut b, &t, &h),
        Scene::Hit(h) => hit(&mut b, &t, &h),
        // The families' stacks and pyramids (`family.rs`): whether the top
        // box stayed within half a box of where it began.
        Scene::Stack { .. } | Scene::Pyramid { .. } | Scene::PyramidAt { .. } => {
            let top = first.len() - 1;
            let off = moved(&first[top], &end[top]);
            b.put("top moved", off);
            b.put("most moved", first.iter().zip(end).map(|(a, z)| moved(a, z)).fold(0.0, f64::max));
            b.put("stands", (off < 0.5) as u8 as f64);
        }
        _ => panic!("{} is not a behaviour scene", scene.text()),
    }
    // At rest from the step every body stayed slower than `REST` to the end.
    let still = t.iter().rposition(|bodies| fastest(bodies) >= REST).map_or(0, |k| k + 1);
    b.put("at rest from", if still >= t.len() { NEVER } else { still as f64 });
    b.put("deepest at end", q.max_depth as f64);
    b.put("deepest during", depths.iter().copied().fold(0.0, f32::max) as f64);
    b.put("contacts a body", q.contacts_per_body);
    b.put("islands", q.islands as f64);
    b.put("escaped", q.escaped as f64);
    b
}

/// Along the slope: the acceleration from the speed at steps 30 and 90
/// (past the first contact's settling in), against the hand calculation;
/// how far a box that should hold crept in the 2 s; for a disc, how far
/// from rolling (its contact point's speed along the slope, `v − ω r`,
/// the mean over the same steps); and the most speed off the slope.
fn ramp(b: &mut Behaviour, t: &[Vec<Dyn>], deg: f32, mu: f32, circle: bool) {
    let a = deg.to_radians();
    let (tangent, normal) = ((a.cos(), a.sin()), (a.sin(), -a.cos()));
    let along = |d: &Dyn| d.vx * tangent.0 + d.vy * tangent.1;
    let g = GRAVITY;
    let sliding = g * (a.sin() - mu * a.cos());
    let expected = if circle { if mu >= a.tan() / 3.0 { 2.0 / 3.0 * g * a.sin() } else { sliding } } else { sliding.max(0.0) };
    let (v30, v90) = (along(&t[30][0]), along(&t[90][0]));
    b.put("expected a", expected as f64);
    b.put("a", ((v90 - v30) / (60.0 * DT)) as f64);
    let (a0, z) = (t[0][0], t[t.len() - 1][0]);
    b.put("crept", ((z.x - a0.x) * tangent.0 + (z.y - a0.y) * tangent.1).abs() as f64);
    if circle {
        let slip: f32 = t[30..90].iter().map(|s| (along(&s[0]) - s[0].w * s[0].hx).abs()).sum::<f32>() / 60.0;
        b.put("slip", slip as f64);
    }
    let off = t.iter().map(|s| (s[0].vx * normal.0 + s[0].vy * normal.1).abs()).fold(0.0, f32::max);
    b.put("off the slope", off as f64);
}

/// Each apex after a bounce, as a share of the drop: the first against
/// e², and over many bounces, the most and the last (a lossless ball
/// gaining height gains energy).
fn bounce(b: &mut Behaviour, t: &[Vec<Dyn>], e: f32) {
    // The ball's bottom above the floor's top, at y = 0.
    let height = |d: &Dyn| -d.y - d.hx;
    let (mut apexes, mut rising, mut top) = (Vec::new(), false, 0f32);
    for s in &t[1..] {
        let d = s[0];
        if !rising && d.vy < 0.0 {
            (rising, top) = (true, height(&d));
        } else if rising {
            top = top.max(height(&d));
            if d.vy > 0.0 {
                apexes.push(top / scene::DROP);
                rising = false;
            }
        }
    }
    b.put("expected", (e * e) as f64);
    b.put("first apex", apexes.first().copied().unwrap_or(0.0) as f64);
    b.put("bounces", apexes.len() as f64);
    b.put("most apex", apexes.iter().copied().fold(0.0, f32::max) as f64);
    b.put("last apex", apexes.last().copied().unwrap_or(0.0) as f64);
}

/// A body's moment of inertia over its mass: a disc's r² / 2, a box's
/// (hx² + hy²) / 3.
fn inertia(d: &Dyn) -> f32 {
    if d.circle { 0.5 * d.hx * d.hx } else { (d.hx * d.hx + d.hy * d.hy) / 3.0 }
}

/// How far below its centre a body reaches (y down), turned as it is.
fn reach(d: &Dyn) -> f32 {
    if d.circle { d.hx } else { d.hx * d.angle.sin().abs() + d.hy * d.angle.cos().abs() }
}

/// How far above the floor its centre is when it lies flat.
fn flat(d: &Dyn) -> f32 {
    if d.circle { d.hx } else { d.hy }
}

/// Whether a body flew free from state `a` to the next, `z`: gravity alone
/// changed its velocity, and nothing its spin. How the bounce measures find
/// the contact from positions and velocities alone, in every engine: the
/// step before the first that isn't free is where it came in, and the first
/// free one after is where it left.
fn free(a: &Dyn, z: &Dyn, g: f32, dt: f32) -> bool {
    let tol = 1e-4 * (1.0 + a.vy.abs().max(z.vy.abs()) + a.vx.abs());
    (z.vy - a.vy - g * dt).abs() <= tol && (z.vx - a.vx).abs() <= tol && (z.w - a.w).abs() <= 1e-4 * (1.0 + a.w.abs())
}

/// One bounce (`Scene::Hit`), from the state before the first step a
/// contact acted in to the first after which every body flew free again (or
/// the last, if one never did: it came to rest). Against a floor, per unit
/// mass: the energy it came in with, the gravity it would gain over the gap
/// included (so where in a step it meets doesn't count), against what it
/// left with, from the height it met at; `normal` the rebound speed over
/// the impact speed along the normal from those, which restitution sets to
/// e; `gain` the energy ratio past what restitution gives (e² along the
/// normal, and for a corner the single impact's 1 − (1 − e²) m_eff / m of
/// it), none below `BOUNCE_THRESHOLD`, and past what the push-out of its
/// deepest overlap lifts it by (`excess`, of the whole energy, the same
/// past none). Between two free bodies, the same of
/// their closing speed, and the momentum lost, over the impulse scale.
fn hit(b: &mut Behaviour, t: &[Vec<Dyn>], h: &Hit) {
    let (g, dt) = (h.g, h.dt());
    let n = t[0].len();
    let all_free = |i: usize| (0..n).all(|k| free(&t[i][k], &t[i + 1][k], g, dt));
    let Some(first) = (0..t.len() - 1).find(|&i| !all_free(i)) else {
        panic!("hit {h:?}: nothing met");
    };
    // Left: free, and moving apart (up, off a floor), since a speculative
    // contact may slow a body in the step before it meets.
    let apart = |i: usize| if h.target.floor() { t[i][0].vy < 0.0 } else { t[i][1].vy > t[i][0].vy };
    let left = (first + 1..t.len() - 1).find(|&i| all_free(i) && apart(i));
    let after = left.unwrap_or(t.len() - 1);
    let e = if h.v > solver::BOUNCE_THRESHOLD { h.e } else { 0.0 };
    b.put("expected e", e as f64);
    if !h.target.floor() {
        let (a0, b0, a1, b1) = (t[first][0], t[first][1], t[after][0], t[after][1]);
        let (ma, mb) = (1.0, h.ratio);
        let (closing, parting) = (a0.vy - b0.vy, b1.vy - a1.vy);
        let rel = parting / closing;
        b.put("bounced", (left.is_some() && rel > 0.01) as u8 as f64);
        b.put("rel", rel as f64);
        b.put("gain", (rel.max(0.0).powi(2) - e * e) as f64);
        let moved = (after - first) as f32 * dt;
        let lost = (ma * a1.vy + mb * b1.vy) - (ma * a0.vy + mb * b0.vy) - (ma + mb) * g * moved;
        let sideways = (ma * a1.vx + mb * b1.vx) - (ma * a0.vx + mb * b0.vx);
        let scale = ma * mb / (ma + mb) * closing;
        b.put("momentum", (lost.hypot(sideways) / scale) as f64);
        return;
    }
    let (a, z) = (t[first][0], t[after][0]);
    let k = inertia(&a);
    // y is down: the gap closes as y grows, and height is gained as it falls.
    let gap = -(a.y + reach(&a));
    let met = a.y + gap;
    let (normal_in, normal_out) = (0.5 * a.vy * a.vy + g * gap, 0.5 * z.vy * z.vy + g * (met - z.y));
    // The whole energy, its height from where it lies flat: a corner's tilt
    // is height it may turn into speed as it tips, which a share of what it
    // came in with alone would count as gained.
    let whole = |d: &Dyn| 0.5 * (d.vx * d.vx + d.vy * d.vy) + 0.5 * k * d.w * d.w + g * (-d.y - flat(d));
    let energy_in = whole(&a);
    let energy = whole(&z) / energy_in;
    let normal = (normal_out.max(0.0) / normal_in).sqrt();
    // A soft contact pushes an overlap out through positions, which lifts
    // the body without costing its speed (as Box2D's does): up to gravity
    // times its deepest overlap, the energy a bounce may leave with past
    // what it came in with, beyond which it is energy from nowhere.
    let deepest = t.iter().map(|s| s[0].y + reach(&s[0])).fold(0.0, f32::max);
    b.put("excess", (energy - 1.0 - g * deepest / energy_in) as f64);
    b.put("bounced", (left.is_some() && z.vy < -0.05) as u8 as f64);
    // The speed toward the floor at the start of the step it met in, and
    // at the moment it met: what an engine can take restitution from.
    b.put("came in", a.vy as f64);
    b.put("impact", (2.0 * normal_in).sqrt() as f64);
    // Under the surface already at the step it met in: it passed it in a
    // free step (the speculative margin missed it), gravity speeding it on
    // over the overlap, which the push-out gives back as height.
    b.put("through", (gap < -1e-3) as u8 as f64);
    b.put("energy", energy as f64);
    b.put("normal", normal as f64);
    let gain = if h.target == Target::Corner {
        // The lowest corner's arm across the normal, as the body came in.
        let (s, c) = a.angle.sin_cos();
        let arm = c * s.signum() * a.hx - s * c.signum() * a.hy;
        let share = 1.0 / (1.0 + arm * arm / k);
        let expected = 1.0 - (1.0 - e * e) * share * normal_in / energy_in;
        b.put("expected", expected as f64);
        energy - expected - g * deepest / energy_in
    } else {
        b.put("expected", (e * e) as f64);
        normal * normal - e * e - g * deepest / normal_in
    };
    b.put("gain", gain as f64);
    if h.along != 0.0 {
        b.put("tangent", (z.vx / a.vx) as f64);
    }
    b.put("spin", z.w as f64);
}

/// Many bounces (a `Scene::Hit` with `secs`): each apex of its lowest point
/// over the height it fell from, the most, and each apex over the one
/// before against e², while the bounce is well above `BOUNCE_THRESHOLD` (a
/// rebound faster than 2): the median and the most. A lossless ball's never
/// rises; one of restitution e keeps e² of its height a bounce. A height is
/// the energy's, its height and its speed up over 2g, which free flight
/// keeps: the highest step would read an apex low by up to g (dt / 2)² / 2
/// (3% of a low bounce at gravity 80), and their ratios so pass e².
fn series(b: &mut Behaviour, t: &[Vec<Dyn>], h: &Hit) {
    // Its centre's, which free flight keeps whatever it turns.
    let height = |d: &Dyn| -d.y - flat(d) + 0.5 * d.vy * d.vy / h.g;
    let drop = height(&t[0][0]);
    let (mut apexes, mut rising, mut top) = (Vec::new(), false, 0f32);
    for st in &t[1..] {
        let d = st[0];
        if !rising && d.vy < 0.0 {
            (rising, top) = (true, height(&d));
        } else if rising {
            if d.vy < 0.0 {
                top = top.max(height(&d));
            }
            if d.vy > 0.0 {
                apexes.push(top / drop);
                rising = false;
            }
        }
    }
    let e2 = h.e * h.e;
    let fast = |a: f32| (2.0 * h.g * a * drop).sqrt() > 2.0;
    let mut ratios: Vec<f64> = std::iter::once(1.0)
        .chain(apexes.iter().copied())
        .collect::<Vec<f32>>()
        .windows(2)
        .filter(|w| fast(w[0]) && fast(w[1]))
        .map(|w| (w[1] / w[0] / e2) as f64)
        .collect();
    b.put("apexes", apexes.len() as f64);
    b.put("rise most", apexes.iter().copied().fold(0.0, f32::max) as f64);
    ratios.sort_by(f64::total_cmp);
    b.put("decay median", ratios.get(ratios.len() / 2).copied().unwrap_or(f64::NAN));
    b.put("decay most", ratios.last().copied().unwrap_or(f64::NAN));
}

/// Each domino's step past 45°: how many fell, whether in order, how fast
/// the wave ran (dominoes a second, first fall to last), and the last one's
/// lean at the end.
fn dominoes(b: &mut Behaviour, t: &[Vec<Dyn>], n: u32) {
    let over = std::f32::consts::FRAC_PI_4;
    let fell: Vec<Option<usize>> = (0..n as usize).map(|i| t.iter().position(|s| s[i].angle.abs() > over)).collect();
    let steps: Vec<usize> = fell.iter().flatten().copied().collect();
    b.put("toppled", steps.len() as f64);
    let in_order = fell.windows(2).all(|w| match (w[0], w[1]) {
        (Some(a), Some(b)) => a <= b,
        (None, Some(_)) => false,
        _ => true,
    });
    b.put("in order", in_order as u8 as f64);
    let (first, last) = (steps.first().copied(), steps.last().copied());
    b.put("last fell at", last.map_or(NEVER, |s| s as f64));
    let wave = match (first, last) {
        (Some(f), Some(l)) if l > f => (steps.len() - 1) as f64 / ((l - f) as f64 * DT as f64),
        _ => 0.0,
    };
    b.put("wave", wave);
    b.put("last lean", t[t.len() - 1][n as usize - 1].angle.abs().to_degrees() as f64);
}
