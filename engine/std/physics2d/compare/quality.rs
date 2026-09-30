//! How well a scene settled, measured the same way for every engine from
//! positions and velocities alone, since each engine's own contact data
//! means something different (Box2D keeps two points a box contact, the
//! engine one normal, Rapier its own manifolds).

use std::collections::HashMap;

use physics2d::{Placed, Rot, Shape, Vec2};

use crate::Dyn;
use crate::scene::{Scene, Spec};

/// Closer than this counts as touching: the contacts per body and the
/// islands that tell a real pile from columns.
const TOUCH: f32 = 0.01;

#[derive(Clone, Copy, Debug, Default)]
pub struct Quality {
    pub bodies: usize,
    pub contacts_per_body: f64,
    pub islands: usize,
    /// Overlap between two shapes (or a shape and a static): the deepest,
    /// the mean over overlapping pairs, and how many overlap more than the
    /// engines' slop (0.005 in all three) twice over.
    pub max_depth: f32,
    pub mean_depth: f64,
    pub deep: usize,
    pub mean_speed: f64,
    pub max_speed: f32,
    /// Kinetic energy per body (mass 1).
    pub energy: f64,
    pub escaped: usize,
    /// The most a box is turned from resting on a face, in degrees (0 to
    /// 45): how far a pyramid or a stack of boxes has toppled.
    pub tilt: f32,
}

/// Signed distance between two shapes, negative when they overlap: turned
/// ones by their faces' separating axes (`physics2d::separation`), the rest
/// axis-aligned as before rotation.
fn gap(a: &Dyn, b: &Dyn) -> f32 {
    if a.angle != 0.0 || b.angle != 0.0 {
        let placed = |d: &Dyn| {
            let shape = if d.circle { Shape::Circle(d.hx) } else { Shape::Box(Vec2::new(d.hx, d.hy)) };
            Placed { shape, at: Vec2::new(d.x, d.y), rot: Some(Rot::from_angle(d.angle)) }
        };
        return physics2d::separation(&placed(a), &placed(b));
    }
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    match (a.circle, b.circle) {
        (true, true) => (dx * dx + dy * dy).sqrt() - a.hx - b.hx,
        (false, false) => (dx.abs() - a.hx - b.hx).max(dy.abs() - a.hy - b.hy),
        _ => {
            let (bx, c) = if a.circle { (b, a) } else { (a, b) };
            let (ox, oy) = ((c.x - bx.x).abs() - bx.hx, (c.y - bx.y).abs() - bx.hy);
            if ox > 0.0 || oy > 0.0 {
                let (ox, oy) = (ox.max(0.0), oy.max(0.0));
                (ox * ox + oy * oy).sqrt() - c.hx
            } else {
                ox.max(oy) - c.hx
            }
        }
    }
}

fn as_dyn(s: &Spec) -> Dyn {
    Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: s.x, y: s.y, vx: 0.0, vy: 0.0, angle: s.angle, w: 0.0 }
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

/// `turning`: the bodies may turn; if not, each must still face as it
/// started, or an engine turned what should have been locked.
pub fn measure(scene: &Scene, bodies: &[Dyn], turning: bool) -> Quality {
    let turned = bodies.iter().filter(|b| b.angle.abs() > 1e-6).count();
    assert!(turning || turned == 0, "{turned} bodies turned: rotation is not locked");
    let statics: Vec<Dyn> = scene.build().iter().filter(|s| !s.dynamic).map(as_dyn).collect();
    // A grid of cells as wide as the widest body (a unit in every settling
    // scene), so a pair within `TOUCH` is in neighboring cells.
    let size = bodies.iter().map(|b| 2.0 * b.hx.hypot(b.hy) + TOUCH).fold(1.0, f32::max);
    let cell = |x: f32| (x / size).floor() as i32;
    let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, b) in bodies.iter().enumerate() {
        grid.entry((cell(b.x), cell(b.y))).or_default().push(i);
    }
    let mut parent: Vec<usize> = (0..bodies.len()).collect();
    let (mut touching, mut overlaps, mut depth_sum, mut max_depth, mut deep) = (0usize, 0usize, 0f64, 0f32, 0usize);
    let mut pair = |g: f32| {
        if g <= TOUCH {
            touching += 1;
        }
        if g < 0.0 {
            overlaps += 1;
            depth_sum += -g as f64;
            max_depth = max_depth.max(-g);
            if -g > 0.01 {
                deep += 1;
            }
        }
    };
    for (i, a) in bodies.iter().enumerate() {
        let (cx, cy) = (cell(a.x), cell(a.y));
        for gx in cx - 1..=cx + 1 {
            for gy in cy - 1..=cy + 1 {
                for &j in grid.get(&(gx, gy)).map_or(&[][..], |v| v.as_slice()) {
                    if j <= i {
                        continue;
                    }
                    let g = gap(a, &bodies[j]);
                    pair(g);
                    if g <= TOUCH {
                        let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                        parent[ri] = rj;
                    }
                }
            }
        }
        for s in &statics {
            pair(gap(a, s));
        }
    }
    let islands = (0..bodies.len()).filter(|&i| find(&mut parent, i) == i).count();
    let speeds: Vec<f32> = bodies.iter().map(Dyn::speed).collect();
    let n = bodies.len().max(1) as f64;
    // Kinetic energy, with mass 1: the turning part by each shape's inertia.
    let energy = |b: &Dyn| {
        let inertia = if b.circle { b.hx * b.hx / 2.0 } else { (b.hx * b.hx + b.hy * b.hy) / 3.0 };
        0.5 * ((b.vx * b.vx + b.vy * b.vy) as f64 + (inertia * b.w * b.w) as f64)
    };
    let quarter = std::f32::consts::FRAC_PI_2;
    let tilt = bodies
        .iter()
        .filter(|b| !b.circle)
        .map(|b| (b.angle - (b.angle / quarter).round() * quarter).abs().to_degrees())
        .fold(0.0, f32::max);
    Quality {
        bodies: bodies.len(),
        contacts_per_body: touching as f64 / n,
        islands,
        max_depth,
        mean_depth: depth_sum / overlaps.max(1) as f64,
        deep,
        mean_speed: speeds.iter().map(|&s| s as f64).sum::<f64>() / n,
        max_speed: speeds.iter().copied().fold(0.0, f32::max),
        energy: bodies.iter().map(energy).sum::<f64>() / n,
        escaped: bodies.iter().filter(|b| scene.escaped(b.x, b.y)).count(),
        tilt,
    }
}

/// How far bodies moved between two looks at the same bodies: the mean and
/// the greatest distance.
pub fn moved(before: &[Dyn], after: &[Dyn]) -> (f64, f32) {
    assert_eq!(before.len(), after.len(), "the same bodies");
    let d: Vec<f32> = before.iter().zip(after).map(|(a, b)| ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()).collect();
    (d.iter().map(|&d| d as f64).sum::<f64>() / d.len().max(1) as f64, d.iter().copied().fold(0.0, f32::max))
}

/// Calibration: each measure on bodies placed where its value is known by
/// hand (physics-testing.md, "Measurements that are themselves tested"),
/// so a measure that reads zero or reads the wrong thing fails here, where
/// every upper bound on it would pass.
#[cfg(test)]
mod tests {
    use super::*;

    /// A floor whose top is at y = 0 (y down), 20 wide, and nothing else.
    const FLOOR: Scene = Scene::Stack { n: 0 };

    /// A unit box with its middle at `x, y`, at rest.
    fn unit(x: f32, y: f32) -> Dyn {
        Dyn { circle: false, hx: 0.5, hy: 0.5, x, y, vx: 0.0, vy: 0.0, angle: 0.0, w: 0.0 }
    }

    fn near(a: f64, b: f64, what: &str) {
        assert!((a - b).abs() < 1e-5, "{what}: {a}, expected {b}");
    }

    #[test]
    fn depth_is_the_overlap_planted() {
        // Sunk 0.03 and 0.004 into the floor, and one turned a quarter
        // (the separating-axis path) sunk 0.02; a disc of radius 0.45 sunk 0.008.
        let mut turned = unit(6.0, -0.48);
        turned.angle = std::f32::consts::FRAC_PI_2;
        let disc = Dyn { circle: true, hx: 0.45, hy: 0.45, y: -0.442, ..unit(9.0, 0.0) };
        let q = measure(&FLOOR, &[unit(0.0, -0.47), unit(3.0, -0.496), turned, disc], true);
        near(q.max_depth as f64, 0.03, "deepest");
        near(q.mean_depth, (0.03 + 0.004 + 0.02 + 0.008) / 4.0, "mean");
        assert_eq!(q.deep, 2, "deeper than 0.01: the 0.03 and the 0.02");
        let q = measure(&FLOOR, &[unit(0.0, -0.5), unit(1.0, -0.5)], false);
        assert_eq!((q.max_depth, q.deep), (0.0, 0), "touching is not overlapping");
    }

    #[test]
    fn energy_is_kinetic_energy_at_mass_one_moving_and_turning() {
        // 3-4-5 moving: 12.5. A unit box turning at 2 a second, inertia
        // (0.25 + 0.25) / 3: 1/3. A disc of radius 0.5 at 4 a second,
        // inertia 0.125: 1.
        let moving = Dyn { vx: 3.0, vy: 4.0, ..unit(0.0, -5.0) };
        let turning = Dyn { w: 2.0, ..unit(3.0, -5.0) };
        let disc = Dyn { circle: true, w: 4.0, ..unit(6.0, -5.0) };
        let q = measure(&FLOOR, &[moving, turning, disc], true);
        near(q.energy, (12.5 + 1.0 / 3.0 + 1.0) / 3.0, "energy a body");
        near(q.max_speed as f64, 5.0, "fastest");
    }

    #[test]
    fn tilt_is_how_far_a_box_is_turned_from_resting_on_a_face() {
        for (deg, tilt) in [(10.0, 10.0), (-10.0, 10.0), (100.0, 10.0), (135.0, 45.0), (90.0, 0.0)] {
            let b = Dyn { angle: f32::to_radians(deg), ..unit(0.0, -5.0) };
            let t = measure(&FLOOR, &[b], true).tilt;
            assert!((t - tilt).abs() < 1e-3, "turned {deg}°: {t}, expected {tilt}");
        }
        let disc = Dyn { circle: true, angle: 0.3, ..unit(0.0, -5.0) };
        assert_eq!(measure(&FLOOR, &[disc], true).tilt, 0.0, "a disc doesn't lean");
    }

    #[test]
    fn contacts_and_islands_count_what_touches() {
        // Two boxes side by side on the floor and one alone: touching, the
        // two and each with the floor, 4 over 3 bodies; islands, the two
        // and the one (the floor joins none).
        let q = measure(&FLOOR, &[unit(0.0, -0.5), unit(1.0, -0.5), unit(5.0, -0.5)], false);
        near(q.contacts_per_body, 4.0 / 3.0, "contacts a body");
        assert_eq!(q.islands, 2);
        // Just past touching, the second is an island of its own.
        let q = measure(&FLOOR, &[unit(0.0, -0.5), unit(1.0 + 2.0 * TOUCH, -0.5), unit(5.0, -0.5)], false);
        near(q.contacts_per_body, 1.0, "contacts a body");
        assert_eq!(q.islands, 3);
    }

    #[test]
    fn escaped_counts_the_bodies_outside_the_scene() {
        let q = measure(&FLOOR, &[unit(0.0, -0.5), unit(11.0, -0.5), unit(0.0, 2.0)], false);
        assert_eq!(q.escaped, 2, "one past the floor's end, one under it");
    }
}
