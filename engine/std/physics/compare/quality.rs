//! How well a scene settled, measured the same way for every engine from
//! positions and velocities alone, since each engine's own contact data
//! means something different (Box2D keeps two points a box contact, the
//! engine one normal, Rapier its own manifolds).

use std::collections::HashMap;

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
}

/// Signed distance between two shapes, negative when they overlap:
/// axis-aligned boxes and circles, since rotation is locked everywhere.
fn gap(a: &Dyn, b: &Dyn) -> f32 {
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
    Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: s.x, y: s.y, vx: 0.0, vy: 0.0, angle: 0.0 }
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

pub fn measure(scene: &Scene, bodies: &[Dyn]) -> Quality {
    // Every measure here takes shapes to be axis-aligned.
    let turned = bodies.iter().filter(|b| b.angle.abs() > 1e-6).count();
    assert_eq!(turned, 0, "{turned} bodies turned: rotation is not locked");
    let statics: Vec<Dyn> = scene.build().iter().filter(|s| !s.dynamic).map(as_dyn).collect();
    // A grid of unit cells: no body is wider than one, so a pair within
    // `TOUCH` is in neighboring cells.
    let cell = |x: f32| x.floor() as i32;
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
    let speeds: Vec<f32> = bodies.iter().map(|b| (b.vx * b.vx + b.vy * b.vy).sqrt()).collect();
    let n = bodies.len().max(1) as f64;
    Quality {
        bodies: bodies.len(),
        contacts_per_body: touching as f64 / n,
        islands,
        max_depth,
        mean_depth: depth_sum / overlaps.max(1) as f64,
        deep,
        mean_speed: speeds.iter().map(|&s| s as f64).sum::<f64>() / n,
        max_speed: speeds.iter().copied().fold(0.0, f32::max),
        energy: speeds.iter().map(|&s| 0.5 * (s as f64) * (s as f64)).sum::<f64>() / n,
        escaped: bodies.iter().filter(|b| scene.escaped(b.x, b.y)).count(),
    }
}

/// How far bodies moved between two looks at the same bodies: the mean and
/// the greatest distance.
pub fn moved(before: &[Dyn], after: &[Dyn]) -> (f64, f32) {
    assert_eq!(before.len(), after.len(), "the same bodies");
    let d: Vec<f32> = before.iter().zip(after).map(|(a, b)| ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()).collect();
    (d.iter().map(|&d| d as f64).sum::<f64>() / d.len().max(1) as f64, d.iter().copied().fold(0.0, f32::max))
}
