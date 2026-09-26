//! The physics mod's pure parts, compiled on their own for their unit
//! tests: the mod itself is a cdylib.

#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

/// Whole steps over plain arrays: gravity, every pair's contact warm-started
/// by pair, the solver, then positions. What the mod does, minus the world.
#[cfg(test)]
mod stack {
    use physics::{Placed, Vec2};

    use crate::narrow::collide;
    use crate::solver::{Constraint, SolverBody, solve};

    pub const DT: f32 = 1.0 / 60.0;

    /// Body 0 is the ground; the rest fall under `g`. Returns the speeds after
    /// `steps`, and the deepest overlap.
    pub fn simulate(shapes: &[Placed], g: f32, steps: usize) -> (Vec<f32>, f32) {
        let mut placed = shapes.to_vec();
        let mut bodies: Vec<SolverBody> =
            (0..placed.len()).map(|i| SolverBody { inv_mass: if i == 0 { 0.0 } else { 1.0 }, ..Default::default() }).collect();
        let mut cache: Vec<Constraint> = Vec::new();
        let mut deepest = 0.0f32;
        for _ in 0..steps {
            for b in &mut bodies[1..] {
                b.gravity = Vec2::new(0.0, g * DT);
                b.v += b.gravity;
            }
            let mut contacts = Vec::new();
            for i in 0..placed.len() {
                for j in i + 1..placed.len() {
                    let rel = bodies[j].v - bodies[i].v;
                    if let Some(m) = collide(&placed[i], &placed[j], rel) {
                        let old = cache.iter().find(|c| (c.a, c.b) == (i as u32, j as u32));
                        contacts.push(Constraint {
                            a: i as u32,
                            b: j as u32,
                            normal: m.normal,
                            depth: m.depth,
                            friction: 0.4,
                            restitution: 0.1,
                            jn: old.map_or(0.0, |c| c.jn),
                            jt: old.map_or(0.0, |c| c.jt),
                            speed: 0.0,
                        });
                    }
                }
            }
            solve(&mut bodies, &mut contacts, DT);
            for (p, b) in placed.iter_mut().zip(&bodies).skip(1) {
                p.at += b.displacement(DT);
            }
            deepest = contacts.iter().map(|c| c.depth).fold(0.0, f32::max);
            cache = contacts;
        }
        (bodies[1..].iter().map(|b| b.v.len()).collect(), deepest)
    }

    #[test]
    fn a_tall_column_comes_to_rest_without_sinking() {
        use physics::{circle, rect};
        for (label, shape) in [("boxes", rect(Vec2::ZERO, Vec2::new(0.45, 0.45))), ("circles", circle(Vec2::ZERO, 0.45))] {
            let mut shapes = vec![rect(Vec2::new(0.0, 0.5), Vec2::new(5.0, 0.5))];
            for k in 0..10 {
                shapes.push(Placed { at: Vec2::new(0.0, -0.5 - k as f32 * 1.0), ..shape });
            }
            let (speeds, deepest) = simulate(&shapes, 20.0, 600);
            assert!(speeds.iter().all(|&s| s < 0.01), "{label}: {speeds:?}");
            assert!(deepest < 0.01, "{label}: sunk {deepest}");
        }
    }
}
