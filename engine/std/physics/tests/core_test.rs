//! The physics mod's pure parts, compiled on their own for their unit
//! tests: the mod itself is a cdylib.

#[path = "../narrow.rs"]
mod narrow;
// The mod reads `Sleepers`'s lists; the arithmetic is what is tested.
#[allow(dead_code)]
#[path = "../sleep.rs"]
mod sleep;
#[path = "../solver.rs"]
mod solver;

/// Whole steps over plain arrays: gravity, every pair's contact warm-started
/// by pair (and each point by feature), the solver, then positions and
/// rotations. What the mod does, minus the world.
#[cfg(test)]
mod stack {
    use physics::{Collider, Placed, Rot, Shape, Vec2};

    use crate::narrow::{collide, collide_turned};
    use crate::solver::{Constraint, ContactPoint, Points, SolverBody, Spinning, solve_points};

    pub const DT: f32 = 1.0 / 60.0;

    /// Body 0 is the ground; the rest fall under `g`, those `turns` says
    /// turning (they need a rotation in `shapes`). Returns each body's shape,
    /// solver state and angular velocity after `steps`, and the deepest
    /// overlap.
    pub fn run(shapes: &[Placed], turns: &[bool], g: f32, steps: usize) -> (Vec<Placed>, Vec<(SolverBody, f32)>, f32) {
        let mut placed = shapes.to_vec();
        let inertia = |p: &Placed| match p.shape {
            Shape::Box(h) => Collider::rect(h.x, h.y).inertia_per_mass(),
            Shape::Circle(r) => Collider::circle(r).inertia_per_mass(),
        };
        let mut bodies: Vec<SolverBody> =
            (0..placed.len()).map(|i| SolverBody { inv_mass: if i == 0 { 0.0 } else { 1.0 }, ..Default::default() }).collect();
        let mut spinning: Vec<Spinning> =
            (1..placed.len()).filter(|&i| turns[i]).map(|i| Spinning::new(i as u32, 0.0, inertia(&placed[i]))).collect();
        // Each contact, its points and their features.
        let mut cache: Vec<(Constraint, Points, [u16; 2])> = Vec::new();
        let mut deepest = 0.0f32;
        for _ in 0..steps {
            for b in &mut bodies[1..] {
                b.gravity = Vec2::new(0.0, g * DT);
                b.v += b.gravity;
            }
            let mut contacts = Vec::new();
            let mut points = Vec::new();
            let mut ids = Vec::new();
            for i in 0..placed.len() {
                for j in i + 1..placed.len() {
                    let (a, b) = (&placed[i], &placed[j]);
                    let old = cache.iter().find(|(c, _, _)| (c.a, c.b) == (i as u32, j as u32));
                    let mut c = Constraint {
                        a: i as u32,
                        b: j as u32,
                        friction: 0.4,
                        restitution: 0.1,
                        jn: old.map_or(0.0, |(c, _, _)| c.jn),
                        jt: old.map_or(0.0, |(c, _, _)| c.jt),
                        ..Constraint::default()
                    };
                    if a.rot.is_none() && b.rot.is_none() {
                        let Some(m) = collide(a, b, bodies[j].v - bodies[i].v) else { continue };
                        (c.normal, c.depth) = (m.normal, m.depth);
                    } else {
                        let Some(m) = collide_turned(a, b) else { continue };
                        (c.normal, c.depth) = (m.normal, m.depth);
                        let mut pts = Points { count: m.count, ..Points::default() };
                        for (k, p) in m.points[..m.count as usize].iter().enumerate() {
                            let last =
                                old.and_then(|(_, was, ids)| (0..was.count as usize).find(|&n| ids[n] == p.id).map(|n| was.point[n]));
                            let (jn, jt) = last.filter(|_| old.is_some_and(|o| o.1.solved)).map_or((0.0, 0.0), |l| (l.jn, l.jt));
                            pts.point[k] = ContactPoint { ra: p.ra, rb: p.rb, separation: p.separation, jn, jt };
                        }
                        points.push(pts);
                        c = c.with_points(points.len() - 1);
                        ids.push((contacts.len(), [m.points[0].id, m.points[1].id]));
                    }
                    contacts.push(c);
                }
            }
            solve_points(&mut bodies, &mut spinning, &mut contacts, &mut points, DT);
            for (p, b) in placed.iter_mut().zip(&bodies).skip(1) {
                p.at += b.displacement(DT);
            }
            for s in &spinning {
                let p = &mut placed[s.body as usize];
                p.rot = p.rot.map(|q| s.rotation(q));
            }
            deepest = contacts.iter().map(|c| c.depth).fold(0.0, f32::max);
            let features = |k: usize| ids.iter().find(|(at, _)| *at == k).map_or([0; 2], |(_, f)| *f);
            cache = contacts
                .into_iter()
                .enumerate()
                .map(|(k, c)| {
                    let pts = c.points.checked_sub(1).map_or(Points::default(), |at| points[at as usize]);
                    (c, pts, features(k))
                })
                .collect();
        }
        let spin = |i: usize| spinning.iter().find(|s| s.body as usize == i).map_or(0.0, |s| s.w);
        let bodies = bodies.iter().enumerate().map(|(i, b)| (*b, spin(i))).collect();
        (placed, bodies, deepest)
    }

    /// Body 0 is the ground; the rest fall under `g`. Returns the speeds after
    /// `steps`, and the deepest overlap.
    pub fn simulate(shapes: &[Placed], g: f32, steps: usize) -> (Vec<f32>, f32) {
        let (_, bodies, deepest) = run(shapes, &vec![false; shapes.len()], g, steps);
        (bodies[1..].iter().map(|(b, _)| b.v.len()).collect(), deepest)
    }

    /// Turned by `a`.
    pub fn turned(p: Placed, a: f32) -> Placed {
        p.turned(Rot::from_angle(a))
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

    /// A platform whose top is y = 0 (y down) and whose right edge is x = 0,
    /// and a unit box resting on it with its center at `x`.
    fn on_the_edge(x: f32, turns: bool) -> (Vec<Placed>, Vec<bool>) {
        use physics::rect;
        let platform = rect(Vec2::new(-2.5, 0.5), Vec2::new(2.5, 0.5));
        let b = rect(Vec2::new(x, -0.5), Vec2::new(0.5, 0.5));
        (vec![platform, if turns { turned(b, 0.0) } else { b }], vec![false, turns])
    }

    #[test]
    fn a_box_past_the_edge_tips_over_it_and_falls() {
        // Its center 0.2 past the edge: gravity turns it about the corner.
        let (shapes, turns) = on_the_edge(0.2, true);
        let (placed, _, _) = run(&shapes, &turns, 20.0, 90);
        let b = placed[1];
        assert!(b.rot.unwrap().angle().abs() > 0.5, "turned: {b:?}");
        assert!(b.at.y > 0.0, "and fell past the top: {b:?}");
        assert!(b.at.x > 0.2, "off the side it leaned to: {b:?}");
    }

    #[test]
    fn a_box_that_cannot_turn_stays_on_the_edge() {
        let (shapes, turns) = on_the_edge(0.2, false);
        let (placed, bodies, _) = run(&shapes, &turns, 20.0, 90);
        assert!((placed[1].at.y + 0.5).abs() < 0.01 && bodies[1].0.v.len() < 0.01, "{:?}", placed[1]);
    }

    #[test]
    fn a_box_mostly_on_the_platform_stays_upright() {
        let (shapes, turns) = on_the_edge(-0.2, true);
        let (placed, bodies, deepest) = run(&shapes, &turns, 20.0, 120);
        let b = placed[1];
        assert!(b.rot.unwrap().angle().abs() < 0.01, "{b:?}");
        assert!(bodies[1].0.v.len() < 0.01 && bodies[1].1.abs() < 0.01, "at rest: {:?}", bodies[1]);
        assert!(deepest < 0.01, "sunk {deepest}");
    }

    #[test]
    fn a_stack_of_boxes_that_turn_settles_upright() {
        use physics::rect;
        let mut shapes = vec![rect(Vec2::new(0.0, 0.5), Vec2::new(5.0, 0.5))];
        for k in 0..6 {
            // A little off each other, as a stack is never exact.
            let jitter = [0.0, 0.03, -0.02, 0.04, -0.03, 0.01][k];
            shapes.push(turned(rect(Vec2::new(jitter, -0.45 - k as f32 * 0.92), Vec2::new(0.45, 0.45)), 0.0));
        }
        let turns: Vec<bool> = (0..shapes.len()).map(|i| i > 0).collect();
        let (placed, bodies, deepest) = run(&shapes, &turns, 20.0, 300);
        for (p, (b, w)) in placed.iter().zip(&bodies).skip(1) {
            assert!(p.rot.unwrap().angle().abs() < 0.02, "upright: {p:?}");
            assert!(b.v.len() < 0.01 && w.abs() < 0.01, "at rest: {b:?} {w}");
        }
        let top = placed.last().unwrap().at.y;
        assert!((top - (-0.45 - 5.0 * 0.9)).abs() < 0.05, "stands 6 high: top at {top}");
        assert!(deepest < 0.02, "sunk {deepest}");
    }
}

/// Sleeping's arithmetic: how long still is long enough, and islands.
#[cfg(test)]
mod sleeping {
    use engine_api::Entity;

    use crate::sleep::{Sleepers, islands};

    fn body(index: u32) -> Entity {
        Entity { index, generation: 0 }
    }

    /// As many steps as a body counting seconds in `f32` took to reach the
    /// time, and so falling asleep in the step it did then, where the
    /// quotient is a step early at 120 a second.
    #[test]
    fn enough_steps_are_as_many_as_seconds_summed_took() {
        let mut s = Sleepers::default();
        for (hz, steps) in [(60.0, 30), (30.0, 15), (120.0, 61), (64.0, 32)] {
            let dt = 1.0 / hz;
            let (mut sum, mut n) = (0.0f32, 0);
            while sum < 0.5 {
                (sum, n) = (sum + dt, n + 1);
            }
            assert_eq!((s.enough(dt, 0.5), n), (steps, steps), "{hz} a second");
        }
        assert_eq!(s.enough(1e-9, 1.0), u64::MAX, "a sum that stops growing never gets there");
    }

    /// Three bodies pressed together are one island, which falls asleep
    /// when the least still of them has been for `enough` steps, numbered
    /// after the last; a moving body against a sleeping island wakes it, a
    /// still one waits to fall asleep in its own.
    #[test]
    fn an_island_falls_asleep_as_one_and_a_moving_body_wakes_what_it_presses() {
        let between = [(body(1), body(2)), (body(2), body(3))];
        let mut last = 7;
        let (wake, fell) = islands(30, &[(body(1), 30), (body(2), 29), (body(3), 40)], &between, &[], &mut last);
        assert_eq!((wake, fell, last), (vec![], vec![], 7), "one of them still for too short a time");
        let (wake, fell) = islands(30, &[(body(1), 30), (body(2), 31), (body(3), 40), (body(9), 30)], &between, &[], &mut last);
        assert!(wake.is_empty());
        assert_eq!(fell, [(body(1), 8), (body(2), 8), (body(3), 8), (body(9), 9)], "two islands, numbered in turn");
        let against = [(body(4), 3), (body(5), 5)];
        let (wake, fell) = islands(30, &[(body(4), 0), (body(5), 12)], &[], &against, &mut last);
        assert_eq!((wake, fell), (vec![3, 5], vec![]), "moving, or not still for long enough, wakes what it presses on");
        let (wake, _) = islands(30, &[(body(5), 30)], &[], &against, &mut last);
        assert!(wake.is_empty(), "still: it falls asleep on its own");
    }
}
