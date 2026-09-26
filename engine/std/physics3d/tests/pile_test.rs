//! A pile of spheres and boxes dropped into a walled box settles in the
//! world: at rest, above the floor, inside the walls, barely overlapping,
//! touching several neighbors each (a pile, not columns), with the 3D
//! spatial order intact.

use engine_ecs::{Build, World};
use physics3d::{Body, Collider, ContactPair, Manifold, Position, Static, Vec3, Velocity, step};

/// Half the floor's inner width.
const HALF: f32 = 3.0;

fn wall(m: &mut engine_ecs::WorldMut<'_>, at: Vec3, half: Vec3) {
    m.spawn((Position { x: at.x, y: at.y, z: at.z }, Collider::cuboid(half), Body::new(0.0), Static {}));
}

fn pile(n: usize) -> World {
    let w = World::new();
    let s = step(&w);
    drop(s);
    {
        let mut m = w.between_frames(Build::default()).unwrap();
        wall(&mut m, Vec3::new(0.0, -0.5, 0.0), Vec3::new(HALF + 1.0, 0.5, HALF + 1.0));
        for (x, z) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
            let at = Vec3::new(x * (HALF + 0.5), 5.0, z * (HALF + 0.5));
            let half = Vec3::new(if x != 0.0 { 0.5 } else { HALF + 1.0 }, 5.0, if z != 0.0 { 0.5 } else { HALF + 1.0 });
            wall(&mut m, at, half);
        }
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut jitter = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            ((seed >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.4
        };
        let per = 5;
        for k in 0..n {
            let (i, j, l) = (k % per, (k / per) % per, k / (per * per));
            let at = Vec3::new(i as f32 * 1.1 - 2.2 + jitter(), 1.0 + l as f32 * 1.2, j as f32 * 1.1 - 2.2 + jitter());
            let c = if k % 2 == 0 { Collider::sphere(0.5) } else { Collider::cuboid(Vec3::splat(0.5)) };
            m.spawn((Position { x: at.x, y: at.y, z: at.z }, c, Body::new(1.0), Velocity::default()));
        }
    }
    w
}

#[test]
fn a_pile_settles_in_the_world() {
    let n = 200;
    let w = pile(n);
    let s = step(&w);
    for _ in 0..600 {
        s.run_sequential(&w);
    }
    let bodies: Vec<(engine_ecs::Entity, Position)> = w.values::<Position>().unwrap();
    let speeds = w.values::<Velocity>().unwrap();
    assert_eq!(speeds.len(), n);
    let fastest = speeds.iter().map(|(_, v)| Vec3::new(v.x, v.y, v.z).len()).fold(0.0, f32::max);
    assert!(fastest < 0.05, "still moving at {fastest}");
    let moving: std::collections::HashSet<_> = speeds.iter().map(|(e, _)| *e).collect();
    for (e, p) in bodies.iter().filter(|(e, _)| moving.contains(e)) {
        assert!(p.y > 0.4 && p.x.abs() < HALF && p.z.abs() < HALF, "{e:?} at {p:?}");
    }
    let manifolds = w.values::<Manifold>().unwrap();
    let deepest = manifolds.iter().map(|(_, m)| m.depth).fold(0.0, f32::max);
    assert!(deepest < 0.02, "a contact {deepest} deep");
    // A pile: bodies rest on several others. Columns would be about two
    // contacts a body, the one below and the one above (4.1 measured here).
    let touching = manifolds.iter().filter(|(_, m)| m.depth > -0.01).count();
    let per_body = 2.0 * touching as f32 / n as f32;
    assert!(per_body > 3.0, "{touching} touching contacts for {n} bodies: {per_body} a body");
    assert_eq!(w.values::<ContactPair>().unwrap().len(), manifolds.len());
    for t in w.tables() {
        let Some(spatial) = &t.spatial else { continue };
        let order = spatial.pages.read().unwrap();
        assert_eq!(order.dims(), 3);
        order.check(&t.rows.read().unwrap(), 2.0, 1.0).unwrap();
    }
}
