//! Structural links, the other half of the question: few, long-lived, read
//! every frame. A transform hierarchy (each child's global position from
//! its parent's) in four shapes, and a body's colliders in two. They must
//! agree; `bench` times them.
//!
//! - `ChildOf { parent }` on the child, looked up per child: an entity
//!   field and nothing else.
//! - `Children { list }` on the parent too, as an engine-kept
//!   back-reference would be: the parent is read once, each child looked
//!   up.
//! - `ChildOf` with children stored in parent order, as a table ordered by
//!   that field would keep them (spatial tables are one such order): the
//!   loop keeps the last parent, so lookups are one per run of siblings.
//! - Children in a table per parent, as archetype-per-target relations
//!   (Flecs pairs) store them, emulated with marker combinations: the cost
//!   is on every query that crosses those tables, measured in `bench`.

use engine_ecs::{Build, Entity, Query, With, Without, World, component, field_struct};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Local: "rel::Local" { pub x: f32, pub y: f32 }
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Global: "rel::Global" { pub x: f32, pub y: f32 }
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ChildOf: "rel::ChildOf" { pub parent: Entity }
}
component! {
    #[derive(Debug, Default, PartialEq)]
    pub struct Children: "rel::Children" { pub list: Vec<Entity> }
}

/// A hierarchy of `parents` roots with `per` children each, spawned in
/// parent order or shuffled. Returns the children in spawn order.
pub fn hierarchy(w: &World, parents: usize, per: usize, shuffled: bool) -> Vec<Entity> {
    let mut m = w.between_frames(Build::default()).unwrap();
    let roots: Vec<Entity> = (0..parents).map(|i| m.spawn((Global { x: i as f32, y: 0.0 }, Children::default()))).collect();
    let mut order: Vec<(usize, usize)> = (0..parents).flat_map(|p| (0..per).map(move |c| (p, c))).collect();
    if shuffled {
        // A fixed permutation, so runs are reproducible.
        let mut s = 0x9e3779b9u32;
        for i in (1..order.len()).rev() {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            order.swap(i, s as usize % (i + 1));
        }
    }
    let children: Vec<Entity> = order
        .iter()
        .map(|&(p, c)| m.spawn((ChildOf { parent: roots[p] }, Local { x: 0.0, y: c as f32 + 1.0 }, Global::default())))
        .collect();
    for (&(p, _), &c) in order.iter().zip(&children) {
        m.with_mut::<Children, _>(roots[p], |ch| ch.list.push(c));
    }
    children
}

/// Parents move, so propagation has something to do each run.
pub fn move_roots(roots: &mut Query<&mut Global, Without<ChildOf>>) {
    roots.for_each(|_, mut g| g.y += 1.0);
}

pub fn by_lookup(children: &mut Query<(&ChildOf, &Local, &mut Global)>, roots: &mut Query<&Global, Without<ChildOf>>) {
    children.for_each(|_, (of, l, mut g)| {
        if let Some(p) = roots.with(of.parent, |_, p| *p) {
            *g = Global { x: p.x + l.x, y: p.y + l.y };
        }
    });
}

pub fn by_children(roots: &mut Query<(&Global, &Children), Without<ChildOf>>, children: &mut Query<(&Local, &mut Global), With<ChildOf>>) {
    roots.for_each(|_, (p, ch)| {
        for &c in &ch.list {
            children.with(c, |_, (l, mut g)| *g = Global { x: p.x + l.x, y: p.y + l.y });
        }
    });
}

/// `by_lookup`, remembering the last parent: one lookup per run of
/// siblings when they're stored together, one per child when they aren't.
pub fn by_runs(children: &mut Query<(&ChildOf, &Local, &mut Global)>, roots: &mut Query<&Global, Without<ChildOf>>) {
    let mut last: Option<(Entity, Global)> = None;
    children.for_each(|_, (of, l, mut g)| {
        let p = match last {
            Some((e, p)) if e == of.parent => p,
            _ => {
                let Some(p) = roots.with(of.parent, |_, p| *p) else { return };
                last = Some((of.parent, p));
                p
            }
        };
        *g = Global { x: p.x + l.x, y: p.y + l.y };
    });
}

// ---- A body's colliders ----

field_struct! {
    #[derive(Debug, Default, Copy, PartialEq)]
    pub struct Part { pub dx: f32, pub dy: f32, pub hx: f32, pub hy: f32 }
}
component! {
    /// Compound data: every collider in the body's own row.
    #[derive(Debug, Default, PartialEq)]
    pub struct Parts: "rel::Parts" { pub list: Vec<Part> }
}
component! {
    /// A collider entity, placed relative to its body.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ColliderOf: "rel::ColliderOf" { pub body: Entity, pub part: Part }
}

/// Every collider's world box, `[min x, min y, max x, max y]`, in the order
/// the query visits them.
pub type Boxes = Vec<[f32; 4]>;

fn place(g: &Global, p: &Part) -> [f32; 4] {
    let (x, y) = (g.x + p.dx, g.y + p.dy);
    [x - p.hx, y - p.hy, x + p.hx, y + p.hy]
}

pub fn compound_boxes(bodies: &mut Query<(&Global, &Parts)>, out: &mut Boxes) {
    out.clear();
    bodies.for_each(|_, (g, parts)| out.extend(parts.list.iter().map(|p| place(g, p))));
}

pub fn collider_boxes(colliders: &mut Query<&ColliderOf>, bodies: &mut Query<&Global, With<Parts>>, out: &mut Boxes) {
    out.clear();
    colliders.for_each(|_, c| {
        if let Some(g) = bodies.with(c.body, |_, g| *g) {
            out.push(place(&g, &c.part));
        }
    });
}

/// `n` bodies with `per` parts each, both ways: `Parts` on the body and a
/// `ColliderOf` entity per part (spawned after all bodies, in body order).
pub fn bodies(w: &World, n: usize, per: usize) {
    let mut m = w.between_frames(Build::default()).unwrap();
    let parts = |i: usize| (0..per).map(move |k| Part { dx: k as f32, dy: k as f32 * 0.5, hx: 0.5, hy: 0.5 + i as f32 * 1e-3 });
    let bodies: Vec<Entity> = (0..n).map(|i| m.spawn((Global { x: i as f32 * 3.0, y: 0.0 }, Parts { list: parts(i).collect() }))).collect();
    for (i, &b) in bodies.iter().enumerate() {
        for part in parts(i) {
            m.spawn((ColliderOf { body: b, part },));
        }
    }
}

// ---- Fragmentation ----

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Vel2: "rel::Vel2" { pub x: f32, pub y: f32 }
}

macro_rules! markers {
    ($($m:ident $name:literal),*) => {
        $(component! {
            #[derive(Debug, Default, PartialEq, Copy)]
            pub struct $m: $name {}
        })*
        /// Gives `e` the markers of `bits`: 2^8 combinations, a table each.
        fn mark(m: &mut engine_ecs::WorldMut<'_>, e: Entity, bits: usize) {
            let mut k = 0;
            $(if bits >> k & 1 == 1 { m.insert(e, $m {}); } k += 1;)*
            let _ = k;
        }
    };
}
markers!(M0 "rel::M0", M1 "rel::M1", M2 "rel::M2", M3 "rel::M3", M4 "rel::M4", M5 "rel::M5", M6 "rel::M6", M7 "rel::M7");

/// `n` moving rows spread over `tables` tables (a power of two up to 256),
/// as relations stored by target would spread one component's rows.
pub fn fragmented(w: &World, n: usize, tables: usize) {
    let mut m = w.between_frames(Build::default()).unwrap();
    for i in 0..n {
        let e = m.spawn((Global::default(), Vel2 { x: 1.0, y: 0.5 }));
        mark(&mut m, e, i % tables);
    }
}

pub fn integrate(q: &mut Query<(&mut Global, &Vel2)>) {
    q.for_each(|_, (mut g, v)| {
        g.x += v.x;
        g.y += v.y;
    });
}

#[cfg(test)]
mod tests {
    use engine_ecs::harness::{Cx, IntoSystem, Schedule};

    use super::*;

    fn globals(w: &World, children: &[Entity]) -> Vec<Global> {
        let all = w.values::<Global>().unwrap();
        children.iter().map(|c| all.iter().find(|(e, _)| e == c).unwrap().1).collect()
    }

    #[test]
    fn every_propagation_agrees() {
        for shuffled in [false, true] {
            let mut results = Vec::new();
            for way in 0..3 {
                let w = World::new();
                let children = hierarchy(&w, 20, 5, shuffled);
                let moved = |_: &mut Cx, mut r: Query<&mut Global, Without<ChildOf>>| move_roots(&mut r);
                let step = match way {
                    0 => (|_: &mut Cx, mut c: Query<(&ChildOf, &Local, &mut Global)>, mut r: Query<&Global, Without<ChildOf>>| by_lookup(&mut c, &mut r)).system(&w, "p"),
                    1 => (|_: &mut Cx, mut r: Query<(&Global, &Children), Without<ChildOf>>, mut c: Query<(&Local, &mut Global), With<ChildOf>>| by_children(&mut r, &mut c)).system(&w, "p"),
                    _ => (|_: &mut Cx, mut c: Query<(&ChildOf, &Local, &mut Global)>, mut r: Query<&Global, Without<ChildOf>>| by_runs(&mut c, &mut r)).system(&w, "p"),
                };
                let s = Schedule { systems: vec![moved.system(&w, "move"), step] };
                s.run_sequential(&w);
                s.run_sequential(&w);
                results.push(globals(&w, &children));
            }
            // Root i is at (i, 2) after two moves; its c-th child 1 + c below.
            assert!(results[0].iter().all(|g| g.y >= 3.0), "{:?}", &results[0][..5]);
            assert_eq!(results[0], results[1], "shuffled {shuffled}");
            assert_eq!(results[0], results[2], "shuffled {shuffled}");
        }
    }

    #[test]
    fn both_collider_layouts_place_the_same_boxes() {
        use std::sync::{Arc, Mutex};
        let w = World::new();
        bodies(&w, 30, 3);
        let (a, b) = (Arc::new(Mutex::new(Vec::new())), Arc::new(Mutex::new(Vec::new())));
        let (a2, b2) = (a.clone(), b.clone());
        let s = Schedule {
            systems: vec![
                (move |_: &mut Cx, mut q: Query<(&Global, &Parts)>| compound_boxes(&mut q, &mut a2.lock().unwrap())).system(&w, "a"),
                (move |_: &mut Cx, mut c: Query<&ColliderOf>, mut q: Query<&Global, With<Parts>>| {
                    collider_boxes(&mut c, &mut q, &mut b2.lock().unwrap())
                })
                .system(&w, "b"),
            ],
        };
        s.run_sequential(&w);
        let (a, b) = (a.lock().unwrap().clone(), b.lock().unwrap().clone());
        assert_eq!(a.len(), 90);
        // Body 7's second part, from the scene's numbers.
        let want = [21.5, 0.5 - 0.507, 22.5, 0.5 + 0.507];
        assert!(a.iter().any(|b| b.iter().zip(&want).all(|(x, y)| (x - y).abs() < 1e-5)), "{:?}", &a[..6]);
        assert_eq!(a, b);
    }
}

