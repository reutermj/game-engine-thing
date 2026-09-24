//! What both contact models share: components, the pile, detection
//! (broadphase and narrowphase over gathered arrays) and the solve over
//! arrays. Only where contacts live, and how games reach them, differ.

use std::collections::HashMap;

use engine_ecs::{Build, Entity, Mut, Query, Without, World, component};
use physics::{Placed, Shape as PShape, Vec2};

use crate::narrow::{MARGIN, collide};
use crate::solver::{Constraint, SolverBody, solve};

pub const DT: f32 = 1.0 / 60.0;
pub const GRAVITY: f32 = 20.0;

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Pos: "rel::Pos" { pub x: f32, pub y: f32 }
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Vel: "rel::Vel" { pub x: f32, pub y: f32 }
}
component! {
    /// A moving body; statics have none.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Body: "rel::Body" { pub inv_mass: f32, pub kinematic: bool }
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Shape: "rel::Shape" { pub hx: f32, pub hy: f32, pub circle: bool }
}
component! {
    /// Solid from above, passable from below.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct OneWay: "rel::OneWay" {}
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Player: "rel::Player" {}
}

component! {
    /// What the player stood on last step, if anything.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Grounded: "rel::Grounded" { pub on: Entity, pub grounded: bool }
}
component! {
    /// Bounces whatever lands on it back up.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Bouncy: "rel::Bouncy" {}
}

/// Whether `q` matches `e`: how a query asks "is this a platform?".
pub fn has<D: engine_ecs::Data, F, C>(q: &mut Query<D, F, C>, e: Entity) -> bool {
    q.with(e, |_, _| ()).is_some()
}

pub fn placed(p: &Pos, s: &Shape) -> Placed {
    let shape = if s.circle { PShape::Circle(s.hx) } else { PShape::Box(Vec2::new(s.hx, s.hy)) };
    Placed { shape, at: Vec2::new(p.x, p.y) }
}

/// A walled box of `n` bodies in rows from the floor up, as the physics
/// pile: `falling` spaces them so they drop; otherwise they start stacked.
pub fn pile(w: &World, n: usize, falling: bool) {
    let mut m = w.between_frames(Build::default()).unwrap();
    let (width, height) = (40.0, 30.0);
    m.spawn((Pos { x: width / 2.0, y: height + 0.5 }, Shape { hx: width / 2.0 + 1.0, hy: 0.5, circle: false }));
    m.spawn((Pos { x: -0.5, y: height / 2.0 }, Shape { hx: 0.5, hy: height, circle: false }));
    m.spawn((Pos { x: width + 0.5, y: height / 2.0 }, Shape { hx: 0.5, hy: height, circle: false }));
    let per_row = ((width - 2.0) / 1.2) as usize;
    let gap = if falling { 1.2 } else { 0.9 };
    for k in 0..n {
        let (col, row) = (k % per_row, k / per_row);
        let jitter = ((k * 7919) % 100) as f32 / 100.0 * 0.2 - 0.1;
        let at = Pos { x: 1.5 + col as f32 * 1.2 + jitter, y: height - 0.46 - row as f32 * gap };
        m.spawn((at, Vel::default(), Body { inv_mass: 1.0, kinematic: false }, Shape { hx: 0.45, hy: 0.45, circle: k % 2 == 0 }));
    }
}

/// One contact the narrowphase found, in entity order (`a < b`).
#[derive(Clone, Copy, Debug)]
pub struct Found {
    pub a: Entity,
    pub b: Entity,
    pub normal: Vec2,
    pub depth: f32,
}

/// A collider as detection sees it.
struct Item {
    e: Entity,
    placed: Placed,
    v: Vec2,
    moves: bool,
}

/// Every contact between colliders, at least one of them moving: a grid
/// broadphase over gathered colliders, then the physics mod's narrowphase.
/// Sorted by pair.
pub fn detect(
    bodies: &mut Query<(&Pos, &Shape, &Vel, &Body)>,
    statics: &mut Query<(&Pos, &Shape), Without<Body>>,
) -> Vec<Found> {
    let mut items = Vec::new();
    bodies.for_each(|row, (p, s, v, _)| items.push(Item { e: row.entity(), placed: placed(p, s), v: Vec2::new(v.x, v.y), moves: true }));
    statics.for_each(|row, (p, s)| items.push(Item { e: row.entity(), placed: placed(p, s), v: Vec2::ZERO, moves: false }));
    items.sort_by_key(|i| i.e);
    // Cells of 2: every item in each cell its grown box covers.
    let cell = 2.0;
    let mut cells: Vec<(i32, i32, u32)> = Vec::new();
    for (i, it) in items.iter().enumerate() {
        let b = it.placed.aabb();
        let c = |v: f32| ((v - MARGIN) / cell).floor() as i32;
        let d = |v: f32| ((v + MARGIN) / cell).floor() as i32;
        for cx in c(b.min.x)..=d(b.max.x) {
            for cy in c(b.min.y)..=d(b.max.y) {
                cells.push((cx, cy, i as u32));
            }
        }
    }
    cells.sort_unstable();
    let mut pairs = Vec::new();
    let mut s = 0;
    while s < cells.len() {
        let key = (cells[s].0, cells[s].1);
        let end = s + cells[s..].iter().take_while(|c| (c.0, c.1) == key).count();
        for i in s..end {
            for j in i + 1..end {
                let (a, b) = (cells[i].2, cells[j].2);
                if items[a as usize].moves || items[b as usize].moves {
                    pairs.push((a.min(b), a.max(b)));
                }
            }
        }
        s = end;
    }
    pairs.sort_unstable();
    pairs.dedup();
    pairs
        .into_iter()
        .filter_map(|(i, j)| {
            let (a, b) = (&items[i as usize], &items[j as usize]);
            collide(&a.placed, &b.placed, b.v - a.v).map(|m| Found { a: a.e, b: b.e, normal: m.normal, depth: m.depth })
        })
        .collect()
}

/// A contact as the solver takes it: the pair, its manifold, what the game
/// made of it, and last step's impulses.
#[derive(Clone, Copy, Debug)]
pub struct Solvable {
    pub a: Entity,
    pub b: Entity,
    pub normal: Vec2,
    pub depth: f32,
    pub restitution: f32,
    pub friction: f32,
    pub jn: f32,
    pub jt: f32,
}

/// Solves `contacts` over the moving bodies, gathered into arrays, and
/// writes velocities and positions back. Returns each contact's impulses.
pub fn solve_step(bodies: &mut Query<(&Body, &mut Vel, &mut Pos)>, contacts: &[Solvable]) -> Vec<(f32, f32)> {
    let mut ids = Vec::new();
    let mut solver_bodies = Vec::new();
    bodies.for_each(|row, (b, v, _)| {
        ids.push(row.entity());
        solver_bodies.push(SolverBody { v: Vec2::new(v.x, v.y), inv_mass: if b.kinematic { 0.0 } else { b.inv_mass }, pseudo: Vec2::ZERO });
    });
    let slot: HashMap<Entity, u32> = ids.iter().enumerate().map(|(i, &e)| (e, i as u32)).collect();
    let still = solver_bodies.len() as u32;
    solver_bodies.push(SolverBody::default());
    let at = |e: Entity| slot.get(&e).copied().unwrap_or(still);
    let mut constraints: Vec<Constraint> = contacts
        .iter()
        .map(|c| Constraint {
            a: at(c.a),
            b: at(c.b),
            normal: c.normal,
            depth: c.depth,
            friction: c.friction,
            restitution: c.restitution,
            jn: c.jn,
            jt: c.jt,
            speed: 0.0,
        })
        .collect();
    solve(&mut solver_bodies, &mut constraints, DT);
    for (e, b) in ids.iter().zip(&solver_bodies) {
        bodies.with(*e, |_, (body, mut v, mut p): (&Body, Mut<Vel>, Mut<Pos>)| {
            (v.x, v.y) = (b.v.x, b.v.y);
            let step = if body.kinematic { b.v } else { b.v + b.pseudo };
            p.x += step.x * DT;
            p.y += step.y * DT;
        });
    }
    constraints.iter().map(|c| (c.jn, c.jt)).collect()
}

/// Gravity into dynamic bodies' velocities: the same first step for both.
pub fn gravity(bodies: &mut Query<(&Body, &mut Vel)>) {
    bodies.for_each(|_, (b, mut v)| {
        if !b.kinematic {
            v.y += GRAVITY * DT;
        }
    });
}
