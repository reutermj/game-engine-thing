//! The physics step on plain arrays, as the mod takes it: bodies as
//! indices in entity order, contacts in pair order, the same narrowphase and
//! solver. Shared by `:tax`, which checks it against the mod bit for bit,
//! and `:parallel_solver`, which swaps the solver under it.

use std::collections::HashMap;
use std::time::Instant;

use engine_ecs::{Entity, World};
use physics::{
    Body, Collider, ContactPair, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Placed, Position, Response, STATIC, Shape, Vec2, Velocity,
};

use crate::narrow;
use crate::solver::{Constraint, SolverBody};

pub const DT: f32 = 1.0 / 60.0;

/// One contact as the arrays keep it: by body index, `a < b`.
#[derive(Clone, Copy)]
pub struct Cached {
    pub a: u32,
    pub b: u32,
    pub normal: Vec2,
    pub depth: f32,
    pub friction: f32,
    pub restitution: f32,
    pub jn: f32,
    pub jt: f32,
    pub pressed: bool,
    pub was_pressed: bool,
}

/// The pile as arrays, in entity order: index `i` is the `i`th entity with
/// a collider, so index order is entity order and pairs sort the same.
pub struct Arrays {
    pub entity: Vec<Entity>,
    pub pos: Vec<Vec2>,
    pub vel: Vec<Vec2>,
    pub collider: Vec<Collider>,
    /// `Body::fixed()` for colliders without a body, as the mod treats them.
    pub body: Vec<Body>,
    /// Indices of the bodies with a `Body`: what the solver moves.
    pub moving: Vec<u32>,
    pub gravity: Vec2,
    pub contacts: Vec<Cached>,
    /// Indices by the left edge of their boxes, kept across steps: the
    /// broadphase re-sorts it by insertion, nearly free once sorted.
    pub by_x: Vec<u32>,
    /// Whether each step also times the broadphase sorting afresh, which
    /// only `:tax` reports: it's a second broadphase a step.
    pub time_fresh_sweep: bool,
}

#[derive(Default, Clone, Copy)]
pub struct Stages {
    /// The broadphase again, sorting afresh: not part of the step.
    pub fresh_sweep: f64,
    pub gravity: f64,
    pub broadphase: f64,
    pub narrowphase: f64,
    pub merge: f64,
    pub solve_gather: f64,
    pub solver: f64,
    pub write_back: f64,
}

impl Arrays {
    pub fn snapshot(w: &World) -> Arrays {
        let pos: HashMap<Entity, Position> = w.values::<Position>().unwrap().into_iter().collect();
        let vel: HashMap<Entity, Velocity> = w.values::<Velocity>().unwrap().into_iter().collect();
        let body: HashMap<Entity, Body> = w.values::<Body>().unwrap().into_iter().collect();
        let mut colliders = w.values::<Collider>().unwrap();
        colliders.sort_by_key(|(e, _)| *e);
        let entity: Vec<Entity> = colliders.iter().map(|(e, _)| *e).collect();
        let index: HashMap<Entity, u32> = entity.iter().enumerate().map(|(i, e)| (*e, i as u32)).collect();
        let at = |e: &Entity| pos[e];
        let gravity = w.values::<Gravity>().unwrap().first().map_or(Vec2::ZERO, |(_, g)| Vec2::new(g.x, g.y));
        let manifolds: HashMap<Entity, Manifold> = w.values::<Manifold>().unwrap().into_iter().collect();
        let responses: HashMap<Entity, Response> = w.values::<Response>().unwrap().into_iter().collect();
        let impulses: HashMap<Entity, Impulse> = w.values::<Impulse>().unwrap().into_iter().collect();
        let mut contacts: Vec<Cached> = w
            .values::<ContactPair>()
            .unwrap()
            .into_iter()
            .map(|(e, p)| {
                let (m, r, j) = (manifolds[&e], responses[&e], impulses[&e]);
                Cached {
                    a: index[&p.a],
                    b: index[&p.b],
                    normal: Vec2::new(m.nx, m.ny),
                    depth: m.depth,
                    friction: r.friction,
                    restitution: r.restitution,
                    jn: j.normal,
                    jt: j.tangent,
                    pressed: m.pressed,
                    was_pressed: m.was_pressed,
                }
            })
            .collect();
        contacts.sort_by_key(|c| (c.a, c.b));
        let mut a = Arrays {
            pos: entity.iter().map(|e| Vec2::new(at(e).x, at(e).y)).collect(),
            vel: entity.iter().map(|e| vel.get(e).map_or(Vec2::ZERO, |v| Vec2::new(v.x, v.y))).collect(),
            collider: colliders.iter().map(|(_, c)| *c).collect(),
            body: entity.iter().map(|e| body.get(e).copied().unwrap_or_else(Body::fixed)).collect(),
            moving: entity.iter().enumerate().filter(|(_, e)| body.contains_key(e)).map(|(i, _)| i as u32).collect(),
            by_x: (0..entity.len() as u32).collect(),
            entity,
            gravity,
            contacts,
            time_fresh_sweep: true,
        };
        a.sort_by_x();
        a
    }

    /// A scene built by hand, with no contacts yet: entity `i` is index `i`.
    /// `parallel_solver` builds its scenes this way; `tax`, which compiles
    /// this file too, snapshots a running pile instead.
    #[allow(dead_code)]
    pub fn of(pos: Vec<Vec2>, collider: Vec<Collider>, body: Vec<Body>, moving: Vec<u32>, gravity: Vec2) -> Arrays {
        let n = pos.len() as u32;
        let mut a = Arrays {
            entity: (0..n).map(|index| Entity { index, generation: 0 }).collect(),
            vel: vec![Vec2::ZERO; pos.len()],
            pos,
            collider,
            body,
            moving,
            gravity,
            contacts: Vec::new(),
            by_x: (0..n).collect(),
            time_fresh_sweep: false,
        };
        a.sort_by_x();
        a
    }

    pub fn placed(&self, i: usize) -> Placed {
        Placed { shape: Shape::of(&self.collider[i]), at: self.pos[i] }
    }

    pub fn sort_by_x(&mut self) {
        let min_x: Vec<f32> = (0..self.pos.len()).map(|i| self.placed(i).aabb().min.x).collect();
        // Insertion sort: bodies move a little each step, so this is linear
        // once the order is established.
        for k in 1..self.by_x.len() {
            let mut j = k;
            while j > 0 && min_x[self.by_x[j - 1] as usize] > min_x[self.by_x[j] as usize] {
                self.by_x.swap(j - 1, j);
                j -= 1;
            }
        }
    }

    /// One step, as the physics mod takes it, with `solve` as its solver.
    pub fn step(&mut self, t: &mut Stages, mut solve: impl FnMut(&mut [SolverBody], &mut [Constraint], f32)) {
        let start = Instant::now();
        for &i in &self.moving {
            let (b, v) = (&self.body[i as usize], &mut self.vel[i as usize]);
            if b.kind == DYNAMIC {
                v.x += self.gravity.x * b.gravity_scale * DT;
                v.y += self.gravity.y * b.gravity_scale * DT;
            }
        }
        let gravity = Instant::now();

        // Sweep and prune along x over boxes grown by the margin, as
        // `near_pairs` grows them.
        self.sort_by_x();
        let boxes: Vec<physics::Aabb> = (0..self.pos.len())
            .map(|i| {
                let b = self.placed(i).aabb();
                let m = Vec2::new(narrow::MARGIN, narrow::MARGIN);
                physics::Aabb { min: b.min - m, max: b.max + m }
            })
            .collect();
        let mut pairs: Vec<(u32, u32)> = Vec::new();
        for (k, &i) in self.by_x.iter().enumerate() {
            let a = boxes[i as usize];
            for &j in &self.by_x[k + 1..] {
                let b = boxes[j as usize];
                if b.min.x > a.max.x {
                    break;
                }
                if a.min.y <= b.max.y && b.min.y <= a.max.y {
                    pairs.push((i.min(j), i.max(j)));
                }
            }
        }
        pairs.sort_unstable();
        let broadphase = Instant::now();
        let pair_count = pairs.len();

        let mut found: Vec<Cached> = Vec::new();
        for (i, j) in pairs {
            let (a, b) = (i as usize, j as usize);
            let (ca, cb, ba, bb) = (&self.collider[a], &self.collider[b], &self.body[a], &self.body[b]);
            let meets = ca.mask & cb.layer != 0 && cb.mask & ca.layer != 0;
            if !meets || ca.sensor || cb.sensor || (ba.kind != DYNAMIC && bb.kind != DYNAMIC) {
                continue;
            }
            let Some(m) = narrow::collide(&self.placed(a), &self.placed(b), self.vel[b] - self.vel[a]) else { continue };
            found.push(Cached {
                a: i,
                b: j,
                normal: m.normal,
                depth: m.depth,
                friction: ba.friction.min(bb.friction),
                restitution: ba.restitution.max(bb.restitution),
                jn: 0.0,
                jt: 0.0,
                pressed: false,
                was_pressed: false,
            });
        }
        let narrowphase = Instant::now();

        let mut k = 0;
        for f in &mut found {
            while k < self.contacts.len() && (self.contacts[k].a, self.contacts[k].b) < (f.a, f.b) {
                k += 1;
            }
            if let Some(old) = self.contacts.get(k).filter(|c| (c.a, c.b) == (f.a, f.b)) {
                (f.jn, f.jt, f.was_pressed) = (old.jn, old.jt, old.pressed);
            }
        }
        self.contacts = found;
        let merge = Instant::now();

        // Statics all stand for one immovable body at the end, as in the mod.
        let mut slot = vec![u32::MAX; self.pos.len()];
        let mut bodies: Vec<SolverBody> = Vec::with_capacity(self.moving.len() + 1);
        for (s, &i) in self.moving.iter().enumerate() {
            let b = &self.body[i as usize];
            let inv_mass = if b.kind == DYNAMIC { b.inv_mass } else { 0.0 };
            bodies.push(SolverBody { v: self.vel[i as usize], inv_mass, pseudo: Vec2::ZERO });
            slot[i as usize] = s as u32;
        }
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        let at = |i: u32| if slot[i as usize] == u32::MAX { still } else { slot[i as usize] };
        let mut constraints: Vec<Constraint> = self
            .contacts
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
        let solve_gather = Instant::now();
        solve(&mut bodies, &mut constraints, DT);
        let solved = Instant::now();

        for (s, &i) in self.moving.iter().enumerate() {
            let (b, body) = (bodies[s], self.body[i as usize]);
            if body.kind == STATIC {
                continue;
            }
            self.vel[i as usize] = b.v;
            let step = if body.kind == KINEMATIC { b.v } else { b.v + b.pseudo };
            self.pos[i as usize] = Vec2::new(self.pos[i as usize].x + step.x * DT, self.pos[i as usize].y + step.y * DT);
        }
        for (c, k) in self.contacts.iter_mut().zip(&constraints) {
            (c.jn, c.jt) = (k.jn, k.jt);
            c.pressed = k.jn > 0.0 || c.depth >= 0.0;
        }
        let done = Instant::now();
        let us = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1e6;
        t.gravity += us(start, gravity);
        t.broadphase += us(gravity, broadphase);
        t.narrowphase += us(broadphase, narrowphase);
        t.merge += us(narrowphase, merge);
        t.solve_gather += us(merge, solve_gather);
        t.solver += us(solve_gather, solved);
        t.write_back += us(solved, done);
        if !self.time_fresh_sweep {
            return;
        }

        // The same sweep with no order kept from the last step, over this
        // step's boxes: what the persistent order is worth. Timed only.
        let fresh_start = Instant::now();
        let mut by_x: Vec<u32> = (0..boxes.len() as u32).collect();
        by_x.sort_unstable_by(|&i, &j| boxes[i as usize].min.x.total_cmp(&boxes[j as usize].min.x));
        let mut fresh: Vec<(u32, u32)> = Vec::new();
        for (k, &i) in by_x.iter().enumerate() {
            let a = boxes[i as usize];
            for &j in &by_x[k + 1..] {
                let b = boxes[j as usize];
                if b.min.x > a.max.x {
                    break;
                }
                if a.min.y <= b.max.y && b.min.y <= a.max.y {
                    fresh.push((i.min(j), i.max(j)));
                }
            }
        }
        fresh.sort_unstable();
        t.fresh_sweep += fresh_start.elapsed().as_secs_f64() * 1e6;
        assert_eq!(fresh.len(), pair_count);
    }
}
