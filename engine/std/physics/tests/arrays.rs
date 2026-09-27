//! The physics step on plain arrays, as the mod takes it: bodies as
//! indices in entity order, contacts in pair order, the same narrowphase and
//! solver. Shared by `:tax`, which checks it against the mod bit for bit,
//! and `:parallel_solver`, which swaps the solver under it.

use std::collections::HashMap;
use std::time::Instant;

use engine_ecs::{Entity, World};
use physics::{
    Body, Collider, ContactPair, ContactPoints, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Placed, Position, Response, Rot, Rotation,
    STATIC, Shape, Spin, Tuning, Vec2, Velocity,
};

use crate::narrow;
use crate::solver::{Constraint, ContactPoint, Points, SolverBody, Spinning};

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
    /// Its points in `Arrays::points`, plus one, when either end is
    /// turned; 0 when not.
    pub points: u32,
}

/// A contact's points, as the world has them: how many this step
/// (`Manifold::points`), how many the last solve solved at
/// (`Manifold::solved`), and its `ContactPoints`.
#[derive(Clone, Copy, Default)]
pub struct Pts {
    pub count: u8,
    pub solved: u8,
    pub cp: ContactPoints,
}

/// A solver: `solver::solve`, or one of the comparison's variants, over
/// contacts without points; or, wrapped in `WithPoints`, one over contacts
/// some of which have them.
pub trait Solve {
    fn solve(&mut self, bodies: (&mut [SolverBody], &mut [Spinning]), contacts: &mut [Constraint], points: &mut [Points], dt: f32);
}

impl<F: FnMut(&mut [SolverBody], &mut [Constraint], f32)> Solve for F {
    fn solve(
        &mut self,
        (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
        contacts: &mut [Constraint],
        points: &mut [Points],
        dt: f32,
    ) {
        assert!(points.is_empty() && spinning.is_empty(), "a solver without rotation, where something turns");
        self(bodies, contacts, dt)
    }
}

#[allow(dead_code)] // `:parallel_solver`'s solvers have no points.
pub struct WithPoints<F>(pub F);

impl<F: FnMut(&mut [SolverBody], &mut [Spinning], &mut [Constraint], &mut [Points], f32)> Solve for WithPoints<F> {
    fn solve(
        &mut self,
        (bodies, spinning): (&mut [SolverBody], &mut [Spinning]),
        contacts: &mut [Constraint],
        points: &mut [Points],
        dt: f32,
    ) {
        (self.0)(bodies, spinning, contacts, points, dt)
    }
}

/// How a contact's points start from the last step's impulses: by feature
/// id, as the mod does, or (for the comparison's variants) by the nearest
/// of last step's points, or not at all.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Warm {
    Ids,
    Nearest,
    None,
    /// By feature id, and a point whose feature is new by the nearest.
    Either,
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
    /// Each collider's `Rotation`, and each body's `Spin`, if it has one.
    pub rot: Vec<Option<Rot>>,
    pub spin: Vec<Option<f32>>,
    pub warm: Warm,
    /// Whether a contact keeps only its deepest point (the comparison's
    /// variant of one point a contact).
    pub deepest: bool,
    /// The points of the contacts that have them (`Cached::points`), as
    /// the world's `ContactPoints` has them.
    pub points: Vec<Pts>,
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
        // The arrays solve at the default substeps, so a world tuned
        // otherwise isn't one they can be the mod's step on.
        let tuned = w.values::<Tuning>().unwrap_or_default();
        assert!(tuned.iter().all(|(_, t)| t.substeps() == Tuning::DEFAULT.substeps()), "the arrays can't follow a Tuning: {tuned:?}");
        let manifolds: HashMap<Entity, Manifold> = w.values::<Manifold>().unwrap().into_iter().collect();
        let responses: HashMap<Entity, Response> = w.values::<Response>().unwrap().into_iter().collect();
        let impulses: HashMap<Entity, Impulse> = w.values::<Impulse>().unwrap().into_iter().collect();
        let contact_points: HashMap<Entity, ContactPoints> = w.values::<ContactPoints>().unwrap_or_default().into_iter().collect();
        let rotations: HashMap<Entity, Rotation> = w.values::<Rotation>().unwrap_or_default().into_iter().collect();
        let spins: HashMap<Entity, Spin> = w.values::<Spin>().unwrap_or_default().into_iter().collect();
        let mut points = Vec::new();
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
                    points: if m.points == 0 {
                        0
                    } else {
                        points.push(Pts { count: m.points, solved: m.solved, cp: contact_points[&e] });
                        points.len() as u32
                    },
                }
            })
            .collect();
        contacts.sort_by_key(|c| (c.a, c.b));
        let rot = entity.iter().map(|e| rotations.get(e).map(|q| q.rot())).collect();
        let spin = entity.iter().map(|e| spins.get(e).map(|s| s.w)).collect();
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
            rot,
            spin,
            warm: Warm::Ids,
            deepest: false,
            points,
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
            rot: vec![None; n as usize],
            spin: vec![None; n as usize],
            warm: Warm::Ids,
            deepest: false,
            points: Vec::new(),
        };
        a.sort_by_x();
        a
    }

    pub fn placed(&self, i: usize) -> Placed {
        Placed { shape: Shape::of(&self.collider[i]), at: self.pos[i], rot: self.rot[i] }
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

    /// Every dynamic body turns: a `Rotation` and a `Spin` on each.
    #[allow(dead_code)]
    pub fn turning(mut self) -> Arrays {
        for i in 0..self.pos.len() {
            if self.body[i].kind == DYNAMIC {
                (self.rot[i], self.spin[i]) = (Some(Rot::IDENTITY), Some(0.0));
            }
        }
        self
    }

    /// One step, as the physics mod takes it, with `solve` as its solver.
    pub fn step(&mut self, t: &mut Stages, mut solve: impl Solve) {
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
        let mut found_points: Vec<Pts> = Vec::new();
        for (i, j) in pairs {
            let (a, b) = (i as usize, j as usize);
            let (ca, cb, ba, bb) = (&self.collider[a], &self.collider[b], &self.body[a], &self.body[b]);
            let meets = ca.mask & cb.layer != 0 && cb.mask & ca.layer != 0;
            if !meets || ca.sensor || cb.sensor || (ba.kind != DYNAMIC && bb.kind != DYNAMIC) {
                continue;
            }
            // As the mod's `meet`: points if either is turned.
            let (pa, pb) = (self.placed(a), self.placed(b));
            let (m, points) = if pa.rot.is_none() && pb.rot.is_none() {
                let Some(m) = narrow::collide(&pa, &pb, self.vel[b] - self.vel[a]) else { continue };
                (m, 0)
            } else {
                let Some(mut g) = narrow::collide_turned(&pa, &pb) else { continue };
                if self.deepest && g.count == 2 {
                    let deeper = if g.points[1].separation < g.points[0].separation { 1 } else { 0 };
                    (g.points[0], g.count) = (g.points[deeper], 1);
                }
                found_points.push(Pts { count: g.count, solved: 0, cp: points_of(&g) });
                (narrow::Manifold { normal: g.normal, depth: g.depth }, found_points.len() as u32)
            };
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
                points,
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
                // The world's `ContactPoints` persists with the contact: so do
                // its points' impulses.
                if let (Some(new), Some(was)) = (f.points.checked_sub(1), old.points.checked_sub(1)) {
                    let (new, was) = (&mut found_points[new as usize], &self.points[was as usize]);
                    match self.warm {
                        Warm::Nearest => nearest(new, was, false),
                        Warm::Either => nearest(new, was, true),
                        _ => {
                            new.solved = was.solved;
                            (new.cp.normals, new.cp.tangents, new.cp.solved_ids) = (was.cp.normals, was.cp.tangents, was.cp.solved_ids);
                        }
                    }
                }
            }
        }
        self.contacts = found;
        self.points = found_points;
        let merge = Instant::now();

        // Statics all stand for one immovable body at the end, as in the mod.
        let mut slot = vec![u32::MAX; self.pos.len()];
        let mut bodies: Vec<SolverBody> = Vec::with_capacity(self.moving.len() + 1);
        let mut spinning: Vec<Spinning> = Vec::new();
        for (s, &i) in self.moving.iter().enumerate() {
            let b = &self.body[i as usize];
            // The gravity just added, the solver's to spread over its
            // substeps, computed as it was added.
            let (inv_mass, g) = if b.kind == DYNAMIC { (b.inv_mass, self.gravity) } else { (0.0, Vec2::ZERO) };
            let gravity = Vec2::new(g.x * b.gravity_scale * DT, g.y * b.gravity_scale * DT);
            let v = self.vel[i as usize];
            // As the mod's `Turning` walk fills them in: a body with a
            // rotation and a spin turns, pushed round if it's dynamic.
            match (self.rot[i as usize], self.spin[i as usize], b.kind) {
                (Some(_), Some(w), DYNAMIC) => {
                    spinning.push(Spinning::new(s as u32, w, inv_mass * self.collider[i as usize].inertia_per_mass()))
                }
                (Some(_), Some(w), KINEMATIC) => spinning.push(Spinning::new(s as u32, w, 0.0)),
                _ => {}
            }
            bodies.push(SolverBody::new(v, inv_mass, gravity));
            slot[i as usize] = s as u32;
        }
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        let at = |i: u32| if slot[i as usize] == u32::MAX { still } else { slot[i as usize] };
        let mut points: Vec<Points> = Vec::new();
        let mut constraints: Vec<Constraint> = self
            .contacts
            .iter()
            .map(|c| {
                // Over the split impulse's `Constraint`, which has no points,
                // the update has nothing to fill.
                #[allow(clippy::needless_update)]
                let k = Constraint {
                    a: at(c.a),
                    b: at(c.b),
                    normal: c.normal,
                    depth: c.depth,
                    friction: c.friction,
                    restitution: c.restitution,
                    jn: c.jn,
                    jt: c.jt,
                    speed: 0.0,
                    ..Constraint::default()
                };
                let Some(at) = c.points.checked_sub(1) else { return k };
                let pts = &self.points[at as usize];
                points.push(solver_points(pts.count, if self.warm == Warm::None { 0 } else { pts.solved }, &pts.cp));
                k.with_points(points.len() - 1)
            })
            .collect();
        let solve_gather = Instant::now();
        solve.solve((&mut bodies, &mut spinning), &mut constraints, &mut points, DT);
        let solved = Instant::now();

        for (s, &i) in self.moving.iter().enumerate() {
            let (b, body) = (bodies[s], self.body[i as usize]);
            if body.kind == STATIC {
                continue;
            }
            self.vel[i as usize] = b.v;
            let step = if body.kind == KINEMATIC { b.v * DT } else { b.displacement(DT) };
            self.pos[i as usize] = Vec2::new(self.pos[i as usize].x + step.x, self.pos[i as usize].y + step.y);
        }
        for b in &spinning {
            let i = self.moving[b.body as usize] as usize;
            if let (Some(q), Some(w)) = (self.rot[i], self.spin[i])
                && let Some(to) = b.turned_from(q, w)
                && self.body[i].kind != STATIC
            {
                (self.rot[i], self.spin[i]) = (Some(to), Some(b.w));
            }
        }
        let mut each = points.iter();
        for (c, k) in self.contacts.iter_mut().zip(&constraints) {
            (c.jn, c.jt) = (k.jn, k.jt);
            c.pressed = k.jn > 0.0 || c.depth >= 0.0;
            // As the mod writes `ContactPoints`: each point's only if it was
            // solved at its points.
            let Some(at) = c.points.checked_sub(1) else { continue };
            let (pts, p) = (&mut self.points[at as usize], each.next().expect("a contact's points solved"));
            pts.solved = if p.solved { p.count } else { 0 };
            if p.solved {
                let (q, cp) = (&p.point, &mut pts.cp);
                (cp.normals, cp.tangents, cp.solved_ids) = ([q[0].jn, q[1].jn], [q[0].jt, q[1].jt], cp.ids);
            }
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

/// The narrowphase's points, as the world's `ContactPoints` has them.
fn points_of(g: &narrow::Turned) -> ContactPoints {
    let mut cp = ContactPoints::default();
    for (i, p) in g.points[..g.count as usize].iter().enumerate() {
        cp.anchors[4 * i..4 * i + 4].copy_from_slice(&[p.ra.x, p.ra.y, p.rb.x, p.rb.y]);
        cp.separations[i] = p.separation;
        cp.ids[i] = p.id;
    }
    cp
}

/// A contact's points for the solver, each warm-started from the last
/// solve's impulse at the same feature: what the mod's `solve` gathers.
fn solver_points(count: u8, solved: u8, cp: &ContactPoints) -> Points {
    let mut out = Points { count, ..Points::default() };
    for (i, p) in out.point.iter_mut().enumerate().take(count as usize) {
        let (ra, rb) = cp.anchors(i);
        let (jn, jt) = cp.last(solved, cp.ids[i]);
        *p = ContactPoint { ra, rb, separation: cp.separations[i], jn, jt };
    }
    out
}

/// Last step's impulses carried to this step's points by where they are,
/// not by feature: each new point takes the impulse of the old point
/// nearest it on `a`, within a tenth of a unit, as parry's
/// `match_contacts_using_positions` does. The comparison's alternative to
/// feature ids.
fn nearest(new: &mut Pts, was: &Pts, ids_first: bool) {
    let (mut normals, mut tangents) = ([0.0; 2], [0.0; 2]);
    for i in 0..new.count as usize {
        let same = (0..was.solved as usize).find(|&k| was.cp.solved_ids[k] == new.cp.ids[i]).filter(|_| ids_first);
        let found = same.map(|k| (k, 0.0)).or_else(|| {
            (0..was.solved as usize)
                .map(|k| (k, (was.cp.anchors(k).0 - new.cp.anchors(i).0).len()))
                .filter(|&(_, d)| d < 0.1)
                .min_by(|a, b| a.1.total_cmp(&b.1))
        });
        (normals[i], tangents[i]) = found.map_or((0.0, 0.0), |(k, _)| (was.cp.normals[k], was.cp.tangents[k]));
    }
    new.solved = new.count;
    (new.cp.normals, new.cp.tangents, new.cp.solved_ids) = (normals, tangents, new.cp.ids);
}
