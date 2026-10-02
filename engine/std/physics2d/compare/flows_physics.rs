//! SPIKE, not engine code: physics's solve as a pipeline of flows
//! (docs/architecture/flows-spike.md), on the engine's own world between
//! frames, beside the solve system as built. Shared by the bench
//! (`flows_spike.rs`) and its test (`flows_spike_test.rs`).
//!
//! The pipeline, each line a harness system and each flow an edge:
//!
//! ```text
//! gather_bodies    world -> Make<Bodies>
//! gather_turning   See<Bodies>, world -> Make<Turning>
//! gather_contacts  See<Bodies>, world -> Make<Contacts>
//! prepare          See<Bodies>, See<Turning>, Pass<Contacts> -> Make<Graph>
//! solve            See<Turning>, Pass<Graph>
//! finish           Take<Graph> -> Pass<Bodies>, Pass<Turning>, Pass<Contacts>
//! scatter_bodies   Take<Bodies>, Take<Turning> -> world
//! scatter_contacts Take<Contacts> -> world
//! ```
//!
//! The reference is `solve` as built (lib.rs) in one system: the same
//! gathers into fresh vectors, `solver::solve_across`, the same scatters.
//! The sources and sinks are lib.rs's code, split; what differs is the
//! flows between them and the solve's stages on `flows::Colored::passes`
//! (`flows_lanes.rs`). Sleeping, `Touching` and the `Contact` events are
//! left out of both: neither changes a solved value.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Dt, Entity, Query, Without, Workers, World};
use flows::{Make, Pass, See, Take, flow};
use physics2d::{
    Asleep, Body, Collider, ContactPair, ContactPoints, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Position, Response, Resting,
    Rotation, STATIC, Spin, Tuning, Vec2, Velocity,
};

use crate::solver::lanes::Prepared;
pub use crate::solver::lanes::Shape;
use crate::solver::{self, Constraint, ContactPoint, Points, SolverBody, Spinning};

/// `Slots` as lib.rs has it, refilled in place: entities to places, by
/// entity index.
#[derive(Clone, Default)]
pub struct Slots(Vec<(u32, u32)>);

impl Slots {
    pub fn fill(&mut self, entities: &[Entity]) {
        let len = entities.iter().map(|e| e.index as usize + 1).max().unwrap_or(0);
        self.0.clear();
        self.0.resize(len, (u32::MAX, u32::MAX));
        for (k, e) in entities.iter().enumerate() {
            self.0[e.index as usize] = (e.generation, k as u32);
        }
    }

    #[inline]
    pub fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }
}

impl flows::Recycle for Slots {
    fn recycle(&mut self) {
        self.0.clear();
    }
}

flow! {
    /// The awake bodies, dense, by an index the step makes; the last stands
    /// for every body that doesn't move.
    pub struct Bodies: "physics2d::flow::Bodies" {
        pub entities: Vec<Entity>,
        pub bodies: Vec<SolverBody>,
        /// Per body: whether it moves, and its kind.
        pub kinds: Vec<(bool, u8)>,
        pub slots: Slots,
    }
}

flow! {
    /// The bodies that turn, beside them.
    pub struct Turning: "physics2d::flow::Turning" {
        pub spinning: Vec<Spinning>,
        pub reach: Vec<f32>,
    }
}

flow! {
    /// The contacts in pair order, their ends joined to the bodies' index.
    pub struct Contacts: "physics2d::flow::Contacts" {
        pub constraints: Vec<Constraint>,
        pub points: Vec<Points>,
        /// With threads: by the first contact row of each chunk, its first
        /// constraint, for writing back over the same chunks.
        pub firsts: Vec<(usize, usize)>,
    }
}

flow! {
    /// The contact graph as the passes solve it: colored, in lanes, the
    /// bodies as shared states.
    pub struct Graph: "physics2d::flow::Graph" {
        pub prepared: Prepared<4>,
        pub params: Option<solver::Params>,
    }
}

pub type MovingQ<'w, 'a> = Query<'w, (&'a Body, &'a mut Velocity, &'a mut Position), Without<Asleep>>;
pub type TurningQ<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a mut Rotation, &'a mut Spin), Without<Asleep>>;
pub type ContactsQ<'w, 'a> =
    Query<'w, (&'a ContactPair, &'a mut Manifold, &'a Response, &'a mut Impulse, &'a mut ContactPoints), Without<Resting>>;
pub type MovingR<'w, 'a> = Query<'w, (&'a Body, &'a Velocity, &'a Position), Without<Asleep>>;
pub type TurningR<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a Rotation, &'a Spin), Without<Asleep>>;
pub type ContactsR<'w, 'a> = Query<'w, (&'a ContactPair, &'a Manifold, &'a Response, &'a Impulse, &'a ContactPoints), Without<Resting>>;

// ---- The stages' bodies: lib.rs's `solve`, split ----

macro_rules! gather_bodies {
    ($dt:expr, $g:expr, $moving:expr, $workers:expr, $out:expr) => {{
        let (dt, g, moving, workers, out): (f32, Vec2, _, &Workers, &mut Bodies) = ($dt, $g, $moving, $workers, $out);
        let Bodies { entities, bodies, kinds, slots } = out;
        let solver_body = |body: &Body, v: &Velocity| {
            let (inv_mass, g) = if body.kind == DYNAMIC { (body.inv_mass, g) } else { (0.0, Vec2::ZERO) };
            let gravity = Vec2::new(g.x * body.gravity_scale * dt, g.y * body.gravity_scale * dt);
            SolverBody::new(Vec2::new(v.x, v.y), inv_mass, gravity)
        };
        if workers.threads() > 1 {
            let room = |r: std::ops::Range<usize>| (Vec::with_capacity(r.len()), Vec::with_capacity(r.len()), Vec::with_capacity(r.len()));
            let parts = moving.par_for_each(workers, room, |(e, s, k), row, (body, v, _)| {
                e.push(row.entity());
                s.push(solver_body(body, &v));
                k.push((body.kind != STATIC, body.kind));
            });
            for (e, s, k) in parts {
                entities.extend(e);
                bodies.extend(s);
                kinds.extend(k);
            }
        } else {
            moving.for_each(|row, (body, v, _)| {
                entities.push(row.entity());
                bodies.push(solver_body(body, &v));
                kinds.push((body.kind != STATIC, body.kind));
            });
        }
        bodies.push(SolverBody::default());
        kinds.push((false, STATIC));
        slots.fill(entities);
    }};
}

macro_rules! gather_turning {
    ($turning:expr, $b:expr, $out:expr) => {{
        let (turning, b, out): (_, &Bodies, &mut Turning) = ($turning, $b, $out);
        let Turning { spinning, reach } = out;
        turning.for_each(|row, (body, c, _, spin)| {
            let Some(k) = b.slots.get(row.entity()) else { return };
            spinning.push(match body.kind {
                DYNAMIC => Spinning::new(k, spin.w, b.bodies[k as usize].inv_mass * c.inertia_per_mass()),
                KINEMATIC => Spinning::new(k, spin.w, 0.0),
                _ => return,
            });
            reach.push(c.reach());
        });
    }};
}

macro_rules! gather_contacts {
    ($contacts:expr, $b:expr, $workers:expr, $out:expr) => {{
        let (contacts, b, workers, out): (_, &Bodies, &Workers, &mut Contacts) = ($contacts, $b, $workers, $out);
        let still = b.entities.len() as u32;
        let index_of = |e: Entity| b.slots.get(e).unwrap_or(still);
        let constraint = |pair: &ContactPair, m: &Manifold, r: &Response, j: &Impulse, cp: &ContactPoints, points: &mut Vec<Points>| {
            let c = Constraint {
                a: index_of(pair.a),
                b: index_of(pair.b),
                normal: Vec2::new(m.nx, m.ny),
                depth: m.depth,
                friction: r.friction,
                restitution: r.restitution,
                jn: j.normal,
                jt: j.tangent,
                speed: 0.0,
                points: 0,
            };
            if m.points == 0 {
                return c;
            }
            let mut pts = Points { count: m.points, ..Points::default() };
            for (i, p) in pts.point.iter_mut().enumerate().take(m.points as usize) {
                let (ra, rb) = cp.anchors(i);
                let (jn, jt) = cp.last(m.solved, cp.ids[i]);
                *p = ContactPoint { ra, rb, separation: cp.separations[i], jn, jt };
            }
            points.push(pts);
            c.with_points(points.len() - 1)
        };
        let Contacts { constraints, points, firsts } = out;
        if workers.threads() > 1 {
            let parts = contacts.par_for_each_ordered_page(
                workers,
                |rows| (rows.start, Vec::with_capacity(rows.len()), Vec::new()),
                |(_, out, pts), _, (pair, m, r, j, cp)| {
                    out.extend((0..pair.len()).filter(|&i| !r[i].disabled).map(|i| constraint(&pair[i], &m[i], &r[i], &j[i], &cp[i], pts)));
                },
            );
            for (row, part, pts) in parts {
                firsts.push((row, constraints.len()));
                let offset = points.len() as u32;
                constraints.extend(part.into_iter().map(|c| if c.points > 0 { Constraint { points: c.points + offset, ..c } } else { c }));
                points.extend(pts);
            }
        } else {
            contacts.for_each_ordered_page(|page, (pair, m, r, j, cp)| {
                for i in page.rows() {
                    if !r[i].disabled {
                        constraints.push(constraint(&pair[i], &m[i], &r[i], &j[i], &cp[i], points));
                    }
                }
            });
        }
    }};
}

// The sources' bodies as macros, each made a function twice: over the
// queries the solve as built writes through (one system can't both read
// and write a column), and over read-only ones, which is what a source
// declares. A source over the writing queries would get an apply node that
// re-sorts the bodies' spatial tables after it, for nothing.

pub fn gather_bodies(dt: f32, g: Vec2, moving: &mut MovingQ, workers: &Workers, out: &mut Bodies) {
    gather_bodies!(dt, g, moving, workers, out)
}

pub fn gather_bodies_read(dt: f32, g: Vec2, moving: &mut MovingR, workers: &Workers, out: &mut Bodies) {
    gather_bodies!(dt, g, moving, workers, out)
}

pub fn gather_turning(turning: &mut TurningQ, b: &Bodies, out: &mut Turning) {
    gather_turning!(turning, b, out)
}

pub fn gather_turning_read(turning: &mut TurningR, b: &Bodies, out: &mut Turning) {
    gather_turning!(turning, b, out)
}

pub fn gather_contacts(contacts: &mut ContactsQ, b: &Bodies, workers: &Workers, out: &mut Contacts) {
    gather_contacts!(contacts, b, workers, out)
}

pub fn gather_contacts_read(contacts: &mut ContactsR, b: &Bodies, workers: &Workers, out: &mut Contacts) {
    gather_contacts!(contacts, b, workers, out)
}

pub fn scatter_bodies(dt: f32, moving: &mut MovingQ, turning: &mut TurningQ, b: &Bodies, t: &Turning, workers: &Workers) {
    let write = |body: &Body, s: &SolverBody, mut v: engine_ecs::Mut<'_, Velocity>, mut p: engine_ecs::Mut<'_, Position>| {
        (v.x, v.y) = (s.v.x, s.v.y);
        let step = if body.kind == KINEMATIC { s.v * dt } else { s.displacement(dt) };
        let to = (p.x + step.x, p.y + step.y);
        if (to.0.to_bits(), to.1.to_bits()) != (p.x.to_bits(), p.y.to_bits()) {
            (p.x, p.y) = to;
        }
    };
    if !t.spinning.is_empty() {
        let mut each = t.spinning.iter();
        turning.for_each(|row, (body, _, mut q, mut spin)| {
            let Some(k) = b.slots.get(row.entity()) else { return };
            if body.kind != DYNAMIC && body.kind != KINEMATIC {
                return;
            }
            let s = each.next().expect("a spinning body per one gathered");
            let Some(to) = s.turned_from(q.rot(), spin.w).filter(|_| b.kinds[k as usize].0) else { return };
            if spin.w.to_bits() != s.w.to_bits() {
                spin.w = s.w;
            }
            if (to.c.to_bits(), to.s.to_bits()) != (q.c.to_bits(), q.s.to_bits()) {
                *q = Rotation::of(to);
            }
        });
    }
    if workers.threads() > 1 {
        let (bodies, kinds) = (&b.bodies, &b.kinds);
        moving.par_for_each(
            workers,
            |rows| rows.start,
            |k, _, (body, v, p)| {
                if kinds[*k].0 {
                    write(body, &bodies[*k], v, p);
                }
                *k += 1;
            },
        );
    } else {
        let mut i = 0;
        moving.for_each(|_, (body, v, p)| {
            let (s, moves) = (b.bodies[i], b.kinds[i].0);
            i += 1;
            if moves {
                write(body, &s, v, p);
            }
        });
    }
}

/// Impulses, what was solved at points, and pressing: lib.rs's write-back
/// of the contacts, without the links, sides and events it also makes.
pub fn scatter_contacts(contacts: &mut ContactsQ, c: &Contacts, workers: &Workers) {
    let points = &c.points;
    let wrote = |k: &Constraint, m: &mut Manifold, j: &mut Impulse| {
        *j = Impulse { normal: k.jn, tangent: k.jt };
        m.solved = k.points.checked_sub(1).map(|at| &points[at as usize]).filter(|p| p.solved).map_or(0, |p| p.count);
        m.pressed = k.jn > 0.0 || m.depth >= 0.0;
    };
    let solved_at = |k: &Constraint, mut cp: engine_ecs::Mut<'_, ContactPoints>| {
        let p = &points[k.points as usize - 1];
        let q = &p.point;
        (cp.normals, cp.tangents, cp.solved_ids) = ([q[0].jn, q[1].jn], [q[0].jt, q[1].jt], cp.ids);
    };
    if workers.threads() > 1 {
        let (constraints, firsts) = (&c.constraints, &c.firsts);
        contacts.par_for_each_ordered_page(
            workers,
            |rows| firsts.iter().find(|f| f.0 == rows.start).expect("the chunks the gathering walked").1,
            |k, page, (_, mut m, r, mut j, mut cp)| {
                let (m, j) = (m.write_all(), j.write_all());
                for i in page.rows() {
                    if r[i].disabled {
                        (m[i].pressed, j[i]) = (false, Impulse::default());
                    } else {
                        let c = &constraints[*k];
                        wrote(c, &mut m[i], &mut j[i]);
                        if m[i].solved > 0 {
                            solved_at(c, cp.get_mut(i));
                        }
                        *k += 1;
                    }
                }
            },
        );
    } else {
        let mut solved = c.constraints.iter();
        contacts.for_each_ordered_page(|page, (_, mut m, r, mut j, mut cp)| {
            let (m, j) = (m.write_all(), j.write_all());
            for i in page.rows() {
                if r[i].disabled {
                    (m[i].pressed, j[i]) = (false, Impulse::default());
                    continue;
                }
                let k = solved.next().expect("a constraint per contact solved");
                wrote(k, &mut m[i], &mut j[i]);
                if m[i].solved > 0 {
                    solved_at(k, cp.get_mut(i));
                }
            }
        });
    }
}

// ---- Timing ----

/// µs of each stage of the last frame, by name.
static TIMES: Mutex<Vec<(&'static str, f64)>> = Mutex::new(Vec::new());

fn timed<R>(name: &'static str, f: impl FnOnce() -> R) -> R {
    let t = Instant::now();
    let out = f();
    let us = t.elapsed().as_secs_f64() * 1e6;
    TIMES.lock().unwrap().push((name, us));
    out
}

/// The last frame's stage times, cleared.
pub fn take_times() -> Vec<(&'static str, f64)> {
    std::mem::take(&mut *TIMES.lock().unwrap())
}

/// How the pipeline's `solve` reaches its kernels, for the next frames.
static SHAPE: Mutex<Shape> = Mutex::new(Shape::Batch);

pub fn set_shape(shape: Shape) {
    *SHAPE.lock().unwrap() = shape;
}

// ---- The systems ----

fn params_of(tuning: &mut Query<&Tuning>) -> solver::Params {
    solver::Params::of(&tuning.single(|_, t| *t).unwrap_or(Tuning::DEFAULT))
}

fn gravity_of(gravity: &mut Query<&Gravity>) -> Vec2 {
    gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default()
}

/// The pipeline, in plan order.
pub fn pipeline(world: &World) -> Schedule {
    let gather_b = |_: &mut Cx, (dt, workers): (Dt, Workers), mut gravity: Query<&Gravity>, mut moving: MovingR, mut out: Make<Bodies>| {
        timed("gather_bodies", || gather_bodies_read(*dt, gravity_of(&mut gravity), &mut moving, &workers, &mut out))
    };
    let gather_t = |_: &mut Cx, mut turning: TurningR, b: See<Bodies>, mut out: Make<Turning>| {
        timed("gather_turning", || gather_turning_read(&mut turning, &b, &mut out))
    };
    let gather_c = |_: &mut Cx, workers: Workers, mut contacts: ContactsR, b: See<Bodies>, mut out: Make<Contacts>| {
        timed("gather_contacts", || gather_contacts_read(&mut contacts, &b, &workers, &mut out))
    };
    let prepare = |_: &mut Cx,
                   (dt, workers): (Dt, Workers),
                   mut tuning: Query<&Tuning>,
                   (b, t): (See<Bodies>, See<Turning>),
                   mut c: Pass<Contacts>,
                   mut g: Make<Graph>| {
        timed("prepare", || {
            let params = params_of(&mut tuning);
            g.params = Some(params);
            let Contacts { constraints, points, .. } = &mut *c;
            solver::lanes::prepare_flow(&mut g.prepared, &params, (&b.bodies, &t.spinning), constraints, points, *dt, &workers);
        })
    };
    let solve = |_: &mut Cx, workers: Workers, t: See<Turning>, mut g: Pass<Graph>| {
        timed("passes", || {
            let params = g.params.expect("prepared");
            let shape = *SHAPE.lock().unwrap();
            solver::lanes::passes_flow(&mut g.prepared, &params, &t.spinning, &workers, shape);
        })
    };
    let finish = |_: &mut Cx, workers: Workers, mut b: Pass<Bodies>, mut t: Pass<Turning>, mut c: Pass<Contacts>, g: Take<Graph>| {
        timed("finish", || {
            let params = g.params.expect("prepared");
            let Contacts { constraints, points, .. } = &mut *c;
            solver::lanes::finish_flow(&g.prepared, &params, (&mut b.bodies, &mut t.spinning), constraints, points, &workers);
        })
    };
    let scatter_b =
        |_: &mut Cx, (dt, workers): (Dt, Workers), mut moving: MovingQ, mut turning: TurningQ, b: Take<Bodies>, t: Take<Turning>| {
            timed("scatter_bodies", || scatter_bodies(*dt, &mut moving, &mut turning, &b, &t, &workers))
        };
    let scatter_c = |_: &mut Cx, workers: Workers, mut contacts: ContactsQ, c: Take<Contacts>| {
        timed("scatter_contacts", || scatter_contacts(&mut contacts, &c, &workers))
    };
    let s = Schedule {
        systems: vec![
            gather_b.system(world, "gather_bodies"),
            gather_t.system(world, "gather_turning"),
            gather_c.system(world, "gather_contacts"),
            prepare.system(world, "prepare"),
            solve.system(world, "solve"),
            finish.system(world, "finish"),
            scatter_b.system(world, "scatter_bodies"),
            scatter_c.system(world, "scatter_contacts"),
        ],
    };
    let notes = flows::check(world, &s).expect("the pipeline's plan");
    assert!(notes.is_empty(), "{notes:?}");
    s
}

/// The pipeline with a debug view between `finish` and the sinks: a system
/// that only sees what the solve left, and records it for the comparison.
pub fn pipeline_seen(world: &World) -> Schedule {
    let mut s = pipeline(world);
    let view = |_: &mut Cx, b: See<Bodies>, t: See<Turning>, c: See<Contacts>| {
        *SEEN.lock().unwrap() = Some(solved(&b, &t, &c));
    };
    s.systems.insert(6, view.system(world, "debug_view"));
    assert!(flows::check(world, &s).expect("with a view").is_empty());
    s
}

/// The solve system as built, without sleeping, sides or events: one system,
/// fresh vectors, `solver::solve_across`.
pub fn reference(world: &World) -> Schedule {
    let whole = |_: &mut Cx,
                 (dt, workers): (Dt, Workers),
                 (mut gravity, mut tuning): (Query<&Gravity>, Query<&Tuning>),
                 mut moving: MovingQ,
                 mut turning: TurningQ,
                 mut contacts: ContactsQ| {
        let dt = *dt;
        let (mut b, mut t, mut c) = (Bodies::default(), Turning::default(), Contacts::default());
        timed("gather", || {
            gather_bodies(dt, gravity_of(&mut gravity), &mut moving, &workers, &mut b);
            gather_turning(&mut turning, &b, &mut t);
            gather_contacts(&mut contacts, &b, &workers, &mut c);
        });
        timed("solver", || {
            let params = params_of(&mut tuning);
            solver::solve_across(&params, (&mut b.bodies, &mut t.spinning), &mut c.constraints, &mut c.points, dt, &workers);
        });
        if RECORD.load(std::sync::atomic::Ordering::Relaxed) {
            *SEEN.lock().unwrap() = Some(solved(&b, &t, &c));
        }
        timed("write_back", || {
            scatter_bodies(dt, &mut moving, &mut turning, &b, &t, &workers);
            scatter_contacts(&mut contacts, &c, &workers);
        });
    };
    Schedule { systems: vec![whole.system(world, "solve as built")] }
}

/// The pipeline's stages called in one system, on buffers it keeps: the
/// same work as `pipeline` without the flows between systems, for what the
/// hand-off costs.
pub fn fused(world: &World) -> Schedule {
    #[derive(Default)]
    struct Kept {
        b: Bodies,
        t: Turning,
        c: Contacts,
        g: Graph,
    }
    let kept: Mutex<Kept> = Mutex::default();
    let whole = move |_: &mut Cx,
                      (dt, workers): (Dt, Workers),
                      (mut gravity, mut tuning): (Query<&Gravity>, Query<&Tuning>),
                      mut moving: MovingQ,
                      mut turning: TurningQ,
                      mut contacts: ContactsQ| {
        let dt = *dt;
        let mut k = kept.lock().unwrap();
        let start = Instant::now();
        let Kept { b, t, c, g } = &mut *k;
        flows::Flow::recycle(b);
        flows::Flow::recycle(t);
        flows::Flow::recycle(c);
        gather_bodies(dt, gravity_of(&mut gravity), &mut moving, &workers, b);
        gather_turning(&mut turning, b, t);
        gather_contacts(&mut contacts, b, &workers, c);
        let params = params_of(&mut tuning);
        solver::lanes::prepare_flow(&mut g.prepared, &params, (&b.bodies, &t.spinning), &mut c.constraints, &c.points, dt, &workers);
        solver::lanes::passes_flow(&mut g.prepared, &params, &t.spinning, &workers, Shape::Batch);
        solver::lanes::finish_flow(&g.prepared, &params, (&mut b.bodies, &mut t.spinning), &mut c.constraints, &mut c.points, &workers);
        scatter_bodies(dt, &mut moving, &mut turning, b, t, &workers);
        scatter_contacts(&mut contacts, c, &workers);
        TIMES.lock().unwrap().push(("fused", start.elapsed().as_secs_f64() * 1e6));
    };
    Schedule { systems: vec![whole.system(world, "fused")] }
}

/// The solver's input as the gathers leave it, on one thread.
pub fn gathered(world: &World) -> (Vec<SolverBody>, Vec<Spinning>, Vec<Constraint>, Vec<Points>) {
    let got: std::sync::Arc<Mutex<Option<(Bodies, Turning, Contacts)>>> = std::sync::Arc::default();
    let g = got.clone();
    let sys = move |_: &mut Cx, mut gravity: Query<&Gravity>, mut moving: MovingQ, mut turning: TurningQ, mut contacts: ContactsQ| {
        let one = Workers::default();
        let (mut b, mut t, mut c) = (Bodies::default(), Turning::default(), Contacts::default());
        gather_bodies(1.0 / 60.0, gravity_of(&mut gravity), &mut moving, &one, &mut b);
        gather_turning(&mut turning, &b, &mut t);
        gather_contacts(&mut contacts, &b, &one, &mut c);
        *g.lock().unwrap() = Some((b, t, c));
    };
    Schedule { systems: vec![sys.system(world, "gathered")] }.run_sequential(world);
    let (b, t, c) = got.lock().unwrap().take().expect("gathered");
    (b.bodies, t.spinning, c.constraints, c.points)
}

// ---- What two ways must agree on ----

/// The solved copy: each body's state by entity, and each contact's
/// results in pair order, as bits.
pub type Solved = (HashMap<Entity, Vec<u32>>, Vec<u32>);

static SEEN: Mutex<Option<Solved>> = Mutex::new(None);
/// Whether the reference records its solved copy: only when checked, so
/// its frames aren't timed with the recording.
static RECORD: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn take_seen() -> Option<Solved> {
    SEEN.lock().unwrap().take()
}

fn solved(b: &Bodies, t: &Turning, c: &Contacts) -> Solved {
    let mut out: HashMap<Entity, Vec<u32>> = HashMap::new();
    for (k, &e) in b.entities.iter().enumerate() {
        let s = &b.bodies[k];
        out.insert(e, [s.v.x, s.v.y, s.moved.x, s.moved.y, s.inv_mass].iter().map(|x| x.to_bits()).collect());
    }
    for sp in &t.spinning {
        let x = out.get_mut(&b.entities[sp.body as usize]).expect("a spinning body is a body");
        x.extend([sp.w, sp.inv_inertia, sp.turned.c, sp.turned.s, sp.angle].iter().map(|x| x.to_bits()));
    }
    let mut contacts = Vec::new();
    for k in &c.constraints {
        contacts.extend([k.jn, k.jt, k.speed].iter().map(|x| x.to_bits()));
    }
    for p in &c.points {
        for q in &p.point {
            contacts.extend([q.jn, q.jt].iter().map(|x| x.to_bits()));
        }
        contacts.push(p.solved as u32);
    }
    (out, contacts)
}

/// What the scatters write, by entity, as bits: velocities, positions,
/// rotations and spins, impulses, what was solved and pressed, and the
/// points' impulses.
pub type Written = HashMap<Entity, Vec<u32>>;

pub fn written(world: &World) -> Written {
    let mut out: Written = HashMap::new();
    let mut put = |e: Entity, bits: &[u32]| out.entry(e).or_default().extend_from_slice(bits);
    let f = |x: f32| x.to_bits();
    for (e, v) in world.values::<Velocity>().unwrap() {
        put(e, &[f(v.x), f(v.y)]);
    }
    for (e, p) in world.values::<Position>().unwrap() {
        put(e, &[f(p.x), f(p.y)]);
    }
    for (e, q) in world.values::<Rotation>().unwrap() {
        put(e, &[f(q.c), f(q.s)]);
    }
    for (e, s) in world.values::<Spin>().unwrap() {
        put(e, &[f(s.w)]);
    }
    for (e, j) in world.values::<Impulse>().unwrap() {
        put(e, &[f(j.normal), f(j.tangent)]);
    }
    for (e, m) in world.values::<Manifold>().unwrap() {
        put(e, &[m.solved as u32, m.pressed as u32]);
    }
    for (e, cp) in world.values::<ContactPoints>().unwrap() {
        put(
            e,
            &[f(cp.normals[0]), f(cp.normals[1]), f(cp.tangents[0]), f(cp.tangents[1]), cp.solved_ids[0] as u32, cp.solved_ids[1] as u32],
        );
    }
    out
}

/// What the scatters write, as it was, to put back between ways.
#[derive(Default)]
pub struct Saved {
    bodies: HashMap<Entity, (Velocity, Position, Option<(Rotation, Spin)>)>,
    contacts: HashMap<Entity, (Manifold, Impulse, ContactPoints)>,
}

/// Saves the world's bodies and contacts, and gives a schedule that puts
/// them back, writing only what differs, as bits.
pub fn saver(world: &World) -> (Schedule, Schedule, std::sync::Arc<Mutex<Saved>>) {
    let saved: std::sync::Arc<Mutex<Saved>> = std::sync::Arc::default();
    let s = saved.clone();
    let save = move |_: &mut Cx, mut moving: MovingQ, mut turning: TurningQ, mut contacts: ContactsQ| {
        let mut s = s.lock().unwrap();
        s.bodies.clear();
        s.contacts.clear();
        moving.for_each(|row, (_, v, p)| {
            s.bodies.insert(row.entity(), (*v, *p, None));
        });
        turning.for_each(|row, (_, _, q, spin)| {
            if let Some(x) = s.bodies.get_mut(&row.entity()) {
                x.2 = Some((*q, *spin));
            }
        });
        contacts.for_each(|row, (_, m, _, j, cp)| {
            s.contacts.insert(row.entity(), (*m, *j, *cp));
        });
    };
    let s = saved.clone();
    let restore = move |_: &mut Cx, mut moving: MovingQ, mut turning: TurningQ, mut contacts: ContactsQ| {
        let s = s.lock().unwrap();
        let bits = |v: &[f32]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        moving.for_each(|row, (_, mut v, mut p)| {
            let (v0, p0, _) = &s.bodies[&row.entity()];
            if bits(&[v.x, v.y]) != bits(&[v0.x, v0.y]) {
                *v = *v0;
            }
            if bits(&[p.x, p.y]) != bits(&[p0.x, p0.y]) {
                *p = *p0;
            }
        });
        turning.for_each(|row, (_, _, mut q, mut spin)| {
            let Some((q0, s0)) = s.bodies[&row.entity()].2 else { return };
            if bits(&[q.c, q.s]) != bits(&[q0.c, q0.s]) {
                *q = q0;
            }
            if spin.w.to_bits() != s0.w.to_bits() {
                *spin = s0;
            }
        });
        contacts.for_each(|row, (_, mut m, _, mut j, mut cp)| {
            let (m0, j0, cp0) = &s.contacts[&row.entity()];
            if (m.solved, m.pressed) != (m0.solved, m0.pressed) {
                *m = *m0;
            }
            if bits(&[j.normal, j.tangent]) != bits(&[j0.normal, j0.tangent]) {
                *j = *j0;
            }
            let pts = |c: &ContactPoints| (bits(&[c.normals[0], c.normals[1], c.tangents[0], c.tangents[1]]), c.solved_ids);
            if pts(&cp) != pts(cp0) {
                *cp = *cp0;
            }
        });
    };
    (Schedule { systems: vec![save.system(world, "save")] }, Schedule { systems: vec![restore.system(world, "restore")] }, saved)
}

/// The reference and the pipeline on the world as it stands, each from the
/// same state: their solved copies and what they wrote, which must agree
/// bit for bit. The world is left as it was. Returns how many bodies,
/// turning bodies, contacts and contacts with points were solved.
pub fn check_against_reference(world: &World, shapes: &[Shape]) -> [usize; 4] {
    let (bodies, spinning, constraints, points) = gathered(world);
    let (save, restore, _) = saver(world);
    save.run_sequential(world);
    let before = written(world);
    RECORD.store(true, std::sync::atomic::Ordering::Relaxed);
    reference(world).run_sequential(world);
    RECORD.store(false, std::sync::atomic::Ordering::Relaxed);
    let want_copy = take_seen().expect("the reference saw its copy");
    let want = written(world);
    restore.run_sequential(world);
    assert!(written(world) == before, "the world put back");
    let pipe = pipeline_seen(world);
    for &shape in shapes {
        set_shape(shape);
        pipe.run_sequential(world);
        let got_copy = take_seen().expect("the view saw the copy");
        assert!(got_copy.0 == want_copy.0, "{shape:?}: the pipeline solves the bodies otherwise than the solve as built");
        assert!(got_copy.1 == want_copy.1, "{shape:?}: the pipeline solves the contacts otherwise than the solve as built");
        assert!(written(world) == want, "{shape:?}: the pipeline writes the world otherwise than the solve as built");
        restore.run_sequential(world);
    }
    set_shape(Shape::Batch);
    take_times();
    [bodies.len(), spinning.len(), constraints.len(), points.len()]
}
