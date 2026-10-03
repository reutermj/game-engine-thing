//! The solve, as a pipeline of systems handing flows along
//! (docs/architecture/flows.md; physics.md, "A mod"), as 2D's is:
//!
//! ```text
//! solve            world -> Make<Settings>
//! gather_bodies    See<Settings>, world -> Make<Bodies>
//! gather_contacts  See<Bodies>, world -> Make<Contacts>
//! prepare          See<Settings>, Pass<Bodies>, Pass<Contacts> -> Make<Graph>
//! passes           See<Bodies>, Pass<Graph>, Passes
//! finish           See<Settings>, Take<Graph> -> Pass<Bodies>, Pass<Contacts>
//! scatter_contacts Take<Contacts> -> world
//! scatter_bodies   Take<Bodies> -> world
//! ```
//!
//! The sources copy the moving bodies and the contacts out of the world;
//! `prepare`, `passes` and `finish` are the solver's (`solver::Staged`), its
//! passes a program of stages declared as a shape the scheduler runs
//! (`Passes`), as 2D's; the sinks write the results back. The contacts are
//! grouped by `Tuning`'s order: in Box2D's colors (the default since
//! get-emj.90), the sweep over the colors' order, which threads can
//! share, or by level, the sweep in pair order bit for bit (physics.md,
//! "Colouring the 3D solve"). A step the lanes don't take (few contacts, a width but
//! four, `lanes=0`) is solved whole by `finish`, one contact at a time in
//! the same order or at its width (`solver::solve`).
//!
//! The flows are this mod's alone, not its interface's: what they carry is
//! the solver's own layout, and a mod that saw them would be rebuilt for
//! every change to it (as 2D's, flows.md, "Physics's adoption").

use std::time::Instant;

use engine_api::{Cx, Dt, Entity, Make, Pass, Passes, Query, Recycle, See, Stage, States, Take, flow};

use crate::solver::lanes::Shared;
use crate::solver::{self, Constraint, ContactPoint, SolverBody, Staged, Step};
use crate::{
    AngularVelocity, Body, ContactPair, Gravity, Impulse, Manifold, Physics3d, Position, Quat, Rotation, Slots, Tuning, Vec3, Velocity,
};

flow! {
    /// The step's settings, read once: what the solver solves by, and
    /// gravity.
    pub(crate) struct Settings: "physics3d::flow::Settings" {
        how: Option<solver::Tuning>,
        gravity: Option<Vec3>,
    }
}

flow! {
    /// The moving bodies, dense, in the order the world walks them; then
    /// one that stands for every static.
    pub(crate) struct Bodies: "physics3d::flow::Bodies" {
        entities: Vec<Entity>,
        bodies: Vec<SolverBody>,
        slots: Slots,
    }
}

flow! {
    /// The contacts in pair order, their ends the bodies' indices.
    pub(crate) struct Contacts: "physics3d::flow::Contacts" {
        constraints: Vec<Constraint>,
    }
}

/// How many contacts the passes solve at once: SSE2's width, `Tuning`'s
/// default (`Lanes::Four`). Other widths are variants, solved whole.
const LANES: usize = 4;

flow! {
    /// The contacts as the passes solve them: grouped, in lanes, the
    /// bodies as states.
    pub(crate) struct Graph: "physics3d::flow::Graph" {
        /// Whether the step is solved in lanes; if not, `finish` solves it
        /// whole.
        lanes: bool,
        staged: Staged<LANES>,
        program: Vec<Stage<Step>>,
    }
}

/// Overwritten whole by the step that uses it, so there is nothing to
/// empty: recycling it is keeping its allocations.
impl Recycle for Staged<LANES> {
    fn recycle(&mut self) {}
}

/// Moving bodies, read: the walk `scatter_bodies` writes back in, so the
/// same tables in the same order, but declared read-only, so the source
/// has no apply node to re-sort the bodies after it.
type Moving<'w, 'a> = Query<'w, (&'a Body, &'a Velocity, &'a AngularVelocity, &'a Position, &'a Rotation)>;
type MovingWrite<'w, 'a> = Query<'w, (&'a Body, &'a mut Velocity, &'a mut AngularVelocity, &'a mut Position, &'a mut Rotation)>;
/// As `Moving`: the contacts `scatter_contacts` writes back, in the same
/// order.
type ContactsRead<'w, 'a> = Query<'w, (&'a ContactPair, &'a Manifold, &'a Impulse)>;
type ContactsWrite<'w, 'a> = Query<'w, (&'a ContactPair, &'a Manifold, &'a mut Impulse)>;

impl Physics3d {
    /// Begins the solve with the step's settings. The pipeline's first
    /// system, so a pre-solve hook ordered `.before("physics3d::solve")`
    /// runs before anything is gathered, as in 2D.
    pub(crate) fn solve(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        (mut gravity, mut tuning): (Query<&Gravity>, Query<&Tuning>),
        mut out: Make<Settings>,
    ) {
        let start = Instant::now();
        out.how = Some(solver::Tuning::of(&tuning.single(|_, t| *t).unwrap_or_default()));
        out.gravity = Some(gravity.single(|_, g| g.vec()).unwrap_or_default());
        self.gathered(start);
    }

    pub(crate) fn gather_bodies(&mut self, _: &mut (), _: &mut Cx, dt: Dt, s: See<Settings>, mut moving: Moving, mut out: Make<Bodies>) {
        let start = Instant::now();
        let (dt, g) = (*dt, s.gravity.expect("made by `solve`"));
        let Bodies { entities, bodies, slots } = &mut *out;
        entities.reserve(moving.len());
        bodies.reserve(moving.len() + 1);
        moving.for_each(|row, (body, v, w, _, q)| {
            entities.push(row.entity());
            // The gravity `integrate_velocities` added, for the solver to
            // give back a substep at a time.
            let gravity = if body.inv_mass > 0.0 { g * dt } else { Vec3::ZERO };
            let (v, w) = (Vec3::new(v.x, v.y, v.z), Vec3::new(w.x, w.y, w.z));
            bodies.push(SolverBody::new(v, w, body.inv_mass, body.inv_inertia(), q.quat(), gravity));
        });
        // Statics all stand for one immovable body at the end.
        bodies.push(SolverBody::default());
        slots.fill(entities.iter().copied());
        self.gathered(start);
    }

    pub(crate) fn gather_contacts(&mut self, _: &mut (), _: &mut Cx, b: See<Bodies>, mut contacts: ContactsRead, mut out: Make<Contacts>) {
        let start = Instant::now();
        let still = b.entities.len() as u32;
        let index = |e: Entity| b.slots.get(e).unwrap_or(still);
        let constraints = &mut out.constraints;
        constraints.reserve(contacts.len());
        // In pair order, which storage keeps, and the solve's result
        // depends on.
        contacts.for_each_ordered(|_, (pair, m, j)| {
            let mut points = [ContactPoint::default(); crate::narrow::MAX_POINTS];
            for (k, p) in points[..m.count as usize].iter_mut().enumerate() {
                let (ra, depth) = m.point(k);
                *p = ContactPoint { ra, depth, jn: j.normal[k], speed: 0.0 };
            }
            constraints.push(Constraint {
                a: index(pair.a),
                b: index(pair.b),
                normal: m.normal(),
                offset: m.offset(),
                friction: m.friction,
                restitution: m.restitution,
                count: m.count as usize,
                points,
                jt: Vec3::new(j.tx, j.ty, j.tz),
                twist: j.twist,
            })
        });
        self.gathered(start);
    }

    /// The contacts grouped and into their lanes, the bodies as states,
    /// where the step goes in lanes four wide.
    pub(crate) fn prepare(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        (dt, s): (Dt, See<Settings>),
        (mut b, mut c): (Pass<Bodies>, Pass<Contacts>),
        mut g: Make<Graph>,
    ) {
        let start = Instant::now();
        let how = s.how.expect("made by `solve`");
        let Graph { lanes, staged, .. } = &mut *g;
        *lanes = how.lanes == LANES && staged.prepare(&mut b.bodies, &mut c.constraints, *dt, &how);
        let [groups, overflow, batches, widest, narrowest] = if *lanes { staged.layout() } else { [0; 5] }.map(|x| x as u64);
        let f = &mut self.found;
        (f.groups, f.overflow, f.batches, f.widest, f.narrowest) = (groups, overflow, batches, widest, narrowest);
        self.solver_time(start, |t, x| t.prepare += x);
    }

    /// The substeps and restitution: every pass a stage over the groups'
    /// batches or the bodies' states, which the scheduler runs.
    pub(crate) fn passes(&mut self, _: &mut (), _: &mut Cx, b: See<Bodies>, mut g: Pass<Graph>, passes: Passes) {
        let start = Instant::now();
        let Graph { lanes, staged, program } = &mut *g;
        if *lanes {
            staged.program(program);
            let (layout, mut items, states, kernels) = staged.split(&b.bodies);
            // The kernels are generic over the states' view (`Bodies`), so
            // each view is matched once a call, not once a body (flows.md,
            // "On one thread").
            passes.run(
                layout,
                &mut items,
                states,
                program,
                |k, _, block, s| match s {
                    States::Plain(s) => kernels.block(k, block, s),
                    States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
                },
                |k, r, s| match s {
                    States::Plain(s) => kernels.each(k, r, s),
                    States::Shared(s) => kernels.each(k, r, &mut Shared(s)),
                },
            );
        }
        self.solver_time(start, |t, x| t.passes += x);
    }

    /// The step's impulses and states back into the contacts and bodies;
    /// or, where nothing went in lanes, the step solved whole.
    pub(crate) fn finish(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        (dt, s): (Dt, See<Settings>),
        g: Take<Graph>,
        (mut b, mut c): (Pass<Bodies>, Pass<Contacts>),
    ) {
        let start = Instant::now();
        let how = s.how.expect("made by `solve`");
        let (bodies, contacts) = (&mut b.bodies[..], &mut c.constraints[..]);
        if g.lanes {
            g.staged.finish(bodies, contacts);
        } else if how.lanes == LANES {
            // `prepare` found the lanes don't pay, and changed nothing.
            solver::in_order(bodies, contacts, *dt, &how);
        } else {
            solver::solve(bodies, contacts, *dt, &how);
        }
        self.solver_time(start, |t, x| t.finish += x);
    }

    fn solver_time(&mut self, start: Instant, part: impl FnOnce(&mut crate::Timings, u64)) {
        let t = crate::nanos(start, Instant::now());
        part(&mut self.time, t);
        (self.time.solver, self.time.solve) = (self.time.solver + t, self.time.solve + t);
    }

    /// Each contact's impulses, for the next step to start from.
    pub(crate) fn scatter_contacts(&mut self, _: &mut (), _: &mut Cx, c: Take<Contacts>, mut contacts: ContactsWrite) {
        let start = Instant::now();
        // The walk that gathered them, in the same order.
        let mut solved = c.constraints.iter();
        contacts.for_each_ordered_page(|page, (_, _, mut j)| {
            let j = j.write_all();
            for i in page.rows() {
                let c = solved.next().expect("a constraint per contact gathered");
                let mut normal = [0.0; crate::narrow::MAX_POINTS];
                for (n, p) in normal.iter_mut().zip(&c.points[..c.count]) {
                    *n = p.jn;
                }
                j[i] = Impulse { normal, tx: c.jt.x, ty: c.jt.y, tz: c.jt.z, twist: c.twist };
            }
        });
        self.wrote(start);
    }

    /// Each body's new velocities, position and rotation. The pipeline's
    /// last system, so its apply node re-sorts the bodies that moved or
    /// turned.
    pub(crate) fn scatter_bodies(&mut self, _: &mut (), _: &mut Cx, b: Take<Bodies>, mut moving: MovingWrite) {
        let start = Instant::now();
        // The walk that gathered them, in the same order: nothing between
        // the two moves a body's row.
        let mut k = 0;
        moving.for_each(|row, (_, mut v, mut w, mut p, mut q)| {
            debug_assert_eq!(row.entity(), b.entities[k], "the bodies walked as they were gathered");
            let s = &b.bodies[k];
            k += 1;
            (v.x, v.y, v.z) = (s.v.x, s.v.y, s.v.z);
            (w.x, w.y, w.z) = (s.w.x, s.w.y, s.w.z);
            // Written only when it moves, as in 2D: a write re-bounds the row.
            let to = (p.x + s.moved.x, p.y + s.moved.y, p.z + s.moved.z);
            if (to.0.to_bits(), to.1.to_bits(), to.2.to_bits()) != (p.x.to_bits(), p.y.to_bits(), p.z.to_bits()) {
                (p.x, p.y, p.z) = to;
            }
            // A write re-bounds the row (the rotation is an extent), so only
            // a body that turned is written.
            if s.turned != Quat::IDENTITY {
                *q = Rotation::from(s.rotation());
            }
        });
        self.wrote(start);
    }

    fn gathered(&mut self, start: Instant) {
        let t = crate::nanos(start, Instant::now());
        (self.time.solve_gather, self.time.solve) = (self.time.solve_gather + t, self.time.solve + t);
    }

    fn wrote(&mut self, start: Instant) {
        let t = crate::nanos(start, Instant::now());
        (self.time.write_back, self.time.solve) = (self.time.write_back + t, self.time.solve + t);
    }
}
