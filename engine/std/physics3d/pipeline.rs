//! The solve, as a pipeline of systems handing flows along
//! (docs/architecture/flows.md; physics.md, "A mod"), as 2D's is:
//!
//! ```text
//! solve            world -> Make<Settings>
//! gather_bodies    See<Settings>, world -> Make<Bodies>
//! gather_contacts  See<Bodies>, world -> Make<Contacts>
//! solver           See<Settings>, Pass<Bodies>, Pass<Contacts>
//! scatter_contacts Take<Contacts> -> world
//! scatter_bodies   Take<Bodies> -> world
//! ```
//!
//! The sources copy the moving bodies and the contacts out of the world,
//! `solver` solves them (`solver::solve`), and the sinks write the results
//! back. The solve is one contact at a time in pair order, whose result
//! depends on that order, so it is one system and no shape: 2D's colors
//! and lanes, which a shape can split, would change 3D's results, and are
//! a physics change of their own (get-emj.52, get-emj.75).
//!
//! The flows are this mod's alone, not its interface's: what they carry is
//! the solver's own layout, and a mod that saw them would be rebuilt for
//! every change to it (as 2D's, flows.md, "Physics's adoption").

use std::time::Instant;

use engine_api::{Cx, Dt, Entity, Make, Pass, Query, Recycle, See, Take, flow};

use crate::solver::{self, Constraint, ContactPoint, SolverBody};
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

impl Recycle for Slots {
    fn recycle(&mut self) {
        self.0.clear();
    }
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

    /// The solve, whole, one contact at a time (`solver::solve`).
    pub(crate) fn solver(&mut self, _: &mut (), _: &mut Cx, (dt, s): (Dt, See<Settings>), (mut b, mut c): (Pass<Bodies>, Pass<Contacts>)) {
        let start = Instant::now();
        let how = s.how.expect("made by `solve`");
        solver::solve(&mut b.bodies, &mut c.constraints, *dt, &how);
        let t = crate::nanos(start, Instant::now());
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
