//! The solve, as a pipeline of systems handing flows along
//! (docs/architecture/flows.md; physics.md, "The step"):
//!
//! ```text
//! solve            world -> Make<Settings>
//! gather_bodies    See<Settings>, world -> Make<Bodies>
//! gather_turning   See<Bodies>, world -> Make<Turning>
//! gather_contacts  See<Bodies>, world -> Make<Contacts>
//! prepare          See<Settings>, See<Bodies>, See<Turning>, Pass<Contacts> -> Make<Graph>
//! passes           See<Settings>, See<Turning>, Pass<Graph>, Passes
//! finish           See<Settings>, Take<Graph> -> Pass<Bodies>, Pass<Turning>, Pass<Contacts>
//! scatter_contacts See<Settings>, Pass<Contacts> -> world
//! scatter_bodies   See<Settings>, Take<Bodies>, Take<Turning>, Take<Contacts> -> world
//! ```
//!
//! The sources copy the awake bodies and the contacts out of the world;
//! `prepare`, `passes` and `finish` are the solver's (`solver::staged`), its
//! passes declared as a shape the scheduler runs (`Passes`); the sinks write
//! the results back, with the contacts' sides and events, and sleeping. A
//! step where nothing turns is solved one contact at a time, in pair order,
//! by `finish` (`solver::solve_with`), which no shape can split.
//!
//! The flows are this mod's alone, not its interface's: what they carry is
//! the solver's own layout, and a mod that saw them would be rebuilt for
//! every change to it. A debug view that wants the solve's state would get
//! a flow of its own, of plain values, in the interface.

use std::time::Instant;

use engine_api::{
    Adds, Colored, Coloring, Cx, Dt, Entity, EventWriter, Make, Pass, Passes, Query, Recycle, See, Shareable, Stage, States, Take, Without,
    flow,
};
use physics2d::{
    Asleep, Body, Collider, Contact, ContactPair, ContactPoints, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Position, Response,
    Resting, Rotation, STATIC, Sleep, Spin, Still, Touching, Tuning, Vec2, Velocity,
};

use crate::Turning as TurningQ;
use crate::sleep::Sleepers;
use crate::solver::staged::{Atom, Shared, Staged, State, Step};
use crate::solver::{self, Constraint, ContactPoint, Points, SolverBody, Spinning};
use crate::{Moving, Physics, Records, RestingContacts, SleepingBodies, Slots, Stills, mark, nanos, sleeping_by};

/// How many contacts the passes solve at once: SSE2's width (`Wide`).
const LANES: usize = 4;

flow! {
    /// The step's settings, read once: what the solver solves by, gravity,
    /// and sleeping's thresholds where it's on.
    pub(crate) struct Settings: "physics2d::flow::Settings" {
        params: Option<solver::Params>,
        gravity: Option<Vec2>,
        sleep: Option<Sleep>,
    }
}

impl Settings {
    fn params(&self) -> solver::Params {
        self.params.expect("made by `solve`")
    }
}

flow! {
    /// The awake bodies, dense, by an index the step makes; the last stands
    /// for every body that doesn't move (statics, and sleeping bodies).
    pub(crate) struct Bodies: "physics2d::flow::Bodies" {
        entities: Vec<Entity>,
        bodies: Vec<SolverBody>,
        /// Per body: whether it moves (isn't static), and its kind.
        kinds: Vec<(bool, u8)>,
        slots: Slots,
    }
}

flow! {
    /// The bodies that turn, beside them, and each one's reach (for how
    /// fast its edge moves, which sleeping goes by).
    pub(crate) struct Turning: "physics2d::flow::Turning" {
        spinning: Vec<Spinning>,
        reach: Vec<f32>,
    }
}

flow! {
    /// The contacts in pair order, their ends the bodies' indices; then the
    /// pairs that pressed, for sleeping.
    pub(crate) struct Contacts: "physics2d::flow::Contacts" {
        constraints: Vec<Constraint>,
        points: Vec<Points>,
        links: Vec<(Entity, Entity)>,
    }
}

flow! {
    /// The contacts as the passes solve them: colored, in lanes, the bodies
    /// as states.
    pub(crate) struct Graph: "physics2d::flow::Graph" {
        /// Whether the step is solved in lanes; if not, `finish` solves it
        /// whole.
        lanes: bool,
        staged: Staged<LANES>,
        colors: Colors,
        program: Vec<Stage<Step>>,
    }
}

/// The contacts' colors, and how the lanes lay them out.
#[derive(Default)]
struct Colors {
    coloring: Coloring,
    layout: Colored,
    place: Vec<Option<(u32, u32)>>,
    masks: Vec<u64>,
}

/// Each is overwritten whole by the step that uses it, so there is nothing
/// to empty: recycling them is keeping their allocations.
impl Recycle for Staged<LANES> {
    fn recycle(&mut self) {}
}

/// The lanes' states, shared between threads as relaxed atomics of their
/// bits (`lanes::Atom`), which on x86 are plain loads and stores.
impl Shareable for State {
    type Shared = Atom;

    fn share(&self) -> Atom {
        State::share(self)
    }

    fn load(shared: &Atom) -> State {
        State::load(shared)
    }

    fn store(shared: &Atom, value: State) {
        State::store(shared, value)
    }
}

impl Recycle for Colors {
    fn recycle(&mut self) {}
}

/// Awake bodies, read: the walk `scatter_bodies` writes back in, so the
/// same tables in the same order, but declared read-only, so the source has
/// no apply node to re-sort the bodies after it.
type Awake<'w, 'a> = Query<'w, (&'a Body, &'a Velocity, &'a Position), Without<Asleep>>;
/// Turning bodies, read: as `Awake`, `scatter_bodies`'s walk.
type Turns<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a Rotation, &'a Spin), Without<Asleep>>;
type ContactsRead<'w, 'a> = Query<'w, (&'a ContactPair, &'a Manifold, &'a Response, &'a Impulse, &'a ContactPoints), Without<Resting>>;
type ContactsWrite<'w, 'a> =
    Query<'w, (&'a ContactPair, &'a mut Manifold, &'a Response, &'a mut Impulse, &'a mut ContactPoints), Without<Resting>>;

impl Physics {
    /// Begins the solve with the step's settings. The pipeline's first
    /// system, so a pre-solve hook (`.before("physics2d::solve")`, physics.md,
    /// "The step") runs before anything is gathered.
    pub(crate) fn solve(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        (mut config, mut gravity, mut tuning): (Query<&Sleep>, Query<&Gravity>, Query<&Tuning>),
        mut out: Make<Settings>,
    ) {
        let start = Instant::now();
        out.params = Some(solver::Params::of(&tuning.single(|_, t| *t).unwrap_or(Tuning::DEFAULT)));
        out.gravity = Some(gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default());
        out.sleep = sleeping_by(&mut config);
        let t = nanos(start);
        (self.time.solve_gather, self.time.solve) = (self.time.solve_gather + t, self.time.solve + t);
    }

    pub(crate) fn gather_bodies(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        dt: Dt,
        s: See<Settings>,
        mut moving: Awake,
        mut out: Make<Bodies>,
    ) {
        let start = Instant::now();
        let (dt, g) = (*dt, s.gravity.expect("made by `solve`"));
        let Bodies { entities, bodies, kinds, slots } = &mut *out;
        entities.reserve(moving.len());
        bodies.reserve(moving.len() + 1);
        kinds.reserve(moving.len() + 1);
        moving.for_each(|row, (body, v, _)| {
            // The gravity `integrate_velocities` added, computed as `fall`
            // computed it, for the solver to spread over its substeps.
            let (inv_mass, g) = if body.kind == DYNAMIC { (body.inv_mass, g) } else { (0.0, Vec2::ZERO) };
            let gravity = Vec2::new(g.x * body.gravity_scale * dt, g.y * body.gravity_scale * dt);
            entities.push(row.entity());
            bodies.push(SolverBody::new(Vec2::new(v.x, v.y), inv_mass, gravity));
            kinds.push((body.kind != STATIC, body.kind));
        });
        // Bodies with no velocity (statics) and sleeping ones all stand for
        // one immovable body at the end.
        bodies.push(SolverBody::default());
        kinds.push((false, STATIC));
        slots.fill(entities.iter().copied());
        let t = nanos(start);
        let time = &mut self.time;
        (time.solve_bodies, time.solve_gather, time.solve) = (time.solve_bodies + t, time.solve_gather + t, time.solve + t);
    }

    pub(crate) fn gather_turning(&mut self, _: &mut Sleepers, _: &mut Cx, b: See<Bodies>, mut turning: Turns, mut out: Make<Turning>) {
        let start = Instant::now();
        let Turning { spinning, reach } = &mut *out;
        turning.for_each(|row, (body, c, _, spin)| {
            let Some(k) = b.slots.get(row.entity()) else { return };
            spinning.push(match body.kind {
                DYNAMIC => Spinning::new(k, spin.w, b.bodies[k as usize].inv_mass * c.inertia_per_mass()),
                KINEMATIC => Spinning::new(k, spin.w, 0.0),
                _ => return,
            });
            reach.push(c.reach());
        });
        let t = nanos(start);
        let time = &mut self.time;
        (time.solve_turning, time.solve_gather, time.solve) = (time.solve_turning + t, time.solve_gather + t, time.solve + t);
    }

    pub(crate) fn gather_contacts(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        b: See<Bodies>,
        mut contacts: ContactsRead,
        mut out: Make<Contacts>,
    ) {
        let start = Instant::now();
        let still = b.entities.len() as u32;
        let index_of = |e: Entity| b.slots.get(e).unwrap_or(still);
        // A contact with points has them in `points`, beside the
        // constraints (see `solver::Points`).
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
        let Contacts { constraints, points, .. } = &mut *out;
        constraints.reserve(contacts.len());
        // In pair order, which storage keeps: the solve doesn't depend on
        // when each contact began. By page, so a row costs no dispatch per
        // term (docs/lore on a query's row cost), and `ContactPoints` is
        // read only where a contact has points.
        contacts.for_each_ordered_page(|page, (pair, m, r, j, cp)| {
            for i in page.rows() {
                if !r[i].disabled {
                    constraints.push(constraint(&pair[i], &m[i], &r[i], &j[i], &cp[i], points));
                }
            }
        });
        let t = nanos(start);
        (self.time.solve_gather, self.time.solve) = (self.time.solve_gather + t, self.time.solve + t);
    }

    /// The bodies as states, the contacts colored and in their lanes.
    pub(crate) fn prepare(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        dt: Dt,
        (s, b, t): (See<Settings>, See<Bodies>, See<Turning>),
        mut c: Pass<Contacts>,
        mut g: Make<Graph>,
    ) {
        let start = Instant::now();
        let params = s.params();
        let Contacts { constraints, points, .. } = &mut *c;
        let Graph { lanes, staged, colors, .. } = &mut *g;
        *lanes = Staged::<LANES>::takes(&params, &t.spinning, points);
        if *lanes {
            let moves = staged.begin(&params, (&b.bodies, &t.spinning), *dt);
            // Box2D's rule (`b2AddContactToGraph`): an edge with an end that
            // doesn't move stays out of color 0. `group`'s, contact for
            // contact, so the colors are the arrays' (`lanes::solve`).
            let Colors { coloring, layout, place, masks } = colors;
            coloring.greedy(constraints.len(), |i| (constraints[i].a, constraints[i].b), moves, true, masks);
            *layout = coloring.pack(LANES, place);
            staged.fill(&params, (&b.bodies, t.spinning.len()), (constraints, points), (place, layout.items()), *dt);
        }
        self.solver_time(start, |time, t| time.prepare += t);
    }

    /// The substeps and restitution: every pass a stage over the colors'
    /// batches or the bodies' states, which the scheduler runs.
    pub(crate) fn passes(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        (s, t): (See<Settings>, See<Turning>),
        mut g: Pass<Graph>,
        passes: Passes,
    ) {
        let start = Instant::now();
        let params = s.params();
        let Graph { lanes, staged, colors, program } = &mut *g;
        if *lanes {
            program.clear();
            solver::staged::program(&params, staged.states(), t.spinning.len(), |k, each| {
                program.push(match each {
                    None => Stage::Items(k),
                    Some(n) => Stage::Each(k, n),
                })
            });
            let n = staged.states();
            let (items, states, kernels) = staged.split(&params, &t.spinning);
            // The kernels are generic over the lanes' view of the states
            // (`Bodies`), so each view is matched once a call, not once a
            // body (docs/architecture/flows.md, "On one thread").
            passes.run(
                &colors.layout,
                items,
                states,
                program,
                |k, block, s| match s {
                    States::Plain(s) => kernels.block(k, block, s),
                    States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
                },
                |k, r, s| match s {
                    States::Plain(s) => kernels.each(k, r, s, n),
                    States::Shared(s) => kernels.each(k, r, &mut Shared(s), n),
                },
            );
        }
        self.solver_time(start, |time, t| time.passes += t);
    }

    /// The step's impulses and states back into the bodies and contacts; or,
    /// where nothing is in lanes, the step solved whole.
    pub(crate) fn finish(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        (dt, s): (Dt, See<Settings>),
        g: Take<Graph>,
        (mut b, mut t, mut c): (Pass<Bodies>, Pass<Turning>, Pass<Contacts>),
    ) {
        let start = Instant::now();
        let params = s.params();
        let Contacts { constraints, points, .. } = &mut *c;
        let (bodies, spinning) = (&mut b.bodies[..], &mut t.spinning[..]);
        if g.lanes {
            g.staged.finish(&params, (bodies, spinning), (constraints, points));
        } else {
            solver::solve_with(&params, (bodies, spinning), constraints, points, *dt);
        }
        self.solver_time(start, |time, t| time.finish += t);
    }

    fn solver_time(&mut self, start: Instant, part: impl FnOnce(&mut crate::Timings, u64)) {
        let t = nanos(start);
        let time = &mut self.time;
        part(time, t);
        (time.solver, time.solve) = (time.solver + t, time.solve + t);
    }

    /// Each contact's impulses and whether it's pressed; the sides of the
    /// bodies that ask (`Touching`), and `Contact` for each that began; and
    /// the pairs that press, which sleeping links.
    pub(crate) fn scatter_contacts(
        &mut self,
        _: &mut Sleepers,
        _: &mut Cx,
        s: See<Settings>,
        mut c: Pass<Contacts>,
        // A sleeping body's is left as it fell asleep.
        mut touching: Query<&mut Touching, Without<Asleep>>,
        mut contacts: ContactsWrite,
        began: EventWriter<Contact>,
    ) {
        let start = Instant::now();
        let linked = s.sleep.is_some();
        // Most bodies don't ask (the pile's none), and a failed lookup per
        // end of every contact was half of writing back.
        let mut asking = Vec::new();
        touching.for_each(|row, mut t| {
            *t = Touching::default();
            asking.push(row.entity());
        });
        let asking = Slots::of(asking.iter().copied());
        let (mut marks, mut begun) = (Vec::new(), Vec::new());
        let Contacts { constraints, points, links } = &mut *c;
        let points = &*points;
        // A contact's results, from constraint `k`: the sides of bodies
        // that asked it touches, a link for sleeping, and whether it began.
        let mut wrote = |k: &Constraint, pair: &ContactPair, m: &mut Manifold, j: &mut Impulse| {
            *j = Impulse { normal: k.jn, tangent: k.jt };
            m.solved = k.points.checked_sub(1).map(|at| &points[at as usize]).filter(|p| p.solved).map_or(0, |p| p.count);
            m.pressed = k.jn > 0.0 || m.depth >= 0.0;
            if !m.pressed {
                return;
            }
            if linked {
                links.push((pair.a, pair.b));
            }
            let n = Vec2::new(m.nx, m.ny);
            for (e, n) in [(pair.a, n), (pair.b, -n)] {
                if asking.get(e).is_some() {
                    marks.push((e, n));
                }
            }
            if !m.was_pressed {
                begun.push(Contact { a: pair.a, b: pair.b, nx: m.nx, ny: m.ny, speed: k.speed });
            }
        };
        // Each point's impulses, by feature, for the next step's to start
        // from: only if it was solved at its points, and written only where
        // there are points, or were.
        let solved_at = |k: &Constraint, mut cp: engine_api::Mut<'_, ContactPoints>| {
            let p = &points[k.points as usize - 1];
            let q = &p.point;
            (cp.normals, cp.tangents, cp.solved_ids) = ([q[0].jn, q[1].jn], [q[0].jt, q[1].jt], cp.ids);
        };
        // Every contact's impulse and pressing are written, so pages are
        // stamped whole, as in the merge.
        let mut solved = constraints.iter();
        contacts.for_each_ordered_page(|page, (pair, mut m, r, mut j, mut cp)| {
            let (m, j) = (m.write_all(), j.write_all());
            for i in page.rows() {
                if r[i].disabled {
                    (m[i].pressed, j[i]) = (false, Impulse::default());
                    continue;
                }
                let k = solved.next().expect("a constraint per contact solved");
                wrote(k, &pair[i], &mut m[i], &mut j[i]);
                if m[i].solved > 0 {
                    solved_at(k, cp.get_mut(i));
                }
            }
        });
        // Touching is marked after the walk, which only ever sets sides.
        for (e, n) in marks {
            touching.with(e, |_, mut t| mark(&mut t, n));
        }
        begun.into_iter().for_each(|c| began.send(c));
        let t = nanos(start);
        (self.time.write_back, self.time.solve) = (self.time.write_back + t, self.time.solve + t);
    }

    /// Each body's new velocity and position, rotation and spin; then
    /// sleeping's bookkeeping. The pipeline's last system, so the tick it
    /// ends at (`slept`) is after every write of the step's.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn scatter_bodies(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        (dt, s): (Dt, See<Settings>),
        (b, t, c): (Take<Bodies>, Take<Turning>, Take<Contacts>),
        // Awake bodies only: a sleeping one is immovable, and in tables of
        // its own, so walks over bodies skip it by what they match.
        (mut moving, mut stills, mut turning): (Moving, Stills, TurningQ),
        mut contacts: Query<&ContactPair, Without<Resting>, Adds<Resting>>,
        (mut sleeping, mut resting, mut records): (SleepingBodies, RestingContacts, Records),
    ) {
        let start = Instant::now();
        let dt = *dt;
        let (bodies, kinds) = (&b.bodies, &b.kinds);
        // A body's new velocity and position, `Mut`s stamping only what's
        // written.
        let write = |body: &Body, b: &SolverBody, mut v: engine_api::Mut<'_, Velocity>, mut p: engine_api::Mut<'_, Position>| {
            (v.x, v.y) = (b.v.x, b.v.y);
            // A kinematic body goes where it's told: nothing pushes it.
            let step = if body.kind == KINEMATIC { b.v * dt } else { b.displacement(dt) };
            // Written only when it moves: a write marks the row for the
            // spatial re-sort to re-bound, and a pile at rest comes to rest
            // bit for bit (the pile's 10 000 do by step 3000), when the
            // re-sort then has nothing to do.
            let to = (p.x + step.x, p.y + step.y);
            if (to.0.to_bits(), to.1.to_bits()) != (p.x.to_bits(), p.y.to_bits()) {
                (p.x, p.y) = to;
            }
        };
        // Rotations and spins, written only when they changed, as positions
        // are: a write re-bounds the row.
        if !t.spinning.is_empty() {
            // The walk that gathered them, in the same order.
            let mut each = t.spinning.iter();
            turning.for_each(|row, (body, _, mut q, mut spin)| {
                let Some(k) = b.slots.get(row.entity()) else { return };
                if body.kind != DYNAMIC && body.kind != KINEMATIC {
                    return;
                }
                let s = each.next().expect("a spinning body per one gathered");
                debug_assert_eq!(s.body, k);
                let Some(to) = s.turned_from(q.rot(), spin.w).filter(|_| kinds[k as usize].0) else { return };
                if spin.w.to_bits() != s.w.to_bits() {
                    spin.w = s.w;
                }
                if (to.c.to_bits(), to.s.to_bits()) != (q.c.to_bits(), q.s.to_bits()) {
                    *q = Rotation::of(to);
                }
            });
        }
        // The walk that gathered them, in the same order: nothing between
        // the two moves a body's row.
        let mut i = 0;
        moving.for_each(|row, (body, v, p)| {
            debug_assert_eq!(row.entity(), b.entities[i], "the bodies walked as they were gathered");
            let (s, moves) = (bodies[i], kinds[i].0);
            i += 1;
            if moves {
                write(body, &s, v, p);
            }
        });
        if let Some(config) = s.sleep {
            let t_sleeping = Instant::now();
            self.fall_asleep(
                (sleep, &mut records),
                &config,
                dt,
                (&b.entities, bodies, kinds),
                (&t.spinning, &t.reach),
                &c.links,
                (&mut moving, &mut stills, &mut turning),
                &mut contacts,
            );
            self.move_woken(sleep, &mut sleeping, &mut records, &mut resting);
            self.time.sleeping += nanos(t_sleeping);
        } else {
            // Turned off, sleeping forgets how long bodies were still: turned
            // on again, they start from moving. (Kept, `since` would count
            // the steps it was off.) A walk of the sparse set, so of nothing
            // once they're gone; not behind `is_empty`, which counts the
            // query's table rows, and a query of a sparse component alone
            // has none (see docs/lore).
            stills.for_each(|row, _| row.remove::<Still>());
        }
        // After every write of the step's, the stopping of bodies that fell
        // asleep included.
        self.slept = sleeping.now();
        let t = nanos(start);
        (self.time.write_back, self.time.solve) = (self.time.write_back + t, self.time.solve + t);
    }
}
