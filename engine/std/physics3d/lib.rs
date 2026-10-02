//! The 3D physics step, as a pipeline of systems in the `physics3d::step`
//! phase, at the simulation's fixed rate: gravity into velocities, contacts,
//! then the solve, which also moves and turns bodies, a pipeline of its own
//! handing flows along (`pipeline.rs`), as 2D's is. Experimental:
//! spheres and boxes that turn; speculative contacts of up to four points;
//! the 2D step's soft solver with angular terms; contacts as entities in an
//! ordered table, as in 2D. Positions are a 3D spatial key whose box is
//! each collider turned by its rotation, so the broadphase is `near_pairs`
//! and the solve's last apply node (`scatter_bodies`'s) re-sorts bodies
//! that moved or turned.
//!
//! Everything a step carries to the next is in the world (contacts, their
//! impulses and cached separating axes, the rotations and move a resting
//! pair's manifold is carried by until it is found again, the `Gravity`
//! and `Tuning` it reads) or in this mod's state (steps and timings), so a reload swaps the
//! step under a running simulation, as it does the 2D one.
//!
//! What it leaves out of the 2D step: layers and sensors, kinematic bodies,
//! sleeping, events, parallelism, and the tile-seam rule for boxes. Design
//! and measurements: physics.md, "Rotation in 3D".

mod gjk;
mod narrow;
mod pipeline;
mod solver;

use std::time::Instant;

use engine_api::{Cx, Despawns, Dt, Entity, Live, Mod, Proximity, Query, Spawner, Systems, With, export_mod, field_struct, phase};
use narrow::{Narrow, Solid};
use physics_common::Slots;
pub use physics3d::{
    Anchors, AngularVelocity, Body, BoxBox, Carry, Closing, Collider, ContactPair, Gravity, Impulse, Inertia, Integrate, MAX_POINTS,
    Manifold, Mat3, Position, Quat, Reduce, Rotation, Shape, Static, Tuning, Vec3, Velocity, Warm,
};

field_struct! {
    /// Nanoseconds in each system and stage, summed over steps: for the
    /// benchmarks, as 2D's physics keeps them.
    #[derive(Debug, Default, Copy)]
    struct Timings {
        gravity: u64,
        contacts: u64,
        solve: u64,
        /// Within `contacts`: colliders gathered, the broadphase, the
        /// narrowphase, and the merge with the world's.
        gather: u64,
        broadphase: u64,
        narrowphase: u64,
        merge: u64,
        /// Within `solve`, the pipeline's systems (`pipeline.rs`) summed:
        /// the settings, bodies and contacts gathered, the solver, and the
        /// results written back.
        solve_gather: u64,
        solver: u64,
        write_back: u64,
    }
}

field_struct! {
    /// What the last step found: pairs from the broadphase, contacts, and
    /// points; and of the points of contacts there the step before, how
    /// many there were, and how many found their last impulse.
    #[derive(Debug, Default, Copy)]
    struct Found {
        pairs: u64,
        contacts: u64,
        points: u64,
        kept: u64,
        matched: u64,
        /// Contacts carried from the last step without the narrowphase.
        recycled: u64,
    }
}

/// How far each collider's fat box reaches past its box grown by the
/// speculative margin, as 2D's `FAT`: in 3D a margin's candidates grow as
/// its volume, so the smallest tried was cheaper still (314 µs against 387
/// at 0.05, Box3D's cap, 10 000 boxes settled, 2026-09-27).
const FAT: f32 = 0.02;

/// What the broadphase finds, kept live between steps, as 2D's
/// `Contacts`: the tables of `Moving`, against those of `Statics`.
struct Contacts;

impl Proximity for Contacts {
    type Key = Position;
    type Active = (With<(Position, Rotation, Collider, Body)>, With<Velocity>);
    type Passive = (With<(Position, Rotation, Collider, Body)>, With<Static>);
    const GROW: f32 = narrow::MARGIN;
    const MARGIN: f32 = FAT;
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics3d {
        steps: u64,
        time: Timings,
        found: Found,
    }
}

fn nanos(from: Instant, to: Instant) -> u64 {
    (to - from).as_nanos() as u64
}

struct Item {
    entity: Entity,
    solid: Solid,
    q: Quat,
    /// The radius of a sphere around its collider.
    reach: f32,
    friction: f32,
    restitution: f32,
}

fn quat(q: [f32; 4]) -> Quat {
    Quat { v: Vec3::new(q[0], q[1], q[2]), w: q[3] }
}

fn array(q: Quat) -> [f32; 4] {
    [q.v.x, q.v.y, q.v.z, q.w]
}

/// Last step's manifold carried to this step's poses without finding it
/// again: Box3D's contact recycling (`b3CollideTask`, physics_world.c),
/// which it says eliminates jitter. Each point is the material points of
/// both bodies that were at it last step, moved rigidly with them; its
/// separation grows by how far those came apart along the normal (held,
/// as Box3D holds it), and the point is now halfway between them. The ids
/// stay, so warm starting finds every impulse, and a resting pair's points
/// can't flicker between features or between reductions to four, which
/// kept piles of planks rocking (physics.md, "Still at rest").
///
/// None, so the pair is found again, once it may have moved `tolerance`
/// since it was: Box3D bounds the move since it was found, which needs the
/// poses then; this sums a bound over each step since (its centre's move
/// in `a`'s frame, and the turn of either body or of one against the
/// other at the reach of the larger), which needs only last step's
/// rotations, and is never less.
fn recycle(m: &Manifold, a: &Item, b: &Item, tolerance: f32) -> Option<Manifold> {
    let (qa0, qb0) = (quat(m.qa), quat(m.qb));
    let (dqa, dqb) = (a.q.times(qa0.conj()), b.q.times(qb0.conj()));
    let (ca, cb) = (a.solid.at, b.solid.at);
    let offset = m.offset();
    // b's center in a's frame, then and now; and the turn between them.
    let (rel0, rel) = (qa0.conj().rotate(-offset), a.q.conj().rotate(cb - ca));
    let qr = qa0.conj().times(qb0).conj().times(a.q.conj().times(b.q));
    let reach = a.reach.max(b.reach);
    let turn = qr.v.len().max(dqa.v.len()).max(dqb.v.len());
    // |q.v| is the sine of half the turn: twice it bounds how far a point
    // at `reach` moves.
    let moved = m.moved + (rel - rel0).len() + 2.0 * turn * reach;
    if moved >= tolerance {
        return None;
    }
    let n = m.normal();
    let mut out = Manifold { ox: ca.x - cb.x, oy: ca.y - cb.y, oz: ca.z - cb.z, qa: array(a.q), qb: array(b.q), moved, ..*m };
    let mut near = false;
    for k in 0..m.count as usize {
        let (ra, depth) = m.point(k);
        let pa = ca + dqa.rotate(ra);
        let pb = cb + dqb.rotate(ra + offset);
        let depth = depth - (pb - pa).dot(n);
        near |= depth >= -narrow::MARGIN;
        let r = (pa + pb) * 0.5 - ca;
        out.points[4 * k..4 * k + 4].copy_from_slice(&[r.x, r.y, r.z, depth]);
    }
    near.then_some(out)
}

type Moving<'w, 'a> = Query<'w, (&'a Position, &'a Rotation, &'a Collider, &'a Body, &'a Velocity)>;
type Statics<'w, 'a> = Query<'w, (&'a Position, &'a Rotation, &'a Collider, &'a Body), With<Static>>;

/// This step's manifold as stored: anchors on `a`, from its center.
fn stored(m: &narrow::Manifold, a: &Item, b: &Item) -> Manifold {
    let mut points = [0.0; 4 * narrow::MAX_POINTS];
    let mut ids = [0; narrow::MAX_POINTS];
    for (k, p) in m.points().iter().enumerate() {
        let r = p.at - a.solid.at;
        points[4 * k..4 * k + 4].copy_from_slice(&[r.x, r.y, r.z, p.depth]);
        ids[k] = p.id;
    }
    let o = a.solid.at - b.solid.at;
    // Mixed as Box2D does: friction by geometric mean, restitution by the
    // larger.
    Manifold {
        qa: array(a.q),
        qb: array(b.q),
        moved: 0.0,
        nx: m.normal.x,
        ny: m.normal.y,
        nz: m.normal.z,
        ox: o.x,
        oy: o.y,
        oz: o.z,
        friction: (a.friction * b.friction).sqrt(),
        restitution: a.restitution.max(b.restitution),
        count: m.count as u32,
        points,
        ids,
        axis: m.axis,
        axis_sep: m.axis_sep,
    }
}

/// The last step's impulses carried to this step's points: each point's
/// from the last point it matches, and the contact's friction whole.
fn warm(prev: Option<(&Manifold, &Impulse)>, m: &Manifold, how: Warm, count: &mut u64) -> Impulse {
    let Some((old, j)) = prev else { return Impulse::default() };
    if how == Warm::Cold {
        return Impulse::default();
    }
    let mut out = Impulse { tx: j.tx, ty: j.ty, tz: j.tz, twist: j.twist, ..Default::default() };
    for k in 0..m.count as usize {
        let matched = (0..old.count as usize).find(|&o| match how {
            Warm::Ids => old.ids[o] == m.ids[k],
            _ => {
                let d = old.point(o).0 - m.point(k).0;
                d.dot(d) < 1e-4
            }
        });
        if let Some(o) = matched {
            out.normal[k] = j.normal[o];
            *count += 1;
        }
    }
    out
}

impl Physics3d {
    fn integrate_velocities(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        dt: Dt,
        mut gravity: Query<&Gravity>,
        mut bodies: Query<(&Body, &mut Velocity)>,
    ) {
        let start = Instant::now();
        self.steps += 1;
        let g = gravity.single(|_, g| g.vec()).unwrap_or_default() * *dt;
        bodies.for_each(|_, (body, mut v)| {
            if body.inv_mass > 0.0 {
                (v.x, v.y, v.z) = (v.x + g.x, v.y + g.y, v.z + g.z);
            }
        });
        self.time.gravity += nanos(start, Instant::now());
    }

    /// Finds this step's contacts, and brings the world's in line in one pass,
    /// both in pair order, as the 2D step's `find_contacts` does.
    fn find_contacts(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut tuning: Query<&Tuning>,
        (mut moving, mut statics, mut near): (Moving<'_, '_>, Statics<'_, '_>, Live<'_, Contacts>),
        mut contacts: Query<(&ContactPair, &mut Manifold, &mut Impulse), (), Despawns>,
        new_contacts: Spawner<(ContactPair, Manifold, Impulse)>,
    ) {
        let start = Instant::now();
        let how = tuning.single(|_, t| *t).unwrap_or_default();
        let narrow = Narrow::of(&how);
        let mut items = Vec::with_capacity(moving.len() + statics.len());
        let item = |entity, p: &Position, q: &Rotation, c: &Collider, b: &Body| Item {
            entity,
            solid: Solid { at: p.at(), rot: q.quat().matrix(), shape: c.of() },
            q: q.quat(),
            reach: c.reach(),
            friction: b.friction,
            restitution: b.restitution,
        };
        moving.for_each(|row, (p, q, c, b, _)| items.push(item(row.entity(), p, q, c, b)));
        statics.for_each(|row, (p, q, c, b)| items.push(item(row.entity(), p, q, c, b)));
        let slots = Slots::of(items.iter().map(|i| i.entity));
        // The last step's contacts, for their separating axes and impulses: in
        // pair order, as `near` is.
        let mut old = Vec::with_capacity(contacts.len());
        contacts.for_each_ordered(|_, (pair, m, j)| old.push((*pair, *m, *j)));
        let gathered = Instant::now();
        // Kept live between steps (`Contacts`), as 2D's are.
        let near = near.pairs();
        let paired = Instant::now();
        let key = |p: &ContactPair| (p.a, p.b);
        let mut found: Vec<(ContactPair, Manifold, Impulse)> = Vec::with_capacity(near.len());
        let mut o = 0;
        let (mut points, mut kept, mut matched, mut reused) = (0u64, 0u64, 0u64, 0u64);
        for &(a, b) in near {
            let (i, j) = (&items[slots.get(a).expect("gathered") as usize], &items[slots.get(b).expect("gathered") as usize]);
            let pair = ContactPair { a, b };
            while old.get(o).is_some_and(|x: &(ContactPair, Manifold, Impulse)| key(&x.0) < (a, b)) {
                o += 1;
            }
            let prev = old.get(o).filter(|x| x.0 == pair).map(|x| (&x.1, &x.2));
            let cache = prev.map_or((0, 0.0), |(m, _)| (m.axis, m.axis_sep));
            // Box pairs only: a sphere's one point has no features to
            // flicker between, is found for less than carrying it costs, and
            // carried it rolls away from where the sphere touches (a pile of
            // spheres kept ten times the energy).
            let boxes = matches!((i.solid.shape, j.solid.shape), (Shape::Box(_), Shape::Box(_)));
            let recycled = if how.recycle > 0.0 && boxes { prev.and_then(|(m, _)| recycle(m, i, j, how.recycle)) } else { None };
            if let Some(manifold) = recycled {
                points += manifold.count as u64;
                kept += manifold.count as u64;
                reused += 1;
                found.push((pair, manifold, warm(prev, &manifold, how.warm(), &mut matched)));
            } else if let Some(m) = narrow::collide(&i.solid, &j.solid, cache, narrow) {
                let manifold = stored(&m, i, j);
                points += m.count as u64;
                kept += if prev.is_some() { m.count as u64 } else { 0 };
                found.push((pair, manifold, warm(prev, &manifold, how.warm(), &mut matched)));
            }
        }
        let narrowed = Instant::now();
        let spawn = |f: (ContactPair, Manifold, Impulse)| {
            new_contacts.spawn(f);
        };
        let mut next = 0;
        contacts.for_each_ordered_page(|page, (pair, mut m, mut j)| {
            let (m, j) = (m.write_all(), j.write_all());
            for i in page.rows() {
                while found.get(next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                    spawn(found[next]);
                    next += 1;
                }
                match found.get(next) {
                    Some(f) if f.0 == pair[i] => {
                        (m[i], j[i]) = (f.1, f.2);
                        next += 1;
                    }
                    _ => page.row(i).despawn(),
                }
            }
        });
        found[next..].iter().copied().for_each(spawn);
        let end = Instant::now();
        let t = &mut self.time;
        t.gather += nanos(start, gathered);
        t.broadphase += nanos(gathered, paired);
        t.narrowphase += nanos(paired, narrowed);
        t.merge += nanos(narrowed, end);
        t.contacts += nanos(start, end);
        self.found = Found { pairs: near.len() as u64, contacts: found.len() as u64, points, kept, matched, recycled: reused };
    }
}

impl Mod for Physics3d {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        const STEP: &str = "physics3d::step";
        // At the rate of `simulate`, so the two make one group: a game's
        // rules, then the step, once per step (as 2D's `physics2d::step`).
        s.phase(STEP).after(phase::SIMULATE).before(phase::LATE).fixed_hz(phase::SIMULATE_HZ);
        s.add("integrate_velocities", Self::integrate_velocities).phase(STEP);
        s.add("find_contacts", Self::find_contacts).phase(STEP).after("physics3d::integrate_velocities");
        // The solve, a pipeline of systems handing flows along
        // (`pipeline.rs`), each after the one before; `solve` first, so a
        // pre-solve hook ordered before it runs before anything is gathered.
        s.add("solve", Self::solve).phase(STEP).after("physics3d::find_contacts");
        s.add("gather_bodies", Self::gather_bodies).phase(STEP).after("physics3d::solve");
        s.add("gather_contacts", Self::gather_contacts).phase(STEP).after("physics3d::gather_bodies");
        s.add("solver", Self::solver).phase(STEP).after("physics3d::gather_contacts");
        s.add("scatter_contacts", Self::scatter_contacts).phase(STEP).after("physics3d::solver");
        s.add("scatter_bodies", Self::scatter_bodies).phase(STEP).after("physics3d::scatter_contacts");
    }

    /// `stats`: steps run, contacts held, and time per system; `stages`:
    /// time per stage, and what the last step found; `reset_timings`.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let per = |ns: u64| ns as f64 / self.steps.max(1) as f64 / 1e3;
        let t = self.time;
        match message.trim() {
            "stats" => {
                let mut contacts = 0;
                cx.world().for_each::<&ContactPair>(|_, _| contacts += 1);
                Ok(format!(
                    "steps {} contacts {contacts} us/step gravity {:.1} contacts {:.1} solve {:.1}",
                    self.steps,
                    per(t.gravity),
                    per(t.contacts),
                    per(t.solve)
                ))
            }
            "stages" => {
                let f = self.found;
                let times = format!(
                    "gravity {:.1} gather {:.1} broadphase {:.1} narrowphase {:.1} merge {:.1} solve_gather {:.1} solver {:.1} write_back {:.1}",
                    per(t.gravity),
                    per(t.gather),
                    per(t.broadphase),
                    per(t.narrowphase),
                    per(t.merge),
                    per(t.solve_gather),
                    per(t.solver),
                    per(t.write_back)
                );
                Ok(times
                    + &format!(
                        " pairs {} contacts {} points {} kept {} matched {} recycled {}",
                        f.pairs, f.contacts, f.points, f.kept, f.matched, f.recycled
                    ))
            }
            "reset_timings" => {
                (self.time, self.steps) = (Timings::default(), 0);
                Ok("reset".into())
            }
            other => Err(format!("physics3d does not understand {other:?}; try stats, stages or reset_timings")),
        }
    }
}

export_mod!(Physics3d);
