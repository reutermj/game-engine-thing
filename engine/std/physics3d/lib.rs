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
//! sleeping, events, and the tile-seam rule for boxes. Design and
//! measurements: physics.md, "Rotation in 3D".

mod gjk;
mod narrow;
mod pipeline;
mod solver;

use std::time::Instant;

use engine_api::{Cx, Despawns, Dt, Entity, Live, Mod, ParMap, Proximity, Query, Spawner, Systems, With, export_mod, field_struct, phase};
use narrow::{Narrow, Solid};
use physics_common::{FAT, Slots};
pub use physics3d::{
    Anchors, AngularVelocity, Body, BoxBox, Carry, Closing, Collider, ContactPair, Gravity, Impulse, Inertia, Integrate, Lanes, MAX_POINTS,
    Manifold, Mat3, Order, Position, Quat, Reduce, Rotation, Shape, Static, Tuning, Vec3, Velocity, Warm,
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
        /// Within `solver`: its three systems.
        prepare: u64,
        passes: u64,
        finish: u64,
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
        /// How the solve laid the contacts out in lanes (`solver::Staged`):
        /// groups (levels or colors), the overflow's batches, all batches,
        /// and the most and fewest batches a group has; zeros where it
        /// solved them whole.
        groups: u64,
        overflow: u64,
        batches: u64,
        widest: u64,
        narrowest: u64,
    }
}

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

/// A contact as `find_contacts` finds it and the world holds it.
type Contact = (ContactPair, Manifold, Impulse);

/// One list in order, in the parts a parallel walk or map leaves it in,
/// read as one: joining them would copy every contact again (about 200
/// bytes each). Parts are never empty, so each part's last item says
/// where it ends.
struct Parts<T> {
    parts: Vec<Vec<T>>,
    /// Where each part starts in the whole.
    starts: Vec<usize>,
    len: usize,
}

/// A place in `Parts`, read forward an item at a time: the rest of its
/// part, and the parts after. Not a `Peekable` of chained and flattened
/// slice iterators: the narrowphase walking the last step's contacts
/// through one took 281-284 µs at 8 threads on 10 000 settled boxes,
/// against 252-258 through this, as against a whole `Vec` (step_bench,
/// 2026-10-03).
struct Cursor<'a, T> {
    at: &'a [T],
    rest: &'a [Vec<T>],
}

impl<'a, T> Cursor<'a, T> {
    fn peek(&self) -> Option<&'a T> {
        self.at.first()
    }

    /// The next item if `f` is true of it, as `Peekable::next_if`.
    fn next_if(&mut self, f: impl FnOnce(&T) -> bool) -> Option<&'a T> {
        let x = self.peek().filter(|x| f(x))?;
        self.at = &self.at[1..];
        if self.at.is_empty()
            && let Some((part, rest)) = self.rest.split_first()
        {
            (self.at, self.rest) = (part, rest);
        }
        Some(x)
    }
}

impl<'a, T> Iterator for Cursor<'a, T> {
    type Item = &'a T;
    fn next(&mut self) -> Option<&'a T> {
        self.next_if(|_| true)
    }
}

impl<T> Parts<T> {
    fn new(parts: impl IntoIterator<Item = Vec<T>>) -> Parts<T> {
        let parts: Vec<Vec<T>> = parts.into_iter().filter(|p| !p.is_empty()).collect();
        let starts = parts.iter().scan(0, |n, p| Some(std::mem::replace(n, *n + p.len()))).collect();
        let len = parts.iter().map(Vec::len).sum();
        Parts { parts, starts, len }
    }

    fn len(&self) -> usize {
        self.len
    }

    /// The index of the first item `before` is false of, as a slice's
    /// `partition_point`: the items it's true of all come first.
    fn partition_point(&self, before: impl Fn(&T) -> bool) -> usize {
        let p = self.parts.partition_point(|part| before(part.last().expect("parts are never empty")));
        self.parts.get(p).map_or(self.len, |part| self.starts[p] + part.partition_point(before))
    }

    /// The items from the `n`-th on.
    fn from(&self, n: usize) -> Cursor<'_, T> {
        let p = self.starts.partition_point(|&s| s <= n).saturating_sub(1);
        // Within its part, which no part is empty to end before, unless `n`
        // is past the last.
        match self.parts.get(p) {
            Some(part) => Cursor { at: &part[(n - self.starts[p]).min(part.len())..], rest: &self.parts[p + 1..] },
            None => Cursor { at: &[], rest: &[] },
        }
    }
}

/// What a chunk of the merge did, for `find_contacts` to make after:
/// where in what was found it started and got to, its place there, and
/// the spawns and despawns, in the order a walk makes them.
struct Merged<'a> {
    start: Option<usize>,
    next: usize,
    from: Option<Cursor<'a, Contact>>,
    made: Vec<Made<'a>>,
}

enum Made<'a> {
    Spawn(&'a Contact),
    Despawn(Entity),
}

/// A chunk of the narrowphase's pairs, with the list it fills and what it
/// counts: an item of `ParMap::for_each_mut`.
struct Tested {
    found: Vec<Contact>,
    counts: Counts,
    pairs: std::ops::Range<usize>,
}

/// What a chunk of the narrowphase counts for `Found`: sums, so the
/// chunks' add to one walk's whatever the cut.
#[derive(Default)]
struct Counts {
    points: u64,
    kept: u64,
    matched: u64,
    reused: u64,
}

impl Counts {
    fn plus(self, o: &Counts) -> Counts {
        Counts {
            points: self.points + o.points,
            kept: self.kept + o.kept,
            matched: self.matched + o.matched,
            reused: self.reused + o.reused,
        }
    }
}

/// The fewest pairs a chunk of the narrowphase has (unless there are
/// fewer): 2D's. 64 measured the same on the planks' 2900 pairs at 8
/// threads (52-53 µs either way, step_bench, 2026-10-03).
const PAIRS_A_CHUNK: usize = 256;

/// `0..n` in contiguous ranges of at least `min` (fewer, longer ones if
/// `n` is short), four a thread, one on one thread: 2D's (physics2d's
/// `chunks`), the narrowphase's chunks, each a block of
/// `ParMap::for_each_mut`. Results are kept in chunk order, so any cut
/// gives the same step.
fn chunks(n: usize, min: usize, threads: usize) -> Vec<std::ops::Range<usize>> {
    let k = if threads == 1 { 1 } else { (n / min).clamp(1, threads * 4) }.min(n.max(1));
    (0..k).map(|i| n * i / k..n * (i + 1) / k).collect()
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
    /// both in pair order, as the 2D step's `find_contacts` does; the
    /// gathers, the narrowphase and the merge across the scheduler's threads
    /// (`ParMap`), bit for bit one thread's (physics.md, "The 3D narrowphase
    /// across threads").
    fn find_contacts(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut tuning: Query<&Tuning>,
        (mut moving, mut statics, mut near, map): (Moving<'_, '_>, Statics<'_, '_>, Live<'_, Contacts>, ParMap),
        mut contacts: Query<(&ContactPair, &mut Manifold, &mut Impulse), (), Despawns>,
        new_contacts: Spawner<(ContactPair, Manifold, Impulse)>,
    ) {
        let start = Instant::now();
        let how = tuning.single(|_, t| *t).unwrap_or_default();
        let narrow = Narrow::of(&how);
        let item = |entity, p: &Position, q: &Rotation, c: &Collider, b: &Body| Item {
            entity,
            solid: Solid { at: p.at(), rot: q.quat().matrix(), shape: c.of() },
            q: q.quat(),
            reach: c.reach(),
            friction: b.friction,
            restitution: b.restitution,
        };
        // The moving colliders across threads, joined by a copy onto the
        // first chunk's list, made with room for all, since `items` is read
        // by slot everywhere. Every list a task fills is made here, on this
        // thread: memory a worker allocates is its thread's
        // (docs/lore/memory-a-task-allocates-is-its-threads.md). It gains
        // despite the copy, where 2D's gathers didn't (physics.md, "The 3D
        // narrowphase across threads").
        let total = moving.len() + statics.len();
        let mut first = true;
        let room_items = |r: std::ops::Range<usize>| Vec::with_capacity(if std::mem::take(&mut first) { total } else { r.len() });
        let mut parts =
            moving.par_for_each(&map, room_items, |out, row, (p, q, c, b, _)| out.push(item(row.entity(), p, q, c, b))).into_iter();
        let mut items = parts.next().unwrap_or_else(|| Vec::with_capacity(total));
        parts.for_each(|p| items.extend(p));
        statics.for_each(|row, (p, q, c, b)| items.push(item(row.entity(), p, q, c, b)));
        let slots = Slots::of(items.iter().map(|i| i.entity));
        // The last step's contacts, for their separating axes and impulses: in
        // pair order, as `near` is; in the parts the walk leaves, not joined.
        let room = |r: std::ops::Range<usize>| Vec::with_capacity(r.len());
        let old = Parts::new(contacts.par_for_each_ordered_page(&map, room, |out, page, (pair, m, j)| {
            out.extend(page.rows().map(|i| (pair[i], m[i], j[i])));
        }));
        let gathered = Instant::now();
        // Kept live between steps (`Contacts`), as 2D's are.
        let near = near.pairs();
        let paired = Instant::now();
        let key = |p: &ContactPair| (p.a, p.b);
        let (items, slots, old) = (&items, &slots, &old);
        // Each pair alone: what it reads is this step's colliders and its
        // own last contact (its cached axis, the manifold it recycles), so
        // where a chunk starts changes nothing it finds.
        let test = move |t: &mut Tested| {
            let Some(&first) = near.get(t.pairs.start) else { return };
            // The last step's contacts from this chunk's first pair on, where
            // a walk from the first chunk's would have reached: both are in
            // pair order.
            let mut old = old.from(old.partition_point(|x| key(&x.0) < first));
            let mut counts = Counts::default();
            for &(a, b) in &near[t.pairs.clone()] {
                let (i, j) = (&items[slots.get(a).expect("gathered") as usize], &items[slots.get(b).expect("gathered") as usize]);
                let pair = ContactPair { a, b };
                while old.next_if(|x| key(&x.0) < (a, b)).is_some() {}
                let prev = old.peek().filter(|x| x.0 == pair).map(|x| (&x.1, &x.2));
                let cache = prev.map_or((0, 0.0), |(m, _)| (m.axis, m.axis_sep));
                // Box pairs only: a sphere's one point has no features to
                // flicker between, is found for less than carrying it costs, and
                // carried it rolls away from where the sphere touches (a pile of
                // spheres kept ten times the energy).
                let boxes = matches!((i.solid.shape, j.solid.shape), (Shape::Box(_), Shape::Box(_)));
                let recycled = if how.recycle > 0.0 && boxes { prev.and_then(|(m, _)| recycle(m, i, j, how.recycle)) } else { None };
                let c = &mut counts;
                if let Some(manifold) = recycled {
                    c.points += manifold.count as u64;
                    c.kept += manifold.count as u64;
                    c.reused += 1;
                    t.found.push((pair, manifold, warm(prev, &manifold, how.warm(), &mut c.matched)));
                } else if let Some(m) = narrow::collide(&i.solid, &j.solid, cache, narrow) {
                    let manifold = stored(&m, i, j);
                    c.points += m.count as u64;
                    c.kept += if prev.is_some() { m.count as u64 } else { 0 };
                    t.found.push((pair, manifold, warm(prev, &manifold, how.warm(), &mut c.matched)));
                }
            }
            t.counts = counts;
        };
        // Room for a contact a pair, made here, as the gathers' lists.
        let mut tested: Vec<Tested> = chunks(near.len(), PAIRS_A_CHUNK, map.threads())
            .into_iter()
            .map(|pairs| Tested { found: Vec::with_capacity(pairs.len()), counts: Counts::default(), pairs })
            .collect();
        map.for_each_mut(&mut tested, 1, |_, t| test(t));
        let narrowed = Instant::now();
        let c = tested.iter().fold(Counts::default(), |s, t| s.plus(&t.counts));
        let found = Parts::new(tested.into_iter().map(|t| t.found));
        let found = &found;
        let spawn = |f: &Contact| {
            new_contacts.spawn(*f);
        };
        // 2D's merge across threads (physics2d's `find_contacts`): each chunk
        // of the world's contacts merges with those found from its first key
        // on, writing in place, and records the spawns and despawns it would
        // have made. Those are made here after, in order, those between two
        // chunks' keys first: the log, and the ids spawns reserve, a single
        // walk's.
        let made = |r: std::ops::Range<usize>| Merged { start: None, next: 0, from: None, made: Vec::with_capacity(r.len() / 4 + 8) };
        let merged = contacts.par_for_each_ordered_page(&map, made, |c, page, (pair, mut m, mut j)| {
            let (m, j) = (m.write_all(), j.write_all());
            let mut rows = page.rows().peekable();
            let Some(&first) = rows.peek() else { return };
            if c.from.is_none() {
                let n = found.partition_point(|f| key(&f.0) < key(&pair[first]));
                (c.start, c.next, c.from) = (Some(n), n, Some(found.from(n)));
            }
            let from = c.from.as_mut().expect("set above");
            for i in rows {
                while let Some(f) = from.next_if(|f| key(&f.0) < key(&pair[i])) {
                    c.made.push(Made::Spawn(f));
                    c.next += 1;
                }
                match from.next_if(|f| f.0 == pair[i]) {
                    Some(f) => {
                        (m[i], j[i]) = (f.1, f.2);
                        c.next += 1;
                    }
                    None => c.made.push(Made::Despawn(page.entity(i))),
                }
            }
        });
        let mut done = 0;
        for c in merged {
            let Some(start) = c.start else { continue };
            found.from(done).take(start - done).for_each(spawn);
            for made in c.made {
                match made {
                    Made::Spawn(f) => spawn(f),
                    Made::Despawn(e) => contacts.get(e).expect("a contact the walk had").despawn(),
                }
            }
            done = c.next;
        }
        found.from(done).for_each(spawn);
        let end = Instant::now();
        let count = found.len() as u64;
        let t = &mut self.time;
        t.gather += nanos(start, gathered);
        t.broadphase += nanos(gathered, paired);
        t.narrowphase += nanos(paired, narrowed);
        t.merge += nanos(narrowed, end);
        t.contacts += nanos(start, end);
        self.found = Found {
            pairs: near.len() as u64,
            contacts: count,
            points: c.points,
            kept: c.kept,
            matched: c.matched,
            recycled: c.reused,
            ..Found::default()
        };
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
        s.add("prepare", Self::prepare).phase(STEP).after("physics3d::gather_contacts");
        s.add("passes", Self::passes).phase(STEP).after("physics3d::prepare");
        s.add("finish", Self::finish).phase(STEP).after("physics3d::passes");
        s.add("scatter_contacts", Self::scatter_contacts).phase(STEP).after("physics3d::finish");
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
                    "gravity {:.1} gather {:.1} broadphase {:.1} narrowphase {:.1} merge {:.1} solve_gather {:.1} solver {:.1} write_back {:.1} prepare {:.1} passes {:.1} finish {:.1}",
                    per(t.gravity),
                    per(t.gather),
                    per(t.broadphase),
                    per(t.narrowphase),
                    per(t.merge),
                    per(t.solve_gather),
                    per(t.solver),
                    per(t.write_back),
                    per(t.prepare),
                    per(t.passes),
                    per(t.finish)
                );
                Ok(times
                    + &format!(
                        " pairs {} contacts {} points {} kept {} matched {} recycled {} groups {} overflow {} batches {} widest {} narrowest {}",
                        f.pairs,
                        f.contacts,
                        f.points,
                        f.kept,
                        f.matched,
                        f.recycled,
                        f.groups,
                        f.overflow,
                        f.batches,
                        f.widest,
                        f.narrowest
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
