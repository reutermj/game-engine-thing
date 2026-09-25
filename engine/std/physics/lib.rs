//! The physics step, as a pipeline of systems in the `physics::step` phase:
//! gravity into velocities, contacts, then the solver (which also moves
//! bodies). Positions are kept in spatial order by the ECS, so the
//! broadphase is `near_pairs` and moving bodies re-sorts them at the
//! solver's apply node. Sleeping bodies and their resting contacts are in
//! tables of their own (`Asleep`, `Resting`), which the step's queries
//! exclude. See docs/architecture/physics.md.

//!
//! Everything a step carries to the next is in the world or in this mod's
//! state, so a reload swaps the solver under a running simulation.

mod narrow;
mod sleep;
mod solver;

use std::time::Instant;

use engine_api::{
    Adds, Cx, Despawns, Dt, Entity, EventWriter, Mod, OrderKey, Query, Removes, SpatialKey, Spawner, Systems, With, Without, Workers,
    export_mod, field_struct, phase,
};
use physics::{
    Asleep, Body, Collider, Contact, ContactPair, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Overlap, Placed, Position,
    Response, Resting, STATIC, Shape, Sleep, Touching, Trigger, Vec2, Velocity,
};
use sleep::Sleepers;
use solver::{Constraint, SolverBody};


#[cfg(not(feature = "v2"))]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

field_struct! {
    /// Nanoseconds spent in each system, summed: for the benchmark.
    #[derive(Debug, Default, Copy)]
    struct Timings {
        gravity: u64,
        contacts: u64,
        solve: u64,
        /// Within `contacts`: colliders gathered and sorted, the
        /// broadphase, the narrowphase, and the merge with the world's.
        gather: u64,
        broadphase: u64,
        /// Within `broadphase`: the pairs `near_pairs` found, before they're
        /// mapped to what was gathered.
        near: u64,
        narrowphase: u64,
        merge: u64,
        /// Within `solve`: bodies and contacts gathered, the solver, and
        /// the results written back.
        solve_gather: u64,
        solver: u64,
        write_back: u64,
    }
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics {
        steps: u64,
        /// The world's tick when `find_contacts` last looked for what games
        /// changed under sleeping bodies: what's written after it (by a
        /// game, a pre-solve hook, or a message) wakes them.
        since: u32,
        /// The world's tick after the last solve, whose writes (to bodies
        /// that fell asleep in it) are after `since` but physics's own.
        /// Both carried across reloads, so a new build doesn't take its own
        /// step's writes for a game's.
        slept: u32,
        time: Timings,
    }
}

/// Entities to positions in a list, by entity index: ids are small dense
/// integers, so a vector by index finds one in O(1), where sorting the list
/// and binary-searching it was most of gathering. The generation is kept,
/// so a stale id (despawned since, its index reused) finds nothing.
struct Slots(Vec<(u32, u32)>);

impl Slots {
    fn of(entities: impl Iterator<Item = Entity> + Clone) -> Slots {
        let len = entities.clone().map(|e| e.index as usize + 1).max().unwrap_or(0);
        let mut slots = vec![(u32::MAX, u32::MAX); len];
        for (k, e) in entities.enumerate() {
            slots[e.index as usize] = (e.generation, k as u32);
        }
        Slots(slots)
    }

    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }

    fn insert(&mut self, e: Entity, k: u32) {
        let i = e.index as usize;
        if self.0.len() <= i {
            self.0.resize(i + 1, (u32::MAX, u32::MAX));
        }
        self.0[i] = (e.generation, k);
    }
}

fn nanos(since: Instant) -> u64 {
    since.elapsed().as_nanos() as u64
}

/// A collider as the step sees it: only what pairs are tested with, since
/// gathering is writing every collider out, and with a whole `Collider`
/// and `Body` it took 64 µs at 10 000 against 44 (2026-09-24).
struct Item {
    entity: Entity,
    placed: Placed,
    collider: Layers,
    body: Material,
    v: Vec2,
}

/// A `Collider`'s layers and roles.
struct Layers {
    layer: u32,
    mask: u32,
    senses: u32,
    sensor: bool,
}

/// A `Body`'s part in a pair, and the collider's in the step.
struct Material {
    kind: u8,
    friction: f32,
    restitution: f32,
    /// The solver moves it: a body with a velocity, not static, awake.
    moves: bool,
    asleep: bool,
    /// On the broadphase's passive side (static or asleep): pairs of two
    /// passive colliders aren't looked for.
    passive: bool,
}

/// Sleeping bodies, as the systems that wake them see them: woken is
/// `Asleep` removed. Its terms are what a game writes to wake one, and
/// `Asleep` isn't one: physics writes it (an insert is a write) to every
/// body it puts to sleep, after the tick it watches from. The velocity is
/// for gravity on the ones woken (`fall_woken`); a walk stamps only the
/// pages it hands out, which a walk for what's written keeps to those
/// written.
type SleepingBodies<'w, 'a> = Query<'w, (&'a mut Velocity, &'a Position, &'a Collider, &'a Body), With<Asleep>, Removes<Asleep>>;
/// Each sleeping body's `Asleep`, for the ones a game wrote: put to sleep
/// by the game, not by physics.
type SleepMarks<'w, 'a> = Query<'w, &'a Asleep, With<(Velocity, Position, Collider, Body)>>;
/// Contacts kept as they are while their ends sleep: an end woken is
/// `Resting` removed.
type RestingContacts<'w, 'a> = Query<'w, &'a ContactPair, With<Resting>, Removes<Resting>>;
/// Resting contacts as `find_contacts` has them: one whose end is gone is
/// despawned there.
type RestingHere<'w, 'a> = Query<'w, &'a ContactPair, With<Resting>, (Removes<Resting>, Despawns)>;
/// Sleeping bodies' velocities, for gravity on the ones woken in a step
/// that has had it.
type SleepingVelocities<'w, 'a> = Query<'w, (&'a Body, &'a mut Velocity), With<Asleep>>;
/// Sleeping colliders, as the broadphase's passive side has them.
type AsleepColliders<'w, 'a> = Query<'w, (&'a Position, &'a Collider, &'a Body), With<Asleep>, Removes<Asleep>>;

impl Physics {
    fn integrate_velocities(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        (dt, workers): (Dt, Workers),
        mut gravity: Query<&Gravity>,
        mut bodies: Query<(&Body, &mut Velocity), Without<Asleep>>,
        (mut config, mut sleeping, mut marks, mut resting): (Query<&Sleep>, SleepingBodies<'_, '_>, SleepMarks<'_, '_>, RestingContacts<'_, '_>),
    ) {
        let start = Instant::now();
        let dt = *dt;
        self.steps += 1;
        let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
        if sleep.asleep > 0 || !marks.is_empty() {
            Self::wake_by_games(sleep, sleeping_by(&mut config).is_some(), (self.since, self.slept), &mut sleeping, &mut marks);
            // As `fall_woken`, through the query that has their velocities.
            for &e in &sleep.woken {
                sleeping.with(e, |_, (v, _, _, body)| fall(g, dt, body, v));
            }
            Self::move_woken(sleep, &mut sleeping, &mut resting);
        }
        if workers.threads() > 1 {
            bodies.par_for_each(&workers, |_| (), |_, _, (body, v)| fall(g, dt, body, v));
        } else {
            bodies.for_each(|_, (body, v)| fall(g, dt, body, v));
        }
        self.time.gravity += nanos(start);
    }

    /// Wakes what games changed under sleeping bodies since the last step
    /// (after tick `since`), all of it seen by the world's change detection,
    /// which at rest costs a look per page or table:
    /// - everything, if sleeping was turned off;
    /// - a body a game despawned, woke (removing its `Asleep`) or made
    ///   something else (removing its body): a row left the sleeping
    ///   tables (`left_since`), and a walk of them finds which;
    /// - a body a game wrote (its velocity, position, collider or body),
    ///   and its island (`for_each_written`).
    ///
    /// A body a game put to sleep (inserting `Asleep`, spawning it with one,
    /// or making an entity with one a body) is taken as it is, in the island
    /// it names.
    fn wake_by_games(
        sleep: &mut Sleepers,
        on: bool,
        (since, slept): (u32, u32),
        sleeping: &mut SleepingBodies<'_, '_>,
        marks: &mut SleepMarks<'_, '_>,
    ) {
        if !on {
            sleep.wake_all();
            sleeping.for_each(|row, _| sleep.woken.push(row.entity()));
            return;
        }
        // What a game put to sleep is taken before anything wakes: a row of
        // an island woken here isn't asleep to physics either, and would be
        // taken for new. New rows were given `Asleep` (not a term of
        // `sleeping`, so walked for only when rows arrived; physics's own
        // are asleep to it already), or are new to `sleeping` with their
        // values written: spawned asleep, or asleep before they were bodies.
        let mut new = Vec::new();
        if marks.arrived_since(since) {
            marks.for_each_written(since, |row, a| {
                if !sleep.is_asleep(row.entity()) {
                    sleep.adopt(row.entity(), a.island);
                    new.push(row.entity());
                }
            });
        }
        let new = Slots::of(new.into_iter());
        let mut written = Vec::new();
        sleeping.for_each_written(since, |row, _| written.push(row.entity()));
        written.retain(|&e| {
            if new.get(e).is_some() {
                return false;
            }
            if sleep.is_asleep(e) {
                // Put to sleep by the last solve, which wrote it after
                // `since`: only what's written after the solve is a game's.
                let fell = marks.written(e).is_some_and(|t| t > since);
                return !fell || sleeping.written(e).is_some_and(|t| t > slept);
            }
            if let Some(island) = marks.with(e, |_, a| a.island) {
                sleep.adopt(e, island);
            }
            false
        });
        // Physics's own wakes leave these tables too, so this also walks
        // them in the step after each: as rare as waking.
        if sleeping.left_since(since) {
            let mut alive = Vec::with_capacity(sleeping.len());
            sleeping.for_each(|row, _| alive.push(row.entity()));
            let slots = Slots::of(alive.into_iter());
            sleep.wake_missing(|e| slots.get(e).is_some());
        }
        for e in written {
            sleep.wake(e);
        }
    }

    /// Moves the bodies woken since this last ran out of their sleeping
    /// tables, and the resting contacts they're an end of back into the
    /// step. Rare, so walking every resting contact is fine. Generic over
    /// the queries, since each system that wakes bodies declares its own.
    fn move_woken<D: engine_api::engine_ecs::Data, F, C, G, H>(
        sleep: &mut Sleepers,
        sleeping: &mut Query<'_, D, F, C>,
        resting: &mut Query<'_, &ContactPair, G, H>,
    ) {
        if sleep.woken.is_empty() {
            return;
        }
        let woken = Slots::of(sleep.woken.iter().copied());
        for e in sleep.woken.drain(..) {
            if let Some(row) = sleeping.get(e) {
                row.remove::<Asleep>();
            }
        }
        resting.for_each(|row, pair| {
            if woken.get(pair.a).is_some() || woken.get(pair.b).is_some() {
                row.remove::<Resting>();
            }
        });
    }

    /// Finds this step's contacts and overlaps, and brings the world's in
    /// line with them in one pass each: both are in pair order, the
    /// world's by storage (ordered tables) and this step's by the
    /// broadphase. A contact that persists keeps its entity and impulses.
    #[allow(clippy::type_complexity)]
    fn find_contacts(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        // Every awake collider, by whether it has a body and a velocity:
        // queries have no optional terms, and one query per case gathers
        // each in one walk, where filling velocities in after was a second
        // walk and a lookup per collider.
        (mut moving, mut held, mut drifting, mut statics): (
            Query<(&Position, &Collider, &Body, &Velocity), Without<Asleep>>,
            Query<(&Position, &Collider, &Body), Without<(Velocity, Asleep)>>,
            Query<(&Position, &Collider, &Velocity), Without<(Body, Asleep)>>,
            Query<(&Position, &Collider), Without<(Body, Velocity)>>,
        ),
        // Sleeping ones, looked up only where an awake one meets them, and
        // the contacts they rest on: woken here, and out of their tables
        // before the solve.
        (mut asleep, mut resting): (AsleepColliders<'_, '_>, RestingHere<'_, '_>),
        (mut contacts, new_contacts): (
            Query<(&ContactPair, &mut Manifold, &mut Response), Without<Resting>, Despawns>,
            Spawner<(ContactPair, Manifold, Response, Impulse)>,
        ),
        (mut overlaps, new_overlaps): (Query<&Overlap, (), Despawns>, Spawner<(Overlap,)>),
        triggers: EventWriter<Trigger>,
        // Gravity, for bodies woken here: see `fall_woken`.
        (dt, mut gravity, mut falling): (Dt, Query<&Gravity>, SleepingVelocities<'_, '_>),
        workers: Workers,
    ) {
        let start = Instant::now();
        // Split across threads only while nothing sleeps: waking looks
        // sleeping bodies up as their pairs are found, one at a time.
        let par = workers.threads() > 1 && sleep.asleep == 0 && asleep.is_empty();
        let mut items = Vec::with_capacity(moving.len() + held.len() + drifting.len() + statics.len());
        let v = |v: &Velocity| Vec2::new(v.x, v.y);
        if par {
            // Each chunk's items, then joined in walk order: the one copy
            // the split costs over the walks below. Every list a task fills
            // is made here, on this thread, with room for all it will hold:
            // memory a worker allocates comes from its own arena (glibc),
            // which at a step's rate was fresh pages every time, and made
            // the gathers several times slower than one thread's.
            let room = |r: std::ops::Range<usize>| Vec::with_capacity(r.len());
            let join = |items: &mut Vec<Item>, parts: Vec<Vec<Item>>| parts.into_iter().for_each(|p| items.extend(p));
            let parts = moving.par_for_each(&workers, room, |out, row, (p, c, b, u)| out.push(item(row.entity(), p, c, *b, v(u), b.kind != STATIC, false)));
            join(&mut items, parts);
            let parts = held.par_for_each(&workers, room, |out, row, (p, c, b)| out.push(item(row.entity(), p, c, *b, Vec2::ZERO, false, false)));
            join(&mut items, parts);
            let parts = drifting.par_for_each(&workers, room, |out, row, (p, c, u)| out.push(item(row.entity(), p, c, Body::fixed(), v(u), false, false)));
            join(&mut items, parts);
            let parts = statics.par_for_each(&workers, room, |out, row, (p, c)| out.push(item(row.entity(), p, c, Body::fixed(), Vec2::ZERO, false, true)));
            join(&mut items, parts);
        } else {
            moving.for_each(|row, (p, c, b, u)| items.push(item(row.entity(), p, c, *b, v(u), b.kind != STATIC, false)));
            held.for_each(|row, (p, c, b)| items.push(item(row.entity(), p, c, *b, Vec2::ZERO, false, false)));
            drifting.for_each(|row, (p, c, u)| items.push(item(row.entity(), p, c, Body::fixed(), v(u), false, false)));
            statics.for_each(|row, (p, c)| items.push(item(row.entity(), p, c, Body::fixed(), Vec2::ZERO, false, true)));
        }
        let mut slots = Slots::of(items.iter().map(|i| i.entity));
        if sleep.asleep > 0 {
            self.wake_on_statics(sleep, &mut statics, &mut asleep, &mut resting, &slots);
        }
        // The storage's own order is the broadphase: pairs whose boxes, grown
        // by the speculative margin, meet, and one of which is awake and can
        // move. In entity order, lesser first, so what's found doesn't
        // depend on the order items were gathered in.
        let gathered = Instant::now();
        let near = engine_api::near_pairs_with(&workers, &(&moving, &held, &drifting), &(&statics, &asleep), narrow::MARGIN);
        let found_near = Instant::now();
        let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(near.len());
        if par {
            let slots = &slots;
            let at = |e: Entity| slots.get(e).expect("an awake collider, gathered");
            let parts = engine_api::engine_ecs::par::even(near.len(), workers.chunks(near.len(), 1024));
            let parts = parts.into_iter().map(|r| (Vec::with_capacity(r.len()), r)).collect();
            for part in workers.map_each(parts, |_, (mut out, r): (Vec<(u32, u32)>, _)| {
                out.extend(near[r].iter().map(|&(a, b)| (at(a), at(b))));
                out
            }) {
                pairs.extend(part);
            }
        } else {
            // A sleeping collider is gathered only when an awake one meets it.
            let mut fetch = |e: Entity, items: &mut Vec<Item>, slots: &mut Slots| {
                let k = items.len() as u32;
                let mut it = asleep.with(e, |_, (p, c, b)| item(e, p, c, *b, Vec2::ZERO, false, true)).expect("a collider has a position");
                it.body.asleep = true;
                items.push(it);
                slots.insert(e, k);
                k
            };
            for (a, b) in near {
                let (i, j) = (slots.get(a), slots.get(b));
                let i = i.unwrap_or_else(|| fetch(a, &mut items, &mut slots));
                let j = j.unwrap_or_else(|| fetch(b, &mut items, &mut slots));
                pairs.push((i, j));
            }
        }
        let paired = Instant::now();
        let index = |e: Entity| slots.get(e).expect("a paired collider");
        // Neither end moves, and one sleeps: their contact is `Resting`,
        // kept as it was, and not looked for again.
        let rests = |a: &Item, b: &Item| (a.body.asleep || b.body.asleep) && !a.body.moves && !b.body.moves;

        let mut found: Vec<Found> = Vec::new();
        // Each overlap, and whether it's a sensor's, which triggers.
        let mut overlapping: Vec<(Overlap, bool)> = Vec::new();
        let any_asleep = sleep.asleep > 0;
        let pair = |&(i, j): &(u32, u32)| (&items[i as usize], &items[j as usize]);
        // Split on whether anything sleeps, so a step with nothing asleep
        // tests nothing for it: the always-false tests cost about 7 µs of
        // 110 at 10 000 settled and at rest (2026-09-24, against the arrays'
        // narrowphase over six runs each), as a dead test once did in the
        // solve (physics.md, "What the ECS costs").
        if par {
            // Room for a contact per pair, made here (see the gathering).
            let parts = engine_api::engine_ecs::par::even(pairs.len(), workers.chunks(pairs.len(), 256));
            let parts = parts.into_iter().map(|r| (Vec::with_capacity(r.len()), Vec::new(), r)).collect();
            let tested = workers.map_each(parts, |_, (mut found, mut overlapping, r)| {
                pairs[r].iter().map(pair).for_each(|(a, b)| awake(a, b, &mut found, &mut overlapping));
                (found, overlapping)
            });
            found.reserve_exact(tested.iter().map(|(f, _)| f.len()).sum());
            for (f, o) in tested {
                found.extend(f);
                overlapping.extend(o);
            }
        } else if any_asleep {
            // What `awake` does, where a pair may rest, or wake what rests.
            let mut test = |a: &Item, b: &Item| {
                let (collide, sense) = (collides(a, b), senses(a, b));
                let rest = rests(a, b);
                if !sense && (!collide || rest) {
                    return;
                }
                let Some(m) = narrow::collide(&a.placed, &b.placed, b.v - a.v) else { return };
                let sensor = collide && (a.collider.sensor || b.collider.sensor);
                if (sensor || sense) && m.depth >= 0.0 {
                    overlapping.push((Overlap { a: a.entity, b: b.entity }, sensor));
                }
                // A pair that rested and now moves (a static a game made a body,
                // a shelf given a velocity) still has its `Resting` contact:
                // not a new one, but that one woken, back from the next step.
                let mut woke = || {
                    let key = ContactPair { a: a.entity, b: b.entity }.key();
                    let mut kept = false;
                    resting.in_keys::<ContactPair>(key..=key, |_, _| kept = true);
                    kept
                };
                if collide && !sensor && !rest && (a.body.asleep || b.body.asleep) && woke() {
                    sleep.wake(a.entity);
                    sleep.wake(b.entity);
                } else if collide && !sensor && !rest {
                    found.push(contact(a, b, &m));
                }
            };
            pairs.iter().map(pair).for_each(|(a, b)| test(a, b));
        } else {
            pairs.iter().map(pair).for_each(|(a, b)| awake(a, b, &mut found, &mut overlapping));
        }

        let narrowed = Instant::now();
        let key = |p: &ContactPair| (p.a, p.b);
        let spawn = |(pair, m, r): Found| {
            new_contacts.spawn((pair, m, r, Impulse::default()));
        };
        // Contacts pressed last step that ended: an end asleep has lost
        // what it rested on or against, and wakes.
        let mut ended = Vec::new();
        if par {
            // Each chunk of contacts merges with the contacts found from its
            // first key on, updating in place, and records the spawns and
            // despawns it would have made. Those are made here after, in
            // order, those between two chunks' keys first: the log, and the
            // ids spawns reserve, a single walk's.
            let found = &found;
            let made = |r: std::ops::Range<usize>| Merged { made: Vec::with_capacity(r.len() / 4 + 8), ..Merged::default() };
            let merged = contacts.par_for_each_ordered_page(&workers, made, |c, page, (pair, mut m, mut r)| {
                let (m, r) = (m.write_all(), r.write_all());
                let from = *c.start.get_or_insert_with(|| found.partition_point(|f| key(&f.0) < key(&pair[0])));
                c.next = c.next.max(from);
                for i in page.rows() {
                    while found.get(c.next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                        c.made.push(Made::Spawn(c.next));
                        c.next += 1;
                    }
                    match found.get(c.next) {
                        Some(f) if f.0 == pair[i] => {
                            m[i] = Manifold { was_pressed: m[i].pressed, ..f.1 };
                            r[i] = f.2;
                            c.next += 1;
                        }
                        _ => c.made.push(Made::Despawn(page.entity(i))),
                    }
                }
            });
            let mut done = 0;
            for c in merged {
                found[done..c.start.expect("a chunk has rows")].iter().copied().for_each(spawn);
                for made in c.made {
                    match made {
                        Made::Spawn(n) => spawn(found[n]),
                        Made::Despawn(e) => contacts.get(e).expect("a contact the walk had").despawn(),
                    }
                }
                done = c.next;
            }
            found[done..].iter().copied().for_each(spawn);
        } else {
            let mut next = 0;
            // Every contact is written or despawned, so each page is stamped
            // written as a whole, not row by row: 54 µs to 34 at 10 000
            // contacts, against `for_each_ordered` and `Mut` (2026-09-24).
            contacts.for_each_ordered_page(|page, (pair, mut m, mut r)| {
                let (m, r) = (m.write_all(), r.write_all());
                for i in page.rows() {
                    while found.get(next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                        spawn(found[next]);
                        next += 1;
                    }
                    match found.get(next) {
                        Some(f) if f.0 == pair[i] => {
                            m[i] = Manifold { was_pressed: m[i].pressed, ..f.1 };
                            r[i] = f.2;
                            next += 1;
                        }
                        _ => {
                            if sleep.asleep > 0 && m[i].pressed {
                                ended.push(pair[i]);
                            }
                            page.row(i).despawn();
                        }
                    }
                }
            });
            found[next..].iter().copied().for_each(spawn);
        }
        for pair in ended {
            sleep.wake(pair.a);
            sleep.wake(pair.b);
        }
        // What woke here moves at this system's apply node, so this step's
        // solve has it: movable, falling, and its resting contacts solved
        // again.
        if !sleep.woken.is_empty() {
            let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
            fall_woken(sleep, g, *dt, &mut falling);
        }
        Self::move_woken(sleep, &mut asleep, &mut resting);
        // Physics has looked at everything a game may have changed under
        // sleeping bodies; what's written after is for the next step's
        // looks, a pre-solve hook's (between here and the solve) included.
        self.since = asleep.now();

        let key = |o: &Overlap| (o.a, o.b);
        let mut next = 0;
        let begin = |(o, sensor): (Overlap, bool)| {
            new_overlaps.spawn((o,));
            if sensor {
                let (sensor, other) = if items[index(o.a) as usize].collider.sensor { (o.a, o.b) } else { (o.b, o.a) };
                triggers.send(Trigger { sensor, other });
            }
        };
        // An overlap of two passive colliders (a sleeping body in a static
        // sensor) isn't looked for, so it lasts while they stay so.
        let mut passive = |e: Entity| slots.get(e).map_or_else(|| asleep.with(e, |_, _| ()).is_some(), |k| items[k as usize].body.passive);
        overlaps.for_each_ordered(|row, o| {
            while overlapping.get(next).is_some_and(|f| key(&f.0) < key(o)) {
                begin(overlapping[next]);
                next += 1;
            }
            if overlapping.get(next).is_some_and(|f| f.0 == *o) {
                next += 1;
            } else if !(sleep.asleep > 0 && passive(o.a) && passive(o.b)) {
                row.despawn();
            }
        });
        overlapping[next..].iter().copied().for_each(begin);
        let t = &mut self.time;
        t.gather += (gathered - start).as_nanos() as u64;
        t.broadphase += (paired - gathered).as_nanos() as u64;
        t.near += (found_near - gathered).as_nanos() as u64;
        t.narrowphase += (narrowed - paired).as_nanos() as u64;
        t.merge += nanos(narrowed);
        t.contacts += nanos(start);
    }

    /// Wakes sleeping bodies whose statics a game changed since the last
    /// step: put one into them or out from under them (a static written or
    /// spawned, which is a write), or took one from under them (a row left
    /// the statics' tables: despawned, or no longer a static). Statics and
    /// sleeping bodies are both passive, so the broadphase would never pair
    /// them again. `alive` is every awake collider and static.
    fn wake_on_statics(
        &mut self,
        sleep: &mut Sleepers,
        statics: &mut Query<(&Position, &Collider), Without<(Body, Velocity)>>,
        asleep: &mut AsleepColliders<'_, '_>,
        resting: &mut RestingHere<'_, '_>,
        alive: &Slots,
    ) {
        let mut moved = Vec::new();
        statics.for_each_written(self.since, |row, (p, c)| moved.push((row.entity(), p.bounds(Some(c)).grown(narrow::MARGIN))));
        let left = statics.left_since(self.since);
        if moved.is_empty() && !left {
            return;
        }
        for &(_, around) in &moved {
            asleep.in_region(around, |row, _| sleep.wake(row.entity()));
        }
        let moved = Slots::of(moved.iter().map(|(e, _)| *e));
        resting.for_each(|row, pair| {
            for (end, other) in [(pair.a, pair.b), (pair.b, pair.a)] {
                if moved.get(end).is_some() {
                    sleep.wake(other);
                } else if left && alive.get(end).is_none() && asleep.with(end, |_, _| ()).is_none() {
                    // Rested on something gone: despawned, where leaving
                    // `Resting` would have this step's solve push on it as
                    // on a static.
                    sleep.wake(other);
                    row.despawn();
                    return;
                }
            }
        });
    }

    #[allow(clippy::too_many_arguments)]
    fn solve(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        (dt, workers): (Dt, Workers),
        mut config: Query<&Sleep>,
        // Awake bodies only: a sleeping one is immovable, and in tables of
        // its own, so walks over bodies skip it by what they match.
        mut moving: Query<(&Body, &mut Velocity, &mut Position), Without<Asleep>, Adds<Asleep>>,
        // A sleeping body's is left as it fell asleep.
        mut touching: Query<&mut Touching, Without<Asleep>>,
        mut contacts: Query<(&ContactPair, &mut Manifold, &Response, &mut Impulse), Without<Resting>, Adds<Resting>>,
        (mut sleeping, mut resting): (SleepingBodies<'_, '_>, RestingContacts<'_, '_>),
        began: EventWriter<Contact>,
    ) {
        let start = Instant::now();
        let dt = *dt;
        let par = workers.threads() > 1;
        let mut bodies = Vec::with_capacity(moving.len() + 1);
        let mut entities = Vec::with_capacity(moving.len());
        // Per body: whether it moves (isn't static), and its kind.
        let mut kinds: Vec<(bool, u8)> = Vec::with_capacity(moving.len() + 1);
        let solver_body = |body: &Body, v: &Velocity| {
            let inv_mass = if body.kind == DYNAMIC { body.inv_mass } else { 0.0 };
            SolverBody { v: Vec2::new(v.x, v.y), inv_mass, pseudo: Vec2::ZERO }
        };
        if par {
            // Lists made here, as in `find_contacts`'s gathering.
            let room = |r: std::ops::Range<usize>| (Vec::with_capacity(r.len()), Vec::with_capacity(r.len()), Vec::with_capacity(r.len()));
            let parts = moving.par_for_each(&workers, room, |(e, s, k), row, (body, v, _)| {
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
        // Bodies with no velocity (statics) and sleeping ones all stand for
        // one immovable body at the end.
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        kinds.push((false, STATIC));
        let slots = Slots::of(entities.iter().copied());
        let index_of = |e: Entity| slots.get(e).unwrap_or(still);
        let constraint = |pair: &ContactPair, m: &Manifold, r: &Response, j: &Impulse| Constraint {
            a: index_of(pair.a),
            b: index_of(pair.b),
            normal: Vec2::new(m.nx, m.ny),
            depth: m.depth,
            friction: r.friction,
            restitution: r.restitution,
            jn: j.normal,
            jt: j.tangent,
            speed: 0.0,
        };

        // In pair order, which storage keeps: the solve doesn't depend on
        // when each contact began. (History, 2026-09-24: contacts were
        // copied out of the walk and mapped after, which measured faster
        // until `for_each` walked slices.)
        let mut constraints: Vec<Constraint> = Vec::with_capacity(contacts.len());
        // With threads: by the first contact row of each chunk, its first
        // constraint, for writing back over the same chunks.
        let mut firsts: Vec<(usize, usize)> = Vec::new();
        if par {
            let parts = contacts.par_for_each_ordered_page(&workers, |rows| (rows.start, Vec::with_capacity(rows.len())), |(_, out), _, (pair, m, r, j)| {
                out.extend((0..pair.len()).filter(|&i| !r[i].disabled).map(|i| constraint(&pair[i], &m[i], &r[i], &j[i])));
            });
            for (row, part) in parts {
                firsts.push((row, constraints.len()));
                constraints.extend(part);
            }
        } else {
            contacts.for_each_ordered(|_, (pair, m, r, j)| {
                if !r.disabled {
                    constraints.push(constraint(pair, &m, r, &j));
                }
            });
        }
        let gathered = Instant::now();
        let config = sleeping_by(&mut config);
        solver::solve(&mut bodies, &mut constraints, dt);
        let after_solver = Instant::now();

        // A body's new velocity and position, `Mut`s stamping only what's
        // written.
        let write = |body: &Body, b: &SolverBody, mut v: engine_api::Mut<'_, Velocity>, mut p: engine_api::Mut<'_, Position>| {
            (v.x, v.y) = (b.v.x, b.v.y);
            // A kinematic body gets no pseudo velocity: nothing pushes it.
            let step = if body.kind == KINEMATIC { b.v } else { b.v + b.pseudo };
            // Written only when it moves: a write marks the row for the
            // spatial re-sort to re-bound, and a pile at rest comes to rest
            // bit for bit (the pile's 10 000 do by step 3000), when the
            // re-sort then has nothing to do.
            let to = (p.x + step.x * dt, p.y + step.y * dt);
            if (to.0.to_bits(), to.1.to_bits()) != (p.x.to_bits(), p.y.to_bits()) {
                (p.x, p.y) = to;
            }
        };
        if par {
            let (bodies, kinds) = (&bodies, &kinds);
            moving.par_for_each(&workers, |rows| rows.start, |k, _, (body, v, p)| {
                if kinds[*k].0 {
                    write(body, &bodies[*k], v, p);
                }
                *k += 1;
            });
        } else {
            let mut i = 0;
            moving.for_each(|_, (body, v, p)| {
                let (b, moves) = (bodies[i], kinds[i].0);
                i += 1;
                if moves {
                    write(body, &b, v, p);
                }
            });
        }

        // Most bodies don't ask (the pile's none), and a failed lookup per
        // end of every contact was half of writing back.
        let mut asking = Vec::new();
        touching.for_each(|row, mut t| {
            *t = Touching::default();
            asking.push(row.entity());
        });
        let asking = Slots::of(asking.iter().copied());
        let mut links = Vec::new();
        // A contact's results, from constraint `k`: the sides of bodies
        // that asked it touches, a link for sleeping, and whether it began.
        let wrote = |k: &Constraint, pair: &ContactPair, m: &mut Manifold, j: &mut Impulse, out: &mut WroteBack| {
            *j = Impulse { normal: k.jn, tangent: k.jt };
            m.pressed = k.jn > 0.0 || m.depth >= 0.0;
            if !m.pressed {
                return;
            }
            if config.is_some() {
                out.links.push((pair.a, pair.b));
            }
            let n = Vec2::new(m.nx, m.ny);
            for (e, n) in [(pair.a, n), (pair.b, -n)] {
                if asking.get(e).is_some() {
                    out.marks.push((e, n));
                }
            }
            if !m.was_pressed {
                out.began.push(Contact { a: pair.a, b: pair.b, nx: m.nx, ny: m.ny, speed: k.speed });
            }
        };
        // Every contact's impulse and pressing are written, so pages are
        // stamped whole, as in the merge.
        let results = if par {
            let (constraints, firsts) = (&constraints, &firsts);
            contacts.par_for_each_ordered_page(
                &workers,
                |rows| {
                    let at = firsts.iter().find(|f| f.0 == rows.start).expect("the chunks the gathering walked").1;
                    (at, WroteBack { began: Vec::with_capacity(rows.len() / 4 + 8), ..WroteBack::default() })
                },
                |(k, out), page, (pair, mut m, r, mut j)| {
                    let (m, j) = (m.write_all(), j.write_all());
                    for i in page.rows() {
                        if r[i].disabled {
                            (m[i].pressed, j[i]) = (false, Impulse::default());
                        } else {
                            wrote(&constraints[*k], &pair[i], &mut m[i], &mut j[i], out);
                            *k += 1;
                        }
                    }
                },
            )
            .into_iter()
            .map(|(_, out)| out)
            .collect()
        } else {
            let mut solved = constraints.iter();
            let mut out = WroteBack::default();
            contacts.for_each_ordered_page(|page, (pair, mut m, r, mut j)| {
                let (m, j) = (m.write_all(), j.write_all());
                for i in page.rows() {
                    if r[i].disabled {
                        (m[i].pressed, j[i]) = (false, Impulse::default());
                        continue;
                    }
                    let k = solved.next().expect("a constraint per contact solved");
                    wrote(k, &pair[i], &mut m[i], &mut j[i], &mut out);
                }
            });
            vec![out]
        };
        // In walk order: the order the events and links are in with one
        // thread. Touching is marked after, which only ever sets sides.
        for out in results {
            links.extend(out.links);
            for (e, n) in out.marks {
                touching.with(e, |_, mut t| mark(&mut t, n));
            }
            out.began.into_iter().for_each(|c| began.send(c));
        }
        if let Some(c) = config {
            self.fall_asleep(sleep, &c, dt, &entities, &bodies, &kinds, &links, &mut moving, &mut contacts);
            Self::move_woken(sleep, &mut sleeping, &mut resting);
        }
        // After every write of the step's, the stopping of bodies that fell
        // asleep included.
        self.slept = sleeping.now();
        let t = &mut self.time;
        t.solve_gather += (gathered - start).as_nanos() as u64;
        t.solver += (after_solver - gathered).as_nanos() as u64;
        t.write_back += nanos(after_solver);
        t.solve += nanos(start);
    }
}


impl Physics {
    /// Sleeping's bookkeeping after a step: see `Sleepers::update`. Links
    /// between dynamic bodies make islands; a kinematic body moving into a
    /// sleeping one wakes it. The bodies that fall asleep stop and move to
    /// their sleeping tables, and so do the contacts that then rest.
    #[allow(clippy::too_many_arguments)]
    fn fall_asleep(
        &mut self,
        sleep: &mut Sleepers,
        c: &Sleep,
        dt: f32,
        entities: &[Entity],
        bodies: &[SolverBody],
        kinds: &[(bool, u8)],
        links: &[(Entity, Entity)],
        moving: &mut Query<(&Body, &mut Velocity, &mut Position), Without<Asleep>, Adds<Asleep>>,
        contacts: &mut Query<(&ContactPair, &mut Manifold, &Response, &mut Impulse), Without<Resting>, Adds<Resting>>,
    ) {
        let slots = Slots::of(entities.iter().copied());
        // Sleeping bodies aren't among the step's, but are all dynamic.
        let kind = |e: Entity| slots.get(e).map_or(if sleep.is_asleep(e) { (false, DYNAMIC) } else { (false, STATIC) }, |k| kinds[k as usize]);
        let awake: Vec<(Entity, f32)> = (0..entities.len())
            .filter(|&k| kinds[k] == (true, DYNAMIC))
            .map(|k| (entities[k], bodies[k].v.x.hypot(bodies[k].v.y)))
            .collect();
        let mut between = Vec::with_capacity(links.len());
        let mut kinematic = Vec::new();
        for &(a, b) in links {
            match (kind(a).1, kind(b).1) {
                (DYNAMIC, DYNAMIC) => between.push((a, b)),
                (KINEMATIC, _) if kind(a).0 && bodies[slots.get(a).unwrap() as usize].v != Vec2::ZERO => kinematic.push(b),
                (_, KINEMATIC) if kind(b).0 && bodies[slots.get(b).unwrap() as usize].v != Vec2::ZERO => kinematic.push(a),
                _ => {}
            }
        }
        for e in kinematic {
            sleep.wake(e);
        }
        let fell = sleep.update(dt, c.speed, c.time, &awake, &between);
        if fell.is_empty() {
            return;
        }
        for &(e, island) in &fell {
            moving.with(e, |row, (_, mut v, _)| {
                *v = Velocity::default();
                row.insert(Asleep { island });
            });
        }
        // A contact rests once neither end moves and one sleeps. A body
        // woken this step moves from the next, though it was still in this
        // one.
        let woken = Slots::of(sleep.woken.iter().copied());
        let moves = |e: Entity| !sleep.is_asleep(e) && (woken.get(e).is_some() || slots.get(e).is_some_and(|k| kinds[k as usize].0));
        contacts.for_each(|row, (pair, _, _, _)| {
            if (sleep.is_asleep(pair.a) || sleep.is_asleep(pair.b)) && !moves(pair.a) && !moves(pair.b) {
                row.insert(Resting {});
            }
        });
    }
}

/// What writing back a chunk of contacts leaves for after: links for
/// sleeping, sides of bodies touched, and contacts begun, in walk order.
#[derive(Default)]
struct WroteBack {
    links: Vec<(Entity, Entity)>,
    marks: Vec<(Entity, Vec2)>,
    began: Vec<Contact>,
}

/// A contact found this step: what the merge makes or updates it with.
type Found = (ContactPair, Manifold, Response);

/// One chunk of a parallel merge: where in the contacts found it started
/// (the first not before its first key) and got to, and the spawns (of
/// found contacts, by index) and despawns it would have made, in order.
#[derive(Default)]
struct Merged {
    start: Option<usize>,
    next: usize,
    made: Vec<Made>,
}

enum Made {
    Spawn(usize),
    Despawn(Entity),
}

/// The thresholds sleeping goes by, if it's on: the `Sleep` entity's, or
/// `Sleep::DEFAULT` with none.
fn sleeping_by(config: &mut Query<&Sleep>) -> Option<Sleep> {
    let c = config.single(|_, c| *c).unwrap_or(Sleep::DEFAULT);
    (c.speed > 0.0).then_some(c)
}

/// Gravity's step on a body's velocity.
#[inline(always)]
fn fall(g: Vec2, dt: f32, body: &Body, mut v: engine_api::Mut<'_, Velocity>) {
    if body.kind == DYNAMIC {
        v.x += g.x * body.gravity_scale * dt;
        v.y += g.y * body.gravity_scale * dt;
    }
}

/// Gravity on the bodies woken since `integrate_velocities` gave the awake
/// ones theirs, still in their sleeping tables: so a body woken in a step
/// moves in it as one that was awake would, from rest.
fn fall_woken(sleep: &Sleepers, g: Vec2, dt: f32, falling: &mut SleepingVelocities<'_, '_>) {
    for &e in &sleep.woken {
        falling.with(e, |_, (body, v)| fall(g, dt, body, v));
    }
}

fn arrives(a: &Item, b: &Item) -> bool {
    a.body.kind != STATIC || b.body.kind != STATIC
}

fn collides(a: &Item, b: &Item) -> bool {
    let meets = a.collider.mask & b.collider.layer != 0 && b.collider.mask & a.collider.layer != 0;
    // Contacts need something to push; a sensor needs something to
    // arrive. Static colliders overlapping (a goal line and the wall
    // across its end) are the level's shape, not an event.
    let pushes = a.body.kind == DYNAMIC || b.body.kind == DYNAMIC;
    meets && if a.collider.sensor || b.collider.sensor { arrives(a, b) } else { pushes }
}

fn senses(a: &Item, b: &Item) -> bool {
    (a.collider.senses & b.collider.layer != 0 || b.collider.senses & a.collider.layer != 0) && arrives(a, b)
}

fn contact(a: &Item, b: &Item, m: &narrow::Manifold) -> Found {
    (
        ContactPair { a: a.entity, b: b.entity },
        Manifold { nx: m.normal.x, ny: m.normal.y, depth: m.depth, pressed: false, was_pressed: false },
        Response { friction: a.body.friction.min(b.body.friction), restitution: a.body.restitution.max(b.body.restitution), disabled: false },
    )
}

/// The narrowphase for a pair in a step where nothing sleeps, so nothing
/// rests or wakes: a function of the pair alone, which is what lets pairs
/// be tested on any thread.
#[inline(always)]
fn awake(a: &Item, b: &Item, found: &mut Vec<Found>, overlapping: &mut Vec<(Overlap, bool)>) {
    let (collide, sense) = (collides(a, b), senses(a, b));
    if !sense && !collide {
        return;
    }
    let Some(m) = narrow::collide(&a.placed, &b.placed, b.v - a.v) else { return };
    let sensor = collide && (a.collider.sensor || b.collider.sensor);
    if (sensor || sense) && m.depth >= 0.0 {
        overlapping.push((Overlap { a: a.entity, b: b.entity }, sensor));
    }
    if collide && !sensor {
        found.push(contact(a, b, &m));
    }
}

fn placed(p: &Position, c: &Collider) -> Placed {
    Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y) }
}

fn item(entity: Entity, p: &Position, c: &Collider, body: Body, v: Vec2, moves: bool, passive: bool) -> Item {
    Item {
        entity,
        placed: placed(p, c),
        collider: Layers { layer: c.layer, mask: c.mask, senses: c.senses, sensor: c.sensor },
        body: Material { kind: body.kind, friction: body.friction, restitution: body.restitution, moves, asleep: false, passive },
        v,
    }
}

/// Marks the side of a body its contact is on: `n` points from it toward
/// what it touches.
fn mark(t: &mut Touching, n: Vec2) {
    if n.y > 0.5 {
        t.below = true;
    } else if n.y < -0.5 {
        t.above = true;
    } else if n.x > 0.5 {
        t.right = true;
    } else if n.x < -0.5 {
        t.left = true;
    }
}

impl Mod for Physics {
    type Transient = Sleepers;

    /// Who's asleep is in the world, so a new build (a reload) picks
    /// sleeping bodies up where they are, rather than waking them all.
    fn load(&mut self, sleep: &mut Sleepers, cx: &mut Cx) {
        cx.world().for_each::<&Asleep>(|e, a| sleep.adopt(e, a.island));
    }

    fn systems(s: &mut Systems<Self>) {
        const STEP: &str = "physics::step";
        // At `simulate`'s rate, so the two make one group: a game's rules,
        // then the step, once per step.
        s.phase(STEP).after(phase::SIMULATE).before(phase::LATE).fixed_hz(phase::SIMULATE_HZ);
        s.add("integrate_velocities", Self::integrate_velocities).phase(STEP);
        s.add("find_contacts", Self::find_contacts).phase(STEP).after("physics::integrate_velocities");
        s.add("solve", Self::solve).phase(STEP).after("physics::find_contacts");
    }

    /// `stats`: the build, steps run, contacts held, and time per system.
    fn message(&mut self, sleep: &mut Sleepers, cx: &mut Cx, message: &str) -> Result<String, String> {
        match message.trim() {
            "stats" => {
                let per = |ns: u64| ns as f64 / self.steps.max(1) as f64 / 1e3;
                let mut contacts = 0;
                cx.world().for_each::<&ContactPair>(|_, _| contacts += 1);
                let t = self.time;
                Ok(format!(
                    "build {BUILD} steps {} contacts {} us/step gravity {:.1} contacts {:.1} solve {:.1}",
                    self.steps,
                    contacts,
                    per(t.gravity),
                    per(t.contacts),
                    per(t.solve)
                ))
            }
            "stages" => {
                let per = |ns: u64| ns as f64 / self.steps.max(1) as f64 / 1e3;
                let t = self.time;
                Ok(format!(
                    "gravity {:.1} gather {:.1} broadphase {:.1} near {:.1} narrowphase {:.1} merge {:.1} solve_gather {:.1} solver {:.1} write_back {:.1}",
                    per(t.gravity),
                    per(t.gather),
                    per(t.broadphase),
                    per(t.near),
                    per(t.narrowphase),
                    per(t.merge),
                    per(t.solve_gather),
                    per(t.solver),
                    per(t.write_back)
                ))
            }
            "sleeping" => Ok(format!("asleep {}", sleep.asleep)),
            "wake" => {
                sleep.wake_all();
                let mut world = cx.world();
                let mut asleep = Vec::new();
                world.for_each::<&Asleep>(|e, _| asleep.push(e));
                let mut resting = Vec::new();
                world.for_each::<(&ContactPair, &Resting)>(|e, _| resting.push(e));
                asleep.into_iter().for_each(|e| world.remove::<Asleep>(e));
                resting.into_iter().for_each(|e| world.remove::<Resting>(e));
                Ok("awake".into())
            }
            "reset_timings" => {
                (self.time, self.steps) = (Timings::default(), 0);
                Ok("reset".into())
            }
            other => Err(format!("physics doesn't understand {other:?}; try stats")),
        }
    }
}

export_mod!(Physics);
