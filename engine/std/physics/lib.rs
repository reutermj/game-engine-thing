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
    Adds, Cx, Despawns, Dt, Entity, EventWriter, Kept, Mod, OrderKey, Query, Removes, Spawner, Systems, With, Without, Workers, export_mod,
    field_struct, phase,
};
use physics::{
    Asleep, Body, Collider, Contact, ContactPair, ContactPoints, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Overlap, Placed, Position,
    Response, Resting, Rotation, STATIC, Shape, Sleep, Slept, Spin, Still, Touching, Trigger, Tuning, Vec2, Velocity,
};
use sleep::Sleepers;
use solver::{Constraint, ContactPoint, Points, SolverBody, Spinning};

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
        /// Within `solve_gather`: the bodies, and their angular state.
        solve_bodies: u64,
        solve_turning: u64,
        /// Within `write_back`: sleeping's bookkeeping after the solve.
        sleeping: u64,
    }
}

/// How far each collider's fat box reaches past its box grown by the
/// speculative margin: the broadphase keeps its pairs while bodies stay
/// inside their fat boxes (`Kept`; docs/architecture/spatial-storage.md,
/// "Keeping pairs"). A settled pile creeps less than any margin tried, so
/// the smallest was cheapest, with the fewest candidates (0.02 against
/// Box2D's 0.05: 131 µs against 139 at 10 000 settled, 2026-09-27).
const FAT: f32 = 0.02;

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics {
        steps: u64,
        /// Steps run, never reset (`steps` is, with the timings): what
        /// `Still::since` counts in.
        clock: u64,
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
        /// The last island made: ids must stay unique, since a sleeping
        /// body's is in the world.
        islands: u32,
        /// How many `Slept` physics has given and not taken off, after its
        /// last system: see `wake_by_games`.
        sleepers: u64,
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
/// Awake bodies, which the solve moves, and which fall asleep or go slower
/// (`Still`) at its apply node.
type Moving<'w, 'a> =
    Query<'w, (&'a Body, &'a mut Velocity, &'a mut Position), Without<Asleep>, (Adds<(Asleep, Slept, Still)>, Removes<Still>)>;
/// Physics's record of every body it has asleep, by the island each
/// wakes with: what waking an island walks, and a body's island is looked
/// up in.
type Records<'w, 'a> = Query<'w, &'a Slept, (), Removes<Slept>>;
/// Sleeping bodies physics hasn't taken as asleep yet: a game put them to
/// sleep. By table, so at rest it's a look at none.
type Fresh<'w, 'a> = Query<'w, &'a Asleep, (With<(Velocity, Position, Collider, Body)>, Without<Slept>), (Adds<Slept>, Removes<Still>)>;
/// How long each body slower than `Sleep::speed` has been, taken off as it
/// goes faster.
type Stills<'w, 'a> = Query<'w, &'a Still, (), Removes<Still>>;
/// Colliders' rotations: the turned ones among those gathered, looked up
/// by entity after the walks, since queries have no optional terms and a
/// walk per case would double them. Where nothing is turned it's empty,
/// and costs nothing.
type Turned<'w, 'a> = Query<'w, &'a Rotation>;
/// Bodies that turn, awake: what the solve gathers angular state from
/// and writes it back to.
type Turning<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a mut Rotation, &'a mut Spin), Without<Asleep>>;
/// Sleeping bodies' rotations and spins, for what games write to them.
type SleepingTurns<'w, 'a> = Query<'w, (&'a Rotation, &'a Spin), With<Asleep>>;

impl Physics {
    fn integrate_velocities(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        (dt, workers): (Dt, Workers),
        (mut gravity, mut spun): (Query<&Gravity>, SleepingTurns<'_, '_>),
        mut bodies: Query<(&Body, &mut Velocity), Without<Asleep>>,
        (mut config, mut sleeping, mut marks, mut resting): (
            Query<&Sleep>,
            SleepingBodies<'_, '_>,
            SleepMarks<'_, '_>,
            RestingContacts<'_, '_>,
        ),
        (mut fresh, mut records, mut colliders): (Fresh<'_, '_>, Records<'_, '_>, Query<(), With<Collider>>),
    ) {
        let start = Instant::now();
        let dt = *dt;
        self.steps += 1;
        self.clock += 1;
        let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
        if self.sleepers > 0 || !marks.is_empty() {
            let on = sleeping_by(&mut config).is_some();
            self.wake_by_games(sleep, on, (&mut sleeping, &mut marks, &mut resting), (&mut fresh, &mut records, &mut colliders));
            // A game that turned a sleeping body, or set it spinning, since
            // the last solve (whose own writes stop what falls asleep).
            spun.for_each_written(self.slept, |row, _| wake(sleep, &mut records, row.entity()));
            resolve(sleep, &mut records);
            // As `fall_woken`, through the query that has their velocities.
            for &e in &sleep.woken {
                sleeping.with(e, |_, (v, _, _, body)| fall(g, dt, body, v));
            }
            self.move_woken(sleep, &mut sleeping, &mut records, &mut resting);
        }
        if workers.threads() > 1 {
            bodies.par_for_each(&workers, |_| (), |_, _, (body, v)| fall(g, dt, body, v));
        } else {
            bodies.for_each(|_, (body, v)| fall(g, dt, body, v));
        }
        self.time.gravity += nanos(start);
    }

    /// Wakes what games changed under sleeping bodies since the last step
    /// (after tick `since`), and takes what they put to sleep, all of it
    /// found in the world, and at rest in a look per table or page:
    /// - everything, if sleeping was turned off;
    /// - a body a game put to sleep (inserting `Asleep`, spawning it with
    ///   one, or making an entity with one a body): `Asleep` without
    ///   `Slept` (`Fresh`), taken as it is, in the island it names;
    /// - a body a game woke (removing its `Asleep`) or made something else
    ///   (removing its body): more `Slept` than sleeping bodies with it,
    ///   and a walk finds which, whose islands wake;
    /// - a body a game despawned: fewer `Slept` than physics gave, and a
    ///   walk of the resting contacts finds the ends gone, whose other
    ///   ends' islands wake;
    /// - a body a game wrote (its velocity, position, collider or body),
    ///   and its island (`for_each_written`).
    fn wake_by_games(
        &mut self,
        sleep: &mut Sleepers,
        on: bool,
        (sleeping, marks, resting): (&mut SleepingBodies<'_, '_>, &mut SleepMarks<'_, '_>, &mut RestingContacts<'_, '_>),
        (fresh, records, colliders): (&mut Fresh<'_, '_>, &mut Records<'_, '_>, &mut Query<(), With<Collider>>),
    ) {
        let recorded = records.len() as u64;
        // Only physics gives `Slept`, and it counts what it gives and takes
        // off (`sleepers`), so fewer can only be a game's despawn: a count
        // no game can make up for with a `Slept` of its own.
        let despawned = self.sleepers > recorded;
        self.sleepers = recorded;
        if !on {
            sleeping.for_each(|row, _| sleep.woken.push(row.entity()));
            records.for_each(|row, _| sleep.woken.push(row.entity()));
            return;
        }
        // Taken before anything wakes, as physics's own are: one in an
        // island woken here would be taken for new again next step.
        let mut new = Vec::new();
        let islands = &mut self.islands;
        fresh.for_each(|row, a| {
            row.insert(Slept { island: a.island });
            row.remove::<Still>();
            *islands = (*islands).max(a.island);
            new.push(row.entity());
        });
        self.sleepers += new.len() as u64;
        let proper = sleeping.len() - new.len();
        let new = Slots::of(new.into_iter());
        let (since, slept) = (self.since, self.slept);
        let mut written = Vec::new();
        sleeping.for_each_written(since, |row, _| written.push(row.entity()));
        written.retain(|&e| {
            if new.get(e).is_some() {
                return false;
            }
            // Put to sleep by the last solve, which wrote it after
            // `since`: only what's written after the solve is a game's.
            let fell = marks.written(e).is_some_and(|t| t > since);
            !fell || sleeping.written(e).is_some_and(|t| t > slept)
        });
        if recorded as usize > proper {
            records.for_each(|row, s| {
                if sleeping.get(row.entity()).is_none() {
                    sleep.waking.push(s.island);
                    sleep.woken.push(row.entity());
                }
            });
        }
        // Gone, a body left its resting contacts: what rested on it, or
        // under it, may no longer be where it was. The one gone is woken
        // too, so its contacts stop resting (and are despawned by the merge,
        // not being found).
        if despawned {
            resting.for_each(|_, pair| {
                for (end, other) in [(pair.a, pair.b), (pair.b, pair.a)] {
                    if colliders.get(end).is_none() {
                        wake(sleep, records, other);
                        sleep.woken.push(end);
                    }
                }
            });
        }
        for e in written {
            wake(sleep, records, e);
        }
    }

    /// Moves the bodies woken since this last ran out of their sleeping
    /// tables, and the resting contacts they're an end of back into the
    /// step. Rare, so walking every resting contact is fine. Generic over
    /// the queries, since each system that wakes bodies declares its own.
    /// `woken` is as `resolve` leaves it: once each.
    fn move_woken<D: engine_api::engine_ecs::Data, F, C, G, H>(
        &mut self,
        sleep: &mut Sleepers,
        sleeping: &mut Query<'_, D, F, C>,
        records: &mut Records<'_, '_>,
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
            if let Some(row) = records.get(e) {
                row.remove::<Slept>();
                self.sleepers -= 1;
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
            Query<(&ContactPair, &mut Manifold, &mut Response, &mut ContactPoints), Without<Resting>, Despawns>,
            Spawner<(ContactPair, Manifold, Response, Impulse, ContactPoints)>,
        ),
        (mut overlaps, new_overlaps): (Query<&Overlap, (), Despawns>, Spawner<(Overlap,)>),
        (triggers, mut turned): (EventWriter<Trigger>, Turned<'_, '_>),
        // Gravity, for bodies woken here: see `fall_woken`.
        (dt, mut gravity, mut falling, mut records): (Dt, Query<&Gravity>, SleepingVelocities<'_, '_>, Records<'_, '_>),
        (workers, mut kept): (Workers, Kept<'_, Position>),
    ) {
        let start = Instant::now();
        // Split across threads only while nothing sleeps: waking looks
        // sleeping bodies up as their pairs are found, one at a time.
        let par = workers.threads() > 1 && asleep.is_empty();
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
            let parts = moving.par_for_each(&workers, room, |out, row, (p, c, b, u)| {
                out.push(item(row.entity(), p, c, *b, v(u), b.kind != STATIC, false))
            });
            join(&mut items, parts);
            let parts =
                held.par_for_each(&workers, room, |out, row, (p, c, b)| out.push(item(row.entity(), p, c, *b, Vec2::ZERO, false, false)));
            join(&mut items, parts);
            let parts = drifting
                .par_for_each(&workers, room, |out, row, (p, c, u)| out.push(item(row.entity(), p, c, Body::fixed(), v(u), false, false)));
            join(&mut items, parts);
            let parts = statics.par_for_each(&workers, room, |out, row, (p, c)| {
                out.push(item(row.entity(), p, c, Body::fixed(), Vec2::ZERO, false, true))
            });
            join(&mut items, parts);
        } else {
            moving.for_each(|row, (p, c, b, u)| items.push(item(row.entity(), p, c, *b, v(u), b.kind != STATIC, false)));
            held.for_each(|row, (p, c, b)| items.push(item(row.entity(), p, c, *b, Vec2::ZERO, false, false)));
            drifting.for_each(|row, (p, c, u)| items.push(item(row.entity(), p, c, Body::fixed(), v(u), false, false)));
            statics.for_each(|row, (p, c)| items.push(item(row.entity(), p, c, Body::fixed(), Vec2::ZERO, false, true)));
        }
        let mut slots = Slots::of(items.iter().map(|i| i.entity));
        // Turned colliders' rotations, into what was gathered. A sleeping
        // one's is looked up as it's fetched, below.
        let turns = !turned.is_empty();
        if turns {
            turned.for_each(|row, q| {
                if let Some(k) = slots.get(row.entity()) {
                    items[k as usize].placed.rot = Some(q.rot());
                }
            });
        }
        if !asleep.is_empty() {
            self.wake_on_statics((sleep, &mut records), &mut statics, &mut asleep, &mut resting, &slots);
            resolve(sleep, &mut records);
        }
        // The storage's own order is the broadphase: pairs whose boxes, grown
        // by the speculative margin, meet, and one of which is awake and can
        // move. In entity order, lesser first, so what's found doesn't
        // depend on the order items were gathered in.
        let gathered = Instant::now();
        // Kept between steps (`Kept`): a pile at rest or creeping inside its
        // fat boxes finds its pairs without looking.
        let near = kept.near_pairs_with(&workers, &(&moving, &held, &drifting), &(&statics, &asleep), narrow::MARGIN, FAT);
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
                if turns {
                    it.placed.rot = turned.with(e, |_, q| q.rot());
                }
                items.push(it);
                slots.insert(e, k);
                k
            };
            for &(a, b) in near {
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
        // The points of those found with points, which `Found` indexes.
        let mut points: Vec<ContactPoints> = Vec::new();
        // Each overlap, and whether it's a sensor's, which triggers.
        let mut overlapping: Vec<(Overlap, bool)> = Vec::new();
        // Unless what woke above was all of it.
        let any_asleep = asleep.len() > sleep.woken.len();
        let pair = |&(i, j): &(u32, u32)| (&items[i as usize], &items[j as usize]);
        // Split on whether anything sleeps, so a step with nothing asleep
        // tests nothing for it: the always-false tests cost about 7 µs of
        // 110 at 10 000 settled and at rest (2026-09-24, against the arrays'
        // narrowphase over six runs each), as a dead test once did in the
        // solve (physics.md, "What the ECS costs").
        if par {
            // Room for a contact per pair, made here (see the gathering).
            let parts = engine_api::engine_ecs::par::even(pairs.len(), workers.chunks(pairs.len(), 256));
            let parts = parts.into_iter().map(|r| (Vec::with_capacity(r.len()), Vec::new(), Vec::new(), r)).collect();
            let tested = workers.map_each(parts, |_, (mut found, mut overlapping, mut points, r)| {
                pairs[r].iter().map(pair).for_each(|(a, b)| awake(a, b, &mut found, &mut overlapping, &mut points));
                (found, overlapping, points)
            });
            found.reserve_exact(tested.iter().map(|(f, _, _)| f.len()).sum());
            for (f, o, p) in tested {
                // Each chunk's points after the chunks' before it.
                let offset = points.len() as u32;
                found.extend(f.into_iter().map(|f| if f.3 > 0 { (f.0, f.1, f.2, f.3 + offset) } else { f }));
                overlapping.extend(o);
                points.extend(p);
            }
        } else if any_asleep {
            // What `awake` does, where a pair may rest, or wake what rests.
            let mut test = |a: &Item, b: &Item| {
                let (collide, sense) = (collides(a, b), senses(a, b));
                let rest = rests(a, b);
                if !sense && (!collide || rest) {
                    return;
                }
                let Some(f) = meet(a, b, &mut points) else { return };
                let sensor = collide && (a.collider.sensor || b.collider.sensor);
                if (sensor || sense) && f.1.depth >= 0.0 {
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
                    wake(sleep, &mut records, a.entity);
                    wake(sleep, &mut records, b.entity);
                } else if collide && !sensor && !rest {
                    found.push(f);
                }
            };
            pairs.iter().map(pair).for_each(|(a, b)| test(a, b));
        } else {
            pairs.iter().map(pair).for_each(|(a, b)| awake(a, b, &mut found, &mut overlapping, &mut points));
        }

        let narrowed = Instant::now();
        let key = |p: &ContactPair| (p.a, p.b);
        let points = &points;
        let geometry = |f: &Found| f.3.checked_sub(1).map(|k| &points[k as usize]);
        let spawn = |f: Found| {
            new_contacts.spawn((f.0, f.1, f.2, Impulse::default(), geometry(&f).copied().unwrap_or_default()));
        };
        // A persisting contact: this step's manifold, and its points'
        // geometry with the last solve's impulses kept, unless it has no
        // points now. `ContactPoints` is written only where there are
        // points, so a world where nothing turns never touches it.
        let merge = |f: &Found, m: &mut Manifold, cp: &mut engine_api::ColumnMut<'_, ContactPoints>, i: usize| {
            *m = Manifold { was_pressed: m.pressed, solved: if f.3 > 0 { m.solved } else { 0 }, ..f.1 };
            if let Some(g) = geometry(f) {
                let mut cp = cp.get_mut(i);
                (cp.anchors, cp.separations, cp.ids) = (g.anchors, g.separations, g.ids);
            }
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
            let merged = contacts.par_for_each_ordered_page(&workers, made, |c, page, (pair, mut m, mut r, mut cp)| {
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
                            merge(f, &mut m[i], &mut cp, i);
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
            contacts.for_each_ordered_page(|page, (pair, mut m, mut r, mut cp)| {
                let (m, r) = (m.write_all(), r.write_all());
                for i in page.rows() {
                    while found.get(next).is_some_and(|f| key(&f.0) < key(&pair[i])) {
                        spawn(found[next]);
                        next += 1;
                    }
                    match found.get(next) {
                        Some(f) if f.0 == pair[i] => {
                            merge(f, &mut m[i], &mut cp, i);
                            r[i] = f.2;
                            next += 1;
                        }
                        _ => {
                            if any_asleep && m[i].pressed {
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
            wake(sleep, &mut records, pair.a);
            wake(sleep, &mut records, pair.b);
        }
        resolve(sleep, &mut records);
        // What woke here moves at this system's apply node, so this step's
        // solve has it: movable, falling, and its resting contacts solved
        // again.
        if !sleep.woken.is_empty() {
            let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
            fall_woken(sleep, g, *dt, &mut falling);
        }
        self.move_woken(sleep, &mut asleep, &mut records, &mut resting);
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
            } else if !(any_asleep && passive(o.a) && passive(o.b)) {
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
        (sleep, records): (&mut Sleepers, &mut Records<'_, '_>),
        statics: &mut Query<(&Position, &Collider), Without<(Body, Velocity)>>,
        asleep: &mut AsleepColliders<'_, '_>,
        resting: &mut RestingHere<'_, '_>,
        alive: &Slots,
    ) {
        let mut moved = Vec::new();
        // Bounded as though turned every way, since this walk doesn't have
        // their rotations: it only finds what to wake.
        statics.for_each_written(self.since, |row, (p, c)| moved.push((row.entity(), any_way(p, c).grown(narrow::MARGIN))));
        let left = statics.left_since(self.since);
        if moved.is_empty() && !left {
            return;
        }
        for &(_, around) in &moved {
            asleep.in_region(around, |row, _| wake(sleep, records, row.entity()));
        }
        let moved = Slots::of(moved.iter().map(|(e, _)| *e));
        resting.for_each(|row, pair| {
            for (end, other) in [(pair.a, pair.b), (pair.b, pair.a)] {
                if moved.get(end).is_some() {
                    wake(sleep, records, other);
                } else if left && alive.get(end).is_none() && asleep.with(end, |_, _| ()).is_none() {
                    // Rested on something gone: despawned, where leaving
                    // `Resting` would have this step's solve push on it as
                    // on a static.
                    wake(sleep, records, other);
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
        (mut config, mut gravity, mut tuning): (Query<&Sleep>, Query<&Gravity>, Query<&Tuning>),
        // Awake bodies only: a sleeping one is immovable, and in tables of
        // its own, so walks over bodies skip it by what they match.
        (mut moving, mut stills): (Moving<'_, '_>, Stills<'_, '_>),
        // A sleeping body's is left as it fell asleep.
        (mut touching, mut turning): (Query<&mut Touching, Without<Asleep>>, Turning<'_, '_>),
        mut contacts: Query<(&ContactPair, &mut Manifold, &Response, &mut Impulse, &mut ContactPoints), Without<Resting>, Adds<Resting>>,
        (mut sleeping, mut resting, mut records): (SleepingBodies<'_, '_>, RestingContacts<'_, '_>, Records<'_, '_>),
        began: EventWriter<Contact>,
    ) {
        let start = Instant::now();
        let dt = *dt;
        let par = workers.threads() > 1;
        let mut bodies = Vec::with_capacity(moving.len() + 1);
        let mut entities = Vec::with_capacity(moving.len());
        // Per body: whether it moves (isn't static), and its kind.
        let mut kinds: Vec<(bool, u8)> = Vec::with_capacity(moving.len() + 1);
        let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
        let solver_body = |body: &Body, v: &Velocity| {
            // The gravity `integrate_velocities` added, computed as `fall`
            // computed it, for the solver to spread over its substeps.
            let (inv_mass, g) = if body.kind == DYNAMIC { (body.inv_mass, g) } else { (0.0, Vec2::ZERO) };
            let gravity = Vec2::new(g.x * body.gravity_scale * dt, g.y * body.gravity_scale * dt);
            SolverBody::new(Vec2::new(v.x, v.y), inv_mass, gravity)
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
        let t_bodies = Instant::now();
        // Angular state, for the bodies that turn, and each one's reach
        // (for how fast its edge moves, which sleeping goes by).
        let (mut spinning, mut reach): (Vec<Spinning>, Vec<f32>) = (Vec::new(), Vec::new());
        turning.for_each(|row, (body, c, _, spin)| {
            let Some(k) = slots.get(row.entity()) else { return };
            spinning.push(match body.kind {
                DYNAMIC => Spinning::new(k, spin.w, bodies[k as usize].inv_mass * c.inertia_per_mass()),
                KINEMATIC => Spinning::new(k, spin.w, 0.0),
                _ => return,
            });
            reach.push(c.reach());
        });
        let t_turning = Instant::now();
        let index_of = |e: Entity| slots.get(e).unwrap_or(still);
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

        // In pair order, which storage keeps: the solve doesn't depend on
        // when each contact began. (History, 2026-09-24: contacts were
        // copied out of the walk and mapped after, which measured faster
        // until `for_each` walked slices.)
        let mut constraints: Vec<Constraint> = Vec::with_capacity(contacts.len());
        let mut points: Vec<Points> = Vec::new();
        // With threads: by the first contact row of each chunk, its first
        // constraint, for writing back over the same chunks.
        let mut firsts: Vec<(usize, usize)> = Vec::new();
        if par {
            let parts = contacts.par_for_each_ordered_page(
                &workers,
                |rows| (rows.start, Vec::with_capacity(rows.len()), Vec::new()),
                |(_, out, pts), _, (pair, m, r, j, cp)| {
                    out.extend((0..pair.len()).filter(|&i| !r[i].disabled).map(|i| constraint(&pair[i], &m[i], &r[i], &j[i], &cp[i], pts)));
                },
            );
            for (row, part, pts) in parts {
                firsts.push((row, constraints.len()));
                // Each chunk's points after the chunks before it.
                let offset = points.len() as u32;
                constraints.extend(part.into_iter().map(|c| if c.points > 0 { Constraint { points: c.points + offset, ..c } } else { c }));
                points.extend(pts);
            }
        } else {
            // By page, so a row costs no dispatch per term (docs/lore on a
            // query's row cost), and `ContactPoints` is read only where a
            // contact has points.
            contacts.for_each_ordered_page(|page, (pair, m, r, j, cp)| {
                for i in page.rows() {
                    if !r[i].disabled {
                        constraints.push(constraint(&pair[i], &m[i], &r[i], &j[i], &cp[i], &mut points));
                    }
                }
            });
        }
        let gathered = Instant::now();
        self.time.solve_bodies += (t_bodies - start).as_nanos() as u64;
        self.time.solve_turning += (t_turning - t_bodies).as_nanos() as u64;
        let config = sleeping_by(&mut config);
        let params = solver::Params::of(&tuning.single(|_, t| *t).unwrap_or(Tuning::DEFAULT));
        solver::solve_with(&params, (&mut bodies, &mut spinning), &mut constraints, &mut points, dt);
        let after_solver = Instant::now();

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
        if !spinning.is_empty() {
            // The walk that gathered them, in the same order.
            let mut each = spinning.iter();
            turning.for_each(|row, (body, _, mut q, mut spin)| {
                let Some(k) = slots.get(row.entity()) else { return };
                if body.kind != DYNAMIC && body.kind != KINEMATIC {
                    return;
                }
                let b = each.next().expect("a spinning body per one gathered");
                debug_assert_eq!(b.body, k);
                let Some(to) = b.turned_from(q.rot(), spin.w).filter(|_| kinds[k as usize].0) else { return };
                if spin.w.to_bits() != b.w.to_bits() {
                    spin.w = b.w;
                }
                if (to.c.to_bits(), to.s.to_bits()) != (q.c.to_bits(), q.s.to_bits()) {
                    *q = Rotation::of(to);
                }
            });
        }
        if par {
            let (bodies, kinds) = (&bodies, &kinds);
            moving.par_for_each(
                &workers,
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
        let points = &points;
        let wrote = |k: &Constraint, pair: &ContactPair, m: &mut Manifold, j: &mut Impulse, out: &mut WroteBack| {
            *j = Impulse { normal: k.jn, tangent: k.jt };
            m.solved = k.points.checked_sub(1).map(|at| &points[at as usize]).filter(|p| p.solved).map_or(0, |p| p.count);
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
        let results = if par {
            let (constraints, firsts) = (&constraints, &firsts);
            contacts
                .par_for_each_ordered_page(
                    &workers,
                    |rows| {
                        let at = firsts.iter().find(|f| f.0 == rows.start).expect("the chunks the gathering walked").1;
                        (at, WroteBack { began: Vec::with_capacity(rows.len() / 4 + 8), ..WroteBack::default() })
                    },
                    |(k, out), page, (pair, mut m, r, mut j, mut cp)| {
                        let (m, j) = (m.write_all(), j.write_all());
                        for i in page.rows() {
                            if r[i].disabled {
                                (m[i].pressed, j[i]) = (false, Impulse::default());
                            } else {
                                let c = &constraints[*k];
                                wrote(c, &pair[i], &mut m[i], &mut j[i], out);
                                if m[i].solved > 0 {
                                    solved_at(c, cp.get_mut(i));
                                }
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
            contacts.for_each_ordered_page(|page, (pair, mut m, r, mut j, mut cp)| {
                let (m, j) = (m.write_all(), j.write_all());
                for i in page.rows() {
                    if r[i].disabled {
                        (m[i].pressed, j[i]) = (false, Impulse::default());
                        continue;
                    }
                    let k = solved.next().expect("a constraint per contact solved");
                    wrote(k, &pair[i], &mut m[i], &mut j[i], &mut out);
                    if m[i].solved > 0 {
                        solved_at(k, cp.get_mut(i));
                    }
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
            let t_sleeping = Instant::now();
            self.fall_asleep(
                (sleep, &mut records),
                &c,
                dt,
                (&entities, &bodies, &kinds),
                (&spinning, &reach),
                &links,
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
        let t = &mut self.time;
        t.solve_gather += (gathered - start).as_nanos() as u64;
        t.solver += (after_solver - gathered).as_nanos() as u64;
        t.write_back += nanos(after_solver);
        t.solve += nanos(start);
    }
}

impl Physics {
    /// Sleeping's bookkeeping after a step: see `sleep::islands`. Links
    /// between dynamic bodies make islands; a kinematic body moving into a
    /// sleeping one wakes it. The bodies that fall asleep stop and move to
    /// their sleeping tables, and so do the contacts that then rest.
    #[allow(clippy::too_many_arguments)]
    fn fall_asleep(
        &mut self,
        (sleep, records): (&mut Sleepers, &mut Records<'_, '_>),
        c: &Sleep,
        dt: f32,
        (entities, bodies, kinds): (&[Entity], &[SolverBody], &[(bool, u8)]),
        (spinning, reach): (&[Spinning], &[f32]),
        links: &[(Entity, Entity)],
        (moving, stills, turning): (&mut Moving<'_, '_>, &mut Stills<'_, '_>, &mut Turning<'_, '_>),
        contacts: &mut Query<(&ContactPair, &mut Manifold, &Response, &mut Impulse, &mut ContactPoints), Without<Resting>, Adds<Resting>>,
    ) {
        let slots = Slots::of(entities.iter().copied());
        let mut speed: Vec<f32> = bodies.iter().map(|b| b.v.x.hypot(b.v.y)).collect();
        // A turning body is as fast as its edge, as Box2D has it
        // (`maxExtent` in `b2FinalizeBodies`).
        for (s, r) in spinning.iter().zip(reach) {
            speed[s.body as usize] = speed[s.body as usize].max(s.w.abs() * r);
        }
        // How long each awake body has been still, from its `Still`, which
        // is taken off one going faster here and put on one going slower
        // below. Still is slower, so a speed that isn't a number is moving,
        // as it was when this counted seconds.
        let still = |s: f32| s < c.speed;
        let mut since: Vec<Option<u64>> = vec![None; entities.len()];
        stills.for_each(|row, s| match slots.get(row.entity()) {
            Some(k) if !still(speed[k as usize]) => row.remove::<Still>(),
            Some(k) => since[k as usize] = Some(s.since),
            None => {}
        });
        let clock = self.clock;
        let awake: Vec<(Entity, u64)> = (0..entities.len())
            .filter(|&k| kinds[k] == (true, DYNAMIC))
            .map(|k| (entities[k], if still(speed[k]) { clock - since[k].unwrap_or(clock) + 1 } else { 0 }))
            .collect();
        // Each end of a link as the step has it (whether it moves, and its
        // kind), and, for one not among the step's bodies (a sleeping body
        // or a static), its island if physics has it asleep: sleeping bodies
        // are all dynamic.
        let mut link_end = |e: Entity| match slots.get(e) {
            Some(k) => (kinds[k as usize], None),
            None => match records.with(e, |_, s| s.island) {
                Some(island) => ((false, DYNAMIC), Some(island)),
                None => ((false, STATIC), None),
            },
        };
        let pushes =
            |e: Entity, (moves, kind): (bool, u8)| kind == KINEMATIC && moves && bodies[slots.get(e).unwrap() as usize].v != Vec2::ZERO;
        let (mut between, mut against) = (Vec::with_capacity(links.len()), Vec::new());
        for &(a, b) in links {
            let ((ka, ia), (kb, ib)) = (link_end(a), link_end(b));
            match (ka.1, kb.1) {
                (DYNAMIC, DYNAMIC) => match (ia, ib) {
                    (None, None) => between.push((a, b)),
                    (None, Some(i)) => against.push((a, i)),
                    (Some(i), None) => against.push((b, i)),
                    (Some(_), Some(_)) => {}
                },
                (KINEMATIC, _) if pushes(a, ka) => sleep.waking.extend(ib),
                (_, KINEMATIC) if pushes(b, kb) => sleep.waking.extend(ia),
                _ => {}
            }
        }
        let enough = sleep.enough(dt, c.time);
        let (wake, fell) = sleep::islands(enough, &awake, &between, &against, &mut self.islands);
        sleep.waking.extend(wake);
        for &(e, _) in awake.iter().filter(|&&(_, steps)| steps == 1) {
            if let Some(row) = moving.get(e) {
                row.insert(Still { since: clock });
            }
        }
        // What woke is known before contacts are marked resting, below.
        resolve(sleep, records);
        if fell.is_empty() {
            return;
        }
        // Asleep, a body doesn't keep its `Still`: woken, it starts from
        // moving, and the walk above is of awake bodies alone.
        for &(e, island) in &fell {
            turning.with(e, |_, (_, _, _, mut s)| *s = Spin::default());
            moving.with(e, |row, (_, mut v, _)| {
                *v = Velocity::default();
                row.insert(Asleep { island });
                row.insert(Slept { island });
                row.remove::<Still>();
            });
        }
        self.sleepers += fell.len() as u64;
        // A contact rests once neither end moves and one sleeps, as the
        // world will have them after this system's apply node. A body woken
        // this step moves from the next, though it was still in this one.
        let (fell, woken) = (Slots::of(fell.iter().map(|&(e, _)| e)), Slots::of(sleep.woken.iter().copied()));
        let mut end = |e: Entity| {
            let asleep = fell.get(e).is_some() || (slots.get(e).is_none() && woken.get(e).is_none() && records.get(e).is_some());
            let moves = !asleep && (woken.get(e).is_some() || slots.get(e).is_some_and(|k| kinds[k as usize].0));
            (asleep, moves)
        };
        contacts.for_each(|row, (pair, _, _, _, _)| {
            let ((a, a_moves), (b, b_moves)) = (end(pair.a), end(pair.b));
            if (a || b) && !a_moves && !b_moves {
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
/// Its points, if it has any, are `points[.3 - 1]` of the step's.
type Found = (ContactPair, Manifold, Response, u32);

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
/// Wakes `e`'s island, if physics has it asleep: a kinematic body moving
/// into it, a game writing it, or its support gone. As islands, which
/// `resolve` makes bodies of.
fn wake(sleep: &mut Sleepers, records: &mut Records<'_, '_>, e: Entity) {
    if let Some(island) = records.with(e, |_, s| s.island) {
        sleep.waking.push(island);
    }
}

/// The bodies of the islands woken since this last ran, into `woken`,
/// which is left in entity order and once each: gravity on them and their
/// move out of the sleeping tables would count one twice. A walk of every
/// sleeping body, and as rare as waking.
fn resolve(sleep: &mut Sleepers, records: &mut Records<'_, '_>) {
    if !sleep.waking.is_empty() {
        let mut islands = std::mem::take(&mut sleep.waking);
        islands.sort_unstable();
        islands.dedup();
        records.for_each(|row, s| {
            if islands.binary_search(&s.island).is_ok() {
                sleep.woken.push(row.entity());
            }
        });
    }
    sleep.woken.sort_unstable();
    sleep.woken.dedup();
}

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

/// The box around a collider however it's turned.
fn any_way(p: &Position, c: &Collider) -> engine_api::Bounds {
    engine_api::Bounds::around([p.x, p.y], [c.reach(), c.reach()])
}

/// The narrowphase for a pair: its contact as the world keeps it, if
/// they're within the margin. Shapes that aren't turned meet as they always
/// did, with no points; a turned one (and so any body that turns, which
/// has a rotation) has its points found, and put in `points`.
#[inline(always)]
fn meet(a: &Item, b: &Item, points: &mut Vec<ContactPoints>) -> Option<Found> {
    if a.placed.rot.is_none() && b.placed.rot.is_none() {
        let m = narrow::collide(&a.placed, &b.placed, b.v - a.v)?;
        return Some(contact(a, b, Manifold { nx: m.normal.x, ny: m.normal.y, depth: m.depth, ..Manifold::default() }, 0));
    }
    let m = narrow::collide_turned(&a.placed, &b.placed)?;
    let mut cp = ContactPoints::default();
    for (i, p) in m.points[..m.count as usize].iter().enumerate() {
        cp.anchors[4 * i..4 * i + 4].copy_from_slice(&[p.ra.x, p.ra.y, p.rb.x, p.rb.y]);
        cp.separations[i] = p.separation;
        cp.ids[i] = p.id;
    }
    points.push(cp);
    let manifold = Manifold { nx: m.normal.x, ny: m.normal.y, depth: m.depth, points: m.count, ..Manifold::default() };
    Some(contact(a, b, manifold, points.len() as u32))
}

fn contact(a: &Item, b: &Item, manifold: Manifold, points: u32) -> Found {
    (
        ContactPair { a: a.entity, b: b.entity },
        manifold,
        Response {
            friction: a.body.friction.min(b.body.friction),
            restitution: a.body.restitution.max(b.body.restitution),
            disabled: false,
        },
        points,
    )
}

/// The narrowphase for a pair in a step where nothing sleeps, so nothing
/// rests or wakes: a function of the pair alone, which is what lets pairs
/// be tested on any thread.
#[inline(always)]
fn awake(a: &Item, b: &Item, found: &mut Vec<Found>, overlapping: &mut Vec<(Overlap, bool)>, points: &mut Vec<ContactPoints>) {
    let (collide, sense) = (collides(a, b), senses(a, b));
    if !sense && !collide {
        return;
    }
    let Some(f) = meet(a, b, points) else { return };
    let sensor = collide && (a.collider.sensor || b.collider.sensor);
    if (sensor || sense) && f.1.depth >= 0.0 {
        overlapping.push((Overlap { a: a.entity, b: b.entity }, sensor));
    }
    if collide && !sensor {
        found.push(f);
    }
}

/// Not turned: a turned one's rotation is filled in after.
fn placed(p: &Position, c: &Collider) -> Placed {
    Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y), rot: None }
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

    /// Nothing to take over: all sleeping keeps is in the world, or in the
    /// state, which a reload carries. A state that starts empty (a first
    /// build's, or one reset) counts from the world: the `Slept` there (so
    /// a sleeping body despawned since the last step wakes nothing, the one
    /// change a reset state can't see), the greatest island, and the last
    /// step a `Still` began at, which its clock mustn't be behind.
    /// (History, 2026-09-26: the mod kept a copy of who was asleep and how
    /// long each awake body had been still, handed to the next build in
    /// `unload`; see docs/architecture/physics.md, [^sleep-copy].)
    fn load(&mut self, _: &mut Sleepers, cx: &mut Cx) {
        if self.clock > 0 {
            return;
        }
        let mut world = cx.world();
        world.for_each::<&Slept>(|_, s| {
            self.sleepers += 1;
            self.islands = self.islands.max(s.island);
        });
        world.for_each::<&Asleep>(|_, a| self.islands = self.islands.max(a.island));
        world.for_each::<&Still>(|_, s| self.clock = self.clock.max(s.since));
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
    fn message(&mut self, _: &mut Sleepers, cx: &mut Cx, message: &str) -> Result<String, String> {
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
                ) + &format!(
                    " solve_bodies {:.1} solve_turning {:.1} sleeping {:.1}",
                    per(t.solve_bodies),
                    per(t.solve_turning),
                    per(t.sleeping)
                ))
            }
            "sleeping" => {
                let mut asleep = 0;
                cx.world().for_each::<(&Asleep, &Slept)>(|_, _| asleep += 1);
                Ok(format!("asleep {asleep}"))
            }
            "wake" => {
                let mut world = cx.world();
                let mut asleep = Vec::new();
                world.for_each::<&Asleep>(|e, _| asleep.push(e));
                let mut slept = Vec::new();
                world.for_each::<&Slept>(|e, _| slept.push(e));
                slept.into_iter().for_each(|e| world.remove::<Slept>(e));
                self.sleepers = 0;
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
