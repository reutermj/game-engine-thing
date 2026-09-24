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
    Adds, Cx, Despawns, Dt, Entity, EventWriter, Mod, OrderKey, Query, Removes, SpatialKey, Spawner, Systems, With, Without,
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
        /// The world's tick after the last solve: what games wrote to a
        /// sleeping body since then woke it. Carried across reloads, so a
        /// new build doesn't take its own step's writes for a game's.
        since: u32,
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
/// `Asleep` removed. Its terms are what a game writes to wake one.
type SleepingBodies<'w, 'a> =
    Query<'w, (&'a Asleep, &'a Velocity, &'a Position, &'a Collider, &'a Body), (), Removes<Asleep>>;
/// Contacts kept as they are while their ends sleep: an end woken is
/// `Resting` removed.
type RestingContacts<'w, 'a> = Query<'w, &'a ContactPair, With<Resting>, Removes<Resting>>;

impl Physics {
    fn integrate_velocities(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        dt: Dt,
        mut gravity: Query<&Gravity>,
        mut bodies: Query<(&Body, &mut Velocity), Without<Asleep>>,
        (mut config, mut sleeping, mut resting): (Query<&Sleep>, SleepingBodies<'_, '_>, RestingContacts<'_, '_>),
    ) {
        let start = Instant::now();
        let dt = *dt;
        self.steps += 1;
        if sleep.asleep > 0 || !sleeping.is_empty() {
            Self::wake_by_games(sleep, config.single(|_, _| ()).is_some(), self.since, &mut sleeping);
            Self::move_woken(sleep, &mut sleeping, &mut resting);
        }
        let g = gravity.single(|_, g| Vec2::new(g.x, g.y)).unwrap_or_default();
        bodies.for_each(|_, (body, mut v)| {
            if body.kind == DYNAMIC {
                v.x += g.x * body.gravity_scale * dt;
                v.y += g.y * body.gravity_scale * dt;
            }
        });
        self.time.gravity += nanos(start);
    }

    /// Wakes what games changed under sleeping bodies since the last step:
    /// everything if sleeping was turned off; a body a game despawned or
    /// woke (removing its `Asleep`), and its island; a body a game wrote
    /// (its velocity, position, collider or body) after tick `since`, found
    /// by pages written (`for_each_written`), so at rest it costs a look
    /// per page.
    fn wake_by_games(sleep: &mut Sleepers, on: bool, since: u32, sleeping: &mut SleepingBodies<'_, '_>) {
        if !on {
            sleep.wake_all();
            sleeping.for_each(|row, _| sleep.woken.push(row.entity()));
            return;
        }
        let there = sleeping.len();
        if there != sleep.asleep {
            let mut alive = Vec::with_capacity(there);
            sleeping.for_each(|row, (a, ..)| alive.push((row.entity(), a.island)));
            let slots = Slots::of(alive.iter().map(|(e, _)| *e));
            sleep.wake_missing(|e| slots.get(e).is_some());
            // Put to sleep by a game: taken as it is.
            for (e, island) in alive {
                sleep.adopt(e, island);
            }
        }
        let mut set = Vec::new();
        sleeping.for_each_written(since, |row, _| set.push(row.entity()));
        for e in set {
            sleep.wake(e);
        }
    }

    /// Moves the bodies woken since this last ran out of their sleeping
    /// tables, and the resting contacts they're an end of (or whose end is
    /// gone) back into the step. Rare, so walking every resting contact is
    /// fine.
    fn move_woken(sleep: &mut Sleepers, sleeping: &mut SleepingBodies<'_, '_>, resting: &mut RestingContacts<'_, '_>) {
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
        // the contacts they rest on.
        (mut asleep, mut resting): (Query<(&Position, &Collider, &Body), With<Asleep>>, Query<&ContactPair, With<Resting>>),
        (mut contacts, new_contacts): (
            Query<(&ContactPair, &mut Manifold, &mut Response), Without<Resting>, Despawns>,
            Spawner<(ContactPair, Manifold, Response, Impulse)>,
        ),
        (mut overlaps, new_overlaps): (Query<&Overlap, (), Despawns>, Spawner<(Overlap,)>),
        triggers: EventWriter<Trigger>,
    ) {
        let start = Instant::now();
        let mut items = Vec::with_capacity(moving.len() + held.len() + drifting.len() + statics.len());
        let v = |v: &Velocity| Vec2::new(v.x, v.y);
        moving.for_each(|row, (p, c, b, u)| items.push(item(row.entity(), p, c, *b, v(u), b.kind != STATIC, false)));
        held.for_each(|row, (p, c, b)| items.push(item(row.entity(), p, c, *b, Vec2::ZERO, false, false)));
        drifting.for_each(|row, (p, c, u)| items.push(item(row.entity(), p, c, Body::fixed(), v(u), false, false)));
        statics.for_each(|row, (p, c)| items.push(item(row.entity(), p, c, Body::fixed(), Vec2::ZERO, false, true)));
        let mut slots = Slots::of(items.iter().map(|i| i.entity));
        if sleep.asleep > 0 {
            self.wake_on_statics(sleep, &mut statics, &mut asleep, &mut resting, &slots);
        }
        // The storage's own order is the broadphase: pairs whose boxes, grown
        // by the speculative margin, meet, and one of which is awake and can
        // move. In entity order, lesser first, so what's found doesn't
        // depend on the order items were gathered in.
        let gathered = Instant::now();
        let near = engine_api::near_pairs(&(&moving, &held, &drifting), &(&statics, &asleep), narrow::MARGIN);
        let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(near.len());
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
        let paired = Instant::now();
        let index = |e: Entity| slots.get(e).expect("a paired collider");
        let arrives = |a: &Item, b: &Item| a.body.kind != STATIC || b.body.kind != STATIC;
        let collides = |a: &Item, b: &Item| {
            let meets = a.collider.mask & b.collider.layer != 0 && b.collider.mask & a.collider.layer != 0;
            // Contacts need something to push; a sensor needs something to
            // arrive. Static colliders overlapping (a goal line and the wall
            // across its end) are the level's shape, not an event.
            let pushes = a.body.kind == DYNAMIC || b.body.kind == DYNAMIC;
            meets && if a.collider.sensor || b.collider.sensor { arrives(a, b) } else { pushes }
        };
        let senses = |a: &Item, b: &Item| {
            (a.collider.senses & b.collider.layer != 0 || b.collider.senses & a.collider.layer != 0) && arrives(a, b)
        };
        // Neither end moves, and one sleeps: their contact is `Resting`,
        // kept as it was, and not looked for again.
        let rests = |a: &Item, b: &Item| (a.body.asleep || b.body.asleep) && !a.body.moves && !b.body.moves;

        let mut found: Vec<(ContactPair, Manifold, Response)> = Vec::new();
        // Each overlap, and whether it's a sensor's, which triggers.
        let mut overlapping: Vec<(Overlap, bool)> = Vec::new();
        let any_asleep = sleep.asleep > 0;
        // `sleeping`: something does, so a pair may rest.
        let mut test = |a: &Item, b: &Item, sleeping: bool| {
            let (collide, sense) = (collides(a, b), senses(a, b));
            let rest = sleeping && rests(a, b);
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
            if sleeping && collide && !sensor && !rest && (a.body.asleep || b.body.asleep) && woke() {
                sleep.wake(a.entity);
                sleep.wake(b.entity);
            } else if collide && !sensor && !rest {
                found.push((
                    ContactPair { a: a.entity, b: b.entity },
                    Manifold { nx: m.normal.x, ny: m.normal.y, depth: m.depth, pressed: false, was_pressed: false },
                    Response {
                        friction: a.body.friction.min(b.body.friction),
                        restitution: a.body.restitution.max(b.body.restitution),
                        disabled: false,
                    },
                ));
            }
        };
        // Split on whether anything sleeps, so a step with nothing asleep
        // tests nothing for it: the always-false tests cost about 7 µs of
        // 110 at 10 000 settled and at rest (2026-09-24, against the arrays'
        // narrowphase over six runs each), as a dead test once did in the
        // solve (physics.md, "What the ECS costs").
        let pair = |&(i, j): &(u32, u32)| (&items[i as usize], &items[j as usize]);
        if any_asleep {
            pairs.iter().map(pair).for_each(|(a, b)| test(a, b, true));
        } else {
            pairs.iter().map(pair).for_each(|(a, b)| test(a, b, false));
        }

        let narrowed = Instant::now();
        let key = |p: &ContactPair| (p.a, p.b);
        let mut next = 0;
        let spawn = |(pair, m, r): (ContactPair, Manifold, Response)| {
            new_contacts.spawn((pair, m, r, Impulse::default()));
        };
        // Contacts pressed last step that ended: an end asleep has lost
        // what it rested on or against, and wakes.
        let mut ended = Vec::new();
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
        for pair in ended {
            sleep.wake(pair.a);
            sleep.wake(pair.b);
        }

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
        t.narrowphase += (narrowed - paired).as_nanos() as u64;
        t.merge += nanos(narrowed);
        t.contacts += nanos(start);
    }

    /// Wakes sleeping bodies whose statics a game changed: moved one into
    /// or out from under them (a static written since the last step), or
    /// despawned one under them (when there are fewer or more statics than
    /// the last step). Statics and sleeping bodies are both passive, so the
    /// broadphase would never pair them again. `alive` is every awake
    /// collider and static.
    fn wake_on_statics(
        &mut self,
        sleep: &mut Sleepers,
        statics: &mut Query<(&Position, &Collider), Without<(Body, Velocity)>>,
        asleep: &mut Query<(&Position, &Collider, &Body), With<Asleep>>,
        resting: &mut Query<&ContactPair, With<Resting>>,
        alive: &Slots,
    ) {
        let mut moved = Vec::new();
        statics.for_each_written(self.since, |row, (p, c)| moved.push((row.entity(), p.bounds(Some(c)).grown(narrow::MARGIN))));
        let recount = std::mem::replace(&mut sleep.statics, statics.len()) != sleep.statics;
        if moved.is_empty() && !recount {
            return;
        }
        for &(_, around) in &moved {
            asleep.in_region(around, |row, _| sleep.wake(row.entity()));
        }
        let moved = Slots::of(moved.iter().map(|(e, _)| *e));
        let mut gone = Vec::new();
        resting.for_each(|_, pair| {
            for (end, other) in [(pair.a, pair.b), (pair.b, pair.a)] {
                if moved.get(end).is_some() {
                    sleep.wake(other);
                } else if recount && alive.get(end).is_none() && asleep.with(end, |_, _| ()).is_none() {
                    gone.push((end, other));
                }
            }
        });
        for (end, other) in gone {
            sleep.wake(other);
            // So its contacts leave `Resting`, and end in the next merge.
            sleep.woken.push(end);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn solve(
        &mut self,
        sleep: &mut Sleepers,
        _: &mut Cx,
        dt: Dt,
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
        let mut bodies = Vec::with_capacity(moving.len() + 1);
        let mut entities = Vec::with_capacity(moving.len());
        // Per body: whether it moves (isn't static), and its kind.
        let mut kinds: Vec<(bool, u8)> = Vec::with_capacity(moving.len() + 1);
        moving.for_each(|row, (body, v, _)| {
            entities.push(row.entity());
            let inv_mass = if body.kind == DYNAMIC { body.inv_mass } else { 0.0 };
            bodies.push(SolverBody { v: Vec2::new(v.x, v.y), inv_mass, pseudo: Vec2::ZERO });
            kinds.push((body.kind != STATIC, body.kind));
        });
        // Bodies with no velocity (statics) and sleeping ones all stand for
        // one immovable body at the end.
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        kinds.push((false, STATIC));
        let slots = Slots::of(entities.iter().copied());
        let index_of = |e: Entity| slots.get(e).unwrap_or(still);

        // In pair order, which storage keeps: the solve doesn't depend on
        // when each contact began. (History, 2026-09-24: contacts were
        // copied out of the walk and mapped after, which measured faster
        // until `for_each` walked slices.)
        let mut constraints: Vec<Constraint> = Vec::with_capacity(contacts.len());
        contacts.for_each_ordered(|_, (pair, m, r, j)| {
            if !r.disabled {
                constraints.push(Constraint {
                    a: index_of(pair.a),
                    b: index_of(pair.b),
                    normal: Vec2::new(m.nx, m.ny),
                    depth: m.depth,
                    friction: r.friction,
                    restitution: r.restitution,
                    jn: j.normal,
                    jt: j.tangent,
                    speed: 0.0,
                });
            }
        });
        let gathered = Instant::now();
        let config = config.single(|_, c| *c);
        solver::solve(&mut bodies, &mut constraints, dt);
        let after_solver = Instant::now();

        let mut i = 0;
        moving.for_each(|_, (body, mut v, mut p)| {
            let (b, moves) = (bodies[i], kinds[i].0);
            i += 1;
            if !moves {
                return;
            }
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
        });

        // Most bodies don't ask (the pile's none), and a failed lookup per
        // end of every contact was half of writing back.
        let mut asking = Vec::new();
        touching.for_each(|row, mut t| {
            *t = Touching::default();
            asking.push(row.entity());
        });
        let asking = Slots::of(asking.iter().copied());
        let mut solved = constraints.iter();
        let mut links = Vec::new();
        // Every contact's impulse and pressing are written, so pages are
        // stamped whole, as in the merge.
        contacts.for_each_ordered_page(|page, (pair, mut m, r, mut j)| {
            let (m, j) = (m.write_all(), j.write_all());
            for i in page.rows() {
                if r[i].disabled {
                    (m[i].pressed, j[i]) = (false, Impulse::default());
                    continue;
                }
                let k = solved.next().expect("a constraint per contact solved");
                j[i] = Impulse { normal: k.jn, tangent: k.jt };
                m[i].pressed = k.jn > 0.0 || m[i].depth >= 0.0;
                if !m[i].pressed {
                    continue;
                }
                let (pair, m) = (pair[i], m[i]);
                if config.is_some() {
                    links.push((pair.a, pair.b));
                }
                let n = Vec2::new(m.nx, m.ny);
                for (e, n) in [(pair.a, n), (pair.b, -n)] {
                    if asking.get(e).is_some() {
                        touching.with(e, |_, mut t| mark(&mut t, n));
                    }
                }
                if !m.was_pressed {
                    began.send(Contact { a: pair.a, b: pair.b, nx: m.nx, ny: m.ny, speed: k.speed });
                }
            }
        });
        if let Some(c) = config {
            self.fall_asleep(sleep, &c, dt, &entities, &bodies, &kinds, &links, &mut moving, &mut contacts);
            Self::move_woken(sleep, &mut sleeping, &mut resting);
        }
        // After every write of the step's, the stopping of bodies that fell
        // asleep included.
        self.since = sleeping.now();
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
                    "gravity {:.1} gather {:.1} broadphase {:.1} narrowphase {:.1} merge {:.1} solve_gather {:.1} solver {:.1} write_back {:.1}",
                    per(t.gravity),
                    per(t.gather),
                    per(t.broadphase),
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
