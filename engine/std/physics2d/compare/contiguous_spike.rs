//! SPIKE, not engine code: the physics measurements behind
//! docs/architecture/contiguous-columns.md. Kept as a bench target so its
//! numbers can be taken again; nothing depends on it.
//!
//! The question: the solve copies the awake bodies out of the world every
//! step because the world's storage can't be indexed as a plain array.
//! Pages are separate allocations, rows move every step, and derived values
//! aren't stored. If storage kept each table column in one block, holes and
//! all, could the solve work in the world's columns instead, and what would
//! that buy?
//!
//! The 2D mod runs a scene in the engine to a step (turning, as the
//! default). Then, between frames, harness systems over the engine's own
//! queries run the solve's whole system each way, on the same world:
//!
//! - **copy**: the copy as the mod would make it after working-sets.md's
//!   phase 1 and 3: one page walk over the awake bodies (every one of them
//!   turns in these scenes, checked), buffers kept, `Slots` built from the
//!   walk, contacts gathered through it; the lanes' own solve; written back
//!   by a page walk, each value only where it changed.
//! - **copy, entity order**: the same, renumbered in entity order before
//!   the solve.
//! - **in place**, each over bodies where a world would keep them. The
//!   index is each row's place in its table's column, found in one page
//!   walk: `(page * 16 + row)`, pages numbered across tables, holes and
//!   empty pages included, which is what a column kept in one block per
//!   table, tables one after another in a reserved range, would be indexed
//!   by. Velocities and spins are read and written there by the lanes' own
//!   kernels (`contiguous_lanes.rs`), what only the step has (how far each
//!   body moved and turned, its gravity) is scratch by the same index, and
//!   positions and rotations are written back from the scratch:
//!   - **contiguous**: each column one block, one index space;
//!   - **contiguous, derived stored**: the same, with the inverse mass by
//!     kind, the step's gravity and the inverse inertia read from blocks
//!     that keep them, rather than worked out from `Body` and `Collider`;
//!   - **contiguous by table**: a block per table, the index's high bits
//!     the table: what one block per table column costs over one range;
//!   - **the world's pages**: in the world's own `Velocity` and `Spin`
//!     pages, by page pointer, as storage is today;
//!   - **a state column**: velocity, spin and the step's scratch in one
//!     32-byte value a row, as if the world kept the solver's `State` as a
//!     component: layout as the copy has it, place as the world has it.
//! - **copy through the in-place driver**: the copy's own dense states,
//!   solved through `contiguous_lanes.rs`: checks that the driver costs
//!   what the lanes' solve does, so the rows above measure the storage.
//!
//! The contiguous blocks are filled from the world's pages before each
//! solve, untimed: they stand for storage that already keeps them so. Each
//! way's solver output (every body's velocity, spin, motion and turn, every
//! contact's impulses) is checked bit for bit against the copy's, by
//! entity, and so is the world after its write-back. The world is put back
//! after each way, untimed.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:contiguous_spike
//!
//! `REPS` (21), the median of each; `ONLY=falling`, `settled` or `pyramid`;
//! `THREADS` also times the passes across that many kept threads: the
//! copy's, in walk and entity order, and in place over atomic views of the
//! contiguous columns and over a state column (`across`); `ACROSS_ONLY`
//! times only those.

#[allow(dead_code)]
#[path = "contiguous_spike_arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[path = "contiguous_spike_narrow.rs"]
mod narrow;
#[allow(dead_code)]
#[path = "contiguous_spike_pool.rs"]
mod pool;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
#[allow(dead_code)]
#[path = "contiguous_spike_solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "contiguous_spike_split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::collections::HashMap;
use std::ops::Range;
use std::sync::atomic::AtomicU32;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Entity, Executor, Query, Without, Workers, World};
use physics2d::{
    Asleep, Body, Collider, ContactPair, ContactPoints, DYNAMIC, Gravity, Impulse, KINEMATIC, Manifold, Position, Response, Resting, Rot,
    Rotation, STATIC, Spin, Tuning, Vec2, Velocity,
};
pub use sim::{Dyn, Sim};

use scene::Scene;
use solver::lanes::{Place, Source};
use solver::{Constraint, ContactPoint, Params, Points, SolverBody, Spinning};

const DT: f32 = 1.0 / 60.0;
/// Rows a spatial page holds (`engine_ecs::spatial::SPATIAL_PAGE_ROWS`),
/// checked against the bodies' tables.
const ROWS: usize = 16;
/// Where the table is in a by-table index: above any table's rows (the
/// pile's one table of bodies has about 17 000, holes included; checked).
/// As small as that allows, since the solve's scratch is by index.
const TABLE_SHIFT: u32 = 15;
/// The lanes' width as the default `Tuning` solves (`Wide::Colored(4)`),
/// checked.
const N: usize = 4;

/// `Slots` as lib.rs has it: entities to places, by entity index.
#[derive(Clone, Default)]
struct Slots(Vec<(u32, u32)>);

impl Slots {
    fn of(entities: impl Iterator<Item = (Entity, u32)> + Clone) -> Slots {
        let len = entities.clone().map(|(e, _)| e.index as usize + 1).max().unwrap_or(0);
        let mut slots = vec![(u32::MAX, u32::MAX); len];
        for (e, k) in entities {
            slots[e.index as usize] = (e.generation, k);
        }
        Slots(slots)
    }

    #[inline]
    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }
}

fn solver_body(body: &Body, v: Vec2, g: Vec2) -> SolverBody {
    // As lib.rs's `solve`: the gravity `integrate_velocities` added, for
    // the solver to spread over its substeps.
    let (inv_mass, g) = if body.kind == DYNAMIC { (body.inv_mass, g) } else { (0.0, Vec2::ZERO) };
    let gravity = Vec2::new(g.x * body.gravity_scale * DT, g.y * body.gravity_scale * DT);
    SolverBody::new(v, inv_mass, gravity)
}

/// A body's turn rate and inverse inertia as the solve gathers them: none
/// for a static one, which the copy leaves out of `spinning`.
#[inline(always)]
fn ang(body: &Body, c: &Collider, w: f32) -> (f32, f32) {
    match body.kind {
        DYNAMIC => (w, body.inv_mass * c.inertia_per_mass()),
        KINEMATIC => (w, 0.0),
        _ => (0.0, 0.0),
    }
}

fn constraint(
    index: impl Fn(Entity) -> u32,
    pair: &ContactPair,
    m: &Manifold,
    r: &Response,
    j: &Impulse,
    cp: &ContactPoints,
    points: &mut Vec<Points>,
) -> Constraint {
    let c = Constraint {
        a: index(pair.a),
        b: index(pair.b),
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
}

type All<'w, 'a> = Query<'w, (&'a Body, &'a Collider, &'a mut Velocity, &'a mut Position, &'a mut Rotation, &'a mut Spin), Without<Asleep>>;
type Contacts<'w, 'a> = Query<'w, (&'a ContactPair, &'a Manifold, &'a Response, &'a Impulse, &'a ContactPoints), Without<Resting>>;

fn us(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1e6
}

// ---- Where bodies are kept ----

/// A column of `T` by the in-place index.
trait Col<T> {
    fn get(&self, i: usize) -> T;
    fn set(&mut self, i: usize, v: T);
}

/// One block, one index space: `(page * 16 + row)`, pages across tables.
struct Flat<T>(Vec<T>);

impl<T: Copy> Col<T> for Flat<T> {
    #[inline(always)]
    fn get(&self, i: usize) -> T {
        self.0[i]
    }
    #[inline(always)]
    fn set(&mut self, i: usize, v: T) {
        self.0[i] = v;
    }
}

/// A block per table, by a pointer per table, as storage would hand a
/// system its tables' columns: the index's bits from `TABLE_SHIFT` up are
/// the table, the rest the row in its block.
struct Tabled<T> {
    /// Owns the blocks, which never grow while `bases` points into them.
    _blocks: Vec<Vec<T>>,
    bases: Vec<*mut T>,
}

impl<T: Copy> Col<T> for Tabled<T> {
    #[inline(always)]
    fn get(&self, i: usize) -> T {
        // SAFETY: `bases[t]` is block `t`'s buffer, alive and unresized as
        // long as `self`; the index's low bits are a row within it (made by
        // `Blocks::by_table` from a row that exists).
        unsafe { *self.bases[i >> TABLE_SHIFT].add(i & ((1 << TABLE_SHIFT) - 1)) }
    }
    #[inline(always)]
    fn set(&mut self, i: usize, v: T) {
        // SAFETY: as `get`; the blocks are this column's alone.
        unsafe { *self.bases[i >> TABLE_SHIFT].add(i & ((1 << TABLE_SHIFT) - 1)) = v }
    }
}

/// Pages of 16 rows, each its own allocation, by a pointer per page: the
/// world's own pages, as `ColumnMut::write_all` and `&[T]` hand them over.
struct Paged<T>(Vec<*mut T>);

impl<T: Copy> Col<T> for Paged<T> {
    #[inline(always)]
    fn get(&self, i: usize) -> T {
        // SAFETY: the pointers are to pages of the world whose guards the
        // in-place system holds for the whole solve (or to the stand-in
        // page it owns), unmoved meanwhile; `i` is the index of a row that
        // exists, made from the same walk (or the stand-in's).
        unsafe { *self.0[i / ROWS].add(i % ROWS) }
    }
    #[inline(always)]
    fn set(&mut self, i: usize, v: T) {
        // SAFETY: as `get`; written only through pages taken for writing
        // (`write_all`), and nothing else reads or writes them meanwhile.
        unsafe { *self.0[i / ROWS].add(i % ROWS) = v }
    }
}

/// Derived values a world could keep by the same index, refreshed when
/// `Body`, `Collider` or gravity change (not measured: what keeping them
/// costs is working-sets.md's (f)).
struct Derived {
    inv_mass: Vec<f32>,
    gravity: Vec<Vec2>,
    inertia: Vec<f32>,
    /// Dynamic or kinematic: the bodies whose spin the solve reads.
    turns: Vec<bool>,
}

/// Bodies in place: velocity and spin in the world's columns, `Body` and
/// `Collider` read there, the step's motion and turn in scratch.
struct InWorld<V, W, B, C> {
    v: V,
    w: W,
    body: B,
    collider: C,
    derived: Option<Derived>,
    moved: Vec<Vec2>,
    turned: Vec<Rot>,
    g: Vec2,
}

impl<V: Col<Velocity>, W: Col<Spin>, B: Col<Body>, C: Col<Collider>> Place for InWorld<V, W, B, C> {
    #[inline(always)]
    fn v(&self, i: usize) -> Vec2 {
        let v = self.v.get(i);
        Vec2::new(v.x, v.y)
    }
    #[inline(always)]
    fn set_v(&mut self, i: usize, v: Vec2) {
        self.v.set(i, Velocity { x: v.x, y: v.y });
    }
    #[inline(always)]
    fn w(&self, i: usize) -> f32 {
        self.w.get(i).w
    }
    #[inline(always)]
    fn set_w(&mut self, i: usize, w: f32) {
        self.w.set(i, Spin { w });
    }
    #[inline(always)]
    fn moved(&self, i: usize) -> Vec2 {
        self.moved[i]
    }
    #[inline(always)]
    fn set_moved(&mut self, i: usize, m: Vec2) {
        self.moved[i] = m;
    }
    #[inline(always)]
    fn turned(&self, i: usize) -> Rot {
        self.turned[i]
    }
    #[inline(always)]
    fn set_turned(&mut self, i: usize, q: Rot) {
        self.turned[i] = q;
    }
}

impl<V: Col<Velocity>, W: Col<Spin>, B: Col<Body>, C: Col<Collider>> Source for InWorld<V, W, B, C> {
    #[inline(always)]
    fn body(&self, i: usize) -> SolverBody {
        match &self.derived {
            Some(d) => SolverBody::new(Place::v(self, i), d.inv_mass[i], d.gravity[i]),
            None => solver_body(&self.body.get(i), Place::v(self, i), self.g),
        }
    }
    #[inline(always)]
    fn ang(&self, i: usize) -> (f32, f32) {
        match &self.derived {
            Some(d) => (if d.turns[i] { Place::w(self, i) } else { 0.0 }, d.inertia[i]),
            None => ang(&self.body.get(i), &self.collider.get(i), Place::w(self, i)),
        }
    }
}

/// The solver's `State` as a column: what a pass reads of a body, in one
/// 32-byte line.
#[derive(Clone, Copy)]
#[repr(C, align(32))]
struct St {
    v: Vec2,
    w: f32,
    moved: Vec2,
    turned: Rot,
}

impl St {
    const STILL: St = St { v: Vec2::ZERO, w: 0.0, moved: Vec2::ZERO, turned: Rot::IDENTITY };
}

/// Bodies as a column of `St`: by the in-place index, `Body` and `Collider`
/// read beside it; or, for the driver's own check, by the copy's index,
/// with the copy's bodies as the source.
struct StateCol {
    s: Vec<St>,
    body: Flat<Body>,
    collider: Flat<Collider>,
    copy: Option<(Vec<SolverBody>, Vec<(f32, f32)>)>,
    g: Vec2,
}

impl Place for StateCol {
    #[inline(always)]
    fn v(&self, i: usize) -> Vec2 {
        self.s[i].v
    }
    #[inline(always)]
    fn set_v(&mut self, i: usize, v: Vec2) {
        self.s[i].v = v;
    }
    #[inline(always)]
    fn w(&self, i: usize) -> f32 {
        self.s[i].w
    }
    #[inline(always)]
    fn set_w(&mut self, i: usize, w: f32) {
        self.s[i].w = w;
    }
    #[inline(always)]
    fn moved(&self, i: usize) -> Vec2 {
        self.s[i].moved
    }
    #[inline(always)]
    fn set_moved(&mut self, i: usize, m: Vec2) {
        self.s[i].moved = m;
    }
    #[inline(always)]
    fn turned(&self, i: usize) -> Rot {
        self.s[i].turned
    }
    #[inline(always)]
    fn set_turned(&mut self, i: usize, q: Rot) {
        self.s[i].turned = q;
    }
}

impl Source for StateCol {
    #[inline(always)]
    fn body(&self, i: usize) -> SolverBody {
        match &self.copy {
            Some((b, _)) => SolverBody { moved: Vec2::ZERO, ..b[i] },
            None => solver_body(&self.body.get(i), self.s[i].v, self.g),
        }
    }
    #[inline(always)]
    fn ang(&self, i: usize) -> (f32, f32) {
        match &self.copy {
            Some((_, a)) => a[i],
            None => ang(&self.body.get(i), &self.collider.get(i), self.s[i].w),
        }
    }
}

// ---- The ways ----

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Way {
    Copy,
    CopyEntity,
    Contiguous,
    ContiguousDerived,
    ByTable,
    Pages,
    StateColumn,
    Driver,
}

const WAYS: [Way; 8] =
    [Way::Copy, Way::CopyEntity, Way::Contiguous, Way::ContiguousDerived, Way::ByTable, Way::Pages, Way::StateColumn, Way::Driver];

impl Way {
    fn name(self) -> &'static str {
        match self {
            Way::Copy => "copy (one page walk, buffers kept)",
            Way::CopyEntity => "copy, renumbered in entity order",
            Way::Contiguous => "in place: contiguous columns",
            Way::ContiguousDerived => "in place: contiguous, derived stored",
            Way::ByTable => "in place: contiguous, a block per table",
            Way::Pages => "in place: the world's pages",
            Way::StateColumn => "in place: a state column (32 B a row)",
            Way::Driver => "copy through the in-place driver",
        }
    }

    fn copies(self) -> bool {
        matches!(self, Way::Copy | Way::CopyEntity | Way::Driver)
    }
}

/// Microseconds of each part, for one way, one rep.
#[derive(Clone, Copy, Default, Debug)]
struct Times {
    /// The copy: the body walk into dense arrays. In place: the walk that
    /// finds each row's index (and the old spins the write-back needs).
    walk: f64,
    /// The copy: `Slots` (and the renumbering). In place: the map from
    /// entity to index.
    index: f64,
    contacts: f64,
    prepare: f64,
    passes: f64,
    finish: f64,
    write_back: f64,
}

impl Times {
    fn total(&self) -> f64 {
        self.walk + self.index + self.contacts + self.prepare + self.passes + self.finish + self.write_back
    }
}

/// A way's output, by entity: velocity, spin, motion and turn as bits, and
/// each contact's impulses and points in pair order.
type Outcome = (HashMap<Entity, [u32; 7]>, Vec<u32>);

fn contacts_out(constraints: &[Constraint], points: &[Points]) -> Vec<u32> {
    let mut out = Vec::new();
    for c in constraints {
        out.extend([c.jn, c.jt, c.speed].iter().map(|x| x.to_bits()));
    }
    for p in points {
        for q in &p.point {
            out.extend([q.jn, q.jt].iter().map(|x| x.to_bits()));
        }
        out.push(p.solved as u32);
    }
    out
}

fn bits(v: Vec2, w: f32, m: Vec2, q: Rot) -> [u32; 7] {
    [v.x, v.y, w, m.x, m.y, q.c, q.s].map(f32::to_bits)
}

/// What every way shares with the systems: the way, its times and output.
struct Spike {
    way: Way,
    /// The world the systems run on, as an address: see `in_place_way`.
    world: usize,
    g: Vec2,
    params: Params,
    times: Times,
    out: Option<Outcome>,
    /// Kept between reps, as a mod keeps them in its `Transient`.
    buffers: Buffers,
    /// The contiguous blocks a way solves in, filled untimed before it.
    blocks: Option<Blocks>,
    saved: HashMap<Entity, (Velocity, Position, Rotation, Spin)>,
    /// Bodies, turning bodies and contacts, for the report.
    counts: (usize, usize, usize, usize),
}

#[derive(Default)]
struct Buffers {
    entities: Vec<Entity>,
    bodies: Vec<SolverBody>,
    kinds: Vec<(bool, u8)>,
    spinning: Vec<Spinning>,
    constraints: Vec<Constraint>,
    points: Vec<Points>,
    // In place: each row's index (by table, for `ByTable`) and flat index
    // in walk order, entities to indices, the live rows by page.
    index: Vec<u32>,
    flat: Vec<u32>,
    map: Vec<(Entity, u32)>,
    live: Vec<Range<usize>>,
    turning: Vec<u32>,
    angle: Vec<f32>,
    w0: Vec<f32>,
}

/// The world's body columns copied into blocks by the in-place index, and
/// where each table's pages start in it.
struct Blocks {
    v: Vec<Velocity>,
    w: Vec<Spin>,
    body: Vec<Body>,
    collider: Vec<Collider>,
    /// By table id: its first page's number, or `u32::MAX`.
    page_base: Vec<u32>,
    /// By page: its table's place among the bodies' tables (the stand-in's
    /// last), and the table's first page, for the by-table index.
    page_table: Vec<(u32, u32)>,
    /// Pages in all, the stand-in's included (the last).
    pages: usize,
}

type Spikes = Arc<Mutex<Spike>>;

/// The bodies' tables in world order, each with its pages: what the
/// in-place index numbers. The stand-in for statics, and `nowhere`, get a
/// page of their own after them.
fn blocks(world: &World) -> Blocks {
    let ids = ["physics2d::Body", "physics2d::Velocity", "physics2d::Position", "physics2d::Spin", "physics2d::Collider"]
        .map(|n| world.id(n).unwrap());
    let mut b = Blocks {
        v: Vec::new(),
        w: Vec::new(),
        body: Vec::new(),
        collider: Vec::new(),
        page_base: Vec::new(),
        page_table: Vec::new(),
        pages: 0,
    };
    fn read(
        c: &std::sync::RwLock<Vec<engine_ecs::erased::ErasedColumn>>,
    ) -> std::sync::RwLockReadGuard<'_, Vec<engine_ecs::erased::ErasedColumn>> {
        c.read().unwrap_or_else(std::sync::PoisonError::into_inner)
    }
    let mut tables = 0;
    for t in world.tables() {
        let i = t.id.0 as usize;
        if b.page_base.len() <= i {
            b.page_base.resize(i + 1, u32::MAX);
        }
        let Some(cols) = ids.iter().map(|&c| t.column_index(c)).collect::<Option<Vec<_>>>() else { continue };
        assert_eq!(t.page_rows, ROWS, "bodies are in a spatial table");
        let rows = t.rows.read().unwrap();
        b.page_base[i] = b.pages as u32;
        b.page_table.extend(std::iter::repeat_n((tables, b.pages as u32), rows.len()));
        tables += 1;
        let (body, v, w, collider) =
            (read(&t.columns[cols[0]]), read(&t.columns[cols[1]]), read(&t.columns[cols[3]]), read(&t.columns[cols[4]]));
        for (p, es) in rows.iter().enumerate() {
            let fill = |n: usize| ROWS - n;
            b.v.extend_from_slice(v[p].as_slice::<Velocity>());
            b.v.extend(std::iter::repeat_n(Velocity { x: 0.0, y: 0.0 }, fill(es.len())));
            b.w.extend_from_slice(w[p].as_slice::<Spin>());
            b.w.extend(std::iter::repeat_n(Spin { w: 0.0 }, fill(es.len())));
            b.body.extend_from_slice(body[p].as_slice::<Body>());
            b.body.extend(std::iter::repeat_n(Body::fixed(), fill(es.len())));
            b.collider.extend_from_slice(collider[p].as_slice::<Collider>());
            b.collider.extend(std::iter::repeat_n(Collider::circle(0.5), fill(es.len())));
        }
        b.pages += rows.len();
    }
    // The stand-in for statics (row 0) and `nowhere` (row 1): still, fixed.
    b.v.extend(std::iter::repeat_n(Velocity { x: 0.0, y: 0.0 }, ROWS));
    b.w.extend(std::iter::repeat_n(Spin { w: 0.0 }, ROWS));
    b.body.extend(std::iter::repeat_n(Body::fixed(), ROWS));
    b.collider.extend(std::iter::repeat_n(Collider::circle(0.5), ROWS));
    b.page_table.push((tables, b.pages as u32));
    b.pages += 1;
    b
}

impl Blocks {
    fn still(&self) -> u32 {
        ((self.pages - 1) * ROWS) as u32
    }

    /// The by-table index of a flat one: the table's place in the high bits,
    /// the row within its block below.
    #[inline(always)]
    fn by_table(&self, flat: u32) -> u32 {
        let (t, base) = self.page_table[flat as usize / ROWS];
        (t << TABLE_SHIFT) | (flat - base * ROWS as u32)
    }

    fn tabled<T: Copy>(&self, flat: &[T]) -> Tabled<T> {
        let mut bases: Vec<u32> = self.page_base.iter().copied().filter(|b| *b != u32::MAX).collect();
        bases.sort();
        bases.push((self.pages - 1) as u32);
        let mut out = Vec::new();
        for (k, &b) in bases.iter().enumerate() {
            let end = bases.get(k + 1).map_or(self.pages, |&e| e as usize);
            assert!((end - b as usize) * ROWS <= 1 << TABLE_SHIFT, "a table's rows fit below the table's bits");
            out.push(flat[b as usize * ROWS..end * ROWS].to_vec());
        }
        let bases = out.iter_mut().map(|b| b.as_mut_ptr()).collect();
        Tabled { _blocks: out, bases }
    }

    /// Tables of bodies, the stand-in's counted as one.
    fn tables(&self) -> usize {
        self.page_base.iter().filter(|b| **b != u32::MAX).count() + 1
    }

    fn derived(&self, g: Vec2) -> Derived {
        let n = self.body.len();
        let (mut inv_mass, mut gravity, mut inertia) = (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
        for (b, c) in self.body.iter().zip(&self.collider) {
            let s = solver_body(b, Vec2::ZERO, g);
            inv_mass.push(s.inv_mass);
            gravity.push(s.gravity);
            inertia.push(ang(b, c, 0.0).1);
        }
        let turns = self.body.iter().map(|b| b.kind == DYNAMIC || b.kind == KINEMATIC).collect();
        Derived { inv_mass, gravity, inertia, turns }
    }
}

/// The copy: the bodies in one page walk, `Slots`, the contacts; then the
/// lanes' solve on it; then the write-back. As lib.rs, but one walk (every
/// awake body turns here) and buffers kept.
fn copy_way(s: &mut Spike, all: &mut All, contacts: &mut Contacts) {
    let (way, g, params) = (s.way, s.g, s.params);
    let mut t = Times::default();
    let b = &mut s.buffers;
    let start = Instant::now();
    b.entities.clear();
    b.bodies.clear();
    b.kinds.clear();
    b.spinning.clear();
    {
        let Buffers { entities, bodies, kinds, spinning, .. } = b;
        all.for_each_page(|page, (body, c, v, _, _, spin)| {
            let (v, spin) = (v.as_slice(), spin.as_slice());
            for (r, &e) in page.entities().iter().enumerate() {
                let k = entities.len() as u32;
                entities.push(e);
                let sb = solver_body(&body[r], Vec2::new(v[r].x, v[r].y), g);
                bodies.push(sb);
                kinds.push((body[r].kind != STATIC, body[r].kind));
                match body[r].kind {
                    DYNAMIC => spinning.push(Spinning::new(k, spin[r].w, sb.inv_mass * c[r].inertia_per_mass())),
                    KINEMATIC => spinning.push(Spinning::new(k, spin[r].w, 0.0)),
                    _ => assert_eq!(spin[r].w.to_bits(), 0, "a static body that spins would be read otherwise in place"),
                }
            }
        });
    }
    b.bodies.push(SolverBody::default());
    b.kinds.push((false, STATIC));
    t.walk = us(start);
    let at = Instant::now();
    let mut slots = Slots::of(b.entities.iter().enumerate().map(|(k, &e)| (e, k as u32)));
    let mut walk_slot = Vec::new();
    if way == Way::CopyEntity {
        // In entity order: the index scanned by entity index gives each its
        // rank, and the gathered values move to it (working-sets.md's (e)).
        let n = b.entities.len();
        walk_slot.resize(n, 0u32);
        let (mut entities, mut bodies, mut kinds) = (Vec::with_capacity(n), Vec::with_capacity(n + 1), Vec::with_capacity(n + 1));
        for (k, at) in slots.0.iter_mut().filter(|(_, i)| *i != u32::MAX).enumerate() {
            let i = at.1 as usize;
            walk_slot[i] = k as u32;
            entities.push(b.entities[i]);
            bodies.push(b.bodies[i]);
            kinds.push(b.kinds[i]);
            at.1 = k as u32;
        }
        bodies.push(SolverBody::default());
        kinds.push((false, STATIC));
        for sp in b.spinning.iter_mut() {
            sp.body = walk_slot[sp.body as usize];
        }
        (b.entities, b.bodies, b.kinds) = (entities, bodies, kinds);
    }
    t.index = us(at);
    let at = Instant::now();
    gather_contacts(contacts, |e| slots.get(e).unwrap_or(b.entities.len() as u32), &mut b.constraints, &mut b.points);
    t.contacts = us(at);
    if way == Way::Driver {
        // The copy's own states, through the in-place driver.
        let n = b.bodies.len();
        let mut a = vec![(0.0, 0.0); n];
        for sp in &b.spinning {
            a[sp.body as usize] = (sp.w, sp.inv_inertia);
        }
        let mut st: Vec<St> = b.bodies.iter().map(|x| St { v: x.v, ..St::STILL }).collect();
        for sp in &b.spinning {
            st[sp.body as usize].w = sp.w;
        }
        st.push(St::STILL);
        let mut col = StateCol { s: st, body: Flat(Vec::new()), collider: Flat(Vec::new()), copy: Some((b.bodies.clone(), a)), g };
        // Every body but the stand-in, one run.
        let live = [Range { start: 0, end: n - 1 }];
        let turning: Vec<u32> = b.spinning.iter().map(|sp| sp.body).collect();
        let mut angle = vec![0.0; turning.len()];
        let ts = solve_in(&params, &mut col, (n + 1, &live, n as u32), (&turning, &mut angle), (&mut b.constraints, &mut b.points));
        (t.prepare, t.passes, t.finish) = (ts[0], ts[1], ts[2]);
        for (k, x) in b.bodies.iter_mut().enumerate() {
            (x.v, x.moved) = (col.s[k].v, col.s[k].moved);
        }
        for (sp, angle) in b.spinning.iter_mut().zip(angle) {
            let st = col.s[sp.body as usize];
            (sp.w, sp.turned, sp.angle) = (st.w, st.turned, angle);
        }
    } else {
        let ts = solver::lanes::solve_timed::<N>(&params, (&mut b.bodies, &mut b.spinning), &mut b.constraints, &mut b.points, DT);
        (t.prepare, t.passes, t.finish) = (ts[0], ts[1], ts[2]);
    }
    let mut out: HashMap<Entity, [u32; 7]> = HashMap::new();
    let mut w = vec![0.0f32; b.bodies.len()];
    let mut q = vec![Rot::IDENTITY; b.bodies.len()];
    for sp in &b.spinning {
        (w[sp.body as usize], q[sp.body as usize]) = (sp.w, sp.turned);
    }
    for (k, &e) in b.entities.iter().enumerate() {
        out.insert(e, bits(b.bodies[k].v, w[k], b.bodies[k].moved, q[k]));
    }
    let start = Instant::now();
    // By page, each value written only where it changed (working-sets.md's
    // `scatter_one`).
    let mut k = 0;
    let mut each = b.spinning.iter();
    let order = |k: usize| if way == Way::CopyEntity { walk_slot[k] as usize } else { k };
    all.for_each_page(|page, (body, _, mut v, mut p, mut q, mut spin)| {
        for r in page.rows() {
            let i = order(k);
            let (sb, moves) = (&b.bodies[i], b.kinds[i].0);
            k += 1;
            if body[r].kind == DYNAMIC || body[r].kind == KINEMATIC {
                // `spinning` stays in walk order when renumbered (only its
                // bodies' indices change), so it pairs with this walk.
                let sp = each.next().expect("a spinning body per one gathered");
                let (q0, w0) = (q.as_slice()[r], spin.as_slice()[r].w);
                if let Some(to) = sp.turned_from(q0.rot(), w0).filter(|_| moves) {
                    if w0.to_bits() != sp.w.to_bits() {
                        spin.set(r, Spin { w: sp.w });
                    }
                    if (to.c.to_bits(), to.s.to_bits()) != (q0.c.to_bits(), q0.s.to_bits()) {
                        q.set(r, Rotation::of(to));
                    }
                }
            }
            if moves {
                v.set(r, Velocity { x: sb.v.x, y: sb.v.y });
                let step = if body[r].kind == KINEMATIC { sb.v * DT } else { sb.displacement(DT) };
                let p0 = p.as_slice()[r];
                let to = (p0.x + step.x, p0.y + step.y);
                if (to.0.to_bits(), to.1.to_bits()) != (p0.x.to_bits(), p0.y.to_bits()) {
                    p.set(r, Position { x: to.0, y: to.1 });
                }
            }
        }
    });
    t.write_back = us(start);
    s.counts = (b.entities.len(), b.spinning.len(), b.constraints.len(), b.points.len());
    s.out = Some((out, contacts_out(&b.constraints, &b.points)));
    s.times = t;
}

fn gather_contacts(contacts: &mut Contacts, index_of: impl Fn(Entity) -> u32, constraints: &mut Vec<Constraint>, points: &mut Vec<Points>) {
    constraints.clear();
    points.clear();
    contacts.for_each_ordered_page(|page, (pair, m, r, j, cp)| {
        for i in page.rows() {
            if !r[i].disabled {
                constraints.push(constraint(&index_of, &pair[i], &m[i], &r[i], &j[i], &cp[i], points));
            }
        }
    });
}

/// `setup_in`, `run_in` and `finish_in`, timed: µs of each.
fn solve_in<P: Place + Source>(
    params: &Params,
    place: &mut P,
    (len, live, nowhere): (usize, &[Range<usize>], u32),
    (turning, angle): (&[u32], &mut [f32]),
    (constraints, points): (&mut [Constraint], &mut [Points]),
) -> [f64; 3] {
    let t0 = Instant::now();
    let mut p = solver::lanes::setup_in::<N, _>(params, place, (len, live, nowhere), constraints, points, DT);
    let t1 = Instant::now();
    solver::lanes::run_in(&mut p, params, place, live, (turning, angle));
    let t2 = Instant::now();
    solver::lanes::finish_in(&p, params, constraints, points);
    let t3 = Instant::now();
    let us = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1e6;
    [us(t0, t1), us(t1, t2), us(t2, t3)]
}

/// In place: the walk that finds each row's index, the contacts through
/// it, the solve in the way's columns, and the write-back of positions and
/// rotations from the scratch.
fn in_place_way(s: &mut Spike, all: &mut All, contacts: &mut Contacts) {
    let (way, g, params) = (s.way, s.g, s.params);
    // SAFETY: `measure` sets this to the world these systems run on, which
    // outlives every run of them (the harness wants `'static` systems, so
    // the world can't be captured as a reference).
    let world: &World = unsafe { &*(s.world as *const World) };
    let blocks = s.blocks.take().expect("blocks filled before an in-place way");
    let mut t = Times::default();
    let b = &mut s.buffers;
    let by_table = way == Way::ByTable;
    let start = Instant::now();
    // The walk: each row's index from its page's place, found once a page
    // from its first row's location; the map from entity to index; which
    // rows turn; the spins as they were, which the write-back's rule for
    // rotations reads. Velocity and spin pages are taken for writing as
    // they're walked, every way (stamped written, as the solve writes every
    // row of them, which storage would stamp wherever the values were); in
    // the world's own pages, that's how the solve reaches them, and nothing
    // else takes them until it's done.
    b.index.clear();
    b.flat.clear();
    b.live.clear();
    b.turning.clear();
    b.map.clear();
    b.w0.clear();
    b.w0.resize(blocks.pages * ROWS, 0.0);
    let (mut vp, mut wp, mut bp, mut cp) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    {
        let Buffers { index, flat, live, turning, w0, map, .. } = b;
        all.for_each_page(|page, (body, c, mut v, _, _, mut spin)| {
            let es = page.entities();
            let at = world.entities.location(es[0]).expect("a placed row");
            let base = (blocks.page_base[at.table.0 as usize] as usize + at.page as usize) * ROWS;
            let spins = spin.as_slice();
            for (r, &e) in es.iter().enumerate() {
                let i = (base + r) as u32;
                let k = if by_table { blocks.by_table(i) } else { i };
                index.push(k);
                flat.push(i);
                map.push((e, k));
                w0[i as usize] = spins[r].w;
                if body[r].kind == DYNAMIC || body[r].kind == KINEMATIC {
                    turning.push(k);
                }
            }
            live.push(base..base + es.len());
            let (v, spin) = (v.write_all().as_mut_ptr(), spin.write_all().as_mut_ptr());
            if way == Way::Pages {
                // By the page's number in the index: empty pages and the
                // stand-in's are filled in below.
                let p = base / ROWS;
                if vp.len() <= p {
                    vp.resize(p + 1, std::ptr::null_mut());
                    wp.resize(p + 1, std::ptr::null_mut());
                    bp.resize(p + 1, std::ptr::null_mut());
                    cp.resize(p + 1, std::ptr::null_mut());
                }
                (vp[p], wp[p]) = (v, spin);
                bp[p] = body.as_ptr() as *mut Body;
                cp[p] = c.as_ptr() as *mut Collider;
            }
        });
    }
    t.walk = us(start);
    let at = Instant::now();
    let slots = Slots::of(b.map.iter().copied());
    t.index = us(at);
    let still = if by_table { blocks.by_table(blocks.still()) } else { blocks.still() };
    let at = Instant::now();
    gather_contacts(contacts, |e| slots.get(e).unwrap_or(still), &mut b.constraints, &mut b.points);
    t.contacts = us(at);

    // The live rows by the way's index, for the solve's body loops: a page's
    // rows are consecutive by table too, so its run maps whole.
    let live: Vec<Range<usize>> = if by_table {
        b.live
            .iter()
            .map(|r| {
                let k = blocks.by_table(r.start as u32) as usize;
                k..k + r.len()
            })
            .collect()
    } else {
        b.live.clone()
    };
    let len = if by_table { blocks.tables() << TABLE_SHIFT } else { blocks.pages * ROWS };
    let mut stand_in = (
        vec![Velocity { x: 0.0, y: 0.0 }; ROWS],
        vec![Spin { w: 0.0 }; ROWS],
        vec![Body::fixed(); ROWS],
        vec![Collider::circle(0.5); ROWS],
    );
    let mut cx = InPlaceCx { params: &params, b, live: &live, len, still, all };
    // The scratch a way's solve keeps by index: made with the prepare.
    let scratch = |n: usize| (vec![Vec2::ZERO; n], vec![Rot::IDENTITY; n]);
    let out = match way {
        Way::Contiguous | Way::ContiguousDerived => {
            let derived = (way == Way::ContiguousDerived).then(|| blocks.derived(g));
            let (v, w, body, collider) =
                (Flat(blocks.v.clone()), Flat(blocks.w.clone()), Flat(blocks.body.clone()), Flat(blocks.collider.clone()));
            cx.run(&mut t, false, |n| {
                let (moved, turned) = scratch(n);
                InWorld { v, w, body, collider, derived, moved, turned, g }
            })
        }
        Way::ByTable => {
            let (v, w, body, collider) =
                (blocks.tabled(&blocks.v), blocks.tabled(&blocks.w), blocks.tabled(&blocks.body), blocks.tabled(&blocks.collider));
            cx.run(&mut t, false, |n| {
                let (moved, turned) = scratch(n);
                InWorld { v, w, body, collider, derived: None, moved, turned, g }
            })
        }
        Way::Pages => {
            // Pages the walk didn't reach (empty ones) and the stand-in's
            // point at a page of still bodies, which no index reaches but
            // the stand-in's and `nowhere`.
            vp.resize(blocks.pages, std::ptr::null_mut());
            wp.resize(blocks.pages, std::ptr::null_mut());
            bp.resize(blocks.pages, std::ptr::null_mut());
            cp.resize(blocks.pages, std::ptr::null_mut());
            for p in 0..blocks.pages {
                if vp[p].is_null() || p == blocks.pages - 1 {
                    vp[p] = stand_in.0.as_mut_ptr();
                    wp[p] = stand_in.1.as_mut_ptr();
                    bp[p] = stand_in.2.as_mut_ptr();
                    cp[p] = stand_in.3.as_mut_ptr();
                }
            }
            let (v, w, body, collider) = (Paged(vp), Paged(wp), Paged(bp), Paged(cp));
            cx.run(&mut t, true, |n| {
                let (moved, turned) = scratch(n);
                InWorld { v, w, body, collider, derived: None, moved, turned, g }
            })
        }
        Way::StateColumn => {
            // As the world would keep it: the step's motion and turn left
            // from the last step, cleared with the prepare.
            let mut st: Vec<St> = blocks.v.iter().zip(&blocks.w).map(|(v, w)| St { v: Vec2::new(v.x, v.y), w: w.w, ..St::STILL }).collect();
            for x in st.iter_mut() {
                (x.moved, x.turned) = (Vec2::new(0.25, 0.5), Rot { c: 0.0, s: 1.0 });
            }
            let (body, collider) = (Flat(blocks.body.clone()), Flat(blocks.collider.clone()));
            cx.run(&mut t, false, |_| {
                for x in st.iter_mut() {
                    (x.moved, x.turned) = (Vec2::ZERO, Rot::IDENTITY);
                }
                StateCol { s: st, body, collider, copy: None, g }
            })
        }
        _ => unreachable!("a copy"),
    };
    drop(stand_in);
    s.out = Some((out, contacts_out(&s.buffers.constraints, &s.buffers.points)));
    s.times = t;
    s.blocks = Some(blocks);
}

/// What an in-place way's solve and write-back share, whatever keeps the
/// bodies.
struct InPlaceCx<'s, 'q, 'w, 'a> {
    params: &'s Params,
    b: &'s mut Buffers,
    live: &'s [Range<usize>],
    len: usize,
    still: u32,
    all: &'q mut All<'w, 'a>,
}

impl InPlaceCx<'_, '_, '_, '_> {
    /// Makes the place (its scratch timed with the prepare: the copy's
    /// states are made there too), solves, writes back, and reads out each
    /// body's state by entity. `pages`: velocity and spin are the world's
    /// own, so the write-back reads them from the pages it walks.
    fn run<P: Place + Source>(&mut self, t: &mut Times, pages: bool, make: impl FnOnce(usize) -> P) -> HashMap<Entity, [u32; 7]> {
        let (params, live, len) = (self.params, self.live, self.len);
        let b = &mut *self.b;
        let start = Instant::now();
        let mut place = make(len);
        let made = us(start);
        b.angle.clear();
        b.angle.resize(b.turning.len(), 0.0);
        let nowhere = self.still + 1;
        let ts = solve_in(params, &mut place, (len, live, nowhere), (&b.turning, &mut b.angle), (&mut b.constraints, &mut b.points));
        (t.prepare, t.passes, t.finish) = (made + ts[0], ts[1], ts[2]);
        // In the world's own pages, read out before the write-back takes
        // them again: after it, the pointers the solve used are stale.
        let read = |place: &P, i: usize| (Place::v(place, i), Place::w(place, i), place.moved(i), place.turned(i));
        let early: Option<HashMap<Entity, [u32; 7]>> = pages.then(|| {
            b.map
                .iter()
                .map(|&(e, i)| {
                    let (v, w, m, q) = read(&place, i as usize);
                    (e, bits(v, w, m, q))
                })
                .collect()
        });

        // The write-back: velocity and spin are in place already (in the
        // world's pages, or in the blocks that stand for them); positions
        // from how far each body moved, rotations from how far it turned,
        // each only where it changed, by the copy's rules.
        let start = Instant::now();
        let mut k = 0;
        self.all.for_each_page(|page, (body, _, v, mut p, mut q, w)| {
            for r in page.rows() {
                let (i, flat) = (b.index[k] as usize, b.flat[k] as usize);
                k += 1;
                if body[r].kind == STATIC {
                    continue;
                }
                let (vel, spin) = if pages {
                    let (v, w) = (v.as_slice()[r], w.as_slice()[r]);
                    (Vec2::new(v.x, v.y), w.w)
                } else {
                    (Place::v(&place, i), Place::w(&place, i))
                };
                let sp = Spinning { body: 0, w: spin, inv_inertia: 0.0, turned: place.turned(i), angle: 0.0 };
                let q0 = q.as_slice()[r];
                if let Some(to) = sp.turned_from(q0.rot(), b.w0[flat])
                    && (to.c.to_bits(), to.s.to_bits()) != (q0.c.to_bits(), q0.s.to_bits())
                {
                    q.set(r, Rotation::of(to));
                }
                let step = if body[r].kind == KINEMATIC { vel * DT } else { place.moved(i) };
                let p0 = p.as_slice()[r];
                let to = (p0.x + step.x, p0.y + step.y);
                if (to.0.to_bits(), to.1.to_bits()) != (p0.x.to_bits(), p0.y.to_bits()) {
                    p.set(r, Position { x: to.0, y: to.1 });
                }
            }
        });
        t.write_back = us(start);
        early.unwrap_or_else(|| {
            b.map
                .iter()
                .map(|&(e, i)| {
                    let (v, w, m, q) = read(&place, i as usize);
                    (e, bits(v, w, m, q))
                })
                .collect()
        })
    }
}

/// Every body's velocity, position, rotation and spin as bits, by entity:
/// what the world holds after a way's write-back. In place, velocity and
/// spin come from the solve's own state where the blocks stood for the
/// world's columns.
type WorldBits = HashMap<Entity, [u32; 8]>;

fn world_bits(all: &mut All) -> WorldBits {
    let mut out = HashMap::new();
    all.for_each(|row, (_, _, v, p, q, w)| {
        out.insert(row.entity(), [v.x, v.y, p.x, p.y, q.c, q.s, w.w, 0.0].map(f32::to_bits));
    });
    out
}

fn save(s: &mut Spike, all: &mut All) {
    s.saved.clear();
    all.for_each(|row, (_, _, v, p, q, w)| {
        s.saved.insert(row.entity(), (*v, *p, *q, *w));
    });
}

fn restore(s: &Spike, all: &mut All) {
    let same = |a: &[f32], b: &[f32]| a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
    all.for_each(|row, (_, _, mut v, mut p, mut q, mut w)| {
        let (v0, p0, q0, w0) = &s.saved[&row.entity()];
        if !same(&[v.x, v.y], &[v0.x, v0.y]) {
            *v = *v0;
        }
        if !same(&[p.x, p.y], &[p0.x, p0.y]) {
            *p = *p0;
        }
        if !same(&[q.c, q.s], &[q0.c, q0.s]) {
            *q = *q0;
        }
        if !same(&[w.w], &[w0.w]) {
            *w = *w0;
        }
    });
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

fn measure(label: &str, world: &World, reps: usize) {
    let g = world.values::<Gravity>().unwrap().first().map_or(Vec2::ZERO, |(_, g)| Vec2::new(g.x, g.y));
    let tuning = world.values::<Tuning>().unwrap().first().map_or(Tuning::DEFAULT, |(_, t)| *t);
    let params = Params::of(&tuning);
    assert_eq!(params.wide, solver::Wide::Colored(N), "the default tuning solves in lanes of {N}");
    let spikes: Spikes = Arc::new(Mutex::new(Spike {
        way: Way::Copy,
        world: world as *const World as usize,
        g,
        params,
        times: Times::default(),
        out: None,
        buffers: Buffers::default(),
        blocks: None,
        saved: HashMap::new(),
        counts: (0, 0, 0, 0),
    }));
    let sys = |f: fn(&mut Spike, &mut All, &mut Contacts), name: &str| {
        let s = spikes.clone();
        let run = move |_: &mut Cx, mut all: All, mut contacts: Contacts| {
            let mut s = s.lock().unwrap();
            f(&mut s, &mut all, &mut contacts);
        };
        Schedule { systems: vec![run.system(world, name)] }
    };
    let ways = sys(|s, all, contacts| if s.way.copies() { copy_way(s, all, contacts) } else { in_place_way(s, all, contacts) }, "way");
    let saves = sys(|s, all, _| save(s, all), "save");
    let restores = sys(|s, all, _| restore(s, all), "restore");
    let after = Arc::new(Mutex::new(WorldBits::new()));
    let snap = {
        let after = after.clone();
        let run = move |_: &mut Cx, mut all: All| *after.lock().unwrap() = world_bits(&mut all);
        Schedule { systems: vec![run.system(world, "snap")] }
    };
    // Every awake body turns, so one walk over `All` is every awake body
    // (checked against a query without the turning terms).
    let counted = Arc::new(Mutex::new((0, 0)));
    {
        let m = counted.clone();
        let run = move |_: &mut Cx, all: All, moving: Query<&Body, (engine_ecs::With<Velocity>, Without<Asleep>)>| {
            *m.lock().unwrap() = (all.len(), moving.len());
        };
        Schedule { systems: vec![run.system(world, "count")] }.run_sequential(world);
    }
    let (turning, moving) = *counted.lock().unwrap();
    assert_eq!(turning, moving, "every awake body turns");
    saves.run_sequential(world);
    let mut times: HashMap<Way, Vec<Times>> = HashMap::new();
    for rep in 0..reps {
        let mut ways_now = WAYS.to_vec();
        ways_now.rotate_left(rep % WAYS.len());
        let mut reference: Option<(Outcome, WorldBits)> = None;
        let mut results = Vec::new();
        for way in ways_now {
            {
                let mut s = spikes.lock().unwrap();
                s.way = way;
                if !way.copies() {
                    s.blocks = Some(blocks(world));
                }
            }
            ways.run_sequential(world);
            snap.run_sequential(world);
            let mut wb = std::mem::take(&mut *after.lock().unwrap());
            let mut s = spikes.lock().unwrap();
            let out = s.out.take().unwrap();
            if !way.copies() && way != Way::Pages {
                // Velocity and spin live in the blocks, which stand for the
                // world's columns: the world's own still hold the old ones.
                for (e, x) in wb.iter_mut() {
                    let o = out.0[e];
                    (x[0], x[1], x[6]) = (o[0], o[1], o[2]);
                }
            }
            for x in wb.values_mut() {
                x[7] = 0;
            }
            if way == Way::Copy {
                reference = Some((out.clone(), wb.clone()));
            }
            results.push((way, out, wb));
            times.entry(way).or_default().push(s.times);
            drop(s);
            restores.run_sequential(world);
        }
        let (r_out, r_world) = reference.unwrap();
        for (way, out, wb) in results {
            assert!(out.0 == r_out.0, "{label}: {} solves the bodies otherwise than the copy", way.name());
            assert!(out.1 == r_out.1, "{label}: {} solves the contacts otherwise than the copy", way.name());
            assert!(wb == r_world, "{label}: {} leaves the world otherwise than the copy", way.name());
        }
    }
    let s = spikes.lock().unwrap();
    let b = s.blocks.as_ref();
    let (bodies, turning, contacts, points) = s.counts;
    println!(
        "\n### {label}: {bodies} awake bodies in {} tables, {turning} turning, {contacts} contacts ({points} with points); {} pages of {ROWS} ({:.1} rows each)",
        b.map_or(0, |b| b.tables() - 1),
        b.map_or(0, |b| b.pages - 1),
        b.map_or(0.0, |b| bodies as f64 / (b.pages - 1) as f64),
    );
    println!("\nµs, the median of {reps}; every way bit for bit the copy, solver and world, by entity\n");
    println!("| way | walk | index | contacts | prepare | passes | finish | write-back | **all** |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for way in WAYS {
        let ts = &times[&way];
        let m = |f: fn(&Times) -> f64| median(ts.iter().map(f).collect());
        println!(
            "| {} | {:.1} | {:.1} | {:.1} | {:.1} | {:.0} | {:.1} | {:.1} | **{:.0}** |",
            way.name(),
            m(|t| t.walk),
            m(|t| t.index),
            m(|t| t.contacts),
            m(|t| t.prepare),
            m(|t| t.passes),
            m(|t| t.finish),
            m(|t| t.write_back),
            m(Times::total)
        );
    }
    drop(s);
    resolve_by_location(label, world, reps);
}

/// The other way to find a contact's bodies in place: through each end's
/// entity location (table, page, row), which in contiguous storage is the
/// index by arithmetic, so nothing is built. Against the walk and `Slots`.
fn resolve_by_location(label: &str, world: &World, reps: usize) {
    let blocks = blocks(world);
    let pairs: Vec<ContactPair> = {
        let mut p = world.values::<ContactPair>().unwrap();
        p.sort_by_key(|(_, p)| (p.a, p.b));
        p.into_iter().map(|(_, p)| p).collect()
    };
    let still = blocks.still();
    let mut by_location = Vec::new();
    let mut sum = 0u64;
    for _ in 0..reps {
        let start = Instant::now();
        for p in &pairs {
            for e in [p.a, p.b] {
                let i = match world.entities.location(e) {
                    Some(at) if blocks.page_base[at.table.0 as usize] != u32::MAX => {
                        (blocks.page_base[at.table.0 as usize] + at.page) * ROWS as u32 + at.row
                    }
                    _ => still,
                };
                sum = sum.wrapping_add(i as u64);
            }
        }
        by_location.push(us(start));
    }
    std::hint::black_box(sum);
    println!(
        "\n{label}: each contact's two ends found through entity locations instead of a map built from the walk: {:.1} µs for {} pairs ({:.1} ns a pair), no map to build",
        median(by_location.clone()),
        pairs.len(),
        median(by_location) * 1e3 / pairs.len().max(1) as f64
    );
}

// ---- Across threads ----

/// The copy's input, gathered once: bodies in walk order with the
/// stand-in for statics last, as `copy_way` gathers them.
#[derive(Clone, Default)]
struct Gathered {
    entities: Vec<Entity>,
    bodies: Vec<SolverBody>,
    spinning: Vec<Spinning>,
    constraints: Vec<Constraint>,
    points: Vec<Points>,
}

fn gather(all: &mut All, contacts: &mut Contacts, g: Vec2) -> Gathered {
    let mut x = Gathered::default();
    let Gathered { entities, bodies, spinning, .. } = &mut x;
    all.for_each_page(|page, (body, c, v, _, _, spin)| {
        let (v, spin) = (v.as_slice(), spin.as_slice());
        for (r, &e) in page.entities().iter().enumerate() {
            let k = entities.len() as u32;
            entities.push(e);
            let sb = solver_body(&body[r], Vec2::new(v[r].x, v[r].y), g);
            bodies.push(sb);
            match body[r].kind {
                DYNAMIC => spinning.push(Spinning::new(k, spin[r].w, sb.inv_mass * c[r].inertia_per_mass())),
                KINEMATIC => spinning.push(Spinning::new(k, spin[r].w, 0.0)),
                _ => (),
            }
        }
    });
    x.bodies.push(SolverBody::default());
    let slots = Slots::of(x.entities.iter().enumerate().map(|(k, &e)| (e, k as u32)));
    let still = x.entities.len() as u32;
    gather_contacts(contacts, |e| slots.get(e).unwrap_or(still), &mut x.constraints, &mut x.points);
    x
}

impl Gathered {
    /// The same bodies renumbered by `order` (`order[k]` the body put at k),
    /// the stand-in last as before.
    fn renumbered(&self, order: &[u32]) -> Gathered {
        let n = self.entities.len();
        let mut at = vec![0u32; n + 1];
        for (k, &i) in order.iter().enumerate() {
            at[i as usize] = k as u32;
        }
        at[n] = n as u32;
        let mut x = self.clone();
        x.entities = order.iter().map(|&i| self.entities[i as usize]).collect();
        x.bodies = order.iter().map(|&i| self.bodies[i as usize]).chain([SolverBody::default()]).collect();
        for sp in x.spinning.iter_mut() {
            sp.body = at[sp.body as usize];
        }
        for c in x.constraints.iter_mut() {
            (c.a, c.b) = (at[c.a as usize], at[c.b as usize]);
        }
        x
    }

    /// The copy's states as a place (by its own index), and its source.
    fn state_col(&self, g: Vec2) -> StateCol {
        let n = self.bodies.len();
        let mut a = vec![(0.0, 0.0); n];
        let mut s: Vec<St> = self.bodies.iter().map(|b| St { v: b.v, ..St::STILL }).collect();
        for sp in &self.spinning {
            a[sp.body as usize] = (sp.w, sp.inv_inertia);
            s[sp.body as usize].w = sp.w;
        }
        s.push(St::STILL);
        StateCol { s, body: Flat(Vec::new()), collider: Flat(Vec::new()), copy: Some((self.bodies.clone(), a)), g }
    }
}

/// Views a column of `f32` fields as words, for threads to share relaxed.
/// SPIKE ONLY: the small unsafe cast parallel-relations.md costs as "atomic
/// views of a column held for the walk" (`AtomicU32::from_mut_slice` is
/// unstable); landing it would be the user's decision (CLAUDE.md).
fn words<T: Copy>(x: &mut [T]) -> &[AtomicU32] {
    assert!(size_of::<T>().is_multiple_of(4) && align_of::<T>() >= align_of::<AtomicU32>(), "a column of f32 fields");
    // SAFETY: `T` is `Velocity` or `Spin`, structs of `f32`s, so the memory
    // is `len * size / 4` initialized 4-byte words, aligned for `u32`, any
    // of whose bit patterns is an `f32`; `AtomicU32` has `u32`'s size and
    // alignment. The `&mut` is held for the view's lifetime, so nothing
    // reads or writes the memory but through the view.
    unsafe { std::slice::from_raw_parts(x.as_mut_ptr() as *const AtomicU32, std::mem::size_of_val(x) / 4) }
}

fn atomic_words(n: usize, fill: [f32; 2]) -> Vec<AtomicU32> {
    (0..n).flat_map(|_| fill).map(|x| AtomicU32::new(x.to_bits())).collect()
}

/// The passes across `gang`'s threads, each way: the copy in walk order and
/// in entity order (its `Atom`s, as `solve_across` shares them), and in
/// place over atomic views of contiguous columns, and over a state column.
/// Each way's batches are prepared on the calling thread (`setup_in`), and
/// only the stages are timed; each is checked bit for bit against the
/// copy's solve on one thread.
fn across(label: &str, world: &World, gang: &Workers, reps: usize) {
    let g = world.values::<Gravity>().unwrap().first().map_or(Vec2::ZERO, |(_, g)| Vec2::new(g.x, g.y));
    let params = Params::of(&world.values::<Tuning>().unwrap().first().map_or(Tuning::DEFAULT, |(_, t)| *t));
    let got: Arc<Mutex<Option<Gathered>>> = Arc::default();
    {
        let got = got.clone();
        let run = move |_: &mut Cx, mut all: All, mut contacts: Contacts| *got.lock().unwrap() = Some(gather(&mut all, &mut contacts, g));
        Schedule { systems: vec![run.system(world, "gather")] }.run_sequential(world);
    }
    let copy = got.lock().unwrap().take().unwrap();
    let n = copy.entities.len();
    let mut by_entity: Vec<u32> = (0..n as u32).collect();
    by_entity.sort_by_key(|&k| copy.entities[k as usize]);
    let entity_order = copy.renumbered(&by_entity);
    let blocks = blocks(world);
    // Each body's in-place index, by its copy index; the stand-in's last.
    let flat: Vec<u32> = copy
        .entities
        .iter()
        .map(|&e| {
            let at = world.entities.location(e).unwrap();
            (blocks.page_base[at.table.0 as usize] + at.page) * ROWS as u32 + at.row
        })
        .chain([blocks.still()])
        .collect();
    let mut live: Vec<Range<usize>> = Vec::new();
    {
        let mut sorted = flat[..n].to_vec();
        sorted.sort();
        for i in sorted {
            match live.last_mut() {
                Some(r) if r.end == i as usize && !(i as usize).is_multiple_of(ROWS) => r.end += 1,
                _ => live.push(i as usize..i as usize + 1),
            }
        }
    }
    let turning_flat: Vec<u32> = copy.spinning.iter().map(|sp| flat[sp.body as usize]).collect();
    let in_place_contacts = || {
        let mut c = copy.constraints.clone();
        for c in c.iter_mut() {
            (c.a, c.b) = (flat[c.a as usize], flat[c.b as usize]);
        }
        c
    };
    let copy_live =
        |x: &Gathered| -> Vec<Range<usize>> { (0..x.entities.len()).step_by(ROWS).map(|k| k..(k + ROWS).min(x.entities.len())).collect() };

    // The reference: the copy on one thread, through the in-place driver.
    let reference = {
        let mut col = copy.state_col(g);
        let (mut c, mut p) = (copy.constraints.clone(), copy.points.clone());
        let turning: Vec<u32> = copy.spinning.iter().map(|sp| sp.body).collect();
        let mut angle = vec![0.0; turning.len()];
        solve_in(&params, &mut col, (n + 2, &copy_live(&copy), n as u32 + 1), (&turning, &mut angle), (&mut c, &mut p));
        let bodies: Vec<[u32; 7]> = (0..n).map(|k| bits(col.s[k].v, col.s[k].w, col.s[k].moved, col.s[k].turned)).collect();
        (bodies, contacts_out(&c, &p))
    };
    let (vx, vy) = (std::mem::offset_of!(Velocity, x) / 4, std::mem::offset_of!(Velocity, y) / 4);
    let mut rows: Vec<(&str, Vec<f64>)> = Vec::new();
    for way in [
        "copy, walk order (`Atom`s)",
        "copy, entity order (`Atom`s)",
        "in place: contiguous columns, atomic views",
        "in place: a state column (`Atom`s)",
    ] {
        let mut times = Vec::new();
        for _ in 0..reps {
            warm(gang);
            let (us, bodies, contacts) = match way {
                w if w.starts_with("copy") => {
                    let x = if w.contains("entity") { &entity_order } else { &copy };
                    let mut col = x.state_col(g);
                    let (mut c, mut p) = (x.constraints.clone(), x.points.clone());
                    let turning: Vec<u32> = x.spinning.iter().map(|sp| sp.body).collect();
                    let mut angle = vec![0.0; turning.len()];
                    let live = copy_live(x);
                    let mut s = solver::lanes::setup_in::<N, _>(&params, &mut col, (n + 2, &live, n as u32 + 1), &mut c, &mut p, DT);
                    let atoms = solver::lanes::Atoms::of(&col, n + 2);
                    let start = Instant::now();
                    solver::lanes::run_across_in(&mut s, &params, &atoms, &live, (&turning, &mut angle), gang);
                    let us = us(start);
                    solver::lanes::finish_in(&s, &params, &mut c, &mut p);
                    // Each body's place, by its place in the walk.
                    let mut at: Vec<usize> = (0..n).collect();
                    if w.contains("entity") {
                        for (pos, &i) in by_entity.iter().enumerate() {
                            at[i as usize] = pos;
                        }
                    }
                    let bodies: Vec<[u32; 7]> = (0..n)
                        .map(|k| {
                            let (v, w, m, q) = atoms.state(at[k]);
                            bits(v, w, m, q)
                        })
                        .collect();
                    (us, bodies, contacts_out(&c, &p))
                }
                w if w.contains("contiguous") => {
                    let len = blocks.pages * ROWS;
                    let mut place = InWorld {
                        v: Flat(blocks.v.clone()),
                        w: Flat(blocks.w.clone()),
                        body: Flat(blocks.body.clone()),
                        collider: Flat(blocks.collider.clone()),
                        derived: None,
                        moved: vec![Vec2::ZERO; len],
                        turned: vec![Rot::IDENTITY; len],
                        g,
                    };
                    let (mut c, mut p) = (in_place_contacts(), copy.points.clone());
                    let mut angle = vec![0.0; turning_flat.len()];
                    let mut s = solver::lanes::setup_in::<N, _>(&params, &mut place, (len, &live, blocks.still() + 1), &mut c, &mut p, DT);
                    let (moved, turned) = (atomic_words(len, [0.0, 0.0]), atomic_words(len, [1.0, 0.0]));
                    let InWorld { v, w, .. } = &mut place;
                    let shared = solver::lanes::Words { v: words(&mut v.0), vx, vy, w: words(&mut w.0), moved: &moved, turned: &turned };
                    let start = Instant::now();
                    solver::lanes::run_across_in(&mut s, &params, &shared, &live, (&turning_flat, &mut angle), gang);
                    let us = us(start);
                    solver::lanes::finish_in(&s, &params, &mut c, &mut p);
                    let bodies: Vec<[u32; 7]> = (0..n)
                        .map(|k| {
                            let (v, w, m, q) = shared.state(flat[k] as usize);
                            bits(v, w, m, q)
                        })
                        .collect();
                    (us, bodies, contacts_out(&c, &p))
                }
                _ => {
                    let len = blocks.pages * ROWS;
                    let s: Vec<St> =
                        blocks.v.iter().zip(&blocks.w).map(|(v, w)| St { v: Vec2::new(v.x, v.y), w: w.w, ..St::STILL }).collect();
                    let mut col = StateCol { s, body: Flat(blocks.body.clone()), collider: Flat(blocks.collider.clone()), copy: None, g };
                    let (mut c, mut p) = (in_place_contacts(), copy.points.clone());
                    let mut angle = vec![0.0; turning_flat.len()];
                    let mut s = solver::lanes::setup_in::<N, _>(&params, &mut col, (len, &live, blocks.still() + 1), &mut c, &mut p, DT);
                    let atoms = solver::lanes::Atoms::of(&col, len);
                    let start = Instant::now();
                    solver::lanes::run_across_in(&mut s, &params, &atoms, &live, (&turning_flat, &mut angle), gang);
                    let us = us(start);
                    solver::lanes::finish_in(&s, &params, &mut c, &mut p);
                    let bodies: Vec<[u32; 7]> = (0..n)
                        .map(|k| {
                            let (v, w, m, q) = atoms.state(flat[k] as usize);
                            bits(v, w, m, q)
                        })
                        .collect();
                    (us, bodies, contacts_out(&c, &p))
                }
            };
            assert!(bodies == reference.0, "{label}: {way} across threads solves the bodies otherwise than the copy on one");
            assert!(contacts == reference.1, "{label}: {way} across threads solves the contacts otherwise than the copy on one");
            times.push(us);
        }
        rows.push((way, times));
    }
    println!(
        "\n{label}, the passes across {} kept threads (gravity, warm start, push, move, relax, bounce stages; batches prepared on one thread), µs, the median of {reps}, each bit for bit the copy on one thread:\n",
        gang.threads()
    );
    println!("| way | passes |");
    println!("|---|---|");
    for (way, times) in rows {
        println!("| {way} | {:.0} |", median(times));
    }
}

/// Keeps the pool's threads busy for a while, so its cores are clocked up
/// before a solve across them
/// (docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md).
fn warm(gang: &Workers) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(30) {
        gang.run(gang.threads(), |_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_micros(500) {
                std::hint::spin_loop();
            }
        });
    }
}

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(21);
    let only = std::env::var("ONLY").unwrap_or_default();
    let threads: Option<usize> = std::env::var("THREADS").ok().and_then(|t| t.parse().ok());
    let pool = threads.map(|n| Arc::new(pool::Pool::new(n)));
    let gang = pool.as_ref().map(|p| Workers::new(Some(p.clone() as Arc<dyn Executor>)));
    if let (Some(p), Some(n)) = (&pool, threads) {
        let warm = Instant::now();
        while warm.elapsed() < Duration::from_millis(300) {
            p.run(n, &|_| {
                let t = Instant::now();
                while t.elapsed() < Duration::from_millis(1) {
                    std::hint::spin_loop();
                }
            });
        }
    }
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    let pile = Scene::Pile { n: 10000, width: 401.0, stagger: true };
    let pyramid = Scene::Pyramid { base: 100 };
    let cases = [
        ("pile 10 000, falling, step 31", pile, 31),
        ("pile 10 000, settled, step 430", pile, 430),
        ("pyramid 5050, step 630", pyramid, 630),
    ];
    for (name, scene, steps) in cases.iter().filter(|c| only.is_empty() || c.0.contains(only.as_str())) {
        let mut ecs = ecs::Ecs::new(&manifest, scene, false, true);
        ecs.step(*steps);
        if std::env::var("ACROSS_ONLY").is_err() {
            measure(name, ecs.engine().world(), reps);
        }
        if let Some(gang) = &gang {
            across(name, ecs.engine().world(), gang, reps);
        }
    }
}
