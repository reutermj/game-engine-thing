//! Operation sequences against trivially correct models, compared after
//! every step: the differential tests' driver, shared by `model_test`
//! (seeded, natively and under Miri), the fuzzers (`//engine/ecs/fuzz`,
//! choices from libFuzzer's bytes) and the corpus replays. One driver, so
//! an input the fuzzer finds replays exactly under Miri.
//!
//! Two drivers, one per layer of the unsafe core:
//! - `world`: structural changes through `Structural` (spawns, inserts,
//!   removes, despawns, keys and positions written in place, and the
//!   re-sorts of ordered and spatial tables they cause) and layout
//!   migrations, against a map from entity to values.
//! - `columns`: `ErasedColumn`'s own operations (moves, drops, gathers,
//!   migrations, ticks, and panics partway through), against vectors.
//!
//! Heap values are `Canary`s, which count themselves, so a value dropped
//! twice or leaked fails the step it happened in, natively, and not only
//! under Miri.

use std::cell::Cell;
use std::collections::BTreeMap;

use engine_ecs::erased::{ErasedColumn, ValueType};
use engine_ecs::world::PAGE_ROWS;
use engine_ecs::{
    Bounds, Build, Component, ComponentDesc, ComponentId, Crossing, Entity, FieldKind, FieldType, OrderKey, SpatialKey,
    Structural, World, component, schema,
};

/// Where a driver's choices come from.
pub trait Choices {
    /// A number below `n` (0 if `n` is 0 or 1, without using a choice up).
    fn below(&mut self, n: u64) -> u64;
    /// Any 64-bit number.
    fn word(&mut self) -> u64;
    /// Whether the choices have run out: the drivers stop.
    fn done(&self) -> bool;
}

/// xorshift64*: deterministic, and enough for choosing operations. Never
/// runs out.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed | 1)
    }
}

impl Choices for Rng {
    fn below(&mut self, n: u64) -> u64 {
        if n <= 1 { 0 } else { self.word() % n }
    }
    fn word(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn done(&self) -> bool {
        false
    }
}

/// Choices read from a fuzzer's input: a byte per small choice, so a
/// mutation of one byte changes one choice. Zeros once the input is used
/// up, which only matters for the step that ran it out.
pub struct Bytes<'a>(&'a [u8]);

impl<'a> Bytes<'a> {
    pub fn new(data: &'a [u8]) -> Bytes<'a> {
        Bytes(data)
    }
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0; N];
        let n = N.min(self.0.len());
        out[..n].copy_from_slice(&self.0[..n]);
        self.0 = &self.0[n..];
        out
    }
}

impl Choices for Bytes<'_> {
    fn below(&mut self, n: u64) -> u64 {
        match n {
            0 | 1 => 0,
            2..=256 => self.take::<1>()[0] as u64 % n,
            _ => u32::from_le_bytes(self.take::<4>()) as u64 % n,
        }
    }
    fn word(&mut self) -> u64 {
        u64::from_le_bytes(self.take::<8>())
    }
    fn done(&self) -> bool {
        self.0.is_empty()
    }
}

thread_local! {
    /// Canaries alive on this thread: each test and fuzz run is one thread.
    static CANARIES: Cell<i64> = const { Cell::new(0) };
    /// Inside `panics`, whose panic is the expected outcome.
    static EXPECTING: Cell<bool> = const { Cell::new(false) };
}

/// Whether `f` panicked: for operations that should refuse. Quiet under
/// `quiet_expected_panics`, since a fuzzer runs millions.
pub fn panics(f: impl FnOnce()) -> bool {
    EXPECTING.with(|x| x.set(true));
    let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).is_err();
    EXPECTING.with(|x| x.set(false));
    panicked
}

/// Leaves panics inside `panics` unprinted; every other panic prints as
/// before.
pub fn quiet_expected_panics() {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if !EXPECTING.with(Cell::get) {
            previous(info);
        }
    }));
}

/// Canaries alive now (on this thread).
pub fn canaries() -> i64 {
    CANARIES.with(Cell::get)
}

const ALIVE: u64 = 0x9E37_79B9_7F4A_7C15;
const DEAD: u64 = 0xDEAD_DEAD_DEAD_DEAD;

/// A heap value that knows its id and whether it's alive: dropped twice,
/// or read after it was dropped, it panics (natively, while its memory
/// hasn't been reused; Miri catches every case), and the live count shows
/// a leak or a lost drop.
pub struct Canary(Box<[u64; 2]>);

impl Canary {
    pub fn new(id: u64) -> Canary {
        CANARIES.with(|c| c.set(c.get() + 1));
        Canary(Box::new([id, id ^ ALIVE]))
    }

    pub fn id(&self) -> u64 {
        let [id, check] = *self.0;
        assert_eq!(check, id ^ ALIVE, "a canary read after it was dropped, or never written");
        id
    }
}

impl Clone for Canary {
    fn clone(&self) -> Canary {
        Canary::new(self.id())
    }
}

impl Drop for Canary {
    fn drop(&mut self) {
        self.id();
        self.0[1] = DEAD;
        CANARIES.with(|c| c.set(c.get() - 1));
    }
}

impl std::fmt::Debug for Canary {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Canary({})", self.id())
    }
}

/// The id a default canary has: the one a migration gives a field the old
/// layout lacked.
pub const DEFAULT_ID: u64 = u64::MAX;

impl Default for Canary {
    fn default() -> Canary {
        Canary::new(DEFAULT_ID)
    }
}

// SAFETY: a box of plain data, which holds nothing of the mod's image.
unsafe impl Crossing for Canary {}
// SAFETY: opaque, so it's only ever moved whole into a field of the same
// fingerprint.
unsafe impl FieldType for Canary {
    const KIND: FieldKind = FieldKind::OPAQUE;
    const FINGERPRINT: u64 = engine_ecs::__fingerprint("test::Canary", &[]);
}

/// `test::A` in two layouts, for migrations back and forth.
pub mod a {
    use super::Canary;

    engine_ecs::component! {
        /// 16 bytes with 4 of padding: moved as a fixed-size copy.
        #[derive(Debug, Default)]
        pub struct V1: "test::A" {
            pub c: Canary,
            pub n: u32,
        }
    }

    engine_ecs::component! {
        /// Reordered, `n` widened, a heap field added (its default a
        /// canary of its own), and over-aligned: 32 bytes, 16 of padding.
        #[derive(Debug, Default)]
        #[repr(align(32))]
        pub struct V2: "test::A" {
            pub n: u64,
            pub tag: Canary,
            pub c: Canary,
        }
    }
}

component! {
    #[derive(Debug, Default)]
    pub struct B: "test::B" { pub c: Canary }
}

component! {
    #[derive(Debug, Default, PartialEq)]
    pub struct Tag: "test::Tag" {}
}

component! {
    #[derive(Debug, Default)]
    pub struct S: "test::S", storage = sparse { pub c: Canary }
}

component! {
    /// An ordered key with many ties: its tables are re-sorted, by
    /// `ErasedColumn::gather`, whenever a row joins or a key is written.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct K: "test::K", order = key { pub k: u32 }
}

impl OrderKey for K {
    #[inline]
    fn key(&self) -> u128 {
        self.k as u128
    }
}

component! {
    /// A spatial key: its tables are re-sorted into Z-order, which moves
    /// rows between pages, whenever a row joins or a position is written.
    /// It wins over `K` in a table holding both. On a grid, so the model
    /// compares it exactly.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct P: "test::P", order = spatial { pub x: f32, pub y: f32 }
}

impl SpatialKey for P {
    type Extent = P;
    fn bounds(&self, _: Option<&P>) -> Bounds {
        Bounds::around([self.x, self.y], [0.25, 0.25])
    }
}

fn position(ch: &mut impl Choices) -> (u32, u32) {
    (ch.below(40) as u32, ch.below(40) as u32)
}

fn p((x, y): (u32, u32)) -> P {
    P { x: x as f32, y: y as f32 }
}

/// What the model holds of an entity: its values' ids and numbers.
#[derive(Clone, Debug, Default, PartialEq)]
struct Model {
    /// `A`'s canary, its number, and in the second layout its tag.
    a: Option<(u64, u64, Option<u64>)>,
    b: Option<u64>,
    tag: bool,
    s: Option<u64>,
    k: Option<u32>,
    p: Option<(u32, u32)>,
}

impl Model {
    fn canaries(&self) -> i64 {
        let a = self.a.map_or(0, |(_, _, tag)| 1 + tag.is_some() as i64);
        a + self.b.is_some() as i64 + self.s.is_some() as i64
    }
}

struct Ids {
    a: ComponentId,
    b: ComponentId,
    tag: ComponentId,
    s: ComponentId,
    k: ComponentId,
    p: ComponentId,
}

/// The world's contents as the model would hold them, checking on the way
/// that every row's entity is placed there, pages are consistent, an
/// ordered table's rows are in key order, and a spatial table's order
/// keeps its invariants.
fn contents(w: &World, ids: &Ids, v2: bool) -> BTreeMap<Entity, Model> {
    let mut out: BTreeMap<Entity, Model> = BTreeMap::new();
    for t in w.tables() {
        let rows = t.rows.read().unwrap();
        let cols: Vec<_> = t.columns.iter().map(|c| c.read().unwrap()).collect();
        assert_eq!(rows.len(), cols.first().map_or(rows.len(), |c| c.len()), "every column has the table's pages");
        if let Some(spatial) = &t.spatial {
            let pages = spatial.pages.read().unwrap();
            pages.check(&rows, P::BIG, P::CELL).unwrap_or_else(|e| panic!("table {:?}: {e}", t.id));
        }
        let ordered = t.ordered.is_some();
        let mut last_key: Option<(u32, Entity)> = None;
        for (p, page) in rows.iter().enumerate() {
            assert!(page.len() <= PAGE_ROWS);
            for col in &cols {
                assert_eq!(col[p].len(), page.len(), "every column has the page's rows");
                assert_eq!(col[p].ticks().len(), page.len(), "every value has a tick");
            }
            for (r, &e) in page.iter().enumerate() {
                let loc = w.entities.location(e).expect("a row's entity is placed");
                assert_eq!((loc.table, loc.page as usize, loc.row as usize), (t.id, p, r), "location matches row");
                let mut m = Model::default();
                for (i, &c) in t.components.iter().enumerate() {
                    let col = &cols[i][p];
                    if c == ids.a && v2 {
                        let v = &col.as_slice::<a::V2>()[r];
                        m.a = Some((v.c.id(), v.n, Some(v.tag.id())));
                    } else if c == ids.a {
                        let v = &col.as_slice::<a::V1>()[r];
                        m.a = Some((v.c.id(), v.n as u64, None));
                    } else if c == ids.b {
                        m.b = Some(col.as_slice::<B>()[r].c.id());
                    } else if c == ids.tag {
                        assert_eq!(col.as_slice::<Tag>().len(), page.len());
                        m.tag = true;
                    } else if c == ids.k {
                        let k = col.as_slice::<K>()[r].k;
                        if ordered {
                            assert!(last_key.is_none_or(|l| l < (k, e)), "an ordered table in key order, then entity");
                            last_key = Some((k, e));
                        }
                        m.k = Some(k);
                    } else if c == ids.p {
                        let at = col.as_slice::<P>()[r];
                        m.p = Some((at.x as u32, at.y as u32));
                    }
                }
                assert!(out.insert(e, m).is_none(), "an entity in two rows");
            }
        }
    }
    let set = w.sparse_set(ids.s).read().unwrap();
    for (e, m) in out.iter_mut() {
        m.s = set.get::<S>(*e).map(|s| s.c.id());
    }
    out
}

/// A build loaded at `loaded_at`: a later one's layout takes over.
fn build(loaded_at: u64) -> Build {
    Build { name: format!("build {loaded_at}"), loaded_at, keepalive: None }
}

/// Up to `steps` random structural changes and migrations, applied to a
/// world and to the model, compared after each. Operations on dead
/// entities, which the world must ignore, are part of the mix. `spawns`
/// weighs the mix towards spawning, which otherwise barely outpaces
/// despawning, so tables fill pages. Returns the most pages a table had.
pub fn world(ch: &mut impl Choices, steps: usize, spawns: u64) -> usize {
    let before = canaries();
    let pages = {
        let w = World::new();
        run_world(&w, ch, steps, spawns)
    };
    assert_eq!(canaries(), before, "every value the world held was dropped once");
    pages
}

fn run_world(w: &World, ch: &mut impl Choices, steps: usize, spawns: u64) -> usize {
    let mut most_pages = 0;
    let ids = {
        let m = w.between_frames(build(0)).unwrap();
        Ids { a: m.id::<a::V1>(), b: m.id::<B>(), tag: m.id::<Tag>(), s: m.id::<S>(), k: m.id::<K>(), p: m.id::<P>() }
    };
    let (mut v2, mut loaded_at) = (false, 0);
    let mut model: BTreeMap<Entity, Model> = BTreeMap::new();
    let mut dead: Vec<Entity> = Vec::new();
    let mut next_id = 0u64;
    let mut id = move || {
        next_id += 1;
        next_id
    };
    for step in 0..steps {
        if ch.done() {
            break;
        }
        let live: Vec<Entity> = model.keys().copied().collect();
        let pick = |ch: &mut dyn FnMut(u64) -> u64| -> Entity {
            // Sometimes a dead entity, which every operation must ignore.
            if !dead.is_empty() && (live.is_empty() || ch(8) == 0) {
                dead[ch(dead.len() as u64) as usize]
            } else {
                live[ch(live.len() as u64) as usize]
            }
        };
        // Ops past 15 are the extra spawns.
        let op = if live.is_empty() && dead.is_empty() { 0 } else { ch.below(16 + spawns) };
        let op = if op >= 16 { 0 } else { op };
        if op == 15 {
            // A newer build with the other layout of `A`: every value
            // migrates, in every table. Between frames, so no `Structural`.
            loaded_at += 1;
            let desc = if v2 { ComponentDesc::of::<a::V1>() } else { ComponentDesc::of::<a::V2>() };
            let report = w.install(&desc, &build(loaded_at)).expect("a newer build's layout");
            assert!(report.is_some_and(|r| r.starts_with("migrated")), "a layout change migrates");
            v2 = !v2;
            for (_, n, tag) in model.values_mut().filter_map(|m| m.a.as_mut()) {
                if v2 {
                    *tag = Some(DEFAULT_ID);
                } else {
                    (*n, *tag) = (*n as u32 as u64, None);
                }
            }
        } else {
            // Sparse entries of dead entities are only purged when a set is
            // written with purging; half the time, leave them, as a frame
            // would.
            let mut s = Structural::new(w);
            let tables: Vec<_> = w.tables().map(|t| t.id).collect();
            for t in tables {
                s.lock_table(t);
            }
            s.lock_sparse_with(ids.s, ch.below(2) == 0);
            match op {
                0 | 1 => {
                    // Reuses dead entities' indices, so a new entity can meet
                    // a dead one's sparse entry.
                    let e = w.entities.reserve();
                    let (n, k) = (ch.below(1000), ch.below(8) as u32);
                    let mut m = Model::default();
                    match ch.below(6) {
                        0 => {
                            let (ca, cb) = (id(), id());
                            if v2 {
                                s.spawn(e, (a::V2 { n, tag: Canary::new(n), c: Canary::new(ca) }, B { c: Canary::new(cb) }), &[ids.a, ids.b]);
                                m.a = Some((ca, n, Some(n)));
                            } else {
                                s.spawn(e, (a::V1 { c: Canary::new(ca), n: n as u32 }, B { c: Canary::new(cb) }), &[ids.a, ids.b]);
                                m.a = Some((ca, n, None));
                            }
                            m.b = Some(cb);
                        }
                        1 => {
                            let cb = id();
                            s.spawn(e, (B { c: Canary::new(cb) }, K { k }), &[ids.b, ids.k]);
                            (m.b, m.k) = (Some(cb), Some(k));
                        }
                        2 => {
                            s.spawn(e, (K { k }, Tag {}), &[ids.k, ids.tag]);
                            (m.k, m.tag) = (Some(k), true);
                        }
                        3 => {
                            let (cb, at) = (id(), position(ch));
                            s.spawn(e, (p(at), B { c: Canary::new(cb) }), &[ids.p, ids.b]);
                            (m.p, m.b) = (Some(at), Some(cb));
                        }
                        4 => {
                            let at = position(ch);
                            s.spawn(e, (p(at), K { k }), &[ids.p, ids.k]);
                            (m.p, m.k) = (Some(at), Some(k));
                        }
                        _ => {
                            let cs = id();
                            s.spawn(e, (Tag {}, S { c: Canary::new(cs) }), &[ids.tag, ids.s]);
                            (m.tag, m.s) = (true, Some(cs));
                        }
                    }
                    model.insert(e, m);
                }
                2 => {
                    let e = pick(&mut |n| ch.below(n));
                    let (c, n) = (id(), ch.below(1000));
                    if v2 {
                        s.insert_id(e, ids.a, a::V2 { n, tag: Canary::new(n), c: Canary::new(c) });
                    } else {
                        s.insert_id(e, ids.a, a::V1 { c: Canary::new(c), n: n as u32 });
                    }
                    if let Some(m) = model.get_mut(&e) {
                        m.a = Some((c, n, v2.then_some(n)));
                    }
                }
                3 => {
                    let e = pick(&mut |n| ch.below(n));
                    let c = id();
                    s.insert_id(e, ids.b, B { c: Canary::new(c) });
                    if let Some(m) = model.get_mut(&e) {
                        m.b = Some(c);
                    }
                }
                4 => {
                    let e = pick(&mut |n| ch.below(n));
                    s.insert_id(e, ids.tag, Tag {});
                    if let Some(m) = model.get_mut(&e) {
                        m.tag = true;
                    }
                }
                5 => {
                    let e = pick(&mut |n| ch.below(n));
                    let c = id();
                    s.insert_id(e, ids.s, S { c: Canary::new(c) });
                    if let Some(m) = model.get_mut(&e) {
                        m.s = Some(c);
                    }
                }
                6 => {
                    let e = pick(&mut |n| ch.below(n));
                    let k = ch.below(8) as u32;
                    s.insert_id(e, ids.k, K { k });
                    if let Some(m) = model.get_mut(&e) {
                        m.k = Some(k);
                    }
                }
                7 => {
                    let e = pick(&mut |n| ch.below(n));
                    let (c, clear): (ComponentId, fn(&mut Model)) = match ch.below(6) {
                        0 => (ids.a, |m| m.a = None),
                        1 => (ids.b, |m| m.b = None),
                        2 => (ids.tag, |m| m.tag = false),
                        3 => (ids.s, |m| m.s = None),
                        4 => (ids.k, |m| m.k = None),
                        _ => (ids.p, |m| m.p = None),
                    };
                    s.remove_id(e, c);
                    if let Some(m) = model.get_mut(&e) {
                        clear(m);
                    }
                }
                8 | 9 => {
                    let e = pick(&mut |n| ch.below(n));
                    s.despawn(e);
                    if model.remove(&e).is_some() {
                        dead.push(e);
                    }
                }
                10 => {
                    // A key written in place: its row moves when `s` drops.
                    let e = pick(&mut |n| ch.below(n));
                    let k = ch.below(8) as u32;
                    if let Some(key) = s.get::<K>(e, ids.k) {
                        key.k = k;
                    }
                    if let Some(m) = model.get_mut(&e).filter(|m| m.k.is_some()) {
                        m.k = Some(k);
                    }
                }
                11 => {
                    // A heap value replaced in place, dropping the old.
                    let e = pick(&mut |n| ch.below(n));
                    let c = id();
                    if let Some(b) = s.get::<B>(e, ids.b) {
                        b.c = Canary::new(c);
                    }
                    if let Some(m) = model.get_mut(&e).filter(|m| m.b.is_some()) {
                        m.b = Some(c);
                    }
                }
                12 => {
                    // A sparse value replaced in place.
                    let e = pick(&mut |n| ch.below(n));
                    let c = id();
                    if let Some(v) = s.get::<S>(e, ids.s) {
                        v.c = Canary::new(c);
                    }
                    if let Some(m) = model.get_mut(&e).filter(|m| m.s.is_some()) {
                        m.s = Some(c);
                    }
                }
                13 => {
                    let e = pick(&mut |n| ch.below(n));
                    let at = position(ch);
                    s.insert_id(e, ids.p, p(at));
                    if let Some(m) = model.get_mut(&e) {
                        m.p = Some(at);
                    }
                }
                _ => {
                    // A position written in place: its row may move to
                    // another page when `s` drops.
                    let e = pick(&mut |n| ch.below(n));
                    let at = position(ch);
                    if let Some(v) = s.get::<P>(e, ids.p) {
                        *v = p(at);
                    }
                    if let Some(m) = model.get_mut(&e).filter(|m| m.p.is_some()) {
                        m.p = Some(at);
                    }
                }
            }
            drop(s);
        }
        let got = contents(w, &ids, v2);
        assert_eq!(got, model, "step {step}, op {op}");
        // Dead entities' sparse entries may linger until purged, so only
        // as many canaries as the model's, or more.
        let expected: i64 = model.values().map(Model::canaries).sum();
        let lingering = w.sparse_set(ids.s).read().unwrap().len() as i64 - model.values().filter(|m| m.s.is_some()).count() as i64;
        assert_eq!(canaries() - expected - lingering, 0, "step {step}, op {op}: values dropped once, none leaked");
        most_pages = w.tables().map(|t| t.rows.read().unwrap().len()).fold(most_pages, usize::max);
    }
    most_pages
}

/// The column driver's two layouts of one value, for migrations.
pub mod v {
    use super::Canary;

    engine_ecs::component! {
        #[derive(Debug, Default)]
        pub struct V1: "test::V" {
            pub c: Canary,
            pub n: u32,
        }
    }

    engine_ecs::component! {
        #[derive(Debug, Default)]
        #[repr(align(32))]
        pub struct V2: "test::V" {
            pub n: u64,
            pub c: Canary,
        }
    }
}

/// What the model holds of a value: its canary, number and tick.
type Value = (u64, u64, u32);

/// A layout's schema, as the world reads it to migrate.
struct Schema {
    ty: ValueType,
    fields: Vec<schema::Field>,
    drops: Vec<Option<engine_ecs::DropFn>>,
    default: engine_ecs::DefaultFn,
}

fn schema_of<T: Component>() -> Schema {
    let desc = ComponentDesc::of::<T>();
    // SAFETY: a `ComponentDesc` made by `of`, which describes `T`.
    let (fields, drops) = unsafe { schema::read_fields(desc.fields, desc.field_count, desc.size, desc.align) }.expect("a valid schema");
    Schema { ty: ValueType::from_desc(&desc), fields, drops, default: desc.default }
}

fn read(pages: &[ErasedColumn], v2: bool) -> Vec<Vec<Value>> {
    pages
        .iter()
        .map(|p| {
            let values: Vec<(u64, u64)> = if v2 {
                p.as_slice::<v::V2>().iter().map(|v| (v.c.id(), v.n)).collect()
            } else {
                p.as_slice::<v::V1>().iter().map(|v| (v.c.id(), v.n as u64)).collect()
            };
            assert_eq!(p.ticks().len(), values.len(), "a tick per value");
            assert!(p.ticks().iter().all(|&t| t <= p.written()), "a page's written tick is at least its values'");
            values.into_iter().zip(p.ticks()).map(|((c, n), &t)| (c, n, t)).collect()
        })
        .collect()
}

/// Every value's place, page by page, as `gather` takes an order.
fn places(model: &[Vec<Value>]) -> Vec<(u32, u32)> {
    model.iter().enumerate().flat_map(|(p, page)| (0..page.len()).map(move |r| (p as u32, r as u32))).collect()
}

/// Up to `steps` random operations on the pages of one column, applied to
/// the pages and to vectors, compared after each: moves between pages,
/// drops, re-sorts by `gather`, migrations between layouts, ticks, and
/// gathers and migrations that panic partway.
pub fn columns(ch: &mut impl Choices, steps: usize) {
    let before = canaries();
    {
        let schemas = [schema_of::<v::V1>(), schema_of::<v::V2>()];
        let mut v2 = false;
        let mut pages: Vec<ErasedColumn> = vec![ErasedColumn::new(schemas[0].ty)];
        let mut model: Vec<Vec<Value>> = vec![Vec::new()];
        let mut next_id = 0u64;
        for step in 0..steps {
            if ch.done() {
                break;
            }
            let op = ch.below(12);
            let p = ch.below(pages.len() as u64) as usize;
            let len = model[p].len() as u64;
            match op {
                0 | 1 => {
                    // Onto a page, or a new one.
                    let p = if ch.below(8) == 0 {
                        pages.push(ErasedColumn::new(schemas[v2 as usize].ty));
                        model.push(Vec::new());
                        pages.len() - 1
                    } else {
                        p
                    };
                    next_id += 1;
                    let n = if v2 { ch.word() } else { ch.word() as u32 as u64 };
                    if v2 {
                        pages[p].push(v::V2 { n, c: Canary::new(next_id) });
                    } else {
                        pages[p].push(v::V1 { c: Canary::new(next_id), n: n as u32 });
                    }
                    model[p].push((next_id, n, 0));
                }
                2 if len > 0 => {
                    let r = ch.below(len) as usize;
                    pages[p].swap_remove_drop(r);
                    model[p].swap_remove(r);
                }
                3 if len > 0 => {
                    let r = ch.below(len) as usize;
                    let to = ch.below(pages.len() as u64) as usize;
                    if to == p {
                        continue;
                    }
                    let [from, into] = pages.get_disjoint_mut([p, to]).unwrap();
                    from.swap_remove_into(r, into);
                    let moved = model[p].swap_remove(r);
                    model[to].push(moved);
                }
                4 if len > 0 => {
                    let r = ch.below(len) as usize;
                    next_id += 1;
                    let n = ch.below(1000);
                    if v2 {
                        pages[p].replace(r, v::V2 { n, c: Canary::new(next_id) });
                    } else {
                        pages[p].replace(r, v::V1 { c: Canary::new(next_id), n: n as u32 });
                    }
                    let tick = model[p][r].2;
                    model[p][r] = (next_id, n, tick);
                }
                5 => {
                    let n = ch.below(len + 1) as usize;
                    pages[p].drop_front(n);
                    model[p].drain(..n);
                }
                6 if len > 0 => {
                    let r = ch.below(len) as usize;
                    let tick = ch.below(1 << 20) as u32;
                    pages[p].set_tick(r, tick);
                    model[p][r].2 = tick;
                }
                7 => {
                    // A re-sort: every value, in a random order, into
                    // pages of a random size.
                    let mut order = places(&model);
                    for i in (1..order.len()).rev() {
                        order.swap(i, ch.below(i as u64 + 1) as usize);
                    }
                    let page_rows = 1 + ch.below(6) as usize;
                    pages = ErasedColumn::gather(&mut pages, &order, page_rows);
                    let flat: Vec<Value> = order.iter().map(|&(p, r)| model[p as usize][r as usize]).collect();
                    model = flat.chunks(page_rows).map(<[Value]>::to_vec).collect();
                    if model.is_empty() {
                        model.push(Vec::new());
                    }
                }
                8 => {
                    // A re-sort whose order is wrong: refused, with nothing
                    // moved.
                    let mut order = places(&model);
                    if order.is_empty() {
                        continue;
                    }
                    let i = ch.below(order.len() as u64) as usize;
                    order[i] = match ch.below(3) {
                        0 => order[ch.below(order.len() as u64) as usize],
                        1 => (pages.len() as u32 + ch.below(2) as u32, 0),
                        _ => (order[i].0, model[order[i].0 as usize].len() as u32 + ch.below(2) as u32),
                    };
                    let (p, r) = (order[i].0 as usize, order[i].1 as usize);
                    let twice = order.iter().enumerate().any(|(j, x)| j != i && *x == order[i]);
                    if !twice && model.get(p).is_some_and(|page| r < page.len()) {
                        // Replaced with itself: still right.
                        continue;
                    }
                    let refused = panics(|| {
                        ErasedColumn::gather(&mut pages, &order, 3);
                    });
                    assert!(refused, "step {step}: a wrong order is refused");
                }
                9 | 10 => {
                    // A newer layout, for every page; first, for op 10, a
                    // migration of one page that panics partway, which
                    // leaves that page empty.
                    let fail_at = (op == 10 && len > 0).then(|| ch.below(len) as usize);
                    let (from, to) = (&schemas[v2 as usize], &schemas[!v2 as usize]);
                    let migrate = |page: &mut ErasedColumn, fail_at: Option<usize>| {
                        let mut row = 0;
                        // SAFETY: `schema::migrate` with each layout's own
                        // schema; the one it fails on is dropped first.
                        unsafe {
                            page.migrate(to.ty, |old, new| {
                                if fail_at == Some(row) {
                                    engine_ecs::erased::drop_value(from.ty, old);
                                    panic!("a migration that fails partway");
                                }
                                row += 1;
                                schema::migrate((old, &from.fields, &from.drops), (new, &to.fields, &to.drops, to.default))
                            })
                        }
                    };
                    if let Some(at) = fail_at {
                        assert!(panics(|| migrate(&mut pages[p], Some(at))));
                        assert!(pages[p].is_empty() && pages[p].ticks().is_empty(), "a failed migration empties its page");
                        model[p].clear();
                        // The page is left with its old layout, so it's
                        // migrated (empty) with the rest.
                    }
                    for page in &mut pages {
                        migrate(page, None);
                    }
                    v2 = !v2;
                    for value in model.iter_mut().flatten() {
                        if !v2 {
                            value.1 = value.1 as u32 as u64;
                        }
                    }
                }
                _ => {
                    // An empty page dropped, as a table's last page may be.
                    if pages.len() > 1 && len == 0 {
                        pages.remove(p);
                        model.remove(p);
                    }
                }
            }
            assert_eq!(read(&pages, v2), model, "step {step}, op {op}");
            let expected = model.iter().map(Vec::len).sum::<usize>() as i64;
            assert_eq!(canaries() - before, expected, "step {step}, op {op}: values dropped once, none leaked");
        }
    }
    assert_eq!(canaries(), before, "every value the pages held was dropped once");
}
