//! System parameters: queries, their rows, spawners and event readers and
//! writers. A system's parameters are its declaration: what it reads and
//! writes, and which changes it may make. See docs/architecture/storage.md.
//!
//! A query is `Query<Data, Filter, Changes>`. `Changes` lists the shape
//! changes rows from this query may make (`Adds<..>`, `Removes<..>`,
//! `Despawns`); a row only comes from its query, so the query's tables bound
//! the change before the system runs. Changes go into the system's log, in
//! call order, applied by its apply node after it returns.
//!
//! Each parameter holds the guards for what it declared, taken when the
//! system starts, so a query is iterated without borrowing anything else.

use std::cell::RefCell;
use std::marker::PhantomData;
use std::sync::Arc;

use crate::component::{Component, ComponentDesc, Entity, Storage};
use crate::events::{Event, EventQueue};
use crate::ordered::OrderKey;
use crate::spatial::{Bounds, PageKind, RUN};
use crate::world::{ColumnGuard, ComponentId, NewRow, SparseGuard, SparseSet, Structural, Table, TableId, TableRead, TakeGuard, World};

// ---- Declaring ----

/// Where parameters declare themselves: interns the components they name,
/// and collects their descriptions, which the loader installs if the build's
/// load commits.
pub struct Declare<'w> {
    pub world: &'w World,
    pub components: Vec<ComponentDesc>,
    pub events: Vec<ComponentDesc>,
    /// The first component that couldn't be interned (its storage changed).
    pub error: Option<String>,
}

impl<'w> Declare<'w> {
    pub fn new(world: &'w World) -> Declare<'w> {
        Declare { world, components: Vec::new(), events: Vec::new(), error: None }
    }

    pub fn component<T: Component>(&mut self) -> ComponentId {
        let desc = ComponentDesc::of::<T>();
        if !self.components.iter().any(|d| d.name == desc.name) {
            self.components.push(desc);
        }
        match self.world.intern(&desc) {
            Ok(id) => id,
            Err(e) => {
                self.error.get_or_insert(e);
                ComponentId(u32::MAX)
            }
        }
    }

    pub fn event<E: Event>(&mut self) -> usize {
        let desc = ComponentDesc::of::<E>();
        if !self.events.iter().any(|d| d.name == desc.name) {
            self.events.push(desc);
        }
        self.world.intern_event(&desc)
    }
}

/// What a running system's parameters are fetched with.
pub struct FrameCx<'w> {
    pub world: &'w World,
    pub log: &'w Log,
    /// `mod::system`: whose event cursors a reader advances.
    pub system: &'w str,
    /// Seconds this run covers: its phase's step if fixed-rate, else the
    /// frame's time. What `Dt` hands out.
    pub dt: f32,
}

/// Components named by type, one or a tuple: `Adds<Burning>`, `With<(A, B)>`.
pub trait ComponentSet: 'static {
    fn ids(d: &mut Declare<'_>) -> Vec<ComponentId>;
}

impl<T: Component> ComponentSet for T {
    fn ids(d: &mut Declare<'_>) -> Vec<ComponentId> {
        vec![d.component::<T>()]
    }
}

macro_rules! component_set {
    ($($t:ident),+) => {
        impl<$($t: Component),+> ComponentSet for ($($t,)+) {
            fn ids(d: &mut Declare<'_>) -> Vec<ComponentId> {
                vec![$(d.component::<$t>()),+]
            }
        }
    };
}
component_set!(A, B);
component_set!(A, B, C);
component_set!(A, B, C, D);

// ---- Filters ----

pub struct With<T>(PhantomData<T>);
pub struct Without<T>(PhantomData<T>);

#[derive(Clone, Debug, Default)]
pub struct FilterDecl {
    pub with: Vec<ComponentId>,
    pub without: Vec<ComponentId>,
}

pub trait Filter: 'static {
    fn declare(d: &mut Declare<'_>, out: &mut FilterDecl);
}

impl Filter for () {
    fn declare(_: &mut Declare<'_>, _: &mut FilterDecl) {}
}

impl<T: ComponentSet> Filter for With<T> {
    fn declare(d: &mut Declare<'_>, out: &mut FilterDecl) {
        out.with.extend(T::ids(d));
    }
}

impl<T: ComponentSet> Filter for Without<T> {
    fn declare(d: &mut Declare<'_>, out: &mut FilterDecl) {
        out.without.extend(T::ids(d));
    }
}

macro_rules! filter_tuple {
    ($($t:ident),+) => {
        impl<$($t: Filter),+> Filter for ($($t,)+) {
            fn declare(d: &mut Declare<'_>, out: &mut FilterDecl) {
                $($t::declare(d, out);)+
            }
        }
    };
}
filter_tuple!(A, B);
filter_tuple!(A, B, C);

// ---- Changes ----

pub struct Adds<T>(PhantomData<T>);
pub struct Removes<T>(PhantomData<T>);
pub struct Despawns;

#[derive(Clone, Debug, Default)]
pub struct ChangeDecl {
    pub adds: Vec<ComponentId>,
    pub removes: Vec<ComponentId>,
    pub despawns: bool,
}

impl ChangeDecl {
    pub fn is_empty(&self) -> bool {
        self.adds.is_empty() && self.removes.is_empty() && !self.despawns
    }
}

pub trait Changes: 'static {
    fn declare(d: &mut Declare<'_>, out: &mut ChangeDecl);
}

impl Changes for () {
    fn declare(_: &mut Declare<'_>, _: &mut ChangeDecl) {}
}

impl<T: ComponentSet> Changes for Adds<T> {
    fn declare(d: &mut Declare<'_>, out: &mut ChangeDecl) {
        out.adds.extend(T::ids(d));
    }
}

impl<T: ComponentSet> Changes for Removes<T> {
    fn declare(d: &mut Declare<'_>, out: &mut ChangeDecl) {
        out.removes.extend(T::ids(d));
    }
}

impl Changes for Despawns {
    fn declare(_: &mut Declare<'_>, out: &mut ChangeDecl) {
        out.despawns = true;
    }
}

macro_rules! changes_tuple {
    ($($t:ident),+) => {
        impl<$($t: Changes),+> Changes for ($($t,)+) {
            fn declare(d: &mut Declare<'_>, out: &mut ChangeDecl) {
                $($t::declare(d, out);)+
            }
        }
    };
}
changes_tuple!(A, B);
changes_tuple!(A, B, C);

// ---- Query data ----

/// A write to a component's value, handed out by a `&mut T` term: reads as
/// `&T`, and records the write (a tick on the row) the first time it's
/// written through, so change detection sees only what really changed.
pub struct Mut<'a, T> {
    value: &'a mut T,
    tick: &'a mut u32,
    now: u32,
}

impl<T> std::ops::Deref for Mut<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.value
    }
}

impl<T> std::ops::DerefMut for Mut<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        *self.tick = self.now;
        self.value
    }
}

impl<T: std::fmt::Debug> std::fmt::Debug for Mut<'_, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.value.fmt(f)
    }
}

/// What one term of a query holds while the system runs: a column guard per
/// matched table, or the component's sparse set; and, for a write, the tick
/// its writes are stamped with.
pub enum TermState<'w> {
    Table(Vec<ColumnGuard<'w>>, u32),
    Sparse(SparseGuard<'w>, u32),
}

impl<'w> TermState<'w> {
    fn take<T: Component>(world: &'w World, tables: &[TableId], c: ComponentId, write: bool) -> TermState<'w> {
        // A build compiled against another layout of `T` would misread it.
        assert!(
            world.installed_as(c, T::FINGERPRINT),
            "this build has another layout of {} than the one installed; reload it with the rest of the game",
            T::NAME
        );
        let now = if write { world.next_tick() } else { 0 };
        match T::STORAGE {
            Storage::Table => TermState::Table(
                tables
                    .iter()
                    .map(|&t| {
                        let table = world.table(t);
                        ColumnGuard::take(&table.columns[table.column_index(c).expect("a matched table")], write)
                    })
                    .collect(),
                now,
            ),
            Storage::Sparse => TermState::Sparse(SparseGuard::take(world.sparse_set(c), write), now),
        }
    }

    /// The entities of this term's sparse set, and how many, if it's sparse.
    fn sparse_entities(&self) -> Option<(usize, Vec<Entity>)> {
        match self {
            TermState::Sparse(set, _) => Some((set.set().len(), set.set().entities().to_vec())),
            TermState::Table(..) => None,
        }
    }

    fn sparse_len(&self) -> Option<usize> {
        match self {
            TermState::Sparse(set, _) => Some(set.set().len()),
            TermState::Table(..) => None,
        }
    }
}

/// One term's view of one page: a slice for a table term, taken once per
/// page; the set for a sparse term, looked up by entity per row.
pub enum PageView<'a, T> {
    Read(&'a [T]),
    Write(&'a mut [T], &'a mut [u32], u32),
    Sparse(&'a SparseSet),
    SparseMut(&'a mut SparseSet, u32),
}

/// One term: `&T` or `&mut T`, over a table or a sparse component.
pub trait Term {
    type C: Component;
    const WRITE: bool;
    type Item<'a>;
    /// The term's value for one entity, by its location: random access.
    fn item<'a>(state: &'a mut TermState<'_>, table: usize, page: usize, row: usize, e: Entity) -> Option<Self::Item<'a>>;
    /// The term's view of one page, for walking its rows in order.
    fn page<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> PageView<'a, Self::C>;
    fn at<'b>(page: &'b mut PageView<'_, Self::C>, row: usize, e: Entity) -> Option<Self::Item<'b>>;
}

impl<T: Component> Term for &T {
    type C = T;
    const WRITE: bool = false;
    type Item<'a> = &'a T;
    fn item<'a>(state: &'a mut TermState<'_>, table: usize, page: usize, row: usize, e: Entity) -> Option<&'a T> {
        match state {
            TermState::Table(guards, _) => Some(&guards[table].pages()[page].as_slice::<T>()[row]),
            TermState::Sparse(set, _) => set.set().get::<T>(e),
        }
    }
    fn page<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> PageView<'a, T> {
        match state {
            TermState::Table(guards, _) => PageView::Read(guards[table].pages()[page].as_slice::<T>()),
            TermState::Sparse(set, _) => PageView::Sparse(set.set()),
        }
    }
    fn at<'b>(page: &'b mut PageView<'_, T>, row: usize, e: Entity) -> Option<&'b T> {
        match page {
            PageView::Read(rows) => Some(&rows[row]),
            PageView::Sparse(set) => set.get::<T>(e),
            _ => unreachable!("a read term's view"),
        }
    }
}

impl<T: Component> Term for &mut T {
    type C = T;
    const WRITE: bool = true;
    type Item<'a> = Mut<'a, T>;
    fn item<'a>(state: &'a mut TermState<'_>, table: usize, page: usize, row: usize, e: Entity) -> Option<Mut<'a, T>> {
        match state {
            TermState::Table(guards, now) => {
                let (values, ticks) = guards[table].pages_mut()[page].as_mut_slice_ticked::<T>();
                Some(Mut { value: &mut values[row], tick: &mut ticks[row], now: *now })
            }
            TermState::Sparse(set, now) => {
                set.set_mut().get_mut_ticked::<T>(e).map(|(value, tick)| Mut { value, tick, now: *now })
            }
        }
    }
    fn page<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> PageView<'a, T> {
        match state {
            TermState::Table(guards, now) => {
                let (values, ticks) = guards[table].pages_mut()[page].as_mut_slice_ticked::<T>();
                let n = values.len();
                PageView::Write(values, &mut ticks[..n], *now)
            }
            TermState::Sparse(set, now) => PageView::SparseMut(set.set_mut(), *now),
        }
    }
    fn at<'b>(page: &'b mut PageView<'_, T>, row: usize, e: Entity) -> Option<Mut<'b, T>> {
        match page {
            PageView::Write(values, ticks, now) => Some(Mut { value: &mut values[row], tick: &mut ticks[row], now: *now }),
            PageView::SparseMut(set, now) => {
                let now = *now;
                set.get_mut_ticked::<T>(e).map(|(value, tick)| Mut { value, tick, now })
            }
            _ => unreachable!("a write term's view"),
        }
    }
}

/// A query's data: `()`, a term, or a tuple of up to eight.
pub trait Data {
    type Items<'a>;
    type States<'w>;
    type Pages<'a>;
    fn terms(d: &mut Declare<'_>) -> Vec<(ComponentId, bool)>;
    fn take<'w>(world: &'w World, tables: &[TableId], ids: &[(ComponentId, bool)]) -> Self::States<'w>;
    fn fetch<'a>(states: &'a mut Self::States<'_>, table: usize, page: usize, row: usize, e: Entity) -> Option<Self::Items<'a>>;
    /// The terms' views of one page, taken once per page.
    fn pages<'a>(states: &'a mut Self::States<'_>, table: usize, page: usize) -> Self::Pages<'a>;
    fn at<'b>(pages: &'b mut Self::Pages<'_>, row: usize, e: Entity) -> Option<Self::Items<'b>>;
    /// The entities of its smallest sparse term, and how many: what a query
    /// walks instead of its tables when that's fewer.
    fn driver(states: &Self::States<'_>) -> Option<(usize, Vec<Entity>)>;
}

impl Data for () {
    type Items<'a> = ();
    type States<'w> = ();
    type Pages<'a> = ();
    fn terms(_: &mut Declare<'_>) -> Vec<(ComponentId, bool)> {
        Vec::new()
    }
    fn take<'w>(_: &'w World, _: &[TableId], _: &[(ComponentId, bool)]) {}
    fn fetch<'a>(_: &'a mut (), _: usize, _: usize, _: usize, _: Entity) -> Option<()> {
        Some(())
    }
    fn pages<'a>(_: &'a mut (), _: usize, _: usize) {}
    fn at<'b>(_: &'b mut (), _: usize, _: Entity) -> Option<()> {
        Some(())
    }
    fn driver(_: &()) -> Option<(usize, Vec<Entity>)> {
        None
    }
}

impl<A: Term> Data for A {
    type Items<'a> = A::Item<'a>;
    type States<'w> = TermState<'w>;
    type Pages<'a> = PageView<'a, A::C>;
    fn terms(d: &mut Declare<'_>) -> Vec<(ComponentId, bool)> {
        vec![(d.component::<A::C>(), A::WRITE)]
    }
    fn take<'w>(world: &'w World, tables: &[TableId], ids: &[(ComponentId, bool)]) -> TermState<'w> {
        TermState::take::<A::C>(world, tables, ids[0].0, A::WRITE)
    }
    fn fetch<'a>(s: &'a mut TermState<'_>, t: usize, p: usize, r: usize, e: Entity) -> Option<A::Item<'a>> {
        A::item(s, t, p, r, e)
    }
    fn pages<'a>(s: &'a mut TermState<'_>, t: usize, p: usize) -> PageView<'a, A::C> {
        A::page(s, t, p)
    }
    fn at<'b>(page: &'b mut PageView<'_, A::C>, r: usize, e: Entity) -> Option<A::Item<'b>> {
        A::at(page, r, e)
    }
    fn driver(s: &TermState<'_>) -> Option<(usize, Vec<Entity>)> {
        s.sparse_entities()
    }
}

macro_rules! data_tuple {
    ($($t:ident : $s:ident : $i:tt),+) => {
        impl<$($t: Term),+> Data for ($($t,)+) {
            type Items<'a> = ($($t::Item<'a>,)+);
            type States<'w> = ($(data_tuple!(@state $t, 'w),)+);
            type Pages<'a> = ($(PageView<'a, $t::C>,)+);
            fn terms(d: &mut Declare<'_>) -> Vec<(ComponentId, bool)> {
                vec![$((d.component::<$t::C>(), $t::WRITE)),+]
            }
            fn take<'w>(world: &'w World, tables: &[TableId], ids: &[(ComponentId, bool)]) -> Self::States<'w> {
                ($(TermState::take::<$t::C>(world, tables, ids[$i].0, $t::WRITE),)+)
            }
            fn fetch<'a>(states: &'a mut Self::States<'_>, t: usize, p: usize, r: usize, e: Entity) -> Option<Self::Items<'a>> {
                // Each term's state is its own field, so each item borrows
                // its own guard.
                let ($($s,)+) = states;
                Some(($($t::item($s, t, p, r, e)?,)+))
            }
            fn pages<'a>(states: &'a mut Self::States<'_>, t: usize, p: usize) -> Self::Pages<'a> {
                let ($($s,)+) = states;
                ($($t::page($s, t, p),)+)
            }
            fn at<'b>(pages: &'b mut Self::Pages<'_>, r: usize, e: Entity) -> Option<Self::Items<'b>> {
                let ($($s,)+) = pages;
                Some(($($t::at($s, r, e)?,)+))
            }
            fn driver(states: &Self::States<'_>) -> Option<(usize, Vec<Entity>)> {
                let ($($s,)+) = states;
                let smallest = [$($s.sparse_len().map(|n| (n, $s as &TermState))),+]
                    .into_iter()
                    .flatten()
                    .min_by_key(|(n, _)| *n)?;
                smallest.1.sparse_entities()
            }
        }
    };
    (@state $t:ident, $w:lifetime) => { TermState<$w> };
}
data_tuple!(A: a: 0);
data_tuple!(A: a: 0, B: b: 1);
data_tuple!(A: a: 0, B: b: 1, C: c: 2);
data_tuple!(A: a: 0, B: b: 1, C: c: 2, D: d: 3);
data_tuple!(A: a: 0, B: b: 1, C: c: 2, D: d: 3, E: e_: 4);
data_tuple!(A: a: 0, B: b: 1, C: c: 2, D: d: 3, E: e_: 4, F: f: 5);
data_tuple!(A: a: 0, B: b: 1, C: c: 2, D: d: 3, E: e_: 4, F: f: 5, G: g: 6);
data_tuple!(A: a: 0, B: b: 1, C: c: 2, D: d: 3, E: e_: 4, F: f: 5, G: g: 6, H: h: 7);

// ---- Declarations ----

#[derive(Clone, Debug)]
pub struct QueryDecl {
    /// Data terms, and whether each is written.
    pub terms: Vec<(ComponentId, bool)>,
    pub filter: FilterDecl,
    pub changes: ChangeDecl,
    /// Writes a spatial key or extent, so its spatial tables re-sort after
    /// the system: a change like its rows', with an apply node.
    pub reorders: bool,
}

impl QueryDecl {
    fn of(world: &World, ids: impl Iterator<Item = ComponentId>, storage: Storage) -> Vec<ComponentId> {
        ids.filter(|c| world.storage(*c) == storage).collect()
    }

    /// Table components an entity must have: table terms and `With`.
    pub fn table_with(&self, world: &World) -> Vec<ComponentId> {
        Self::of(world, self.terms.iter().map(|t| t.0).chain(self.filter.with.iter().copied()), Storage::Table)
    }

    pub fn table_without(&self, world: &World) -> Vec<ComponentId> {
        Self::of(world, self.filter.without.iter().copied(), Storage::Table)
    }

    /// Sparse sets the query reads or writes: its sparse terms, and its
    /// sparse filters (reads of membership).
    pub fn sparse(&self, world: &World) -> Vec<(ComponentId, bool)> {
        let terms = self.terms.iter().copied().filter(|(c, _)| world.storage(*c) == Storage::Sparse);
        let filters = self.filter.with.iter().chain(&self.filter.without).copied();
        terms.chain(filters.filter(|c| world.storage(*c) == Storage::Sparse).map(|c| (c, false))).collect()
    }

    /// Driven by a sparse set rather than tables: sparse terms and no table
    /// terms. Such a query iterates the set, and observes who's alive.
    pub fn sparse_driven(&self, world: &World) -> bool {
        let table_terms = self.terms.iter().any(|(c, _)| world.storage(*c) == Storage::Table);
        !table_terms && self.terms.iter().any(|(c, _)| world.storage(*c) == Storage::Sparse)
    }

    /// Whether the query uses tables at all: to walk them, or to check the
    /// entities its sparse set yields against table filters.
    pub fn uses_tables(&self, world: &World) -> bool {
        !self.sparse_driven(world) || !self.table_with(world).is_empty() || !self.table_without(world).is_empty()
    }

    /// Whether the query reads the rows of a table with exactly `set`.
    pub fn matches(&self, world: &World, set: &[ComponentId]) -> bool {
        self.uses_tables(world)
            && self.table_with(world).iter().all(|c| set.contains(c))
            && !self.table_without(world).iter().any(|c| set.contains(c))
    }
}

#[derive(Clone, Debug)]
pub enum ParamDecl {
    Query(QueryDecl),
    Spawner { components: Vec<ComponentId> },
    /// An event queue, read or written.
    Events { queue: usize, write: bool },
    /// A parameter made of others (a tuple of parameters, or a crate's own
    /// parameter built from them): its footprint is its members'.
    Group(Vec<ParamDecl>),
    /// The run's length in time: touches nothing.
    Dt,
}

impl ParamDecl {
    /// Every parameter `params` declare, groups flattened: what footprints
    /// and conflict checks look at, so a group is never a way around them.
    pub fn leaves(params: &[ParamDecl]) -> Vec<&ParamDecl> {
        let mut out = Vec::new();
        for p in params {
            match p {
                ParamDecl::Group(members) => out.extend(ParamDecl::leaves(members)),
                p => out.push(p),
            }
        }
        out
    }

    pub fn query(&self) -> Option<&QueryDecl> {
        match self {
            ParamDecl::Query(q) => Some(q),
            _ => None,
        }
    }

    /// Whether this parameter can change the world, needing an apply node.
    pub fn changes(&self) -> bool {
        match self {
            ParamDecl::Query(q) => !q.changes.is_empty() || q.reorders,
            ParamDecl::Spawner { .. } => true,
            ParamDecl::Events { write, .. } => *write,
            ParamDecl::Group(members) => members.iter().any(ParamDecl::changes),
            ParamDecl::Dt => false,
        }
    }
}

// ---- Changes, as logged ----

type Apply = Box<dyn for<'x> FnOnce(&mut Structural<'x>) + Send>;
type Publish = Box<dyn FnOnce(&mut EventQueue, u64) + Send>;

/// One change a system made. A system's changes are applied in the order it
/// made them, by its apply node.
pub enum Change {
    Insert { e: Entity, c: ComponentId, apply: Apply },
    Remove { e: Entity, c: ComponentId },
    Despawn(Entity),
    Spawn { e: Entity, components: Arc<[ComponentId]>, apply: Apply },
    Event { queue: usize, publish: Publish },
    /// A spatial table whose keys or extents the system could write, to
    /// re-sort after it.
    Reorder(TableId),
}

impl Change {
    pub fn apply(self, s: &mut Structural<'_>) {
        match self {
            Change::Insert { apply, .. } | Change::Spawn { apply, .. } => apply(s),
            Change::Remove { e, c } => s.remove_id(e, c),
            Change::Despawn(e) => s.despawn(e),
            Change::Event { queue, publish } => {
                let frame = s.world.frame();
                publish(s.events(queue), frame)
            }
            Change::Reorder(t) => s.resort(t),
        }
    }
}

pub type Log = RefCell<Vec<Change>>;

/// An entity a query yielded, through which its shape is changed. The
/// changes land after the system returns, so the system itself never sees
/// them.
pub struct Row<'a> {
    entity: Entity,
    world: &'a World,
    changes: &'a ChangeDecl,
    log: &'a Log,
}

/// The id of `T` among `ids`, by name: the declaration interned it.
fn declared<T: Component>(world: &World, ids: &[ComponentId]) -> Option<ComponentId> {
    ids.iter().copied().find(|&c| world.name(c) == T::NAME)
}

impl Row<'_> {
    pub fn entity(&self) -> Entity {
        self.entity
    }

    pub fn insert<T: Component>(&self, value: T) {
        let c = declared::<T>(self.world, &self.changes.adds)
            .unwrap_or_else(|| panic!("{} isn't in this query's Adds", T::NAME));
        let e = self.entity;
        self.log.borrow_mut().push(Change::Insert { e, c, apply: Box::new(move |s| s.insert_id(e, c, value)) });
    }

    pub fn remove<T: Component>(&self) {
        let c = declared::<T>(self.world, &self.changes.removes)
            .unwrap_or_else(|| panic!("{} isn't in this query's Removes", T::NAME));
        self.log.borrow_mut().push(Change::Remove { e: self.entity, c });
    }

    pub fn despawn(&self) {
        assert!(self.changes.despawns, "this query doesn't declare Despawns");
        self.log.borrow_mut().push(Change::Despawn(self.entity));
    }
}

/// Sorts pairs as `(Entity, Entity)` would, but by a radix sort of their
/// indices: live entities never share an index, so indices alone order
/// them, and a broadphase's tens of thousands of pairs sort in a few
/// passes instead of `n log n` comparisons. Only the bytes that differ
/// between keys are passed over.
fn sort_pairs(pairs: &mut Vec<(Entity, Entity)>) {
    let key = |(a, b): &(Entity, Entity)| ((a.index as u64) << 32) | b.index as u64;
    let (mut any, mut all) = (0u64, u64::MAX);
    for p in pairs.iter() {
        any |= key(p);
        all &= key(p);
    }
    let differ = any ^ all;
    let mut buf = pairs.clone();
    for byte in 0..8 {
        let shift = byte * 8;
        if (differ >> shift) & 0xff == 0 {
            continue;
        }
        let mut count = [0usize; 257];
        for p in pairs.iter() {
            count[((key(p) >> shift) & 0xff) as usize + 1] += 1;
        }
        for i in 1..257 {
            count[i] += count[i - 1];
        }
        for p in pairs.iter() {
            let d = ((key(p) >> shift) & 0xff) as usize;
            buf[count[d]] = *p;
            count[d] += 1;
        }
        std::mem::swap(pairs, &mut buf);
    }
}

/// Pairs within one page's rows, sorted by left edge: each row meets the
/// rows after it until one starts past its right edge.
fn sweep_one(rows: &[(Bounds, Entity)], out: &mut Vec<(Entity, Entity)>) {
    for (i, (a, e)) in rows.iter().enumerate() {
        for (b, f) in &rows[i + 1..] {
            if b.min[0] > a.max[0] {
                break;
            }
            if a.overlaps(b) {
                out.push((*e.min(f), *e.max(f)));
            }
        }
    }
}

/// Pairs between two pages' rows, each sorted by left edge: one sweep over
/// both in order, each row meeting the other side's rows still open.
fn sweep_two(
    a: &[(Bounds, Entity)],
    b: &[(Bounds, Entity)],
    (open_a, open_b): (&mut Vec<usize>, &mut Vec<usize>),
    out: &mut Vec<(Entity, Entity)>,
) {
    let (mut i, mut j) = (0, 0);
    open_a.clear();
    open_b.clear();
    while i < a.len() || j < b.len() {
        let from_a = j == b.len() || (i < a.len() && a[i].0.min[0] <= b[j].0.min[0]);
        let (mine, theirs, open_mine, open_theirs, k) =
            if from_a { (a, b, &mut *open_a, &mut *open_b, i) } else { (b, a, &mut *open_b, &mut *open_a, j) };
        let (bx, e) = mine[k];
        open_theirs.retain(|&t| theirs[t].0.max[0] >= bx.min[0]);
        for &t in open_theirs.iter() {
            let (by, f) = theirs[t];
            if bx.overlaps(&by) {
                out.push((e.min(f), e.max(f)));
            }
        }
        open_mine.push(k);
        if from_a {
            i += 1;
        } else {
            j += 1;
        }
    }
}

// ---- Parameters ----

pub struct Query<'w, D: Data, F = (), C = ()> {
    world: &'w World,
    decl: &'w QueryDecl,
    table_ids: Vec<TableId>,
    rows: Vec<TableRead<'w>>,
    states: D::States<'w>,
    /// Sparse sets its filters check: (component, with).
    filters: Vec<(ComponentId, bool, SparseGuard<'w>)>,
    log: &'w Log,
    _marker: PhantomData<fn() -> (F, C)>,
}

impl<'w, D: Data, F, C> Query<'w, D, F, C> {
    pub(crate) fn take(world: &'w World, decl: &'w QueryDecl, log: &'w Log) -> Self {
        let table_ids: Vec<TableId> =
            world.tables().filter(|t| decl.matches(world, &t.components)).map(|t| t.id).collect();
        let rows = table_ids.iter().map(|&t| TableRead::new(world.table(t))).collect();
        let states = D::take(world, &table_ids, &decl.terms);
        let is_sparse = |c: &&ComponentId| world.storage(**c) == Storage::Sparse;
        let filters = decl
            .filter
            .with
            .iter()
            .filter(is_sparse)
            .map(|&c| (c, true))
            .chain(decl.filter.without.iter().filter(is_sparse).map(|&c| (c, false)))
            .map(|(c, with)| (c, with, SparseGuard::take(world.sparse_set(c), false)))
            .collect();
        Query { world, decl, table_ids, rows, states, filters, log, _marker: PhantomData }
    }

    /// Logs a re-sort of each spatial or ordered table this query can write
    /// keys or extents in: it writes in place, so its apply node puts rows
    /// back in order afterwards.
    pub(crate) fn log_reorders(&self) {
        if !self.decl.reorders {
            return;
        }
        for &t in &self.table_ids {
            let table = self.world.table(t);
            if table.spatial.is_some() || table.ordered.is_some() {
                self.log.borrow_mut().push(Change::Reorder(t));
            }
        }
    }

    /// The empty case first, and apart: most queries have no sparse
    /// filters, and without it the check (a call per row) kept walks from
    /// being unswitched and vectorized; a one-term walk measured 4x
    /// faster with it (2026-09-24).
    fn passes(filters: &[(ComponentId, bool, SparseGuard<'_>)], e: Entity) -> bool {
        filters.is_empty() || filters.iter().all(|(_, with, set)| set.set().contains(e) == *with)
    }

    fn row<'a>(world: &'a World, decl: &'a QueryDecl, log: &'a Log, e: Entity) -> Row<'a> {
        Row { entity: e, world, changes: &decl.changes, log }
    }

    /// Every entity the query matches, with its row and values.
    pub fn for_each(&mut self, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let Query { world, decl, table_ids, rows, states, filters, log, .. } = self;
        // Walk whichever is smaller: the matched tables' rows, or the
        // smallest sparse term's set (which a query with no table terms
        // always walks).
        let table_rows: usize = rows.iter().map(|t| t.rows.iter().map(Vec::len).sum::<usize>()).sum();
        let driver = D::driver(states).filter(|(n, _)| decl.sparse_driven(world) || *n < table_rows);
        if let Some((_, entities)) = driver {
            let uses_tables = decl.uses_tables(world);
            for e in entities {
                if !world.entities.is_alive(e) || !Self::passes(filters, e) {
                    continue;
                }
                // Random access into the matched tables, by location.
                let (t, p, r) = if uses_tables {
                    let Some(at) = world.entities.location(e) else { continue };
                    let Some(t) = table_ids.iter().position(|&id| id == at.table) else { continue };
                    (t, at.page as usize, at.row as usize)
                } else {
                    (usize::MAX, 0, 0)
                };
                if let Some(items) = D::fetch(states, t, p, r, e) {
                    f(Self::row(world, decl, log, e), items);
                }
            }
            return;
        }
        for (t, table_rows) in rows.iter().enumerate() {
            for (p, page) in table_rows.rows.iter().enumerate() {
                // Each term's slice once per page, then plain indexing.
                let mut pages = D::pages(states, t, p);
                for (r, &e) in page.iter().enumerate() {
                    if !Self::passes(filters, e) {
                        continue;
                    }
                    if let Some(items) = D::at(&mut pages, r, e) {
                        f(Self::row(world, decl, log, e), items);
                    }
                }
            }
        }
    }

    /// Every entity the query matches whose key `K` is in `keys`: a seek in
    /// tables kept in `K`'s order, which come out in key order, and a scan
    /// of tables that hold `K` but are kept in another (a spatial one: a
    /// table has one order), which come out in theirs. The query must read
    /// `K`. Keys are as of the last re-sort, as spatial boxes are.
    pub fn in_keys<K: OrderKey>(&mut self, keys: std::ops::RangeInclusive<u128>, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let Query { world, decl, rows, states, filters, log, .. } = self;
        let k = world.id(K::NAME).filter(|&k| decl.terms.iter().any(|&(c, write)| c == k && !write));
        let k = k.unwrap_or_else(|| panic!("in_keys::<{}> is for a query that reads {0}", K::NAME));
        for (t, table) in rows.iter().enumerate() {
            let visit = |r: usize, e: Entity, pages: &mut D::Pages<'_>, f: &mut dyn FnMut(Row<'_>, D::Items<'_>)| {
                if Self::passes(filters, e)
                    && let Some(items) = D::at(pages, r, e)
                {
                    f(Self::row(world, decl, log, e), items);
                }
            };
            match (&table.ordered, table.table.ordered.as_ref().is_some_and(|o| o.key == k)) {
                (Some(order), true) => {
                    let (mut p, mut r) = order.seek(*keys.start());
                    'pages: while p < table.rows.len() {
                        let mut pages = D::pages(states, t, p);
                        while r < table.rows[p].len() {
                            if order.keys[p][r] > *keys.end() {
                                break 'pages;
                            }
                            visit(r, table.rows[p][r], &mut pages, &mut f);
                            r += 1;
                        }
                        (p, r) = (p + 1, 0);
                    }
                }
                _ => {
                    let Some(i) = table.table.column_index(k) else { continue };
                    // The query reads `K`, so its footprint already covers
                    // this column; a second read guard beside its own.
                    let column = table.table.columns[i].take_read();
                    for (p, page) in table.rows.iter().enumerate() {
                        let values = column[p].as_slice::<K>();
                        let mut pages = D::pages(states, t, p);
                        for (r, &e) in page.iter().enumerate() {
                            if keys.contains(&values[r].key()) {
                                visit(r, e, &mut pages, &mut f);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Every entity the query matches, in key order across all its ordered
    /// tables (ties by entity): `for_each` walks table by table, so rows of
    /// one key in two tables would come out as two sorted runs. Tables that
    /// aren't ordered come after, in their own order.
    pub fn for_each_ordered(&mut self, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let Query { world, decl, rows, states, filters, log, .. } = self;
        // Not `for_each`, which may walk a sparse set's order instead.
        fn walk<'w, D: Data, F, C>(
            (world, decl, log, filters): (&World, &QueryDecl, &Log, &[(ComponentId, bool, SparseGuard<'_>)]),
            t: usize,
            table: &TableRead<'_>,
            states: &mut D::States<'w>,
            f: &mut impl FnMut(Row<'_>, D::Items<'_>),
        ) {
            for (p, page) in table.rows.iter().enumerate() {
                let mut pages = D::pages(states, t, p);
                for (r, &e) in page.iter().enumerate() {
                    if Query::<D, F, C>::passes(filters, e)
                        && let Some(items) = D::at(&mut pages, r, e)
                    {
                        f(Query::<D, F, C>::row(world, decl, log, e), items);
                    }
                }
            }
        }
        let cx = (*world, *decl, *log, filters.as_slice());
        let ordered: Vec<usize> = (0..rows.len()).filter(|&t| rows[t].ordered.is_some()).collect();
        if ordered.len() <= 1 {
            // One sorted run already: no merge.
            for &t in &ordered {
                walk::<D, F, C>(cx, t, &rows[t], states, &mut f);
            }
        } else {
            let mut all: Vec<(u128, Entity, u32, u32, u32)> = Vec::new();
            for &t in &ordered {
                let (table, order) = (&rows[t], rows[t].ordered.as_ref().expect("ordered"));
                for (p, page) in table.rows.iter().enumerate() {
                    all.extend(page.iter().enumerate().map(|(r, &e)| (order.keys[p][r], e, t as u32, p as u32, r as u32)));
                }
            }
            // One sorted run per table: a stable sort merges them.
            all.sort_by(|x, y| (x.0, x.1).cmp(&(y.0, y.1)));
            for (_, e, t, p, r) in all {
                if Self::passes(filters, e)
                    && let Some(items) = D::fetch(states, t as usize, p as usize, r as usize, e)
                {
                    f(Self::row(world, decl, log, e), items);
                }
            }
        }
        for t in (0..rows.len()).filter(|&t| rows[t].ordered.is_none()) {
            walk::<D, F, C>(cx, t, &rows[t], states, &mut f);
        }
    }

    /// Every entity the query matches in its spatial tables whose box meets
    /// `region`, as `for_each` would hand it. Tables that aren't spatial
    /// have no boxes, and are skipped. Boxes are as of the last re-sort:
    /// after whoever last wrote the keys, never mid-system.
    pub fn in_region(&mut self, region: Bounds, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let Query { world, decl, rows, states, filters, log, .. } = self;
        for (t, table) in rows.iter().enumerate() {
            let Some(order) = &table.spatial else { continue };
            for p in order.pages_near(&region) {
                let mut pages = D::pages(states, t, p);
                for (r, &e) in table.rows[p].iter().enumerate() {
                    if !order.row_bounds[p][r].overlaps(&region) || !Self::passes(filters, e) {
                        continue;
                    }
                    if let Some(items) = D::at(&mut pages, r, e) {
                        f(Self::row(world, decl, log, e), items);
                    }
                }
            }
        }
    }

    /// Every pair of entities the query matches in its spatial tables whose
    /// boxes, grown by `grow`, meet: a broadphase. Each pair once, the
    /// lesser entity first, sorted.
    pub fn near_pairs(&mut self, grow: f32) -> Vec<(Entity, Entity)> {
        let Query { rows, filters, .. } = self;
        // Groups of pages to test against each other before their pages:
        // each run of ordered pages, and each big page on its own.
        let mut groups: Vec<(usize, Vec<usize>, Bounds)> = Vec::new();
        for (t, table) in rows.iter().enumerate() {
            let Some(order) = &table.spatial else { continue };
            for (run, b) in order.order.chunks(RUN).zip(&order.runs) {
                groups.push((t, run.iter().map(|&p| p as usize).collect(), b.grown(grow)));
            }
            for p in (0..order.kind.len()).filter(|&p| order.kind[p] != PageKind::Ordered) {
                groups.push((t, vec![p], order.bounds[p].grown(grow)));
            }
        }
        // Each page's rows that pass the filters, their boxes grown, sorted
        // by left edge: the pairs of two pages are then a sweep along x,
        // not every row against every row.
        let swept: Vec<Vec<Vec<(Bounds, Entity)>>> = rows
            .iter()
            .map(|t| {
                t.spatial.as_ref().map_or_else(Vec::new, |o| {
                    (0..t.rows.len())
                        .map(|p| {
                            let mut page: Vec<(Bounds, Entity)> = t.rows[p]
                                .iter()
                                .zip(&o.row_bounds_all()[p])
                                .filter(|(e, _)| filters.is_empty() || Self::passes(filters, **e))
                                .map(|(&e, b)| (b.grown(grow), e))
                                .collect();
                            page.sort_by(|a, b| a.0.min[0].total_cmp(&b.0.min[0]));
                            page
                        })
                        .collect()
                })
            })
            .collect();
        let mut out = Vec::new();
        let (mut open_a, mut open_b) = (Vec::new(), Vec::new());
        for (i, (ta, pa, ba)) in groups.iter().enumerate() {
            for (j, (tb, pb, bb)) in groups.iter().enumerate().skip(i) {
                if !ba.overlaps(bb) {
                    continue;
                }
                let (oa, ob) = (rows[*ta].spatial.as_ref().unwrap(), rows[*tb].spatial.as_ref().unwrap());
                for (x, &p) in pa.iter().enumerate() {
                    let from = if i == j { x } else { 0 };
                    let page_a = oa.bounds[p].grown(grow);
                    for &q in &pb[from..] {
                        if !page_a.overlaps(&ob.bounds[q].grown(grow)) {
                            continue;
                        }
                        if i == j && p == q {
                            sweep_one(&swept[*ta][p], &mut out);
                        } else {
                            sweep_two(&swept[*ta][p], &swept[*tb][q], (&mut open_a, &mut open_b), &mut out);
                        }
                    }
                }
            }
        }
        sort_pairs(&mut out);
        out
    }

    /// The row of `e`, if the query matches it: how a system changes an
    /// entity it didn't iterate to (one kept in state, or named by an event).
    pub fn get(&mut self, e: Entity) -> Option<Row<'_>> {
        self.with(e, |_, _| ())?;
        Some(Self::row(self.world, self.decl, self.log, e))
    }

    /// Calls `f` with `e`'s row and values, if the query matches it.
    pub fn with<R>(&mut self, e: Entity, f: impl FnOnce(Row<'_>, D::Items<'_>) -> R) -> Option<R> {
        let Query { world, decl, table_ids, states, filters, log, .. } = self;
        if !world.entities.is_alive(e) || !Self::passes(filters, e) {
            return None;
        }
        let at = world.entities.location(e);
        let (t, p, r) = if decl.sparse_driven(world) {
            if decl.uses_tables(world) && !at.is_some_and(|at| table_ids.contains(&at.table)) {
                return None;
            }
            (usize::MAX, 0, 0)
        } else {
            let at = at?;
            let t = table_ids.iter().position(|&id| id == at.table)?;
            (t, at.page as usize, at.row as usize)
        };
        let items = D::fetch(states, t, p, r, e)?;
        Some(f(Self::row(world, decl, log, e), items))
    }

    /// The first entity the query matches, with its values: for a query
    /// meant to match one (a clock, a level).
    pub fn single<R>(&mut self, f: impl FnOnce(Row<'_>, D::Items<'_>) -> R) -> Option<R> {
        let mut f = Some(f);
        let mut out = None;
        self.for_each(|row, items| {
            if let Some(f) = f.take() {
                out = Some(f(row, items));
            }
        });
        out
    }
}

/// Spawns entities with the components `B` names.
pub struct Spawner<'w, B> {
    world: &'w World,
    /// Shared by every spawn this run logs, so the apply resolves their
    /// table once.
    components: Arc<[ComponentId]>,
    log: &'w Log,
    _marker: PhantomData<fn() -> B>,
}

impl<B: Bundle> Spawner<'_, B> {
    /// A new entity, with its id now; its components land after the system.
    pub fn spawn(&self, bundle: B) -> Entity {
        let e = self.world.entities.reserve();
        let ids = self.components.clone();
        let components = ids.clone();
        self.log.borrow_mut().push(Change::Spawn { e, components, apply: Box::new(move |s| s.spawn_shared(e, bundle, &ids)) });
        e
    }
}

/// Something a system can take as a parameter.
pub trait Param: 'static {
    type Item<'w>;
    fn declare(d: &mut Declare<'_>) -> ParamDecl;
    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Self::Item<'w>;
}

impl<D: Data + 'static, F: Filter, C: Changes> Param for Query<'static, D, F, C> {
    type Item<'w> = Query<'w, D, F, C>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        let terms = D::terms(d);
        for (i, (c, _)) in terms.iter().enumerate() {
            // Two guards on one column would contend with each other.
            assert!(!terms[..i].iter().any(|(x, _)| x == c), "a query names a component twice");
        }
        let mut filter = FilterDecl::default();
        F::declare(d, &mut filter);
        let mut changes = ChangeDecl::default();
        C::declare(d, &mut changes);
        let reorders = terms.iter().any(|&(c, write)| write && d.world.moves_rows(c));
        ParamDecl::Query(QueryDecl { terms, filter, changes, reorders })
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Query<'w, D, F, C> {
        let q = Query::take(cx.world, decl.query().expect("a query's declaration"), cx.log);
        q.log_reorders();
        q
    }
}

impl<B: Bundle> Param for Spawner<'static, B> {
    type Item<'w> = Spawner<'w, B>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Spawner { components: B::components(d) }
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Spawner<'w, B> {
        let ParamDecl::Spawner { components } = decl else { panic!("a spawner's declaration") };
        Spawner { world: cx.world, components: components.as_slice().into(), log: cx.log, _marker: PhantomData }
    }
}

/// A tuple of parameters is a parameter: how a crate builds its own from
/// existing ones (declare the tuple, fetch it, wrap the items), without
/// touching the footprint rules, which see the members.
macro_rules! param_tuple {
    ($($p:ident),+) => {
        impl<$($p: Param),+> Param for ($($p,)+) {
            type Item<'w> = ($($p::Item<'w>,)+);

            fn declare(d: &mut Declare<'_>) -> ParamDecl {
                ParamDecl::Group(vec![$($p::declare(d)),+])
            }

            #[allow(non_snake_case)]
            fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Self::Item<'w> {
                let ParamDecl::Group(members) = decl else { panic!("a tuple's declaration") };
                let mut members = members.iter();
                ($($p::fetch(cx, members.next().expect("one declaration per member")),)+)
            }
        }
    };
}
param_tuple!(A);
param_tuple!(A, B);
param_tuple!(A, B, C);
param_tuple!(A, B, C, D);

/// Whether no table can match both queries: one requires a table-stored
/// component the other excludes. Their guards on a shared table-stored
/// component are then on different tables' columns, so they never meet.
/// A sparse component doesn't count: a sparse set is one guard for every
/// entity in it, whatever tables they're in.
fn disjoint(world: &World, a: &QueryDecl, b: &QueryDecl) -> bool {
    let table = |c: &ComponentId| world.storage(*c) == Storage::Table;
    let requires = |q: &QueryDecl, c: ComponentId| q.terms.iter().any(|&(t, _)| t == c) || q.filter.with.contains(&c);
    let excludes = |x: &QueryDecl, y: &QueryDecl| x.filter.without.iter().filter(|c| table(c)).any(|&c| requires(y, c));
    excludes(a, b) || excludes(b, a)
}

/// Seconds this run of the system covers: in a fixed-rate phase its step
/// (1/60 in `simulate`), else the frame's time. Reads as an `f32`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dt(pub f32);

impl std::ops::Deref for Dt {
    type Target = f32;
    fn deref(&self) -> &f32 {
        &self.0
    }
}

impl Param for Dt {
    type Item<'w> = Dt;

    fn declare(_: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Dt
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> Dt {
        Dt(cx.dt)
    }
}

/// Refuses two queries of one system that would take conflicting guards:
/// both on one component, one writing, and not kept apart by their filters
/// (`With<A>` and `Without<A>`; see `disjoint`).
pub fn check_conflicts(world: &World, name: &str, params: &[ParamDecl]) -> Result<(), String> {
    let leaves = ParamDecl::leaves(params);
    let queries: Vec<&QueryDecl> = leaves.iter().filter_map(|p| p.query()).collect();
    for (i, a) in queries.iter().enumerate() {
        for b in &queries[i + 1..] {
            for &(c, wa) in &a.terms {
                let apart = world.storage(c) == Storage::Table && disjoint(world, a, b);
                if !apart && b.terms.iter().any(|&(d, wb)| c == d && (wa || wb)) {
                    return Err(format!("{name}: two queries access {} and one writes it", world.name(c)));
                }
            }
        }
    }
    let queues: Vec<(usize, bool)> = leaves
        .iter()
        .filter_map(|p| match p {
            ParamDecl::Events { queue, write } => Some((*queue, *write)),
            _ => None,
        })
        .collect();
    for (i, &(q, w)) in queues.iter().enumerate() {
        if queues[i + 1..].iter().any(|&(r, v)| q == r && (w || v)) {
            return Err(format!("{name}: reads and writes one event type, or writes it twice"));
        }
    }
    Ok(())
}

// ---- Spawning ----

/// A bundle of components to spawn with: a tuple of one to eight. A
/// physics body alone is four (position, velocity, body, collider).
pub trait Bundle: Send + 'static {
    fn components(d: &mut Declare<'_>) -> Vec<ComponentId>;
    fn put(self, sink: &mut BundleSink<'_, '_>);
}

type SparseInsert = Box<dyn for<'x> FnOnce(&mut Structural<'x>, Entity) + Send>;

/// Where a bundle's values go: table components into the new row, sparse
/// ones into their sets after it.
pub struct BundleSink<'s, 'c> {
    table: &'s Table,
    /// The bundle's declared ids, in its order: `put` is called in that
    /// order, so the `n`th value is `ids[n]` without comparing names.
    ids: &'s [ComponentId],
    next: usize,
    row: NewRow<'s, 'c>,
    sparse: Vec<SparseInsert>,
}

impl BundleSink<'_, '_> {
    pub fn put<T: Component>(&mut self, value: T) {
        let c = self.ids[self.next];
        self.next += 1;
        match self.table.column_index(c) {
            Some(i) => self.row.column(i).push(value),
            None => self.sparse.push(Box::new(move |s, e| s.insert_id(e, c, value))),
        }
    }
}

macro_rules! bundle {
    ($($t:ident),+) => {
        impl<$($t: Component),+> Bundle for ($($t,)+) {
            fn components(d: &mut Declare<'_>) -> Vec<ComponentId> {
                vec![$(d.component::<$t>()),+]
            }
            #[allow(non_snake_case)]
            fn put(self, sink: &mut BundleSink<'_, '_>) {
                let ($($t,)+) = self;
                $(sink.put($t);)+
            }
        }
    };
}
/// No components: an entity to add them to later, or one that only marks
/// something by existing.
impl Bundle for () {
    fn components(_: &mut Declare<'_>) -> Vec<ComponentId> {
        Vec::new()
    }
    fn put(self, _: &mut BundleSink<'_, '_>) {}
}
bundle!(A);
bundle!(A, B);
bundle!(A, B, C);
bundle!(A, B, C, D);
bundle!(A, B, C, D, E);
bundle!(A, B, C, D, E, F);
bundle!(A, B, C, D, E, F, G);
bundle!(A, B, C, D, E, F, G, H);

impl<'w> Structural<'w> {
    /// Places reserved entity `e` with `bundle`'s components, `ids` being
    /// their declared ids.
    pub fn spawn<B: Bundle>(&mut self, e: Entity, bundle: B, ids: &[ComponentId]) {
        let world = self.world;
        let table_components: Vec<ComponentId> =
            ids.iter().copied().filter(|c| world.storage(*c) == Storage::Table).collect();
        let to = world.table_for(&table_components);
        self.spawn_into(to, e, bundle, ids);
    }

    /// `spawn`, remembering the table `ids` resolved to for the next spawn
    /// with the same list.
    pub fn spawn_shared<B: Bundle>(&mut self, e: Entity, bundle: B, ids: &Arc<[ComponentId]>) {
        let to = match &self.spawned_into {
            Some((last, t)) if Arc::ptr_eq(last, ids) => *t,
            _ => {
                let world = self.world;
                let table_components: Vec<ComponentId> =
                    ids.iter().copied().filter(|c| world.storage(*c) == Storage::Table).collect();
                let t = world.table_for(&table_components);
                self.spawned_into = Some((ids.clone(), t));
                t
            }
        };
        self.spawn_into(to, e, bundle, ids);
    }

    fn spawn_into<B: Bundle>(&mut self, to: TableId, e: Entity, bundle: B, ids: &[ComponentId]) {
        let world = self.world;
        self.lock_table(to);
        let table = world.table(to);
        let mut deferred = Vec::new();
        self.push_row(to, e, |row| {
            let mut sink = BundleSink { table, ids, next: 0, row, sparse: Vec::new() };
            bundle.put(&mut sink);
            deferred = sink.sparse;
        });
        for insert in deferred {
            insert(self, e);
        }
    }
}
