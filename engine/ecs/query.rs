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
use crate::erased::ErasedColumn;
use crate::events::{Event, EventQueue};
use crate::ordered::OrderKey;
use crate::par::{Workers, even};
use crate::spatial::{Bounds, Lanes, PageKind, RUN, SpatialPages};
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

/// A write term's column on one page, for `for_each_page`: reads as a
/// slice, and records writes as `Mut` does, per row (`set`, `get_mut`) or
/// for the whole page at once (`write_all`), which change detection then
/// sees as every row written. A plain `&mut [T]` would skip the ticks, and
/// the spatial re-sort and change detection read them.
pub struct ColumnMut<'a, T> {
    values: &'a mut [T],
    ticks: &'a mut [u32],
    now: u32,
}

impl<T> ColumnMut<'_, T> {
    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    pub fn as_slice(&self) -> &[T] {
        self.values
    }

    pub fn get_mut(&mut self, row: usize) -> Mut<'_, T> {
        Mut { value: &mut self.values[row], tick: &mut self.ticks[row], now: self.now }
    }

    pub fn set(&mut self, row: usize, value: T) {
        self.values[row] = value;
        self.ticks[row] = self.now;
    }

    /// Every row, stamped written now whether or not it changes.
    pub fn write_all(&mut self) -> &mut [T] {
        self.ticks.fill(self.now);
        self.values
    }
}

impl<T> std::ops::Index<usize> for ColumnMut<'_, T> {
    type Output = T;
    fn index(&self, row: usize) -> &T {
        &self.values[row]
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

    /// When page `p` of matched table `t` was last written, or its row
    /// `row` if given: table components only, as page walks are.
    fn written(&self, t: usize, p: usize, row: Option<usize>) -> u32 {
        match self {
            TermState::Table(guards, _) => {
                let column = &guards[t].pages()[p];
                row.map_or(column.written(), |r| column.ticks()[r])
            }
            TermState::Sparse(..) => panic!("a walk for what changed is over table components"),
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
    /// The term's column on one page, whole: for `for_each_page`, over
    /// table components only.
    type Slice<'a>;
    fn slice<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> Self::Slice<'a>;
    /// A table component's: every matched table has its column, so a walk
    /// can take the term's slices with no per-row check.
    const TABLE: bool = matches!(<Self::C as Component>::STORAGE, Storage::Table);
    /// The term's value on `row` of a page's slice.
    fn item_of<'b>(slice: &'b mut Self::Slice<'_>, row: usize) -> Self::Item<'b>;
    /// A run of consecutive pages of one matched table's column: what a
    /// task of a parallel walk (`par_for_each_page`) holds of this term.
    type Run<'a>;
    /// The column of every matched table cut into runs, `lens[t]` the
    /// pages in each of table `t`'s, in order: disjoint, so each can go to
    /// another task. One cut per run, not per page, so the split costs the
    /// calling thread nothing per page.
    fn runs<'a>(state: &'a mut TermState<'_>, lens: &[Vec<usize>]) -> Vec<Vec<Self::Run<'a>>>;
    /// Page `page` of a run, as `slice` would give it.
    fn run_page<'b>(run: &'b mut Self::Run<'_>, page: usize) -> Self::Slice<'b>;
}

fn not_paged<T: Component>() -> ! {
    panic!("{} is sparse, and a page walk is over table components", T::NAME)
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
    // Forced, here and in every per-row step of a walk: called through the
    // tuple impls from the crate that monomorphizes them, they stayed calls,
    // and were half a row's cost (docs/lore/a-query-row-cost-its-dispatch-
    // not-its-data.md).
    #[inline(always)]
    fn page<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> PageView<'a, T> {
        match state {
            TermState::Table(guards, _) => PageView::Read(guards[table].pages()[page].as_slice::<T>()),
            TermState::Sparse(set, _) => PageView::Sparse(set.set()),
        }
    }
    #[inline(always)]
    fn at<'b>(page: &'b mut PageView<'_, T>, row: usize, e: Entity) -> Option<&'b T> {
        match page {
            PageView::Read(rows) => Some(&rows[row]),
            PageView::Sparse(set) => set.get::<T>(e),
            _ => unreachable!("a read term's view"),
        }
    }
    type Slice<'a> = &'a [T];
    #[inline(always)]
    fn item_of<'b>(slice: &'b mut &[T], row: usize) -> &'b T {
        &slice[row]
    }
    fn slice<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> &'a [T] {
        match state {
            TermState::Table(guards, _) => guards[table].pages()[page].as_slice::<T>(),
            TermState::Sparse(..) => not_paged::<T>(),
        }
    }
    type Run<'a> = &'a [ErasedColumn];
    fn runs<'a>(state: &'a mut TermState<'_>, lens: &[Vec<usize>]) -> Vec<Vec<&'a [ErasedColumn]>> {
        match state {
            TermState::Table(guards, _) => guards
                .iter()
                .zip(lens)
                .map(|(g, lens)| {
                    let mut rest = g.pages();
                    lens.iter()
                        .map(|&n| {
                            let (run, tail) = rest.split_at(n);
                            rest = tail;
                            run
                        })
                        .collect()
                })
                .collect(),
            TermState::Sparse(..) => not_paged::<T>(),
        }
    }
    fn run_page<'b>(run: &'b mut &[ErasedColumn], page: usize) -> &'b [T] {
        run[page].as_slice::<T>()
    }
}

impl<T: Component> Term for &mut T {
    type C = T;
    const WRITE: bool = true;
    type Item<'a> = Mut<'a, T>;
    fn item<'a>(state: &'a mut TermState<'_>, table: usize, page: usize, row: usize, e: Entity) -> Option<Mut<'a, T>> {
        match state {
            TermState::Table(guards, now) => {
                let (values, ticks) = guards[table].pages_mut()[page].as_mut_slice_ticked::<T>(*now);
                Some(Mut { value: &mut values[row], tick: &mut ticks[row], now: *now })
            }
            TermState::Sparse(set, now) => {
                set.set_mut().get_mut_ticked::<T>(e, *now).map(|(value, tick)| Mut { value, tick, now: *now })
            }
        }
    }
    #[inline(always)]
    fn page<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> PageView<'a, T> {
        match state {
            TermState::Table(guards, now) => {
                let (values, ticks) = guards[table].pages_mut()[page].as_mut_slice_ticked::<T>(*now);
                let n = values.len();
                PageView::Write(values, &mut ticks[..n], *now)
            }
            TermState::Sparse(set, now) => PageView::SparseMut(set.set_mut(), *now),
        }
    }
    #[inline(always)]
    fn at<'b>(page: &'b mut PageView<'_, T>, row: usize, e: Entity) -> Option<Mut<'b, T>> {
        match page {
            PageView::Write(values, ticks, now) => Some(Mut { value: &mut values[row], tick: &mut ticks[row], now: *now }),
            PageView::SparseMut(set, now) => {
                let now = *now;
                set.get_mut_ticked::<T>(e, now).map(|(value, tick)| Mut { value, tick, now })
            }
            _ => unreachable!("a write term's view"),
        }
    }
    type Slice<'a> = ColumnMut<'a, T>;
    #[inline(always)]
    fn item_of<'b>(slice: &'b mut ColumnMut<'_, T>, row: usize) -> Mut<'b, T> {
        slice.get_mut(row)
    }
    fn slice<'a>(state: &'a mut TermState<'_>, table: usize, page: usize) -> ColumnMut<'a, T> {
        match state {
            TermState::Table(guards, now) => {
                let (values, ticks) = guards[table].pages_mut()[page].as_mut_slice_ticked::<T>(*now);
                ColumnMut { values, ticks, now: *now }
            }
            TermState::Sparse(..) => not_paged::<T>(),
        }
    }
    type Run<'a> = (&'a mut [ErasedColumn], u32);
    fn runs<'a>(state: &'a mut TermState<'_>, lens: &[Vec<usize>]) -> Vec<Vec<(&'a mut [ErasedColumn], u32)>> {
        match state {
            TermState::Table(guards, now) => {
                let now = *now;
                guards
                    .iter_mut()
                    .zip(lens)
                    .map(|(g, lens)| crate::par::carve(g.pages_mut(), lens.iter().copied()).into_iter().map(|run| (run, now)).collect())
                    .collect()
            }
            TermState::Sparse(..) => not_paged::<T>(),
        }
    }
    fn run_page<'b>(run: &'b mut (&mut [ErasedColumn], u32), page: usize) -> ColumnMut<'b, T> {
        let now = run.1;
        let (values, ticks) = run.0[page].as_mut_slice_ticked::<T>(now);
        ColumnMut { values, ticks, now }
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
    type Slices<'a>;
    /// The terms' columns on one page, whole.
    fn slices<'a>(states: &'a mut Self::States<'_>, table: usize, page: usize) -> Self::Slices<'a>;
    /// Every term is a table component's.
    const TABLE: bool;
    /// The items of `row`, from a page's slices.
    fn items_of<'b>(slices: &'b mut Self::Slices<'_>, row: usize) -> Self::Items<'b>;
    /// The latest tick any term was written at on a page, or on one of its
    /// rows.
    fn written(states: &Self::States<'_>, table: usize, page: usize, row: Option<usize>) -> u32;
    /// Every term's runs of pages, by matched table: see `Term::runs`.
    type Runs<'a>;
    fn runs<'a>(states: &'a mut Self::States<'_>, lens: &[Vec<usize>]) -> Vec<Vec<Self::Runs<'a>>>;
    fn run_page<'b>(runs: &'b mut Self::Runs<'_>, page: usize) -> Self::Slices<'b>;
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
    #[inline(always)]
    fn pages<'a>(_: &'a mut (), _: usize, _: usize) {}
    #[inline(always)]
    fn at<'b>(_: &'b mut (), _: usize, _: Entity) -> Option<()> {
        Some(())
    }
    fn driver(_: &()) -> Option<(usize, Vec<Entity>)> {
        None
    }
    type Slices<'a> = ();
    fn slices<'a>(_: &'a mut (), _: usize, _: usize) {}
    const TABLE: bool = true;
    #[inline(always)]
    fn items_of<'b>(_: &'b mut (), _: usize) {}
    fn written(_: &(), _: usize, _: usize, _: Option<usize>) -> u32 {
        0
    }
    type Runs<'a> = ();
    fn runs<'a>(_: &'a mut (), lens: &[Vec<usize>]) -> Vec<Vec<()>> {
        lens.iter().map(|l| vec![(); l.len()]).collect()
    }
    fn run_page<'b>(_: &'b mut (), _: usize) {}
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
    #[inline(always)]
    fn pages<'a>(s: &'a mut TermState<'_>, t: usize, p: usize) -> PageView<'a, A::C> {
        A::page(s, t, p)
    }
    #[inline(always)]
    fn at<'b>(page: &'b mut PageView<'_, A::C>, r: usize, e: Entity) -> Option<A::Item<'b>> {
        A::at(page, r, e)
    }
    fn driver(s: &TermState<'_>) -> Option<(usize, Vec<Entity>)> {
        s.sparse_entities()
    }
    type Slices<'a> = A::Slice<'a>;
    fn slices<'a>(s: &'a mut TermState<'_>, t: usize, p: usize) -> A::Slice<'a> {
        A::slice(s, t, p)
    }
    const TABLE: bool = A::TABLE;
    #[inline(always)]
    fn items_of<'b>(slice: &'b mut A::Slice<'_>, row: usize) -> A::Item<'b> {
        A::item_of(slice, row)
    }
    fn written(s: &TermState<'_>, t: usize, p: usize, row: Option<usize>) -> u32 {
        s.written(t, p, row)
    }
    type Runs<'a> = A::Run<'a>;
    fn runs<'a>(s: &'a mut TermState<'_>, lens: &[Vec<usize>]) -> Vec<Vec<A::Run<'a>>> {
        A::runs(s, lens)
    }
    fn run_page<'b>(run: &'b mut A::Run<'_>, page: usize) -> A::Slice<'b> {
        A::run_page(run, page)
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
            #[inline(always)]
            fn pages<'a>(states: &'a mut Self::States<'_>, t: usize, p: usize) -> Self::Pages<'a> {
                let ($($s,)+) = states;
                ($($t::page($s, t, p),)+)
            }
            #[inline(always)]
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
            type Slices<'a> = ($($t::Slice<'a>,)+);
            fn slices<'a>(states: &'a mut Self::States<'_>, t: usize, p: usize) -> Self::Slices<'a> {
                let ($($s,)+) = states;
                ($($t::slice($s, t, p),)+)
            }
            const TABLE: bool = $($t::TABLE &&)+ true;
            #[inline(always)]
            fn items_of<'b>(slices: &'b mut Self::Slices<'_>, row: usize) -> Self::Items<'b> {
                let ($($s,)+) = slices;
                ($($t::item_of($s, row),)+)
            }
            fn written(states: &Self::States<'_>, t: usize, p: usize, row: Option<usize>) -> u32 {
                let ($($s,)+) = states;
                0 $(.max($s.written(t, p, row)))+
            }
            type Runs<'a> = ($($t::Run<'a>,)+);
            fn runs<'a>(states: &'a mut Self::States<'_>, lens: &[Vec<usize>]) -> Vec<Vec<Self::Runs<'a>>> {
                let ($($s,)+) = states;
                let mut tables = ($($t::runs($s, lens).into_iter(),)+);
                lens.iter()
                    .map(|_| {
                        let mut runs = ($(tables.$i.next().expect("runs per matched table").into_iter(),)+);
                        std::iter::from_fn(|| Some(($(runs.$i.next()?,)+))).collect()
                    })
                    .collect()
            }
            fn run_page<'b>(runs: &'b mut Self::Runs<'_>, page: usize) -> Self::Slices<'b> {
                let ($($s,)+) = runs;
                ($($t::run_page($s, page),)+)
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

/// Rows of one page, in a page walk (`for_each_page`): its entities, and
/// a row for each, indexed as every term's column on the page is. A walk
/// in key order hands out runs of a page (`rows`).
pub struct Page<'a> {
    entities: &'a [Entity],
    rows: std::ops::Range<usize>,
    world: &'a World,
    changes: &'a ChangeDecl,
    log: &'a Log,
}

impl<'a> Page<'a> {
    /// The rows this call covers, as indices into the page's columns.
    pub fn rows(&self) -> std::ops::Range<usize> {
        self.rows.clone()
    }

    /// Every entity on the page, by row.
    pub fn entities(&self) -> &'a [Entity] {
        self.entities
    }

    pub fn entity(&self, row: usize) -> Entity {
        self.entities[row]
    }

    /// The row through which `row`'s entity is changed, as `for_each`'s.
    pub fn row(&self, row: usize) -> Row<'a> {
        Row { entity: self.entities[row], world: self.world, changes: self.changes, log: self.log }
    }
}

/// Every row of `ordered` tables, as (key, entity, table, page, row), in
/// key order and then entity order across them.
fn merged(rows: &[TableRead<'_>], ordered: &[usize]) -> Vec<(u128, Entity, u32, u32, u32)> {
    let mut all: Vec<(u128, Entity, u32, u32, u32)> = Vec::new();
    for &t in ordered {
        let (table, order) = (&rows[t], rows[t].ordered.as_ref().expect("ordered"));
        for (p, page) in table.rows.iter().enumerate() {
            all.extend(page.iter().enumerate().map(|(r, &e)| (order.keys[p][r], e, t as u32, p as u32, r as u32)));
        }
    }
    // One sorted run per table: a stable sort merges them.
    all.sort_by(|x, y| (x.0, x.1).cmp(&(y.0, y.1)));
    all
}

/// Sorts pair keys, the lesser entity index high and the greater low, by
/// counting each key's lesser index into a bucket, then sorting each
/// bucket's few keys by insertion: three passes, where a broadphase's pairs
/// would take `n log n` comparisons. Every lesser index is from `lo` to
/// before `hi`: a range of them, when ranges are sorted in parallel.
/// (History: 2026-09-24, an LSD radix sort over the bytes that differ
/// between keys, four passes at 10 000 bodies: on the dense bench's 39 000
/// pairs, 205 µs with the entities' lookup after it, against 149 for this.)
fn sort_pair_keys(keys: &mut Vec<u64>, lo: usize, hi: usize, scratch: &mut SortScratch) {
    let span = hi.saturating_sub(lo);
    // A bucket per index costs passes over every index, where the few
    // pairs of a broadphase that's mostly passive sort faster by comparison:
    // 800 pairs among 10 000 indices, 25 µs to 18 (2026-09-24). The same
    // order either way.
    if keys.len() * 4 < span {
        keys.sort_unstable();
        return;
    }
    let SortScratch { start, at, sorted } = scratch;
    start.clear();
    start.resize(span + 1, 0);
    for &k in keys.iter() {
        start[(k >> 32) as usize - lo + 1] += 1;
    }
    for i in 1..=span {
        start[i] += start[i - 1];
    }
    sorted.clear();
    sorted.resize(keys.len(), 0);
    at.clear();
    at.extend_from_slice(start);
    for &k in keys.iter() {
        let b = (k >> 32) as usize - lo;
        sorted[at[b] as usize] = k;
        at[b] += 1;
    }
    for w in start.windows(2) {
        let bucket = &mut sorted[w[0] as usize..w[1] as usize];
        for i in 1..bucket.len() {
            let mut j = i;
            while j > 0 && bucket[j - 1] > bucket[j] {
                bucket.swap(j - 1, j);
                j -= 1;
            }
        }
    }
    std::mem::swap(keys, sorted);
}

/// What `sort_pair_keys` counts and sorts into, made by the caller: a task
/// sorting a range gets it from the thread that made the task, since memory
/// a worker allocates comes from its own arena (glibc), and at a step's rate
/// that was fresh pages every time.
#[derive(Default)]
struct SortScratch {
    start: Vec<u32>,
    at: Vec<u32>,
    sorted: Vec<u64>,
}

impl SortScratch {
    fn with_capacity(span: usize, keys: usize) -> SortScratch {
        SortScratch { start: Vec::with_capacity(span + 1), at: Vec::with_capacity(span + 1), sorted: Vec::with_capacity(keys) }
    }
}

#[inline(always)]
fn passes(filters: &[(ComponentId, bool, SparseGuard<'_>)], e: Entity) -> bool {
    filters.is_empty() || filters.iter().all(|(_, with, set)| set.set().contains(e) == *with)
}

/// `Bounds::overlaps` without branches: a broadphase's box tests are
/// about even odds in a dense pile, so branches on them mispredict.
#[inline(always)]
fn meets(a: &Bounds, b: &Bounds) -> bool {
    (a.min[0] <= b.max[0]) & (b.min[0] <= a.max[0]) & (a.min[1] <= b.max[1]) & (b.min[1] <= a.max[1])
}

// ---- Parameters ----

/// One task of a parallel page walk: its pages, its own log, and what it
/// makes.
struct ParChunk<'a, R, A> {
    /// Each run's rows, page by page, and its columns.
    runs: Vec<(&'a [Vec<Entity>], R)>,
    log: Log,
    out: A,
}

/// The fewest rows worth a task of their own: below it, handing a chunk
/// to another thread costs more than walking it.
const PAR_ROWS: usize = 128;

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
    #[inline(always)]
    fn passes(filters: &[(ComponentId, bool, SparseGuard<'_>)], e: Entity) -> bool {
        passes(filters, e)
    }

    #[inline(always)]
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
        for (t, table) in rows.iter().enumerate() {
            Self::walk((world, decl, log, filters), t, table, states, &mut f);
        }
    }

    /// Every row of matched table `t` that passes the filters, in page order.
    fn walk<'s>(
        (world, decl, log, filters): (&World, &QueryDecl, &Log, &[(ComponentId, bool, SparseGuard<'_>)]),
        t: usize,
        table: &TableRead<'_>,
        states: &mut D::States<'s>,
        f: &mut impl FnMut(Row<'_>, D::Items<'_>),
    ) {
        // Table terms and no sparse filter: every row of every page
        // matches, so each is its terms' slices indexed, with no check per
        // row. Half the cost of the general walk below, or less
        // (query_bench, 2026-09-24).
        if D::TABLE && filters.is_empty() {
            for (p, page) in table.rows.iter().enumerate() {
                if page.is_empty() {
                    continue;
                }
                let mut slices = D::slices(states, t, p);
                for (r, &e) in page.iter().enumerate() {
                    f(Self::row(world, decl, log, e), D::items_of(&mut slices, r));
                }
            }
            return;
        }
        for (p, page) in table.rows.iter().enumerate() {
            // Each term's view once per page, then plain indexing.
            let mut pages = D::pages(states, t, p);
            for (r, &e) in page.iter().enumerate() {
                if Self::passes(filters, e)
                    && let Some(items) = D::at(&mut pages, r, e)
                {
                    f(Self::row(world, decl, log, e), items);
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
        // Tables walked as `for_each` walks them, never by a sparse set's
        // order, which `for_each` may take instead.
        let cx = (*world, *decl, *log, filters.as_slice());
        let ordered: Vec<usize> = (0..rows.len()).filter(|&t| rows[t].ordered.is_some()).collect();
        if ordered.len() <= 1 {
            // One sorted run already: no merge.
            for &t in &ordered {
                Self::walk(cx, t, &rows[t], states, &mut f);
            }
        } else {
            for (_, e, t, p, r) in merged(rows, &ordered) {
                if Self::passes(filters, e)
                    && let Some(items) = D::fetch(states, t as usize, p as usize, r as usize, e)
                {
                    f(Self::row(world, decl, log, e), items);
                }
            }
        }
        for t in (0..rows.len()).filter(|&t| rows[t].ordered.is_none()) {
            Self::walk(cx, t, &rows[t], states, &mut f);
        }
    }

    /// Every page of the query's rows, with each term's column on it whole:
    /// `&[T]` for a read, `ColumnMut` for a write, so a system that writes
    /// every row can stamp a page at once (`write_all`) rather than a row
    /// at a time. In the order `for_each` walks them. Table components
    /// only, and no sparse filters: a sparse term or filter has no slice.
    pub fn for_each_page(&mut self, mut f: impl FnMut(Page<'_>, D::Slices<'_>)) {
        self.paged();
        let Query { world, decl, rows, states, log, .. } = self;
        for (t, table) in rows.iter().enumerate() {
            for (p, page) in table.rows.iter().enumerate() {
                if !page.is_empty() {
                    let page_rows = Page { entities: page, rows: 0..page.len(), world, changes: &decl.changes, log };
                    f(page_rows, D::slices(states, t, p));
                }
            }
        }
    }

    /// `for_each_ordered`, a run at a time: each call is a run of rows
    /// consecutive on one page and in key order, with the page's columns
    /// whole (index them by `Page::rows`). One ordered table is a run per
    /// page; several are merged, and each row where the merge changes
    /// table ends a run. Tables that aren't ordered come after, a page at a
    /// time. As `for_each_page`, table components only.
    pub fn for_each_ordered_page(&mut self, mut f: impl FnMut(Page<'_>, D::Slices<'_>)) {
        self.paged();
        let Query { world, decl, rows, states, log, .. } = self;
        let changes = &decl.changes;
        let ordered: Vec<usize> = (0..rows.len()).filter(|&t| rows[t].ordered.is_some()).collect();
        let whole = |t: usize, p: usize| (t, p, 0..rows[t].rows[p].len());
        let mut runs: Vec<(usize, usize, std::ops::Range<usize>)> = Vec::new();
        if ordered.len() <= 1 {
            for &t in &ordered {
                runs.extend((0..rows[t].rows.len()).map(|p| whole(t, p)));
            }
        } else {
            for (_, _, t, p, r) in merged(rows, &ordered) {
                let (t, p, r) = (t as usize, p as usize, r as usize);
                // A table's rows are in key order, so a run breaks where the
                // merge changes table or page; the row check keeps a run to
                // rows the merge visited should that ever not hold.
                match runs.last_mut() {
                    Some((lt, lp, run)) if *lt == t && *lp == p && run.end == r => run.end += 1,
                    _ => runs.push((t, p, r..r + 1)),
                }
            }
        }
        for t in (0..rows.len()).filter(|&t| rows[t].ordered.is_none()) {
            runs.extend((0..rows[t].rows.len()).map(|p| whole(t, p)));
        }
        for (t, p, run) in runs {
            if run.is_empty() {
                continue;
            }
            let page = Page { entities: &rows[t].rows[p], rows: run, world, changes, log };
            f(page, D::slices(states, t, p));
        }
    }

    /// `for_each_page` split across `workers`: the pages, in walk order,
    /// cut into chunks of about equal rows, each chunk a task that walks
    /// its pages in order. `make` is called for each chunk, in order, with
    /// the rows it covers (indices into the walk, as `for_each` counts
    /// them) and makes what its task works on (outputs carved per chunk,
    /// an accumulator); those come back in chunk order. Rows' changes go
    /// into a log per chunk, joined in chunk order: the log one thread
    /// walking every page would have written, whatever the threads did.
    /// With one thread, one chunk on the caller's thread.
    pub fn par_for_each_page<A: Send>(
        &mut self,
        workers: &Workers,
        make: impl FnMut(std::ops::Range<usize>) -> A,
        f: impl Fn(&mut A, Page<'_>, D::Slices<'_>) + Sync,
    ) -> Vec<A>
    where
        for<'a> D::Runs<'a>: Send,
    {
        self.paged();
        let tables = (0..self.rows.len()).collect();
        self.par_pages(workers, tables, make, f)
    }

    /// `for_each_ordered_page` split across `workers`, as
    /// `par_for_each_page`: for queries matching at most one ordered table,
    /// whose runs are its pages whole. Rows of one key in two tables would
    /// make runs within pages, and a page can't be two tasks'.
    pub fn par_for_each_ordered_page<A: Send>(
        &mut self,
        workers: &Workers,
        make: impl FnMut(std::ops::Range<usize>) -> A,
        f: impl Fn(&mut A, Page<'_>, D::Slices<'_>) + Sync,
    ) -> Vec<A>
    where
        for<'a> D::Runs<'a>: Send,
    {
        self.paged();
        let ordered = |t: &usize| self.rows[*t].ordered.is_some();
        let tables: Vec<usize> = (0..self.rows.len()).filter(ordered).chain((0..self.rows.len()).filter(|t| !ordered(t))).collect();
        assert!(tables.iter().filter(|t| ordered(t)).count() <= 1, "a parallel walk in key order is over one ordered table");
        self.par_pages(workers, tables, make, f)
    }

    /// `for_each` split across `workers`, as `par_for_each_page` splits
    /// pages: rows in walk order, chunk by chunk, each row's items as
    /// `for_each` hands them. Table components only, and no sparse filters,
    /// as page walks. Where each row is its own work, this rather than a
    /// loop over a page's columns: a page loop indexing columns and writing
    /// through `get_mut` measured half again `for_each`'s cost a row
    /// (`query_bench`, 2026-09-24), and this is `for_each`'s.
    pub fn par_for_each<A: Send>(
        &mut self,
        workers: &Workers,
        make: impl FnMut(std::ops::Range<usize>) -> A,
        f: impl Fn(&mut A, Row<'_>, D::Items<'_>) + Sync,
    ) -> Vec<A>
    where
        for<'a> D::Runs<'a>: Send,
    {
        self.paged();
        let tables = (0..self.rows.len()).collect();
        self.par_pages(workers, tables, make, |out, page, mut slices| {
            let (world, changes, log) = (page.world, page.changes, page.log);
            for (r, &entity) in page.entities.iter().enumerate() {
                f(out, Row { entity, world, changes, log }, D::items_of(&mut slices, r));
            }
        })
    }

    /// The walk of `tables`' pages, in that order, cut into chunks of
    /// about equal rows, each chunk a run of pages per table it reaches.
    fn par_pages<A: Send>(
        &mut self,
        workers: &Workers,
        tables: Vec<usize>,
        mut make: impl FnMut(std::ops::Range<usize>) -> A,
        f: impl Fn(&mut A, Page<'_>, D::Slices<'_>) + Sync,
    ) -> Vec<A>
    where
        for<'a> D::Runs<'a>: Send,
    {
        let Query { world, decl, rows, states, log, .. } = self;
        let rows = &*rows;
        // The walk's pages with rows, which chunks are cut between.
        let walk: Vec<(usize, usize)> =
            tables.iter().flat_map(|&t| (0..rows[t].rows.len()).filter(move |&p| !rows[t].rows[p].is_empty()).map(move |p| (t, p))).collect();
        let weights: Vec<usize> = walk.iter().map(|&(t, p)| rows[t].rows[p].len()).collect();
        let chunks = crate::par::balanced(&weights, workers.chunks(weights.iter().sum(), PAR_ROWS));
        // Each table's pages cut where a chunk starts in it, so the runs
        // tile it: empty pages go with the chunk before them, and a table
        // no chunk reaches is one run no chunk takes.
        let mut cuts: Vec<Vec<usize>> = rows.iter().map(|_| vec![0]).collect();
        let mut reached = vec![false; rows.len()];
        let mut reach: Vec<Vec<usize>> = chunks.iter().map(|_| Vec::new()).collect();
        for (k, range) in chunks.iter().enumerate() {
            for &(t, p) in &walk[range.clone()] {
                if reach[k].last() != Some(&t) {
                    if std::mem::replace(&mut reached[t], true) {
                        cuts[t].push(p);
                    }
                    reach[k].push(t);
                }
            }
        }
        let lens: Vec<Vec<usize>> = cuts
            .iter()
            .zip(rows.iter())
            .map(|(c, table)| c.iter().zip(c.iter().skip(1).chain([&table.rows.len()])).map(|(a, b)| b - a).collect())
            .collect();
        let mut runs: Vec<std::collections::VecDeque<(usize, usize, D::Runs<'_>)>> = D::runs(states, &lens)
            .into_iter()
            .enumerate()
            .map(|(t, runs)| runs.into_iter().zip(&cuts[t]).zip(&lens[t]).map(|((r, &from), &n)| (from, n, r)).collect())
            .collect();
        let (world, changes) = (*world, &decl.changes);
        let mut row = 0;
        let tasks: Vec<ParChunk<'_, D::Runs<'_>, A>> = chunks
            .iter()
            .zip(&reach)
            .map(|(range, reach)| {
                let n: usize = weights[range.clone()].iter().sum();
                let runs = reach
                    .iter()
                    .map(|&t| {
                        let (from, n, run) = runs[t].pop_front().expect("a run per chunk reaching a table");
                        (&rows[t].rows[from..from + n], run)
                    })
                    .collect();
                let chunk = ParChunk { runs, log: Log::default(), out: make(row..row + n) };
                row += n;
                chunk
            })
            .collect();
        let done = workers.map_each(tasks, |_, mut chunk| {
            for (pages, mut run) in chunk.runs.drain(..) {
                for (i, entities) in pages.iter().enumerate().filter(|(_, e)| !e.is_empty()) {
                    let page = Page { entities, rows: 0..entities.len(), world, changes, log: &chunk.log };
                    f(&mut chunk.out, page, D::run_page(&mut run, i));
                }
            }
            (chunk.log.into_inner(), chunk.out)
        });
        let mut main = log.borrow_mut();
        done.into_iter()
            .map(|(changes, out)| {
                main.extend(changes);
                out
            })
            .collect()
    }

    /// The world's change tick while this query holds its guards: every
    /// write to what it matches so far is at or before it, and any after,
    /// later. What `for_each_written` takes to see what's changed since.
    pub fn now(&self) -> u32 {
        self.world.current_tick()
    }

    /// Every row the query matches any term of which was written after tick
    /// `since` (a `now` from before), in page order: change detection. A
    /// page none of whose terms was written since is skipped whole, by its
    /// own tick, so a walk over things at rest costs a look per page, not
    /// per row. A spawned row's values, and an inserted value, are written
    /// then; rows only moved (between tables or pages) keep their ticks.
    /// Rows that left aren't here to be seen: `left_since` says whether any
    /// did. Table components only, as page walks are.
    pub fn for_each_written(&mut self, since: u32, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let Query { world, decl, rows, states, filters, log, .. } = self;
        for (t, table) in rows.iter().enumerate() {
            for (p, page) in table.rows.iter().enumerate() {
                if page.is_empty() || D::written(states, t, p, None) <= since {
                    continue;
                }
                for (r, &e) in page.iter().enumerate() {
                    if D::written(states, t, p, Some(r)) > since
                        && Self::passes(filters, e)
                        && let Some(items) = D::fetch(states, t, p, r, e)
                    {
                        f(Self::row(world, decl, log, e), items);
                    }
                }
            }
        }
    }

    /// Whether a row left any table the query matches after tick `since`
    /// (a `now` from before): despawned, or moved to another table by an
    /// insert or a removal, whether or not the query would still match it.
    /// What's gone has no value left to be written, so this is change
    /// detection's other half, a look per table; it says only that some row
    /// left, not which, and a row that left and came back still left.
    pub fn left_since(&self, since: u32) -> bool {
        self.rows.iter().any(|t| t.table.left.load(std::sync::atomic::Ordering::Relaxed) > since)
    }

    /// Whether a row arrived in any table the query matches after tick
    /// `since`: spawned, or moved from another table. A look per table,
    /// before a walk for which (`for_each_written`, since a spawned value
    /// or an inserted one is written) when only new rows matter.
    pub fn arrived_since(&self, since: u32) -> bool {
        self.rows.iter().any(|t| t.table.arrived.load(std::sync::atomic::Ordering::Relaxed) > since)
    }

    /// The tick `e`'s row was last written at, the latest of the query's
    /// terms, if the query matches it: change detection for one entity,
    /// where `for_each_written` is for all of them. Table components only.
    pub fn written(&self, e: Entity) -> Option<u32> {
        if !self.world.entities.is_alive(e) || !Self::passes(&self.filters, e) {
            return None;
        }
        let at = self.world.entities.location(e)?;
        let t = self.table_ids.iter().position(|&id| id == at.table)?;
        Some(D::written(&self.states, t, at.page as usize, Some(at.row as usize)))
    }

    /// How many rows the query's tables hold: the length of a page walk.
    pub fn len(&self) -> usize {
        self.rows.iter().map(|t| t.rows.iter().map(Vec::len).sum::<usize>()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Page walks see every row of a table, so a sparse filter, which
    /// would skip some, has no place in them.
    fn paged(&self) {
        assert!(self.filters.is_empty(), "a page walk is over whole pages, and this query filters by a sparse component");
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
                let mut meeting = order.lanes[p].meeting(&region, 0.0);
                while meeting != 0 {
                    let r = meeting.trailing_zeros() as usize;
                    meeting &= meeting - 1;
                    let e = table.rows[p][r];
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

    /// Every pair of entities the query matches in its spatial tables whose
    /// boxes, grown by `grow`, meet: a broadphase. Each pair once, the
    /// lesser entity first, sorted. `near_pairs` with no passive side.
    pub fn near_pairs(&mut self, grow: f32) -> Vec<(Entity, Entity)> {
        near_pairs(&*self, &(), grow)
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

// ---- The broadphase ----

/// One side of a broadphase (`near_pairs`): a query, or a tuple of
/// queries, as the spatial tables it matches and the sparse filters its
/// rows must pass. What the queries read doesn't matter, only what they
/// match.
pub trait NearSide {
    #[doc(hidden)]
    fn spatial_tables<'a>(&'a self, out: &mut Vec<SideTable<'a>>);
}

/// A spatial table as a broadphase sees it.
#[doc(hidden)]
pub struct SideTable<'a> {
    id: TableId,
    rows: &'a [Vec<Entity>],
    order: &'a SpatialPages,
    filters: &'a [(ComponentId, bool, SparseGuard<'a>)],
}

impl SideTable<'_> {
    /// Which rows of page `p` pass the filters, as bits by row.
    fn pass(&self, p: usize) -> u32 {
        let page = &self.rows[p];
        if page.is_empty() {
            return 0;
        }
        if self.filters.is_empty() {
            return u32::MAX >> (32 - page.len());
        }
        page.iter().enumerate().fold(0, |m, (r, &e)| m | (passes(self.filters, e) as u32) << r)
    }
}

impl<D: Data, F, C> NearSide for Query<'_, D, F, C> {
    fn spatial_tables<'a>(&'a self, out: &mut Vec<SideTable<'a>>) {
        for (t, table) in self.rows.iter().enumerate() {
            if let Some(order) = &table.spatial {
                out.push(SideTable { id: self.table_ids[t], rows: &table.rows, order, filters: &self.filters });
            }
        }
    }
}

impl NearSide for () {
    fn spatial_tables<'a>(&'a self, _: &mut Vec<SideTable<'a>>) {}
}

impl<A: NearSide + ?Sized> NearSide for &A {
    fn spatial_tables<'a>(&'a self, out: &mut Vec<SideTable<'a>>) {
        (**self).spatial_tables(out)
    }
}

macro_rules! near_side_tuple {
    ($($s:ident),+) => {
        impl<$($s: NearSide),+> NearSide for ($($s,)+) {
            #[allow(non_snake_case)]
            fn spatial_tables<'a>(&'a self, out: &mut Vec<SideTable<'a>>) {
                let ($($s,)+) = self;
                $($s.spatial_tables(out);)+
            }
        }
    };
}
near_side_tuple!(A, B);
near_side_tuple!(A, B, C);
near_side_tuple!(A, B, C, D);

/// Every pair of rows whose boxes, grown by `grow`, meet, and at least one
/// of which is in `active`'s tables: a broadphase in which pairs of two
/// `passive` rows (things at rest, which meet as they did last time) cost
/// nothing. Each pair once, the lesser entity first, sorted. A row is
/// active if any of `active`'s queries matches it, whatever else does.
///
/// Active pages are swept along x against each other, and each meeting
/// pair of pages tests only the rows that reach the other page, each row
/// against all of the other page's at once (`Lanes::meeting`). Passive
/// pages are met with the active ones through their tables' runs, so a
/// passive page no active one is near is never looked at. See
/// docs/architecture/spatial-storage.md.
pub fn near_pairs(active: &impl NearSide, passive: &impl NearSide, grow: f32) -> Vec<(Entity, Entity)> {
    near_pairs_with(&Workers::default(), active, passive, grow)
}

/// An active page as the broadphase sweeps it: its rows, which of them
/// pass the filters, and its box grown.
type SweptPage<'a> = (&'a Lanes, u32, Bounds);

/// Where a broadphase's pair keys go: one list, or a list per range of
/// lesser index when the pairs are sorted in parallel, a range a task.
trait Keys {
    fn push(&mut self, key: u64);
    fn len(&self) -> usize;
}

impl Keys for Vec<u64> {
    #[inline(always)]
    fn push(&mut self, key: u64) {
        Vec::push(self, key)
    }
    fn len(&self) -> usize {
        Vec::len(self)
    }
}

/// Keys by range of their lesser index, ranges `1 << shift` indices wide.
struct Buckets {
    lists: Vec<Vec<u64>>,
    shift: u32,
    len: usize,
}

impl Keys for Buckets {
    #[inline(always)]
    fn push(&mut self, key: u64) {
        self.lists[(key >> (32 + self.shift)) as usize].push(key);
        self.len += 1;
    }
    fn len(&self) -> usize {
        self.len
    }
}

/// Active page `i` against itself and every page after it in the sweep
/// that its box meets.
#[inline(always)]
fn sweep(keys: &mut impl Keys, pages: &[SweptPage<'_>], i: usize, grow: f32) {
    let (a, pass_a, box_a) = pages[i];
    let mut rows = pass_a;
    while rows != 0 {
        let x = rows.trailing_zeros() as usize;
        rows &= rows - 1;
        // Only the rows after `x`: each pair once.
        let mut m = a.meeting(&a.grown(x, grow), grow) & rows;
        while m != 0 {
            keys.push(pair_of(a.index[x], a.index[m.trailing_zeros() as usize]));
            m &= m - 1;
        }
    }
    for &(b, pass_b, box_b) in &pages[i + 1..] {
        if box_b.min[0] > box_a.max[0] {
            break;
        }
        if !meets(&box_a, &box_b) {
            continue;
        }
        // Only rows that reach the other page can pair with its rows.
        let ma = a.meeting(&box_b, grow) & pass_a;
        if ma == 0 {
            continue;
        }
        let mb = b.meeting(&box_a, grow) & pass_b;
        if mb != 0 {
            cross(keys, a, ma, b, mb, grow);
        }
    }
}

/// A passive table's unit of the broadphase: a run of its ordered pages,
/// or a page out of the order (a big one).
#[derive(Clone, Copy)]
enum Unit {
    Run(usize),
    Big(usize),
}

impl Unit {
    /// Every unit of `table`, runs first.
    fn of(table: &SideTable<'_>) -> impl Iterator<Item = Unit> {
        let order = table.order;
        let big = (0..order.kind.len()).filter(move |&p| order.kind[p] != PageKind::Ordered && !table.rows[p].is_empty());
        (0..order.runs.len()).map(Unit::Run).chain(big.map(Unit::Big))
    }
}

/// A passive unit met with the active pages near it along x, found by
/// search in their sweep order: a passive table is mostly far from what's
/// active, and a unit no active page is near costs a search. `widest`, the
/// widest active page, bounds how far left of a unit one can start.
/// `noted` gets each passive row a pair was found with.
#[inline(always)]
fn meet_unit(
    keys: &mut impl Keys,
    noted: &mut impl FnMut(Entity),
    (pages, widest, grow): (&[SweptPage<'_>], f32, f32),
    table: &SideTable<'_>,
    unit: Unit,
) {
    let order = table.order;
    // Passive page `p` (its box grown, `box_b`) against an active page
    // whose box meets it.
    let mut visit = |keys: &mut _, (a, pass_a, box_a): SweptPage<'_>, p: usize, box_b: &Bounds| {
        // The passive page's rows that reach the active one first: a big
        // page (a level's walls) meets every page near it, and few of
        // its rows reach any one of them.
        let b = &order.lanes[p];
        let mb = b.meeting(&box_a, grow) & table.pass(p);
        if mb == 0 {
            return;
        }
        let ma = a.meeting(box_b, grow) & pass_a;
        // From the side with fewer rows reaching the other: a wall
        // reaching a page is one test, not one per row of the page.
        // Not in the active sweep, where choosing cost more than it
        // saved (2026-09-24, `spatial_bench`'s dense layout).
        let fewer = ma.count_ones() <= mb.count_ones();
        let found = ma != 0 && if fewer { cross(keys, a, ma, b, mb, grow) } else { cross(keys, b, mb, a, ma, grow) };
        if found {
            let mut m = mb;
            while m != 0 {
                noted(table.rows[p][m.trailing_zeros() as usize]);
                m &= m - 1;
            }
        }
    };
    let near = |unit: Bounds| {
        let from = pages.partition_point(|p| p.2.min[0] < unit.min[0] - widest);
        pages[from..].iter().take_while(move |p| p.2.min[0] <= unit.max[0]).filter(move |p| meets(&p.2, &unit))
    };
    match unit {
        Unit::Run(r) => {
            let run_pages = &order.order[r * RUN..((r + 1) * RUN).min(order.order.len())];
            for &active in near(order.runs[r].grown(grow)) {
                for &p in run_pages {
                    let box_b = order.bounds[p as usize].grown(grow);
                    if !table.rows[p as usize].is_empty() && meets(&active.2, &box_b) {
                        visit(keys, active, p as usize, &box_b);
                    }
                }
            }
        }
        Unit::Big(p) => {
            let box_b = order.bounds[p].grown(grow);
            for &active in near(box_b) {
                visit(keys, active, p, &box_b);
            }
        }
    }
}

/// `near_pairs` split across `workers`: the sweep in ranges of active
/// pages, the passive side in ranges of its units, each a task, with the
/// keys each finds already split by range of lesser index, then each
/// range sorted and turned into entities by a task of its own. Pairs are
/// unique and sorted, so the result is the one thread's, bit for bit.
pub fn near_pairs_with(workers: &Workers, active: &impl NearSide, passive: &impl NearSide, grow: f32) -> Vec<(Entity, Entity)> {
    let (mut act, mut pas) = (Vec::new(), Vec::new());
    active.spatial_tables(&mut act);
    passive.spatial_tables(&mut pas);
    // A table matched twice (by two queries of a side, or by both sides)
    // pairs its rows twice, and a row in both with itself: the pairs are
    // made unique after, only then, since it's a pass over all of them.
    let mut ids: Vec<TableId> = act.iter().chain(&pas).map(|t| t.id).collect();
    ids.sort_unstable();
    let twice = ids.windows(2).any(|w| w[0] == w[1]);
    // Each active page in use. A filtered-out row still widens its page's
    // box, which only costs a test.
    let mut pages: Vec<SweptPage<'_>> = Vec::new();
    // Each paired row's generation, by index: pairs are keyed by indices.
    let mut generation: Vec<u32> = Vec::new();
    let note = |generation: &mut Vec<u32>, e: Entity| {
        let i = e.index as usize;
        if generation.len() <= i {
            generation.resize(i + 1, 0);
        }
        generation[i] = e.generation;
    };
    for table in &act {
        for p in 0..table.rows.len() {
            let pass = table.pass(p);
            if pass == 0 {
                continue;
            }
            for &e in &table.rows[p] {
                note(&mut generation, e);
            }
            pages.push((&table.order.lanes[p], pass, table.order.bounds[p].grown(grow)));
        }
    }
    pages.sort_unstable_by(|a, b| a.2.min[0].total_cmp(&b.2.min[0]));
    let widest = pages.iter().fold(0.0f32, |w, p| w.max(p.2.max[0] - p.2.min[0]));
    let units: Vec<(usize, Unit)> = pas.iter().enumerate().flat_map(|(t, table)| Unit::of(table).map(move |u| (t, u))).collect();
    let entity = |generation: &[u32], i: u64| Entity { index: i as u32, generation: generation[i as usize] };

    if workers.threads() == 1 {
        let mut keys: Vec<u64> = Vec::new();
        for i in 0..pages.len() {
            sweep(&mut keys, &pages, i, grow);
        }
        let mut noted = Vec::new();
        for &(t, unit) in &units {
            meet_unit(&mut keys, &mut |e| noted.push(e), (&pages, widest, grow), &pas[t], unit);
        }
        for e in noted {
            note(&mut generation, e);
        }
        sort_pair_keys(&mut keys, 0, generation.len(), &mut SortScratch::default());
        if twice {
            keys.dedup();
            keys.retain(|&k| k >> 32 != k & 0xffff_ffff);
        }
        return keys.iter().map(|&k| (entity(&generation, k >> 32), entity(&generation, k & 0xffff_ffff))).collect();
    }

    // Passive rows are noted as found, and a passive index may be past
    // every active one: the ranges cover every index a row has.
    let max = act.iter().chain(&pas).flat_map(|t| t.rows.iter().flatten()).map(|e| e.index as usize + 1).max().unwrap_or(0);
    let shift = (max.div_ceil(workers.threads() * 2).max(1)).next_power_of_two().trailing_zeros();
    let ranges = (max >> shift) + 1;
    // Every list a task pushes to is made here, with room for about as
    // many pairs as the pile had rows (two and a half a row, settled): see
    // `SortScratch`. More only costs a task a reallocation.
    let buckets = |rows: usize| Buckets { lists: (0..ranges).map(|_| Vec::with_capacity(rows * 3 / ranges + 8)).collect(), shift, len: 0 };
    let rows_of = |r: &std::ops::Range<usize>| pages[r.clone()].iter().map(|p| p.1.count_ones() as usize).sum::<usize>();
    let sweeps: Vec<_> = even(pages.len(), workers.chunks(pages.len(), 16)).into_iter().map(|r| (buckets(rows_of(&r)), r)).collect();
    let swept = workers.map_each(sweeps, |_, (mut keys, r)| {
        for i in r {
            sweep(&mut keys, &pages, i, grow);
        }
        keys.lists
    });
    let meets: Vec<_> = even(units.len(), workers.chunks(units.len(), 4)).into_iter().map(|r| (buckets(r.len() * 4), Vec::with_capacity(64), r)).collect();
    let met = workers.map_each(meets, |_, (mut keys, mut noted, r)| {
        for &(t, unit) in &units[r] {
            meet_unit(&mut keys, &mut |e| noted.push(e), (&pages, widest, grow), &pas[t], unit);
        }
        (keys.lists, noted)
    });
    let mut parts: Vec<Vec<Vec<u64>>> = swept;
    for (keys, noted) in met {
        parts.push(keys);
        for e in noted {
            note(&mut generation, e);
        }
    }
    // Range `r`'s keys from every part, sorted: what a sort of them all
    // holds from `r << shift` to the next range.
    let mut parts: Vec<Vec<Option<Vec<u64>>>> = parts.into_iter().map(|p| p.into_iter().map(Some).collect()).collect();
    let by_range: Vec<_> = (0..ranges)
        .map(|r| {
            let lists: Vec<Vec<u64>> = parts.iter_mut().map(|p| p[r].take().expect("each range once")).collect();
            let n = lists.iter().map(Vec::len).sum();
            (lists, Vec::with_capacity(n), SortScratch::with_capacity(1 << shift, n))
        })
        .collect();
    let sorted = workers.map_each(by_range, |r, (lists, mut keys, mut scratch)| {
        lists.iter().for_each(|l| keys.extend_from_slice(l));
        let lo = r << shift;
        sort_pair_keys(&mut keys, lo, (lo + (1 << shift)).min(max), &mut scratch);
        if twice {
            keys.dedup();
            keys.retain(|&k| k >> 32 != k & 0xffff_ffff);
        }
        // Freed here, by the thread that made them: see `SortScratch`.
        (keys, lists, scratch)
    });
    let sorted: Vec<Vec<u64>> = sorted.into_iter().map(|(keys, ..)| keys).collect();
    let mut out = vec![(Entity { index: 0, generation: 0 }, Entity { index: 0, generation: 0 }); sorted.iter().map(Vec::len).sum()];
    let pieces: Vec<(&mut [(Entity, Entity)], Vec<u64>)> =
        crate::par::carve(&mut out, sorted.iter().map(Vec::len)).into_iter().zip(sorted).collect();
    let generation = &generation;
    let freed = workers.map_each(pieces, |_, (piece, keys)| {
        for (o, &k) in piece.iter_mut().zip(&keys) {
            *o = (entity(generation, k >> 32), entity(generation, k & 0xffff_ffff));
        }
        keys
    });
    drop(freed);
    out
}

/// A pair as a key, the lesser index high: live entities never share an
/// index, so indices alone order pairs.
#[inline(always)]
fn pair_of(a: u32, b: u32) -> u64 {
    ((a.min(b) as u64) << 32) | a.max(b) as u64
}

/// Pairs the rows of `a` in `ma` with the rows of `b` in `mb` whose grown
/// boxes meet, a row of `a` at a time against all of `b`'s at once.
/// Returns whether it found any.
#[inline(always)]
fn cross(keys: &mut impl Keys, a: &Lanes, mut ma: u32, b: &Lanes, mb: u32, grow: f32) -> bool {
    let before = keys.len();
    while ma != 0 {
        let x = ma.trailing_zeros() as usize;
        ma &= ma - 1;
        let mut m = b.meeting(&a.grown(x, grow), grow) & mb;
        while m != 0 {
            keys.push(pair_of(a.index[x], b.index[m.trailing_zeros() as usize]));
            m &= m - 1;
        }
    }
    keys.len() > before
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
