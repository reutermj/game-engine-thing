//! Ordered tables: a table holding an ordered key component (declared with
//! `order = key`) keeps its rows sorted by that key, then by entity, across
//! its pages. So a query walks them in key order, a range of keys is a
//! contiguous run of rows, and a merge with another sorted list (last
//! step's contacts with this step's) is one pass with no lookups. See
//! docs/architecture/relationships.md.
//!
//! Kept like spatial order, by `Structural`: anything that moves a row into
//! or out of an ordered table, or writes its key, marks the table, and it's
//! re-sorted when the `Structural` drops. A system never sees its own rows
//! move; the systems after it do.

use crate::component::{Component, Entity};
use crate::erased::ErasedColumn;
use crate::world::{Entities, Location, TableId};

/// A component whose tables are kept in order of `key`: declared with
/// `order = key` in `component!`. Keys order as unsigned integers; ties are
/// broken by entity, so the order never depends on history.
pub trait OrderKey: Component {
    fn key(&self) -> u128;
}

/// The key glue: reads a key as the type the declaring build compiled.
pub type KeyFn = unsafe fn(key: *const u8) -> u128;

/// What an ordered key declares, carried in its `ComponentDesc`.
#[derive(Clone, Copy)]
pub struct OrderDesc {
    pub key: KeyFn,
}

impl OrderDesc {
    pub const fn of<T: OrderKey>() -> OrderDesc {
        OrderDesc { key: __key::<T> }
    }
}

#[doc(hidden)]
pub unsafe fn __key<T: OrderKey>(key: *const u8) -> u128 {
    // SAFETY: the caller passes a value of the installed layout of `T`,
    // whose build this glue came from.
    unsafe { &*(key as *const T) }.key()
}

/// The key of an entity pair, ordering as `(a, b)` does: for relation
/// components such as contacts.
pub fn pair_key(a: Entity, b: Entity) -> u128 {
    (entity_key(a) << 64) | entity_key(b)
}

/// The key of one entity, ordering as `Entity` does: for a link to one
/// entity (a parent).
pub fn entity_key(e: Entity) -> u128 {
    ((e.index as u128) << 32) | e.generation as u128
}

/// Every pair key whose first entity is `a`.
pub fn pairs_from(a: Entity) -> std::ops::RangeInclusive<u128> {
    let lo = entity_key(a) << 64;
    lo..=lo | u64::MAX as u128
}

/// Each row's key as of the last re-sort, parallel to the table's pages.
#[derive(Default)]
pub struct KeyOrder {
    pub keys: Vec<Vec<u128>>,
    pub dirty: bool,
    /// Times the rows were rearranged: for tests.
    pub sorts: usize,
}

impl KeyOrder {
    pub(crate) fn add_page(&mut self) {
        self.keys.push(Vec::new());
    }

    /// A row pushed onto `page`: keyed at the next re-sort.
    pub(crate) fn push_row(&mut self, page: usize) {
        self.keys[page].push(0);
        self.dirty = true;
    }

    /// A row swap-removed from `page`, as the table's rows are: the row
    /// swapped in is likely out of place.
    pub(crate) fn swap_remove(&mut self, page: usize, row: usize) {
        self.keys[page].swap_remove(row);
        self.dirty = true;
    }

    /// The first position, in page order, whose key is at least `key`.
    pub fn seek(&self, key: u128) -> (usize, usize) {
        let p = self.keys.partition_point(|page| page.last().is_some_and(|&k| k < key));
        let r = self.keys.get(p).map_or(0, |page| page.partition_point(|&k| k < key));
        (p, r)
    }

    /// Checks the rows are in key order, then entity order: for tests.
    #[doc(hidden)]
    pub fn check(&self, rows: &[Vec<Entity>], keys_now: impl Fn(usize, usize) -> u128) -> Result<(), String> {
        if self.keys.iter().map(Vec::len).ne(rows.iter().map(Vec::len)) {
            return Err("the keys' pages don't match the table's".into());
        }
        let mut last: Option<(u128, Entity)> = None;
        for (p, page) in rows.iter().enumerate() {
            for (r, &e) in page.iter().enumerate() {
                let k = self.keys[p][r];
                if k != keys_now(p, r) {
                    return Err(format!("{e:?} is recorded at key {k}, and its key is {}", keys_now(p, r)));
                }
                if last.is_some_and(|l| l > (k, e)) {
                    return Err(format!("{e:?} at key {k} is after {last:?}"));
                }
                last = Some((k, e));
            }
        }
        Ok(())
    }
}

/// An ordered table, locked for re-sorting.
pub(crate) struct Resort<'a> {
    pub table: TableId,
    pub rows: &'a mut Vec<Vec<Entity>>,
    pub columns: Vec<&'a mut Vec<ErasedColumn>>,
    pub order: &'a mut KeyOrder,
    pub key: usize,
    pub desc: OrderDesc,
    pub page_rows: usize,
    pub entities: &'a Entities,
}

impl Resort<'_> {
    /// Keys every row, and if they're out of order, rearranges every
    /// column into pages in order. Keys are cheap to compute, so every row
    /// is keyed afresh rather than tracking which were written.
    pub fn run(self) {
        let mut sorted = true;
        let mut last: Option<(u128, Entity)> = None;
        for p in 0..self.rows.len() {
            for r in 0..self.rows[p].len() {
                // SAFETY: a value of the key's installed layout, whose build
                // the glue came from.
                let k = unsafe { (self.desc.key)(self.columns[self.key][p].value_ptr(r)) };
                self.order.keys[p][r] = k;
                let now = (k, self.rows[p][r]);
                sorted &= last.is_none_or(|l| l < now);
                last = Some(now);
            }
        }
        self.order.dirty = false;
        // Full pages but the last are what a rearrangement leaves; a sorted
        // table with holes (rows despawned from its end) keeps them.
        if sorted {
            return;
        }
        let mut all: Vec<(u128, Entity, u32, u32)> = Vec::with_capacity(self.rows.iter().map(Vec::len).sum());
        for (p, page) in self.rows.iter().enumerate() {
            all.extend(page.iter().enumerate().map(|(r, &e)| (self.order.keys[p][r], e, p as u32, r as u32)));
        }
        // Stable and adaptive: the rows are mostly in order already (new
        // ones appended, a few swapped by removals), so this is close to a
        // merge of a few runs.
        all.sort_by(|x, y| (x.0, x.1).cmp(&(y.0, y.1)));
        let at: Vec<(u32, u32)> = all.iter().map(|&(_, _, p, r)| (p, r)).collect();
        for c in self.columns {
            *c = ErasedColumn::gather(c, &at, self.page_rows);
        }
        let pages = at.len().div_ceil(self.page_rows).max(1);
        *self.rows = (0..pages).map(|_| Vec::with_capacity(self.page_rows)).collect();
        self.order.keys = (0..pages).map(|_| Vec::with_capacity(self.page_rows)).collect();
        for (i, &(k, e, _, _)) in all.iter().enumerate() {
            let (p, r) = (i / self.page_rows, i % self.page_rows);
            self.rows[p].push(e);
            self.order.keys[p].push(k);
            self.entities.place(e, Location { table: self.table, page: p as u32, row: r as u32 });
        }
        self.order.sorts += 1;
    }
}

crate::component! {
    /// A link to a parent: what a level's tiles belong to, or a body's
    /// colliders. Kept in parent order, so an entity's children are one
    /// range of rows (`children_of`).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ChildOf: "ecs::ChildOf", order = key {
        pub parent: Entity,
    }
}

impl OrderKey for ChildOf {
    fn key(&self) -> u128 {
        entity_key(self.parent)
    }
}

/// The keys of `parent`'s children, for `Query::in_keys`.
pub fn children_of(parent: Entity) -> std::ops::RangeInclusive<u128> {
    entity_key(parent)..=entity_key(parent)
}
