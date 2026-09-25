//! The unsafe core: a column of values whose type is known at runtime only,
//! as a layout, a drop function and a schema fingerprint. The storage above
//! it is safe Rust. See docs/architecture/storage.md, "Where the unsafe is,
//! and isn't".
//!
//! Invariants, all local to this file:
//! - `ptr` points to `cap` slots of `layout` (or dangles, aligned, when the
//!   layout is zero-sized or nothing is allocated), of which the first `len`
//!   hold initialized values of the column's type.
//! - Typed access checks the component's fingerprint and layout first, so a
//!   `&[T]` is only ever made over values of `T`'s schema. Types are matched
//!   by schema rather than `TypeId` because a component crosses builds: the
//!   same component compiled into two mods is the same data.
//! - Nothing here is shared between threads without `&mut` or `&`: callers
//!   hold the column through a lock guard, so no invariant depends on
//!   concurrency.

use std::alloc::Layout;
use std::ptr::NonNull;

use crate::component::{Component, ComponentDesc, DropFn};

/// What the column needs to know about its values' type. Private fields,
/// made only from a `ComponentDesc`, which only `ComponentDesc::of` makes,
/// so they always describe one real component. The drop function is the
/// registering build's code; the world keeps that build mapped.
#[derive(Clone, Copy, Debug)]
pub struct ValueType {
    fingerprint: u64,
    layout: Layout,
    drop: Option<DropFn>,
}

impl ValueType {
    pub fn of<T: Component>() -> ValueType {
        ValueType::from_desc(&ComponentDesc::of::<T>())
    }

    /// Components are `Send + Sync` (the trait requires it), so a column of
    /// them may be shared between threads.
    pub fn from_desc(desc: &ComponentDesc) -> ValueType {
        let layout = Layout::from_size_align(desc.size, desc.align).expect("a component's layout");
        ValueType { fingerprint: desc.fingerprint, layout, drop: desc.drop }
    }

    pub fn fingerprint(&self) -> u64 {
        self.fingerprint
    }

    pub fn same_values(&self, other: &ValueType) -> bool {
        self.fingerprint == other.fingerprint && self.layout == other.layout
    }
}

/// Drops a value of `ty` in place.
///
/// # Safety
/// `value` must be a live value of `ty`, whose drop code is still mapped.
pub unsafe fn drop_value(ty: ValueType, value: *mut u8) {
    if let Some(drop) = ty.drop {
        unsafe { drop(value) };
    }
}

pub struct ErasedColumn {
    ty: ValueType,
    ptr: NonNull<u8>,
    len: usize,
    cap: usize,
    /// Per value, the world tick it was last written at: what change
    /// detection reads. Kept parallel to the values by every row operation.
    ticks: Vec<u32>,
    /// At least the latest of `ticks`, and of any value's that has left: so
    /// a walk for what changed skips a page none of whose values did, where
    /// the ticks alone cost a look at every row. Stamped when writes are
    /// handed out, whether or not any is made, and never lowered: either
    /// only costs a look at a page for nothing, where stamping it with each
    /// write cost 1.2 ns a row in a loop writing every row (2026-09-24).
    written: u32,
}

// SAFETY: the column owns its values like a `Vec<T>` does, and a
// `ValueType` can only be made for a component, which is `Send + Sync`, so
// the values may cross threads and be shared. Shared access only ever hands
// out `&T`.
unsafe impl Send for ErasedColumn {}
unsafe impl Sync for ErasedColumn {}

impl ErasedColumn {
    pub fn new(ty: ValueType) -> ErasedColumn {
        let cap = if ty.layout.size() == 0 { usize::MAX } else { 0 };
        ErasedColumn { ty, ptr: dangling(ty.layout), len: 0, cap, ticks: Vec::new(), written: 0 }
    }

    /// Each value's last-written tick.
    pub fn ticks(&self) -> &[u32] {
        &self.ticks
    }

    /// The latest tick any value here was written at, or later.
    pub fn written(&self) -> u32 {
        self.written
    }

    pub fn set_tick(&mut self, row: usize, tick: u32) {
        self.ticks[row] = tick;
        self.written = self.written.max(tick);
    }

    /// The values and their ticks, for handing out writes at tick `now`
    /// that record themselves (`Mut`): the column counts as written then.
    pub fn as_mut_slice_ticked<T: Component>(&mut self, now: u32) -> (&mut [T], &mut [u32]) {
        self.check::<T>();
        self.written = self.written.max(now);
        // SAFETY: as `as_mut_slice`; the ticks are a separate allocation.
        let values = unsafe { std::slice::from_raw_parts_mut(self.ptr.as_ptr() as *mut T, self.len) };
        (values, &mut self.ticks)
    }

    pub fn value_type(&self) -> ValueType {
        self.ty
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The value at `row`, as bytes of this column's type: for glue compiled
    /// with that type (a component's bounds), which reads it through the
    /// pointer. Valid until the column is next changed.
    pub fn value_ptr(&self, row: usize) -> *const u8 {
        assert!(row < self.len, "row {row} of {}", self.len);
        // SAFETY: `row` is initialized, so within the allocation.
        unsafe { self.slot(row) }
    }

    pub fn push<T: Component>(&mut self, value: T) {
        self.check::<T>();
        self.reserve_one();
        // SAFETY: slot `len` is allocated (reserve_one) and uninitialized;
        // the type was checked.
        unsafe { (self.slot(self.len) as *mut T).write(value) };
        self.len += 1;
        self.ticks.push(0);
    }

    pub fn as_slice<T: Component>(&self) -> &[T] {
        self.check::<T>();
        // SAFETY: the first `len` slots hold initialized `T`s (type checked),
        // and `ptr` is aligned for `T`, dangling only when `len` is 0 or `T`
        // is zero-sized, both of which `from_raw_parts` allows.
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr() as *const T, self.len) }
    }

    pub fn as_mut_slice<T: Component>(&mut self) -> &mut [T] {
        self.check::<T>();
        // SAFETY: as `as_slice`, and `&mut self` makes it unique.
        unsafe { std::slice::from_raw_parts_mut(self.ptr.as_ptr() as *mut T, self.len) }
    }

    /// Replaces the value at `row`, dropping the old one.
    pub fn replace<T: Component>(&mut self, row: usize, value: T) {
        self.as_mut_slice::<T>()[row] = value;
    }

    /// Drops the value at `row` and moves the last value into its place.
    pub fn swap_remove_drop(&mut self, row: usize) {
        assert!(row < self.len, "row {row} out of {}", self.len);
        // SAFETY: `row` holds an initialized value; after dropping it the
        // slot is refilled from `last` (or `row` is `last`), and `len` shrinks
        // so the moved-out last slot is no longer counted as initialized.
        unsafe {
            if let Some(drop) = self.ty.drop {
                drop(self.slot(row));
            }
            self.fill_from_last(row);
        }
        self.ticks.swap_remove(row);
    }

    /// Moves the value at `row` onto the end of `to` (a column of the same
    /// type), and the last value into its place.
    pub fn swap_remove_into(&mut self, row: usize, to: &mut ErasedColumn) {
        assert!(row < self.len, "row {row} out of {}", self.len);
        // By schema: two copies of one drop function (two builds') can have
        // different addresses.
        assert!(self.ty.same_values(&to.ty), "moving a value between columns of different types");
        to.reserve_one();
        // SAFETY: `row` is initialized; its bytes are copied to `to`'s next
        // (allocated, uninitialized) slot, which takes ownership, and the
        // hole left here is refilled as in `swap_remove_drop`.
        unsafe {
            copy_value(self.slot(row), to.slot(to.len), self.ty.layout.size());
            to.len += 1;
            self.fill_from_last(row);
        }
        let tick = self.ticks.swap_remove(row);
        to.ticks.push(tick);
        to.written = to.written.max(tick);
    }

    /// Drops the first `n` values and moves the rest down, keeping their
    /// order: for queues, oldest first.
    pub fn drop_front(&mut self, n: usize) {
        assert!(n <= self.len, "dropping {n} of {}", self.len);
        // SAFETY: the first `n` slots are initialized and dropped once; the
        // rest are moved (overlapping, so `copy`) to the front, and `len`
        // shrinks so the vacated tail isn't counted.
        unsafe {
            if let Some(drop) = self.ty.drop {
                for row in 0..n {
                    drop(self.slot(row));
                }
            }
            std::ptr::copy(self.slot(n), self.slot(0), (self.len - n) * self.ty.layout.size());
        }
        self.len -= n;
        self.ticks.drain(..n);
    }

    fn check<T: Component>(&self) {
        assert!(
            T::FINGERPRINT == self.ty.fingerprint && Layout::new::<T>() == self.ty.layout,
            "column of {} accessed with another layout of it",
            T::NAME
        );
    }

    /// Hands the values to another build's code for the same schema: a newer
    /// build took the component over, and the older one may be unmapped.
    pub fn adopt(&mut self, ty: ValueType) {
        assert!(self.ty.same_values(&ty), "adopting code for another schema");
        self.ty = ty;
    }

    /// Moves every value of `pages` into new pages of `page_rows` values,
    /// in the order `order` names them, as (page, row): how an ordered
    /// table is re-sorted. Each value keeps its tick, and `pages` are left
    /// empty. Panics unless `order` names every value exactly once, which
    /// is what makes the moves sound, and panics before moving any: a
    /// value moved before a panic would be owned by the old page and the
    /// new, and dropped by both as it unwound.
    pub fn gather(pages: &mut [ErasedColumn], order: &[(u32, u32)], page_rows: usize) -> Vec<ErasedColumn> {
        let ty = pages.first().expect("a table has a page").ty;
        assert!(pages.iter().all(|p| p.ty.same_values(&ty)), "gathering one column's pages");
        assert!(page_rows > 0, "pages hold rows");
        assert_eq!(order.len(), pages.iter().map(|p| p.len).sum::<usize>(), "gathering every value");
        let mut seen: Vec<Vec<bool>> = pages.iter().map(|p| vec![false; p.len]).collect();
        for &(p, r) in order {
            let slot = seen.get_mut(p as usize).and_then(|page| page.get_mut(r as usize));
            assert!(slot.is_some_and(|s| !std::mem::replace(s, true)), "row {r} of page {p} gathered once");
        }
        // Allocated up front too, so nothing between the first move and
        // the last can panic.
        let mut out: Vec<ErasedColumn> = order
            .chunks(page_rows)
            .map(|chunk| {
                let mut column = ErasedColumn::new(ty);
                column.reserve(chunk.len());
                column.ticks.reserve_exact(chunk.len());
                column
            })
            .collect();
        let size = ty.layout.size();
        for (column, chunk) in out.iter_mut().zip(order.chunks(page_rows)) {
            for &(p, r) in chunk {
                let (page, r) = (&pages[p as usize], r as usize);
                // SAFETY: `(p, r)` is an initialized value, moved once
                // (checked above) into `column`'s next allocated slot.
                unsafe { std::ptr::copy_nonoverlapping(page.slot(r), column.slot(column.len), size) };
                column.len += 1;
                column.ticks.push(page.ticks[r]);
                column.written = column.written.max(page.ticks[r]);
            }
        }
        // Every value was moved out: the old pages own nothing.
        for p in pages.iter_mut() {
            p.len = 0;
            p.ticks.clear();
        }
        if out.is_empty() {
            out.push(ErasedColumn::new(ty));
        }
        out
    }

    /// Rewrites every value into `to`'s layout with `migrate`, which must
    /// move or drop every part of the old value and initialize the new one
    /// fully (see `schema::migrate`). If `migrate` panics, the column is
    /// left empty with each value dropped once: those already rewritten,
    /// those not yet reached, and the one it was given, which is its own.
    ///
    /// # Safety
    /// `migrate(old, new)` must leave `old` fully moved out or dropped and
    /// `new` a valid value of `to`'s type, or panic with `old` moved out or
    /// dropped (or leaked) and `new` untouched.
    pub unsafe fn migrate(&mut self, to: ValueType, mut migrate: impl FnMut(*mut u8, *mut u8)) {
        /// Drops the values `migrate` hasn't been given when it panics.
        /// Without it the column would still count every row, rewritten
        /// ones included, and drop them again.
        struct Unmigrated<'a> {
            column: &'a mut ErasedColumn,
            next: usize,
        }
        impl Drop for Unmigrated<'_> {
            fn drop(&mut self) {
                let (next, len) = (self.next, self.column.len);
                self.column.len = 0;
                // SAFETY: rows from `next` hold values no one else owns.
                unsafe {
                    for row in next..len {
                        drop_value(self.column.ty, self.column.slot(row));
                    }
                }
            }
        }
        let len = self.len;
        let mut out = ErasedColumn::new(to);
        out.reserve(len);
        // The rows are the same rows, so they keep their ticks.
        let (ticks, written) = (std::mem::take(&mut self.ticks), self.written);
        let mut rest = Unmigrated { column: self, next: 0 };
        while rest.next < len {
            let row = rest.next;
            // Before the call: once given to `migrate`, the value is its.
            rest.next += 1;
            // SAFETY: `row` is initialized and `out`'s next slot allocated;
            // the caller's `migrate` consumes one and fills the other.
            unsafe { migrate(rest.column.slot(row), out.slot(row)) };
            out.len += 1;
        }
        // Every value was handed over, so this only empties the column,
        // and swapping frees its buffer.
        drop(rest);
        out.ticks = ticks;
        out.written = written;
        std::mem::swap(self, &mut out);
    }

    /// # Safety
    /// `row` must be within the allocation.
    unsafe fn slot(&self, row: usize) -> *mut u8 {
        unsafe { self.ptr.as_ptr().add(row * self.ty.layout.size()) }
    }

    /// Moves the last value into `row` (whose value has been dropped or moved
    /// out) and shrinks the column by one.
    ///
    /// # Safety
    /// `row < len`, and the value at `row` must already be gone.
    unsafe fn fill_from_last(&mut self, row: usize) {
        let last = self.len - 1;
        if row != last {
            unsafe { copy_value(self.slot(last), self.slot(row), self.ty.layout.size()) };
        }
        self.len = last;
    }

    fn reserve_one(&mut self) {
        self.reserve(1);
    }

    /// Room for `n` more values.
    fn reserve(&mut self, n: usize) {
        if self.cap - self.len >= n {
            return;
        }
        let cap = (self.cap * 2).max(4).max(self.len + n);
        let new = array(self.ty.layout, cap);
        // SAFETY: `new` has a non-zero size (zero-sized types never get here:
        // their cap is usize::MAX). Growing an existing allocation uses the
        // layout it was made with.
        let ptr = unsafe {
            if self.cap == 0 {
                std::alloc::alloc(new)
            } else {
                std::alloc::realloc(self.ptr.as_ptr(), array(self.ty.layout, self.cap), new.size())
            }
        };
        self.ptr = NonNull::new(ptr).unwrap_or_else(|| std::alloc::handle_alloc_error(new));
        self.cap = cap;
    }
}

impl Drop for ErasedColumn {
    fn drop(&mut self) {
        // SAFETY: the first `len` slots are initialized and dropped once; the
        // allocation, if any, was made with this layout.
        unsafe {
            if let Some(drop) = self.ty.drop {
                for row in 0..self.len {
                    drop(self.slot(row));
                }
            }
            if self.ty.layout.size() != 0 && self.cap != 0 {
                std::alloc::dealloc(self.ptr.as_ptr(), array(self.ty.layout, self.cap));
            }
        }
    }
}

/// Copies one value of `size` bytes: small sizes as fixed-size copies, since
/// a copy of a size known only at runtime is a call to `memcpy`, and moving
/// a row of a spatial table makes two per column (2026-09-24).
///
/// # Safety
/// As `copy_nonoverlapping`: `size` bytes readable at `src`, writable at
/// `dst`, not overlapping.
#[inline(always)]
unsafe fn copy_value(src: *const u8, dst: *mut u8, size: usize) {
    /// As `MaybeUninit`, not `[u8; N]`: a copy typed as integers drops the
    /// provenance of any pointer in the value (a `String`'s buffer, which
    /// is then dangling) and reads padding as initialized, both undefined
    /// behavior that Miri caught (docs/lore/copying-a-value-as-bytes-drops-its-pointers.md).
    /// The same instructions either way.
    #[inline(always)]
    unsafe fn fixed<const N: usize>(src: *const u8, dst: *mut u8) {
        type Bytes<const N: usize> = std::mem::MaybeUninit<[u8; N]>;
        unsafe { (dst as *mut Bytes<N>).write_unaligned((src as *const Bytes<N>).read_unaligned()) }
    }
    unsafe {
        match size {
            0 => {}
            4 => fixed::<4>(src, dst),
            8 => fixed::<8>(src, dst),
            12 => fixed::<12>(src, dst),
            16 => fixed::<16>(src, dst),
            20 => fixed::<20>(src, dst),
            24 => fixed::<24>(src, dst),
            32 => fixed::<32>(src, dst),
            _ => std::ptr::copy_nonoverlapping(src, dst, size),
        }
    }
}

fn dangling(layout: Layout) -> NonNull<u8> {
    NonNull::new(std::ptr::without_provenance_mut(layout.align())).expect("alignment is non-zero")
}

fn array(layout: Layout, n: usize) -> Layout {
    Layout::from_size_align(layout.size().checked_mul(n).expect("column too large"), layout.align()).expect("column layout")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::component;

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Word: "test::Word" {
            text: String,
        }
    }

    component! {
        #[derive(Debug, Default, PartialEq)]
        struct Tag: "test::Tag" {}
    }

    component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        struct Small: "test::Small" {
            n: u32,
        }
    }

    component! {
        #[derive(Debug, Default, PartialEq, Copy)]
        struct Large: "test::Small" {
            n: u64,
        }
    }

    fn word(s: &str) -> Word {
        Word { text: s.into() }
    }

    /// Values of `N` bytes, each byte its own, for `copy_value`'s sizes.
    macro_rules! sized {
        ($($name:ident: $id:literal, $ty:ty;)+) => {$(
            component! {
                #[derive(Debug, Default, PartialEq, Copy)]
                struct $name: $id { b: $ty }
            }
        )+};
    }
    sized! {
        B1: "test::B1", [u8; 1];
        B3: "test::B3", [u8; 3];
        B4: "test::B4", [u8; 4];
        B8: "test::B8", [u8; 8];
        B12: "test::B12", [u8; 12];
        B16: "test::B16", [u8; 16];
        B20: "test::B20", [u8; 20];
        B24: "test::B24", [u8; 24];
        B28: "test::B28", [u8; 28];
        B32: "test::B32", [u8; 32];
        B40: "test::B40", [u32; 10];
    }

    /// Moves between columns and fills holes with every byte intact, at
    /// the sizes copied as fixed sizes and the ones between and past them.
    #[test]
    fn values_of_every_size_move_whole() {
        fn moves<T: Component + Copy + PartialEq + std::fmt::Debug>(make: impl Fn(u8) -> T) {
            let (mut from, mut to) = (ErasedColumn::new(ValueType::of::<T>()), ErasedColumn::new(ValueType::of::<T>()));
            for i in 0..6 {
                from.push(make(i));
            }
            // Row 1 out, the last into its place; then the last out.
            from.swap_remove_into(1, &mut to);
            from.swap_remove_into(4, &mut to);
            from.swap_remove_drop(0);
            assert_eq!(from.as_slice::<T>(), [make(3), make(5), make(2)], "{}", T::NAME);
            assert_eq!(to.as_slice::<T>(), [make(1), make(4)], "{}", T::NAME);
        }
        fn bytes<const N: usize>(i: u8) -> [u8; N] {
            std::array::from_fn(|k| i.wrapping_mul(31).wrapping_add(k as u8 * 7 + 1))
        }
        moves(|i| B1 { b: bytes(i) });
        moves(|i| B3 { b: bytes(i) });
        moves(|i| B4 { b: bytes(i) });
        moves(|i| B8 { b: bytes(i) });
        moves(|i| B12 { b: bytes(i) });
        moves(|i| B16 { b: bytes(i) });
        moves(|i| B20 { b: bytes(i) });
        moves(|i| B24 { b: bytes(i) });
        moves(|i| B28 { b: bytes(i) });
        moves(|i| B32 { b: bytes(i) });
        moves(|i| B40 { b: std::array::from_fn(|k| u32::from_ne_bytes(bytes(i + k as u8))) });
    }

    #[test]
    fn values_are_pushed_read_replaced_and_removed() {
        let mut c = ErasedColumn::new(ValueType::of::<Word>());
        for w in ["a", "b", "c", "d", "e"] {
            c.push(word(w));
        }
        c.replace(1, word("B"));
        c.swap_remove_drop(0);
        let words: Vec<&str> = c.as_slice::<Word>().iter().map(|w| w.text.as_str()).collect();
        assert_eq!(words, ["e", "B", "c", "d"]);
    }

    /// Counts its drops, per thread: tests run on threads of their own, and
    /// each counts only the drops it makes.
    #[derive(Debug, Default, PartialEq)]
    struct Counted(u32);
    thread_local! {
        static DROPS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    }
    fn drops() -> usize {
        DROPS.with(|d| d.get())
    }
    impl Drop for Counted {
        fn drop(&mut self) {
            DROPS.with(|d| d.set(d.get() + 1));
        }
    }
    // SAFETY: plain data; no schema.
    unsafe impl Component for Counted {
        const NAME: &'static str = "test::Counted";
    }

    #[test]
    fn a_moved_value_is_owned_by_its_new_column_and_dropped_once() {
        let (mut from, mut to) = (ErasedColumn::new(ValueType::of::<Counted>()), ErasedColumn::new(ValueType::of::<Counted>()));
        let before = drops();
        for i in 0..3 {
            from.push(Counted(i));
        }
        from.swap_remove_into(0, &mut to);
        assert_eq!((from.len(), to.len()), (2, 1));
        assert_eq!(drops(), before, "a move drops nothing");
        drop(from);
        assert_eq!(drops(), before + 2);
        drop(to);
        assert_eq!(drops(), before + 3);
    }

    /// An order that names a value twice, or one that isn't there, is
    /// refused before anything moves: refused partway, the values already
    /// moved would be owned by the new pages and the old, and dropped by
    /// both as the panic unwound.
    #[test]
    fn a_gather_refused_partway_leaves_every_value_where_it_was() {
        let ty = ValueType::of::<Word>();
        let mut pages = [ErasedColumn::new(ty), ErasedColumn::new(ty)];
        pages[0].push(word("a"));
        pages[0].push(word("b"));
        pages[1].push(word("c"));
        for bad in [[(0, 0), (1, 0), (0, 0)], [(0, 1), (1, 0), (2, 0)], [(0, 0), (1, 0), (1, 1)]] {
            let refused = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| ErasedColumn::gather(&mut pages, &bad, 2)));
            assert!(refused.is_err(), "{bad:?} names a value twice or one that isn't there");
            assert_eq!(pages[0].as_slice::<Word>(), [word("a"), word("b")]);
            assert_eq!(pages[1].as_slice::<Word>(), [word("c")]);
        }
        let out = ErasedColumn::gather(&mut pages, &[(1, 0), (0, 1), (0, 0)], 2);
        let words: Vec<Vec<Word>> = out.iter().map(|c| c.as_slice::<Word>().to_vec()).collect();
        assert_eq!(words, [vec![word("c"), word("b")], vec![word("a")]]);
        assert!(pages.iter().all(ErasedColumn::is_empty));
    }

    /// A migration that panics partway drops each value once: those it
    /// wrote in the new layout, those it never reached in the old, and the
    /// one it was given, which was its own. The column is left empty and
    /// usable.
    #[test]
    fn a_migration_that_panics_drops_every_value_once() {
        let mut c = ErasedColumn::new(ValueType::of::<Counted>());
        for i in 0..5 {
            c.push(Counted(i));
        }
        let before = drops();
        let mut row = 0;
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            // SAFETY: moves each value whole into the same layout, and drops
            // the one it panics on.
            unsafe {
                c.migrate(ValueType::of::<Counted>(), |old, new| {
                    if row == 3 {
                        std::ptr::drop_in_place(old as *mut Counted);
                        panic!("a migration that fails partway");
                    }
                    row += 1;
                    (new as *mut Counted).write((old as *mut Counted).read());
                })
            }
        }));
        assert!(panicked.is_err());
        assert_eq!(drops() - before, 5, "each value dropped once");
        assert!(c.is_empty() && c.ticks().is_empty());
        c.push(Counted(9));
        assert_eq!(c.as_slice::<Counted>(), [Counted(9)]);
    }

    component! {
        /// Three bytes of padding, which are never initialized.
        #[derive(Debug, Default, PartialEq, Copy)]
        struct Padded: "test::Padded" {
            a: u8,
            b: u32,
        }
    }

    /// A move copies padding (uninitialized) and pointers (whose provenance
    /// must survive the copy) as they are. Only Miri sees the difference:
    /// natively a copy as integers moves the same bytes.
    #[test]
    fn values_with_padding_and_pointers_move_whole() {
        let (mut from, mut to) = (ErasedColumn::new(ValueType::of::<Padded>()), ErasedColumn::new(ValueType::of::<Padded>()));
        for i in 0..3 {
            from.push(Padded { a: i, b: i as u32 * 7 });
        }
        from.swap_remove_into(0, &mut to);
        assert_eq!(from.as_slice::<Padded>(), [Padded { a: 2, b: 14 }, Padded { a: 1, b: 7 }]);
        assert_eq!(to.as_slice::<Padded>(), [Padded { a: 0, b: 0 }]);
        let (mut from, mut to) = (ErasedColumn::new(ValueType::of::<Word>()), ErasedColumn::new(ValueType::of::<Word>()));
        for w in ["a", "b", "c"] {
            from.push(word(w));
        }
        from.swap_remove_into(0, &mut to);
        assert_eq!(from.as_slice::<Word>(), [word("c"), word("b")]);
        assert_eq!(to.as_slice::<Word>(), [word("a")]);
    }

    #[test]
    fn dropping_from_the_front_keeps_the_rest_in_order() {
        let mut c = ErasedColumn::new(ValueType::of::<Word>());
        for w in ["a", "b", "c", "d", "e"] {
            c.push(word(w));
        }
        c.drop_front(2);
        let words: Vec<&str> = c.as_slice::<Word>().iter().map(|w| w.text.as_str()).collect();
        assert_eq!(words, ["c", "d", "e"]);
    }

    #[test]
    fn zero_sized_values_need_no_memory() {
        let mut c = ErasedColumn::new(ValueType::of::<Tag>());
        for _ in 0..1000 {
            c.push(Tag {});
        }
        c.swap_remove_drop(500);
        assert_eq!(c.as_slice::<Tag>().len(), 999);
    }

    #[test]
    #[should_panic(expected = "another layout")]
    fn reading_with_another_layout_panics() {
        let mut c = ErasedColumn::new(ValueType::of::<Small>());
        c.push(Small { n: 1 });
        c.as_slice::<Large>();
    }

    #[test]
    fn ticks_follow_their_values() {
        let mut c = ErasedColumn::new(ValueType::of::<Small>());
        let mut to = ErasedColumn::new(ValueType::of::<Small>());
        for n in 0..5 {
            c.push(Small { n });
            c.set_tick(n as usize, 10 + n);
        }
        // 1 leaves for `to`, 4 fills its place; 0 is dropped, 3 fills it.
        c.swap_remove_into(1, &mut to);
        c.swap_remove_drop(0);
        let pairs: Vec<(u32, u32)> = c.as_slice::<Small>().iter().map(|s| s.n).zip(c.ticks().iter().copied()).collect();
        assert_eq!(pairs, [(3, 13), (4, 14), (2, 12)]);
        assert_eq!(to.ticks(), [11]);
        // A page's tick is its latest, even of a value that left.
        assert_eq!((c.written(), to.written()), (14, 11));
        c.drop_front(1);
        assert_eq!(c.ticks(), [14, 12]);
    }

    #[test]
    fn migrating_rewrites_every_value() {
        let mut c = ErasedColumn::new(ValueType::of::<Small>());
        for n in 0..10 {
            c.push(Small { n });
        }
        c.set_tick(3, 7);
        // SAFETY: reads each u32 and writes a whole u64; nothing to drop.
        unsafe {
            c.migrate(ValueType::of::<Large>(), |old, new| (new as *mut Large).write(Large { n: (*(old as *const Small)).n as u64 * 2 }))
        };
        assert_eq!(c.as_slice::<Large>().iter().map(|l| l.n).sum::<u64>(), 90);
        // The same rows, written when they were.
        assert_eq!((c.ticks()[3], c.written()), (7, 7));
    }

    #[test]
    fn over_aligned_values_stay_aligned_as_the_column_grows() {
        #[derive(Default)]
        #[repr(align(64))]
        struct Wide(u8);
        // SAFETY: plain data; no schema.
        unsafe impl Component for Wide {
            const NAME: &'static str = "test::Wide";
        }
        let mut c = ErasedColumn::new(ValueType::of::<Wide>());
        for i in 0..100 {
            c.push(Wide(i));
            assert_eq!(c.as_slice::<Wide>().as_ptr() as usize % 64, 0);
        }
        assert_eq!(c.as_slice::<Wide>()[99].0, 99);
    }
}
