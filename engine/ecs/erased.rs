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
        ErasedColumn { ty, ptr: dangling(ty.layout), len: 0, cap, ticks: Vec::new() }
    }

    /// Each value's last-written tick.
    pub fn ticks(&self) -> &[u32] {
        &self.ticks
    }

    pub fn set_tick(&mut self, row: usize, tick: u32) {
        self.ticks[row] = tick;
    }

    /// The values and their ticks, for handing out writes that record
    /// themselves (`Mut`).
    pub fn as_mut_slice_ticked<T: Component>(&mut self) -> (&mut [T], &mut [u32]) {
        self.check::<T>();
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
        to.ticks.push(self.ticks.swap_remove(row));
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
    /// is what makes the moves sound.
    pub fn gather(pages: &mut [ErasedColumn], order: &[(u32, u32)], page_rows: usize) -> Vec<ErasedColumn> {
        let ty = pages.first().expect("a table has a page").ty;
        assert!(pages.iter().all(|p| p.ty.same_values(&ty)), "gathering one column's pages");
        assert_eq!(order.len(), pages.iter().map(|p| p.len).sum::<usize>(), "gathering every value");
        let mut seen: Vec<Vec<bool>> = pages.iter().map(|p| vec![false; p.len]).collect();
        let size = ty.layout.size();
        let mut out = Vec::with_capacity(order.len().div_ceil(page_rows).max(1));
        for chunk in order.chunks(page_rows) {
            let mut column = ErasedColumn::new(ty);
            column.reserve(chunk.len());
            column.ticks.reserve(chunk.len());
            for &(p, r) in chunk {
                let (p, r) = (p as usize, r as usize);
                assert!(r < pages[p].len && !std::mem::replace(&mut seen[p][r], true), "row {r} of page {p} gathered once");
                // SAFETY: `(p, r)` is an initialized value, moved once
                // (checked above) into `column`'s next allocated slot.
                unsafe { std::ptr::copy_nonoverlapping(pages[p].slot(r), column.slot(column.len), size) };
                column.len += 1;
                column.ticks.push(pages[p].ticks[r]);
            }
            out.push(column);
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
    /// fully (see `schema::migrate`).
    ///
    /// # Safety
    /// `migrate(old, new)` must leave `old` fully moved out or dropped and
    /// `new` a valid value of `to`'s type.
    pub unsafe fn migrate(&mut self, to: ValueType, mut migrate: impl FnMut(*mut u8, *mut u8)) {
        let mut out = ErasedColumn::new(to);
        for row in 0..self.len {
            out.reserve_one();
            // SAFETY: `row` is initialized and `out`'s next slot allocated;
            // the caller's `migrate` consumes one and fills the other.
            unsafe { migrate(self.slot(row), out.slot(out.len)) };
            out.len += 1;
        }
        // Every value was moved out or dropped: free the buffer, nothing else.
        // The rows are the same rows, so they keep their ticks.
        self.len = 0;
        out.ticks = std::mem::take(&mut self.ticks);
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
    #[inline(always)]
    unsafe fn fixed<const N: usize>(src: *const u8, dst: *mut u8) {
        unsafe { (dst as *mut [u8; N]).write_unaligned((src as *const [u8; N]).read_unaligned()) }
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
    Layout::from_size_align(layout.size().checked_mul(n).expect("column too large"), layout.align())
        .expect("column layout")
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

    /// Counts its drops.
    #[derive(Default)]
    struct Counted(#[allow(dead_code)] u32);
    static DROPS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    impl Drop for Counted {
        fn drop(&mut self) {
            DROPS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        }
    }
    // SAFETY: plain data; no schema.
    unsafe impl Component for Counted {
        const NAME: &'static str = "test::Counted";
    }

    #[test]
    fn a_moved_value_is_owned_by_its_new_column_and_dropped_once() {
        use std::sync::atomic::Ordering::SeqCst;
        let (mut from, mut to) = (ErasedColumn::new(ValueType::of::<Counted>()), ErasedColumn::new(ValueType::of::<Counted>()));
        let before = DROPS.load(SeqCst);
        for i in 0..3 {
            from.push(Counted(i));
        }
        from.swap_remove_into(0, &mut to);
        assert_eq!((from.len(), to.len()), (2, 1));
        assert_eq!(DROPS.load(SeqCst), before, "a move drops nothing");
        drop(from);
        assert_eq!(DROPS.load(SeqCst), before + 2);
        drop(to);
        assert_eq!(DROPS.load(SeqCst), before + 3);
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
        c.drop_front(1);
        assert_eq!(c.ticks(), [14, 12]);
    }

    #[test]
    fn migrating_rewrites_every_value() {
        let mut c = ErasedColumn::new(ValueType::of::<Small>());
        for n in 0..10 {
            c.push(Small { n });
        }
        // SAFETY: reads each u32 and writes a whole u64; nothing to drop.
        unsafe {
            c.migrate(ValueType::of::<Large>(), |old, new| {
                (new as *mut Large).write(Large { n: (*(old as *const Small)).n as u64 * 2 })
            })
        };
        assert_eq!(c.as_slice::<Large>().iter().map(|l| l.n).sum::<u64>(), 90);
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
