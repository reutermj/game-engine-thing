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
        ErasedColumn { ty, ptr: dangling(ty.layout), len: 0, cap }
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

    pub fn push<T: Component>(&mut self, value: T) {
        self.check::<T>();
        self.reserve_one();
        // SAFETY: slot `len` is allocated (reserve_one) and uninitialized;
        // the type was checked.
        unsafe { (self.slot(self.len) as *mut T).write(value) };
        self.len += 1;
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
            std::ptr::copy_nonoverlapping(self.slot(row), to.slot(to.len), self.ty.layout.size());
            to.len += 1;
            self.fill_from_last(row);
        }
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
        self.len = 0;
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
            unsafe { std::ptr::copy_nonoverlapping(self.slot(last), self.slot(row), self.ty.layout.size()) };
        }
        self.len = last;
    }

    fn reserve_one(&mut self) {
        if self.len < self.cap {
            return;
        }
        let cap = (self.cap * 2).max(4);
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
