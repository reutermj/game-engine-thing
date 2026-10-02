//! SPIKE, not engine code: what keeping each table column in one block
//! would cost storage, for docs/architecture/contiguous-columns.md. Kept as
//! a bench target so its numbers can be taken again; nothing depends on it,
//! and nothing in `engine_ecs` uses its column.
//!
//! A table here is four columns of the physics bodies' sizes (8, 8, 20 and
//! 24 bytes) and the entities on each page, kept three ways:
//!
//! - **paged**: as storage keeps it today, each page of each column its own
//!   `ErasedColumn` (the engine's own type), grown by doubling up to the
//!   page's rows;
//! - **block (b)**: each column one heap allocation holding whole pages,
//!   page `p` at slots `p * R .. (p + 1) * R`, holes and all; grown by
//!   `realloc` when a page is added past its capacity, which moves it. Only
//!   ever grown under `&mut`, as storage grows a table under its write
//!   guards, so nothing borrowed can see it move;
//! - **reserved (a)**: the same layout in a range of address space reserved
//!   for the column up front (`mmap`, `PROT_NONE`) and committed
//!   (`mprotect`) an OS page at a time as pages are added. It never moves.
//!
//! The bench times what storage does with them: spawning into a table that
//! grows, despawning and spawning a few hundred a frame, rows moving
//! between pages (the spatial re-sort's moves), a whole-table re-sort
//! (`gather`, the ordered tables'), walks by page on one thread and split
//! over threads, a migration to a new layout, and finding a value by
//! (page, row) as a system indexing the column would.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/ecs:contiguous_spike
//!
//! `REPS` (21), the median of each. As a test (`:contiguous_spike_test`,
//! and under Miri `:miri_contiguous_spike_sb` and `_tb`), the block column
//! against a model, with heap values that check they're dropped once.
//!
//! The unsafe code is all in `Block`, the spike's own: landing anything like
//! it is the user's decision (CLAUDE.md, "Growing the unsafe core").

use std::alloc::Layout;
use std::ptr::NonNull;
use std::time::Instant;

use engine_ecs::Component;
use engine_ecs::component;
use engine_ecs::erased::{ErasedColumn, ValueType};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Pos: "spike::Pos" { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Vel: "spike::Vel" { pub x: f32, pub y: f32 }
}

component! {
    /// `Vel` with a third axis: what a reload's migration rewrites to.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Vel3: "spike::Vel" { pub x: f32, pub y: f32, pub z: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Mass: "spike::Mass" { pub a: f32, pub b: f32, pub c: f32, pub d: f32, pub e: u32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Shape: "spike::Shape" { pub a: f32, pub b: f32, pub c: f32, pub d: f32, pub e: f32, pub f: u32 }
}

// ---- The block column ----

/// A value type as the block knows it: a layout and how to drop one.
#[derive(Clone, Copy)]
struct Ty {
    layout: Layout,
    drop: Option<unsafe fn(*mut u8)>,
}

impl Ty {
    fn of<T>() -> Ty {
        /// # Safety
        /// `p` is a live `T`, dropped here once.
        unsafe fn drop_as<T>(p: *mut u8) {
            // SAFETY: the caller's.
            unsafe { std::ptr::drop_in_place(p as *mut T) }
        }
        assert!(Layout::new::<T>().size() > 0, "the spike's block holds sized values");
        Ty { layout: Layout::new::<T>(), drop: std::mem::needs_drop::<T>().then_some(drop_as::<T> as unsafe fn(*mut u8)) }
    }
}

/// Copies one value of `size` bytes, fixed-size where it can, as
/// `erased.rs`'s `copy_value` does (a copy of a size known only at run time
/// is a call to `memcpy`): so a move costs here what it costs there.
///
/// # Safety
/// As `copy_nonoverlapping`: `size` bytes readable at `src`, writable at
/// `dst`, not overlapping.
#[inline(always)]
unsafe fn copy_value(src: *const u8, dst: *mut u8, size: usize) {
    #[inline(always)]
    unsafe fn fixed<const N: usize>(src: *const u8, dst: *mut u8) {
        type Bytes<const N: usize> = std::mem::MaybeUninit<[u8; N]>;
        // SAFETY: the caller's, for `N` bytes; unaligned reads and writes,
        // and as `MaybeUninit`, so padding and pointers survive.
        unsafe { (dst as *mut Bytes<N>).write_unaligned((src as *const Bytes<N>).read_unaligned()) }
    }
    // SAFETY: the caller's, for `size` bytes.
    unsafe {
        match size {
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

/// Where a block's memory comes from.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Mem {
    /// The global allocator, grown by `realloc`, which may move it.
    Heap,
    /// A reserved range of address space this many pages long, committed as
    /// pages are added, never moved.
    Reserved(usize),
}

/// Raw memory for `cap` pages of `bytes` each: on the heap, or reserved.
struct Region {
    ptr: NonNull<u8>,
    align: usize,
    /// Bytes a page takes.
    bytes: usize,
    /// Pages room is made for.
    cap: usize,
    mem: Mem,
    /// Reserved: bytes committed so far (whole OS pages).
    committed: usize,
}

const OS_PAGE: usize = 4096;

impl Region {
    fn new(bytes: usize, align: usize, mem: Mem) -> Region {
        let mut r = Region { ptr: NonNull::dangling(), align, bytes, cap: 0, mem, committed: 0 };
        if let Mem::Reserved(pages) = mem {
            let len = (pages * bytes).next_multiple_of(OS_PAGE);
            r.ptr = os::reserve(len);
        }
        r
    }

    /// Room for at least `pages` pages; may move the memory (on the heap).
    fn grow(&mut self, pages: usize) {
        if pages <= self.cap {
            return;
        }
        match self.mem {
            Mem::Heap => {
                // Exactly what's asked at first (a lean page is one page),
                // then doubling.
                let cap = if self.cap == 0 { pages } else { pages.max(self.cap * 2) };
                let new = Layout::from_size_align(cap * self.bytes, self.align).expect("a block's layout");
                // SAFETY: a non-zero size (pages hold sized values); a block
                // that has memory grows it with the layout it was made with.
                let ptr = unsafe {
                    if self.cap == 0 {
                        std::alloc::alloc(new)
                    } else {
                        let old = Layout::from_size_align(self.cap * self.bytes, self.align).unwrap();
                        std::alloc::realloc(self.ptr.as_ptr(), old, new.size())
                    }
                };
                self.ptr = NonNull::new(ptr).unwrap_or_else(|| std::alloc::handle_alloc_error(new));
                self.cap = cap;
            }
            Mem::Reserved(limit) => {
                assert!(pages <= limit, "a reserved column holds at most {limit} pages");
                // Committed in doubling chunks, at least 64 KiB, as a `Vec`
                // grows: a call to `mprotect` per OS page made growing four
                // to five times the heap block's cost (2026-10-02).
                let reserved = (limit * self.bytes).next_multiple_of(OS_PAGE);
                let want = (pages * self.bytes).next_multiple_of(OS_PAGE).max(self.committed * 2).max(1 << 16).min(reserved);
                if want > self.committed {
                    // SAFETY: inside the reservation (`want` is at most its
                    // length, checked above), which this region owns.
                    unsafe { os::commit(self.ptr.as_ptr().add(self.committed), want - self.committed) };
                    self.committed = want;
                }
                self.cap = self.committed / self.bytes;
            }
        }
    }
}

impl Drop for Region {
    fn drop(&mut self) {
        match self.mem {
            Mem::Heap if self.cap > 0 => {
                // SAFETY: allocated with this layout, by `grow`.
                unsafe { std::alloc::dealloc(self.ptr.as_ptr(), Layout::from_size_align(self.cap * self.bytes, self.align).unwrap()) }
            }
            Mem::Heap => (),
            Mem::Reserved(pages) => os::release(self.ptr, (pages * self.bytes).next_multiple_of(OS_PAGE)),
        }
    }
}

/// The OS calls a reserved block makes, declared as the loader's poison
/// mode declares them (no `libc` crate in the workspace for this).
#[cfg(not(miri))]
mod os {
    use std::ffi::{c_int, c_void};
    use std::ptr::NonNull;

    const PROT_NONE: c_int = 0;
    const PROT_READ: c_int = 1;
    const PROT_WRITE: c_int = 2;
    const MAP_PRIVATE: c_int = 0x02;
    const MAP_ANONYMOUS: c_int = 0x20;
    const MAP_NORESERVE: c_int = 0x4000;
    const MAP_FAILED: *mut c_void = !0usize as *mut c_void;

    unsafe extern "C" {
        fn mmap(addr: *mut c_void, len: usize, prot: c_int, flags: c_int, fd: c_int, off: i64) -> *mut c_void;
        fn mprotect(addr: *mut c_void, len: usize, prot: c_int) -> c_int;
        fn munmap(addr: *mut c_void, len: usize) -> c_int;
    }

    /// `len` bytes of address space, inaccessible until committed.
    pub fn reserve(len: usize) -> NonNull<u8> {
        // SAFETY: a fresh anonymous mapping anywhere: it aliases nothing.
        let p = unsafe { mmap(std::ptr::null_mut(), len, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0) };
        assert!(p != MAP_FAILED, "reserving {len} bytes");
        NonNull::new(p as *mut u8).expect("a mapping isn't at null")
    }

    /// Makes `len` bytes at `at` readable and writable (zeroed, as fresh
    /// anonymous memory is).
    ///
    /// # Safety
    /// `at .. at + len` is inside a reservation, page-aligned.
    pub unsafe fn commit(at: *mut u8, len: usize) {
        // SAFETY: the caller's.
        let r = unsafe { mprotect(at as *mut c_void, len, PROT_READ | PROT_WRITE) };
        assert_eq!(r, 0, "committing {len} bytes");
    }

    pub fn release(at: NonNull<u8>, len: usize) {
        // SAFETY: the whole reservation, which its region owned and drops.
        let r = unsafe { munmap(at.as_ptr() as *mut c_void, len) };
        assert_eq!(r, 0, "releasing {len} bytes");
    }
}

/// Under Miri, which has no `mprotect`, a reservation is an allocation of
/// its whole length up front: the same addresses never move, which is the
/// property the tests check.
#[cfg(miri)]
mod os {
    use std::alloc::Layout;
    use std::ptr::NonNull;

    pub fn reserve(len: usize) -> NonNull<u8> {
        // SAFETY: a non-zero size.
        NonNull::new(unsafe { std::alloc::alloc(Layout::from_size_align(len, 4096).unwrap()) }).expect("reserved")
    }

    /// # Safety
    /// Nothing to do: allocated whole.
    pub unsafe fn commit(_: *mut u8, _: usize) {}

    pub fn release(at: NonNull<u8>, len: usize) {
        // SAFETY: allocated by `reserve` with this layout.
        unsafe { std::alloc::dealloc(at.as_ptr(), Layout::from_size_align(len, 4096).unwrap()) }
    }
}

/// A column kept in one block: page `p`'s rows at slots `p * rows ..`, the
/// first `lens[p]` of them live; each slot's tick beside it in a block of
/// the same shape.
///
/// Invariants, all local to this type: `values` and `ticks` have room for
/// at least `lens.len()` pages; the first `lens[p]` slots of page `p` hold
/// values of `ty`, and no other slot does.
struct Block {
    ty: Ty,
    rows: usize,
    values: Region,
    ticks: Region,
    lens: Vec<u32>,
}

impl Block {
    fn new(ty: Ty, rows: usize, mem: Mem) -> Block {
        let values = Region::new(rows * ty.layout.size(), ty.layout.align(), mem);
        let ticks = Region::new(rows * 4, 4, mem);
        Block { ty, rows, values, ticks, lens: Vec::new() }
    }

    fn size(&self) -> usize {
        self.ty.layout.size()
    }

    /// # Safety
    /// `(page, row)` is within the room made: `page < cap`, `row < rows`.
    #[inline(always)]
    unsafe fn slot(&self, page: usize, row: usize) -> *mut u8 {
        // SAFETY: the caller's; within the region's allocation.
        unsafe { self.values.ptr.as_ptr().add((page * self.rows + row) * self.size()) }
    }

    #[inline(always)]
    fn tick_at(&self, page: usize, row: usize) -> *mut u32 {
        assert!(page < self.lens.len() && row < self.rows);
        // SAFETY: room for `lens.len()` pages of ticks (the invariant).
        unsafe { (self.ticks.ptr.as_ptr() as *mut u32).add(page * self.rows + row) }
    }

    fn pages(&self) -> usize {
        self.lens.len()
    }

    fn add_page(&mut self) {
        let n = self.lens.len() + 1;
        self.values.grow(n);
        self.ticks.grow(n);
        self.lens.push(0);
    }

    fn check<T>(&self) {
        assert!(Layout::new::<T>() == self.ty.layout, "a block's values are one type");
    }

    fn push<T>(&mut self, page: usize, value: T, tick: u32) {
        self.check::<T>();
        let row = self.lens[page] as usize;
        assert!(row < self.rows, "a page has room");
        // SAFETY: `(page, row)` is room made (page in use, row under the
        // page's rows) holding no value; written, then counted.
        unsafe {
            (self.slot(page, row) as *mut T).write(value);
            *self.tick_at(page, row) = tick;
        }
        self.lens[page] += 1;
    }

    /// Drops row `row` of `page`, its page's last row moved into its place.
    fn swap_remove_drop(&mut self, page: usize, row: usize) {
        let last = self.lens[page] as usize - 1;
        assert!(row <= last, "a live row");
        // SAFETY: `row` and `last` are live slots of `page`; the value at
        // `row` is dropped once, then the last value moved over it and no
        // longer counted.
        unsafe {
            if let Some(drop) = self.ty.drop {
                drop(self.slot(page, row));
            }
            if row != last {
                copy_value(self.slot(page, last), self.slot(page, row), self.size());
                *self.tick_at(page, row) = *self.tick_at(page, last);
            }
        }
        self.lens[page] -= 1;
    }

    /// Moves row `row` of `from` to the end of page `to` (another page),
    /// `from`'s last row moved into its place: `swap_remove_into`.
    fn move_row(&mut self, (from, row): (usize, usize), to: usize) {
        assert!(from != to, "a move between pages");
        let (last, at) = (self.lens[from] as usize - 1, self.lens[to] as usize);
        assert!(row <= last && at < self.rows, "a live row, to a page with room");
        // SAFETY: `(from, row)` and `(from, last)` are live, `(to, at)` is
        // room holding nothing; slots of different pages don't overlap. The
        // value moves to `to`, the last of `from` into its place, each
        // counted once.
        unsafe {
            copy_value(self.slot(from, row), self.slot(to, at), self.size());
            *self.tick_at(to, at) = *self.tick_at(from, row);
            if row != last {
                copy_value(self.slot(from, last), self.slot(from, row), self.size());
                *self.tick_at(from, row) = *self.tick_at(from, last);
            }
        }
        self.lens[to] += 1;
        self.lens[from] -= 1;
    }

    fn page<T>(&self, p: usize) -> &[T] {
        self.check::<T>();
        // SAFETY: the first `lens[p]` slots of page `p` are live `T`s
        // (checked type), borrowed with `self`.
        unsafe { std::slice::from_raw_parts(self.slot(p, 0) as *const T, self.lens[p] as usize) }
    }

    fn page_mut<T>(&mut self, p: usize) -> &mut [T] {
        self.check::<T>();
        // SAFETY: as `page`, borrowed mutably with `self`.
        unsafe { std::slice::from_raw_parts_mut(self.slot(p, 0) as *mut T, self.lens[p] as usize) }
    }

    /// Every page's live rows at once, each its own `&mut`: the block split
    /// at page boundaries, so the borrows are disjoint by construction, as
    /// `split_at_mut` makes them. What a parallel walk hands its tasks.
    fn pages_mut<T>(&mut self) -> Vec<&mut [T]> {
        self.check::<T>();
        let (rows, n) = (self.rows, self.lens.len());
        // SAFETY: room for `n` pages of `rows` slots each is allocated (or
        // committed), borrowed mutably with `self`; as `MaybeUninit`, the
        // holes are fine to cover.
        let all: &mut [std::mem::MaybeUninit<T>] =
            unsafe { std::slice::from_raw_parts_mut(self.values.ptr.as_ptr() as *mut std::mem::MaybeUninit<T>, n * rows) };
        all.chunks_mut(rows)
            .zip(&self.lens)
            .map(|(page, &len)| {
                let live = &mut page[..len as usize];
                // SAFETY: the first `len` slots of a page are initialized
                // `T`s (the invariant); `MaybeUninit<T>` has `T`'s layout.
                unsafe { &mut *(live as *mut [std::mem::MaybeUninit<T>] as *mut [T]) }
            })
            .collect()
    }

    /// Every value moved into a new block in `order`, pages of `rows` filled
    /// in turn, ticks kept; this block left empty: an ordered table's
    /// re-sort. Panics before moving anything unless `order` names every
    /// live value once.
    fn gather(&mut self, order: &[(u32, u32)]) -> Block {
        let mut seen: Vec<Vec<bool>> = self.lens.iter().map(|&n| vec![false; n as usize]).collect();
        for &(p, r) in order {
            let s = seen.get_mut(p as usize).and_then(|page| page.get_mut(r as usize));
            assert!(s.is_some_and(|s| !std::mem::replace(s, true)), "row {r} of page {p} gathered once");
        }
        assert_eq!(order.len(), self.lens.iter().map(|&n| n as usize).sum::<usize>(), "gathering every value");
        let mut out = Block::new(self.ty, self.rows, self.values.mem);
        let pages = order.len().div_ceil(self.rows);
        out.values.grow(pages);
        out.ticks.grow(pages);
        out.lens = vec![0; pages];
        for (k, &(p, r)) in order.iter().enumerate() {
            let (q, at) = (k / self.rows, k % self.rows);
            // SAFETY: `(p, r)` live and moved once (checked above), into
            // `(q, at)`, room in the new block holding nothing.
            unsafe {
                copy_value(self.slot(p as usize, r as usize), out.slot(q, at), self.size());
                *out.tick_at(q, at) = *self.tick_at(p as usize, r as usize);
            }
            out.lens[q] += 1;
        }
        // Every value moved out: this block owns none.
        self.lens.iter_mut().for_each(|n| *n = 0);
        out
    }

    /// Each value rewritten as `f` makes a `U` of it, into a new block of
    /// the same pages, which this becomes: a reload's migration. The old
    /// values are dropped with the old block.
    fn migrate<T, U>(&mut self, mut f: impl FnMut(&T) -> U) {
        self.check::<T>();
        let mut out = Block::new(Ty::of::<U>(), self.rows, self.values.mem);
        for p in 0..self.pages() {
            out.add_page();
            for r in 0..self.lens[p] as usize {
                // SAFETY: `(p, r)` is a live `T` (checked type).
                let v = f(unsafe { &*(self.slot(p, r) as *const T) });
                let tick = *self.tick_at_read(p, r);
                out.push(p, v, tick);
            }
        }
        drop(std::mem::replace(self, out));
    }

    fn tick_at_read(&self, page: usize, row: usize) -> &u32 {
        // SAFETY: a slot of a page in use, read with `self`.
        unsafe { &*self.tick_at(page, row) }
    }

    /// Moves row `row` of page `from` to the end of page `to` of another
    /// block, `from`'s last row moved into its place: `move_row` across
    /// blocks, for pages kept one block each.
    fn move_to(&mut self, (from, row): (usize, usize), other: &mut Block, to: usize) {
        assert!(self.ty.layout == other.ty.layout, "one type");
        let (last, at) = (self.lens[from] as usize - 1, other.lens[to] as usize);
        assert!(row <= last && at < other.rows, "a live row, to a page with room");
        // SAFETY: as `move_row`, the destination in another block, which
        // `&mut` keeps apart from this one.
        unsafe {
            copy_value(self.slot(from, row), other.slot(to, at), self.size());
            *other.tick_at(to, at) = *self.tick_at(from, row);
            if row != last {
                copy_value(self.slot(from, last), self.slot(from, row), self.size());
                *self.tick_at(from, row) = *self.tick_at(from, last);
            }
        }
        other.lens[to] += 1;
        self.lens[from] -= 1;
    }

    /// Copies row `row` of page `from` to the end of page `to` of `other`,
    /// leaving it counted here: half of a gather, whose caller then has
    /// every block forget its rows.
    ///
    /// # Safety
    /// The value is moved: the caller copies each value once, then has this
    /// block forget it (`forget`) before anything reads or drops it here.
    unsafe fn copy_to(&self, (from, row): (usize, usize), other: &mut Block, to: usize) {
        let at = other.lens[to] as usize;
        assert!(row < self.lens[from] as usize && at < other.rows, "a live row, to a page with room");
        // SAFETY: a live value, to room holding nothing, in another block.
        unsafe {
            copy_value(self.slot(from, row), other.slot(to, at), self.size());
            *other.tick_at(to, at) = *self.tick_at(from, row);
        }
        other.lens[to] += 1;
    }

    /// Owns no values any more: they've all been moved out.
    fn forget(&mut self) {
        self.lens.iter_mut().for_each(|n| *n = 0);
    }
}

impl Drop for Block {
    fn drop(&mut self) {
        if let Some(drop) = self.ty.drop {
            for p in 0..self.lens.len() {
                for r in 0..self.lens[p] as usize {
                    // SAFETY: each live value, dropped once; the regions
                    // free the memory after.
                    unsafe { drop(self.slot(p, r)) };
                }
            }
        }
    }
}

// ---- The three columns behind one interface ----

/// A table column, kept some way.
trait Column {
    const NAME: &'static str;
    fn new<T: Component>(rows: usize) -> Self;
    fn add_page(&mut self);
    fn push<T: Component>(&mut self, page: usize, v: T, tick: u32);
    fn swap_remove_drop(&mut self, page: usize, row: usize);
    fn move_row(&mut self, from: (usize, usize), to: usize);
    fn page<T: Component>(&self, p: usize) -> &[T];
    fn page_mut<T: Component>(&mut self, p: usize) -> &mut [T];
    fn pages_mut<T: Component>(&mut self) -> Vec<&mut [T]>;
    fn gather(&mut self, order: &[(u32, u32)], rows: usize) -> Self;
    fn migrate<T: Component, U: Component>(&mut self, f: impl Fn(&T) -> U);
    /// Where page `p`'s values start, as a system indexing the column
    /// would hold them: a pointer per page, or one for the whole column.
    fn base(&self) -> *const u8;
    fn page_ptrs(&self) -> Vec<*const u8>;
}

/// Today's: a page per `ErasedColumn`.
struct Paged {
    ty: ValueType,
    pages: Vec<ErasedColumn>,
}

impl Column for Paged {
    const NAME: &'static str = "paged (today)";
    fn new<T: Component>(_: usize) -> Paged {
        Paged { ty: ValueType::of::<T>(), pages: Vec::new() }
    }
    fn add_page(&mut self) {
        self.pages.push(ErasedColumn::new(self.ty));
    }
    #[inline]
    fn push<T: Component>(&mut self, page: usize, v: T, tick: u32) {
        let c = &mut self.pages[page];
        c.push(v);
        c.set_tick(c.len() - 1, tick);
    }
    #[inline]
    fn swap_remove_drop(&mut self, page: usize, row: usize) {
        self.pages[page].swap_remove_drop(row);
    }
    #[inline]
    fn move_row(&mut self, (from, row): (usize, usize), to: usize) {
        let [a, b] = self.pages.get_disjoint_mut([from, to]).expect("two pages");
        a.swap_remove_into(row, b);
    }
    fn page<T: Component>(&self, p: usize) -> &[T] {
        self.pages[p].as_slice()
    }
    fn page_mut<T: Component>(&mut self, p: usize) -> &mut [T] {
        self.pages[p].as_mut_slice()
    }
    fn pages_mut<T: Component>(&mut self) -> Vec<&mut [T]> {
        self.pages.iter_mut().map(|c| c.as_mut_slice()).collect()
    }
    fn gather(&mut self, order: &[(u32, u32)], rows: usize) -> Paged {
        Paged { ty: self.ty, pages: ErasedColumn::gather(&mut self.pages, order, rows) }
    }
    fn migrate<T: Component, U: Component>(&mut self, f: impl Fn(&T) -> U) {
        self.ty = ValueType::of::<U>();
        for c in self.pages.iter_mut() {
            // SAFETY: each old value is read as the `T` it is and left to be
            // forgotten (they're `Copy`, so nothing to drop), and each new
            // slot written with a whole `U`.
            unsafe { c.migrate(ValueType::of::<U>(), |old, new| (new as *mut U).write(f(&*(old as *const T)))) };
        }
    }
    fn base(&self) -> *const u8 {
        std::ptr::null()
    }
    fn page_ptrs(&self) -> Vec<*const u8> {
        self.pages.iter().map(|c| c.value_ptr(0)).collect()
    }
}

/// Pages as separate allocations, as today, but each the spike's own lean
/// block of one page made whole at once: what separate pages cost apart
/// from `ErasedColumn`'s own checks and its growth by doubling within a
/// page.
struct Lean {
    ty: Ty,
    rows: usize,
    pages: Vec<Block>,
}

impl Column for Lean {
    const NAME: &'static str = "lean pages";
    fn new<T: Component>(rows: usize) -> Lean {
        Lean { ty: Ty::of::<T>(), rows, pages: Vec::new() }
    }
    fn add_page(&mut self) {
        let mut b = Block::new(self.ty, self.rows, Mem::Heap);
        b.add_page();
        self.pages.push(b);
    }
    #[inline]
    fn push<T: Component>(&mut self, page: usize, v: T, tick: u32) {
        self.pages[page].push(0, v, tick);
    }
    #[inline]
    fn swap_remove_drop(&mut self, page: usize, row: usize) {
        self.pages[page].swap_remove_drop(0, row);
    }
    #[inline]
    fn move_row(&mut self, (from, row): (usize, usize), to: usize) {
        let [a, b] = self.pages.get_disjoint_mut([from, to]).expect("two pages");
        a.move_to((0, row), b, 0);
    }
    fn page<T: Component>(&self, p: usize) -> &[T] {
        self.pages[p].page(0)
    }
    fn page_mut<T: Component>(&mut self, p: usize) -> &mut [T] {
        self.pages[p].page_mut(0)
    }
    fn pages_mut<T: Component>(&mut self) -> Vec<&mut [T]> {
        self.pages.iter_mut().map(|b| b.page_mut(0)).collect()
    }
    fn gather(&mut self, order: &[(u32, u32)], rows: usize) -> Lean {
        let mut out = Lean { ty: self.ty, rows, pages: Vec::new() };
        for _ in 0..order.len().div_ceil(rows) {
            out.add_page();
        }
        for (k, &(p, r)) in order.iter().enumerate() {
            // SAFETY: the bench's orders name every row once; every page
            // forgets its rows below, so each value is moved once.
            unsafe { self.pages[p as usize].copy_to((0, r as usize), &mut out.pages[k / rows], 0) };
        }
        self.pages.iter_mut().for_each(Block::forget);
        out
    }
    fn migrate<T: Component, U: Component>(&mut self, f: impl Fn(&T) -> U) {
        self.ty = Ty::of::<U>();
        for b in self.pages.iter_mut() {
            b.migrate::<T, U>(&f);
        }
    }
    fn base(&self) -> *const u8 {
        std::ptr::null()
    }
    fn page_ptrs(&self) -> Vec<*const u8> {
        self.pages.iter().map(|b| b.values.ptr.as_ptr() as *const u8).collect()
    }
}

/// A block on the heap (b), or reserved (a, `RESERVED`).
struct Blocked<const RESERVED: bool>(Block);

/// Pages a reserved column has address space for: enough for the bench's
/// largest table at 16 rows a page.
const RESERVE_PAGES: usize = 1 << 16;

impl<const RESERVED: bool> Column for Blocked<RESERVED> {
    const NAME: &'static str = if RESERVED { "reserved (a)" } else { "block (b)" };
    fn new<T: Component>(rows: usize) -> Self {
        Blocked(Block::new(Ty::of::<T>(), rows, if RESERVED { Mem::Reserved(RESERVE_PAGES) } else { Mem::Heap }))
    }
    fn add_page(&mut self) {
        self.0.add_page();
    }
    #[inline]
    fn push<T: Component>(&mut self, page: usize, v: T, tick: u32) {
        self.0.push(page, v, tick);
    }
    #[inline]
    fn swap_remove_drop(&mut self, page: usize, row: usize) {
        self.0.swap_remove_drop(page, row);
    }
    #[inline]
    fn move_row(&mut self, from: (usize, usize), to: usize) {
        self.0.move_row(from, to);
    }
    fn page<T: Component>(&self, p: usize) -> &[T] {
        self.0.page(p)
    }
    fn page_mut<T: Component>(&mut self, p: usize) -> &mut [T] {
        self.0.page_mut(p)
    }
    fn pages_mut<T: Component>(&mut self) -> Vec<&mut [T]> {
        self.0.pages_mut()
    }
    fn gather(&mut self, order: &[(u32, u32)], _: usize) -> Self {
        Blocked(self.0.gather(order))
    }
    fn migrate<T: Component, U: Component>(&mut self, f: impl Fn(&T) -> U) {
        self.0.migrate::<T, U>(f);
    }
    fn base(&self) -> *const u8 {
        self.0.values.ptr.as_ptr()
    }
    fn page_ptrs(&self) -> Vec<*const u8> {
        // SAFETY: page starts within the room made.
        (0..self.0.pages()).map(|p| unsafe { self.0.slot(p, 0) as *const u8 }).collect()
    }
}

/// A table: four columns and the entities on each page.
struct Table<C> {
    rows: usize,
    entities: Vec<Vec<u32>>,
    pos: C,
    vel: C,
    mass: C,
    shape: C,
    tick: u32,
}

fn body(i: u32) -> (Pos, Vel, Mass, Shape) {
    let f = i as f32;
    (
        Pos { x: f, y: -f },
        Vel { x: 0.5, y: f * 0.25 },
        Mass { a: 1.0, b: f, c: 2.0, d: 3.0, e: i },
        Shape { a: 0.5, f: i, ..Shape::default() },
    )
}

impl<C: Column> Table<C> {
    fn new(rows: usize) -> Table<C> {
        Table {
            rows,
            entities: Vec::new(),
            pos: C::new::<Pos>(rows),
            vel: C::new::<Vel>(rows),
            mass: C::new::<Mass>(rows),
            shape: C::new::<Shape>(rows),
            tick: 0,
        }
    }

    fn add_page(&mut self) {
        self.entities.push(Vec::new());
        self.pos.add_page();
        self.vel.add_page();
        self.mass.add_page();
        self.shape.add_page();
    }

    /// A row at the end of the last page, a page added if it's full.
    fn spawn(&mut self, e: u32) {
        if self.entities.last().is_none_or(|p| p.len() == self.rows) {
            self.add_page();
        }
        self.push(self.entities.len() - 1, e);
    }

    fn push(&mut self, p: usize, e: u32) {
        let (a, b, c, d) = body(e);
        self.tick += 1;
        self.pos.push(p, a, self.tick);
        self.vel.push(p, b, self.tick);
        self.mass.push(p, c, self.tick);
        self.shape.push(p, d, self.tick);
        self.entities[p].push(e);
    }

    fn despawn(&mut self, p: usize, r: usize) {
        self.pos.swap_remove_drop(p, r);
        self.vel.swap_remove_drop(p, r);
        self.mass.swap_remove_drop(p, r);
        self.shape.swap_remove_drop(p, r);
        self.entities[p].swap_remove(r);
    }

    fn move_row(&mut self, from: (usize, usize), to: usize) {
        self.pos.move_row(from, to);
        self.vel.move_row(from, to);
        self.mass.move_row(from, to);
        self.shape.move_row(from, to);
        let e = self.entities[from.0].swap_remove(from.1);
        self.entities[to].push(e);
    }

    fn len(&self) -> usize {
        self.entities.iter().map(Vec::len).sum()
    }

    /// `pos += vel * dt`, page by page: what a system's page walk does.
    fn walk(&mut self) {
        for p in 0..self.entities.len() {
            let v = self.vel.page::<Vel>(p);
            let x = self.pos.page_mut::<Pos>(p);
            for (x, v) in x.iter_mut().zip(v) {
                x.x += v.x * 0.016;
                x.y += v.y * 0.016;
            }
        }
    }

    /// The same, the pages carved into `threads` runs, each a thread's:
    /// what `par_for_each_page` hands its tasks.
    fn par_walk(&mut self, threads: usize) -> (f64, f64) {
        let start = Instant::now();
        let n = self.entities.len();
        let mut xs = self.pos.pages_mut::<Pos>();
        let vs: Vec<&[Vel]> = (0..n).map(|p| self.vel.page::<Vel>(p)).collect();
        let mut runs = Vec::new();
        for k in (0..threads).rev() {
            let at = n * k / threads;
            runs.push((xs.split_off(at), &vs[at..]));
        }
        let carved = us(start);
        std::thread::scope(|s| {
            for (xs, vs) in runs {
                s.spawn(move || {
                    for (x, v) in xs.into_iter().zip(vs) {
                        for (x, v) in x.iter_mut().zip(v.iter()) {
                            x.x += v.x * 0.016;
                            x.y += v.y * 0.016;
                        }
                    }
                });
            }
        });
        (carved, us(start))
    }

    fn checksum(&self) -> f64 {
        (0..self.entities.len()).map(|p| self.pos.page::<Pos>(p).iter().map(|x| (x.x + x.y) as f64).sum::<f64>()).sum()
    }
}

fn us(since: Instant) -> f64 {
    since.elapsed().as_secs_f64() * 1e6
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(f64::total_cmp);
    xs[xs.len() / 2]
}

/// A small xorshift, so every way meets the same choices.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// A table of `n` rows in pages of `rows`, each page then thinned to about
/// `keep` rows by despawning, as spatial pages run 9 to 11 of 16.
fn thinned<C: Column>(n: usize, rows: usize, keep: usize) -> Table<C> {
    let mut t = Table::<C>::new(rows);
    let full = n * rows / keep;
    for e in 0..full as u32 {
        t.spawn(e);
    }
    for p in 0..t.entities.len() {
        while t.entities[p].len() > keep {
            t.despawn(p, 0);
        }
    }
    t
}

/// Times each way, as one row of a table of µs or ns.
fn row<C: Column>(reps: usize, label: &str, unit: f64, mut setup_run: impl FnMut() -> f64) -> (String, f64) {
    let _ = C::NAME;
    let times: Vec<f64> = (0..reps).map(|_| setup_run()).collect();
    (label.to_string(), median(times) * unit)
}

fn bench<C: Column>(reps: usize, threads: usize) -> Vec<(String, f64)> {
    let mut out = Vec::new();
    // Spawning into a table that grows from nothing.
    for (rows, n) in [(256usize, 100_000usize), (16, 100_000)] {
        out.push(row::<C>(reps, &format!("spawn {n} into an empty table, pages of {rows} (ns a row)"), 1e3 / n as f64, || {
            let mut t = Table::<C>::new(rows);
            let start = Instant::now();
            for e in 0..n as u32 {
                t.spawn(e);
            }
            let us = us(start);
            assert_eq!(t.len(), n);
            us
        }));
    }
    // Despawning and spawning a few hundred a frame, spread over the pages.
    out.push(row::<C>(reps, "300 despawned and 300 spawned a frame, 10 000 rows in pages of 16 (ns each)", 1e3 / 600.0, {
        let mut t = thinned::<C>(10_000, 16, 10);
        let mut rng = Rng(0x2545_f491_4f6c_dd1d);
        let mut next = 1_000_000u32;
        move || {
            let start = Instant::now();
            for _ in 0..300 {
                let mut p = rng.below(t.entities.len());
                while t.entities[p].is_empty() {
                    p = rng.below(t.entities.len());
                }
                let r = rng.below(t.entities[p].len());
                t.despawn(p, r);
            }
            for _ in 0..300 {
                t.spawn(next);
                next += 1;
            }
            us(start)
        }
    }));
    // Rows moving between pages: the spatial re-sort's moves.
    for share in [5usize, 18] {
        out.push(row::<C>(
            reps,
            &format!("{share}% of 10 000 rows moved between pages of 16 (ns a move)"),
            1e3 / (100.0 * share as f64),
            {
                let mut t = thinned::<C>(10_000, 16, 10);
                let mut rng = Rng(0x9e37_79b9_7f4a_7c15 ^ share as u64);
                move || {
                    let start = Instant::now();
                    let mut moved = 0;
                    while moved < 100 * share {
                        let (p, q) = (rng.below(t.entities.len()), rng.below(t.entities.len()));
                        if p == q || t.entities[p].is_empty() || t.entities[q].len() == t.rows {
                            continue;
                        }
                        let r = rng.below(t.entities[p].len());
                        t.move_row((p, r), q);
                        moved += 1;
                    }
                    us(start)
                }
            },
        ));
    }
    // A whole-table re-sort: every row into a new order (an ordered table's).
    out.push(row::<C>(reps, "re-sort 10 000 rows in pages of 16 into a new order, every column (µs)", 1.0, || {
        let mut t = thinned::<C>(10_000, 16, 10);
        let mut order: Vec<(u32, u32)> =
            (0..t.entities.len()).flat_map(|p| (0..t.entities[p].len()).map(move |r| (p as u32, r as u32))).collect();
        let mut rng = Rng(7);
        for i in (1..order.len()).rev() {
            order.swap(i, rng.below(i + 1));
        }
        let start = Instant::now();
        let rows = t.rows;
        let (pos, vel, mass, shape) =
            (t.pos.gather(&order, rows), t.vel.gather(&order, rows), t.mass.gather(&order, rows), t.shape.gather(&order, rows));
        let us = us(start);
        drop((pos, vel, mass, shape));
        us
    }));
    // Walks.
    for (rows, keep) in [(256usize, 256usize), (16, 10)] {
        out.push(row::<C>(reps, &format!("walk 100 000 rows, pages of {rows} holding {keep}: pos += vel (µs)"), 1.0, {
            let mut t = thinned::<C>(100_000, rows, keep);
            move || {
                let start = Instant::now();
                t.walk();
                let us = us(start);
                std::hint::black_box(t.checksum());
                us
            }
        }));
        out.push(row::<C>(reps, &format!("the same split over {threads} threads (spawned per walk): carving the pages (µs)"), 1.0, {
            let mut t = thinned::<C>(100_000, rows, keep);
            move || t.par_walk(threads).0
        }));
        out.push(row::<C>(reps, &format!("the same split over {threads} threads (spawned per walk): all (µs)"), 1.0, {
            let mut t = thinned::<C>(100_000, rows, keep);
            move || t.par_walk(threads).1
        }));
    }
    // A reload's migration of one column to a new layout.
    out.push(row::<C>(reps, "migrate 100 000 rows of `Vel` (8 bytes) to `Vel3` (12), pages of 256 (µs)", 1.0, || {
        let mut t = thinned::<C>(100_000, 256, 256);
        let start = Instant::now();
        t.vel.migrate::<Vel, Vel3>(|v| Vel3 { x: v.x, y: v.y, z: 0.0 });
        let us = us(start);
        assert_eq!(t.vel.page::<Vel3>(0)[3], Vel3 { x: 0.5, y: 0.75, z: 0.0 });
        us
    }));
    // Finding a value by (page, row): a pointer per page, or the column's one
    // base pointer (holes and all, as the in-place solve indexes it).
    out.push(row::<C>(reps, "1 000 000 values read at random (page, row), 16-row pages of 10 000 rows (µs)", 1.0, {
        let t = thinned::<C>(10_000, 16, 10);
        let mut rng = Rng(11);
        let at: Vec<(u32, u32)> = (0..1_000_000)
            .map(|_| {
                let p = rng.below(t.entities.len());
                (p as u32, rng.below(t.entities[p].len()) as u32)
            })
            .collect();
        let (base, pages) = (t.vel.base(), t.vel.page_ptrs());
        move || {
            // The pointers are into `t`'s columns: it lives as long as this.
            let _ = &t;
            let start = Instant::now();
            let mut sum = 0.0f32;
            if base.is_null() {
                for &(p, r) in &at {
                    // SAFETY: a live row of a page the table keeps, read only.
                    sum += unsafe { (*(pages[p as usize] as *const Vel).add(r as usize)).y };
                }
            } else {
                for &(p, r) in &at {
                    // SAFETY: as above, by the column's one base.
                    sum += unsafe { (*(base as *const Vel).add(p as usize * 16 + r as usize)).y };
                }
            }
            std::hint::black_box(sum);
            us(start)
        }
    }));
    out
}

#[cfg_attr(test, allow(dead_code))]
fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(21);
    let threads = 8;
    let paged = bench::<Paged>(reps, threads);
    let lean = bench::<Lean>(reps, threads);
    let block = bench::<Blocked<false>>(reps, threads);
    let reserved = bench::<Blocked<true>>(reps, threads);
    println!("Columns kept four ways, the median of {reps}\n");
    println!("| | {} | {} | {} | {} |", Paged::NAME, Lean::NAME, Blocked::<false>::NAME, Blocked::<true>::NAME);
    println!("|---|---|---|---|---|");
    for (((p, l), b), r) in paged.iter().zip(&lean).zip(&block).zip(&reserved) {
        println!("| {} | {:.1} | {:.1} | {:.1} | {:.1} |", p.0, p.1, l.1, b.1, r.1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    thread_local! {
        static LIVE: Cell<i64> = const { Cell::new(0) };
    }

    /// A heap value that counts itself and checks it's alive when read or
    /// dropped, as `ops.rs`'s canaries do: a double drop, a leak or a read of
    /// a moved-out value fails where it happens.
    #[derive(Debug)]
    struct Canary(Box<[u64; 2]>);

    const ALIVE: u64 = 0x9E37_79B9_7F4A_7C15;

    impl Canary {
        fn new(id: u64) -> Canary {
            LIVE.with(|c| c.set(c.get() + 1));
            Canary(Box::new([id, id ^ ALIVE]))
        }
        fn id(&self) -> u64 {
            assert_eq!(self.0[1], self.0[0] ^ ALIVE, "a canary read after it was dropped");
            self.0[0]
        }
    }

    impl Drop for Canary {
        fn drop(&mut self) {
            self.id();
            self.0[1] = 0;
            LIVE.with(|c| c.set(c.get() - 1));
        }
    }

    fn ids(b: &Block) -> Vec<Vec<u64>> {
        (0..b.pages()).map(|p| b.page::<Canary>(p).iter().map(Canary::id).collect()).collect()
    }

    fn ticks(b: &Block) -> Vec<Vec<u32>> {
        (0..b.pages()).map(|p| (0..b.lens[p] as usize).map(|r| *b.tick_at_read(p, r)).collect()).collect()
    }

    /// The block against a vector of pages, through pushes, removes, moves
    /// and growth, on the heap and reserved: every value where the model
    /// says, with its tick, each dropped once.
    #[test]
    fn a_block_holds_what_a_vector_of_pages_would() {
        let steps = if cfg!(miri) { 300 } else { 20_000 };
        for mem in [Mem::Heap, Mem::Reserved(64)] {
            {
                let rows = 4;
                let mut b = Block::new(Ty::of::<Canary>(), rows, mem);
                let mut model: Vec<Vec<(u64, u32)>> = Vec::new();
                let mut rng = Rng(0x1234_5678 ^ steps as u64);
                let mut next = 0u64;
                for step in 0..steps as u32 {
                    match rng.below(5) {
                        0 if model.len() < 40 => {
                            b.add_page();
                            model.push(Vec::new());
                        }
                        0 | 1 => {
                            let Some(p) = (0..model.len()).find(|&p| model[(p + step as usize) % model.len()].len() < rows) else {
                                continue;
                            };
                            let p = (p + step as usize) % model.len();
                            b.push(p, Canary::new(next), step);
                            model[p].push((next, step));
                            next += 1;
                        }
                        2 => {
                            let Some(p) = (0..model.len()).map(|k| (k + step as usize) % model.len()).find(|&p| !model[p].is_empty())
                            else {
                                continue;
                            };
                            let r = rng.below(model[p].len());
                            b.swap_remove_drop(p, r);
                            model[p].swap_remove(r);
                        }
                        _ => {
                            if model.len() < 2 {
                                continue;
                            }
                            let (p, q) = (rng.below(model.len()), rng.below(model.len()));
                            if p == q || model[p].is_empty() || model[q].len() == rows {
                                continue;
                            }
                            let r = rng.below(model[p].len());
                            b.move_row((p, r), q);
                            let v = model[p].swap_remove(r);
                            model[q].push(v);
                        }
                    }
                    let want: Vec<Vec<u64>> = model.iter().map(|p| p.iter().map(|v| v.0).collect()).collect();
                    assert_eq!(ids(&b), want, "values, step {step}, {mem:?}");
                    let want: Vec<Vec<u32>> = model.iter().map(|p| p.iter().map(|v| v.1).collect()).collect();
                    assert_eq!(ticks(&b), want, "ticks, step {step}, {mem:?}");
                    let live: usize = model.iter().map(Vec::len).sum();
                    assert_eq!(LIVE.with(Cell::get), live as i64, "every value alive once, step {step}");
                }
            }
            assert_eq!(LIVE.with(Cell::get), 0, "dropping the block dropped every value once");
        }
    }

    /// Moves and copies between blocks (pages kept one block each) leave
    /// every value in one place, with its tick, dropped once.
    #[test]
    fn moves_across_blocks_keep_each_value_once() {
        {
            let (mut a, mut b) = (Block::new(Ty::of::<Canary>(), 4, Mem::Heap), Block::new(Ty::of::<Canary>(), 4, Mem::Heap));
            a.add_page();
            b.add_page();
            for i in 0..4 {
                a.push(0, Canary::new(i), i as u32 + 10);
            }
            a.move_to((0, 1), &mut b, 0);
            a.move_to((0, 2), &mut b, 0);
            assert_eq!(ids(&a), vec![vec![0, 3]]);
            assert_eq!(ids(&b), vec![vec![1, 2]]);
            assert_eq!(ticks(&a), vec![vec![10, 13]]);
            assert_eq!(ticks(&b), vec![vec![11, 12]]);
            let mut c = Block::new(Ty::of::<Canary>(), 4, Mem::Heap);
            c.add_page();
            // SAFETY: each value copied once, then its block forgets it.
            unsafe {
                b.copy_to((0, 1), &mut c, 0);
                a.copy_to((0, 0), &mut c, 0);
                b.copy_to((0, 0), &mut c, 0);
                a.copy_to((0, 1), &mut c, 0);
            }
            a.forget();
            b.forget();
            assert_eq!(ids(&c), vec![vec![2, 0, 1, 3]]);
            assert_eq!(ticks(&c), vec![vec![12, 10, 11, 13]]);
            assert_eq!(LIVE.with(Cell::get), 4);
        }
        assert_eq!(LIVE.with(Cell::get), 0);
    }

    /// A reserved block never moves as it grows; one on the heap may.
    #[test]
    fn a_reserved_block_never_moves() {
        let mut b = Block::new(Ty::of::<u64>(), 16, Mem::Reserved(1 << 10));
        b.add_page();
        b.push(0, 7u64, 1);
        let at = b.page::<u64>(0).as_ptr();
        for p in 1..(1 << 10) {
            b.add_page();
            b.push(p, p as u64, 1);
        }
        assert_eq!(b.page::<u64>(0).as_ptr(), at, "page 0 where it was");
        assert_eq!(b.page::<u64>(0), &[7]);
        assert_eq!(b.page::<u64>(1023), &[1023]);
    }

    /// Every page's rows at once, each its own `&mut`, written through all
    /// of them interleaved: disjoint, as the split makes them.
    #[test]
    fn pages_borrowed_at_once_are_disjoint() {
        let mut b = Block::new(Ty::of::<u32>(), 4, Mem::Heap);
        for p in 0..5 {
            b.add_page();
            for r in 0..(p % 4 + 1) {
                b.push(p, (p * 10 + r) as u32, 0);
            }
        }
        let mut pages = b.pages_mut::<u32>();
        for r in 0..4 {
            for page in pages.iter_mut() {
                if let Some(x) = page.get_mut(r) {
                    *x += 1000;
                }
            }
        }
        drop(pages);
        let got: Vec<Vec<u32>> = (0..5).map(|p| b.page::<u32>(p).to_vec()).collect();
        let want: Vec<Vec<u32>> = (0..5).map(|p| (0..(p % 4 + 1)).map(|r| (p * 10 + r) as u32 + 1000).collect()).collect();
        assert_eq!(got, want);
    }

    /// A re-sort moves every value once into the new order; a migration
    /// rewrites each and drops the old.
    #[test]
    fn gather_and_migrate_move_each_value_once() {
        {
            let mut b = Block::new(Ty::of::<Canary>(), 3, Mem::Heap);
            for p in 0..4 {
                b.add_page();
                for r in 0..(p % 3 + 1) {
                    b.push(p, Canary::new((p * 10 + r) as u64), (p * 10 + r) as u32);
                }
            }
            let mut order: Vec<(u32, u32)> = (0..4u32).flat_map(|p| (0..(p % 3 + 1)).map(move |r| (p, r))).collect();
            order.reverse();
            let g = b.gather(&order);
            let want: Vec<u64> = order.iter().map(|&(p, r)| (p * 10 + r) as u64).collect();
            assert_eq!(ids(&g).concat(), want);
            assert_eq!(ticks(&g).concat(), want.iter().map(|&x| x as u32).collect::<Vec<_>>());
            assert_eq!(ids(&b).concat(), Vec::<u64>::new(), "the old block owns nothing");
            assert_eq!(LIVE.with(Cell::get), want.len() as i64);
            drop((b, g));
            assert_eq!(LIVE.with(Cell::get), 0);
        }
        {
            let mut b = Block::new(Ty::of::<Canary>(), 3, Mem::Heap);
            b.add_page();
            b.push(0, Canary::new(5), 9);
            b.push(0, Canary::new(6), 9);
            b.migrate::<Canary, (u64, u64)>(|c| (c.id(), c.id() * 2));
            assert_eq!(LIVE.with(Cell::get), 0, "the old values dropped");
            assert_eq!(b.page::<(u64, u64)>(0), &[(5, 10), (6, 12)]);
            assert_eq!(ticks(&b), vec![vec![9, 9]], "the rows keep their ticks");
        }
    }
}
