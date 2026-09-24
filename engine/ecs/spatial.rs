//! Spatial tables: a table holding a spatial key component (a position)
//! keeps its rows in pages by Z-order (Morton) of their cells, so a page is
//! a neighborhood with a box around it, and region queries and the
//! broadphase walk only the pages near what they ask about. See
//! docs/architecture/spatial-storage.md.
//!
//! The order is kept by `Structural`: anything that moves a row into a
//! spatial table, or writes its key or extent, marks the table, and the
//! table is re-sorted when the `Structural` is dropped, at the end of an
//! apply node or a between-frames change. A system never sees its own rows
//! move; the systems after it do.

use crate::component::{Component, Entity};
use crate::erased::ErasedColumn;
use crate::world::{Entities, Location, TableId};

/// Rows per page in a spatial table: a page is a neighborhood, so small
/// (the spike measured 8 to 16 as best; see the doc).
pub const SPATIAL_PAGE_ROWS: usize = 16;
/// Pages per run: the level above pages. Pages are in Z-order, so a run of
/// consecutive pages is a neighborhood too.
pub(crate) const RUN: usize = 16;

/// An axis-aligned box: what a spatial key's bounds are.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f32; 2],
    pub max: [f32; 2],
}

impl Bounds {
    pub const EMPTY: Bounds = Bounds { min: [f32::INFINITY; 2], max: [f32::NEG_INFINITY; 2] };

    pub fn new(min: [f32; 2], max: [f32; 2]) -> Bounds {
        Bounds { min, max }
    }

    /// The box `half` around `center`.
    pub fn around(center: [f32; 2], half: [f32; 2]) -> Bounds {
        Bounds { min: [center[0] - half[0], center[1] - half[1]], max: [center[0] + half[0], center[1] + half[1]] }
    }

    /// Touching counts.
    pub fn overlaps(&self, o: &Bounds) -> bool {
        self.min[0] <= o.max[0] && o.min[0] <= self.max[0] && self.min[1] <= o.max[1] && o.min[1] <= self.max[1]
    }

    pub fn union(&self, o: &Bounds) -> Bounds {
        Bounds {
            min: [self.min[0].min(o.min[0]), self.min[1].min(o.min[1])],
            max: [self.max[0].max(o.max[0]), self.max[1].max(o.max[1])],
        }
    }

    pub fn grown(&self, by: f32) -> Bounds {
        Bounds { min: [self.min[0] - by, self.min[1] - by], max: [self.max[0] + by, self.max[1] + by] }
    }

    fn center(&self) -> [f32; 2] {
        [(self.min[0] + self.max[0]) / 2.0, (self.min[1] + self.max[1]) / 2.0]
    }

    /// The larger half extent.
    fn reach(&self) -> f32 {
        ((self.max[0] - self.min[0]) / 2.0).max((self.max[1] - self.min[1]) / 2.0)
    }
}

/// A component whose tables are kept in spatial order: declared with
/// `order = spatial` in `component!`. `bounds` is the box around an entity
/// with this key, and its `Extent` if it has one (a position's collider);
/// a key with no extent of its own names itself as its extent.
pub trait SpatialKey: Component {
    type Extent: Component;
    /// Cells of the Z-order: about the size of the smallest things, so
    /// neighbors in the order are neighbors in space.
    const CELL: f32 = 1.0;
    /// Bigger than this (the larger half extent), a row is kept out of the
    /// order in pages of its own: a floor across the level would otherwise
    /// stretch its page's box over everything.
    const BIG: f32 = 2.0;
    fn bounds(&self, extent: Option<&Self::Extent>) -> Bounds;
}

/// The bounds glue: reads a key and, if the row has it, an extent, as the
/// types the declaring build compiled.
pub type BoundsFn = unsafe fn(key: *const u8, extent: *const u8) -> Bounds;

/// What a spatial key declares, carried in its `ComponentDesc`.
#[derive(Clone, Copy)]
pub struct SpatialDesc {
    pub extent: &'static str,
    /// The extent's layout this glue reads: a row whose extent is installed
    /// with another is bounded without it.
    pub extent_fingerprint: u64,
    pub bounds: BoundsFn,
    pub cell: f32,
    pub big: f32,
}

impl SpatialDesc {
    pub const fn of<T: SpatialKey>() -> SpatialDesc {
        SpatialDesc {
            extent: <T::Extent as Component>::NAME,
            extent_fingerprint: <T::Extent as Component>::FINGERPRINT,
            bounds: __bounds::<T>,
            cell: T::CELL,
            big: T::BIG,
        }
    }
}

#[doc(hidden)]
pub unsafe fn __bounds<T: SpatialKey>(key: *const u8, extent: *const u8) -> Bounds {
    // SAFETY: the caller passes a value of the installed layout of `T`, the
    // key, which is this build's (the glue is installed with the layout),
    // and an extent only if it's installed with the layout this build read.
    let key = unsafe { &*(key as *const T) };
    let extent = (!extent.is_null()).then(|| unsafe { &*(extent as *const T::Extent) });
    key.bounds(extent)
}

fn contains(outer: &Bounds, inner: &Bounds) -> bool {
    outer.min[0] <= inner.min[0] && outer.min[1] <= inner.min[1] && outer.max[0] >= inner.max[0] && outer.max[1] >= inner.max[1]
}

fn spread(v: u32) -> u64 {
    let mut x = v as u64;
    x = (x | (x << 16)) & 0x0000_FFFF_0000_FFFF;
    x = (x | (x << 8)) & 0x00FF_00FF_00FF_00FF;
    x = (x | (x << 4)) & 0x0F0F_0F0F_0F0F_0F0F;
    x = (x | (x << 2)) & 0x3333_3333_3333_3333;
    (x | (x << 1)) & 0x5555_5555_5555_5555
}

/// The Z-order key of a box's cell. Offset so negative cells sort too.
fn morton(b: &Bounds, cell: f32) -> u64 {
    let c = |v: f32| ((v / cell).floor() as i64 + (1 << 20)).clamp(0, u32::MAX as i64) as u32;
    let [x, y] = b.center();
    spread(c(x)) | (spread(c(y)) << 1)
}

/// What a page holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PageKind {
    /// Rows whose keys are from this page's `lo` up to the next page's in
    /// the order.
    Ordered,
    /// Rows bigger than `BIG`, in no order.
    Big,
    /// Rows just placed, before the table is next re-sorted.
    Staging,
}

/// A spatial table's order: per physical page (the table's own pages,
/// whose indices entity locations hold, so they never change), its kind,
/// range and box, and each row's box and key, parallel to the page's rows.
pub struct SpatialPages {
    pub kind: Vec<PageKind>,
    /// The lowest key of an ordered page.
    lo: Vec<u64>,
    /// Ordered pages, by `lo`.
    pub order: Vec<u32>,
    pub bounds: Vec<Bounds>,
    pub row_bounds: Vec<Vec<Bounds>>,
    keys: Vec<Vec<u64>>,
    /// Boxes over runs of `RUN` pages of `order`.
    pub runs: Vec<Bounds>,
    /// Empty staging pages, for splits to reuse: rebuilt at each re-sort,
    /// since new rows land on whichever page is last, freed or not.
    free: Vec<u32>,
    pub dirty: bool,
}

impl Default for SpatialPages {
    fn default() -> SpatialPages {
        // One ordered page for every key to start.
        SpatialPages {
            kind: vec![PageKind::Ordered],
            lo: vec![0],
            order: vec![0],
            bounds: vec![Bounds::EMPTY],
            row_bounds: vec![Vec::new()],
            keys: vec![Vec::new()],
            runs: vec![Bounds::EMPTY],
            free: Vec::new(),
            dirty: false,
        }
    }
}

impl SpatialPages {
    /// A new physical page, for rows placed before the next re-sort.
    pub(crate) fn add_page(&mut self) {
        self.kind.push(PageKind::Staging);
        self.lo.push(u64::MAX);
        self.bounds.push(Bounds::EMPTY);
        self.row_bounds.push(Vec::new());
        self.keys.push(Vec::new());
    }

    /// A row pushed onto `page`: placeholders until the re-sort.
    pub(crate) fn push_row(&mut self, page: usize) {
        self.row_bounds[page].push(Bounds::EMPTY);
        self.keys[page].push(u64::MAX);
        self.dirty = true;
    }

    /// A row swap-removed from `page`, as the table's own rows are.
    pub(crate) fn swap_remove(&mut self, page: usize, row: usize) {
        self.row_bounds[page].swap_remove(row);
        self.keys[page].swap_remove(row);
    }

    /// The ordered page a key belongs to: the last whose `lo` is at most it.
    fn page_for(&self, key: u64) -> usize {
        let i = self.order.partition_point(|&p| self.lo[p as usize] <= key).max(1) - 1;
        self.order[i] as usize
    }

    /// The pages whose boxes meet `region`, runs first, then big pages.
    pub fn pages_near(&self, region: &Bounds) -> Vec<usize> {
        let mut out = Vec::new();
        for (r, b) in self.runs.iter().enumerate() {
            if !b.overlaps(region) {
                continue;
            }
            let run = &self.order[r * RUN..((r + 1) * RUN).min(self.order.len())];
            out.extend(run.iter().map(|&p| p as usize).filter(|&p| self.bounds[p].overlaps(region)));
        }
        out.extend((0..self.kind.len()).filter(|&p| self.kind[p] != PageKind::Ordered && self.bounds[p].overlaps(region)));
        out
    }

    /// Checks the order against `rows`, the table's pages: for tests. Every
    /// ordered page's rows have keys in its range, big pages hold only big
    /// rows, nothing is left staged, and page and run boxes hold their rows.
    #[doc(hidden)]
    pub fn check(&self, rows: &[Vec<Entity>], big: f32) -> Result<(), String> {
        if rows.len() != self.kind.len() || self.row_bounds.iter().map(Vec::len).ne(rows.iter().map(Vec::len)) {
            return Err("the order's pages don't match the table's".into());
        }
        if let Some(p) = rows.iter().position(|r| r.len() > SPATIAL_PAGE_ROWS) {
            return Err(format!("page {p} holds {} rows", rows[p].len()));
        }
        if self.order.windows(2).any(|w| self.lo[w[0] as usize] > self.lo[w[1] as usize]) {
            return Err("the ordered pages aren't in order".into());
        }
        for (i, &p) in self.order.iter().enumerate() {
            let p = p as usize;
            let hi = self.order.get(i + 1).map_or(u64::MAX, |&n| self.lo[n as usize]);
            for (r, &k) in self.keys[p].iter().enumerate() {
                if k < self.lo[p] || (k > hi || (k == hi && hi != self.lo[p])) {
                    return Err(format!("{:?} on page {p} has key {k}, outside {}..{hi}", rows[p][r], self.lo[p]));
                }
                if self.row_bounds[p][r].reach() > big {
                    return Err(format!("{:?} is big, and on ordered page {p}", rows[p][r]));
                }
            }
        }
        for p in 0..self.kind.len() {
            if self.kind[p] == PageKind::Staging && !rows[p].is_empty() {
                return Err(format!("page {p} still has staged rows"));
            }
            if self.kind[p] == PageKind::Big && self.row_bounds[p].iter().any(|b| b.reach() <= big) {
                return Err(format!("big page {p} holds a row that isn't"));
            }
            for b in &self.row_bounds[p] {
                if !contains(&self.bounds[p], b) {
                    return Err(format!("page {p}'s box doesn't hold its row {b:?}"));
                }
            }
        }
        for (r, run) in self.order.chunks(RUN).enumerate() {
            if run.iter().any(|&p| !contains(&self.runs[r], &self.bounds[p as usize]) && !self.row_bounds[p as usize].is_empty()) {
                return Err(format!("run {r}'s box doesn't hold its pages"));
            }
        }
        Ok(())
    }

    /// Row `r` of page `p`'s box, as of the last re-sort.
    pub fn row_bounds(&self, p: usize, r: usize) -> Bounds {
        self.row_bounds[p][r]
    }

    /// Every row's box, by page.
    pub fn row_bounds_all(&self) -> &[Vec<Bounds>] {
        &self.row_bounds
    }

    fn rebound(&mut self) {
        for (p, rows) in self.row_bounds.iter().enumerate() {
            self.bounds[p] = rows.iter().fold(Bounds::EMPTY, |b, r| b.union(r));
        }
        self.runs = self.order.chunks(RUN).map(|run| run.iter().fold(Bounds::EMPTY, |b, &p| b.union(&self.bounds[p as usize]))).collect();
    }
}

/// A spatial table's pages, locked for re-sorting: its rows and columns,
/// and what to call the bounds glue with.
pub(crate) struct Resort<'a> {
    pub table: TableId,
    pub rows: &'a mut Vec<Vec<Entity>>,
    pub columns: Vec<&'a mut Vec<ErasedColumn>>,
    pub pages: &'a mut SpatialPages,
    pub key: usize,
    /// The extent's column, if the table has it and it's installed with the
    /// layout the glue reads.
    pub extent: Option<usize>,
    pub desc: SpatialDesc,
    pub entities: &'a Entities,
}

impl Resort<'_> {
    /// Bounds every row, then moves each row that isn't in the page its key
    /// (or its size) says, splitting full pages. Returns rows moved.
    pub fn run(mut self) -> usize {
        for p in 0..self.rows.len() {
            for r in 0..self.rows[p].len() {
                let key = self.columns[self.key][p].value_ptr(r);
                let extent = self.extent.map_or(std::ptr::null(), |x| self.columns[x][p].value_ptr(r));
                // SAFETY: `key` is a value of the key's installed layout,
                // whose build the glue came from, and `extent` is only passed
                // when installed with the layout the glue reads.
                let b = unsafe { (self.desc.bounds)(key, extent) };
                self.pages.row_bounds[p][r] = b;
                self.pages.keys[p][r] = morton(&b, self.desc.cell);
            }
        }
        self.pages.free = (0..self.rows.len())
            .filter(|&p| self.pages.kind[p] == PageKind::Staging && self.rows[p].is_empty())
            .map(|p| p as u32)
            .collect();
        let mut moved = 0;
        // Pages a split makes are walked too, since they may take rows
        // that haven't been placed yet.
        let mut p = 0;
        while p < self.rows.len() {
            let mut r = 0;
            while r < self.rows[p].len() {
                let target = self.target(p, r);
                if target == p {
                    r += 1;
                    continue;
                }
                // Swap-removed: the row now at `r` is another, checked next.
                self.move_row(p, r, target);
                moved += 1;
            }
            p += 1;
        }
        self.merge();
        self.pages.rebound();
        self.pages.dirty = false;
        moved
    }

    /// Where row `r` of page `p` belongs, making room there if it's full.
    fn target(&mut self, p: usize, r: usize) -> usize {
        let b = self.pages.row_bounds[p][r];
        if b.reach() > self.desc.big {
            if self.pages.kind[p] == PageKind::Big {
                return p;
            }
            let big = (0..self.rows.len()).find(|&q| self.pages.kind[q] == PageKind::Big && self.rows[q].len() < SPATIAL_PAGE_ROWS);
            return big.unwrap_or_else(|| self.new_page(PageKind::Big, u64::MAX));
        }
        let key = self.pages.keys[p][r];
        let q = self.pages.page_for(key);
        if q == p || self.rows[q].len() < SPATIAL_PAGE_ROWS {
            return q;
        }
        self.split(q, key)
    }

    /// Folds each ordered page into the one before it where both fit in
    /// one page, so splitting (which halves) and motion (which empties)
    /// don't leave the table in pages of a row or two. The earlier page
    /// takes over the later's range; the later is freed for reuse.
    fn merge(&mut self) {
        let mut i = 0;
        while i + 1 < self.pages.order.len() {
            let (a, b) = (self.pages.order[i] as usize, self.pages.order[i + 1] as usize);
            if self.rows[a].len() + self.rows[b].len() > SPATIAL_PAGE_ROWS {
                i += 1;
                continue;
            }
            while !self.rows[b].is_empty() {
                self.move_row(b, 0, a);
            }
            self.pages.order.remove(i + 1);
            self.pages.kind[b] = PageKind::Staging;
            self.pages.lo[b] = u64::MAX;
            self.pages.free.push(b as u32);
        }
    }

    fn new_page(&mut self, kind: PageKind, lo: u64) -> usize {
        if let Some(p) = self.pages.free.pop() {
            let p = p as usize;
            assert!(self.rows[p].is_empty(), "a free page is empty");
            self.pages.kind[p] = kind;
            self.pages.lo[p] = lo;
            return p;
        }
        let world_page = self.rows.len();
        self.rows.push(Vec::new());
        for c in self.columns.iter_mut() {
            let ty = c[0].value_type();
            c.push(ErasedColumn::new(ty));
        }
        self.pages.add_page();
        self.pages.kind[world_page] = kind;
        self.pages.lo[world_page] = lo;
        world_page
    }

    /// Splits full ordered page `q` at its median key, and returns the half
    /// `key` belongs in. If every row has one key, `k`, `q` narrows to
    /// exactly `k` and new pages take what's around it: a page below for
    /// keys under `k`, one above for keys over it, or another page of `k`
    /// (pages may share a `lo`; the last takes new rows).
    fn split(&mut self, q: usize, key: u64) -> usize {
        let at = self.pages.order.iter().position(|&p| p as usize == q).expect("an ordered page");
        // Only the rows already in `q`'s range: the others haven't been
        // placed yet, and will leave for their own pages when walked.
        let (lo, hi) = (self.pages.lo[q], self.pages.order.get(at + 1).map_or(u64::MAX, |&n| self.pages.lo[n as usize]));
        let in_range = |k: u64| k >= lo && (k < hi || (k == hi && hi == lo));
        let mut keys: Vec<u64> = self.pages.keys[q].iter().copied().filter(|&k| in_range(k)).collect();
        if keys.is_empty() {
            // Full of rows that are leaving: another page for this range.
            let fresh = self.new_page(PageKind::Ordered, key);
            self.pages.order.insert(at + 1, fresh as u32);
            return fresh;
        }
        keys.sort_unstable();
        let mid = keys[keys.len() / 2];
        if mid == keys[0] {
            let k = mid;
            if key > k {
                let above = self.new_page(PageKind::Ordered, key);
                self.pages.order.insert(at + 1, above as u32);
                return above;
            }
            let mut at = at;
            if self.pages.lo[q] < k {
                let below = self.new_page(PageKind::Ordered, self.pages.lo[q]);
                self.pages.order.insert(at, below as u32);
                self.pages.lo[q] = k;
                at += 1;
                if key < k {
                    return below;
                }
            }
            let again = self.new_page(PageKind::Ordered, k);
            self.pages.order.insert(at + 1, again as u32);
            return again;
        }
        let upper = self.new_page(PageKind::Ordered, mid);
        self.pages.order.insert(at + 1, upper as u32);
        let mut r = 0;
        while r < self.rows[q].len() {
            let k = self.pages.keys[q][r];
            if in_range(k) && k >= mid {
                self.move_row(q, r, upper);
            } else {
                r += 1;
            }
        }
        if key >= mid { upper } else { q }
    }

    /// Moves row `r` of page `p` to the end of page `q`, in every column.
    fn move_row(&mut self, p: usize, r: usize, q: usize) {
        for c in self.columns.iter_mut() {
            let [from, to] = c.get_disjoint_mut([p, q]).expect("two pages");
            from.swap_remove_into(r, to);
        }
        let e = self.rows[p].swap_remove(r);
        self.rows[q].push(e);
        let b = self.pages.row_bounds[p].swap_remove(r);
        let k = self.pages.keys[p].swap_remove(r);
        self.pages.row_bounds[q].push(b);
        self.pages.keys[q].push(k);
        let place = |e: Entity, page: usize, row: usize| {
            self.entities.place(e, Location { table: self.table, page: page as u32, row: row as u32 })
        };
        place(e, q, self.rows[q].len() - 1);
        if let Some(&swapped) = self.rows[p].get(r) {
            place(swapped, p, r);
        }
    }
}
