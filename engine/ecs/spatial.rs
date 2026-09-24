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

/// The bounds glue: reads one page's keys and, if the rows have them,
/// extents, as the types the declaring build compiled, and writes the box of
/// each row in `rows` to `out` (parallel to the page). A page at a time, not
/// a row: the call can't be inlined, and one per row, with the caller's
/// state saved around each, measured a third of a re-sort (2026-09-24).
pub type BoundsFn = unsafe fn(keys: *const u8, extents: *const u8, rows: &[u32], out: &mut [Bounds]);

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
pub unsafe fn __bounds<T: SpatialKey>(keys: *const u8, extents: *const u8, rows: &[u32], out: &mut [Bounds]) {
    for &r in rows {
        let r = r as usize;
        // Indexed first, so a row past the page panics before it's read.
        let slot = &mut out[r];
        // SAFETY: the caller passes `out.len()` values of the installed
        // layout of `T`, the key, which is this build's (the glue is
        // installed with the layout), and as many extents only if they're
        // installed with the layout this build read; `r` is within them.
        let key = unsafe { &*(keys as *const T).add(r) };
        let extent = (!extents.is_null()).then(|| unsafe { &*(extents as *const T::Extent).add(r) });
        *slot = key.bounds(extent);
    }
}

/// Whether an ordered page's range, from `lo` up to `hi` (the next page's
/// `lo`), holds `key`. Pages may share a `lo` (a run of one key too many
/// for a page); all but the last of them hold exactly that key.
fn holds(lo: u64, hi: u64, key: u64) -> bool {
    key >= lo && (key < hi || (key == hi && hi == lo))
}

fn contains(outer: &Bounds, inner: &Bounds) -> bool {
    outer.min[0] <= inner.min[0] && outer.min[1] <= inner.min[1] && outer.max[0] >= inner.max[0] && outer.max[1] >= inner.max[1]
}

/// The cell `v` is in, along one axis, offset by 2^20 cells so negative
/// cells sort too, and saturating at the ends. Offset before truncating,
/// so truncation is the floor, without `floor`, which is a function call
/// here (docs/lore/f32-floor-is-a-function-call-on-the-default-target.md).
/// In `f64`, where a coordinate times the cells per unit is exact, and so
/// is the offset but within 2^-32 of a cell's edge.
fn cell(v: f32, per_cell: f32) -> u32 {
    (f64::from(v) * f64::from(per_cell) + (1u32 << 20) as f64) as u32
}

fn spread(v: u32) -> u64 {
    let mut x = v as u64;
    x = (x | (x << 16)) & 0x0000_FFFF_0000_FFFF;
    x = (x | (x << 8)) & 0x00FF_00FF_00FF_00FF;
    x = (x | (x << 4)) & 0x0F0F_0F0F_0F0F_0F0F;
    x = (x | (x << 2)) & 0x3333_3333_3333_3333;
    (x | (x << 1)) & 0x5555_5555_5555_5555
}

/// The cell of a box's center, both axes packed, given cells per unit:
/// multiplying by it measured a third cheaper than dividing by the cell,
/// and a key only has to be the same for the same box within one table.
fn cells(b: &Bounds, per_cell: f32) -> u64 {
    let [x, y] = b.center();
    cell(x, per_cell) as u64 | (cell(y, per_cell) as u64) << 32
}

/// The Z-order key of packed cells.
fn morton(cells: u64) -> u64 {
    spread(cells as u32) | (spread((cells >> 32) as u32) << 1)
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
    /// Each row's cell on each axis, packed, parallel to `keys`: a row that
    /// moved within its cell keeps its key, and interleaving the bits for
    /// the key was most of re-keying a creeping row.
    cells: Vec<Vec<u64>>,
    /// Each ordered page's upper key (the next page's `lo`), by physical
    /// page, as of the start of a re-sort: so a row can be checked against
    /// its own page's range without a search. Not rebuilt as splits narrow
    /// ranges, since a split moves out every row the narrowing excludes,
    /// and the pages it makes aren't walked.
    hi: Vec<u64>,
    /// Pages a re-sort must walk for rows to move: those with a row that
    /// isn't where its key says. Kept here only to reuse the allocation.
    unplaced: Vec<bool>,
    /// Pages whose box may no longer be the union of their rows': a row
    /// arrived, left, or moved since it was last computed.
    stale: Vec<bool>,
    /// Boxes over runs of `RUN` pages of `order`.
    pub runs: Vec<Bounds>,
    /// Empty staging pages, for splits to reuse: rebuilt at each re-sort,
    /// since new rows land on whichever page is last, freed or not.
    free: Vec<u32>,
    pub dirty: bool,
    /// The world tick the order was last sorted at: rows whose key and
    /// extent haven't been written since keep their boxes.
    sorted_tick: u32,
    /// Rows whose box was recomputed at the last re-sort: for tests.
    pub rebounded: usize,
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
            cells: vec![Vec::new()],
            hi: vec![u64::MAX],
            unplaced: Vec::new(),
            stale: vec![false],
            runs: vec![Bounds::EMPTY],
            free: Vec::new(),
            dirty: false,
            sorted_tick: 0,
            rebounded: 0,
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
        self.cells.push(Vec::new());
        self.stale.push(false);
    }

    /// A row pushed onto `page`: placeholders until the re-sort.
    pub(crate) fn push_row(&mut self, page: usize) {
        self.row_bounds[page].push(Bounds::EMPTY);
        self.keys[page].push(u64::MAX);
        self.cells[page].push(u64::MAX);
        self.stale[page] = true;
        self.dirty = true;
    }

    /// A row swap-removed from `page`, as the table's own rows are.
    pub(crate) fn swap_remove(&mut self, page: usize, row: usize) {
        self.row_bounds[page].swap_remove(row);
        self.keys[page].swap_remove(row);
        self.cells[page].swap_remove(row);
        self.stale[page] = true;
    }

    fn rebuild_hi(&mut self) {
        self.hi.resize(self.kind.len(), u64::MAX);
        for (i, &p) in self.order.iter().enumerate() {
            self.hi[p as usize] = self.order.get(i + 1).map_or(u64::MAX, |&n| self.lo[n as usize]);
        }
    }

    fn holds(&self, p: usize, key: u64) -> bool {
        holds(self.lo[p], self.hi[p], key)
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
    /// row's key is its box's, with cells of `cell`; every ordered page's
    /// rows have keys in its range, big pages hold only big rows, nothing is
    /// left staged, page boxes hold their rows (exactly, unless stale), and
    /// run boxes hold them.
    #[doc(hidden)]
    pub fn check(&self, rows: &[Vec<Entity>], big: f32, cell: f32) -> Result<(), String> {
        if rows.len() != self.kind.len() || self.row_bounds.iter().map(Vec::len).ne(rows.iter().map(Vec::len)) {
            return Err("the order's pages don't match the table's".into());
        }
        if let Some(p) = rows.iter().position(|r| r.len() > SPATIAL_PAGE_ROWS) {
            return Err(format!("page {p} holds {} rows", rows[p].len()));
        }
        for p in 0..rows.len() {
            for (r, b) in self.row_bounds[p].iter().enumerate() {
                let c = cells(b, 1.0 / cell);
                if (self.cells[p][r], self.keys[p][r]) != (c, morton(c)) {
                    return Err(format!("{:?} on page {p} is keyed as {:x}, and its box's key is {:x}", rows[p][r], self.keys[p][r], morton(c)));
                }
            }
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
            // Rows leaving between re-sorts leave a box loose, marked stale.
            let tight = self.row_bounds[p].iter().fold(Bounds::EMPTY, |b, r| b.union(r));
            if !self.stale[p] && self.bounds[p] != tight {
                return Err(format!("page {p}'s box is {:?}, and its rows' is {tight:?}", self.bounds[p]));
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

    /// Recomputes the boxes of stale pages, and, if any page's box may have
    /// changed, every run's: runs are few (a sixteenth of pages), and splits
    /// and merges reshape them.
    fn rebound(&mut self, runs: bool) {
        for (p, rows) in self.row_bounds.iter().enumerate() {
            if std::mem::take(&mut self.stale[p]) {
                self.bounds[p] = rows.iter().fold(Bounds::EMPTY, |b, r| b.union(r));
            }
        }
        if !runs {
            return;
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
    /// The world's tick as the re-sort starts: every write so far.
    pub now: u32,
}

impl Resort<'_> {
    /// Bounds every row, then moves each row that isn't in the page its key
    /// (or its size) says, splitting full pages. Returns rows moved.
    pub fn run(mut self) -> usize {
        let since = self.pages.sorted_tick;
        let mut rebounded = 0;
        // Rows arrived or left since the last sort: pages to merge.
        let arrived_or_left = self.pages.stale.contains(&true);
        self.pages.rebuild_hi();
        self.pages.unplaced.clear();
        self.pages.unplaced.resize(self.rows.len(), false);
        let (big, per_cell) = (self.desc.big, 1.0 / self.desc.cell);
        let mut written = Vec::with_capacity(SPATIAL_PAGE_ROWS);
        for p in 0..self.rows.len() {
            let key = &self.columns[self.key][p];
            let extent = self.extent.map(|x| &self.columns[x][p]);
            let (key_ticks, extent_ticks) = (key.ticks(), extent.map(ErasedColumn::ticks));
            // Only rows new to the table (a placeholder key) or whose key or
            // extent was written since the last sort: the rest keep their
            // boxes and keys.
            written.clear();
            let keys = &self.pages.keys[p];
            written.extend((0..keys.len() as u32).filter(|&r| {
                let r = r as usize;
                keys[r] == u64::MAX || key_ticks[r] > since || extent_ticks.is_some_and(|t| t[r] > since)
            }));
            if written.is_empty() {
                continue;
            }
            rebounded += written.len();
            let row_bounds = &mut self.pages.row_bounds[p];
            assert!(key.len() == row_bounds.len() && extent.is_none_or(|c| c.len() == row_bounds.len()), "a page's order is its rows'");
            // SAFETY: the page's keys, of the key's installed layout, whose
            // build the glue came from, and its extents only when installed
            // with the layout the glue reads, as many as `row_bounds` holds
            // (checked), which `written` indexes.
            unsafe { (self.desc.bounds)(key.value_ptr(0), extent.map_or(std::ptr::null(), |c| c.value_ptr(0)), &written, row_bounds) };
            let (keys, row_cells) = (&mut self.pages.keys[p], &mut self.pages.cells[p]);
            let (kind, lo, hi) = (self.pages.kind[p], self.pages.lo[p], self.pages.hi[p]);
            let mut unplaced = false;
            for &r in &written {
                let (b, r) = (row_bounds[r as usize], r as usize);
                let c = cells(&b, per_cell);
                // A new row's placeholders are consistent too: the cells
                // `u64::MAX` are keyed `u64::MAX`.
                if c != row_cells[r] {
                    (row_cells[r], keys[r]) = (c, morton(c));
                }
                let k = keys[r];
                // Rows that weren't re-bounded are where the last sort put
                // them, and no page's range has changed since: only these
                // can need moving.
                unplaced |= !match kind {
                    PageKind::Ordered => b.reach() <= big && holds(lo, hi, k),
                    PageKind::Big => b.reach() > big,
                    PageKind::Staging => false,
                };
            }
            self.pages.unplaced[p] = unplaced;
            // Boxed here, with its rows' boxes just written, rather than in
            // a second pass; a page no row of which was written keeps its
            // box. A move after marks it stale again.
            self.pages.bounds[p] = row_bounds.iter().fold(Bounds::EMPTY, |b, r| b.union(r));
            self.pages.stale[p] = false;
        }
        // Nothing to move or merge: a table at rest costs only the scan of
        // its ticks above.
        let reshape = arrived_or_left || self.pages.unplaced.contains(&true);
        let moved = if reshape { self.reshape() } else { 0 };
        self.pages.rebound(reshape || rebounded > 0);
        self.pages.dirty = false;
        self.pages.sorted_tick = self.now;
        self.pages.rebounded = rebounded;
        moved
    }

    /// Moves the rows that aren't where their keys say, then merges pages.
    /// Returns rows moved.
    fn reshape(&mut self) -> usize {
        self.pages.free = (0..self.rows.len())
            .filter(|&p| self.pages.kind[p] == PageKind::Staging && self.rows[p].is_empty())
            .map(|p| p as u32)
            .collect();
        let mut moved = 0;
        // Only the pages there were: rows go to a page a split makes only
        // where their keys say, so it has none to move.
        for p in 0..self.pages.unplaced.len() {
            if !self.pages.unplaced[p] {
                continue;
            }
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
        }
        self.merge();
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
        // Most rows stay where they are: a settled pile's all do, and
        // searching the order for each was most of a re-sort.
        if self.pages.kind[p] == PageKind::Ordered && self.pages.holds(p, key) {
            return p;
        }
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
            // A page of exactly one key, `k`, that shares its `lo` with the
            // next holds rows of `k`, and so can only fold into a page of
            // exactly `k` too: any other would take them as its upper
            // bound, which it doesn't hold.
            let lo = |p: usize| self.pages.lo[p];
            let hi_b = self.pages.order.get(i + 2).map_or(u64::MAX, |&n| lo(n as usize));
            let one_key = hi_b == lo(b) && lo(a) != lo(b) && !self.rows[b].is_empty();
            if one_key || self.rows[a].len() + self.rows[b].len() > SPATIAL_PAGE_ROWS * 3 / 4 {
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
    /// `key` belongs in. If the median is the least key, `k` (half the rows
    /// or more have it), the split is around `k` instead: a page above from
    /// `key` for a key over it, else `q` narrows to exactly `k` and new
    /// pages take what's around it, one below for keys under `k` and
    /// another page from `k` (pages may share a `lo`; the last takes new
    /// rows). Either way the rows of `q` the new page's range takes move
    /// to it, so every row the split doesn't move is still in its range.
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
                self.move_from(q, above, |k| in_range(k) && k >= key);
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
            self.move_from(q, again, |j| in_range(j) && j > k);
            return again;
        }
        let upper = self.new_page(PageKind::Ordered, mid);
        self.pages.order.insert(at + 1, upper as u32);
        self.move_from(q, upper, |k| in_range(k) && k >= mid);
        if key >= mid { upper } else { q }
    }

    /// Moves the rows of page `q` whose keys `takes` to page `to`.
    fn move_from(&mut self, q: usize, to: usize, takes: impl Fn(u64) -> bool) {
        let mut r = 0;
        while r < self.rows[q].len() {
            if takes(self.pages.keys[q][r]) {
                self.move_row(q, r, to);
            } else {
                r += 1;
            }
        }
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
        let c = self.pages.cells[p].swap_remove(r);
        self.pages.row_bounds[q].push(b);
        self.pages.keys[q].push(k);
        self.pages.cells[q].push(c);
        (self.pages.stale[p], self.pages.stale[q]) = (true, true);
        let place = |e: Entity, page: usize, row: usize| {
            self.entities.place(e, Location { table: self.table, page: page as u32, row: row as u32 })
        };
        place(e, q, self.rows[q].len() - 1);
        if let Some(&swapped) = self.rows[p].get(r) {
            place(swapped, p, r);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `cell` is `floor` offset, for cells a power of two wide, where it's
    /// exact: across zero, on and either side of cell edges.
    #[test]
    fn a_cell_is_the_floor_offset() {
        for size in [0.25f32, 0.5, 1.0, 2.0, 8.0] {
            for i in -2000..2000 {
                let edge = i as f32 * size / 4.0;
                for v in [edge, edge - 1e-3, edge + 1e-3, edge - size * 0.37] {
                    let floor = ((v / size).floor() as i64 + (1 << 20)) as u32;
                    assert_eq!(cell(v, 1.0 / size), floor, "{v} in cells of {size}");
                }
            }
        }
    }

    #[test]
    fn cells_saturate_at_the_ends() {
        assert_eq!(cell(f32::NEG_INFINITY, 1.0), 0);
        assert_eq!(cell(-2.0e6, 1.0), 0);
        assert_eq!(cell(f32::INFINITY, 1.0), u32::MAX);
        assert_eq!(cell(1.0e10, 1.0), u32::MAX);
        // The placeholder a new row is keyed with is these cells' key.
        assert_eq!(morton(u64::MAX), u64::MAX);
    }
}
