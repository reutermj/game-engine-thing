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

/// `v.floor() as i64`, exactly, without `floor`: on x86-64's baseline (no
/// SSE4.1) that's a call into libm, two for every row a re-sort re-bounds.
/// Truncation rounds toward zero, so a negative value with a fraction is
/// one more than its floor; an `f32` of 2^24 or more is whole, and converts
/// back exactly.
fn floor_i64(v: f32) -> i64 {
    let t = v as i64;
    t.saturating_sub((v < t as f32) as i64)
}

/// The Z-order key of a box's cell. Offset so negative cells sort too.
fn morton(b: &Bounds, cell: f32) -> u64 {
    let c = |v: f32| (floor_i64(v / cell) + (1 << 20)).clamp(0, u32::MAX as i64) as u32;
    let [x, y] = b.center();
    spread(c(x)) | (spread(c(y)) << 1)
}

/// A page's rows' boxes, by coordinate, with their entities' indices: the
/// order's one copy of each row's box, kept by coordinate so a box is
/// tested against every row of a page at once. `meeting` is a mask of which
/// rows meet it, with no branch per row: in a dense pile each test is about
/// even odds, so branches on them mispredict. Lanes past the page's rows
/// hold empty boxes, which meet nothing.
#[derive(Clone, Copy)]
pub struct Lanes {
    min_x: [f32; SPATIAL_PAGE_ROWS],
    min_y: [f32; SPATIAL_PAGE_ROWS],
    max_x: [f32; SPATIAL_PAGE_ROWS],
    max_y: [f32; SPATIAL_PAGE_ROWS],
    pub index: [u32; SPATIAL_PAGE_ROWS],
    len: usize,
}

// Masks are `u32`s.
const _: () = assert!(SPATIAL_PAGE_ROWS <= 32);

impl Lanes {
    const EMPTY: Lanes = Lanes {
        min_x: [f32::INFINITY; SPATIAL_PAGE_ROWS],
        min_y: [f32::INFINITY; SPATIAL_PAGE_ROWS],
        max_x: [f32::NEG_INFINITY; SPATIAL_PAGE_ROWS],
        max_y: [f32::NEG_INFINITY; SPATIAL_PAGE_ROWS],
        index: [0; SPATIAL_PAGE_ROWS],
        len: 0,
    };

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Row `i`'s box.
    pub fn get(&self, i: usize) -> Bounds {
        assert!(i < self.len, "row {i} of {}", self.len);
        Bounds { min: [self.min_x[i], self.min_y[i]], max: [self.max_x[i], self.max_y[i]] }
    }

    fn set(&mut self, i: usize, b: Bounds) {
        assert!(i < self.len, "row {i} of {}", self.len);
        (self.min_x[i], self.min_y[i], self.max_x[i], self.max_y[i]) = (b.min[0], b.min[1], b.max[0], b.max[1]);
    }

    fn push(&mut self, b: Bounds, index: u32) {
        self.len += 1;
        self.set(self.len - 1, b);
        self.index[self.len - 1] = index;
    }

    /// Row `i` out, and the last row in its place, as `Vec::swap_remove`.
    fn swap_remove(&mut self, i: usize) -> (Bounds, u32) {
        let (b, index, last) = (self.get(i), self.index[i], self.len - 1);
        self.set(i, self.get(last));
        self.index[i] = self.index[last];
        self.set(last, Bounds::EMPTY);
        self.len -= 1;
        (b, index)
    }

    /// The box around every row.
    fn bounds(&self) -> Bounds {
        (0..self.len).fold(Bounds::EMPTY, |u, i| u.union(&self.get(i)))
    }

    /// Row `i`'s box, grown by `grow`: unchecked against `len`, for the
    /// broadphase's inner loops, which only ask for rows a mask names.
    #[inline(always)]
    pub fn grown(&self, i: usize, grow: f32) -> Bounds {
        Bounds { min: [self.min_x[i], self.min_y[i]], max: [self.max_x[i], self.max_y[i]] }.grown(grow)
    }

    /// Which rows' boxes, grown by `grow`, meet `b`, as bits by row. Grown
    /// here rather than stored grown, with the same operations as
    /// `Bounds::grown`, so the answer is bit for bit that of growing first
    /// (and a grow of 0 changes nothing).
    #[inline(always)]
    pub fn meeting(&self, b: &Bounds, grow: f32) -> u32 {
        let mut m = 0;
        for i in 0..SPATIAL_PAGE_ROWS {
            let hit = ((self.min_x[i] - grow) <= b.max[0])
                & (b.min[0] <= (self.max_x[i] + grow))
                & ((self.min_y[i] - grow) <= b.max[1])
                & (b.min[1] <= (self.max_y[i] + grow));
            m |= (hit as u32) << i;
        }
        m
    }
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
/// range and box, and each row's box (in `lanes`) and key, parallel to the
/// page's rows.
pub struct SpatialPages {
    pub kind: Vec<PageKind>,
    /// The lowest key of an ordered page.
    lo: Vec<u64>,
    /// Ordered pages, by `lo`.
    pub order: Vec<u32>,
    pub bounds: Vec<Bounds>,
    pub lanes: Vec<Lanes>,
    keys: Vec<Vec<u64>>,
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
            lanes: vec![Lanes::EMPTY],
            keys: vec![Vec::new()],
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
        self.lanes.push(Lanes::EMPTY);
        self.keys.push(Vec::new());
    }

    /// A row pushed onto `page`: placeholders until the re-sort.
    pub(crate) fn push_row(&mut self, page: usize, e: Entity) {
        self.lanes[page].push(Bounds::EMPTY, e.index);
        self.keys[page].push(u64::MAX);
        self.dirty = true;
    }

    /// A row swap-removed from `page`, as the table's own rows are.
    pub(crate) fn swap_remove(&mut self, page: usize, row: usize) {
        self.lanes[page].swap_remove(row);
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
        if rows.len() != self.kind.len() || self.lanes.iter().map(Lanes::len).ne(rows.iter().map(Vec::len)) {
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
                if self.lanes[p].get(r).reach() > big {
                    return Err(format!("{:?} is big, and on ordered page {p}", rows[p][r]));
                }
            }
        }
        for p in 0..self.kind.len() {
            if self.kind[p] == PageKind::Staging && !rows[p].is_empty() {
                return Err(format!("page {p} still has staged rows"));
            }
            let l = &self.lanes[p];
            if rows[p].iter().enumerate().any(|(r, e)| l.index[r] != e.index) {
                return Err(format!("page {p}'s boxes aren't in its rows' order"));
            }
            if self.kind[p] == PageKind::Big && (0..l.len()).any(|r| l.get(r).reach() <= big) {
                return Err(format!("big page {p} holds a row that isn't"));
            }
            for b in (0..l.len()).map(|r| l.get(r)) {
                if !contains(&self.bounds[p], &b) {
                    return Err(format!("page {p}'s box doesn't hold its row {b:?}"));
                }
            }
        }
        for (r, run) in self.order.chunks(RUN).enumerate() {
            if run.iter().any(|&p| !contains(&self.runs[r], &self.bounds[p as usize]) && !self.lanes[p as usize].is_empty()) {
                return Err(format!("run {r}'s box doesn't hold its pages"));
            }
        }
        Ok(())
    }

    /// Row `r` of page `p`'s box, as of the last re-sort.
    pub fn row_bounds(&self, p: usize, r: usize) -> Bounds {
        self.lanes[p].get(r)
    }

    fn rebound(&mut self) {
        for (p, l) in self.lanes.iter().enumerate() {
            self.bounds[p] = l.bounds();
        }
        self.runs = self.order.chunks(RUN).map(|run| run.iter().fold(Bounds::EMPTY, |b, &p| b.union(&self.bounds[p as usize]))).collect();
    }
}

/// Whether a row is where `Resort::target` would put it, found without
/// its search: an ordered page's row whose key is below the next page's
/// `lo` (`hi`), or a big page's big row. Most rows, since motion is small
/// next to a page.
fn stays(kind: PageKind, lo: u64, hi: u64, big: bool, key: u64) -> bool {
    match kind {
        PageKind::Ordered => !big && lo <= key && key < hi,
        PageKind::Big => big,
        PageKind::Staging => false,
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
    /// Per physical page, the next ordered page's `lo`: for `stays`. Only
    /// ordered pages' are read, so a big page made since needn't have one.
    pub hi: Vec<u64>,
    /// Set by a split, which moves ranges' ends: `hi` is stale. Pass true.
    pub ranges_changed: bool,
}

impl Resort<'_> {
    /// Bounds every row, then moves each row that isn't in the page its key
    /// (or its size) says, splitting full pages. Returns rows moved.
    pub fn run(mut self) -> usize {
        let since = self.pages.sorted_tick;
        let mut rebounded = 0;
        self.refresh_ranges();
        // Pages with a row that may not stay: only those are walked. A row
        // that isn't re-bounded stays, since ranges only change as rows move.
        let mut unsettled = vec![false; self.rows.len()];
        let Resort { rows, columns, pages, key, extent, desc, hi, .. } = &mut self;
        for p in 0..rows.len() {
            let key_column = &columns[*key][p];
            let extent_column = extent.map(|x| &columns[x][p]);
            let (lanes, keys) = (&mut pages.lanes[p], &mut pages.keys[p]);
            for r in 0..rows[p].len() {
                // Only rows new to the table (a placeholder key) or whose key
                // or extent was written since the last sort: the rest keep
                // their boxes and keys.
                let new = keys[r] == u64::MAX;
                if !new && key_column.ticks()[r] <= since && extent_column.is_none_or(|c| c.ticks()[r] <= since) {
                    continue;
                }
                rebounded += 1;
                let k = key_column.value_ptr(r);
                let x = extent_column.map_or(std::ptr::null(), |c| c.value_ptr(r));
                // SAFETY: `k` is a value of the key's installed layout,
                // whose build the glue came from, and `x` is only passed
                // when installed with the layout the glue reads.
                let b = unsafe { (desc.bounds)(k, x) };
                lanes.set(r, b);
                keys[r] = morton(&b, desc.cell);
                unsettled[p] |= !stays(pages.kind[p], pages.lo[p], hi[p], b.reach() > desc.big, keys[r]);
            }
        }
        self.pages.free = (0..self.rows.len())
            .filter(|&p| self.pages.kind[p] == PageKind::Staging && self.rows[p].is_empty())
            .map(|p| p as u32)
            .collect();
        let mut moved = 0;
        // Pages a split makes are walked too (they're past `unsettled`),
        // since they may take rows that haven't been placed yet.
        let mut p = 0;
        while p < self.rows.len() {
            if unsettled.get(p) == Some(&false) {
                p += 1;
                continue;
            }
            let mut r = 0;
            while r < self.rows[p].len() {
                if self.row_stays(p, r) {
                    r += 1;
                    continue;
                }
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
        self.pages.sorted_tick = self.now;
        self.pages.rebounded = rebounded;
        moved
    }

    /// The end of each ordered page's range, `hi`, if a split has changed
    /// them since they were last found.
    fn refresh_ranges(&mut self) {
        if self.ranges_changed {
            self.hi.clear();
            self.hi.resize(self.rows.len(), 0);
            for (i, &q) in self.pages.order.iter().enumerate() {
                self.hi[q as usize] = self.pages.order.get(i + 1).map_or(u64::MAX, |&n| self.pages.lo[n as usize]);
            }
            self.ranges_changed = false;
        }
    }

    /// `stays` for row `r` of page `p`.
    fn row_stays(&mut self, p: usize, r: usize) -> bool {
        self.refresh_ranges();
        let big = self.pages.lanes[p].get(r).reach() > self.desc.big;
        let hi = self.hi.get(p).copied().unwrap_or(0);
        stays(self.pages.kind[p], self.pages.lo[p], hi, big, self.pages.keys[p][r])
    }

    /// Where row `r` of page `p` belongs, making room there if it's full.
    fn target(&mut self, p: usize, r: usize) -> usize {
        let b = self.pages.lanes[p].get(r);
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
        self.ranges_changed = true;
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
        let (b, index) = self.pages.lanes[p].swap_remove(r);
        let k = self.pages.keys[p].swap_remove(r);
        self.pages.lanes[q].push(b, index);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_i64_is_floor() {
        let mut vs = vec![0.0, -0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0e-45, -2.0e-45, 16_777_215.5, -16_777_215.5];
        vs.extend([16_777_216.0, -16_777_217.0, 3.0e9, -3.0e9, 9.3e18, -9.3e18, 1.0e30, -1.0e30]);
        vs.extend([f32::MAX, f32::MIN, f32::INFINITY, f32::NEG_INFINITY, f32::NAN]);
        vs.extend((-2000..2000).map(|i| i as f32 * 0.37));
        for v in vs {
            assert_eq!(floor_i64(v), v.floor() as i64, "{v}");
        }
    }

    #[test]
    fn lanes_meet_as_grown_boxes_overlap() {
        let mut l = Lanes::EMPTY;
        let boxes: Vec<Bounds> = (0..SPATIAL_PAGE_ROWS - 3)
            .map(|i| Bounds::around([i as f32 * 0.7, (i % 3) as f32], [0.3 + (i % 2) as f32 * 0.1, 0.4]))
            .collect();
        for (i, b) in boxes.iter().enumerate() {
            l.push(*b, i as u32);
        }
        l.swap_remove(2);
        let now: Vec<Bounds> = (0..l.len()).map(|i| l.get(i)).collect();
        assert_eq!(now[2], boxes[boxes.len() - 1], "the last row took the removed one's place");
        assert_eq!(l.index[2], boxes.len() as u32 - 1);
        for grow in [0.0, 0.05, 0.3] {
            for probe in (0..40).map(|i| Bounds::around([i as f32 * 0.25 - 1.0, (i % 5) as f32 * 0.5], [0.2, 0.1])) {
                let want = now.iter().enumerate().fold(0, |m, (i, b)| m | ((b.grown(grow).overlaps(&probe) as u32) << i));
                assert_eq!(l.meeting(&probe, grow), want, "{probe:?} grown {grow}");
            }
        }
        // Lanes past the rows, the removed one's included, meet nothing.
        assert_eq!(l.meeting(&Bounds::new([f32::MIN; 2], [f32::MAX; 2]), 1.0) >> l.len(), 0);
    }
}
