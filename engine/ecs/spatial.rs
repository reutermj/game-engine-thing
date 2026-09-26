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
use crate::par::Workers;
use crate::world::{Entities, Location, TableId};

/// Rows per page in a spatial table: a page is a neighborhood, so small
/// (the spike measured 8 to 16 as best; see the doc).
pub const SPATIAL_PAGE_ROWS: usize = 16;
/// Pages per run: the level above pages. Pages are in Z-order, so a run of
/// consecutive pages is a neighborhood too.
pub(crate) const RUN: usize = 16;

/// An axis-aligned box: what a spatial key's bounds are, in `D` dimensions
/// (2 or 3). `D` defaults to 2, so a plane's code names `Bounds` alone.
/// Every loop over the axes is over a constant, so each dimension compiles
/// to the code it would have had written out (spatial-storage.md, "In 3D").
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds<const D: usize = 2> {
    pub min: [f32; D],
    pub max: [f32; D],
}

impl<const D: usize> Bounds<D> {
    pub const EMPTY: Bounds<D> = Bounds { min: [f32::INFINITY; D], max: [f32::NEG_INFINITY; D] };

    pub fn new(min: [f32; D], max: [f32; D]) -> Bounds<D> {
        Bounds { min, max }
    }

    /// The box `half` around `center`.
    pub fn around(center: [f32; D], half: [f32; D]) -> Bounds<D> {
        Bounds { min: std::array::from_fn(|a| center[a] - half[a]), max: std::array::from_fn(|a| center[a] + half[a]) }
    }

    /// Touching counts.
    pub fn overlaps(&self, o: &Bounds<D>) -> bool {
        (0..D).all(|a| self.min[a] <= o.max[a] && o.min[a] <= self.max[a])
    }

    pub fn union(&self, o: &Bounds<D>) -> Bounds<D> {
        Bounds { min: std::array::from_fn(|a| self.min[a].min(o.min[a])), max: std::array::from_fn(|a| self.max[a].max(o.max[a])) }
    }

    pub fn grown(&self, by: f32) -> Bounds<D> {
        Bounds { min: std::array::from_fn(|a| self.min[a] - by), max: std::array::from_fn(|a| self.max[a] + by) }
    }

    fn center(&self) -> [f32; D] {
        std::array::from_fn(|a| (self.min[a] + self.max[a]) / 2.0)
    }

    /// The largest half extent.
    fn reach(&self) -> f32 {
        let mut r = (self.max[0] - self.min[0]) / 2.0;
        for a in 1..D {
            r = r.max((self.max[a] - self.min[a]) / 2.0);
        }
        r
    }
}

/// A component whose tables are kept in spatial order: declared with
/// `order = spatial` in `component!`. `bounds` is the box around an entity
/// with this key, and its `Extent` if it has one (a position's collider);
/// a key with no extent of its own names itself as its extent. Mark
/// `bounds` `#[inline]`: the glue calls it per row from another crate
/// (docs/lore/a-trait-impl-the-glue-calls-is-not-inlined-across-crates.md).
/// `D`, the dimensions, is 2 unless the key implements `SpatialKey<3>`.
pub trait SpatialKey<const D: usize = 2>: Component {
    type Extent: Component;
    /// Cells of the Z-order: about the size of the smallest things, so
    /// neighbors in the order are neighbors in space.
    const CELL: f32 = 1.0;
    /// Bigger than this (the larger half extent), a row is kept out of the
    /// order in pages of its own: a floor across the level would otherwise
    /// stretch its page's box over everything.
    const BIG: f32 = 2.0;
    fn bounds(&self, extent: Option<&Self::Extent>) -> Bounds<D>;
}

/// The bounds glue: reads one page's keys and, if the rows have them,
/// extents, as the types the declaring build compiled, and writes the box of
/// each row in `rows` to `out` (parallel to the page). A page at a time, not
/// a row: the call can't be inlined, and one per row, with the caller's
/// state saved around each, measured a third of a re-sort (2026-09-24).
pub type BoundsFn<const D: usize = 2> = unsafe fn(keys: *const u8, extents: *const u8, rows: &[u32], out: &mut [Bounds<D>]);

/// A key's bounds glue, by its dimensions: each spatial table's order is
/// in its key's dimensions, chosen when the table is made.
#[derive(Clone, Copy)]
pub enum BoundsGlue {
    Plane(BoundsFn<2>),
    Space(BoundsFn<3>),
}

impl BoundsGlue {
    pub fn dims(&self) -> usize {
        match self {
            BoundsGlue::Plane(_) => 2,
            BoundsGlue::Space(_) => 3,
        }
    }
}

/// Picks the parts of the world made for `D` dimensions, for code generic
/// over `D`: a const generic can't pick an enum variant by itself, and
/// arrays sized by a trait's associated constant need an unstable feature,
/// so the pick is a pair of impls, one per dimension.
#[doc(hidden)]
pub struct Axes<const D: usize>;

#[doc(hidden)]
pub trait Dims<const D: usize> {
    fn glue(g: BoundsGlue) -> Option<BoundsFn<D>>;
    fn pages(o: &SpatialOrder) -> Option<&SpatialPages<D>>;
}

impl Dims<2> for Axes<2> {
    fn glue(g: BoundsGlue) -> Option<BoundsFn<2>> {
        if let BoundsGlue::Plane(f) = g { Some(f) } else { None }
    }
    fn pages(o: &SpatialOrder) -> Option<&SpatialPages<2>> {
        if let SpatialOrder::Plane(p) = o { Some(p) } else { None }
    }
}

impl Dims<3> for Axes<3> {
    fn glue(g: BoundsGlue) -> Option<BoundsFn<3>> {
        if let BoundsGlue::Space(f) = g { Some(f) } else { None }
    }
    fn pages(o: &SpatialOrder) -> Option<&SpatialPages<3>> {
        if let SpatialOrder::Space(p) = o { Some(p) } else { None }
    }
}

/// The glue for key `T` in `D` dimensions, as a constant `SpatialDesc::of`
/// can name: the same pair-of-impls pick as `Axes`, in a `const fn`.
#[doc(hidden)]
pub struct GlueOf<T, const D: usize>(std::marker::PhantomData<T>);

#[doc(hidden)]
pub trait MakeGlue {
    const GLUE: BoundsGlue;
}

impl<T: SpatialKey<2>> MakeGlue for GlueOf<T, 2> {
    const GLUE: BoundsGlue = BoundsGlue::Plane(__bounds::<T, 2>);
}

impl<T: SpatialKey<3>> MakeGlue for GlueOf<T, 3> {
    const GLUE: BoundsGlue = BoundsGlue::Space(__bounds::<T, 3>);
}

/// What a spatial key declares, carried in its `ComponentDesc`.
#[derive(Clone, Copy)]
pub struct SpatialDesc {
    pub extent: &'static str,
    /// The extent's layout this glue reads: a row whose extent is installed
    /// with another is bounded without it.
    pub extent_fingerprint: u64,
    pub bounds: BoundsGlue,
    pub cell: f32,
    pub big: f32,
}

impl SpatialDesc {
    pub const fn of<T: SpatialKey<D>, const D: usize>() -> SpatialDesc
    where
        GlueOf<T, D>: MakeGlue,
    {
        SpatialDesc {
            extent: <<T as SpatialKey<D>>::Extent as Component>::NAME,
            extent_fingerprint: <<T as SpatialKey<D>>::Extent as Component>::FINGERPRINT,
            bounds: <GlueOf<T, D> as MakeGlue>::GLUE,
            cell: <T as SpatialKey<D>>::CELL,
            big: <T as SpatialKey<D>>::BIG,
        }
    }

    pub fn dims(&self) -> usize {
        self.bounds.dims()
    }
}

#[doc(hidden)]
pub unsafe fn __bounds<T: SpatialKey<D>, const D: usize>(keys: *const u8, extents: *const u8, rows: &[u32], out: &mut [Bounds<D>]) {
    for &r in rows {
        let r = r as usize;
        // Indexed first, so a row past the page panics before it's read.
        let slot = &mut out[r];
        // SAFETY: the caller passes `out.len()` values of the installed
        // layout of `T`, the key, which is this build's (the glue is
        // installed with the layout), and as many extents only if they're
        // installed with the layout this build read; `r` is within them.
        let key = unsafe { &*(keys as *const T).add(r) };
        let extent = (!extents.is_null()).then(|| unsafe { &*(extents as *const <T as SpatialKey<D>>::Extent).add(r) });
        *slot = key.bounds(extent);
    }
}

/// Whether an ordered page's range, from `lo` up to `hi` (the next page's
/// `lo`), holds `key`. Pages may share a `lo` (a run of one key too many
/// for a page); all but the last of them hold exactly that key.
fn holds(lo: u64, hi: u64, key: u64) -> bool {
    key >= lo && (key < hi || (key == hi && hi == lo))
}

fn contains<const D: usize>(outer: &Bounds<D>, inner: &Bounds<D>) -> bool {
    (0..D).all(|a| outer.min[a] <= inner.min[a] && outer.max[a] >= inner.max[a])
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

/// Bits of a cell along each axis in 3D: three axes to a 64-bit key.
const SPACE_BITS: u32 = 21;

fn spread(v: u32) -> u64 {
    let mut x = v as u64;
    x = (x | (x << 16)) & 0x0000_FFFF_0000_FFFF;
    x = (x | (x << 8)) & 0x00FF_00FF_00FF_00FF;
    x = (x | (x << 4)) & 0x0F0F_0F0F_0F0F_0F0F;
    x = (x | (x << 2)) & 0x3333_3333_3333_3333;
    (x | (x << 1)) & 0x5555_5555_5555_5555
}

/// `spread` for three axes: the low 21 bits of `v`, two zeros after each.
fn spread3(v: u32) -> u64 {
    let mut x = (v as u64) & ((1 << SPACE_BITS) - 1);
    x = (x | (x << 32)) & 0x001F_0000_0000_FFFF;
    x = (x | (x << 16)) & 0x001F_0000_FF00_00FF;
    x = (x | (x << 8)) & 0x100F_00F0_0F00_F00F;
    x = (x | (x << 4)) & 0x10C3_0C30_C30C_30C3;
    (x | (x << 2)) & 0x1249_2492_4924_9249
}

/// The cell of a box's center, every axis packed, given cells per unit:
/// multiplying by it measured a third cheaper than dividing by the cell,
/// and a key only has to be the same for the same box within one table.
/// In 3D an axis keeps 21 bits, saturating at 2^21 - 1: a million cells
/// either side of zero, as in 2D, whose negative side ends there too.
fn cells<const D: usize>(b: &Bounds<D>, per_cell: f32) -> u64 {
    let c = b.center();
    if D == 2 {
        cell(c[0], per_cell) as u64 | (cell(c[1], per_cell) as u64) << 32
    } else {
        let max = (1u32 << SPACE_BITS) - 1;
        (0..D).fold(0, |packed, a| packed | (cell(c[a], per_cell).min(max) as u64) << (SPACE_BITS * a as u32))
    }
}

/// The Z-order key of packed cells. A new row's placeholder cells,
/// `u64::MAX`, key as `u64::MAX`: in 2D because every bit is set, in 3D
/// because it's the one key no packed cells make (they leave the top bit
/// clear), so a placeholder is never taken for a real key.
fn morton<const D: usize>(cells: u64) -> u64 {
    if D == 2 {
        spread(cells as u32) | (spread((cells >> 32) as u32) << 1)
    } else if cells == u64::MAX {
        u64::MAX
    } else {
        let axis = |a: u32| ((cells >> (SPACE_BITS * a)) as u32) & ((1 << SPACE_BITS) - 1);
        spread3(axis(0)) | spread3(axis(1)) << 1 | spread3(axis(2)) << 2
    }
}

/// A page's rows' boxes, by coordinate, with their entities' indices: the
/// order's one copy of each row's box, kept by coordinate so a box is
/// tested against every row of a page at once. `meeting` is a mask of which
/// rows meet it, with no branch per row: in a dense pile each test is about
/// even odds, so branches on them mispredict. Lanes past the page's rows
/// hold empty boxes, which meet nothing.
#[derive(Clone, Copy)]
pub struct Lanes<const D: usize = 2> {
    /// By axis, then row.
    min: [[f32; SPATIAL_PAGE_ROWS]; D],
    max: [[f32; SPATIAL_PAGE_ROWS]; D],
    pub index: [u32; SPATIAL_PAGE_ROWS],
    /// Each row's Z-order key, and its cell on each axis, packed: a row that
    /// moved within its cell keeps its key, and interleaving the bits for
    /// the key was most of re-keying a creeping row. Here rather than in
    /// vectors by page, so a row's box, key and cell move together and a
    /// page's are one allocation with its boxes.
    key: [u64; SPATIAL_PAGE_ROWS],
    cell: [u64; SPATIAL_PAGE_ROWS],
    len: usize,
}

// Masks are `u32`s.
const _: () = assert!(SPATIAL_PAGE_ROWS <= 32);

impl<const D: usize> Lanes<D> {
    const EMPTY: Lanes<D> = Lanes {
        min: [[f32::INFINITY; SPATIAL_PAGE_ROWS]; D],
        max: [[f32::NEG_INFINITY; SPATIAL_PAGE_ROWS]; D],
        index: [0; SPATIAL_PAGE_ROWS],
        key: [u64::MAX; SPATIAL_PAGE_ROWS],
        cell: [u64::MAX; SPATIAL_PAGE_ROWS],
        len: 0,
    };

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Row `i`'s box.
    pub fn get(&self, i: usize) -> Bounds<D> {
        assert!(i < self.len, "row {i} of {}", self.len);
        self.at(i)
    }

    #[inline(always)]
    fn at(&self, i: usize) -> Bounds<D> {
        Bounds { min: std::array::from_fn(|a| self.min[a][i]), max: std::array::from_fn(|a| self.max[a][i]) }
    }

    fn set(&mut self, i: usize, b: Bounds<D>) {
        assert!(i < self.len, "row {i} of {}", self.len);
        for a in 0..D {
            (self.min[a][i], self.max[a][i]) = (b.min[a], b.max[a]);
        }
    }

    /// Row `i`'s key.
    fn key(&self, i: usize) -> u64 {
        assert!(i < self.len, "row {i} of {}", self.len);
        self.key[i]
    }

    fn keys(&self) -> &[u64] {
        &self.key[..self.len]
    }

    /// A row with its box, entity index, key and cells.
    fn push(&mut self, (b, index, key, cell): (Bounds<D>, u32, u64, u64)) {
        assert!(self.len < SPATIAL_PAGE_ROWS, "a page holds {SPATIAL_PAGE_ROWS} rows");
        self.len += 1;
        let i = self.len - 1;
        self.set(i, b);
        (self.index[i], self.key[i], self.cell[i]) = (index, key, cell);
    }

    /// Row `i` out, and the last row in its place, as `Vec::swap_remove`.
    fn swap_remove(&mut self, i: usize) -> (Bounds<D>, u32, u64, u64) {
        let (b, last) = (self.get(i), self.len - 1);
        let out = (b, self.index[i], self.key[i], self.cell[i]);
        self.set(i, self.get(last));
        (self.index[i], self.key[i], self.cell[i]) = (self.index[last], self.key[last], self.cell[last]);
        self.set(last, Bounds::EMPTY);
        self.len -= 1;
        out
    }

    /// The box around every row: over every lane, since those past the rows
    /// hold empty boxes, halving the lanes each round, so each round is one
    /// vector operation, where a fold is a chain of dependent compares
    /// (floats aren't reassociated to vectorize it).
    fn bounds(&self) -> Bounds<D> {
        fn reduce(l: &[f32; SPATIAL_PAGE_ROWS], pick: impl Fn(f32, f32) -> f32) -> f32 {
            let mut v = *l;
            let mut n = SPATIAL_PAGE_ROWS;
            while n > 1 {
                let half = n.div_ceil(2);
                for i in 0..n - half {
                    v[i] = pick(v[i], v[i + half]);
                }
                n = half;
            }
            v[0]
        }
        let (min, max) = (|a: f32, b: f32| if a < b { a } else { b }, |a: f32, b: f32| if a > b { a } else { b });
        Bounds { min: std::array::from_fn(|a| reduce(&self.min[a], min)), max: std::array::from_fn(|a| reduce(&self.max[a], max)) }
    }

    /// Row `i`'s box, grown by `grow`: unchecked against `len`, for the
    /// broadphase's inner loops, which only ask for rows a mask names.
    #[inline(always)]
    pub fn grown(&self, i: usize, grow: f32) -> Bounds<D> {
        self.at(i).grown(grow)
    }

    /// Which rows' boxes, grown by `grow`, meet `b`, as bits by row. Grown
    /// here rather than stored grown, with the same operations as
    /// `Bounds::grown`, so the answer is bit for bit that of growing first
    /// (and a grow of 0 changes nothing).
    #[inline(always)]
    pub fn meeting(&self, b: &Bounds<D>, grow: f32) -> u32 {
        let mut m = 0;
        for i in 0..SPATIAL_PAGE_ROWS {
            let mut hit = true;
            for a in 0..D {
                hit &= ((self.min[a][i] - grow) <= b.max[a]) & (b.min[a] <= (self.max[a][i] + grow));
            }
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
/// range and box, and each row's box and key, parallel to the page's rows.
pub struct SpatialPages<const D: usize = 2> {
    pub kind: Vec<PageKind>,
    /// The lowest key of an ordered page.
    lo: Vec<u64>,
    /// Ordered pages, by `lo`.
    pub order: Vec<u32>,
    pub bounds: Vec<Bounds<D>>,
    /// Each row's box, key and cells, parallel to the page's rows.
    pub lanes: Vec<Lanes<D>>,
    /// Each ordered page's upper key (the next page's `lo`), by physical
    /// page, as of the start of a re-sort: so a row can be checked against
    /// its own page's range without a search. Not rebuilt as splits narrow
    /// ranges, since a split moves out every row the narrowing excludes,
    /// and the pages it makes aren't walked.
    hi: Vec<u64>,
    /// By page, as bits by row, the rows a re-sort must move: those that
    /// aren't where their keys say. Kept through the moves (a swap-removed
    /// row's bit goes with it), so a page's other rows aren't asked again;
    /// asking every row of a page with one to move was about a fifth of
    /// moving them, falling (2026-09-24). Kept here only to reuse the allocation.
    misplaced: Vec<u32>,
    /// What the bounds glue writes a page's boxes to, before they go into
    /// its lanes: the glue writes whole `Bounds`. Only the rows it's asked
    /// for are read back, so it's never cleared.
    written_bounds: [Bounds<D>; SPATIAL_PAGE_ROWS],
    /// Pages whose box may no longer be the union of their rows': a row
    /// arrived, left, or moved since it was last computed.
    stale: Vec<bool>,
    /// Boxes over runs of `RUN` pages of `order`.
    pub runs: Vec<Bounds<D>>,
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

impl<const D: usize> Default for SpatialPages<D> {
    fn default() -> SpatialPages<D> {
        // One ordered page for every key to start.
        SpatialPages {
            kind: vec![PageKind::Ordered],
            lo: vec![0],
            order: vec![0],
            bounds: vec![Bounds::EMPTY],
            lanes: vec![Lanes::EMPTY],
            hi: vec![u64::MAX],
            misplaced: Vec::new(),
            written_bounds: [Bounds::EMPTY; SPATIAL_PAGE_ROWS],
            stale: vec![false],
            runs: vec![Bounds::EMPTY],
            free: Vec::new(),
            dirty: false,
            sorted_tick: 0,
            rebounded: 0,
        }
    }
}

impl<const D: usize> SpatialPages<D> {
    /// A new physical page, for rows placed before the next re-sort.
    pub(crate) fn add_page(&mut self) {
        self.kind.push(PageKind::Staging);
        self.lo.push(u64::MAX);
        self.bounds.push(Bounds::EMPTY);
        self.lanes.push(Lanes::EMPTY);
        self.stale.push(false);
        self.misplaced.push(0);
    }

    /// A row pushed onto `page`: placeholders until the re-sort.
    pub(crate) fn push_row(&mut self, page: usize, e: Entity) {
        self.lanes[page].push((Bounds::EMPTY, e.index, u64::MAX, u64::MAX));
        self.stale[page] = true;
        self.dirty = true;
    }

    /// A row swap-removed from `page`, as the table's own rows are.
    pub(crate) fn swap_remove(&mut self, page: usize, row: usize) {
        self.lanes[page].swap_remove(row);
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
    pub fn pages_near(&self, region: &Bounds<D>) -> Vec<usize> {
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
        if rows.len() != self.kind.len() || self.lanes.iter().map(Lanes::len).ne(rows.iter().map(Vec::len)) {
            return Err("the order's pages don't match the table's".into());
        }
        if let Some(p) = rows.iter().position(|r| r.len() > SPATIAL_PAGE_ROWS) {
            return Err(format!("page {p} holds {} rows", rows[p].len()));
        }
        for p in 0..rows.len() {
            let l = &self.lanes[p];
            if rows[p].iter().enumerate().any(|(r, e)| l.index[r] != e.index) {
                return Err(format!("page {p}'s boxes aren't in its rows' order"));
            }
            for (r, b) in (0..l.len()).map(|r| (r, l.get(r))) {
                let c = cells(&b, 1.0 / cell);
                if (l.cell[r], l.key[r]) != (c, morton::<D>(c)) {
                    return Err(format!(
                        "{:?} on page {p} is keyed as {:x}, and its box's key is {:x}",
                        rows[p][r],
                        l.key[r],
                        morton::<D>(c)
                    ));
                }
            }
        }
        if self.order.windows(2).any(|w| self.lo[w[0] as usize] > self.lo[w[1] as usize]) {
            return Err("the ordered pages aren't in order".into());
        }
        for (i, &p) in self.order.iter().enumerate() {
            let p = p as usize;
            let hi = self.order.get(i + 1).map_or(u64::MAX, |&n| self.lo[n as usize]);
            for (r, &k) in self.lanes[p].keys().iter().enumerate() {
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
            if self.kind[p] == PageKind::Big && (0..l.len()).any(|r| l.get(r).reach() <= big) {
                return Err(format!("big page {p} holds a row that isn't"));
            }
            for b in (0..l.len()).map(|r| l.get(r)) {
                if !contains(&self.bounds[p], &b) {
                    return Err(format!("page {p}'s box doesn't hold its row {b:?}"));
                }
            }
            // Rows leaving between re-sorts leave a box loose, marked stale.
            let tight = l.bounds();
            if !self.stale[p] && self.bounds[p] != tight {
                return Err(format!("page {p}'s box is {:?}, and its rows' is {tight:?}", self.bounds[p]));
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
    pub fn row_bounds(&self, p: usize, r: usize) -> Bounds<D> {
        self.lanes[p].get(r)
    }

    /// Recomputes the boxes of stale pages, and, if any page's box may have
    /// changed, every run's: runs are few (a sixteenth of pages), and splits
    /// and merges reshape them.
    fn rebound(&mut self, runs: bool) {
        for (p, l) in self.lanes.iter().enumerate() {
            if std::mem::take(&mut self.stale[p]) {
                self.bounds[p] = l.bounds();
            }
        }
        if !runs {
            return;
        }
        self.runs = self.order.chunks(RUN).map(|run| run.iter().fold(Bounds::EMPTY, |b, &p| b.union(&self.bounds[p as usize]))).collect();
    }
}

/// A spatial table's order, in its key's dimensions: the world holds one per
/// spatial table and picks the dimensions once per operation (a re-sort, a
/// region query, a broadphase), never per row, so each dimension's code is
/// its own monomorphized loop.
pub enum SpatialOrder {
    Plane(SpatialPages<2>),
    Space(SpatialPages<3>),
}

macro_rules! each {
    ($o:expr, $p:ident => $e:expr) => {
        match $o {
            SpatialOrder::Plane($p) => $e,
            SpatialOrder::Space($p) => $e,
        }
    };
}

impl SpatialOrder {
    /// An empty order in `dims` dimensions, 2 or 3.
    pub fn new(dims: usize) -> SpatialOrder {
        match dims {
            2 => SpatialOrder::Plane(SpatialPages::default()),
            3 => SpatialOrder::Space(SpatialPages::default()),
            _ => panic!("spatial keys have 2 or 3 dimensions, not {dims}"),
        }
    }

    pub fn dims(&self) -> usize {
        match self {
            SpatialOrder::Plane(_) => 2,
            SpatialOrder::Space(_) => 3,
        }
    }

    /// Whether it must be re-sorted when the `Structural` holding it drops.
    pub fn dirty(&self) -> bool {
        each!(self, p => p.dirty)
    }

    pub(crate) fn mark(&mut self) {
        each!(self, p => p.dirty = true)
    }

    pub(crate) fn add_page(&mut self) {
        each!(self, p => p.add_page())
    }

    pub(crate) fn push_row(&mut self, page: usize, e: Entity) {
        each!(self, p => p.push_row(page, e))
    }

    pub(crate) fn swap_remove(&mut self, page: usize, row: usize) {
        each!(self, p => p.swap_remove(page, row))
    }

    /// The order, if it's in 2D: for tests.
    #[doc(hidden)]
    pub fn plane(&self) -> Option<&SpatialPages<2>> {
        <Axes<2> as Dims<2>>::pages(self)
    }

    /// The order, if it's in 3D: for tests.
    #[doc(hidden)]
    pub fn space(&self) -> Option<&SpatialPages<3>> {
        <Axes<3> as Dims<3>>::pages(self)
    }

    /// Rows whose box was recomputed at the last re-sort: for tests.
    pub fn rebounded(&self) -> usize {
        each!(self, p => p.rebounded)
    }

    /// `SpatialPages::check`, in whichever dimensions.
    #[doc(hidden)]
    pub fn check(&self, rows: &[Vec<Entity>], big: f32, cell: f32) -> Result<(), String> {
        each!(self, p => p.check(rows, big, cell))
    }

    /// Mean page extent per axis, for the benchmarks: how tight pages are.
    #[doc(hidden)]
    pub fn page_sizes(&self) -> (usize, Vec<f32>) {
        each!(self, p => {
            let full: Vec<&Bounds<_>> = p.order.iter().map(|&q| &p.bounds[q as usize]).filter(|b| b.min[0] <= b.max[0]).collect();
            let n = full.len().max(1) as f32;
            let d = p.bounds.first().map_or(0, |b| b.min.len());
            (full.len(), (0..d).map(|a| full.iter().map(|b| b.max[a] - b.min[a]).sum::<f32>() / n).collect())
        })
    }

    /// Re-sorts it: see `Resort`.
    pub(crate) fn resort(&mut self, parts: ResortParts<'_>) -> usize {
        match self {
            SpatialOrder::Plane(p) => resort_in::<2>(p, parts),
            SpatialOrder::Space(p) => resort_in::<3>(p, parts),
        }
    }
}

/// What a re-sort takes besides the order: see `Resort`.
pub(crate) struct ResortParts<'a> {
    pub table: TableId,
    pub rows: &'a mut Vec<Vec<Entity>>,
    pub columns: Vec<&'a mut Vec<ErasedColumn>>,
    pub key: usize,
    pub extent: Option<usize>,
    pub desc: SpatialDesc,
    pub entities: &'a Entities,
    pub now: u32,
    pub workers: Workers,
}

fn resort_in<const D: usize>(pages: &mut SpatialPages<D>, parts: ResortParts<'_>) -> usize
where
    Axes<D>: Dims<D>,
{
    let ResortParts { table, rows, columns, key, extent, desc, entities, now, workers } = parts;
    // A key installed with glue of other dimensions than its tables' order
    // is refused at install (a restart changes it), so this always finds one.
    let glue = <Axes<D> as Dims<D>>::glue(desc.bounds).expect("a key keeps its dimensions");
    Resort { table, rows, columns, pages, key, extent, desc, glue, entities, now, workers }.run()
}

/// A spatial table's pages, locked for re-sorting: its rows and columns,
/// and what to call the bounds glue with.
pub(crate) struct Resort<'a, const D: usize> {
    pub table: TableId,
    pub rows: &'a mut Vec<Vec<Entity>>,
    pub columns: Vec<&'a mut Vec<ErasedColumn>>,
    pub pages: &'a mut SpatialPages<D>,
    pub key: usize,
    /// The extent's column, if the table has it and it's installed with the
    /// layout the glue reads.
    pub extent: Option<usize>,
    pub desc: SpatialDesc,
    /// The glue, as `desc` has it for this table's dimensions.
    pub glue: BoundsFn<D>,
    pub entities: &'a Entities,
    /// The world's tick as the re-sort starts: every write so far.
    pub now: u32,
    /// The world's executor, for re-bounding pages in parallel.
    pub workers: Workers,
}

/// The fewest rows a re-sort's re-bounding is split across threads for
/// (as pages, at half full): a page takes about 100 ns, a task's hand-off
/// about a microsecond or more.
const PAR_REBOUND_ROWS: usize = 2048;

/// What re-bounding a page reads besides the page: the glue, the tick of
/// the last sort, and every page's range.
struct Rebound<'a, const D: usize> {
    desc: &'a SpatialDesc,
    glue: BoundsFn<D>,
    since: u32,
    kind: &'a [PageKind],
    lo: &'a [u64],
    hi: &'a [u64],
}

impl<const D: usize> Rebound<'_, D> {
    /// Re-bounds and re-keys the rows of page `p` written since the last
    /// sort, marks those its range no longer holds, and re-boxes it.
    /// `scratch` is what the glue writes boxes to. Returns rows re-bounded.
    #[inline(always)]
    fn page(
        &self,
        p: usize,
        key: &ErasedColumn,
        extent: Option<&ErasedColumn>,
        scratch: &mut [Bounds<D>; SPATIAL_PAGE_ROWS],
        (lanes, misplaced_at, bounds, stale): (&mut Lanes<D>, &mut u32, &mut Bounds<D>, &mut bool),
    ) -> usize {
        let (since, big, per_cell) = (self.since, self.desc.big, 1.0 / self.desc.cell);
        let n = lanes.len();
        if n == 0 {
            return 0;
        }
        assert!(key.len() == n && extent.is_none_or(|c| c.len() == n), "a page's order is its rows'");
        // Only rows new to the table (a placeholder key) or whose key or
        // extent was written since the last sort: the rest keep their
        // boxes and keys. As a mask with no branch per row, and the
        // extent's ticks in a loop of their own, not an `Option` per row.
        let mut mask = 0u32;
        for (r, (&k, &t)) in lanes.keys().iter().zip(&key.ticks()[..n]).enumerate() {
            mask |= (((k == u64::MAX) | (t > since)) as u32) << r;
        }
        if let Some(x) = extent {
            for (r, &t) in x.ticks()[..n].iter().enumerate() {
                mask |= ((t > since) as u32) << r;
            }
        }
        if mask == 0 {
            return 0;
        }
        let count = mask.count_ones() as usize;
        let mut written = [0u32; SPATIAL_PAGE_ROWS];
        let mut bits = mask;
        for w in &mut written[..count] {
            *w = bits.trailing_zeros();
            bits &= bits - 1;
        }
        let written = &written[..count];
        let row_bounds = &mut scratch[..n];
        // SAFETY: the page's keys, of the key's installed layout, whose
        // build the glue came from, and its extents only when installed
        // with the layout the glue reads, as many as `row_bounds` holds
        // (checked), which `written` indexes.
        unsafe { (self.glue)(key.value_ptr(0), extent.map_or(std::ptr::null(), |c| c.value_ptr(0)), written, row_bounds) };
        let (kind, lo, hi) = (self.kind[p], self.lo[p], self.hi[p]);
        let mut misplaced = 0u32;
        for &r in written {
            let (b, r) = (row_bounds[r as usize], r as usize);
            lanes.set(r, b);
            let c = cells(&b, per_cell);
            // A new row's placeholders are consistent too: the cells
            // `u64::MAX` are keyed `u64::MAX`.
            if c != lanes.cell[r] {
                (lanes.cell[r], lanes.key[r]) = (c, morton::<D>(c));
            }
            let k = lanes.key[r];
            // Rows that weren't re-bounded are where the last sort put
            // them, and no page's range has changed since: only these
            // can need moving.
            // Without short circuits, which a falling row leaving its range
            // would often mispredict (measured with the page box, not apart).
            let placed = match kind {
                PageKind::Ordered => (b.reach() <= big) & (k >= lo) & ((k < hi) | ((k == hi) & (hi == lo))),
                PageKind::Big => b.reach() > big,
                PageKind::Staging => false,
            };
            misplaced |= (!placed as u32) << r;
        }
        *misplaced_at = misplaced;
        // Boxed here, with its rows' boxes just written, rather than in
        // a second pass; a page no row of which was written keeps its
        // box. A move after marks it stale again.
        *bounds = lanes.bounds();
        *stale = false;
        count
    }
}

impl<const D: usize> Resort<'_, D> {
    /// Bounds every row, then moves each row that isn't in the page its key
    /// (or its size) says, splitting full pages. Returns rows moved.
    pub fn run(mut self) -> usize {
        let since = self.pages.sorted_tick;
        let mut rebounded = 0;
        // Rows arrived or left since the last sort: pages to merge.
        let arrived_or_left = self.pages.stale.contains(&true);
        self.pages.rebuild_hi();
        self.pages.misplaced.clear();
        self.pages.misplaced.resize(self.rows.len(), 0);
        let (key_column, extent_column) = (&*self.columns[self.key], self.extent.map(|x| &*self.columns[x]));
        let pages = &mut *self.pages;
        let order = Rebound { desc: &self.desc, glue: self.glue, since, kind: &pages.kind, lo: &pages.lo, hi: &pages.hi };
        let n = self.rows.len();
        if self.workers.threads() > 1 && n * SPATIAL_PAGE_ROWS / 2 >= PAR_REBOUND_ROWS {
            // Pages are re-bounded independently: in ranges, a task each,
            // with a scratch buffer each. Moves, which cross pages, stay on
            // this thread.
            let ranges = crate::par::even(n, self.workers.chunks(n, PAR_REBOUND_ROWS / SPATIAL_PAGE_ROWS));
            let lens = ranges.iter().map(|r| r.len());
            let tasks: Vec<_> = ranges
                .iter()
                .cloned()
                .zip(crate::par::carve(&mut pages.lanes[..n], lens.clone()))
                .zip(crate::par::carve(&mut pages.misplaced[..n], lens.clone()))
                .zip(crate::par::carve(&mut pages.bounds[..n], lens.clone()))
                .zip(crate::par::carve(&mut pages.stale[..n], lens))
                .collect();
            let counts = self.workers.map_each(tasks, |_, ((((range, lanes), misplaced), bounds), stale)| {
                let mut scratch = [Bounds::EMPTY; SPATIAL_PAGE_ROWS];
                let mut count = 0;
                for (i, p) in range.enumerate() {
                    let at = (&mut lanes[i], &mut misplaced[i], &mut bounds[i], &mut stale[i]);
                    count += order.page(p, &key_column[p], extent_column.map(|c| &c[p]), &mut scratch, at);
                }
                count
            });
            rebounded = counts.iter().sum();
        } else {
            for p in 0..n {
                let at = (&mut pages.lanes[p], &mut pages.misplaced[p], &mut pages.bounds[p], &mut pages.stale[p]);
                rebounded += order.page(p, &key_column[p], extent_column.map(|c| &c[p]), &mut pages.written_bounds, at);
            }
        }
        // Nothing to move or merge: a table at rest costs only the scan of
        // its ticks above.
        let reshape = arrived_or_left || self.pages.misplaced.iter().any(|&m| m != 0);
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
        for p in 0..self.rows.len() {
            // The last first, so the row swapped into a moved row's place is
            // one that stays; `move_row` carries a moved-in bit anyway, for
            // a split's moves out of a page not yet walked.
            while let Some(r) = self.pages.misplaced[p].checked_ilog2() {
                let r = r as usize;
                self.pages.misplaced[p] &= !(1 << r);
                let target = self.target(p, r);
                if target != p {
                    self.move_row(p, r, target);
                    moved += 1;
                }
            }
        }
        self.merge();
        moved
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
        let key = self.pages.lanes[p].key(r);
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

    /// Splits full ordered page `q` at a block boundary between its keys'
    /// quartiles (the median, if they're one key), and returns the half
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
        let mut keys: Vec<u64> = self.pages.lanes[q].keys().iter().copied().filter(|&k| in_range(k)).collect();
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
        // At the key between the quartiles with the most trailing zeros: a
        // boundary of the largest block of the Z-order there, so pages
        // tend to be whole blocks, squares or halves of one, whose boxes
        // are tight; a split at the median leaves ranges straddling blocks.
        // The key is `b` with the bits under its highest difference from
        // `a` cleared: over `a` and at most `b`, so a quarter of the rows
        // or more is on each side.
        let (a, b) = (keys[keys.len() / 4], keys[keys.len() * 3 / 4]);
        let mid = if a < b { b & !((1u64 << (63 - (a ^ b).leading_zeros())) - 1) } else { mid };
        let upper = self.new_page(PageKind::Ordered, mid);
        self.pages.order.insert(at + 1, upper as u32);
        self.move_from(q, upper, |k| in_range(k) && k >= mid);
        if key >= mid { upper } else { q }
    }

    /// Moves the rows of page `q` whose keys `takes` to page `to`.
    fn move_from(&mut self, q: usize, to: usize, takes: impl Fn(u64) -> bool) {
        let mut r = 0;
        while r < self.rows[q].len() {
            if takes(self.pages.lanes[q].key(r)) {
                self.move_row(q, r, to);
            } else {
                r += 1;
            }
        }
    }

    /// Moves row `r` of page `p` to the end of page `q`, in every column.
    fn move_row(&mut self, p: usize, r: usize, q: usize) {
        // The last row's bit follows it to `r`; the moved row is placed.
        let (m, last) = (&mut self.pages.misplaced, self.rows[p].len() - 1);
        m[p] = (m[p] & !(1 << r) & !(1 << last)) | ((m[p] >> last) & 1) << r;
        for c in self.columns.iter_mut() {
            let [from, to] = c.get_disjoint_mut([p, q]).expect("two pages");
            from.swap_remove_into(r, to);
        }
        let e = self.rows[p].swap_remove(r);
        self.rows[q].push(e);
        let row = self.pages.lanes[p].swap_remove(r);
        self.pages.lanes[q].push(row);
        (self.pages.stale[p], self.pages.stale[q]) = (true, true);
        let place =
            |e: Entity, page: usize, row: usize| self.entities.place(e, Location { table: self.table, page: page as u32, row: row as u32 });
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
        assert_eq!(morton::<2>(u64::MAX), u64::MAX);
    }

    #[test]
    fn lanes_meet_as_grown_boxes_overlap() {
        let mut l = Lanes::<2>::EMPTY;
        let boxes: Vec<Bounds> = (0..SPATIAL_PAGE_ROWS - 3)
            .map(|i| Bounds::around([i as f32 * 0.7, (i % 3) as f32], [0.3 + (i % 2) as f32 * 0.1, 0.4]))
            .collect();
        for (i, b) in boxes.iter().enumerate() {
            l.push((*b, i as u32, 0, 0));
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

    /// The 3D key interleaves the three axes bit by bit, x lowest, as a
    /// loop over the bits does: the magic masks against the obvious code.
    #[test]
    fn a_3d_key_interleaves_three_axes() {
        let naive = |x: u32, y: u32, z: u32| {
            (0..SPACE_BITS).fold(0u64, |k, b| {
                k | (((x >> b) & 1) as u64) << (3 * b) | (((y >> b) & 1) as u64) << (3 * b + 1) | (((z >> b) & 1) as u64) << (3 * b + 2)
            })
        };
        let mut s = 0x9e37_79b9_7f4a_7c15u64;
        for _ in 0..10_000 {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            let (x, y, z) = (s as u32 & 0x1f_ffff, (s >> 21) as u32 & 0x1f_ffff, (s >> 42) as u32 & 0x1f_ffff);
            let packed = x as u64 | (y as u64) << 21 | (z as u64) << 42;
            assert_eq!(morton::<3>(packed), naive(x, y, z), "{x:x} {y:x} {z:x}");
        }
        // Saturated cells are the greatest real key, under the placeholder.
        let top = cells(&Bounds::<3>::around([1.0e10; 3], [0.1; 3]), 1.0);
        assert!(morton::<3>(top) < u64::MAX);
        assert_eq!(morton::<3>(u64::MAX), u64::MAX);
        assert_eq!(cells(&Bounds::<3>::around([-1.0e10; 3], [0.1; 3]), 1.0), 0);
    }

    #[test]
    fn lanes_meet_as_grown_boxes_overlap_in_3d() {
        let mut l = Lanes::<3>::EMPTY;
        let boxes: Vec<Bounds<3>> = (0..SPATIAL_PAGE_ROWS - 2)
            .map(|i| Bounds::around([i as f32 * 0.7, (i % 3) as f32, (i % 4) as f32 * 0.6], [0.3, 0.4, 0.2 + (i % 2) as f32 * 0.1]))
            .collect();
        for (i, b) in boxes.iter().enumerate() {
            l.push((*b, i as u32, 0, 0));
        }
        l.swap_remove(1);
        let now: Vec<Bounds<3>> = (0..l.len()).map(|i| l.get(i)).collect();
        for grow in [0.0, 0.05, 0.3] {
            for probe in
                (0..60).map(|i| Bounds::around([i as f32 * 0.2 - 1.0, (i % 5) as f32 * 0.5, (i % 7) as f32 * 0.4], [0.2, 0.1, 0.15]))
            {
                let want = now.iter().enumerate().fold(0, |m, (i, b)| m | ((b.grown(grow).overlaps(&probe) as u32) << i));
                assert_eq!(l.meeting(&probe, grow), want, "{probe:?} grown {grow}");
            }
        }
        // Apart on z alone is apart.
        let above = Bounds::around([0.0, 0.0, 50.0], [100.0, 100.0, 1.0]);
        assert_eq!(l.meeting(&above, 0.0), 0);
        assert_eq!(l.bounds(), now.iter().fold(Bounds::EMPTY, |b, x| b.union(x)));
    }
}
