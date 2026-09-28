//! Live relations: a result the world keeps current as the rows under it
//! change, and always exact. The one kind so far is `Proximity`, a spatial
//! key's pairs, taken by a system as `Live<R>` (docs/architecture/live.md).
//!
//! Its broadphase keeps its pairs between calls: what `near_pairs`
//! answers, found from what changed since the last call rather than
//! afresh. Each row has a fat box, its box grown by a margin when it was
//! last found, kept until the row leaves it; pairs whose fat boxes meet are
//! kept as candidates. A call walks only the pages whose rows changed since
//! the last (`SpatialPages::changed`), looks for new candidates only for
//! rows that left their fat boxes, and tests each candidate's boxes as
//! `near_pairs` does, so the answer is `near_pairs`' bit for bit, and a pile
//! at rest costs a look per page. See docs/architecture/spatial-storage.md,
//! "Keeping pairs".

use std::marker::PhantomData;
use std::sync::RwLockWriteGuard;

use crate::component::{Component, Entity, Storage};
use crate::par::Workers;
use crate::query::{
    ChangeDecl, Declare, Filter, FilterDecl, FrameCx, NearSide, Param, ParamDecl, Query, QueryDecl, Side, SideTable, SweptPage, Unit,
    meet_unit, meets, near_pairs_with, sweep,
};
use crate::spatial::{Axes, Bounds, Dims, SPATIAL_PAGE_ROWS, contains};
use crate::world::{ComponentId, TableId, TakeGuard, World};

/// A row's side, as its table's: gone (from every table the broadphase
/// covers), active or passive.
const GONE: u8 = 0;
const ACTIVE: u8 = 1;
const PASSIVE: u8 = 2;

/// What's kept of a row, by entity index: one record, since a candidate
/// reads its two ends' at random, and a load each is what testing one costs.
#[derive(Clone, Copy)]
struct Row<const D: usize> {
    /// Its box grown by the call's `grow`, as `near_pairs` tests it: the
    /// same operations on the same values, so the same answer.
    grown: Bounds<D>,
    /// Its fat box, grown as `grown` is: in the record too, so walking a
    /// row that moved is one line of memory, not two.
    fat: Bounds<D>,
    generation: u32,
    /// The call it last left its fat box at (or arrived at), and the call
    /// it was last seen at.
    moved: u32,
    seen: u32,
    side: u8,
}

impl<const D: usize> Default for Row<D> {
    fn default() -> Row<D> {
        Row { grown: Bounds::EMPTY, fat: Bounds::EMPTY, generation: 0, moved: 0, seen: 0, side: GONE }
    }
}

/// A page's rows' entity indices as of the call that last walked it: a row
/// gone from a walked page and seen on none is gone.
#[derive(Clone, Copy)]
struct PageRows {
    len: u8,
    index: [u32; SPATIAL_PAGE_ROWS],
}

impl PageRows {
    const EMPTY: PageRows = PageRows { len: 0, index: [0; SPATIAL_PAGE_ROWS] };
}

/// What a call did, for benches and tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LiveStats {
    /// Rows on pages that changed since the last call, walked.
    pub walked: usize,
    /// Rows that left their fat boxes, or arrived.
    pub moved: usize,
    /// Candidates kept after the call, and pairs found.
    pub candidates: usize,
    pub found: usize,
    pub how: How,
    /// Nanoseconds walking changed pages, looking for new candidates and
    /// testing candidates: for benches (three clock reads a call).
    pub ns: [u64; 3],
}

/// How a call found its pairs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum How {
    /// Nothing changed: the last call's pairs.
    #[default]
    Same,
    /// From the kept candidates and those of what moved.
    Kept,
    /// Nothing moved, and only the candidates of the few rows that changed
    /// were tested.
    Few,
    /// Candidates found afresh, then kept: after a call found afresh.
    Rebuilt,
    /// `near_pairs` afresh, keeping no candidates: too much moved.
    Afresh,
}

/// The pairs kept for one broadphase in `D` dimensions.
struct Pairs<const D: usize> {
    /// The world tick of the last call: pages changed after it are walked.
    tick: u32,
    /// Calls so far, which `Row::moved` and `Row::seen` are stamped with.
    call: u32,
    grow: f32,
    margin: f32,
    /// The tables and each one's side, by id: another set starts over.
    tables: Vec<(TableId, u8)>,
    /// By table (in `tables`' order), by page.
    pages: Vec<Vec<PageRows>>,
    /// By entity index.
    rows: Vec<Row<D>>,
    /// Pairs whose fat boxes meet, at least one end of which is active, as
    /// `pair_of` keys, sorted.
    candidates: Vec<u64>,
    /// Whether `candidates` are all of them (not after a call afresh).
    kept: bool,
    /// Whether nothing has been walked since starting over.
    fresh: bool,
    /// After a call found too much moving: calls left to find afresh
    /// without walking, and how many the last such wait was. Falling,
    /// everything leaves its fat box every step, and the walk that finds
    /// that out cost a quarter of `near_pairs` again (2026-09-27, 47 µs of
    /// 190 at 10 000); so it waits, twice as long each time, up to 16 calls.
    skip: u32,
    wait: u32,
    /// Whether the last call didn't walk, so what's kept lags the tables.
    lags: bool,
    out: Vec<(Entity, Entity)>,
    /// Whether each candidate met at the last call, and the candidates by
    /// their greater end (positions in `candidates`, made when first needed
    /// after they change): so a call where few rows changed, and none
    /// moved, retests only their candidates, found by search from either end.
    met: Vec<bool>,
    by_b: Vec<u32>,
    by_b_ok: bool,
    /// Scratch, kept for their allocations: rows seen leaving, and rows
    /// whose boxes changed in their fat boxes.
    left: Vec<u32>,
    touched: Vec<u32>,
    stats: LiveStats,
}

impl<const D: usize> Default for Pairs<D> {
    fn default() -> Pairs<D> {
        Pairs {
            tick: 0,
            call: 0,
            grow: f32::NAN,
            margin: f32::NAN,
            tables: Vec::new(),
            pages: Vec::new(),
            rows: Vec::new(),
            candidates: Vec::new(),
            kept: false,
            fresh: true,
            skip: 0,
            wait: 0,
            lags: false,
            out: Vec::new(),
            met: Vec::new(),
            by_b: Vec::new(),
            by_b_ok: false,
            left: Vec::new(),
            touched: Vec::new(),
            stats: LiveStats::default(),
        }
    }
}

/// Whether a row keeps its fat box: while its (grown) box is inside it,
/// and it is inside the box grown by twice the margin. The second bound,
/// which Box2D's and Rapier's fat boxes don't have, is what lets new
/// candidates be looked for among the order's boxes rather than fat boxes: a
/// fat box is never more than `2 * margin` past its row's box, however the
/// box shrank (a box turning back square).
#[inline(always)]
fn keeps<const D: usize>(fat: &Bounds<D>, b: &Bounds<D>, margin: f32) -> bool {
    contains(fat, b) && contains(&b.grown(2.0 * margin), fat)
}

impl<const D: usize> Pairs<D>
where
    Axes<D>: Dims<D>,
{
    /// This call's pairs, into `out`: as `near_pairs(act, pas, grow)`, which
    /// `afresh` is, called when more than `most` of the active rows moved
    /// (or a side filters by a sparse component, which can change what
    /// passes with no page changing: `Live` refuses such a side, but the
    /// state alone, as benches use it, is still right). `now` is the world's tick.
    ///
    /// Kept out of line: without it, physics's pairs call through `Live` ran
    /// 4% slower than the same call before `Live` (2026-09-27, 2D pile of
    /// 10 000 settled, 110 µs against 106, interleaved runs of both
    /// builds), and with it at the same speed. The work is the same, so
    /// it's where the loop lands, which inlining into a caller as large as
    /// `find_contacts` decides.
    #[inline(never)]
    fn update<'a>(
        &mut self,
        now: u32,
        (act, pas): (&[SideTable<'a>], &[SideTable<'a>]),
        (grow, margin, most): (f32, f32, f32),
        afresh: &mut dyn FnMut() -> Vec<(Entity, Entity)>,
    ) {
        let typed = |t: &SideTable<'a>| -> Side<'a, D> {
            Side { id: t.id, rows: t.rows, order: <Axes<D> as Dims<D>>::pages(t.order).expect("one dimension"), filters: t.filters }
        };
        // Each table once, active if either side has it.
        let mut sides: Vec<(u8, Side<'a, D>)> = Vec::new();
        for (side, list) in [(ACTIVE, act), (PASSIVE, pas)] {
            for t in list {
                if !sides.iter().any(|(_, s)| s.id == t.id) {
                    sides.push((side, typed(t)));
                }
            }
        }
        sides.sort_by_key(|(_, s)| s.id);
        let tables: Vec<(TableId, u8)> = sides.iter().map(|(side, s)| (s.id, *side)).collect();
        let filtered = sides.iter().any(|(_, s)| !s.filters.is_empty());
        // Whether a row left any table since the last call: only then are
        // walked pages asked which rows they lost.
        let left_any = act.iter().chain(pas).any(|t| t.left > self.tick);
        let same = grow.to_bits() == self.grow.to_bits() && margin.to_bits() == self.margin.to_bits();
        if filtered || tables != self.tables || !same {
            *self = Pairs { tables, grow, margin, call: self.call, ..Pairs::default() };
        }
        if filtered {
            self.tick = now;
            self.out = afresh();
            self.stats = LiveStats { found: self.out.len(), how: How::Afresh, ..LiveStats::default() };
            return;
        }
        self.lags = self.skip > 0;
        if self.skip > 0 {
            self.skip -= 1;
            self.out = afresh();
            self.stats = LiveStats { found: self.out.len(), how: How::Afresh, ..LiveStats::default() };
            return;
        }
        let t0 = std::time::Instant::now();
        self.call += 1;
        let call = self.call;
        let fresh = std::mem::take(&mut self.fresh);
        let (mut walked, mut moved, mut changed) = (0, 0, fresh);
        // Pages with rows that moved, and which: (table, page, rows).
        let mut moving: Vec<(usize, usize, u32)> = Vec::new();
        let Pairs { pages, rows, left, touched, tick, .. } = self;
        pages.resize(sides.len(), Vec::new());
        left.clear();
        touched.clear();
        for (ti, (side, t)) in sides.iter().enumerate() {
            let (side, order, pages) = (*side, t.order, &mut pages[ti]);
            pages.resize(t.rows.len(), PageRows::EMPTY);
            for (p, page) in t.rows.iter().enumerate() {
                if !fresh && order.changed[p] <= *tick {
                    continue;
                }
                let was = &mut pages[p];
                if left_any {
                    left.extend_from_slice(&was.index[..was.len as usize]);
                }
                let lanes = &order.lanes[p];
                if let Some(most) = page.iter().map(|e| e.index as usize + 1).max()
                    && rows.len() < most
                {
                    rows.resize(most, Row::default());
                }
                // Each row's record touched first, all at once: the records
                // are by entity index, at random to the page, and out of
                // cache after the rest of a step; loads that don't wait on
                // each other overlap, where the walk's branches keep them
                // apart (2026-09-27: 19 ns a row to 16 in 3D, 10 000 settled).
                let touch = page.iter().fold(0u32, |t, e| t ^ rows[e.index as usize].generation);
                std::hint::black_box(touch);
                let mut mask = 0u32;
                for (r, &e) in page.iter().enumerate() {
                    let i = e.index as usize;
                    let b = lanes.grown(r, grow);
                    let x = &mut rows[i];
                    x.seen = call;
                    let arrived = x.side != side || x.generation != e.generation;
                    if !arrived && x.grown == b {
                        continue;
                    }
                    changed = true;
                    x.grown = b;
                    if arrived || !keeps(&x.fat, &b, margin) {
                        x.fat = b.grown(margin);
                        x.moved = call;
                        mask |= 1 << r;
                    } else {
                        touched.push(i as u32);
                    }
                    (x.side, x.generation) = (side, e.generation);
                }
                walked += page.len();
                moved += mask.count_ones() as usize;
                if mask != 0 {
                    moving.push((ti, p, mask));
                }
                was.len = page.len() as u8;
                was.index[..page.len()].copy_from_slice(&lanes.index[..page.len()]);
            }
        }
        // A row that left a page it was on and is on none now is gone: moved
        // out of the tables, or despawned.
        let mut gone = false;
        for &i in left.iter() {
            let x = &mut rows[i as usize];
            if x.seen != call && x.side != GONE {
                x.side = GONE;
                gone = true;
            }
        }
        changed |= gone;
        *tick = now;
        let active: usize = sides.iter().filter(|(s, _)| *s == ACTIVE).map(|(_, t)| t.rows.iter().map(Vec::len).sum::<usize>()).sum();
        let t1 = std::time::Instant::now();
        let ns = [(t1 - t0).as_nanos() as u64, 0, 0];
        let stats = |how, candidates, found| LiveStats { walked, moved, candidates, found, how, ns };
        if !changed && self.kept {
            self.stats = stats(How::Same, self.candidates.len(), self.out.len());
            return;
        }
        if moved as f32 > most * active as f32 {
            self.out = afresh();
            self.kept = false;
            // Not after starting over, when everything arrived.
            if !fresh {
                self.wait = (self.wait * 2).clamp(1, 16);
                self.skip = self.wait;
            }
            self.stats = stats(How::Afresh, 0, self.out.len());
            return;
        }
        // A fat box is at most `2 * margin` past its row's grown box
        // (`keeps`), so candidates are among the pairs whose boxes grown by
        // `grow + 2 * margin` meet, less what rounding takes: a few ulps of
        // the coordinates, allowed for here.
        let extent = sides
            .iter()
            .flat_map(|(_, t)| t.order.runs.iter().chain(&t.order.bounds))
            .fold(0.0f32, |m, b| (0..D).fold(m, |m, a| if b.min[a] <= b.max[a] { m.max(b.min[a].abs()).max(b.max[a].abs()) } else { m }));
        let wide = grow + 2.0 * margin + 8.0 * f32::EPSILON * extent;
        let how = if self.kept { How::Kept } else { How::Rebuilt };
        self.wait = 0;
        let found = if self.kept && moving.is_empty() {
            Vec::new()
        } else if self.kept {
            // Rows that moved, against every row.
            // Each page with the box around its rows that moved, not its
            // own: a page of one moving row meets far fewer.
            let swept = moving.iter().map(|&(t, p, mask)| {
                let lanes = &sides[t].1.order.lanes[p];
                let mut around = Bounds::EMPTY;
                let mut m = mask;
                while m != 0 {
                    around = around.union(&lanes.get(m.trailing_zeros() as usize));
                    m &= m - 1;
                }
                (lanes, mask, around.grown(wide))
            });
            let all: Vec<&Side<D>> = sides.iter().map(|(_, s)| s).collect();
            search(swept.collect(), &all, wide)
        } else {
            // Every active row, against each other and the passive rows.
            let mut swept = Vec::new();
            for (_, t) in sides.iter().filter(|(s, _)| *s == ACTIVE) {
                for (p, rows) in t.rows.iter().enumerate().filter(|(_, r)| !r.is_empty()) {
                    swept.push((&t.order.lanes[p], u32::MAX >> (32 - rows.len()), t.order.bounds[p].grown(wide)));
                }
            }
            let passive: Vec<&Side<D>> = sides.iter().filter(|(s, _)| *s == PASSIVE).map(|(_, s)| s).collect();
            search(swept, &passive, wide)
        };
        let t2 = std::time::Instant::now();
        let Pairs { rows, candidates, out, met, by_b, touched, .. } = self;
        let ends = |k: u64| ((k >> 32) as usize, (k & 0xffff_ffff) as usize);
        let pair = |(a, b): (usize, usize)| {
            let (x, y) = (&rows[a], &rows[b]);
            (Entity { index: a as u32, generation: x.generation }, Entity { index: b as u32, generation: y.generation })
        };
        if self.kept && moved == 0 && !gone && touched.len() * FEW < candidates.len() {
            // Nothing moved or went, and few rows changed: only their
            // candidates are retested, and the pairs made again only if one
            // of them came to meet or stopped.
            if !self.by_b_ok {
                by_greater(candidates, rows.len(), by_b);
                self.by_b_ok = true;
            }
            let mut flipped = false;
            for &x in touched.iter() {
                let x = x as u64;
                let lesser = candidates.partition_point(|&k| k >> 32 < x)..candidates.partition_point(|&k| k >> 32 <= x);
                let greater = by_b.partition_point(|&c| candidates[c as usize] & 0xffff_ffff < x)
                    ..by_b.partition_point(|&c| candidates[c as usize] & 0xffff_ffff <= x);
                for c in lesser.chain(by_b[greater].iter().map(|&c| c as usize)) {
                    let (a, b) = ends(candidates[c]);
                    let now = meets(&rows[a].grown, &rows[b].grown);
                    flipped |= now != met[c];
                    met[c] = now;
                }
            }
            if flipped {
                out.clear();
                out.extend(candidates.iter().zip(met.iter()).filter(|(_, m)| **m).map(|(&k, _)| pair(ends(k))));
            }
            self.stats = stats(How::Few, candidates.len(), out.len());
            self.stats.ns = [ns[0], (t2 - t1).as_nanos() as u64, t2.elapsed().as_nanos() as u64];
            return;
        }
        out.clear();
        if self.kept && moved == 0 && !gone {
            // Nothing moved or went: the candidates are as they were, and
            // only which of them meet may have changed.
            for (&k, m) in candidates.iter().zip(met.iter_mut()) {
                let (a, b) = ends(k);
                *m = meets(&rows[a].grown, &rows[b].grown);
                if *m {
                    out.push(pair((a, b)));
                }
            }
            self.stats = stats(how, candidates.len(), out.len());
            self.stats.ns = [ns[0], (t2 - t1).as_nanos() as u64, t2.elapsed().as_nanos() as u64];
            return;
        }
        // Neither gone, and one active: pairs of two passive rows aren't looked for.
        let live = |a: &Row<D>, b: &Row<D>| a.side != GONE && b.side != GONE && (a.side == ACTIVE || b.side == ACTIVE);
        let new: Vec<u64> = found
            .into_iter()
            .filter(|&k| {
                let (a, b) = ends(k);
                live(&rows[a], &rows[b]) && meets(&rows[a].fat, &rows[b].fat)
            })
            .collect();
        let old = if self.kept { std::mem::take(candidates) } else { Vec::new() };
        candidates.clear();
        candidates.reserve(old.len() + new.len());
        met.clear();
        self.by_b_ok = false;
        // Each candidate once, in order: kept unless an end is gone, both are
        // passive, or one moved and their fat boxes no longer meet; a pair if
        // their boxes meet.
        let mut visit = |k: u64| {
            let (a, b) = ends(k);
            let (x, y) = (&rows[a], &rows[b]);
            if !live(x, y) || ((x.moved == call || y.moved == call) && !meets(&x.fat, &y.fat)) {
                return;
            }
            candidates.push(k);
            let m = meets(&x.grown, &y.grown);
            met.push(m);
            if m {
                out.push(pair((a, b)));
            }
        };
        // The new are few (those of what moved), so merged in as the old
        // are walked.
        let mut j = 0;
        for &k in old.iter() {
            while j < new.len() && new[j] < k {
                visit(new[j]);
                j += 1;
            }
            j += (j < new.len() && new[j] == k) as usize;
            visit(k);
        }
        new[j..].iter().for_each(|&k| visit(k));
        self.kept = true;
        self.stats = stats(how, self.candidates.len(), self.out.len());
        self.stats.ns = [ns[0], (t2 - t1).as_nanos() as u64, t2.elapsed().as_nanos() as u64];
    }

    /// Checks what's kept against the tables now: every row's side, box and
    /// fat box (holding the box, and within `2 * margin` of it), nothing kept
    /// of a row no longer there, and, if candidates are kept, that they're
    /// exactly the live pairs whose fat boxes meet (by brute force).
    fn check(&self, act: &[SideTable<'_>], pas: &[SideTable<'_>]) -> Result<(), String> {
        if self.lags {
            return Ok(());
        }
        let mut seen: Vec<(usize, u8, Entity)> = Vec::new();
        for (side, list) in [(ACTIVE, act), (PASSIVE, pas)] {
            for t in list {
                let order = <Axes<D> as Dims<D>>::pages(t.order).expect("one dimension");
                for (p, page) in t.rows.iter().enumerate() {
                    for (r, &e) in page.iter().enumerate() {
                        let i = e.index as usize;
                        if let Some(x) = seen.iter_mut().find(|x| x.0 == i) {
                            x.1 = x.1.min(side);
                            continue;
                        }
                        seen.push((i, side, e));
                        let (x, b) = (self.rows.get(i).copied().unwrap_or_default(), order.lanes[p].grown(r, self.grow));
                        if x.generation != e.generation || x.grown != b || !keeps(&x.fat, &b, self.margin) {
                            return Err(format!(
                                "{e:?} is kept as {:?} / {:?} (fat {:?}), and its box is {b:?}",
                                x.generation, x.grown, x.fat
                            ));
                        }
                    }
                }
            }
        }
        for &(i, side, e) in &seen {
            if self.rows[i].side != side {
                return Err(format!("{e:?} is kept on side {}, and is on {side}", self.rows[i].side));
            }
        }
        let there: std::collections::HashSet<usize> = seen.iter().map(|x| x.0).collect();
        if let Some(i) = (0..self.rows.len()).find(|i| self.rows[*i].side != GONE && !there.contains(i)) {
            return Err(format!("index {i} is kept on side {}, and is on none", self.rows[i].side));
        }
        if !self.kept {
            return Ok(());
        }
        let mut want = Vec::new();
        for (k, &(a, sa, _)) in seen.iter().enumerate() {
            for &(b, sb, _) in &seen[k + 1..] {
                if (sa == ACTIVE || sb == ACTIVE) && meets(&self.rows[a].fat, &self.rows[b].fat) {
                    want.push(((a.min(b) as u64) << 32) | a.max(b) as u64);
                }
            }
        }
        want.sort_unstable();
        if self.met.len() != self.candidates.len() {
            return Err(format!("{} candidates, and whether {} of them meet", self.candidates.len(), self.met.len()));
        }
        let ends = |k: u64| ((k >> 32) as usize, (k & 0xffff_ffff) as usize);
        let meet = |k: u64| meets(&self.rows[ends(k).0].grown, &self.rows[ends(k).1].grown);
        if let Some(c) = self.candidates.iter().zip(&self.met).position(|(&k, &m)| m != meet(k)) {
            return Err(format!("candidate {c}'s meeting is kept wrong"));
        }
        if want != self.candidates {
            let missing = want.iter().filter(|k| self.candidates.binary_search(k).is_err()).count();
            let extra = self.candidates.iter().filter(|k| want.binary_search(k).is_err()).count();
            return Err(format!("candidates: {missing} missing, {extra} extra, of {}", want.len()));
        }
        Ok(())
    }
}

/// Fewer changed rows than a `FEW`th of the candidates, and a call retests
/// only theirs: finding a row's candidates is two searches from each end,
/// about as long as testing a hundred candidates in order.
const FEW: usize = 64;

/// `by_b` made the positions of `candidates` ordered by their greater end,
/// then by position, by counting: indices are below `indices`.
fn by_greater(candidates: &[u64], indices: usize, by_b: &mut Vec<u32>) {
    let mut start = vec![0u32; indices + 1];
    for &k in candidates {
        start[(k & 0xffff_ffff) as usize + 1] += 1;
    }
    for i in 1..start.len() {
        start[i] += start[i - 1];
    }
    by_b.clear();
    by_b.resize(candidates.len(), 0);
    for (c, &k) in candidates.iter().enumerate() {
        let b = (k & 0xffff_ffff) as usize;
        by_b[start[b] as usize] = c as u32;
        start[b] += 1;
    }
}

/// Every pair of a row in `swept` (pages, each with the rows to look for and
/// its box grown) and a row of `swept` or `units` whose boxes, grown by
/// `grow`, meet: `near_pairs`' sweep and passive side, over the rows given.
/// A row both swept and in a unit is paired twice, and with itself, so
/// pairs are made unique after. As `pair_of` keys, sorted.
fn search<const D: usize>(mut swept: Vec<SweptPage<'_, D>>, units: &[&Side<'_, D>], grow: f32) -> Vec<u64> {
    swept.sort_unstable_by(|a, b| a.2.min[0].total_cmp(&b.2.min[0]));
    let widest = swept.iter().fold(0.0f32, |w, p| w.max(p.2.max[0] - p.2.min[0]));
    let mut keys: Vec<u64> = Vec::new();
    for i in 0..swept.len() {
        sweep(&mut keys, &swept, i, grow);
    }
    for t in units {
        for unit in Unit::of(t) {
            meet_unit(&mut keys, &mut |_| {}, (&swept, widest, grow), t, unit);
        }
    }
    keys.sort_unstable();
    keys.dedup();
    keys.retain(|&k| k >> 32 != k & 0xffff_ffff);
    keys
}

/// The share of active rows leaving their fat boxes in a call above which
/// it finds its pairs afresh: looking for new candidates costs about 0.4
/// µs a row moving in 2D and 1.1 in 3D (`live_bench`), where `near_pairs`
/// afresh is about 0.04 and 0.2 a row, so the two meet near a tenth.
pub const MOST: f32 = 0.1;

/// The pairs one proximity relation keeps between calls, in whichever
/// dimensions its tables are: what `Live` holds, the world's (one per
/// relation, `World::live_pairs`).
#[derive(Default)]
pub struct LivePairs {
    plane: Pairs<2>,
    space: Pairs<3>,
    dims: usize,
}

impl LivePairs {
    /// `near_pairs_with(workers, active, passive, grow)`, bit for bit, from
    /// what changed since the last call, with fat boxes `margin` past each
    /// row's box grown. `now` is the world's tick while the sides hold their
    /// guards (`Query::now`); `most` is `MOST` but in benches. Sides that
    /// differ from the last call's, or another `grow` or `margin`, start
    /// over: `Live` always passes its relation's, so that's only after a
    /// reload changed them.
    pub fn near_pairs(
        &mut self,
        now: u32,
        workers: &Workers,
        (active, passive): (&impl NearSide, &impl NearSide),
        how: (f32, f32, f32),
    ) -> &[(Entity, Entity)] {
        self.near_pairs_of(now, workers, (active, passive), how, |_, _| {})
    }

    /// `near_pairs`, with `check` shown the sides' tables first.
    fn near_pairs_of(
        &mut self,
        now: u32,
        workers: &Workers,
        (active, passive): (&impl NearSide, &impl NearSide),
        (grow, margin, most): (f32, f32, f32),
        check: impl FnOnce(&[SideTable<'_>], &[SideTable<'_>]),
    ) -> &[(Entity, Entity)] {
        let (mut act, mut pas) = (Vec::new(), Vec::new());
        active.spatial_tables(&mut act);
        passive.spatial_tables(&mut pas);
        check(&act, &pas);
        let dims = act.iter().chain(&pas).map(|t| t.order.dims()).next().unwrap_or(2);
        assert!(act.iter().chain(&pas).all(|t| t.order.dims() == dims), "a broadphase is over tables of one dimension");
        self.dims = dims;
        let mut afresh = || near_pairs_with(workers, active, passive, grow);
        let how = (grow, margin, most);
        if dims == 3 {
            self.space.update(now, (&act, &pas), how, &mut afresh);
            &self.space.out
        } else {
            self.plane.update(now, (&act, &pas), how, &mut afresh);
            &self.plane.out
        }
    }

    /// What the last call did.
    pub fn stats(&self) -> LiveStats {
        if self.dims == 3 { self.space.stats } else { self.plane.stats }
    }

    /// Checks what's kept for the sides of the last call: for tests.
    #[doc(hidden)]
    pub fn check(&self, active: &impl NearSide, passive: &impl NearSide) -> Result<(), String> {
        let (mut act, mut pas) = (Vec::new(), Vec::new());
        active.spatial_tables(&mut act);
        passive.spatial_tables(&mut pas);
        if self.dims == 3 { self.space.check(&act, &pas) } else { self.plane.check(&act, &pas) }
    }
}

/// The margin a fat box is past its row's box, unless a relation says
/// otherwise: the smallest measured, which was the cheapest, since a
/// settled pile creeps less than any margin tried (spatial-storage.md,
/// "Keeping pairs", decision 1).
pub const MARGIN: f32 = 0.02;

/// A proximity relation: the pairs of rows of spatial key `Key` whose
/// boxes, grown by `GROW`, meet, at least one of them `Active`. A
/// declaration of *what* is related; a system takes it as `Live<R>`, which
/// is *how* it's kept. The type names the relation, and each relation is
/// kept apart from every other, on its key or another. See
/// docs/architecture/live.md.
///
/// A side is a filter, or several as `AnyOf<(..)>`: the tables that match
/// any of them. Each must require `Key`, and none may name a sparse
/// component; a system that takes the relation is refused otherwise.
pub trait Proximity: 'static {
    type Key: Component;
    type Active: Tables;
    type Passive: Tables;
    /// How far each box is grown before they're tested: `near_pairs`' grow.
    const GROW: f32;
    /// How far past its grown box a row's fat box is: a cost, never the
    /// answer.
    const MARGIN: f32 = MARGIN;
}

/// One side of a proximity relation, as the filters its tables match: a
/// `Filter`, or several (`AnyOf`), whose tables are those matching any.
pub trait Tables: 'static {
    #[doc(hidden)]
    fn filters(d: &mut Declare<'_>, out: &mut Vec<FilterDecl>);
}

impl<F: Filter> Tables for F {
    fn filters(d: &mut Declare<'_>, out: &mut Vec<FilterDecl>) {
        let mut f = FilterDecl::default();
        F::declare(d, &mut f);
        out.push(f);
    }
}

/// A side made of several filters, `AnyOf<(F1, F2, ..)>`: the tables that
/// match any of them. Only a side: a spatial side is a set of tables, where
/// a query's filter is a conjunction over its rows, so a plain tuple of
/// filters stays what it is everywhere else, their conjunction.
pub struct AnyOf<T>(PhantomData<T>);

macro_rules! any_of {
    ($($f:ident),+) => {
        impl<$($f: Filter),+> Tables for AnyOf<($($f,)+)> {
            fn filters(d: &mut Declare<'_>, out: &mut Vec<FilterDecl>) {
                $(<$f as Tables>::filters(d, out);)+
            }
        }
    };
}

/// No filters: a side with no tables, as a relation with no passive side
/// has. (`()` is the filter that passes every table.)
impl Tables for AnyOf<()> {
    fn filters(_: &mut Declare<'_>, _: &mut Vec<FilterDecl>) {}
}
any_of!(A, B);
any_of!(A, B, C);
any_of!(A, B, C, D);

/// The side queries a `Live` reads its key through, one per filter: what
/// its footprint's reads are, and the guards it holds.
type SideQuery<'w, K> = Query<'w, &'static K>;

/// Proximity relation `R`, kept current as the rows under it change: a
/// system parameter. `pairs()` is `near_pairs(active, passive, R::GROW)`
/// over `R`'s sides, bit for bit, found from what changed since the last
/// call. Taking it is a write of `R` and a read of `R::Key` in the sides'
/// tables, so two systems that take one relation are ordered, a system
/// that only reads the key isn't held up, and different relations, on one
/// key or not, never conflict.
pub struct Live<'w, R: Proximity> {
    pairs: RwLockWriteGuard<'w, LivePairs>,
    world: &'w World,
    key: ComponentId,
    active: Vec<SideQuery<'w, R::Key>>,
    passive: Vec<SideQuery<'w, R::Key>>,
}

impl<R: Proximity> Live<'_, R> {
    /// Every pair of rows whose boxes, grown by `R::GROW`, meet, and at least
    /// one of which is active: `near_pairs(active, passive, R::GROW)`, bit
    /// for bit, lesser entity first, sorted.
    pub fn pairs(&mut self) -> &[(Entity, Entity)] {
        self.pairs_with(&Workers::default())
    }

    /// `pairs`, finding afresh across `workers` when it does.
    pub fn pairs_with(&mut self, workers: &Workers) -> &[(Entity, Entity)] {
        let Live { pairs, world, key, active, passive } = self;
        // A side's tables all hold the key, but one that holds two spatial
        // keys is in the order of the first, which may be another's.
        let check = |act: &[SideTable<'_>], pas: &[SideTable<'_>]| {
            assert!(
                act.iter().chain(pas).all(|t| t.key == *key),
                "{} is {}'s, and a table of its sides is in another key's order",
                std::any::type_name::<R>(),
                world.name(*key)
            )
        };
        let sides = (&active.as_slice(), &passive.as_slice());
        pairs.near_pairs_of(world.current_tick(), workers, sides, (R::GROW, R::MARGIN, MOST), check)
    }

    /// What the last call did.
    pub fn stats(&self) -> LiveStats {
        self.pairs.stats()
    }

    /// Checks what's kept against the sides' tables: for tests.
    #[doc(hidden)]
    pub fn check(&self) -> Result<(), String> {
        self.pairs.check(&self.active.as_slice(), &self.passive.as_slice())
    }
}

impl<R: Proximity> Param for Live<'static, R> {
    type Item<'w> = Live<'w, R>;

    /// The relation, then a read of the key under each filter, the active
    /// side's first: a group, so the reads are footprints and guards like
    /// any query's, and are refused beside a query that writes the key.
    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        let key = d.component::<R::Key>();
        let (mut active, mut passive) = (Vec::new(), Vec::new());
        R::Active::filters(d, &mut active);
        R::Passive::filters(d, &mut passive);
        // By type name: stable across a mod's builds under the one-compiler
        // rule, where a TypeId isn't promised to be.
        let name = std::any::type_name::<R>();
        let relation = d.world.intern_relation(name);
        let read = |filter: &FilterDecl| {
            let terms = vec![(key, false)];
            ParamDecl::Query(QueryDecl { terms, filter: filter.clone(), changes: ChangeDecl::default(), reorders: false })
        };
        let sides: Vec<ParamDecl> = active.iter().chain(&passive).map(read).collect();
        let live = ParamDecl::Live { relation, name, key, active, passive };
        ParamDecl::Group(std::iter::once(live).chain(sides).collect())
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Live<'w, R> {
        let ParamDecl::Group(members) = decl else { panic!("a live relation's declaration") };
        let Some((ParamDecl::Live { relation, key, active, .. }, sides)) = members.split_first() else {
            panic!("a live relation's declaration")
        };
        let side = |q: &'w ParamDecl| Query::take(cx.world, q.query().expect("a side's read"), cx.log);
        let (act, pas) = sides.split_at(active.len());
        Live {
            pairs: cx.world.live_pairs(*relation).take_write(),
            world: cx.world,
            key: *key,
            active: act.iter().map(side).collect(),
            passive: pas.iter().map(side).collect(),
        }
    }
}

/// Refuses what a live relation would otherwise get wrong in silence, or
/// fail on mid-frame: a side filter on a sparse component (a row can join
/// or leave the side with no page changing, so every call would be
/// afresh), a filter that doesn't require the key (its tables needn't be
/// in the key's order), a key that isn't spatial, and a relation taken
/// twice by one system (two write guards on one lock).
pub(crate) fn check_live(world: &World, name: &str, leaves: &[&ParamDecl]) -> Result<(), String> {
    let mut taken: Vec<usize> = Vec::new();
    for p in leaves {
        let ParamDecl::Live { relation, name: r, key, active, passive } = p else { continue };
        if taken.contains(relation) {
            return Err(format!("{name}: takes {r} twice"));
        }
        taken.push(*relation);
        if !world.component(*key).spatial {
            return Err(format!("{name}: {r}'s key {} isn't a spatial key", world.name(*key)));
        }
        for (side, filters) in [("Active", active), ("Passive", passive)] {
            for (i, f) in filters.iter().enumerate() {
                if let Some(&c) = f.with.iter().chain(&f.without).find(|&&c| world.storage(c) == Storage::Sparse) {
                    return Err(format!(
                        "{name}: {r}'s {side} filter {i} names {}, which is sparse: its rows could change with no page changing",
                        world.name(c)
                    ));
                }
                if !f.with.contains(key) {
                    return Err(format!(
                        "{name}: {r}'s {side} filter {i} doesn't require {}, so it could match tables without it",
                        world.name(*key)
                    ));
                }
            }
        }
    }
    Ok(())
}
