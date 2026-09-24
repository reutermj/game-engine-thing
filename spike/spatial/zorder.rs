//! Layout 2b: rows sorted along a Z-order (Morton) curve of their cell, so
//! each run of `PAGE` rows is a neighborhood, with a box around it. A query
//! tests page boxes, then rows; the broadphase pairs pages whose boxes
//! meet. Upkeep re-sorts after writes move rows' keys.
//!
//! With `big_apart`, bodies bigger than `BIG` stay out of the order, in a
//! list every query and page checks: a floor across the level would
//! otherwise stretch its page's box over everything.

use physics::{Aabb, Vec2};

use crate::layout::{EMPTY, Layout, Row, Upkeep, Visit, keep, union};

const BIG_BIT: u32 = 1 << 31;
/// Pages per run: the level above pages. Pages are in Z-order, so a run of
/// consecutive pages is itself a neighborhood, and boxes over runs are a
/// two-level tree for the price of a union.
const RUN: usize = 16;
/// The key's cell: finer than a body, so neighbors in the key are
/// neighbors in space.
const CELL: f32 = 1.0;

pub struct ZOrder {
    /// Sorted by (key, id).
    rows: Vec<(u64, Row)>,
    big: Vec<Row>,
    big_apart: bool,
    /// Id to index in `rows`, or `BIG_BIT | index` in `big`.
    at: Vec<u32>,
    bounds: Vec<Aabb>,
    moving: Vec<bool>,
    /// Boxes, and whether anything moves, per run of `RUN` pages.
    runs: Vec<Aabb>,
    runs_moving: Vec<bool>,
    dirty: Vec<u32>,
    /// Rows per page: how fine a neighborhood is.
    page: usize,
}

fn spread(v: u32) -> u64 {
    let mut x = v as u64;
    x = (x | (x << 16)) & 0x0000_FFFF_0000_FFFF;
    x = (x | (x << 8)) & 0x00FF_00FF_00FF_00FF;
    x = (x | (x << 4)) & 0x0F0F_0F0F_0F0F_0F0F;
    x = (x | (x << 2)) & 0x3333_3333_3333_3333;
    (x | (x << 1)) & 0x5555_5555_5555_5555
}

/// The Morton key of a position's cell. Offset so negative cells sort too.
pub fn key(p: Vec2) -> u64 {
    let c = |v: f32| ((v / CELL).floor() as i64 + (1 << 20)).clamp(0, u32::MAX as i64) as u32;
    spread(c(p.x)) | (spread(c(p.y)) << 1)
}

impl ZOrder {
    pub fn new(all: Vec<Row>, big_apart: bool, page: usize) -> ZOrder {
        let n = all.len();
        let (big, rows): (Vec<Row>, Vec<Row>) = all.into_iter().partition(|r| big_apart && r.big());
        let mut z = ZOrder {
            rows: rows.into_iter().map(|r| (key(r.pos), r)).collect(),
            big,
            big_apart,
            at: vec![0; n],
            bounds: Vec::new(),
            moving: Vec::new(),
            runs: Vec::new(),
            runs_moving: Vec::new(),
            dirty: Vec::new(),
            page,
        };
        z.rows.sort_by_key(|(k, r)| (*k, r.id));
        z.reindex();
        z.rebound_all();
        z
    }

    fn reindex(&mut self) {
        for (i, (_, r)) in self.rows.iter().enumerate() {
            self.at[r.id as usize] = i as u32;
        }
        for (i, r) in self.big.iter().enumerate() {
            self.at[r.id as usize] = BIG_BIT | i as u32;
        }
    }

    fn rebound(&mut self, page: usize) {
        let rows = &self.rows[page * self.page..((page + 1) * self.page).min(self.rows.len())];
        self.bounds[page] = rows.iter().fold(EMPTY, |b, (_, r)| union(b, r.aabb()));
        self.moving[page] = rows.iter().any(|(_, r)| r.moves());
    }

    fn rebound_all(&mut self) {
        let pages = self.rows.len().div_ceil(self.page);
        self.bounds = vec![EMPTY; pages];
        self.moving = vec![false; pages];
        for p in 0..pages {
            self.rebound(p);
        }
        let runs = pages.div_ceil(RUN);
        self.runs = vec![EMPTY; runs];
        self.runs_moving = vec![false; runs];
        for r in 0..runs {
            self.rebound_run(r);
        }
    }

    fn rebound_run(&mut self, run: usize) {
        let pages = run * RUN..((run + 1) * RUN).min(self.bounds.len());
        self.runs[run] = self.bounds[pages.clone()].iter().fold(EMPTY, |b, p| union(b, *p));
        self.runs_moving[run] = self.moving[pages].iter().any(|&m| m);
    }

    fn pages_of(&self, run: usize) -> std::ops::Range<usize> {
        run * RUN..((run + 1) * RUN).min(self.bounds.len())
    }

    fn page_rows(&self, p: usize) -> &[(u64, Row)] {
        &self.rows[p * self.page..((p + 1) * self.page).min(self.rows.len())]
    }
}

fn grown(b: &Aabb, by: f32) -> Aabb {
    let g = Vec2::new(by, by);
    Aabb { min: b.min - g, max: b.max + g }
}

impl Layout for ZOrder {
    fn name(&self) -> String {
        format!("zorder{}/{}", if self.big_apart { "+big" } else { "" }, self.page)
    }

    fn get(&self, id: u32) -> &Row {
        let at = self.at[id as usize];
        if at & BIG_BIT != 0 { &self.big[(at & !BIG_BIT) as usize] } else { &self.rows[at as usize].1 }
    }

    fn get_mut(&mut self, id: u32) -> &mut Row {
        self.dirty.push(id);
        let at = self.at[id as usize];
        if at & BIG_BIT != 0 { &mut self.big[(at & !BIG_BIT) as usize] } else { &mut self.rows[at as usize].1 }
    }

    fn each_velocity(&mut self, f: &mut dyn FnMut(&mut Row)) {
        self.rows.iter_mut().for_each(|(_, r)| f(r));
        self.big.iter_mut().for_each(f);
    }

    fn upkeep(&mut self) -> Upkeep {
        if self.dirty.is_empty() {
            return Upkeep::default();
        }
        // Every written key first, then the order around each: checked
        // against a neighbor not yet rekeyed, a row could look in order and
        // not be.
        let written: Vec<usize> = std::mem::take(&mut self.dirty)
            .into_iter()
            .map(|id| self.at[id as usize])
            .filter(|at| at & BIG_BIT == 0)
            .map(|at| at as usize)
            .collect();
        for &i in &written {
            self.rows[i].0 = key(self.rows[i].1.pos);
        }
        let order = |i: usize| (self.rows[i].0, self.rows[i].1.id);
        let out_of_order = written
            .iter()
            .any(|&i| (i > 0 && order(i - 1) > order(i)) || (i + 1 < self.rows.len() && order(i + 1) < order(i)));
        let mut pages: Vec<usize> = written.iter().map(|i| i / self.page).collect();
        if !out_of_order {
            pages.sort_unstable();
            pages.dedup();
            for &p in &pages {
                self.rebound(p);
            }
            let mut runs: Vec<usize> = pages.iter().map(|p| p / RUN).collect();
            runs.dedup();
            for r in runs {
                self.rebound_run(r);
            }
            return Upkeep::default();
        }
        let old = self.at.clone();
        // Adaptive: a nearly sorted run costs about one pass.
        self.rows.sort_by_key(|(k, r)| (*k, r.id));
        self.reindex();
        self.rebound_all();
        // Rows that changed page: in paged storage, the ones that move. A
        // shift within a page is a reorder of rows already there.
        let page = self.page as u32;
        let moved = old.iter().zip(&self.at).filter(|(a, b)| a != b && (*a & BIG_BIT != 0 || *a / page != *b / page)).count();
        Upkeep { moved }
    }

    fn query(&self, region: &Aabb, out: &mut Vec<u32>) -> Visit {
        let mut visit = Visit::default();
        let pages = (0..self.runs.len())
            .filter(|&r| self.runs[r].overlaps(region))
            .flat_map(|r| self.pages_of(r))
            .filter(|&p| self.bounds[p].overlaps(region));
        for p in pages {
            visit.pages += 1;
            for (_, r) in self.page_rows(p) {
                visit.rows += 1;
                if r.aabb().overlaps(region) {
                    out.push(r.id);
                }
            }
        }
        for r in &self.big {
            visit.rows += 1;
            if r.aabb().overlaps(region) {
                out.push(r.id);
            }
        }
        visit
    }

    fn pairs(&self, grow: f32, out: &mut Vec<(u32, u32)>) -> Visit {
        let mut visit = Visit::default();

        let boxes: Vec<Aabb> = self.bounds.iter().map(|b| grown(b, grow)).collect();
        let run_boxes: Vec<Aabb> = self.runs.iter().map(|b| grown(b, grow)).collect();
        let runs = self.runs.len();
        for (ra, rb) in (0..runs).flat_map(|a| (a..runs).map(move |b| (a, b))) {
            if !(self.runs_moving[ra] || self.runs_moving[rb]) || !run_boxes[ra].overlaps(&run_boxes[rb]) {
                continue;
            }
            for p in self.pages_of(ra) {
                let from = if ra == rb { p } else { self.pages_of(rb).start };
                for q in from..self.pages_of(rb).end {
                    if !(self.moving[p] || self.moving[q]) || !boxes[p].overlaps(&boxes[q]) {
                        continue;
                    }
                    visit.pages += 1;
                    let (a, b) = (self.page_rows(p), self.page_rows(q));
                    for (i, (_, ra_)) in a.iter().enumerate() {
                        let from = if p == q { i + 1 } else { 0 };
                        for (_, rb_) in &b[from..] {
                            visit.rows += 1;
                            keep(ra_, rb_, grow, out);
                        }
                    }
                }
            }
        }
        for (i, big) in self.big.iter().enumerate() {
            let g = big.grown(grow);
            let near = (0..runs).filter(|&r| run_boxes[r].overlaps(&g)).flat_map(|r| self.pages_of(r));
            for p in near {
                if !(big.moves() || self.moving[p]) || !boxes[p].overlaps(&g) {
                    continue;
                }
                visit.pages += 1;
                for (_, r) in self.page_rows(p) {
                    visit.rows += 1;
                    keep(big, r, grow, out);
                }
            }
            for other in &self.big[i + 1..] {
                visit.rows += 1;
                keep(big, other, grow, out);
            }
        }
        out.sort_unstable();
        out.dedup();
        visit
    }

    fn pages(&self) -> usize {
        self.bounds.len() + usize::from(self.big_apart && !self.big.is_empty())
    }
}
