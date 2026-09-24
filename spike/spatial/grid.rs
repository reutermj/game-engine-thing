//! Layout 2a: each grid cell has its own pages, holding the rows whose
//! center is in it. A row whose center crosses into another cell moves
//! there at upkeep. A query visits the cells its region (grown by the
//! largest half extent, since a row can reach out of its cell) covers.
//!
//! With `big_apart`, bodies bigger than `BIG` are kept in a list instead:
//! without it, the largest half extent is a floor's, and every query and
//! every cell's neighborhood grows to the whole level.

use std::collections::{BTreeMap, BTreeSet};

use physics::{Aabb, Vec2};

use crate::layout::{EMPTY, Layout, PAGE, Row, Upkeep, Visit, keep, union};

const CELL: f32 = 2.0;

#[derive(Clone, Copy)]
enum Loc {
    Cell((i32, i32), u32),
    Big(u32),
}

pub struct Grid {
    cells: BTreeMap<(i32, i32), Vec<Row>>,
    bounds: BTreeMap<(i32, i32), Aabb>,
    big: Vec<Row>,
    big_apart: bool,
    at: Vec<Loc>,
    /// The largest half extent of a row in a cell: how far a row reaches
    /// past its cell.
    reach: f32,
    dirty: Vec<u32>,
}

fn cell_of(p: Vec2) -> (i32, i32) {
    ((p.x / CELL).floor() as i32, (p.y / CELL).floor() as i32)
}

impl Grid {
    pub fn new(mut all: Vec<Row>, big_apart: bool) -> Grid {
        all.sort_by_key(|r| r.id);
        let mut g = Grid {
            cells: BTreeMap::new(),
            bounds: BTreeMap::new(),
            big: Vec::new(),
            big_apart,
            at: vec![Loc::Big(0); all.len()],
            reach: 0.0,
            dirty: Vec::new(),
        };
        for r in all {
            if big_apart && r.big() {
                g.at[r.id as usize] = Loc::Big(g.big.len() as u32);
                g.big.push(r);
            } else {
                g.reach = g.reach.max(r.half.x.max(r.half.y));
                let c = cell_of(r.pos);
                let rows = g.cells.entry(c).or_default();
                g.at[r.id as usize] = Loc::Cell(c, rows.len() as u32);
                rows.push(r);
            }
        }
        let cells: Vec<_> = g.cells.keys().copied().collect();
        for c in cells {
            g.rebound(c);
        }
        g
    }

    fn rebound(&mut self, c: (i32, i32)) {
        match self.cells.get(&c) {
            Some(rows) if !rows.is_empty() => {
                self.bounds.insert(c, rows.iter().fold(EMPTY, |b, r| union(b, r.aabb())));
            }
            _ => {
                self.cells.remove(&c);
                self.bounds.remove(&c);
            }
        }
    }

    /// The cells whose rows could reach `region`.
    fn around(&self, region: &Aabb, extra: f32) -> impl Iterator<Item = (&(i32, i32), &Aabb)> {
        let r = self.reach + extra;
        let lo = cell_of(region.min - Vec2::new(r, r));
        let hi = cell_of(region.max + Vec2::new(r, r));
        (lo.0..=hi.0).flat_map(move |cx| self.bounds.range((cx, lo.1)..=(cx, hi.1)))
    }
}

fn grown(b: &Aabb, by: f32) -> Aabb {
    let g = Vec2::new(by, by);
    Aabb { min: b.min - g, max: b.max + g }
}

impl Layout for Grid {
    fn name(&self) -> String {
        if self.big_apart { "grid+big".into() } else { "grid".into() }
    }

    fn get(&self, id: u32) -> &Row {
        match self.at[id as usize] {
            Loc::Cell(c, i) => &self.cells[&c][i as usize],
            Loc::Big(i) => &self.big[i as usize],
        }
    }

    fn get_mut(&mut self, id: u32) -> &mut Row {
        self.dirty.push(id);
        match self.at[id as usize] {
            Loc::Cell(c, i) => &mut self.cells.get_mut(&c).unwrap()[i as usize],
            Loc::Big(i) => &mut self.big[i as usize],
        }
    }

    fn each_velocity(&mut self, f: &mut dyn FnMut(&mut Row)) {
        self.cells.values_mut().flatten().for_each(&mut *f);
        self.big.iter_mut().for_each(f);
    }

    fn upkeep(&mut self) -> Upkeep {
        let mut moved = 0;
        let mut touched = BTreeSet::new();
        for id in std::mem::take(&mut self.dirty) {
            let Loc::Cell(from, i) = self.at[id as usize] else { continue };
            let to = cell_of(self.get(id).pos);
            touched.insert(from);
            if to == from {
                continue;
            }
            let rows = self.cells.get_mut(&from).unwrap();
            let row = rows.swap_remove(i as usize);
            if let Some(swapped) = rows.get(i as usize) {
                self.at[swapped.id as usize] = Loc::Cell(from, i);
            }
            let dest = self.cells.entry(to).or_default();
            self.at[id as usize] = Loc::Cell(to, dest.len() as u32);
            dest.push(row);
            touched.insert(to);
            moved += 1;
        }
        for c in touched {
            self.rebound(c);
        }
        Upkeep { moved }
    }

    fn query(&self, region: &Aabb, out: &mut Vec<u32>) -> Visit {
        let mut visit = Visit::default();
        for (c, b) in self.around(region, 0.0) {
            if !b.overlaps(region) {
                continue;
            }
            let rows = &self.cells[c];
            visit.pages += rows.len().div_ceil(PAGE);
            for r in rows {
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
        for (c, b) in &self.bounds {
            let rows = &self.cells[c];
            let gb = grown(b, grow);
            for (d, e) in self.around(b, 2.0 * grow) {
                // Each pair of cells once, and a cell with itself.
                if d < c || !gb.overlaps(&grown(e, grow)) {
                    continue;
                }
                visit.pages += 1;
                let others = &self.cells[d];
                for (i, ra) in rows.iter().enumerate() {
                    let from = if d == c { i + 1 } else { 0 };
                    for rb in &others[from..] {
                        visit.rows += 1;
                        keep(ra, rb, grow, out);
                    }
                }
            }
        }
        for (i, big) in self.big.iter().enumerate() {
            let g = big.grown(grow);
            for (c, b) in self.around(&g, grow) {
                if !grown(b, grow).overlaps(&g) {
                    continue;
                }
                visit.pages += 1;
                for r in &self.cells[c] {
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
        self.cells.values().map(|r| r.len().div_ceil(PAGE)).sum::<usize>() + usize::from(!self.big.is_empty())
    }
}
