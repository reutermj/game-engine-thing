//! Today's shape: rows in insertion order, and a grid index rebuilt from
//! every row at each upkeep (`publish_index`), with the broadphase building
//! its own grid again (`broad::pairs`).

use physics::Aabb;

use crate::layout::{Layout, Row, Upkeep, Visit, keep};

pub struct Baseline {
    rows: Vec<Row>,
    cell: f32,
    /// (cell x, cell y, row), sorted.
    index: Vec<(i32, i32, u32)>,
}

fn range(cell: f32, b: &Aabb) -> ((i32, i32), (i32, i32)) {
    let c = |v: f32| (v / cell).floor() as i32;
    ((c(b.min.x), c(b.max.x)), (c(b.min.y), c(b.max.y)))
}

fn cells(cell: f32, rows: &[Row], grow: f32, out: &mut Vec<(i32, i32, u32)>) {
    out.clear();
    for (i, r) in rows.iter().enumerate() {
        let ((x0, x1), (y0, y1)) = range(cell, &r.grown(grow));
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                out.push((cx, cy, i as u32));
            }
        }
    }
    out.sort_unstable();
}

impl Baseline {
    pub fn new(mut rows: Vec<Row>) -> Baseline {
        rows.sort_by_key(|r| r.id);
        let mut b = Baseline { rows, cell: 2.0, index: Vec::new() };
        b.upkeep();
        b
    }
}

impl Layout for Baseline {
    fn name(&self) -> String {
        "baseline".into()
    }

    fn get(&self, id: u32) -> &Row {
        &self.rows[id as usize]
    }

    fn get_mut(&mut self, id: u32) -> &mut Row {
        &mut self.rows[id as usize]
    }

    fn each_velocity(&mut self, f: &mut dyn FnMut(&mut Row)) {
        self.rows.iter_mut().for_each(f);
    }

    fn upkeep(&mut self) -> Upkeep {
        cells(self.cell, &self.rows, 0.0, &mut self.index);
        Upkeep::default()
    }

    fn query(&self, region: &Aabb, out: &mut Vec<u32>) -> Visit {
        let ((x0, x1), (y0, y1)) = range(self.cell, region);
        let mut found = Vec::new();
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                let start = self.index.partition_point(|&(x, y, _)| (x, y) < (cx, cy));
                found.extend(self.index[start..].iter().take_while(|&&(x, y, _)| (x, y) == (cx, cy)).map(|e| e.2));
            }
        }
        found.sort_unstable();
        found.dedup();
        let visit = Visit { pages: 0, rows: found.len() };
        out.extend(found.into_iter().map(|i| &self.rows[i as usize]).filter(|r| r.aabb().overlaps(region)).map(|r| r.id));
        visit
    }

    fn pairs(&self, grow: f32, out: &mut Vec<(u32, u32)>) -> Visit {
        let mut grid = Vec::new();
        cells(self.cell, &self.rows, grow, &mut grid);
        let mut visit = Visit { pages: 0, rows: grid.len() };
        let mut start = 0;
        while start < grid.len() {
            let key = (grid[start].0, grid[start].1);
            let end = start + grid[start..].iter().take_while(|e| (e.0, e.1) == key).count();
            for i in start..end {
                for j in i + 1..end {
                    visit.rows += 1;
                    keep(&self.rows[grid[i].2 as usize], &self.rows[grid[j].2 as usize], grow, out);
                }
            }
            start = end;
        }
        out.sort_unstable();
        out.dedup();
        visit
    }

    fn pages(&self) -> usize {
        self.rows.len().div_ceil(256)
    }
}
