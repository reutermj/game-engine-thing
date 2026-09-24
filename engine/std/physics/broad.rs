//! Candidate pairs: colliders whose boxes share a cell of a uniform grid,
//! rebuilt every step. Sorted vectors, not hash maps, so the pairs come out
//! in the same order on every run (physics.md, "Broadphase").

use physics::{Aabb, SpatialIndex, Vec2};

use crate::narrow::MARGIN;

/// Pairs `(i, j)`, `i < j`, of `boxes` indices whose boxes, grown by the
/// contact margin, share a cell, and that `wanted` accepts. Each pair once,
/// sorted.
pub fn pairs(cell: f32, boxes: &[Aabb], wanted: impl Fn(usize, usize) -> bool) -> Vec<(u32, u32)> {
    let grow = Vec2::new(MARGIN, MARGIN);
    let mut cells: Vec<(i32, i32, u32)> = Vec::new();
    for (i, b) in boxes.iter().enumerate() {
        let grown = Aabb { min: b.min - grow, max: b.max + grow };
        let ((x0, x1), (y0, y1)) = SpatialIndex::cell_range(cell, &grown);
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                cells.push((cx, cy, i as u32));
            }
        }
    }
    cells.sort_unstable();
    let mut out = Vec::new();
    let mut start = 0;
    while start < cells.len() {
        let key = (cells[start].0, cells[start].1);
        let end = start + cells[start..].iter().take_while(|c| (c.0, c.1) == key).count();
        for i in start..end {
            for j in i + 1..end {
                let (a, b) = (cells[i].2, cells[j].2);
                if wanted(a as usize, b as usize) {
                    let grown = |k: u32| {
                        let b = boxes[k as usize];
                        Aabb { min: b.min - grow, max: b.max + grow }
                    };
                    if grown(a).overlaps(&grown(b)) {
                        out.push((a, b));
                    }
                }
            }
        }
        start = end;
    }
    out.sort_unstable();
    out.dedup();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit(x: f32, y: f32) -> Aabb {
        Aabb { min: Vec2::new(x - 0.5, y - 0.5), max: Vec2::new(x + 0.5, y + 0.5) }
    }

    #[test]
    fn neighbors_pair_once_however_many_cells_they_share() {
        // 0 and 1 straddle the same cell boundary, so share two cells.
        let boxes = [unit(2.0, 2.0), unit(2.9, 2.0), unit(9.0, 9.0)];
        assert_eq!(pairs(2.0, &boxes, |_, _| true), [(0, 1)]);
    }

    #[test]
    fn a_pair_in_one_cell_but_apart_is_no_candidate() {
        let boxes = [unit(0.5, 0.5), unit(0.5, 1.9)];
        assert_eq!(pairs(4.0, &boxes, |_, _| true), [], "one cell, but 0.4 apart");
    }

    #[test]
    fn the_filter_drops_pairs_before_they_are_kept() {
        let boxes = [unit(1.0, 1.0), unit(1.5, 1.0), unit(2.0, 1.0)];
        assert_eq!(pairs(2.0, &boxes, |a, b| a != 0 && b != 0), [(1, 2)]);
    }
}
