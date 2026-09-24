//! A spike of space as storage structure (get-emj.14): one table of bodies
//! in three layouts, today's (insertion order and a rebuilt grid index),
//! pages per grid cell (2a), and Z-order pages with bounds (2b), each 2a
//! and 2b with and without big bodies kept apart. They run the same
//! simulation, so they must agree exactly; what differs is what upkeep,
//! the broadphase and region queries cost.

#[path = "../../engine/std/physics/narrow.rs"]
pub mod narrow;
#[path = "../../engine/std/physics/solver.rs"]
pub mod solver;

pub mod baseline;
pub mod grid;
pub mod layout;
pub mod sim;
pub mod zorder;

use layout::Layout;
use sim::Scene;

/// Every layout of `scene`, the baseline first.
pub fn layouts(scene: &Scene) -> Vec<Box<dyn Layout>> {
    let rows = || scene.rows.clone();
    vec![
        Box::new(baseline::Baseline::new(rows())),
        Box::new(grid::Grid::new(rows(), false)),
        Box::new(grid::Grid::new(rows(), true)),
        Box::new(zorder::ZOrder::new(rows(), false, 64)),
        Box::new(zorder::ZOrder::new(rows(), true, 64)),
        Box::new(zorder::ZOrder::new(rows(), true, 16)),
        Box::new(zorder::ZOrder::new(rows(), true, 8)),
    ]
}

#[cfg(test)]
mod tests {
    use physics::Aabb;

    use super::layout::{Row, brute_pairs};
    use super::sim::{self, Sim, Timing};
    use super::*;

    fn rows(l: &dyn Layout, n: usize) -> Vec<Row> {
        (0..n as u32).map(|id| *l.get(id)).collect()
    }

    /// Every layout, stepped in lockstep: the same world after every frame,
    /// broadphases that find exactly the overlapping pairs, and queries that
    /// find exactly the overlapping rows.
    fn agree(scene: Scene, frames: usize) {
        let n = scene.rows.len();
        let mut all = layouts(&scene);
        let mut sims: Vec<Sim> = all.iter().map(|_| Sim::new(&scene)).collect();
        let mut probe = Sim::new(&scene);
        for frame in 0..frames {
            for (l, s) in all.iter_mut().zip(&mut sims) {
                s.step(l.as_mut(), &mut Timing::default());
            }
            let world = rows(all[0].as_ref(), n);
            let pairs = brute_pairs(&world, narrow::MARGIN);
            let regions: Vec<Aabb> = probe.regions();
            for l in &all {
                assert_eq!(rows(l.as_ref(), n), world, "{} diverged by frame {frame}", l.name());
                let mut found = Vec::new();
                l.pairs(narrow::MARGIN, &mut found);
                assert_eq!(found, pairs, "{}'s broadphase on frame {frame}", l.name());
                for r in &regions {
                    let mut got = Vec::new();
                    l.query(r, &mut got);
                    got.sort_unstable();
                    let want: Vec<u32> = world.iter().filter(|w| w.aabb().overlaps(r)).map(|w| w.id).collect();
                    assert_eq!(got, want, "{}'s query on frame {frame}", l.name());
                }
            }
        }
    }

    #[test]
    fn every_layout_runs_the_same_pile() {
        agree(sim::pile(200), 150);
    }

    #[test]
    fn every_layout_runs_the_same_platformer() {
        agree(sim::platformer(), 200);
    }

    #[test]
    fn every_layout_runs_the_same_drift() {
        agree(sim::drift(300), 100);
    }
}
