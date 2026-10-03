//! 3D scenes at the edge of stability, judged as families, as 2D's
//! (`//engine/std/physics2d/compare`, `family.rs`; physics-testing.md,
//! "Families at the edge"): a grid of one scene across what decides it (a
//! stack's height, a pyramid's base and friction, a cabin of planks on
//! edge, a card house's lean and friction), each run a yes or a no
//! (`good`), and a family by its share of yeses and the median of when its
//! runs came to rest. Near its edge a scene goes either way on rounding,
//! so one scene says little; a grid's share, against the references' on
//! the same grid, says which way a change moved it.
//!
//! Measured by `bench -- <backends> --families[=names]` for physics.md's
//! "Colouring the 3D solve (proposed)" (get-emj.90); no test bounds them
//! yet.

use crate::behave::Behaviour;
use crate::scenes::{Edge, Kind};

/// One family: its grid of scenes, each a kind and its n.
pub struct Family {
    pub name: &'static str,
    pub scenes: Vec<(Kind, usize)>,
}

pub fn families() -> Vec<Family> {
    vec![
        Family { name: "stacks", scenes: (8..=40).map(|n| (Kind::Stack, n)).collect() },
        Family {
            name: "pyramids",
            scenes: edges(Kind::Pyramid, &[10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60], &[0.0], &[0.0, 0.05, 0.1, 0.2, 0.5]),
        },
        Family { name: "cabins", scenes: edges(Kind::Cabin, &[8, 12, 16, 20, 24, 28, 32, 36, 40], &[0.0], &[0.2, 0.3, 0.5, 0.8]) },
        Family {
            name: "cards",
            scenes: edges(Kind::Cards, &[3, 4, 5, 6], &[23.0, 24.0, 25.0, 26.0, 27.0], &[0.6, 0.65, 0.7, 0.75, 0.8, 0.9]),
        },
    ]
}

fn edges(kind: Kind, sizes: &[usize], leans: &[f32], mus: &[f32]) -> Vec<(Kind, usize)> {
    let mut v = Vec::new();
    for &size in sizes {
        for &lean in leans {
            v.extend(mus.iter().map(|&mu| (kind, Edge { size, lean, mu }.pack())));
        }
    }
    v
}

/// Whether a run did what its scene should: stood (`behave.rs`'s
/// `stands`).
pub fn good(b: &Behaviour) -> bool {
    b.get("stands") == 1.0
}

/// Yes or no on each run, how many yeses, and the median step the runs
/// came to rest from (`NEVER` counted as never).
pub fn share(runs: &[Behaviour]) -> (usize, String, f64) {
    let marks: String = runs.iter().map(|b| if good(b) { 'S' } else { '.' }).collect();
    let rest = physics_testkit::stats::median(runs.iter().map(|b| b.get("at rest from")).collect());
    (marks.matches('S').count(), marks, rest)
}
