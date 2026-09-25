//! Differential test of the storage: random structural changes, layout
//! migrations and column operations applied to the world and to a
//! trivially correct model, compared after every step (the drivers are in
//! `ops.rs`, shared with the fuzzers). Seeded, so a failure names its
//! seed; under Miri, fewer and shorter runs, since each step is
//! interpreted (docs/runbooks/002-run-miri-and-fuzz-the-core.md).

use ecs_ops::Rng;

/// Seeds and steps per seed: natively, and under Miri, where a step costs
/// a thousand times more.
const RUNS: (u64, usize) = if cfg!(miri) { (3, 150) } else { (20, 600) };

#[test]
fn random_changes_match_the_model() {
    for seed in 1..=RUNS.0 {
        ecs_ops::world(&mut Rng::new(seed), RUNS.1, 0);
    }
}

/// Weighted towards spawning, so tables fill and empty pages. Not under
/// Miri, where hundreds of rows a step are too slow: moves between pages
/// are the column driver's there.
#[test]
#[cfg_attr(miri, ignore = "hundreds of rows a step; too slow interpreted")]
fn long_runs_fill_and_empty_several_pages() {
    let most = ecs_ops::world(&mut Rng::new(0xfeed), 5000, 12);
    assert!(most >= 3, "a table filled {most} page(s)");
}

#[test]
fn random_column_operations_match_the_model() {
    for seed in 1..=RUNS.0 {
        ecs_ops::columns(&mut Rng::new(seed), RUNS.1 * 2);
    }
}

/// A long run without Miri's cost: `SEED` and `STEPS` from the environment,
/// for soaking the drivers (the runbook has the command).
#[test]
#[ignore = "a soak, run by hand"]
fn soak() {
    let seed: u64 = std::env::var("SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(1);
    let steps: usize = std::env::var("STEPS").ok().and_then(|s| s.parse().ok()).unwrap_or(100_000);
    ecs_ops::world(&mut Rng::new(seed), steps, 2);
    ecs_ops::columns(&mut Rng::new(seed), steps);
}
