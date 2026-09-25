//! The reload driver (`reload_ops`) in the default suite: the fuzzer's
//! corpus replayed, so an input that once found a bug keeps checking it,
//! and seeded sessions, which must between them reach every kind of reply
//! the model knows. And, by hand, a long seeded soak (the runbook has the
//! command).

use std::path::Path;

use reload_ops::{Bytes, Rng, TALLIED};

/// Relative to the runfiles root, where the tests run: the corpus is their
/// data.
const CORPUS: &str = "engine/tests/fuzz/corpus";

#[test]
fn corpus_replays() {
    let mut inputs: Vec<_> = std::fs::read_dir(Path::new(CORPUS))
        .unwrap_or_else(|e| panic!("{CORPUS}: {e}"))
        .map(|entry| entry.unwrap().path())
        .collect();
    inputs.sort();
    assert!(!inputs.is_empty(), "a corpus in {CORPUS}");
    for input in inputs {
        eprintln!("{}", input.display());
        reload_ops::run(&mut Bytes::new(&std::fs::read(&input).unwrap()), usize::MAX);
    }
}

/// Seeds and operations per seed: enough that the sessions between them
/// reach every reply `TALLIED` names, which a change to the driver's
/// weights could quietly stop.
const RUNS: (u64, usize) = (12, 250);

#[test]
fn seeded_sessions_match_the_model_and_reach_every_reply() {
    let mut replies: std::collections::HashMap<&str, usize> = Default::default();
    for seed in 1..=RUNS.0 {
        eprintln!("seed {seed}");
        let stats = reload_ops::run(&mut Rng::new(seed), RUNS.1);
        for (reply, n) in stats.replies {
            *replies.entry(reply).or_default() += n;
        }
    }
    let missing: Vec<&str> = TALLIED.iter().copied().filter(|t| !replies.contains_key(t)).collect();
    assert!(missing.is_empty(), "no seeded session replied {missing:?}; replies: {replies:?}");
}

/// A long run without libFuzzer: `SEED` and `STEPS` from the environment,
/// in sessions of `SESSION` operations.
#[test]
#[ignore = "a soak, run by hand"]
fn soak() {
    let var = |name: &str, default: u64| std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default);
    let (seed, steps, session) = (var("SEED", 1), var("STEPS", 100_000), var("SESSION", 500));
    let started = std::time::Instant::now();
    let mut done = 0;
    for n in 0.. {
        if done >= steps {
            break;
        }
        let stats = reload_ops::run(&mut Rng::new(seed.wrapping_mul(1_000_003).wrapping_add(n)), session as usize);
        done += stats.ops as u64;
        eprintln!("session {n} (seed {seed}): {done} operations in {:?}", started.elapsed());
    }
}

/// Sessions written out (`reload_ops::script`): what the fuzzer found,
/// reduced to the operations that matter, one file each.
#[test]
fn scripts_replay() {
    let dir = Path::new("engine/tests/fuzz/scripts");
    // One script from anywhere, to reduce what the fuzzer found by hand.
    let mut scripts: Vec<_> = match std::env::var_os("RELOAD_FUZZ_SCRIPT") {
        Some(one) => vec![one.into()],
        None => std::fs::read_dir(dir).unwrap().map(|e| e.unwrap().path()).collect(),
    };
    scripts.sort();
    for script in scripts {
        eprintln!("{}", script.display());
        reload_ops::script(&std::fs::read_to_string(&script).unwrap());
    }
}
