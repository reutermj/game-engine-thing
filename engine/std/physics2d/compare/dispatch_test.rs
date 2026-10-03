//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): the spike's
//! dispatchers run every block of a program once, each stage after the
//! last, and so give the 2D mod's one-thread solve bit for bit at every
//! thread count; and the checks see a dispatcher that doesn't.

#[allow(dead_code)]
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod dispatch;
#[allow(dead_code)]
mod dispatch_2d;
#[allow(dead_code)]
mod ecs;
#[allow(dead_code)]
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
#[path = "../tests/pool.rs"]
mod pool;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
#[allow(dead_code)]
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::Instant;

pub use sim::{Dyn, Sim};

use dispatch::{Plant, Program, Protocol};
use dispatch_2d::{How, Input, Solve, bits};
use engine_ecs::Executor;
use scene::Scene;

/// Small scenes, turning: a settled pile, a pyramid, a falling pile.
fn inputs() -> &'static [Input] {
    static INPUTS: OnceLock<Vec<Input>> = OnceLock::new();
    INPUTS.get_or_init(|| {
        let pile = Scene::Pile { n: 1000, width: 41.0, stagger: true };
        let mut all = dispatch_2d::capture(&pile, &[401, 430]);
        all.extend(dispatch_2d::capture(&Scene::Pyramid { base: 20 }, &[601]));
        all.extend(dispatch_2d::capture(&pile, &[10, 20]));
        all
    })
}

fn base(i: &Input) -> Vec<u32> {
    let mut o = i.clone();
    Solve::default().solve(&mut o, How::One, None);
    bits(&o)
}

/// Kept threads, spawned threads, and threads that come one at a time,
/// the last first.
fn executors() -> Vec<(String, Arc<dyn Executor>)> {
    let mut out: Vec<(String, Arc<dyn Executor>)> = Vec::new();
    for n in [1, 2, 4, 8] {
        out.push((format!("kept {n}"), Arc::new(pool::Pool::new(n))));
    }
    out.push(("spawned 4".into(), Arc::new(engine_ecs::Scoped(4))));
    out.push(("late 4".into(), Arc::new(variants::Backwards(4))));
    out
}

fn solved(i: &Input, how: How) -> Vec<u32> {
    let mut o = i.clone();
    Solve::default().solve(&mut o, how, None);
    bits(&o)
}

#[test]
fn every_dispatcher_is_the_one_thread_solve_bit_for_bit() {
    let inputs = inputs();
    assert!(inputs.iter().all(|i| dispatch_2d::shareable(&i.bodies, &i.spinning)), "the scenes are shareable");
    for (name, exec) in executors() {
        for p in Protocol::ALL {
            for (k, i) in inputs.iter().enumerate() {
                assert!(solved(i, How::Across(p, &*exec, Plant::None)) == base(i), "{} on {name}, input {k}", p.name());
            }
        }
    }
}

/// get-znt.39: where a still body carries a negative zero, the passes stay
/// on one thread, and are the one-thread solve.
#[test]
fn a_step_that_isnt_shareable_is_solved_on_one_thread() {
    let mut i = inputs()[0].clone();
    let still = i.bodies.last_mut().unwrap();
    assert_eq!(still.inv_mass, 0.0, "the last body stands still");
    still.v = physics2d::Vec2::new(-0.0, -0.0);
    assert!(!dispatch_2d::shareable(&i.bodies, &i.spinning));
    let exec = pool::Pool::new(8);
    for p in Protocol::ALL {
        assert!(solved(&i, How::Across(p, &exec, Plant::None)) == base(&i), "{}", p.name());
    }
}

/// The check above sees a block run twice, on any number of threads.
#[test]
fn a_dispatcher_that_runs_a_block_twice_fails_the_check() {
    let i = &inputs()[0];
    for n in [1, 4] {
        let exec = pool::Pool::new(n);
        for p in Protocol::ALL {
            assert!(solved(i, How::Across(p, &exec, Plant::Twice)) != base(i), "{} at {n}", p.name());
        }
    }
}

/// And a stage let start before the last one is done: across threads, where
/// it can overlap, a differing result or a block taken twice at once (its
/// lock) in some of a few tries.
#[test]
fn a_dispatcher_that_starts_a_stage_early_fails_the_check() {
    let i = &inputs()[0];
    let want = base(i);
    let exec = pool::Pool::new(4);
    for p in Protocol::ALL {
        let caught = (0..50).any(|_| {
            let got = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solved(i, How::Across(p, &exec, Plant::Early))));
            got.map_or(true, |got| got != want)
        });
        assert!(caught, "{}: an early stage went unseen", p.name());
    }
}

/// The dispatchers alone, over plain counters: every block of every stage
/// once, none before the stage before it is done, with two programs in one
/// dispatch; and the counters see both planted bugs.
#[test]
fn every_block_runs_once_after_the_stage_before() {
    struct Count {
        runs: Vec<AtomicUsize>,
        done: Vec<AtomicUsize>,
        early: AtomicUsize,
    }
    let shape = |p: usize| -> Vec<usize> { (0..40).map(|t| [1, 3, 8, 17, 2][(t + p) % 5]).collect() };
    let check = |protocol: Protocol, exec: &dyn Executor, plant: Plant| -> (usize, usize) {
        let shapes = [shape(0), shape(1)];
        let counts: Vec<Count> = shapes
            .iter()
            .map(|s| Count {
                runs: (0..s.iter().sum()).map(|_| AtomicUsize::new(0)).collect(),
                done: s.iter().map(|_| AtomicUsize::new(0)).collect(),
                early: AtomicUsize::new(0),
            })
            .collect();
        let firsts: Vec<Vec<usize>> =
            shapes.iter().map(|s| s.iter().scan(0, |a, n| Some(std::mem::replace(a, *a + n))).collect()).collect();
        let runs: Vec<Box<dyn Fn(usize, usize) + Sync>> = (0..2)
            .map(|p| {
                let (c, s, first) = (&counts[p], &shapes[p], &firsts[p]);
                Box::new(move |t: usize, b: usize| {
                    if t > 0 && c.done[t - 1].load(Ordering::Acquire) < s[t - 1] {
                        c.early.fetch_add(1, Ordering::Relaxed);
                    }
                    c.runs[first[t] + b].fetch_add(1, Ordering::Relaxed);
                    // The last block slow, so a stage let start early
                    // overlaps it.
                    let t0 = Instant::now();
                    let us = if b + 1 == s[t] { 200 } else { 5 };
                    while t0.elapsed().as_micros() < us {
                        std::hint::spin_loop();
                    }
                    c.done[t].fetch_add(1, Ordering::Release);
                }) as Box<dyn Fn(usize, usize) + Sync>
            })
            .collect();
        let programs: Vec<Program> = (0..2)
            .map(|p| Program {
                stages: shapes[p].iter().zip(&firsts[p]).map(|(&n, &f)| (n, f)).collect(),
                marks: shapes[p].iter().sum(),
                run: &*runs[p],
            })
            .collect();
        dispatch::run(protocol, exec, &programs, None, plant);
        let wrong = counts.iter().flat_map(|c| c.runs.iter()).filter(|r| r.load(Ordering::Relaxed) != 1).count();
        (wrong, counts.iter().map(|c| c.early.load(Ordering::Relaxed)).sum())
    };
    for (name, exec) in executors() {
        for p in Protocol::ALL {
            assert_eq!(check(p, &*exec, Plant::None), (0, 0), "{} on {name}", p.name());
            assert!(check(p, &*exec, Plant::Twice).0 > 0, "{} on {name}: a block run twice unseen", p.name());
        }
    }
    let exec = pool::Pool::new(4);
    for p in Protocol::ALL {
        assert!(check(p, &exec, Plant::Early).1 > 0, "{}: an early stage unseen", p.name());
    }
}
