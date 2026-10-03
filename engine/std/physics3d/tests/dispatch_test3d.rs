//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): physics3d's
//! colored passes, run by every dispatcher of the spike at 1 to 8 threads,
//! are its one-thread solve bit for bit; and the check sees a block run
//! twice.

#![allow(dead_code)]

#[path = "../../physics2d/compare/dispatch.rs"]
mod dispatch;
#[path = "dispatch_3d.rs"]
mod dispatch_3d;
#[path = "../gjk.rs"]
mod gjk;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../../physics2d/tests/pool.rs"]
mod pool;
#[path = "../solver.rs"]
mod solver;

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Order, Quat, Reduce, Shape, Tuning, Vec3};

use dispatch::{Plant, Protocol};
use dispatch_3d::{How, Input, Solve, bits};

fn inputs() -> Vec<Input> {
    let mut all = dispatch_3d::capture("boxes 1000", &[240, 300]);
    all.extend(dispatch_3d::capture("planks 200", &[300]));
    all
}

fn solved(i: &Input, how: How) -> String {
    let mut o = i.clone();
    Solve::default().solve(&mut o, how, None);
    bits(&o)
}

#[test]
fn every_dispatcher_is_the_one_thread_solve_bit_for_bit_and_sees_a_block_run_twice() {
    let inputs = inputs();
    for i in &inputs {
        assert!(Solve::default().prepare(&mut i.clone()), "the step goes in lanes, colored");
    }
    let bases: Vec<String> = inputs.iter().map(|i| solved(i, How::One)).collect();
    for n in [1, 2, 4, 8] {
        let exec = pool::Pool::new(n);
        for p in Protocol::ALL {
            for (k, (i, base)) in inputs.iter().zip(&bases).enumerate() {
                assert!(solved(i, How::Across(p, &exec, Plant::None)) == *base, "{} at {n}, input {k}", p.name());
                assert!(solved(i, How::Across(p, &exec, Plant::Twice)) != *base, "{} at {n}, input {k}: twice unseen", p.name());
            }
        }
    }
}
