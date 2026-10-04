//! The 2D baseline's tool (physics-testing.md, "The baseline"; runbook
//! 005): runs every scene the baseline records (`record.rs`) and prints,
//! for every value that moved past its band from `baseline.txt`, the old
//! value, the new, the band and which way it went (`physics_testkit::tool`
//! has the flags).
//!
//!     ./bazel run //engine/std/physics2d/compare:baseline
//!     ./bazel run //engine/std/physics2d/compare:baseline -- --write
//!     ./bazel run //engine/std/physics2d/compare:baseline -- --long --write
//!
//! `--long` is minutes at -c opt; `--offset=<percent>` runs every pile
//! family at its sizes moved by that share. `SOLVER=<variant>` compares a
//! variant of `variants.rs` against the baseline, as the tests take it.

#[allow(dead_code)] // `Arrays::snapshot`, which only `:tax` uses.
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)] // The label, which the comparison prints.
mod behave;
mod bounces;
#[allow(dead_code)] // The timings and rain, which only the comparison reads.
mod ecs;
mod family;
mod meets;
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
mod quality;
mod record;
mod runs;
#[allow(dead_code)]
mod scene;
mod settle;
#[allow(dead_code)]
mod sim;
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)] // The constants and tests only the experiments use.
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
mod variants;

pub use sim::{Dyn, Sim};

use physics_testkit::baseline;
use physics_testkit::tool::{self, Tool};

fn main() {
    tool::main(&Tool {
        dim: "2D",
        dir: "engine/std/physics2d/compare",
        default: record::DEFAULT,
        long: record::LONG,
        write: record::WRITE,
        write_long: record::WRITE_LONG,
        offset: "<percent>",
        set_offset: &|n| {
            let percent: i32 = n.parse().expect("--offset=<percent>");
            record::set_offset(percent);
            percent.into()
        },
        // Both groups side by side: each is its runs' slowest scene.
        record: &|long| {
            std::thread::scope(|s| {
                let q = s.spawn(|| record::quality(long));
                let b = s.spawn(|| record::behaviour(long));
                [q.join().expect("quality"), b.join().expect("behaviour")].concat()
            })
        },
        variant: ("SOLVER", runs::solver().is_some()),
    });
}
