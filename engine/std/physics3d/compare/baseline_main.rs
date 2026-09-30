//! The 3D baseline's tool (physics-testing.md, "The baseline"; runbook
//! 005), as 2D's: runs every scene the baseline records (`record.rs`) and
//! prints every value that moved past its band from `baseline.txt`
//! (`physics_testkit::tool` has the flags).
//!
//!     ./bazel run //engine/std/physics3d/compare:baseline
//!     ./bazel run //engine/std/physics3d/compare:baseline -- --write
//!     ./bazel run //engine/std/physics3d/compare:baseline -- --long --write
//!
//! `--offset=<n>` runs each pile at its size and n (another seeded drop),
//! under the size's name. `TUNE=<variant>` compares a tuning of ours
//! against the baseline.

use physics_testkit::tool::{self, Tool};
use physics3d_compare::{record, runs};

fn main() {
    tool::main(&Tool {
        dim: "3D",
        dir: "engine/std/physics3d/compare",
        default: record::DEFAULT,
        long: record::LONG,
        write: record::WRITE,
        write_long: record::WRITE_LONG,
        offset: "<n>",
        set_offset: &|n| {
            let n: usize = n.parse().expect("--offset=<n>");
            record::set_offset(n);
            n as i64
        },
        // One group after the other, as this tool always ran them (2D's
        // runs both at once): each run of ours is an engine with its mods
        // loaded, and the grids are sized to what one process maps
        // (`runs::par`).
        record: &|long| {
            let mut entries = record::quality(long);
            entries.extend(record::behaviour(long));
            entries
        },
        variant: ("TUNE", !runs::tune().is_empty()),
    });
}
