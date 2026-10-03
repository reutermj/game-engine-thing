//! physics3d's exact fingerprint's tool (physics-testing.md, "The exact
//! fingerprint"; runbook 005): runs both layers (`exact.rs`) and says what
//! differs from `exact.txt`; `--write` writes the file in the workspace.
//!
//!     ./bazel run //engine/std/physics3d:exact
//!     ./bazel run //engine/std/physics3d:exact -- --write
//!
//! Only in a commit that changes results on purpose, whose message says
//! why; a change that claims to keep them bit for bit leaves the file as
//! it was. `TUNE=<variant>` (`physics3d::Tuning::parse`) compares a
//! variant's lines with the pinned ones, and never writes:
//!
//!     TUNE=order=colored ./bazel run //engine/std/physics3d:exact

#![allow(dead_code)]

#[path = "exact.rs"]
mod exact;
#[path = "../gjk.rs"]
mod gjk;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Order, Quat, Reduce, Shape, Tuning, Vec3};

fn main() {
    let write = std::env::args().skip(1).any(|a| a == "--write");
    // A variant's lines against the default's, to see where it moves
    // them: every kernel tuning with it, against the pinned lines; and the
    // mod's scene tuned so, against the scene with the default's `Tuning`
    // named (`order=levels`), since a `Tuning` in the world is an entity,
    // which moves every body's index from the pinned run's.
    let tune: &'static str = std::env::var("TUNE").unwrap_or_default().leak();
    assert!(!(write && !tune.is_empty()), "the fingerprint is the default's: unset TUNE to write it");
    let (the_mod, _) = exact::the_mod(exact::Run { tune, shared: false });
    let kernel = exact::the_kernel(tune);
    let named = (!tune.is_empty()).then(|| exact::the_mod(exact::Run { tune: "order=levels", shared: false }).0);
    let mod_against: Vec<&str> = match &named {
        Some(lines) => lines.iter().map(String::as_str).collect(),
        None => exact::pinned("mod"),
    };
    let mut same = true;
    for (layer, lines, against) in [("mod", &the_mod, mod_against), ("kernel", &kernel, exact::pinned("kernel"))] {
        match exact::differ(&against, lines) {
            Some(d) => {
                same = false;
                println!("{layer}: {d}");
            }
            None => println!("{layer}: as pinned ({} lines)", lines.len()),
        }
    }
    if !write {
        if !same {
            println!("--write to pin these, in a commit that says why they moved");
        }
        return;
    }
    let workspace = std::env::var("BUILD_WORKSPACE_DIRECTORY").expect("--write is for `bazel run`, which sets BUILD_WORKSPACE_DIRECTORY");
    let path = std::path::Path::new(&workspace).join(exact::PATH);
    std::fs::write(&path, exact::file(&the_mod, &kernel)).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    println!("wrote {}", path.display());
}
