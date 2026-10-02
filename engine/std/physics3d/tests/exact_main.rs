//! physics3d's exact fingerprint's tool (physics-testing.md, "The exact
//! fingerprint"; runbook 005): runs both layers (`exact.rs`) and says what
//! differs from `exact.txt`; `--write` writes the file in the workspace.
//!
//!     ./bazel run //engine/std/physics3d:exact
//!     ./bazel run //engine/std/physics3d:exact -- --write
//!
//! Only in a commit that changes results on purpose, whose message says
//! why; a change that claims to keep them bit for bit leaves the file as
//! it was.

#![allow(dead_code)]

#[path = "exact.rs"]
mod exact;
#[path = "../gjk.rs"]
mod gjk;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Quat, Reduce, Shape, Tuning, Vec3};

fn main() {
    let write = std::env::args().skip(1).any(|a| a == "--write");
    let (the_mod, _) = exact::the_mod();
    let kernel = exact::the_kernel();
    let mut same = true;
    for (layer, lines) in [("mod", &the_mod), ("kernel", &kernel)] {
        match exact::differ(&exact::pinned(layer), lines) {
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
