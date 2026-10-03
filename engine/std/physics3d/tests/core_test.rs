//! The 3D narrowphase and solver alone, apart from the mod: their own unit
//! tests, over the interface's types, as the mod's crate root has them.

// Only what the tests use of each is used here.
#![allow(dead_code)]

#[path = "../gjk.rs"]
mod gjk;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Order, Quat, Reduce, Shape, Tuning, Vec3};
