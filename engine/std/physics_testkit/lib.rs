//! What the 2D and 3D physics harnesses (`//engine/std/physics/compare`,
//! `//engine/std/physics3d/compare`) share: the parts of testing physics
//! that don't depend on the dimension (physics-testing.md, "Where the
//! tests live"). The baseline's file format, bands and comparison, and its tool's
//! command line; a run's named values and the bounds a test collects; the
//! statistics a family is judged by; each run made once a test binary; and
//! the bounce families' statistics, which are the same rules over either
//! dimension's bounces.
//!
//! Everything here takes plain values or small traits (`Named`, `Values`,
//! `Bounce`), never a 2D or a 3D type: what a scene is, how a body is
//! measured and how each engine is driven stay in each harness.

pub mod baseline;
pub mod behaviour;
pub mod bounces;
pub mod broken;
pub mod runs;
pub mod stats;
pub mod tool;

pub use behaviour::{Behaviour, NEVER};
pub use broken::{Broken, Named, Values};
