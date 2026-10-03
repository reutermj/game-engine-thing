//! What the 2D and 3D physics mods share apart from the dimension
//! (docs/architecture/physics-sharing.md, "Phase 1"): code both had a copy
//! of, moved here so a fix lands in both; and from phase 2, what 2D has
//! and 3D will build on unchanged (`lanes::F`, for 3D's lanes kernel,
//! get-emj.52). It takes plain values (entities,
//! indices, scalars), never a mod's components, which stay in each mod's
//! interface: this crate is a dependency of the mods' implementations
//! only, so changing it reloads the two physics mods and no game.
//!
//! No statics (a reloaded mod's image starts its statics over) and no
//! unsafe code.

mod closing;
pub mod lanes;
mod slots;
mod soft;

pub use closing::Closing;
pub use slots::Slots;

/// How near two shapes count as in contact, with a negative depth: a
/// speculative contact, which lets the solver stop a body at the surface
/// instead of after it has sunk in, and keeps resting contacts from
/// flickering in and out between steps. Box2D's (Erin Catto's,
/// docs/CREDITS.md); each mod's narrowphase finds pairs within it, and
/// the broadphase grows boxes by it.
pub const MARGIN: f32 = 0.05;

/// How far each collider's fat box reaches past its box grown by
/// `MARGIN`: the broadphase keeps its pairs while bodies stay inside
/// their fat boxes (each mod's `Contacts`; spatial-storage.md, "Keeping
/// pairs"). A settled pile creeps less than any margin tried, so the
/// smallest was cheapest, with the fewest candidates: in 2D 131 µs
/// against 139 at Box2D's 0.05, 10 000 settled; in 3D, where a margin's
/// candidates grow as its volume, 314 µs against 387 at 0.05 (Box3D's
/// cap), 10 000 boxes settled (both 2026-09-27).
pub const FAT: f32 = 0.02;
pub use soft::{BOUNCE_THRESHOLD, DAMPING_RATIO, MAX_PUSH, Softness};
