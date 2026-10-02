//! What the 2D and 3D physics mods share apart from the dimension
//! (docs/architecture/physics-sharing.md, "Phase 1"): code both had a copy
//! of, moved here so a fix lands in both. It takes plain values (entities,
//! indices, scalars), never a mod's components, which stay in each mod's
//! interface: this crate is a dependency of the mods' implementations
//! only, so changing it reloads the two physics mods and no game.
//!
//! No statics (a reloaded mod's image starts its statics over) and no
//! unsafe code.

mod slots;
mod soft;

pub use slots::Slots;
pub use soft::Softness;
