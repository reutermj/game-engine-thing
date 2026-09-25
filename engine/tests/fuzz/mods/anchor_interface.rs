//! `anchor`'s interface, which only `tether` depends on: a resident mod may
//! depend only on resident mods, and anchor is built both ways.

/// The name it replies with: something for a dependent to compile against.
pub const NAME: &str = "anchor";
