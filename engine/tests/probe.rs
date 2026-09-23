//! What a test mod reports about itself. Mods write it into the world, and
//! tests read it back from the loader's `WorldStorage`, so assertions are on
//! data rather than on log text.

use engine_api::component;

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Probe: "test::Probe" {
        /// The mod's running total, from its `Mod` state.
        pub value: u64,
        /// Which variant of the mod wrote this.
        pub build: u32,
        /// A `static` in the mod, incremented on every load. 1 after any load
        /// proves the load mapped a fresh image rather than reusing one.
        pub loads_seen_by_statics: u32,
    }
}
