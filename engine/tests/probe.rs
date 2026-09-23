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

component! {
    /// What scheduled test mods did, in order, one line each: the order
    /// systems ran in, and what they saw. One entity holds it.
    #[derive(Debug, Default, PartialEq)]
    pub struct Trace: "test::Trace" {
        pub lines: Vec<String>,
    }
}

engine_api::event! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Ping: "test::Ping" {
        pub n: u64,
    }
}

/// Makes the trace entity, if no mod has yet. For a mod's `load`, which may
/// change the world's structure directly.
pub fn ensure_trace(cx: &mut engine_api::Cx) {
    let mut world = cx.world();
    if world.query::<Trace>().next().is_none() {
        let e = world.spawn();
        world.insert(e, Trace::default());
    }
}

/// Appends to the trace through a query: what a system declares.
pub fn trace(cx: &mut engine_api::Cx, q: &engine_api::Query<&mut Trace>, line: impl Into<String>) {
    let line = line.into();
    for (_, trace) in q.iter(cx) {
        trace.lines.push(line.clone());
    }
}
