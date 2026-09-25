//! Builds of two mods, `a` and `b`, whose systems record into the trace, so a
//! test can see the order a frame ran them in. Loaded `a` then `b`:
//!
//! - `a::input` (input), `a::update` (update, after `b::update`), `a::late` (late)
//! - `b::update` (update), `b::simulate` (simulate)
//!
//! `b_cycle` is `b` with `b::update` also after `a::update`: a cycle.

use engine_api::{Cx, Mod, Query, Systems, export_mod, phase};
use test_probe::{Trace, ensure_trace, trace};

engine_api::mod_state! {
    #[derive(Default)]
    struct Tracer {}
}

// Each build registers only some of these.
#[allow(dead_code)]
impl Tracer {
    fn input(&mut self, _: &mut (), cx: &mut Cx, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{}::input", cx.name()));
    }
    fn update(&mut self, _: &mut (), cx: &mut Cx, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{}::update", cx.name()));
    }
    fn simulate(&mut self, _: &mut (), cx: &mut Cx, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{}::simulate", cx.name()));
    }
    fn late(&mut self, _: &mut (), cx: &mut Cx, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{}::late", cx.name()));
    }
}

impl Mod for Tracer {
    type Transient = ();

    #[cfg(feature = "a")]
    fn systems(s: &mut Systems<Self>) {
        s.add("late", Self::late).phase(phase::LATE);
        s.add("update", Self::update).after("b::update");
        s.add("input", Self::input).phase(phase::INPUT);
    }

    #[cfg(any(feature = "b", feature = "b_cycle"))]
    fn systems(s: &mut Systems<Self>) {
        let update = s.add("update", Self::update);
        #[cfg(feature = "b_cycle")]
        let update = update.after("a::update");
        let _ = update;
        s.add("simulate", Self::simulate).phase(phase::SIMULATE);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Tracer);
