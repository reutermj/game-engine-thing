//! Builds of `streamer`, which makes the flow `test::Samples` (declared in
//! its interface, whose layout v2 changes) every frame, tracing what each
//! `Make` started from: a recycled allocation, or nothing.

use engine_api::{Cx, Make, Mod, Query, Systems, export_mod};
use streamer::Samples;
use test_probe::{Trace, ensure_trace, trace};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

engine_api::mod_state! {
    #[derive(Default)]
    struct Streamer {}
}

impl Streamer {
    fn make(&mut self, _: &mut (), _: &mut Cx, mut q: Query<&mut Trace>, mut out: Make<Samples>) {
        trace(&mut q, format!("{BUILD} made from {}/{}", out.values.len(), out.values.capacity()));
        #[cfg(feature = "v1")]
        out.values.extend(0..8);
        #[cfg(feature = "v2")]
        {
            out.label.push_str("v2");
            out.values.extend(100..108);
        }
    }
}

impl Mod for Streamer {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("make", Self::make);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Streamer);
