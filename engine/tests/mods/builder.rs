//! `builder` inserts a probe onto the trace entity from `build`, looking for
//! it from `build` itself and from systems ordered before and after it, so a
//! test can see who an insert is visible to.

use engine_api::{Adds, Cx, Mod, Query, Systems, export_mod};
use test_probe::{Probe, Trace, ensure_trace, trace};

engine_api::mod_state! {
    #[derive(Default)]
    struct Builder {}
}

impl Builder {
    fn build(&mut self, _: &mut (), _: &mut Cx, mut q: Query<&mut Trace, (), Adds<Probe>>, mut probes: Query<&Probe>) {
        q.for_each(|row, mut trace| {
            row.insert(Probe { value: 7, ..Probe::default() });
            trace.lines.push("inserted".into());
        });
        let mut seen = Vec::new();
        probes.for_each(|_, p| seen.push(p.value));
        trace(&mut q, format!("build saw {seen:?}"));
    }

    fn look(&mut self, _: &mut (), _: &mut Cx, mut traces: Query<&mut Trace>, mut probes: Query<&Probe>) {
        let mut seen = Vec::new();
        probes.for_each(|_, p| seen.push(p.value));
        trace(&mut traces, format!("saw {seen:?}"));
    }
}

impl Mod for Builder {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("look_before", Self::look).before("builder::build");
        s.add("build", Self::build);
        s.add("look_after", Self::look).after("builder::build");
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Builder);
