//! `builder` queues a probe onto the trace entity through commands in
//! `update`, and looks for it right after (same phase) and in `simulate`
//! (the next), so a test can see when commands land.

use engine_api::{Cx, Mod, Query, Systems, export_mod, phase};
use test_probe::{Probe, Trace, ensure_trace, trace};

engine_api::mod_state! {
    #[derive(Default)]
    struct Builder {}
}

impl Builder {
    fn build(&mut self, _: &mut (), cx: &mut Cx, q: Query<&mut Trace>) {
        let Some(e) = q.iter(cx).next().map(|(e, _)| e) else { return };
        cx.commands().insert(e, Probe { value: 7, ..Probe::default() });
        trace(cx, &q, "queued");
    }

    fn look(&mut self, _: &mut (), cx: &mut Cx, traces: Query<&mut Trace>, probes: Query<&Probe>) {
        let seen = probes.iter(cx).map(|(_, p)| p.value).collect::<Vec<_>>();
        trace(cx, &traces, format!("saw {seen:?}"));
    }
}

impl Mod for Builder {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("build", Self::build);
        s.add("look_now", Self::look).after("builder::build");
        s.add("look_next", Self::look).phase(phase::SIMULATE);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Builder);
