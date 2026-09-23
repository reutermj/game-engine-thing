//! Builds of `listener`, which reads `Ping`s in `input` (before `pinger`'s
//! `simulate`), in `simulate` right after `pinger::ping`, and in `late`,
//! tracing what each saw, labeled with the build.

use engine_api::{Cx, EventReader, Mod, Query, Systems, export_mod, phase};
use test_probe::{Ping, Trace, ensure_trace, trace};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

engine_api::mod_state! {
    #[derive(Default)]
    struct Listener {}
}

impl Listener {
    fn listen(&mut self, _: &mut (), cx: &mut Cx, q: Query<&mut Trace>, pings: EventReader<Ping>) {
        let seen: Vec<u64> = pings.read(cx).iter().map(|p| p.n).collect();
        trace(cx, &q, format!("{BUILD} saw {seen:?}"));
    }
}

impl Mod for Listener {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("early", Self::listen).phase(phase::INPUT);
        s.add("mid", Self::listen).phase(phase::SIMULATE).after("pinger::ping");
        s.add("late", Self::listen).phase(phase::LATE);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Listener);
