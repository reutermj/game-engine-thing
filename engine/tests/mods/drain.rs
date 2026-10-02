//! Builds of `drain`, which depends on `streamer`'s interface: it sees the
//! flow `test::Samples` and, unless built as `seer`, takes it, so its
//! allocation goes back to the bin for streamer's next `Make`.

use engine_api::{Cx, Mod, Query, See, Systems, export_mod};
use streamer::Samples;
use test_probe::{Trace, ensure_trace, trace};

engine_api::mod_state! {
    #[derive(Default)]
    struct Drain {}
}

impl Drain {
    fn see(&mut self, _: &mut (), _: &mut Cx, mut q: Query<&mut Trace>, samples: See<Samples>) {
        trace(&mut q, format!("saw {}", samples.values.iter().sum::<u64>()));
    }

    #[cfg(not(feature = "seer"))]
    fn take(&mut self, _: &mut (), _: &mut Cx, mut q: Query<&mut Trace>, samples: engine_api::Take<Samples>) {
        trace(&mut q, format!("took {}", samples.values.len()));
    }
}

impl Mod for Drain {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("see", Self::see).after("streamer::make");
        #[cfg(not(feature = "seer"))]
        s.add("take", Self::take).after("drain::see");
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Drain);
