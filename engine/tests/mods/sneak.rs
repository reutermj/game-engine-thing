//! Builds of `sneak`, whose one system declares only the trace but reaches
//! further through `cx.world()`: `read` reads the probe, `insert` inserts
//! one on the spot rather than through commands, and `declared` reads it
//! having declared it with `.reads`, which is fine.

use engine_api::{Cx, Mod, Query, Systems, export_mod};
use test_probe::{Probe, Trace, ensure_trace, trace};

engine_api::mod_state! {
    #[derive(Default)]
    struct Sneak {}
}

impl Sneak {
    fn sneak(&mut self, _: &mut (), cx: &mut Cx, q: Query<&mut Trace>) {
        let Some(e) = q.iter(cx).next().map(|(e, _)| e) else { return };
        #[cfg(any(feature = "read", feature = "declared"))]
        let found = cx.world().get::<Probe>(e).is_some();
        #[cfg(feature = "insert")]
        let found = cx.world().insert(e, Probe::default());
        trace(cx, &q, format!("sneak got {found}"));
    }
}

impl Mod for Sneak {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        let sneak = s.add("sneak", Self::sneak);
        #[cfg(feature = "declared")]
        let sneak = sneak.reads::<Probe>();
        let _ = sneak;
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Sneak);
