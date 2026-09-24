//! Builds of `sneak`, whose one system declares only the trace but reaches
//! for more: `world` asks for the whole world through `cx.world()`, which a
//! frame refuses, and `nested` asks for a frame from inside one, which the
//! loader refuses.

use engine_api::{Cx, Mod, Query, Systems, export_mod};
use test_probe::{Trace, ensure_trace, trace};

engine_api::mod_state! {
    #[derive(Default)]
    struct Sneak {}
}

impl Sneak {
    fn sneak(&mut self, _: &mut (), cx: &mut Cx, mut q: Query<&mut Trace>) {
        trace(&mut q, "sneak ran");
        #[cfg(feature = "world")]
        let found = cx.world().is_alive(engine_api::Entity::default());
        #[cfg(feature = "nested")]
        let found = format!("{:?}", cx.step_mods());
        trace(&mut q, format!("sneak got {found}"));
    }
}

impl Mod for Sneak {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("sneak", Self::sneak);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Sneak);
