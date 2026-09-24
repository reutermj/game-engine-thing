//! Builds of `rates`, whose systems trace how often they run and the `Dt`
//! each run covers: `sixty` in `simulate` (60 Hz), `ten` in a 10 Hz phase
//! of its own, and `each` once a frame. v1 and v2 differ only in their
//! label, for the reload test.

use engine_api::{Cx, Dt, Mod, Query, Systems, export_mod, phase};
use test_probe::{Trace, ensure_trace, trace};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

engine_api::mod_state! {
    #[derive(Default)]
    struct Rates {}
}

impl Rates {
    fn sixty(&mut self, _: &mut (), _: &mut Cx, dt: Dt, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{BUILD} 60 {:.4}", *dt));
    }
    fn ten(&mut self, _: &mut (), _: &mut Cx, dt: Dt, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{BUILD} 10 {:.4}", *dt));
    }
    fn each(&mut self, _: &mut (), _: &mut Cx, dt: Dt, mut q: Query<&mut Trace>) {
        trace(&mut q, format!("{BUILD} frame {:.4}", *dt));
    }
}

impl Mod for Rates {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.phase("rates::ten").after(phase::LATE).fixed_hz(10.0);
        s.add("each", Self::each);
        s.add("sixty", Self::sixty).phase(phase::SIMULATE);
        s.add("ten", Self::ten).phase("rates::ten");
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        ensure_trace(cx);
    }
}

export_mod!(Rates);
