//! Builds of `calc`, which provides `calc::Calc`. v1 and v2 share the
//! interface and differ in `apply`, so reloading between them is an
//! implementation-only change its callers don't notice.

use engine_api::{Cx, Mod, export_mod};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

engine_api::mod_state! {
    #[derive(Default)]
    struct CalcMod {
        calls: u32,
    }
}

impl Mod for CalcMod {
    type Transient = ();
}

impl calc::Calc for CalcMod {
    fn apply(&mut self, _: &mut (), _cx: &mut Cx, x: i64) -> i64 {
        self.calls += 1;
        if cfg!(feature = "v1") { x + 1 } else { x * 2 }
    }

    fn describe(&mut self, _: &mut (), _cx: &mut Cx) -> String {
        format!("calc {BUILD} ({} calls)", self.calls)
    }

    fn greet(&mut self, _: &mut (), _cx: &mut Cx, name: &str) -> String {
        format!("hello, {name}")
    }

    fn boom(&mut self, _: &mut (), _cx: &mut Cx) {
        panic!("calc blew up");
    }

    fn recurse(&mut self, _: &mut (), cx: &mut Cx) -> String {
        match calc::apply(cx, 1) {
            Ok(v) => v.to_string(),
            Err(e) => e.to_string(),
        }
    }
}

export_mod!(CalcMod, provides = [calc::Calc]);
