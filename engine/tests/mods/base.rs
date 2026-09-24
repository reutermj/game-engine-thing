//! Builds of the mod `base`. v1 and v1b share an interface and differ only in
//! their implementation; v2 changes the interface.

use engine_api::{Cx, Mod, Query, Systems, export_mod};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v1b")]
const BUILD: &str = "v1b";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

engine_api::mod_state! {
    #[derive(Default)]
    struct Base {}
}

impl Base {
    fn touch(&mut self, _: &mut (), _: &mut Cx, mut shared: Query<&base::Shared>) {
        shared.for_each(|_, _| {});
    }
}

impl Mod for Base {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("touch", Self::touch);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log(format!("base {BUILD}"));
    }

}

export_mod!(Base);
