//! Builds of the mod `base`. v1 and v1b share an interface and differ only in
//! their implementation; v2 changes the interface.

use engine_api::{Cx, Mod, Status, export_mod};

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

impl Mod for Base {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log(format!("base {BUILD}"));
    }

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        cx.world().query::<base::Shared>().count();
        Status::OK
    }
}

export_mod!(Base);
