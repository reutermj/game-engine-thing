//! Builds of the mod `base`. v1 and v1b share an interface and differ only in
//! their implementation; v2 changes the interface.

use engine_api::{Cx, Mod, Status, export_mod};

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v1b")]
const BUILD: &str = "v1b";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";

#[derive(Default)]
struct Base;

impl Mod for Base {
    fn load(&mut self, cx: &mut Cx) {
        cx.log(format!("base {BUILD}"));
    }

    fn step(&mut self, cx: &mut Cx) -> Status {
        cx.world().query::<base::Shared>().count();
        Status::OK
    }
}

export_mod!(Base);
