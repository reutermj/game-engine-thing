//! A bootstrap mod with no frame pacing, so tests step as fast as they like.

use engine_api::{Cx, Mod, Status, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Driver {}
}

impl Mod for Driver {
    type Transient = ();

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        cx.step_mods()
    }
}

export_mod!(Driver);
