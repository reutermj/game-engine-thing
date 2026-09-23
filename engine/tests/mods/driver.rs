//! A bootstrap mod with no frame pacing, so tests step as fast as they like.

use engine_api::{Cx, Mod, Status, export_mod};

#[derive(Default)]
struct Driver;

impl Mod for Driver {
    fn step(&mut self, cx: &mut Cx) -> Status {
        cx.step_mods()
    }
}

export_mod!(Driver);
