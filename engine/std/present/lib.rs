//! `present`: the extract, the one producer of the frame's `DrawList`,
//! which every presenter reads. A system in the render phase, so it sees
//! the frame's final places; presenters order themselves after it
//! (`.after("present::extract")`). Its work is `extract.rs`.

use engine_api::{Cx, Make, Mod, Query, Systems, export_mod, phase};
use present::{Camera, Circle, DrawList, Label, Line, Look, Place, Rect, Text};

mod extract;

use extract::Scratch;

engine_api::mod_state! {
    #[derive(Default)]
    struct Present {}
}

impl Present {
    fn extract(
        &mut self,
        scratch: &mut Scratch,
        _: &mut Cx,
        (mut rects, mut circles, mut lines): (
            Query<(&Place, &Rect, &Look)>,
            Query<(&Place, &Circle, &Look)>,
            Query<(&Place, &Line, &Look)>,
        ),
        mut texts: Query<(&Place, &Text, &Look)>,
        mut labels: Query<(&Place, &Label)>,
        mut cameras: Query<&Camera>,
        mut out: Make<DrawList>,
    ) {
        extract::extract(&mut out, scratch, (&mut rects, &mut circles, &mut lines), &mut texts, &mut labels, &mut cameras);
    }
}

impl Mod for Present {
    type Transient = Scratch;

    fn systems(s: &mut Systems<Self>) {
        s.add("extract", Self::extract).phase(phase::RENDER);
    }
}

export_mod!(Present);
