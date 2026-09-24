//! Prints every position once a second, so a reload elsewhere is visible here.

use engine_api::{Cx, Mod, Query, Systems, export_mod, phase};
use transform::Position;

engine_api::mod_state! {
    #[derive(Default)]
    struct Reporter {
        frames: u64,
    }
}

impl Reporter {
    fn report(&mut self, _: &mut (), cx: &mut Cx, mut positions: Query<&Position>) {
        self.frames += 1;
        if self.frames % 60 != 0 {
            return;
        }
        let mut any = false;
        positions.for_each(|row, p| {
            cx.log(format!("entity {}: {p:?}", row.entity().index));
            any = true;
        });
        if !any {
            cx.log("no positions");
        }
    }
}

impl Mod for Reporter {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        // After the frame's movement.
        s.add("report", Self::report).phase(phase::LATE);
    }
}

export_mod!(Reporter);
