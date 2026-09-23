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
    fn report(&mut self, _: &mut (), cx: &mut Cx, positions: Query<&Position>) {
        self.frames += 1;
        if self.frames % 60 != 0 {
            return;
        }
        // Collected first: the query borrows `cx`, and so does logging.
        let lines: Vec<String> = positions.iter(cx).map(|(e, p)| format!("entity {}: {p:?}", e.index)).collect();
        if lines.is_empty() {
            cx.log("no positions");
        }
        for line in lines {
            cx.log(line);
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
