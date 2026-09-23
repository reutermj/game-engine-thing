//! Prints every position once a second, so a reload elsewhere is visible here.

use engine_api::{Cx, Mod, Status, export_mod};
use transform::Position;

engine_api::mod_state! {
    #[derive(Default)]
    struct Reporter {
        frames: u64,
    }
}

impl Mod for Reporter {
    type Transient = ();

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        self.frames += 1;
        if self.frames % 60 != 0 {
            return Status::OK;
        }
        // Collected first: the query borrows `cx`, and so does logging.
        let lines: Vec<String> = cx
            .world()
            .query::<Position>()
            .map(|(e, p)| format!("entity {}: {p:?}", e.index))
            .collect();
        if lines.is_empty() {
            cx.log("no positions");
        }
        for line in lines {
            cx.log(line);
        }
        Status::OK
    }
}

export_mod!(Reporter);
