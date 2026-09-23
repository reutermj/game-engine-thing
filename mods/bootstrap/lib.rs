//! Owns the frame loop: paces frames at a fixed rate and steps every other mod.

use std::time::{Duration, Instant};

use engine_api::{Cx, Mod, Status, export_mod};

const FRAME: Duration = Duration::from_nanos(1_000_000_000 / 60);

#[derive(Default)]
struct Bootstrap {
    frame: u64,
    next_frame: Option<Instant>,
}

impl Mod for Bootstrap {
    fn load(&mut self, cx: &mut Cx) {
        cx.log(format!(
            "loaded (generation {}), frame loop at frame {}",
            cx.generation(),
            self.frame
        ));
    }

    fn step(&mut self, cx: &mut Cx) -> Status {
        cx.step_mods();
        self.frame += 1;

        let next = self.next_frame.unwrap_or_else(Instant::now) + FRAME;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
            self.next_frame = Some(next);
        } else {
            // Fell behind (e.g. paused in a debugger); don't try to catch up.
            self.next_frame = Some(now);
        }
        Status::OK
    }
}

export_mod!(Bootstrap);
