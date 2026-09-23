//! Hot-reload demo. Edit the message or the step size and `bazel run //mods/counter`:
//! the count carries on from where it was, with the new code.

use engine_api::{Cx, Mod, Status, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Counter {
        frames: u64,
        count: u64,
    }
}

impl Mod for Counter {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log(format!("loaded (generation {}), count is {}", cx.generation(), self.count));
    }

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        self.frames += 1;
        if self.frames % 60 == 0 {
            self.count += 1;
            cx.log(format!("count = {}", self.count));
        }
        Status::OK
    }
}

export_mod!(Counter);
