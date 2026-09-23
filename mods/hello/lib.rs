//! Not in the game's manifest: `bazel run //mods/hello` loads it into a running
//! engine that has never heard of it.

use engine_api::{Cx, Mod, Status, export_mod};

#[derive(Default)]
struct Hello {
    frames: u64,
}

impl Mod for Hello {
    fn load(&mut self, cx: &mut Cx) {
        cx.log("hello! loaded live");
    }

    fn step(&mut self, cx: &mut Cx) -> Status {
        self.frames += 1;
        if self.frames % 120 == 0 {
            cx.log(format!("still here after {} frames", self.frames));
        }
        Status::OK
    }

    fn close(&mut self, cx: &mut Cx) {
        cx.log("goodbye");
    }
}

export_mod!(Hello);
