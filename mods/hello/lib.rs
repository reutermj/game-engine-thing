//! Not in the game's manifest: `bazel run //mods/hello` loads it into a running
//! engine that has never heard of it.

use engine_api::{Cx, Mod, Systems, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Hello {
        frames: u64,
    }
}

impl Hello {
    fn tick(&mut self, _: &mut (), cx: &mut Cx) {
        self.frames += 1;
        if self.frames.is_multiple_of(120) {
            cx.log(format!("still here after {} frames", self.frames));
        }
    }
}

impl Mod for Hello {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("tick", Self::tick);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log("hello! loaded live");
    }

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log("goodbye");
    }
}

export_mod!(Hello);
