//! Hot-reload demo. Edit the message or the step size and `bazel run //mods/counter`:
//! the count carries on from where it was, with the new code.

use engine_api::{Cx, Mod, Systems, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Counter {
        frames: u64,
        count: u64,
    }
}

impl Counter {
    fn tick(&mut self, _: &mut (), cx: &mut Cx) {
        self.frames += 1;
        if self.frames.is_multiple_of(60) {
            self.count += 1;
            cx.log(format!("count = {}", self.count));
        }
    }
}

impl Mod for Counter {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("tick", Self::tick);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log(format!("loaded (generation {}), count is {}", cx.generation(), self.count));
    }

}

export_mod!(Counter);
