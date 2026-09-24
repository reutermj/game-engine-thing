//! Builds of `scheduler`, a `Scheduler` that runs frames as the sequential
//! one does and marks the trace with its build first, so a test can tell it
//! ran the frame rather than the loader's own. `panic` panics once, inside
//! the frame, after running the first node.

use engine_api::scheduler::{self, Scheduler};
use engine_api::{Cx, Mod, export_mod};
use test_probe::Trace;

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";
#[cfg(feature = "panic")]
const BUILD: &str = "panic";

engine_api::mod_state! {
    #[derive(Default)]
    struct Marking {}
}

impl Mod for Marking {
    type Transient = ();
}

impl Scheduler for Marking {
    fn run_frame(&mut self, _: &mut (), cx: &mut Cx) {
        // Before the frame opens, so the world may be touched directly.
        cx.world().for_each::<&mut Trace>(|_, trace| trace.lines.push(format!("scheduled by {BUILD}")));
        let Some(frame) = scheduler::begin(cx) else { return };
        for (i, node) in frame.plan().nodes.iter().enumerate() {
            frame.run(node.id);
            if cfg!(feature = "panic") && i == 0 {
                panic!("the scheduler panics mid-frame");
            }
        }
    }
}

export_mod!(Marking, provides = [scheduler::Scheduler]);
