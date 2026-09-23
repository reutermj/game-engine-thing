//! The default `Scheduler`: runs each phase's systems one at a time, in the
//! loader's order. The same frame the loader runs itself when no scheduler
//! is loaded, built from the primitives any scheduler uses; a parallel one
//! differs only in when it calls `Frame::run`.

use engine_api::scheduler::{self, Scheduler};
use engine_api::{Cx, Mod, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Sequential {}
}

impl Mod for Sequential {
    type Transient = ();
}

impl Scheduler for Sequential {
    fn run_frame(&mut self, _: &mut (), cx: &mut Cx) {
        let Some(frame) = scheduler::begin(cx) else { return };
        for phase in &frame.plan().phases {
            for system in &phase.systems {
                frame.run(system.id);
            }
            frame.end_phase();
        }
    }
}

export_mod!(Sequential, provides = [scheduler::Scheduler]);
