//! Scheduling policy as a mod: the `Scheduler` service the engine defines,
//! and the frame primitives a scheduler is built from.
//!
//! A bootstrap runs each frame with `cx.run_frame()`, which calls the loaded
//! `Scheduler` if there is one. A scheduler asks the loader for the frame's
//! plan and runs its systems:
//!
//! ```ignore
//! impl engine_api::scheduler::Scheduler for Sequential {
//!     fn run_frame(&mut self, _: &mut (), cx: &mut Cx) {
//!         let Some(frame) = scheduler::begin(cx) else { return };
//!         for phase in &frame.plan().phases {
//!             for system in &phase.systems {
//!                 frame.run(system.id);
//!             }
//!             frame.end_phase();
//!         }
//!     }
//! }
//! export_mod!(Sequential, provides = [engine_api::scheduler::Scheduler]);
//! ```
//!
//! The loader keeps what makes a frame safe: the plan, the access checks,
//! and refusing a system whose mod is already running. The scheduler decides
//! only when each system runs. See docs/architecture/scheduling.md, "Who
//! schedules".

use std::marker::PhantomData;

use crate::{CallErrorKind, Cx, ModContext, Status};

crate::service! {
    /// Runs one frame. Declared by the engine rather than by a mod, so a
    /// bootstrap reaches whichever scheduler the game loads without
    /// depending on it.
    pub trait Scheduler {
        fn run_frame();
    }
}

/// The systems of one frame, by phase, in the loader's order.
#[derive(Clone, Debug, Default)]
pub struct FramePlan {
    pub phases: Vec<PhasePlan>,
}

#[derive(Clone, Debug)]
pub struct PhasePlan {
    pub name: String,
    pub systems: Vec<PlannedSystem>,
}

#[derive(Clone, Debug)]
pub struct PlannedSystem {
    /// What [`Frame::run`] takes.
    pub id: usize,
    /// `mod::system`.
    pub name: String,
}

/// What [`Frame::run`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ran {
    Ran,
    /// The system failed (it panicked, or touched what it didn't declare);
    /// its mod is now marked failed.
    Failed,
    /// Not run: its mod has failed, or is already running (the bootstrap,
    /// the scheduler itself).
    Skipped,
    /// Not a system of this frame, or no frame is open.
    Refused,
}

/// An open frame. Dropping it ends the frame.
pub struct Frame<'a> {
    ctx: *const ModContext,
    plan: FramePlan,
    _cx: PhantomData<&'a mut ModContext>,
}

/// Opens a frame: publishes what was queued between frames, and returns the
/// plan. `None` if a frame is already open, or the mods are being changed.
pub fn begin<'a>(cx: &'a mut Cx) -> Option<Frame<'a>> {
    let ctx: *const ModContext = cx.raw;
    let plan = unsafe { ((*(*ctx).host).begin_frame)(ctx) }?;
    Some(Frame { ctx, plan, _cx: PhantomData })
}

impl Frame<'_> {
    pub fn plan(&self) -> &FramePlan {
        &self.plan
    }

    /// Runs one system of the plan.
    pub fn run(&self, system: usize) -> Ran {
        unsafe { ((*(*self.ctx).host).run_system)(self.ctx, system) }
    }

    /// A phase boundary: applies the phase's commands and publishes its
    /// events. Call it after each phase's systems, in order.
    pub fn end_phase(&self) {
        unsafe { ((*(*self.ctx).host).end_phase)(self.ctx) }
    }
}

impl Drop for Frame<'_> {
    fn drop(&mut self) {
        unsafe { ((*(*self.ctx).host).end_frame)(self.ctx) }
    }
}

impl Cx<'_> {
    /// Runs one frame: through the loaded [`Scheduler`], or, if none is
    /// loaded, the loader's own sequential frame. What a bootstrap calls.
    pub fn run_frame(&mut self) -> Status {
        match run_frame(self) {
            Ok(()) => Status::OK,
            // A failed scheduler was reported when it failed, and `list`
            // shows it; the game keeps running on the loader's frame until
            // it's reloaded.
            Err(e) if matches!(e.kind, CallErrorKind::NotProvided | CallErrorKind::ProviderFailed) => {
                self.step_mods()
            }
            Err(e) => {
                self.log(format!("no frame: {e}"));
                Status::ERROR
            }
        }
    }
}
