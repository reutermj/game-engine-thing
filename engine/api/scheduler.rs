//! Scheduling policy as a mod: the `Scheduler` service the engine defines,
//! and the frame primitives a scheduler is built from.
//!
//! A bootstrap runs each frame with `cx.run_frame()`, which calls the loaded
//! `Scheduler` if there is one. A scheduler asks the loader for the frame's
//! plan and runs its nodes: each system, then its apply node, which lands
//! the system's changes and publishes its events.
//!
//! ```ignore
//! impl engine_api::scheduler::Scheduler for Sequential {
//!     fn run_frame(&mut self, _: &mut (), cx: &mut Cx) {
//!         let Some(frame) = scheduler::begin(cx) else { return };
//!         for node in &frame.plan().nodes {
//!             frame.run(node.id);
//!         }
//!     }
//! }
//! export_mod!(Sequential, provides = [engine_api::scheduler::Scheduler]);
//! ```
//!
//! The loader keeps what makes a frame safe: the plan, and refusing a
//! system whose mod is already running. The scheduler decides only when
//! each node runs; running them in plan order gives the sequential frame.
//! See docs/architecture/scheduling.md, "Who schedules".

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

/// The nodes of one frame, in the loader's order.
#[derive(Clone, Debug, Default)]
pub struct FramePlan {
    pub nodes: Vec<PlannedNode>,
}

#[derive(Clone, Debug)]
pub struct PlannedNode {
    /// What [`Frame::run`] takes.
    pub id: usize,
    /// `mod::system`, or `apply(mod::system)`.
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

/// Opens a frame and returns its plan. `None` if a frame is already open,
/// or the mods are being changed.
pub fn begin<'a>(cx: &'a mut Cx) -> Option<Frame<'a>> {
    let ctx: *const ModContext = cx.raw;
    let plan = unsafe { ((*(*ctx).host).begin_frame)(ctx) }?;
    Some(Frame { ctx, plan, _cx: PhantomData })
}

impl Frame<'_> {
    pub fn plan(&self) -> &FramePlan {
        &self.plan
    }

    /// Runs one node of the plan.
    pub fn run(&self, node: usize) -> Ran {
        unsafe { ((*(*self.ctx).host).run_node)(self.ctx, node) }
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
