//! The default `Scheduler`: runs the frame's nodes (each system, then its
//! apply node) one at a time, in the loader's order. The same frame the loader runs itself when no scheduler
//! is loaded, built from the primitives any scheduler uses; a parallel one
//! differs only in when it calls `Frame::run`.
//!
//! Messages: `time on` times every node it runs, by name, until `time off`;
//! `times` replies each node's µs a frame since `time on` (or `time reset`),
//! in plan order. What a benchmark breaks a step down by past the systems'
//! own timers: apply nodes (structural changes, re-sorts) included.

use std::time::Instant;

use engine_api::scheduler::{self, Scheduler};
use engine_api::{Cx, Mod, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Sequential {}
}

/// The nodes' times while timing is on. Off, it costs a branch a frame.
#[derive(Default)]
pub struct Timer {
    on: bool,
    frames: u64,
    /// Each node's name and nanoseconds, in the order first run.
    nodes: Vec<(String, u64)>,
}

impl Mod for Sequential {
    type Transient = Timer;

    fn message(&mut self, timer: &mut Timer, _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "time on" => timer.on = true,
            "time off" => timer.on = false,
            "time reset" => (timer.frames, timer.nodes) = (0, Vec::new()),
            "times" => {
                let per = |ns: u64| ns as f64 / timer.frames.max(1) as f64 / 1e3;
                return Ok(timer.nodes.iter().map(|(name, ns)| format!("{name} {:.1}", per(*ns))).collect::<Vec<_>>().join("\n"));
            }
            _ => return Err(format!("sequential takes `time on|off|reset` and `times`, not {message:?}")),
        }
        Ok(String::new())
    }
}

impl Scheduler for Sequential {
    fn run_frame(&mut self, timer: &mut Timer, cx: &mut Cx) {
        let Some(frame) = scheduler::begin(cx) else { return };
        if !timer.on {
            for node in &frame.plan().nodes {
                frame.run(node.id);
            }
            return;
        }
        timer.frames += 1;
        for (k, node) in frame.plan().nodes.iter().enumerate() {
            let start = Instant::now();
            frame.run(node.id);
            let ns = start.elapsed().as_nanos() as u64;
            // The plan is the same frame after frame but for a load: found
            // by place first, then by name.
            let at = match timer.nodes.get(k) {
                Some((name, _)) if *name == node.name => k,
                _ => timer.nodes.iter().position(|(name, _)| *name == node.name).unwrap_or_else(|| {
                    timer.nodes.push((node.name.clone(), 0));
                    timer.nodes.len() - 1
                }),
            };
            timer.nodes[at].1 += ns;
        }
    }
}

export_mod!(Sequential, provides = [scheduler::Scheduler]);
