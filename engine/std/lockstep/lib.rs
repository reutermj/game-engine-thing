//! A bootstrap mod for games played in turns, by an agent or a test: time only
//! moves when something asks. `modctl send lockstep step 30` runs 30 frames
//! as fast as they compute and replies with the frame number; between
//! requests, nothing runs.
//!
//! Every frame is the same length, so a game is deterministic given its
//! inputs and the frames between them.
//!
//! Like every bootstrap it's resident and runs the whole session in one
//! `run`: here, blocked in the loader's pump until a request arrives.

use clock::Clock;
use std::time::Duration;

use engine_api::{Bootstrap, Cx, Entity, Mod, Pumped, Status, export_mod};

/// Each frame covers the same time as one of the real-time bootstrap's.
const DT: f32 = 1.0 / 60.0;
/// Most frames one request may run, so a typo can't hang the engine.
const MAX_STEPS: u64 = 100_000;

engine_api::mod_state! {
    #[derive(Default)]
    struct Lockstep {
        frame: u64,
        clock: Option<Entity>,
    }
}

impl Lockstep {
    /// One frame covering `dt` seconds: fixed-rate phases take as many
    /// steps as that makes.
    fn run_frame(&mut self, cx: &mut Cx, dt: f32) {
        self.frame += 1;
        clock::publish(&mut cx.world(), &mut self.clock, Clock { frame: self.frame, dt });
        cx.run_frame_for(dt);
    }

    fn handle(&mut self, cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut words = message.split_whitespace();
        match (words.next(), words.next(), words.next(), words.next()) {
            // `step N at FPS`: frames of another length, as a game at that
            // frame rate would run them. Fixed-rate phases take the same
            // steps whatever the frames are.
            (Some("step"), n, at, fps) => {
                let n: u64 = match n {
                    None => 1,
                    Some(n) => n.parse().map_err(|_| format!("not a frame count: {n:?}"))?,
                };
                let dt = match (at, fps) {
                    (None, None) => DT,
                    (Some("at"), Some(fps)) => {
                        let fps: f32 = fps.parse().map_err(|_| format!("not a frame rate: {fps:?}"))?;
                        if fps <= 0.0 {
                            return Err("a frame rate is frames per second".into());
                        }
                        1.0 / fps
                    }
                    _ => return Err("usage: step [frames] [at fps] | frame".into()),
                };
                if n > MAX_STEPS {
                    return Err(format!("at most {MAX_STEPS} frames per step"));
                }
                for _ in 0..n {
                    self.run_frame(cx, dt);
                }
                Ok(format!("frame {}", self.frame))
            }
            (Some("frame"), None, None, None) => Ok(format!("frame {}", self.frame)),
            _ => Err("usage: step [frames] [at fps] | frame".into()),
        }
    }
}

impl Mod for Lockstep {
    type Transient = ();

    /// A message delivered directly, as a test driving an `Engine` does.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        self.handle(cx, message)
    }

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        if let Some(e) = self.clock.take() {
            cx.world().despawn(e);
        }
    }
}

impl Bootstrap for Lockstep {
    /// The session: serve the loader's requests as they come, running frames
    /// when a message asks for them.
    fn run(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        loop {
            match cx.pump_loader(Duration::from_secs(1), |cx, message| self.handle(cx, message)) {
                Pumped::Continue => {}
                Pumped::Quit => return Status::QUIT,
                Pumped::Refused => {
                    cx.log("the loader refused to pump; is this build resident?");
                    return Status::ERROR;
                }
            }
        }
    }
}

export_mod!(Lockstep, bootstrap);
