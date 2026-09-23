//! A bootstrap mod for games played in turns, by an agent or a test: time only
//! moves when something asks. `modctl send lockstep step 30` runs 30 frames
//! as fast as they compute and replies with the frame number; between
//! requests, nothing runs.
//!
//! Every frame is the same length, so a game is deterministic given its
//! inputs and the frames between them.

use clock::Clock;
use engine_api::{Cx, Entity, Mod, Status, export_mod};

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
    fn run_frame(&mut self, cx: &mut Cx) {
        self.frame += 1;
        let mut world = cx.world();
        let entity = *self.clock.get_or_insert_with(|| world.spawn());
        world.insert(entity, Clock { frame: self.frame, dt: DT });
        cx.step_mods();
    }
}

impl Mod for Lockstep {
    type Transient = ();

    /// Called by the loader's loop between requests: nothing to do, and a
    /// short sleep so an idle engine doesn't spin a core.
    fn step(&mut self, _: &mut (), _cx: &mut Cx) -> Status {
        std::thread::sleep(std::time::Duration::from_millis(2));
        Status::OK
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut words = message.split_whitespace();
        match (words.next(), words.next(), words.next()) {
            (Some("step"), n, None) => {
                let n: u64 = match n {
                    None => 1,
                    Some(n) => n.parse().map_err(|_| format!("not a frame count: {n:?}"))?,
                };
                if n > MAX_STEPS {
                    return Err(format!("at most {MAX_STEPS} frames per step"));
                }
                for _ in 0..n {
                    self.run_frame(cx);
                }
                Ok(format!("frame {}", self.frame))
            }
            (Some("frame"), None, None) => Ok(format!("frame {}", self.frame)),
            _ => Err("usage: step [frames] | frame".into()),
        }
    }

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        if let Some(e) = self.clock.take() {
            cx.world().despawn(e);
        }
    }
}

export_mod!(Lockstep);
