//! Owns the frame loop in real time: paces frames at a fixed rate, publishes
//! the `Clock`, runs each frame for the time the last one took (so
//! fixed-rate phases catch up after a slow frame), and hands the loader
//! control once a frame to serve requests and swap builds.
//!
//! The whole session is this mod's `Bootstrap::run`, which returns when the engine
//! is asked to quit. It's resident, so the loader never swaps it out while
//! it's running.

use std::time::{Duration, Instant};

use clock::Clock;
use engine_api::{Bootstrap, Cx, Entity, Mod, Pumped, Status, export_mod};

const FRAME: Duration = Duration::from_nanos(1_000_000_000 / 60);

engine_api::mod_state! {
    #[derive(Default)]
    struct Realtime {
        frame: u64,
        clock: Option<Entity>,
    }
}

impl Mod for Realtime {
    type Transient = ();

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        if let Some(e) = self.clock.take() {
            cx.world().despawn(e);
        }
    }
}

impl Bootstrap for Realtime {
    fn run(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        cx.log("running the frame loop");
        let mut next_frame = Instant::now();
        let mut last = Instant::now();
        loop {
            self.frame += 1;
            let now = Instant::now();
            // What this frame covers: the real time since the last began.
            // The loader caps how many fixed steps that can buy.
            let dt = (now - last).as_secs_f32();
            last = now;
            clock::publish(&mut cx.world(), &mut self.clock, Clock { frame: self.frame, dt });
            cx.run_frame_for(dt);

            // Between frames, where nothing but this mod is running.
            match cx.pump_loader(Duration::ZERO, |_, _| Err("bootstrap doesn't take messages".into())) {
                Pumped::Continue => {}
                Pumped::Quit => return Status::QUIT,
                Pumped::Refused => {
                    cx.log("the loader refused to pump; is this build resident?");
                    return Status::ERROR;
                }
            }

            next_frame += FRAME;
            let now = Instant::now();
            if next_frame > now {
                std::thread::sleep(next_frame - now);
            } else {
                // Fell behind: the next frame starts now, and covers the time
                // lost, which fixed-rate phases catch up in steps.
                next_frame = now;
            }
        }
    }
}

export_mod!(Realtime, bootstrap);
