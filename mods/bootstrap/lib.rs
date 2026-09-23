//! Owns the frame loop in real time: paces frames at a fixed rate, publishes
//! the `Clock`, steps every other mod, and hands the loader control once a
//! frame to serve requests and swap builds.
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
        loop {
            self.frame += 1;
            let clock = Clock { frame: self.frame, dt: FRAME.as_secs_f32() };
            let mut world = cx.world();
            let entity = *self.clock.get_or_insert_with(|| world.spawn());
            world.insert(entity, clock);
            cx.step_mods();

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
                // Fell behind (e.g. paused in a debugger); don't try to catch up.
                next_frame = now;
            }
        }
    }
}

export_mod!(Realtime, bootstrap);
