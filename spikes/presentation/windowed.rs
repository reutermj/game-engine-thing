//! SPIKE (get-3hd.1): `spike_windowed`, the realtime bootstrap with one
//! change: once a frame, beside `pump_loader`, it pumps the platform's
//! window (`spike_platform::pump`), and it quits when the window is closed
//! or Escape is pressed. The question it answers: can a window's event
//! loop live inside the bootstrap's loop without owning it?
//!
//! Every 300 frames it logs the pump's and the frame's mean cost.

use std::time::{Duration, Instant};

use clock::Clock;
use engine_api::{Bootstrap, Cx, Entity, Mod, Pumped, Status, export_mod};

const FRAME: Duration = Duration::from_nanos(1_000_000_000 / 60);

engine_api::mod_state! {
    #[derive(Default)]
    struct Windowed {
        frame: u64,
        clock: Option<Entity>,
    }
}

impl Mod for Windowed {
    type Transient = ();

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        if let Some(e) = self.clock.take() {
            cx.world().despawn(e);
        }
    }
}

impl Bootstrap for Windowed {
    fn run(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        cx.log("running the frame loop, pumping the window");
        let mut next_frame = Instant::now();
        let mut last = Instant::now();
        let (mut pump_us, mut work_us, mut loader_us, mut n) = (0u64, 0u64, 0u64, 0u64);
        loop {
            self.frame += 1;
            let now = Instant::now();
            let dt = (now - last).as_secs_f32();
            last = now;

            let window = match spike_platform::pump(cx) {
                Ok(p) => p,
                Err(e) => {
                    cx.log(format!("can't pump the window: {e}"));
                    return Status::ERROR;
                }
            };
            if window.close || window.keys.iter().any(|k| k.contains("Escape")) {
                cx.log("the window was closed: quitting");
                return Status::QUIT;
            }
            pump_us += window.micros as u64;

            let t = Instant::now();
            clock::publish(&mut cx.world(), &mut self.clock, Clock { frame: self.frame, dt });
            cx.run_frame_for(dt);
            work_us += t.elapsed().as_micros() as u64;

            let t = Instant::now();
            match cx.pump_loader(Duration::ZERO, |_, _| Err("bootstrap doesn't take messages".into())) {
                Pumped::Continue => {}
                Pumped::Quit => return Status::QUIT,
                Pumped::Refused => {
                    cx.log("the loader refused to pump; is this build resident?");
                    return Status::ERROR;
                }
            }
            loader_us += t.elapsed().as_micros() as u64;
            n += 1;
            if n == 300 {
                cx.log(format!(
                    "µs a frame, mean of 300: window pump {:.1}, frame {:.1}, loader pump {:.1}",
                    pump_us as f64 / 300.0,
                    work_us as f64 / 300.0,
                    loader_us as f64 / 300.0
                ));
                (pump_us, work_us, loader_us, n) = (0, 0, 0, 0);
            }

            next_frame += FRAME;
            let now = Instant::now();
            if next_frame > now {
                std::thread::sleep(next_frame - now);
            } else {
                next_frame = now;
            }
        }
    }
}

export_mod!(Windowed, bootstrap);
