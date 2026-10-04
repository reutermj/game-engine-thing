//! SPIKE (get-3hd.1): `spike_lockstep_window`, the lockstep bootstrap
//! (`//engine/std/lockstep`) with a spectator window. Its mod name is
//! `lockstep`, so an agent drives it exactly as it drives plain lockstep:
//! `modctl send lockstep step 30`. Two changes:
//!
//! - **The window stays pumped while idle.** Plain lockstep blocks in
//!   `pump_loader` for up to a second. This waits at most `IDLE`, then
//!   pumps the platform's window; when the pump saw events (an expose, a
//!   resize), it asks the presenter to draw the last frame again
//!   (`spike_platform::redraw`), since no frame will. The close button or
//!   Escape quits, between steps or during one.
//! - **Pacing.** With a window and pacing on (the default), a step's frames
//!   are spread out in real time, each `dt / speed` seconds apart, so a
//!   spectator sees `step 30` played out rather than its last frame. It
//!   only sleeps between frames: every frame covers the same `dt` either
//!   way, so pacing changes when frames run, never what they compute.
//!   `pace off` runs steps as fast as they compute, as plain lockstep does.
//!
//! Messages: `step [frames] [at fps]` and `frame`, as lockstep;
//! `pace on|off|<speed>` (a speed of 0.5 is half speed); `pace` reports.

use std::time::{Duration, Instant};

use clock::Clock;
use engine_api::{Bootstrap, Cx, Entity, Mod, Pumped, Status, export_mod};

const DT: f32 = 1.0 / 60.0;
const MAX_STEPS: u64 = 100_000;
/// The longest the window goes unpumped while no requests come: a frame
/// at 60 Hz, so a drag or a close feels immediate.
const IDLE: Duration = Duration::from_millis(16);

engine_api::mod_state! {
    #[derive(Default)]
    struct Lockstep {
        frame: u64,
        clock: Option<Entity>,
        /// Pacing is on unless this is set: the default state is the
        /// spectator's.
        unpaced: bool,
        /// Real-time speed while paced; 0 means 1.
        speed: f32,
        /// The window was closed during a step: quit once it's replied.
        quit: bool,
    }
}

/// When the last frame ran, which the next paced frame waits a frame
/// after. Transient, not state: an `Instant` isn't a `FieldType`, and a
/// resident mod is never carried to another build anyway.
#[derive(Default)]
pub struct Pace {
    last_frame: Option<Instant>,
}

/// What a pump of the window found.
enum Window {
    /// No platform, or no window: nothing to pace or keep drawn.
    None,
    Open {
        events: u32,
    },
    Closed,
}

fn pump_window(cx: &mut Cx) -> Window {
    match spike_platform::pump(cx) {
        Ok(p) if p.close || p.keys.iter().any(|k| k.contains("Escape")) => Window::Closed,
        Ok(p) if p.width > 0 => Window::Open { events: p.events },
        _ => Window::None,
    }
}

impl Lockstep {
    fn run_frame(&mut self, cx: &mut Cx, dt: f32) {
        self.frame += 1;
        clock::publish(&mut cx.world(), &mut self.clock, Clock { frame: self.frame, dt });
        cx.run_frame_for(dt);
    }

    fn speed(&self) -> f32 {
        if self.speed > 0.0 { self.speed } else { 1.0 }
    }

    fn step(&mut self, pace: &mut Pace, cx: &mut Cx, n: u64, dt: f32) -> String {
        let window = pump_window(cx);
        if let Window::Closed = window {
            self.quit = true;
            return format!("frame {} (the window was closed: quitting)", self.frame);
        }
        let paced = !self.unpaced && matches!(window, Window::Open { .. });
        let gap = Duration::from_secs_f32(dt / self.speed());
        for i in 0..n {
            // A frame a gap after the last one, the last step's included, so
            // an agent that sends `step 6` ten times a second shows the same
            // pace as one `step 60`. A slower agent's next step starts at
            // once: the wait is only ever up to a frame.
            if paced && let Some(last) = pace.last_frame {
                let next = last + gap;
                let now = Instant::now();
                if next > now {
                    std::thread::sleep(next - now);
                }
                if let Window::Closed = pump_window(cx) {
                    self.quit = true;
                    return format!("frame {} (the window was closed after {i} of {n} frames: quitting)", self.frame);
                }
            }
            // Taken as the frame starts, so a slow frame isn't paid twice.
            pace.last_frame = Some(Instant::now());
            self.run_frame(cx, dt);
        }
        format!("frame {}", self.frame)
    }

    fn handle(&mut self, pace: &mut Pace, cx: &mut Cx, message: &str) -> Result<String, String> {
        let mut words = message.split_whitespace();
        match (words.next(), words.next(), words.next(), words.next()) {
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
                    _ => return Err(USAGE.into()),
                };
                if n > MAX_STEPS {
                    return Err(format!("at most {MAX_STEPS} frames per step"));
                }
                Ok(self.step(pace, cx, n, dt))
            }
            (Some("frame"), None, None, None) => Ok(format!("frame {}", self.frame)),
            (Some("pace"), arg, None, None) => {
                match arg {
                    None => {}
                    Some("on") => self.unpaced = false,
                    Some("off") => self.unpaced = true,
                    Some(speed) => {
                        let speed: f32 = speed.parse().map_err(|_| USAGE.to_string())?;
                        if !(speed > 0.0 && speed.is_finite()) {
                            return Err("a speed is a positive number (1 is real time)".into());
                        }
                        (self.unpaced, self.speed) = (false, speed);
                    }
                }
                Ok(if self.unpaced {
                    "pacing off: steps run as fast as they compute".into()
                } else {
                    format!("pacing on at {}x real time", self.speed())
                })
            }
            _ => Err(USAGE.into()),
        }
    }
}

const USAGE: &str = "usage: step [frames] [at fps] | frame | pace [on|off|<speed>]";

impl Mod for Lockstep {
    type Transient = Pace;

    fn message(&mut self, pace: &mut Pace, cx: &mut Cx, message: &str) -> Result<String, String> {
        self.handle(pace, cx, message)
    }

    fn close(&mut self, _: &mut Pace, cx: &mut Cx) {
        if let Some(e) = self.clock.take() {
            cx.world().despawn(e);
        }
    }
}

impl Bootstrap for Lockstep {
    fn run(&mut self, pace: &mut Pace, cx: &mut Cx) -> Status {
        cx.log("lockstep with a spectator window: waiting for `step`");
        loop {
            match cx.pump_loader(IDLE, |cx, message| self.handle(pace, cx, message)) {
                Pumped::Continue => {}
                Pumped::Quit => return Status::QUIT,
                Pumped::Refused => {
                    cx.log("the loader refused to pump; is this build resident?");
                    return Status::ERROR;
                }
            }
            if self.quit {
                cx.log("the window was closed: quitting");
                return Status::QUIT;
            }
            match pump_window(cx) {
                Window::Closed => {
                    cx.log("the window was closed: quitting");
                    return Status::QUIT;
                }
                Window::Open { events } if events > 0 => {
                    // Not loaded, or failed: the window just stays as it was.
                    let _ = spike_platform::redraw(cx);
                }
                _ => {}
            }
        }
    }
}

export_mod!(Lockstep, bootstrap);
