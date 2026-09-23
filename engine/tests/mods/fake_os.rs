//! A resident bootstrap shaped like a windowing library's event loop, without
//! a window. `os::EventLoop::run` takes over the thread and calls back once
//! per event until told to exit, as winit's does, so the bootstrap can't
//! return to the loader between frames. It doesn't need to: on `AboutToWait`,
//! once per frame, it steps the mods and pumps the loader.
//!
//! Messages sent to it arrive while it's running, through the pump's handler:
//! `key <c>` makes the fake OS deliver a key press, `keys` lists those
//! delivered, `frames` counts frames, `size` is the window's.

use std::collections::VecDeque;
use std::time::Duration;

use engine_api::{Bootstrap, Cx, Mod, Pumped, Status, export_mod};

/// The fake OS: an event queue, fed by `EventLoop::run` and by whatever
/// injects input (here, messages).
mod os {
    use std::collections::VecDeque;

    pub enum Event {
        Resized(u32, u32),
        Key(char),
        /// The queue is empty: the point to run a frame.
        AboutToWait,
    }

    pub enum Flow {
        Continue,
        Exit,
    }

    /// Runs until `callback` returns `Flow::Exit`. The callback gets the
    /// queue, to post events to, as a real loop's proxy does.
    pub fn run(mut callback: impl FnMut(&mut VecDeque<Event>, Event) -> Flow) {
        let mut queue = VecDeque::from([Event::Resized(640, 480)]);
        loop {
            let event = queue.pop_front().unwrap_or(Event::AboutToWait);
            let waited = matches!(event, Event::AboutToWait);
            if let Flow::Exit = callback(&mut queue, event) {
                return;
            }
            if waited {
                // Vsync.
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        }
    }
}

engine_api::mod_state! {
    #[derive(Default)]
    struct FakeOs {
        frames: u64,
        size: (u32, u32),
        keys: String,
    }
}

impl FakeOs {
    fn handle(&self, queue: &mut VecDeque<os::Event>, message: &str) -> Result<String, String> {
        let mut words = message.split_whitespace();
        match (words.next(), words.next(), words.next()) {
            (Some("key"), Some(key), None) if key.chars().count() == 1 => {
                queue.push_back(os::Event::Key(key.chars().next().unwrap()));
                Ok("queued".into())
            }
            (Some("keys"), None, None) => Ok(self.keys.clone()),
            (Some("frames"), None, None) => Ok(self.frames.to_string()),
            (Some("size"), None, None) => Ok(format!("{}x{}", self.size.0, self.size.1)),
            _ => Err(format!("fake_os doesn't understand {message:?}")),
        }
    }
}

impl Mod for FakeOs {
    type Transient = ();

    /// A message delivered directly (`Engine::send`), not through the pump.
    /// `pump` pumps from here, which the loader refuses: a delivery is under
    /// way.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "pump" => Ok(format!("{:?}", cx.pump_loader(Duration::ZERO, |_, _| Err(String::new())))),
            _ => Err(format!("fake_os doesn't understand {message:?} outside its loop")),
        }
    }
}

impl Bootstrap for FakeOs {
    fn run(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        let mut status = Status::QUIT;
        os::run(|queue, event| match event {
            os::Event::Resized(w, h) => {
                self.size = (w, h);
                os::Flow::Continue
            }
            os::Event::Key(key) => {
                self.keys.push(key);
                os::Flow::Continue
            }
            os::Event::AboutToWait => {
                self.frames += 1;
                cx.step_mods();
                match cx.pump_loader(Duration::ZERO, |_, message| self.handle(queue, message)) {
                    Pumped::Continue => os::Flow::Continue,
                    Pumped::Quit => os::Flow::Exit,
                    Pumped::Refused => {
                        status = Status::ERROR;
                        os::Flow::Exit
                    }
                }
            }
        });
        status
    }
}

export_mod!(FakeOs, bootstrap);
