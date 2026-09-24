//! `pinger` sends a `Ping` numbered by frame from `simulate`, and one with
//! any number on the message `ping <n>`, between frames.

use engine_api::{Cx, EventWriter, Mod, Systems, export_mod, phase};
use test_probe::Ping;

engine_api::mod_state! {
    #[derive(Default)]
    struct Pinger {
        frame: u64,
    }
}

impl Pinger {
    fn ping(&mut self, _: &mut (), _: &mut Cx, pings: EventWriter<Ping>) {
        self.frame += 1;
        pings.send(Ping { n: self.frame });
    }
}

impl Mod for Pinger {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("ping", Self::ping).phase(phase::SIMULATE);
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let n = message.strip_prefix("ping ").and_then(|n| n.parse().ok()).ok_or("usage: ping <n>")?;
        cx.send_event(Ping { n });
        Ok(String::new())
    }
}

export_mod!(Pinger);
