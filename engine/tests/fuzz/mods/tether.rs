//! `tether`, resident and depending on `anchor`: loadable only beside a
//! resident anchor. Its state counts `ping`s.

use engine_api::{Cx, Mod, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Tether {
        pings: u32,
    }
}

impl Mod for Tether {
    type Transient = ();

    /// `ping`, `get`.
    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "ping" => self.pings += 1,
            "get" => {}
            _ => return Err(format!("tether doesn't understand {message:?}")),
        }
        Ok(format!("tether:{} pings={}", anchor::NAME, self.pings))
    }
}

export_mod!(Tether);
