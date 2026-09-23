//! Calls the resident `vault` and replies with what it says: `build`,
//! `made` (transient parts made), `ticks`.

use engine_api::{Cx, Mod, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Teller {}
}

impl Mod for Teller {
    type Transient = ();

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let result = match message {
            "build" => vault::build(cx),
            "made" => vault::transients_made(cx).map(|n| n.to_string()),
            "ticks" => vault::ticks(cx).map(|n| n.to_string()),
            _ => return Err(format!("unknown command {message:?}")),
        };
        result.map_err(|e| e.to_string())
    }
}

export_mod!(Teller);
