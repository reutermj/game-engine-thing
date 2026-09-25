//! Builds of `anchor`: a and b are resident, so once one is loaded the
//! other takes a restart; loose is a's code built reloadable. `tether`,
//! resident too, depends on it, which only a resident anchor can satisfy.
//! Its state counts `ping`s. Mirrored by the reload fuzzer's model
//! (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, Mod, export_mod};

#[cfg(feature = "a")]
const BUILD: &str = "a";
#[cfg(feature = "b")]
const BUILD: &str = "b";
#[cfg(feature = "loose")]
const BUILD: &str = "loose";

engine_api::mod_state! {
    #[derive(Default)]
    struct Anchor {
        pings: u32,
        loads: u32,
    }
}

impl Mod for Anchor {
    type Transient = ();

    fn load(&mut self, _: &mut (), _: &mut Cx) {
        self.loads += 1;
    }

    /// `ping`, `get`.
    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "ping" => self.pings += 1,
            "get" => {}
            _ => return Err(format!("anchor doesn't understand {message:?}")),
        }
        Ok(format!("{}:{BUILD} pings={} loads={}", anchor::NAME, self.pings, self.loads))
    }
}

export_mod!(Anchor);
