//! Builds of `source`, which makes the flow `fz::Stream` every frame: a, b
//! and c each fill it with their own values (c, on channel's v2 layout, tags
//! it), and n declares no system, so it stops making the flow. Its state
//! counts the `Make`s that started from nothing and from a kept allocation:
//! what shows a bin dropped or kept. Mirrored by the reload fuzzer's model
//! (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, Mod, Systems, export_mod};

#[cfg(feature = "a")]
const BUILD: &str = "a";
#[cfg(feature = "b")]
const BUILD: &str = "b";
#[cfg(feature = "c")]
const BUILD: &str = "c";
#[cfg(feature = "n")]
const BUILD: &str = "n";

engine_api::mod_state! {
    #[derive(Default)]
    struct Source {
        made: u64,
        fresh: u64,
        kept: u64,
    }
}

impl Source {
    #[cfg(not(feature = "n"))]
    fn make(&mut self, _: &mut (), _: &mut Cx, mut out: engine_api::Make<channel::Stream>) {
        self.made += 1;
        if out.values.capacity() == 0 {
            self.fresh += 1;
        } else {
            self.kept += 1;
        }
        #[cfg(feature = "a")]
        out.values.extend([1, 2, 3]);
        #[cfg(feature = "b")]
        out.values.extend([10, 20]);
        #[cfg(feature = "c")]
        {
            out.values.push(100);
            out.tag.push('c');
        }
    }
}

impl Mod for Source {
    type Transient = ();

    fn systems(_s: &mut Systems<Self>) {
        #[cfg(not(feature = "n"))]
        _s.add("make", Self::make);
    }

    /// `get`, `boom`.
    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "get" => Ok(format!("source:{BUILD} made={} fresh={} kept={}", self.made, self.fresh, self.kept)),
            "boom" => fz_planted::panic(format!("source:{BUILD} panics on boom")),
            _ => Err(format!("source doesn't understand {message:?}")),
        }
    }
}

export_mod!(Source);
