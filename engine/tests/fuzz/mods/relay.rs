//! Builds of `relay`, which uses the flow `fz::Stream` after source:
//!
//! - p passes it (adding 1000 to each value) before peek sees it;
//! - c does the same on channel's v2 layout, and tags it;
//! - t takes it after peek, so its allocation goes back to the bin;
//! - x takes it before source makes it: a plan the check refuses.
//!
//! Its state counts its runs and what it last held. Mirrored by the reload
//! fuzzer's model (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, Mod, Systems, export_mod};

#[cfg(feature = "p")]
const BUILD: &str = "p";
#[cfg(feature = "c")]
const BUILD: &str = "c";
#[cfg(feature = "t")]
const BUILD: &str = "t";
#[cfg(feature = "x")]
const BUILD: &str = "x";

engine_api::mod_state! {
    #[derive(Default)]
    struct Relay {
        ran: u64,
        last: u64,
    }
}

impl Relay {
    #[cfg(any(feature = "p", feature = "c"))]
    fn run(&mut self, _: &mut (), _: &mut Cx, mut stream: engine_api::Pass<channel::Stream>) {
        stream.values.iter_mut().for_each(|v| *v += 1000);
        #[cfg(feature = "c")]
        stream.tag.push('r');
        self.ran += 1;
        self.last = stream.values.iter().sum();
    }

    #[cfg(any(feature = "t", feature = "x"))]
    fn run(&mut self, _: &mut (), _: &mut Cx, stream: engine_api::Take<channel::Stream>) {
        self.ran += 1;
        self.last = stream.values.iter().sum();
    }
}

impl Mod for Relay {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        let run = s.add("run", Self::run);
        #[cfg(any(feature = "p", feature = "c"))]
        run.after("source::make").before("peek::see");
        #[cfg(feature = "t")]
        run.after("source::make").after("peek::see");
        #[cfg(feature = "x")]
        run.before("source::make");
    }

    fn load(&mut self, _: &mut (), _: &mut Cx) {
        // Silent, as peek's are.
        std::panic::set_hook(Box::new(|_| {}));
    }

    /// `get`, `boom`.
    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "get" => Ok(format!("relay:{BUILD} ran={} last={}", self.ran, self.last)),
            "boom" => fz_planted::panic(format!("relay:{BUILD} panics on boom")),
            _ => Err(format!("relay doesn't understand {message:?}")),
        }
    }
}

export_mod!(Relay);
