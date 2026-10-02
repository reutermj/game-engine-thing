//! Builds of `peek`, which only sees the flow `fz::Stream`, after source
//! makes it: a on channel's v1 layout, c on v2's, whose tag it counts. Its
//! state is how many times it saw the flow and what it last saw. With no
//! value to see (source failed), it panics at fetch, which fails it.
//! Mirrored by the reload fuzzer's model (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, Mod, See, Systems, export_mod};

#[cfg(feature = "a")]
const BUILD: &str = "a";
#[cfg(feature = "c")]
const BUILD: &str = "c";

engine_api::mod_state! {
    #[derive(Default)]
    struct Peek {
        saw: u64,
        last: u64,
    }
}

impl Peek {
    fn see(&mut self, _: &mut (), _: &mut Cx, stream: See<channel::Stream>) {
        self.saw += 1;
        self.last = stream.values.iter().sum::<u64>();
        #[cfg(feature = "c")]
        {
            self.last += stream.tag.len() as u64 * 1_000_000;
        }
    }
}

impl Mod for Peek {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("see", Self::see).after("source::make");
    }

    fn load(&mut self, _: &mut (), _: &mut Cx) {
        // Silent, as fz_planted's panics are: a fetch with nothing to see
        // panics every frame the fuzzer fails source.
        std::panic::set_hook(Box::new(|_| {}));
    }

    /// `get`, `boom`.
    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "get" => Ok(format!("peek:{BUILD} saw={} last={}", self.saw, self.last)),
            "boom" => fz_planted::panic(format!("peek:{BUILD} panics on boom")),
            _ => Err(format!("peek doesn't understand {message:?}")),
        }
    }
}

export_mod!(Peek);
