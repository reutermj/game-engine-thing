//! Builds of `herald`, which sends `fz::Note`s (between frames on `now <n>`,
//! and from its `emit` system for each `queue <n>`) and provides the
//! `Herald` service. a, b and p build interface v1, c v2; b sends a frame's
//! notes newest first, and p panics in `bump`. Mirrored by the reload
//! fuzzer's model (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, EventWriter, Mod, Systems, export_mod};
use herald::Note;

#[cfg(feature = "a")]
const BUILD: &str = "a";
#[cfg(feature = "b")]
const BUILD: &str = "b";
#[cfg(feature = "c")]
const BUILD: &str = "c";
#[cfg(feature = "p")]
const BUILD: &str = "p";

engine_api::mod_state! {
    #[derive(Default)]
    struct HeraldMod {
        calls: u64,
        pending: Vec<u64>,
        loads: u32,
    }
}

fn note(n: u64) -> Note {
    #[cfg(not(feature = "c"))]
    let note = Note { n, from: format!("herald:{BUILD}") };
    #[cfg(feature = "c")]
    let note = Note { n, from: format!("herald:{BUILD}"), weight: 7 };
    note
}

impl HeraldMod {
    fn emit(&mut self, _: &mut (), _: &mut Cx, notes: EventWriter<Note>) {
        let mut pending = std::mem::take(&mut self.pending);
        if cfg!(feature = "b") {
            pending.reverse();
        }
        for n in pending {
            notes.send(note(n));
        }
    }
}

impl Mod for HeraldMod {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("emit", Self::emit);
    }

    fn load(&mut self, _: &mut (), _: &mut Cx) {
        self.loads += 1;
    }

    /// `now <n>`, `queue <n>`, `get`, `boom`.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let (verb, n) = message.split_once(' ').unwrap_or((message, ""));
        let n = || n.parse::<u64>().map_err(|_| format!("herald: bad {message:?}"));
        match verb {
            "now" => {
                let n = n()?;
                cx.send_event(note(n));
                Ok(format!("herald:{BUILD} sent {n}"))
            }
            "queue" => {
                self.pending.push(n()?);
                Ok(format!("herald:{BUILD} queued {}", self.pending.len()))
            }
            "get" => Ok(format!("herald:{BUILD} calls={} loads={} pending={:?}", self.calls, self.loads, self.pending)),
            "boom" => fz_planted::panic(format!("herald:{BUILD} panics on boom")),
            _ => Err(format!("herald doesn't understand {message:?}")),
        }
    }
}

impl herald::Herald for HeraldMod {
    fn bump(&mut self, _: &mut (), _: &mut Cx, by: u64) -> u64 {
        if cfg!(feature = "p") {
            fz_planted::panic(format!("herald:{BUILD} panics in bump"));
        }
        self.calls = self.calls.wrapping_add(by);
        self.calls
    }

    fn build(&mut self, _: &mut (), _: &mut Cx) -> String {
        format!("herald:{BUILD}")
    }
}

export_mod!(HeraldMod, provides = [herald::Herald]);
