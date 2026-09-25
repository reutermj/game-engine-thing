//! Builds of `hearer`, which depends on `herald`: it reads `fz::Note`s in
//! `input` (before herald's `emit`) and in `late` (after it), keeping what
//! each system saw in its state, and calls herald's service, from its
//! `load` and on `ask <n>`. a and b are built against herald's interface v1
//! (b marks what it saw), c against v2, with a field added to its state.
//! Mirrored by the reload fuzzer's model (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, EventReader, Mod, Systems, export_mod, phase};
use herald::Note;

#[cfg(feature = "a")]
const BUILD: &str = "a";
#[cfg(feature = "b")]
const BUILD: &str = "b";
#[cfg(feature = "c")]
const BUILD: &str = "c";

#[cfg(not(feature = "c"))]
engine_api::mod_state! {
    #[derive(Default)]
    struct Hearer {
        early: Vec<String>,
        late: Vec<String>,
        at_load: String,
        loads: u32,
    }
}

#[cfg(feature = "c")]
engine_api::mod_state! {
    #[derive(Default)]
    struct Hearer {
        early: Vec<String>,
        late: Vec<String>,
        at_load: String,
        loads: u32,
        heard: u64,
    }
}

fn entry(note: &Note) -> String {
    #[cfg(not(feature = "c"))]
    let entry = format!("{}@{}", note.n, note.from);
    #[cfg(feature = "c")]
    let entry = format!("{}@{}#{}", note.n, note.from, note.weight);
    if cfg!(feature = "b") { format!("B{entry}") } else { entry }
}

impl Hearer {
    fn heard(&mut self, notes: &[Note]) -> Vec<String> {
        #[cfg(feature = "c")]
        {
            self.heard += notes.len() as u64;
        }
        notes.iter().map(entry).collect()
    }

    fn early(&mut self, _: &mut (), _: &mut Cx, mut notes: EventReader<Note>) {
        let seen = self.heard(notes.read());
        self.early.extend(seen);
    }

    fn late(&mut self, _: &mut (), _: &mut Cx, mut notes: EventReader<Note>) {
        let seen = self.heard(notes.read());
        self.late.extend(seen);
    }
}

impl Mod for Hearer {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("early", Self::early).phase(phase::INPUT);
        s.add("late", Self::late).phase(phase::LATE);
    }

    /// Its dependencies load first, so herald's current build answers.
    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        self.loads += 1;
        self.at_load = match herald::build(cx) {
            Ok(build) => build,
            Err(e) => format!("err {:?}", e.kind),
        };
    }

    /// `dump`, `clear`, `ask <n>`, `boom`.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let (verb, n) = message.split_once(' ').unwrap_or((message, ""));
        match verb {
            "dump" => {
                #[cfg(not(feature = "c"))]
                let heard = "-".to_string();
                #[cfg(feature = "c")]
                let heard = self.heard.to_string();
                Ok(format!(
                    "hearer:{BUILD} loads={} at_load={} heard={heard} early=[{}] late=[{}]",
                    self.loads,
                    self.at_load,
                    self.early.join(","),
                    self.late.join(",")
                ))
            }
            "clear" => {
                self.early.clear();
                self.late.clear();
                Ok(format!("hearer:{BUILD} cleared"))
            }
            "ask" => {
                let n = n.parse::<u64>().map_err(|_| format!("hearer: bad {message:?}"))?;
                Ok(match herald::bump(cx, n) {
                    Ok(calls) => format!("hearer:{BUILD} got {calls}"),
                    Err(e) => format!("hearer:{BUILD} err {:?}", e.kind),
                })
            }
            "boom" => fz_planted::panic(format!("hearer:{BUILD} panics on boom")),
            _ => Err(format!("hearer doesn't understand {message:?}")),
        }
    }
}

export_mod!(Hearer);
