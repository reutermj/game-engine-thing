//! Observations (presentation.md, D3 and D7's `observe`): how an agent
//! asks to see a game, and what it gets back. One request, one answer, in
//! one of these views, every one carrying the events since the asker's
//! last observation:
//!
//! - **text**: a character grid of the labelled entities with axis
//!   coordinates and a legend made from their labels, *and* the same
//!   entities as a list with exact coordinates, since models confuse rows
//!   and columns on a grid alone;
//! - **json**: the entities (id, label, name, position, size, velocity) and
//!   each seat's held actions and metrics, for an LLM;
//! - **vector**: a fixed-length list of numbers with a schema, from the
//!   game's declared slots (`play::Playable::observed`), for RL;
//! - **pixels**: requested here, answered by a pixel presenter (M6); until
//!   one exists, an observer answers that it can't.
//!
//! The observer that answers is a presenter mod reading the draw list
//! (`present::DrawList`'s labels); this crate is what it and the agent's
//! side agree on, and has no dependency on either. Resident, so a
//! bootstrap's "step, then observe" can call `Observe`.
//!
//! Units are the world's, as the game's labels and camera are. Velocities
//! are what the observer measures between frames, since presentation
//! carries positions, not motion.

use engine_api::Entity;
use play::{GameEvent, Playable};

mod json;
mod views;

pub use views::*;

/// What a request asks to see.
#[derive(Clone, Debug, PartialEq)]
pub enum View {
    /// `cell` is a character's size in world units; `region` the world
    /// rectangle (x, y, w, h) to draw, the camera's view by default.
    Text {
        cell: [f32; 2],
        region: Option<[f32; 4]>,
    },
    Json,
    Vector,
    /// RGB8, `width` by `height`, letterboxed; `overlay` draws labels and
    /// ids on the image, as the Gemini Plays Pokémon harness did.
    Pixels {
        width: u32,
        height: u32,
        overlay: bool,
    },
}

/// A view, and who's asking: each seat (and the anonymous observer) has
/// its own "since the last observation".
#[derive(Clone, Debug, PartialEq)]
pub struct Request {
    pub view: View,
    pub seat: Option<String>,
}

const USAGE: &str =
    "views: text [cell <w> [<h>]] [region <x> <y> <w> <h>] | json | vector | pixels <width> <height> [overlay]; then [as <seat>]";

/// The largest pixel side a request may ask for: a 4k frame's width. More
/// is a typo, or a frame no model wants.
pub const MAX_PIXELS: u32 = 4096;

impl Request {
    /// Parses a request as an agent types it: `text`, `text cell 2 region 0
    /// 0 40 20 as left`, `json as right`, `pixels 1008 756 overlay`.
    pub fn parse(request: &str) -> Result<Request, String> {
        let mut words: Vec<&str> = request.split_whitespace().collect();
        let mut seat = None;
        if let Some(at) = words.iter().position(|w| *w == "as") {
            match &words[at..] {
                [_, name] => seat = Some(name.to_string()),
                _ => return Err(format!("`as` takes one seat name, at the end. {USAGE}")),
            }
            words.truncate(at);
        }
        let num = |w: &str| w.parse::<f32>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("`{w}` isn't a number. {USAGE}"));
        let view = match words.as_slice() {
            ["text", rest @ ..] => {
                let (mut cell, mut region) = ([1.0, 1.0], None);
                let mut rest = rest;
                while let Some((&word, tail)) = rest.split_first() {
                    match (word, tail) {
                        ("cell", [w, h, tail @ ..]) if h.parse::<f32>().is_ok() => {
                            cell = [num(w)?, num(h)?];
                            rest = tail;
                        }
                        ("cell", [w, tail @ ..]) => {
                            cell = [num(w)?; 2];
                            rest = tail;
                        }
                        ("region", [x, y, w, h, tail @ ..]) => {
                            let r = [num(x)?, num(y)?, num(w)?, num(h)?];
                            if r[2] <= 0.0 || r[3] <= 0.0 {
                                return Err(format!("a region needs a width and height above 0. {USAGE}"));
                            }
                            region = Some(r);
                            rest = tail;
                        }
                        _ => return Err(format!("can't read `{}`. {USAGE}", rest.join(" "))),
                    }
                }
                if cell[0] <= 0.0 || cell[1] <= 0.0 {
                    return Err(format!("a cell needs a width and height above 0. {USAGE}"));
                }
                View::Text { cell, region }
            }
            ["json"] => View::Json,
            ["vector"] => View::Vector,
            ["pixels", w, h, rest @ ..] => {
                let side = |s: &str| s.parse::<u32>().ok().filter(|&v| v > 0 && v <= MAX_PIXELS);
                let (Some(width), Some(height)) = (side(w), side(h)) else {
                    return Err(format!("pixels are 1 to {MAX_PIXELS} a side, not {w} by {h}. {USAGE}"));
                };
                let overlay = match rest {
                    [] => false,
                    ["overlay"] => true,
                    _ => return Err(format!("can't read `{}`. {USAGE}", rest.join(" "))),
                };
                View::Pixels { width, height, overlay }
            }
            [] => return Err(format!("no view named. {USAGE}")),
            _ => return Err(format!("can't read `{}`. {USAGE}", words.join(" "))),
        };
        Ok(Request { view, seat })
    }
}

engine_api::field_struct! {
    /// The answer to a request. Exactly one view is set, unless `error`
    /// isn't empty, when none is and nothing was observed (the events stay
    /// for the next request). A field struct, not a `Result`, because it
    /// crosses a service call, which takes owned field types.
    #[derive(Debug, Default, PartialEq)]
    pub struct Observation {
        /// The frame it shows (the `Clock`'s).
        pub frame: u64,
        pub error: String,
        pub text: Option<TextView>,
        pub json: Option<Structured>,
        pub vector: Option<VectorView>,
        /// Events reported since this asker's last observation, oldest
        /// first.
        pub events: Vec<Happened>,
    }
}

impl Observation {
    pub fn failed(frame: u64, error: &str) -> Observation {
        Observation { frame, error: error.into(), ..Observation::default() }
    }

    /// As the control socket replies it: a text view as text, the others
    /// as JSON.
    pub fn render(&self) -> String {
        if !self.error.is_empty() {
            return format!("error: {}", self.error);
        }
        if let Some(t) = &self.text {
            let mut out = format!("frame {}\n{}", self.frame, t.render());
            out += &Happened::render_all(&self.events);
            return out;
        }
        let mut o = json::Object::new();
        o.number("frame", self.frame as f64);
        if let Some(s) = &self.json {
            s.write(&mut o);
        }
        if let Some(v) = &self.vector {
            v.write(&mut o);
        }
        o.raw("events", &json::array(self.events.iter().map(Happened::to_json)));
        o.finish()
    }
}

engine_api::field_struct! {
    /// An event, as an observation reports it: the game's `GameEvent`, the
    /// frame it was reported in, and its seat by name.
    #[derive(Debug, Default, PartialEq)]
    pub struct Happened {
        pub frame: u64,
        pub kind: String,
        /// Empty when it has none.
        pub seat: String,
        pub subject: Entity,
        pub other: Entity,
        pub value: f32,
        pub data: Vec<(String, f32)>,
        pub note: String,
    }
}

impl Happened {
    pub fn of(frame: u64, event: &GameEvent, playable: &Playable) -> Happened {
        let seat = event.seat.and_then(|s| playable.players.get(s as usize)).cloned().unwrap_or_default();
        Happened {
            frame,
            kind: event.kind.clone(),
            seat,
            subject: event.subject,
            other: event.other,
            value: event.value,
            data: event.data.clone(),
            note: event.note.clone(),
        }
    }

    fn to_json(&self) -> String {
        let mut o = json::Object::new();
        o.number("frame", self.frame as f64);
        o.string("kind", &self.kind);
        if !self.seat.is_empty() {
            o.string("seat", &self.seat);
        }
        for (key, e) in [("subject", self.subject), ("other", self.other)] {
            if e != Entity::DEAD {
                o.string(key, &id(e));
            }
        }
        o.float("value", self.value);
        // Apart, so a game's own names never collide with these.
        if !self.data.is_empty() {
            let mut data = json::Object::new();
            for (k, v) in &self.data {
                data.float(k, *v);
            }
            o.raw("data", &data.finish());
        }
        if !self.note.is_empty() {
            o.string("note", &self.note);
        }
        o.finish()
    }

    /// One line an event, under a heading; nothing if there are none.
    fn render_all(events: &[Happened]) -> String {
        if events.is_empty() {
            return "events since last observed: none\n".into();
        }
        let mut out = "events since last observed:\n".to_string();
        for e in events {
            out += &format!("  frame {} {}", e.frame, e.kind);
            if !e.seat.is_empty() {
                out += &format!(" seat {}", e.seat);
            }
            for (key, x) in [("subject", e.subject), ("other", e.other)] {
                if x != Entity::DEAD {
                    out += &format!(" {key} {}", id(x));
                }
            }
            if e.value != 0.0 {
                out += &format!(" value {}", e.value);
            }
            for (k, v) in &e.data {
                out += &format!(" {k} {v}");
            }
            if !e.note.is_empty() {
                out += &format!(" ({})", e.note);
            }
            out += "\n";
        }
        out
    }
}

/// An entity as views name it: its index, and its generation after a dot
/// once its index has been reused, so the common case reads as a small
/// number and a stale id never reads as a live one.
pub fn id(e: Entity) -> String {
    if e.generation == 0 { e.index.to_string() } else { format!("{}.{}", e.index, e.generation) }
}

engine_api::service! {
    /// Answers a request (`Request::parse`'s grammar) for the frame last
    /// run. Provided by the observer presenter; called by the agent
    /// interface, and by a bootstrap that steps and observes in one call.
    pub trait Observe {
        fn observe(request: &str) -> Observation;
    }
}
