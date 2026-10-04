//! What happened (presentation.md, D3): events a game reports, which every
//! view carries since the last observation, so an agent stepping thirty
//! frames at a time still hears of a point it didn't see scored. In the
//! spike's matches both players inferred points from score changes and
//! asked for hit events.
//!
//! One event type for every game, `GameEvent`, whose `kind` is a name the
//! game declares and whose payload is the game's: the entities involved,
//! a number, named numbers and a note. A reader that knows nothing of the
//! game can still list, count and filter them; one that knows the game
//! reads its payload. Goals are reported the same way (`goals.rs`).

use engine_api::{Entity, event};

use crate::{Names, Playable, Validator};

/// Kinds most games have, named here so games share a vocabulary. A game
/// still declares each it reports, with what it means there.
pub mod kind {
    /// A point scored; `seat` is the scorer.
    pub const POINT: &str = "point";
    /// Two things struck each other: `subject` and `other`.
    pub const HIT: &str = "hit";
    /// `subject` died (or a seat lost a life: `seat`).
    pub const DEATH: &str = "death";
    pub const SPAWN: &str = "spawn";
    pub const DESPAWN: &str = "despawn";
    /// `seat` (or `subject`) picked `other` up.
    pub const PICKUP: &str = "pickup";
    /// A level began or ended; `value` its number.
    pub const LEVEL: &str = "level";
}

engine_api::field_struct! {
    /// An event kind a game reports, and what it means there.
    #[derive(Debug, Default, PartialEq)]
    pub struct EventDecl {
        pub name: String,
        pub meaning: String,
    }
}

event! {
    /// Something that happened, reported by the game with an
    /// `EventWriter<GameEvent>` (or `send_event` between frames).
    #[derive(Debug, Default, PartialEq)]
    pub struct GameEvent: "play::GameEvent" {
        /// A declared event's or goal's name.
        pub kind: String,
        /// The seat it happened to or for, if any.
        pub seat: Option<u32>,
        /// What it happened to, and what else took part (`Entity::DEAD`
        /// for none): ids an agent can find in a view.
        pub subject: Entity,
        pub other: Entity,
        /// The kind's one number, if it has one (a hit's speed).
        pub value: f32,
        /// Any further numbers, named: the game's own payload.
        pub data: Vec<(String, f32)>,
        pub note: String,
    }
}

impl GameEvent {
    pub fn new(kind: &str) -> GameEvent {
        GameEvent { kind: kind.into(), ..GameEvent::default() }
    }

    pub fn seat(self, seat: u32) -> GameEvent {
        GameEvent { seat: Some(seat), ..self }
    }

    pub fn subject(self, subject: Entity) -> GameEvent {
        GameEvent { subject, ..self }
    }

    pub fn other(self, other: Entity) -> GameEvent {
        GameEvent { other, ..self }
    }

    pub fn value(self, value: f32) -> GameEvent {
        GameEvent { value, ..self }
    }

    pub fn datum(mut self, name: &str, value: f32) -> GameEvent {
        self.data.push((name.into(), value));
        self
    }

    pub fn note(self, note: &str) -> GameEvent {
        GameEvent { note: note.into(), ..self }
    }
}

impl Playable {
    /// Whether the declaration covers `event`: a declared kind (an event or
    /// a goal) and a seat it has. What a reader checks before trusting an
    /// event, and reports when it can't, since a game reporting what it
    /// never declared is a bug in the game.
    pub fn check_event(&self, event: &GameEvent) -> Result<(), String> {
        let declared = self.events.iter().any(|e| e.name == event.kind) || self.goals.iter().any(|g| g.name == event.kind);
        if !declared {
            return Err(format!("event `{}` isn't declared", event.kind));
        }
        if let Some(seat) = event.seat
            && seat as usize >= self.players.len()
        {
            return Err(format!("event `{}` is for seat {seat}, of {}", event.kind, self.players.len()));
        }
        Ok(())
    }
}

/// Events and goals share a namespace, since a goal is reported as an
/// event of its name.
pub(crate) fn validate(p: &Playable, v: &mut Validator) {
    let mut names = Names::default();
    for (i, e) in p.events.iter().enumerate() {
        let at = format!("events[{i}] `{}`", e.name);
        v.name(&at, &e.name);
        v.meaning(&at, &e.meaning);
        names.add(v, &at, &e.name);
    }
    for (i, g) in p.goals.iter().enumerate() {
        names.add(v, &format!("goals[{i}] `{}`", g.name), &g.name);
    }
}
