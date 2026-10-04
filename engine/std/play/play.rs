//! What makes a game playable by anyone, person or agent, with no glue
//! (presentation.md, D4 and D7): the game declares, in one `Playable`,
//!
//! - its **players** (seats: "left", "right");
//! - its **actions** (`actions.rs`): what a player can do, each a button or
//!   an axis in [-1, 1], held until changed or lasting a frame, with
//!   default keys and gamepad controls, and from them a discrete set of
//!   choices for LLMs and RL;
//! - the **events** it reports (`events.rs`), generic with game-specific
//!   payloads, so every view can say what happened since the last one;
//! - its **goals and metrics** (`goals.rs`): success and failure, which
//!   end a session or don't, progress measures, an objective in prose. A
//!   reward and an end are derived from them, never written into the
//!   engine.
//!
//! The game reads its actions (`ActionState`, `Act`), never a device, and
//! sources write them (`act`) with who they were: the keyboard, a
//! gamepad, an agent, a replay. Everything here is data in the world, so a
//! resident reader (a bootstrap, a recorder, the agent interface) uses it
//! without naming the game.
//!
//! Resident, since bootstraps map devices to actions through it. Left for
//! later (presentation.md, "The interfaces, as built"): combining one
//! action from several sources at once, sticky actions, rebinding, the
//! mouse, and per-seat gamepad layouts.

use engine_api::{Entity, WorldMut, component};

mod actions;
mod events;
mod goals;

pub use actions::*;
pub use events::*;
pub use goals::*;

component! {
    /// A game's declaration: one a world, put there with `declare`, which
    /// checks it. Built with the methods below, read by anything that plays
    /// or watches. Every name in it is an identifier (`[a-z][a-z0-9_]*`),
    /// so it can be typed as a command and used as a key.
    #[derive(Debug, Default, PartialEq)]
    pub struct Playable: "play::Playable" {
        /// What a player is trying to do, in prose, for an agent that has
        /// never seen the game.
        pub objective: String,
        /// The seats, in order: a seat's index is its place here.
        pub players: Vec<String>,
        pub actions: Vec<Action>,
        pub events: Vec<EventDecl>,
        pub goals: Vec<Goal>,
        pub metrics: Vec<Metric>,
        /// What the fixed-length vector view has room for (its schema).
        pub observed: Vec<Slot>,
        /// Frames with no metric changing after which a run is cut short
        /// and flagged as a possible softlock; 0 never.
        pub stall_frames: u32,
    }
}

engine_api::field_struct! {
    /// Room in the vector view for `count` entities labelled `what`
    /// (`present::Label::what`): RL wants the same length every frame, so
    /// the game says how many of each it can have.
    #[derive(Debug, Default, PartialEq)]
    pub struct Slot {
        pub what: String,
        pub count: u32,
    }
}

impl Playable {
    pub fn new(objective: &str) -> Playable {
        Playable { objective: objective.into(), players: vec!["player".into()], ..Playable::default() }
    }

    /// The seats, replacing the default one, `player`.
    pub fn players(mut self, names: &[&str]) -> Playable {
        self.players = names.iter().map(|&n| n.into()).collect();
        self
    }

    pub fn action(mut self, action: Action) -> Playable {
        self.actions.push(action);
        self
    }

    pub fn event(mut self, name: &str, meaning: &str) -> Playable {
        self.events.push(EventDecl { name: name.into(), meaning: meaning.into() });
        self
    }

    pub fn goal(mut self, goal: Goal) -> Playable {
        self.goals.push(goal);
        self
    }

    pub fn metric(mut self, metric: Metric) -> Playable {
        self.metrics.push(metric);
        self
    }

    pub fn observe(mut self, what: &str, count: u32) -> Playable {
        self.observed.push(Slot { what: what.into(), count });
        self
    }

    pub fn stall_after(mut self, frames: u32) -> Playable {
        self.stall_frames = frames;
        self
    }

    /// A seat's index, by name.
    pub fn seat(&self, name: &str) -> Result<u32, ActionError> {
        self.players
            .iter()
            .position(|p| p == name)
            .map(|i| i as u32)
            .ok_or_else(|| ActionError::UnknownSeat { seat: name.into(), seats: self.players.clone() })
    }

    /// Every problem with the declaration, or none. `declare` refuses one
    /// with any, so a game finds out at load, by name, not when an agent
    /// first meets the mistake.
    pub fn validate(&self) -> Result<(), Vec<Invalid>> {
        let mut v = Validator::default();
        if self.objective.trim().is_empty() {
            v.say("objective", "is empty: say, in prose, what a player is trying to do");
        }
        if self.players.is_empty() {
            v.say("players", "is empty: a game has at least one seat");
        }
        let mut seats = Names::default();
        for (i, p) in self.players.iter().enumerate() {
            v.name(&format!("players[{i}]"), p);
            seats.add(&mut v, &format!("players[{i}]"), p);
        }
        actions::validate(self, &mut v);
        events::validate(self, &mut v);
        goals::validate(self, &mut v);
        let mut whats = Names::default();
        for (i, s) in self.observed.iter().enumerate() {
            let at = format!("observed[{i}]");
            if s.what.is_empty() {
                v.say(&at, "names no label class");
            }
            if s.count == 0 {
                v.say(&at, &format!("has room for no `{}`", s.what));
            }
            whats.add(&mut v, &at, &s.what);
        }
        if v.found.is_empty() { Ok(()) } else { Err(v.found) }
    }
}

/// One problem with a declaration: where, and what.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Invalid {
    pub at: String,
    pub why: String,
}

impl std::fmt::Display for Invalid {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}: {}", self.at, self.why)
    }
}

/// Collects the problems a validation finds.
#[derive(Default)]
struct Validator {
    found: Vec<Invalid>,
}

impl Validator {
    fn say(&mut self, at: &str, why: &str) {
        self.found.push(Invalid { at: at.into(), why: why.into() });
    }

    /// Checks `name` is an identifier: what a command, a JSON key and a
    /// schema's column can all hold unquoted.
    fn name(&mut self, at: &str, name: &str) {
        let mut chars = name.chars();
        let ok = chars.next().is_some_and(|c| c.is_ascii_lowercase())
            && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
            && name.len() <= 32;
        if !ok {
            self.say(at, &format!("`{name}` isn't a name: lowercase letters, digits and `_`, from a letter, at most 32"));
        }
    }

    fn meaning(&mut self, at: &str, meaning: &str) {
        if meaning.trim().is_empty() {
            self.say(at, "says nothing of what it means");
        }
    }
}

/// A namespace being filled: says where a name was first used when it's
/// used again.
#[derive(Default)]
struct Names {
    seen: Vec<(String, String)>,
}

impl Names {
    fn add(&mut self, v: &mut Validator, at: &str, name: &str) {
        match self.seen.iter().find(|(n, _)| n == name) {
            Some((_, first)) => v.say(at, &format!("`{name}` is already {first}")),
            None => self.seen.push((name.into(), at.into())),
        }
    }
}

component! {
    /// A player's seat: one entity a seat, made by `declare`, which also
    /// carries the seat's `ActionState` and `Metrics`.
    #[derive(Debug, Default, PartialEq)]
    pub struct Seat: "play::Seat" {
        pub index: u32,
        pub name: String,
    }
}

/// The world's declaration, if a game has made one.
pub fn declared(world: &mut WorldMut) -> Option<Playable> {
    world.single::<&Playable, _>(|_, p| p.clone())
}

/// Puts `playable` in the world, if it's valid, with a seat entity for
/// each player. Called again (a reload that changed it), it replaces the
/// declaration and keeps what it can: each seat's held actions and metrics
/// by name, so a new action doesn't reset the others mid-game. Seats no
/// longer declared are despawned. Between frames: a game calls it from its
/// `load`, or where it sets its world up.
pub fn declare(world: &mut WorldMut, playable: Playable) -> Result<Entity, Vec<Invalid>> {
    playable.validate()?;
    let old = world.single::<&Playable, _>(|e, p| (e, p.clone()));
    let mut seats = Vec::new();
    world.for_each::<(&Seat, &ActionState, &Metrics)>(|e, (s, a, m)| seats.push((e, s.clone(), a.clone(), m.clone())));
    let entity = match &old {
        Some((e, _)) => {
            world.insert(*e, playable.clone());
            *e
        }
        None => world.spawn((playable.clone(),)),
    };
    let was = old.map(|(_, p)| p).unwrap_or_default();
    for (i, name) in playable.players.iter().enumerate() {
        let kept = seats.iter().find(|(_, s, _, _)| &s.name == name);
        // A new action starts released, a new metric at its minimum.
        let held = playable
            .actions
            .iter()
            .map(|a| kept.and_then(|(_, _, st, _)| carried(&was.actions, &st.held, &a.name, |x| &x.name)).unwrap_or(0.0))
            .collect();
        let values = playable
            .metrics
            .iter()
            .map(|m| kept.and_then(|(_, _, _, ms)| carried(&was.metrics, &ms.values, &m.name, |x| &x.name)).unwrap_or(m.min))
            .collect();
        let seat = (Seat { index: i as u32, name: name.clone() }, ActionState { held }, Metrics { values });
        match kept {
            Some((e, ..)) => {
                world.insert(*e, seat.0);
                world.insert(*e, seat.1);
                world.insert(*e, seat.2);
            }
            None => {
                world.spawn(seat);
            }
        }
    }
    for (e, s, _, _) in &seats {
        if !playable.players.contains(&s.name) {
            world.despawn(*e);
        }
    }
    Ok(entity)
}

/// The value the old declaration's `values` held for `name`, if it had one.
fn carried<T>(old: &[T], values: &[f32], name: &str, name_of: impl Fn(&T) -> &String) -> Option<f32> {
    old.iter().position(|x| name_of(x) == name).and_then(|i| values.get(i).copied())
}
