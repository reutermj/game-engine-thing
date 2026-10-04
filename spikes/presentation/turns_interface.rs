//! SPIKE (get-3hd.1): `spike_turns`' interface, the turn barrier as data.
//! The windowed lockstep bootstrap (`lockstep_window.rs`) keeps the barrier
//! and publishes a `Turn` whenever it changes; a game mod reads it to steer
//! (`pong_versus.rs` sets both paddles from it) and to describe it. A game
//! that ends spawns an `Outcome`, which the bootstrap reads after every
//! frame to stop.
//!
//! Its own resident mod, like `clock`, because the bootstrap is resident and
//! may only depend on resident mods, while the game reading it reloads.

use engine_api::{WorldMut, component};

/// The players, in the order `Turn`'s per-side fields use. Fixed for the
/// spike: both are always registered, so a turn waits for both.
pub const SIDES: [&str; 2] = ["left", "right"];

/// An action's intent: -1 up, 1 down, 0 stay. What a paddle's
/// `pong::Paddle::intent` takes.
pub fn intent(action: &str) -> Option<f32> {
    match action {
        "up" => Some(-1.0),
        "down" => Some(1.0),
        "stay" => Some(0.0),
        _ => None,
    }
}

pub fn action(intent: f32) -> &'static str {
    if intent < 0.0 {
        "up"
    } else if intent > 0.0 {
        "down"
    } else {
        "stay"
    }
}

engine_api::field_struct! {
    /// The barrier's state. A field struct rather than the component itself
    /// so the bootstrap's mod state can hold it too.
    #[derive(Debug, Default, Copy)]
    pub struct TurnInfo {
        /// The turn open for submissions, or playing; from 1.
        pub turn: u64,
        /// Frames a turn runs.
        pub frames: u32,
        /// Frames of `turn` already run while it plays; 0 while it's open.
        pub played: u32,
        /// Whether `turn` is playing (both submitted) rather than open.
        pub playing: bool,
        /// Whether each side has submitted for `turn` (`SIDES` order). What
        /// they submitted stays the bootstrap's until the turn plays, so
        /// neither side sees the other's move early.
        pub submitted: [bool; 2],
        /// The intents in force: the last played turn's actions (0 before
        /// the first).
        pub intents: [f32; 2],
    }
}

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Turn: "spike_turns::Turn" {
        pub now: TurnInfo,
    }
}

component! {
    /// A game's end, spawned by the game: `winner` is an index into `SIDES`.
    #[derive(Debug, Default, Copy)]
    pub struct Outcome: "spike_turns::Outcome" {
        pub winner: u32,
    }
}

pub fn turn(world: &mut WorldMut) -> Option<TurnInfo> {
    world.single::<&Turn, _>(|_, t| t.now)
}

pub fn outcome(world: &mut WorldMut) -> Option<Outcome> {
    world.single::<&Outcome, _>(|_, o| *o)
}

/// As `clock::publish`.
pub fn publish(world: &mut WorldMut, slot: &mut Option<engine_api::Entity>, turn: Turn) {
    match slot.filter(|&e| world.is_alive(e)) {
        Some(e) => world.insert(e, turn),
        None => *slot = Some(world.spawn((turn,))),
    }
}
