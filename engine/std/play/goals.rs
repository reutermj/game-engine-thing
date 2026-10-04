//! Goals and metrics (presentation.md, D7): what winning and losing are,
//! and how far along a player is, declared beside the actions so an agent
//! gets both from one place.
//!
//! **The game decides; the engine derives.** A game reports a goal reached
//! as a `GameEvent` of the goal's name, for the seat it was reached by or
//! against, and keeps each seat's metrics current in its `Metrics`. From
//! those the agent interface derives a reward (each goal's) and the end of
//! a session (a goal that `ends` it), Gymnasium's `terminated`; and a cut
//! when no metric changes for `Playable::stall_frames` frames, its
//! `truncated`, a cheap softlock detector. So nothing about any game's
//! reward is written into the engine, and the conditions are the game's
//! own code rather than a language the engine would have to grow.
//! Fuzzy goals ("explores the level") are judged afterwards from the
//! recording against a rubric, as SIMA 2 was evaluated: not declared here.

use engine_api::component;

use crate::{Names, Playable, Validator};

engine_api::field_struct! {
    /// A success or a failure, reported by the game when it happens.
    #[derive(Debug, Default, PartialEq)]
    pub struct Goal {
        pub name: String,
        /// When it's reached, in the game's words: "you scored a point".
        pub meaning: String,
        /// A success, or a failure.
        pub success: bool,
        /// Reaching it ends the session (a match won), or not (a point).
        pub ends: bool,
        /// What reaching it is worth to the seat it's reported for: at
        /// least 0 for a success, at most 0 for a failure.
        pub reward: f32,
    }
}

impl Goal {
    /// Worth 1 by default.
    pub fn success(name: &str, meaning: &str) -> Goal {
        Goal { name: name.into(), meaning: meaning.into(), success: true, ends: false, reward: 1.0 }
    }

    /// Worth -1 by default.
    pub fn failure(name: &str, meaning: &str) -> Goal {
        Goal { success: false, reward: -1.0, ..Goal::success(name, meaning) }
    }

    pub fn ends(self) -> Goal {
        Goal { ends: true, ..self }
    }

    pub fn reward(self, reward: f32) -> Goal {
        Goal { reward, ..self }
    }
}

engine_api::field_struct! {
    /// A progress measure, per seat: a score, a distance to go, lives.
    /// `min` and `max` bound it, for the vector view's normalising and a
    /// judge's sense of scale.
    #[derive(Debug, Default, PartialEq)]
    pub struct Metric {
        pub name: String,
        pub meaning: String,
        pub min: f32,
        pub max: f32,
        /// Whether more is better (a score) or less (a distance to go).
        pub higher_is_better: bool,
    }
}

impl Metric {
    pub fn new(name: &str, meaning: &str, min: f32, max: f32) -> Metric {
        Metric { name: name.into(), meaning: meaning.into(), min, max, higher_is_better: true }
    }

    pub fn lower_is_better(self) -> Metric {
        Metric { higher_is_better: false, ..self }
    }
}

component! {
    /// A seat's metrics, by index (`Playable::metrics`), on the seat's
    /// entity; the game keeps them current.
    #[derive(Debug, Default, PartialEq)]
    pub struct Metrics: "play::Metrics" {
        pub values: Vec<f32>,
    }
}

pub(crate) fn validate(p: &Playable, v: &mut Validator) {
    for (i, g) in p.goals.iter().enumerate() {
        let at = format!("goals[{i}] `{}`", g.name);
        v.name(&at, &g.name);
        v.meaning(&at, &g.meaning);
        if !g.reward.is_finite() {
            v.say(&at, &format!("is worth {}", g.reward));
        } else if g.success && g.reward < 0.0 {
            v.say(&at, &format!("is a success worth {}: a success is worth 0 or more", g.reward));
        } else if !g.success && g.reward > 0.0 {
            v.say(&at, &format!("is a failure worth {}: a failure is worth 0 or less", g.reward));
        }
    }
    let mut names = Names::default();
    for (i, m) in p.metrics.iter().enumerate() {
        let at = format!("metrics[{i}] `{}`", m.name);
        v.name(&at, &m.name);
        v.meaning(&at, &m.meaning);
        names.add(v, &at, &m.name);
        if !(m.min.is_finite() && m.max.is_finite() && m.min < m.max) {
            v.say(&at, &format!("runs from {} to {}: it needs a finite range, low to high", m.min, m.max));
        }
    }
}
