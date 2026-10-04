//! Actions (presentation.md, D4): what a player can do, declared by the
//! game, written by sources, read by the game.
//!
//! - **Declared:** an `Action` is a button (0 or 1) or an axis (in
//!   [-1, 1]), *held* until changed (pong's paddle) or lasting the frame
//!   it's sent for (a jump). It says what it means and which keys and
//!   gamepad controls drive it by default, so a person, and a model
//!   trained on devices, plays any game the same way.
//! - **Chosen from:** the declaration gives a discrete set of choices
//!   (`Playable::choices`), as LLMs and most RL libraries want, and parses
//!   a command naming them (`Playable::parse`). Anything else is an error
//!   that lists the choices, and changes nothing.
//! - **Written** between frames with `act`, which checks each setting,
//!   keeps a held one in the seat's `ActionState`, and sends every one as
//!   an `Act` event saying where it came from: the recording's input log.
//!   A writer inside a frame (a game's own AI) takes `&mut ActionState`
//!   and an `EventWriter<Act>`, and does the same with `Playable::check`.
//! - **Read** by the game: held values from `ActionState`, by action index,
//!   and per-frame ones from `Act` events, which each reader sees once,
//!   the frame they're for. So `step 6` repeats a held action six frames
//!   (the Arcade Learning Environment's frame skip) and a jump once.

use engine_api::{WorldMut, component, event};
use platform::{GAMEPAD, KEYBOARD, KEYS, PAD_AXES, PAD_BUTTONS};

use crate::{Names, Playable, Seat, Validator};

engine_api::field_struct! {
    /// One thing a player can do.
    #[derive(Debug, Default, PartialEq)]
    pub struct Action {
        pub name: String,
        /// What it does, in the game's words, for an agent reading the
        /// list of actions: "moves your paddle: up -1, down 1".
        pub meaning: String,
        /// An axis, in [-1, 1]; otherwise a button, 0 or 1.
        pub axis: bool,
        /// Holds its value until set again; otherwise it lasts the frame it
        /// was sent for.
        pub held: bool,
        /// An axis's choices by name, for -1, 0 and 1 ("up", "stay",
        /// "down"): all three, or none, when its choices are `name=-1`,
        /// `name=0` and `name=1`.
        pub negative: String,
        pub neutral: String,
        pub positive: String,
        /// Default keys (`platform::KEYS`): while one is down the action is
        /// its `value`.
        pub keys: Vec<Bind>,
        /// Default gamepad controls: a button as a key; an axis
        /// (`platform::PAD_AXES`) sets the action to its position times
        /// `value` (-1 to turn a stick's up-is-positive into the world's
        /// y).
        pub pad: Vec<Bind>,
    }
}

engine_api::field_struct! {
    /// A control and the value it gives an action. A key drives seat
    /// `seat`; gamepad n always drives seat n, so a pad binding's `seat`
    /// is 0.
    #[derive(Debug, Default, PartialEq)]
    pub struct Bind {
        pub control: String,
        pub value: f32,
        pub seat: u32,
    }
}

impl Action {
    pub fn button(name: &str, meaning: &str) -> Action {
        Action { name: name.into(), meaning: meaning.into(), ..Action::default() }
    }

    pub fn axis(name: &str, meaning: &str) -> Action {
        Action { axis: true, ..Action::button(name, meaning) }
    }

    pub fn held(self) -> Action {
        Action { held: true, ..self }
    }

    /// Names for an axis's -1, 0 and 1.
    pub fn directions(self, negative: &str, neutral: &str, positive: &str) -> Action {
        Action { negative: negative.into(), neutral: neutral.into(), positive: positive.into(), ..self }
    }

    /// A key for the first seat.
    pub fn key(self, control: &str, value: f32) -> Action {
        self.key_for(0, control, value)
    }

    pub fn key_for(mut self, seat: u32, control: &str, value: f32) -> Action {
        self.keys.push(Bind { control: control.into(), value, seat });
        self
    }

    pub fn pad(mut self, control: &str, value: f32) -> Action {
        self.pad.push(Bind { control: control.into(), value, seat: 0 });
        self
    }

    fn named(&self) -> bool {
        !self.negative.is_empty()
    }
}

component! {
    /// A seat's held actions, by action index (`Playable::actions`); a
    /// per-frame action's slot stays 0. On the seat's entity. Read with
    /// `get`, which answers 0 for an index the declaration doesn't have,
    /// so a game reading an action a reload has just added sees it
    /// released rather than panicking.
    #[derive(Debug, Default, PartialEq)]
    pub struct ActionState: "play::ActionState" {
        pub held: Vec<f32>,
    }
}

impl ActionState {
    pub fn get(&self, action: u32) -> f32 {
        self.held.get(action as usize).copied().unwrap_or(0.0)
    }
}

/// `Act::source`: who set an action. What the recording keeps, so a
/// replay says whether a person, an agent or the game itself moved.
pub mod source {
    pub const KEYBOARD: u32 = 1;
    pub const GAMEPAD: u32 = 2;
    /// An agent, over the control socket or MCP.
    pub const AGENT: u32 = 3;
    /// A recorded session played back.
    pub const REPLAY: u32 = 4;
    /// The game's own code, such as an AI opponent writing a seat's actions.
    pub const GAME: u32 = 5;
    pub const NAMES: [&str; 6] = ["", "keyboard", "gamepad", "agent", "replay", "game"];
}

event! {
    /// One action set for one seat, by one source: sent for every setting
    /// `act` makes, held or not, for the frame after it's sent (or the same
    /// frame, from a system before the reader). The game reads per-frame
    /// actions from these; the recorder logs them all.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Act: "play::Act" {
        pub seat: u32,
        pub action: u32,
        pub value: f32,
        pub source: u32,
    }
}

/// An action and the value a command gives it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Setting {
    pub action: u32,
    pub value: f32,
}

/// One of the discrete set: its name, which `parse` accepts, and what it
/// sets (`None` for `noop`, which sets nothing).
#[derive(Clone, Debug, PartialEq)]
pub struct Choice {
    pub name: String,
    pub setting: Option<Setting>,
}

/// What a control drives: an action of a seat, and the value it gives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bound {
    pub seat: u32,
    pub action: u32,
    pub value: f32,
}

/// Why an action wasn't taken. Its message names what would have been
/// accepted, so an agent can correct itself from the error alone.
#[derive(Clone, Debug, PartialEq)]
pub enum ActionError {
    /// The world has no `Playable`.
    NotDeclared,
    UnknownSeat {
        seat: String,
        seats: Vec<String>,
    },
    Unknown {
        token: String,
        choices: Vec<String>,
    },
    /// `name=value` whose value isn't a number.
    NotANumber {
        action: String,
        value: String,
    },
    /// An axis set outside [-1, 1], or to NaN.
    OutOfRange {
        action: String,
        value: f32,
    },
    /// A button set to something but 0 or 1.
    NotAButton {
        action: String,
        value: f32,
    },
    /// One command setting one action twice.
    Twice {
        action: String,
    },
    Empty,
    BadSource {
        source: u32,
    },
}

impl std::fmt::Display for ActionError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            ActionError::NotDeclared => write!(f, "this game declares no actions"),
            ActionError::UnknownSeat { seat, seats } => write!(f, "no seat `{seat}`: the seats are {}", seats.join(", ")),
            ActionError::Unknown { token, choices } => write!(f, "unknown action `{token}`: choose from {}", choices.join(", ")),
            ActionError::NotANumber { action, value } => write!(f, "`{action}={value}`: the value isn't a number"),
            ActionError::OutOfRange { action, value } => write!(f, "`{action}` is an axis, from -1 to 1, not {value}"),
            ActionError::NotAButton { action, value } => write!(f, "`{action}` is a button, 0 or 1, not {value}"),
            ActionError::Twice { action } => write!(f, "`{action}` is set twice"),
            ActionError::Empty => write!(f, "no action given: send `noop` to change nothing"),
            ActionError::BadSource { source } => write!(f, "unknown source {source}"),
        }
    }
}

impl std::error::Error for ActionError {}

impl Playable {
    /// An action's index, by name.
    pub fn action_index(&self, name: &str) -> Option<u32> {
        self.actions.iter().position(|a| a.name == name).map(|i| i as u32)
    }

    /// `value` for `action`, if the action takes it.
    pub fn check(&self, action: u32, value: f32) -> Result<f32, ActionError> {
        let Some(a) = self.actions.get(action as usize) else {
            return Err(ActionError::Unknown { token: format!("#{action}"), choices: self.choice_names() });
        };
        if a.axis {
            if !(-1.0..=1.0).contains(&value) {
                return Err(ActionError::OutOfRange { action: a.name.clone(), value });
            }
        } else if value != 0.0 && value != 1.0 {
            return Err(ActionError::NotAButton { action: a.name.clone(), value });
        }
        Ok(value)
    }

    /// The discrete set: `noop` first, then each action's values by name,
    /// in declaration order. Each choice sets one action: an LLM picks one
    /// command at a time, and a set of every combination would grow as the
    /// product of the actions'. RL that wants every combination takes each
    /// action's `options` as one dimension (Gymnasium's `MultiDiscrete`).
    pub fn choices(&self) -> Vec<Choice> {
        let mut out = vec![Choice { name: "noop".into(), setting: None }];
        for (i, a) in self.actions.iter().enumerate() {
            let at = |value| Some(Setting { action: i as u32, value });
            if a.axis && a.named() {
                for (name, value) in [(&a.negative, -1.0), (&a.neutral, 0.0), (&a.positive, 1.0)] {
                    out.push(Choice { name: name.clone(), setting: at(value) });
                }
            } else if a.axis {
                for value in [-1.0, 0.0, 1.0] {
                    out.push(Choice { name: format!("{}={value}", a.name), setting: at(value) });
                }
            } else {
                out.push(Choice { name: a.name.clone(), setting: at(1.0) });
                // A per-frame button is released by not being sent.
                if a.held {
                    out.push(Choice { name: format!("{}=0", a.name), setting: at(0.0) });
                }
            }
        }
        out
    }

    fn choice_names(&self) -> Vec<String> {
        self.choices().into_iter().map(|c| c.name).collect()
    }

    /// The values a discrete policy picks among for one action: a button's
    /// 0 and 1, an axis's -1, 0 and 1.
    pub fn options(&self, action: u32) -> Vec<f32> {
        match self.actions.get(action as usize) {
            Some(a) if a.axis => vec![-1.0, 0.0, 1.0],
            Some(_) => vec![0.0, 1.0],
            None => Vec::new(),
        }
    }

    /// A command: choices (`up`), or `action=value` (`paddle=-0.5`), several
    /// at once separated by spaces or commas. All of it is taken, or none
    /// (an error).
    pub fn parse(&self, command: &str) -> Result<Vec<Setting>, ActionError> {
        let tokens: Vec<&str> = command.split(|c: char| c.is_whitespace() || c == ',').filter(|t| !t.is_empty()).collect();
        if tokens.is_empty() {
            return Err(ActionError::Empty);
        }
        let choices = self.choices();
        let mut out: Vec<Setting> = Vec::new();
        for token in tokens {
            let setting = if let Some((name, value)) = token.split_once('=') {
                let Some(action) = self.action_index(name) else {
                    return Err(ActionError::Unknown { token: token.into(), choices: self.choice_names() });
                };
                let value: f32 = value.parse().map_err(|_| ActionError::NotANumber { action: name.into(), value: value.into() })?;
                Some(Setting { action, value: self.check(action, value)? })
            } else {
                match choices.iter().find(|c| c.name == token) {
                    Some(c) => c.setting,
                    None => return Err(ActionError::Unknown { token: token.into(), choices: self.choice_names() }),
                }
            };
            if let Some(s) = setting {
                if out.iter().any(|o| o.action == s.action) {
                    return Err(ActionError::Twice { action: self.actions[s.action as usize].name.clone() });
                }
                out.push(s);
            }
        }
        Ok(out)
    }

    /// What `control` on `device` drives by default (gamepad `pad`'s, for a
    /// gamepad), if anything. A source keeps which controls are down and
    /// decides what several at once mean (presentation.md's open question).
    pub fn bound(&self, device: u32, pad: u32, control: &str) -> Option<Bound> {
        self.actions.iter().enumerate().find_map(|(i, a)| {
            let (binds, seat) = match device {
                KEYBOARD => (&a.keys, None),
                GAMEPAD => (&a.pad, Some(pad)),
                _ => return None,
            };
            let b = binds.iter().find(|b| b.control == control)?;
            let seat = seat.unwrap_or(b.seat);
            ((seat as usize) < self.players.len()).then_some(Bound { seat, action: i as u32, value: b.value })
        })
    }
}

/// Sets actions for `seat` from `source`, between frames: checks every
/// setting first, so a command is taken whole or not at all, then keeps
/// the held ones in the seat's `ActionState` and sends each as an `Act`,
/// for the next frame.
pub fn act(world: &mut WorldMut, seat: u32, settings: &[Setting], source: u32) -> Result<(), ActionError> {
    if source == 0 || source as usize >= source::NAMES.len() {
        return Err(ActionError::BadSource { source });
    }
    let playable = crate::declared(world).ok_or(ActionError::NotDeclared)?;
    if seat as usize >= playable.players.len() {
        return Err(ActionError::UnknownSeat { seat: format!("#{seat}"), seats: playable.players.clone() });
    }
    for s in settings {
        playable.check(s.action, s.value)?;
    }
    world.for_each::<(&Seat, &mut ActionState)>(|_, (s, mut state)| {
        if s.index != seat {
            return;
        }
        state.held.resize(playable.actions.len(), 0.0);
        for st in settings.iter().filter(|st| playable.actions[st.action as usize].held) {
            state.held[st.action as usize] = st.value;
        }
    });
    for s in settings {
        world.send_event(Act { seat, action: s.action, value: s.value, source });
    }
    Ok(())
}

/// `act` from a command naming the seat and the choices, as an agent sends
/// it: `left up`. Returns what it set.
pub fn act_command(world: &mut WorldMut, seat: &str, command: &str, source: u32) -> Result<Vec<Setting>, ActionError> {
    let playable = crate::declared(world).ok_or(ActionError::NotDeclared)?;
    let seat = playable.seat(seat)?;
    let settings = playable.parse(command)?;
    act(world, seat, &settings, source)?;
    Ok(settings)
}

pub(crate) fn validate(p: &Playable, v: &mut Validator) {
    let mut choices = Names::default();
    choices.add(v, "the choice `noop`", "noop");
    let mut controls = Names::default();
    for (i, a) in p.actions.iter().enumerate() {
        let at = format!("actions[{i}] `{}`", a.name);
        v.name(&at, &a.name);
        v.meaning(&at, &a.meaning);
        choices.add(v, &at, &a.name);
        let directions = [&a.negative, &a.neutral, &a.positive];
        let named = directions.iter().filter(|d| !d.is_empty()).count();
        if !a.axis && named > 0 {
            v.say(&at, "is a button, and only an axis names its directions");
        } else if named != 0 && named != 3 {
            v.say(&at, "names some of its directions: name all three (-1, 0, 1) or none");
        } else if named == 3 {
            for d in directions {
                v.name(&format!("{at}, direction `{d}`"), d);
                choices.add(v, &format!("a direction of {at}"), d);
            }
        }
        for (kind, binds, device) in [("key", &a.keys, KEYBOARD), ("pad", &a.pad, GAMEPAD)] {
            for (j, b) in binds.iter().enumerate() {
                let bat = format!("{at}, {kind} `{}`", b.control);
                let known = if device == KEYBOARD { KEYS.contains(&b.control.as_str()) } else { PAD_BUTTONS.contains(&b.control.as_str()) };
                let stick = device == GAMEPAD && PAD_AXES.contains(&b.control.as_str());
                if !known && !stick {
                    v.say(&bat, "isn't a control (platform::KEYS, PAD_BUTTONS, PAD_AXES)");
                }
                if stick && !a.axis {
                    v.say(&bat, "is a stick, which only an axis can take");
                }
                if a.axis {
                    if !(-1.0..=1.0).contains(&b.value) || b.value == 0.0 {
                        v.say(&bat, &format!("gives {}: an axis's binding gives -1 to 1, not 0", b.value));
                    }
                } else if b.value != 1.0 {
                    v.say(&bat, &format!("gives {}: a button's binding gives 1", b.value));
                }
                if device == GAMEPAD && b.seat != 0 {
                    v.say(&bat, "names a seat: gamepad n drives seat n");
                }
                if b.seat as usize >= p.players.len().max(1) {
                    v.say(&bat, &format!("drives seat {}, of {}", b.seat, p.players.len()));
                }
                controls.add(v, &format!("{at}'s {kind} {j}"), &format!("{kind} `{}` of seat {}", b.control, b.seat));
            }
        }
    }
}
