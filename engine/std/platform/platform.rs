//! The platform (presentation.md, D6): the window, its event loop and the
//! input devices, behind a service with one provider, which a game picks in
//! `engine_game(platform = ...)`: winit for a window, headless for a run
//! with none, or no provider at all, when every call answers
//! `NotProvided` and a bootstrap carries on without a window.
//!
//! **Never a GPU device.** GPU state can't cross mods (D11): the spike
//! crashed the engine when a reloadable presenter used a resident mod's
//! device. The window reaches a presenter as plain numbers
//! (`WindowHandles`), from which it makes its own surface.
//!
//! **Pumped, never run.** The bootstrap calls `pump` once a frame, between
//! frames, beside `pump_loader` (75 to 85 µs in the spike): winit's loop
//! must run on the main thread, which the bootstrap's is, and a bootstrap
//! that owned a window would be one bootstrap per platform.
//!
//! **Resident**, this interface and every provider: winit sets a
//! process-wide X11 error handler that points into the library that made
//! the event loop, and bootstraps, which call it, may only depend on
//! resident mods.
//!
//! Devices are reported, not interpreted: a key's name and whether it went
//! down. What a key *means* is the game's declaration (`play`), so the
//! platform never names an action and a game never names a key.

use engine_api::component;

/// `DeviceInput::device`.
pub const KEYBOARD: u32 = 1;
pub const GAMEPAD: u32 = 2;

/// `WindowHandles::system`.
pub const NO_WINDOW: u32 = 0;
/// An Xlib window: `display` is a `Display *`, `window` its XID.
pub const XLIB: u32 = 1;

engine_api::field_struct! {
    /// One change on a device since the last pump. `control` is a name
    /// from `KEYS` (a keyboard's) or `PAD_BUTTONS` and `PAD_AXES` (a
    /// gamepad's). `value` is 1 for a key or button going down and 0 for it
    /// coming up (never repeats while held), an axis's position in [-1, 1],
    /// a trigger's in [0, 1]. `pad` says which gamepad, from 0.
    #[derive(Debug, Default, PartialEq)]
    pub struct DeviceInput {
        pub device: u32,
        pub pad: u32,
        pub control: String,
        pub value: f32,
    }
}

engine_api::field_struct! {
    /// What one pump found.
    #[derive(Debug, Default, PartialEq)]
    pub struct Pumped {
        /// The window was asked to close: the bootstrap should quit.
        pub close: bool,
        /// Its inner size in pixels now; 0 by 0 with no window.
        pub width: u32,
        pub height: u32,
        /// Something uncovered or resized it, so a bootstrap that is idling
        /// (lockstep) should ask the presenter to draw again (`Redraw`).
        pub exposed: bool,
        pub inputs: Vec<DeviceInput>,
        /// How long the pump took, for the bootstrap's timing log.
        pub micros: u32,
    }
}

engine_api::field_struct! {
    /// The window as its windowing system knows it, as numbers: what a
    /// presenter makes its surface from. `system` is `NO_WINDOW` (and the
    /// rest 0) when there's none.
    #[derive(Debug, Default, PartialEq)]
    pub struct WindowHandles {
        pub system: u32,
        pub display: u64,
        pub screen: i32,
        pub window: u64,
        pub width: u32,
        pub height: u32,
    }
}

component! {
    /// The window a game asks for, put in the world by the game; the
    /// provider reads it when it makes the window and at each pump, so a
    /// reload that changes it retitles or resizes the window. Fixed-size
    /// by default (min = max), which a tiling window manager floats, as the
    /// spike's did. With none, the provider picks (1280 by 720, untitled).
    #[derive(Debug, Default, PartialEq)]
    pub struct WindowSpec: "platform::WindowSpec" {
        pub title: String,
        pub width: u32,
        pub height: u32,
        pub resizable: bool,
    }
}

engine_api::service! {
    /// The platform, provided by one resident mod. Calls come from the
    /// bootstrap between frames (`pump`) and from presenters (`window`).
    pub trait Platform {
        /// Handles the window's and the devices' pending events without
        /// waiting, and says what they were.
        fn pump() -> Pumped;
        /// The window, for a presenter to draw into; `NO_WINDOW` when
        /// headless. Called each frame by a presenter that follows resizes.
        fn window() -> WindowHandles;
    }
}

engine_api::service! {
    /// Draws the last frame again, between frames: what an idling bootstrap
    /// calls when a pump says the window was exposed, so a spectator window
    /// doesn't go stale while no frames run (presentation.md, D5).
    /// Declared here, not by a presenter, because the bootstrap is resident
    /// and may only depend on resident mods; provided by whichever
    /// presenter draws to the window, resolved by name at each call
    /// (`NotProvided` with none). Returns whether anything was drawn.
    pub trait Redraw {
        fn redraw() -> bool;
    }
}

/// Keyboard controls: the W3C UI Events `code` values, which name a key by
/// where it is, not what it types (so WASD is WASD on any layout), and
/// which winit's `KeyCode` mirrors name for name. Not every key, but every
/// one a game should bind by default.
pub const KEYS: &[&str] = &[
    "KeyA",
    "KeyB",
    "KeyC",
    "KeyD",
    "KeyE",
    "KeyF",
    "KeyG",
    "KeyH",
    "KeyI",
    "KeyJ",
    "KeyK",
    "KeyL",
    "KeyM",
    "KeyN",
    "KeyO",
    "KeyP",
    "KeyQ",
    "KeyR",
    "KeyS",
    "KeyT",
    "KeyU",
    "KeyV",
    "KeyW",
    "KeyX",
    "KeyY",
    "KeyZ",
    "Digit0",
    "Digit1",
    "Digit2",
    "Digit3",
    "Digit4",
    "Digit5",
    "Digit6",
    "Digit7",
    "Digit8",
    "Digit9",
    "ArrowUp",
    "ArrowDown",
    "ArrowLeft",
    "ArrowRight",
    "Space",
    "Enter",
    "Escape",
    "Tab",
    "Backspace",
    "ShiftLeft",
    "ShiftRight",
    "ControlLeft",
    "ControlRight",
    "AltLeft",
    "AltRight",
    "Comma",
    "Period",
    "Slash",
    "Semicolon",
    "Quote",
    "BracketLeft",
    "BracketRight",
    "Minus",
    "Equal",
    "F1",
    "F2",
    "F3",
    "F4",
    "F5",
    "F6",
    "F7",
    "F8",
    "F9",
    "F10",
    "F11",
    "F12",
];

/// Gamepad buttons, by gilrs's names for the standard layout (W3C Gamepad's
/// "standard" mapping): the face buttons by compass point, so `South` is
/// A on an Xbox pad and Cross on a PlayStation one.
pub const PAD_BUTTONS: &[&str] = &[
    "South",
    "East",
    "North",
    "West",
    "LeftTrigger",
    "RightTrigger",
    "LeftTrigger2",
    "RightTrigger2",
    "Select",
    "Start",
    "Mode",
    "LeftThumb",
    "RightThumb",
    "DPadUp",
    "DPadDown",
    "DPadLeft",
    "DPadRight",
];

/// Gamepad axes, by gilrs's names. A stick's y is positive *up*, as gilrs
/// reports it, which is the opposite of the world's y.
pub const PAD_AXES: &[&str] = &["LeftStickX", "LeftStickY", "RightStickX", "RightStickY"];

/// Whether `control` names a control of `device`.
pub fn is_control(device: u32, control: &str) -> bool {
    match device {
        KEYBOARD => KEYS.contains(&control),
        GAMEPAD => PAD_BUTTONS.contains(&control) || PAD_AXES.contains(&control),
        _ => false,
    }
}

/// Whether `control` is a gamepad axis, which reports a position rather
/// than up or down.
pub fn is_axis(control: &str) -> bool {
    PAD_AXES.contains(&control)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controls_belong_to_their_device() {
        assert!(is_control(KEYBOARD, "KeyW"));
        assert!(is_control(KEYBOARD, "F12"));
        assert!(!is_control(KEYBOARD, "South"));
        assert!(!is_control(KEYBOARD, "w"), "a key is named by its code, not what it types");
        assert!(is_control(GAMEPAD, "South"));
        assert!(is_control(GAMEPAD, "LeftStickY"));
        assert!(!is_control(GAMEPAD, "KeyW"));
        assert!(!is_control(0, "KeyW"));
        assert!(is_axis("RightStickX") && !is_axis("DPadUp"));
    }

    #[test]
    fn no_control_is_listed_twice() {
        for list in [KEYS, PAD_BUTTONS, PAD_AXES] {
            let mut sorted = list.to_vec();
            sorted.sort();
            sorted.dedup();
            assert_eq!(sorted.len(), list.len());
        }
    }
}
