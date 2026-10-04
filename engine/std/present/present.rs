//! The presentation vocabulary (presentation.md, D1): what a game attaches
//! to its entities so that anything can show them, and the draw list the
//! extract makes from it each frame (`draw_list.rs`), which every presenter
//! reads. It says *what* is shown, never *how*: no GPU type, no pixel and
//! no font appears here, so a game compiles against it without knowing
//! which presenters (a window, pixels for a vision model, text for an LLM,
//! numbers for RL) will read it.
//!
//! An entity is drawn when it has a `Place`, a `Look` and a shape (`Rect`,
//! `Circle`, `Line` or `Text`; several draw several). A `Label` makes it
//! something an agent is told about: its class (`what`, "ball") and its
//! name ("left"). Its id is its `Entity`, stable for its life and the same
//! in a replay, since the world is deterministic. A `Camera` says which
//! rectangle of the world to show, so a presenter maps world units to its
//! own without knowing what a unit means.
//!
//! Units are the game's own world units, with y growing downward, as in
//! physics2d. Left out on purpose until a game needs them, each a later
//! addition beside these, not a change to them (presentation.md, "The
//! interfaces, as built"): sprites and every other asset (images, fonts,
//! shaders), 3D, several cameras, outlines and gradients, and a text
//! layout beyond one line.

use engine_api::component;

mod draw_list;

pub use draw_list::*;

engine_api::field_struct! {
    /// A colour, in sRGB as a designer writes it (`#3cd2f0`), straight
    /// alpha. sRGB, not linear, because that is what every colour picker
    /// and palette gives; a presenter that blends in linear space converts.
    /// `repr(C)` so a draw item holding one has a fixed layout.
    #[repr(C)]
    #[derive(Debug, Default, Copy, PartialEq, Eq, Hash)]
    pub struct Colour {
        pub r: u8,
        pub g: u8,
        pub b: u8,
        pub a: u8,
    }
}

impl Colour {
    pub const WHITE: Colour = Colour::rgb(0xffffff);
    pub const BLACK: Colour = Colour::rgb(0x000000);

    /// Opaque, from `0xRRGGBB`.
    pub const fn rgb(hex: u32) -> Colour {
        Colour { r: (hex >> 16) as u8, g: (hex >> 8) as u8, b: hex as u8, a: 0xff }
    }

    pub const fn with_alpha(self, a: u8) -> Colour {
        Colour { a, ..self }
    }
}

component! {
    /// Where an entity is drawn: the centre of its shapes, in world units,
    /// and which way they face, as the cosine and sine of the angle (as
    /// physics2d's `Rotation`, so a game copies one into the other). A
    /// game keeps it current from whatever moves the entity; presentation
    /// never writes it.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Place: "present::Place" {
        pub x: f32,
        pub y: f32,
        pub c: f32,
        pub s: f32,
    }
}

/// Unturned: an angle of 0, not the all-zero rotation, which would draw
/// every shape as a point.
impl Default for Place {
    fn default() -> Place {
        Place { x: 0.0, y: 0.0, c: 1.0, s: 0.0 }
    }
}

impl Place {
    pub fn at(x: f32, y: f32) -> Place {
        Place { x, y, ..Place::default() }
    }

    pub fn turned(self, angle: f32) -> Place {
        Place { c: angle.cos(), s: angle.sin(), ..self }
    }
}

component! {
    /// How an entity's shapes look. `layer` orders drawing (higher on
    /// top); within a layer, overlapping items of different materials
    /// have no order a presenter must keep, which is what lets a GPU
    /// presenter draw each material at once (presentation.md, D2). Use a
    /// layer where overlap matters.
    ///
    /// `material` is an identity, not a description: items that share one
    /// may be drawn together. 0 is a flat, opaque-or-blended colour; any
    /// other number is the game's own name for a look that a later
    /// material declaration (with assets) will describe, and until then
    /// draws as 0.
    #[derive(Debug, PartialEq, Copy)]
    pub struct Look: "present::Look" {
        pub colour: Colour,
        pub layer: i32,
        pub material: u32,
    }
}

/// White, so an entity given only a shape and `Look::default()` is seen.
impl Default for Look {
    fn default() -> Look {
        Look { colour: Colour::WHITE, layer: 0, material: 0 }
    }
}

impl Look {
    pub fn colour(colour: Colour) -> Look {
        Look { colour, ..Look::default() }
    }

    pub fn layer(self, layer: i32) -> Look {
        Look { layer, ..self }
    }

    pub fn material(self, material: u32) -> Look {
        Look { material, ..self }
    }
}

component! {
    /// A filled rectangle, `w` by `h` world units, centred on the place.
    /// Full sizes, not physics2d's half extents: this says what is seen.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Rect: "present::Rect" {
        pub w: f32,
        pub h: f32,
    }
}

component! {
    /// A filled circle centred on the place.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Circle: "present::Circle" {
        pub radius: f32,
    }
}

component! {
    /// A straight line from the place to the place plus `(dx, dy)` (turned
    /// with it), `width` world units thick.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Line: "present::Line" {
        pub dx: f32,
        pub dy: f32,
        pub width: f32,
    }
}

component! {
    /// One line of text, `size` world units tall. `anchor` is the point of
    /// the text's box that sits on the place, as fractions of its width and
    /// height: `[0, 0]` its top left (the default), `[0.5, 0.5]` its
    /// centre, `[1, 0]` its top right. The font is the presenter's: none is
    /// named until there are assets.
    #[derive(Debug, Default, PartialEq)]
    pub struct Text: "present::Text" {
        pub text: String,
        pub size: f32,
        pub anchor: [f32; 2],
    }
}

/// A character's advance, as a fraction of the text's size: what a
/// labelled text's box is estimated with, since there is no font to
/// measure. A monospace font's advance is about 0.6 of its size.
pub const TEXT_ADVANCE: f32 = 0.6;

component! {
    /// What an agent is told an entity is: its class (`what`: "ball",
    /// "paddle", "coin"), which text views turn into a legend and the
    /// vector view into slots, and an optional name for one of a class
    /// ("left"). Both are the game's vocabulary, for a reader that doesn't
    /// know the game.
    #[derive(Debug, Default, PartialEq)]
    pub struct Label: "present::Label" {
        pub what: String,
        pub name: String,
    }
}

impl Label {
    pub fn what(what: &str) -> Label {
        Label { what: what.into(), name: String::new() }
    }

    pub fn named(what: &str, name: &str) -> Label {
        Label { what: what.into(), name: name.into() }
    }
}

component! {
    /// The rectangle of the world to show: its top left (`x`, `y`) and size
    /// (`w`, `h`), in world units. A presenter fits it to its own output
    /// keeping its aspect (letterboxed), and fills what's around and
    /// behind with `clear`. One a world: with several, the extract takes
    /// the first and says how many it saw (`DrawList::cameras`).
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Camera: "present::Camera" {
        pub x: f32,
        pub y: f32,
        pub w: f32,
        pub h: f32,
        pub clear: Colour,
    }
}
