//! SPIKE (get-3hd.1): `spike_draw`'s interface, a stand-in for D1's
//! vocabulary: where a thing is (`Place`, the game's own transform in a
//! real game), what it looks like (`Look`), and the draw list the extract
//! makes from them each frame (`DrawList`, a flow). Nothing here names
//! wgpu: a game and a presenter both compile against it.

engine_api::component! {
    /// Centre, in pixels from the top left.
    #[derive(Debug, Default, Copy)]
    pub struct Place: "spike_draw::Place" {
        pub x: f32,
        pub y: f32,
    }
}

engine_api::component! {
    #[derive(Debug, Default, Copy)]
    pub struct Look: "spike_draw::Look" {
        pub w: f32,
        pub h: f32,
        /// RGBA8, red in the low byte.
        pub colour: u32,
        /// 0 a rect, 1 a circle.
        pub shape: u32,
        pub material: u32,
    }
}

/// One drawable as presenters see it: 32 bytes, the same layout as the
/// GPU presenter's vertex (`presentation_gfx::DrawItem`), so its upload
/// is a copy.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Item {
    pub pos: [f32; 2],
    pub size: [f32; 2],
    pub colour: u32,
    pub shape: u32,
    pub material: u32,
    pub layer: f32,
}

impl Item {
    pub fn of(p: &Place, l: &Look) -> Item {
        Item { pos: [p.x, p.y], size: [l.w, l.h], colour: l.colour, shape: l.shape, material: l.material, layer: 0.5 }
    }
}

engine_api::field_struct! {
    /// An `Item` as a service call carries it: `Item` itself is `repr(C)`
    /// for the presenter's upload, and isn't a `FieldType`, which a value
    /// crossing a call must be (making it one would be new unsafe code).
    #[derive(Debug, Default, Copy)]
    pub struct Drawn {
        pub pos: [f32; 2],
        pub size: [f32; 2],
        pub colour: u32,
        pub shape: u32,
    }
}

impl From<&Item> for Drawn {
    fn from(i: &Item) -> Drawn {
        Drawn { pos: i.pos, size: i.size, colour: i.colour, shape: i.shape }
    }
}

impl From<&Drawn> for Item {
    fn from(d: &Drawn) -> Item {
        Item { pos: d.pos, size: d.size, colour: d.colour, shape: d.shape, material: 0, layer: 0.5 }
    }
}

engine_api::service! {
    /// What a stage draws again between frames, over the last frame's
    /// list: the presenter calls it when the bootstrap asks for a redraw
    /// (`spike_platform::Redraw`), so a view of something that changes
    /// while no frames run (the turn barrier's status, a wait timer) stays
    /// current. Called outside a frame, so the provider reads the world
    /// with `cx.world()`. Its items are drawn last, in order, and should
    /// cover what they replace (start with an opaque background). Provided
    /// by `spike_pong_view`; `NotProvided` otherwise, and the presenter
    /// draws the last frame alone.
    pub trait Restage {
        fn restage() -> Vec<Drawn>;
    }
}

engine_api::flow! {
    /// The frame's draw list. `full`: `items` is every drawable, in a
    /// stable order (by material, then entity); otherwise `items` is
    /// last frame's list with `changed` applied, and a presenter that kept
    /// its own copy applies `changed` (slot, item) and skips `items`.
    /// `len` is the list's length either way.
    pub struct DrawList: "spike_draw::DrawList" {
        pub full: bool,
        pub len: u32,
        pub items: Vec<Item>,
        pub changed: Vec<(u32, Item)>,
        /// Which `spike_draw` build and mode made it, for the presenter's log.
        pub made_by: String,
        /// The size of the canvas the items are placed on, in their pixels:
        /// the presenter scales it to fit the window, letterboxed. 0 means
        /// the window's own pixels (the scene's way). Set by a stage that
        /// draws a fixed-size view, such as `spike_pong_view`.
        pub canvas_w: f32,
        pub canvas_h: f32,
    }
}
