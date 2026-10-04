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
