//! The draw list: the frame's drawables as presenters see them, made by
//! `present`'s extract in the render phase and read with `See<DrawList>`
//! by every presenter after it (`.after("present::extract")`).
//!
//! **Rebuilt whole each frame** (presentation.md, D2): a full extract cost
//! 1.1 ns a drawable in the spike, cheaper than keeping a list whenever a
//! game moves things through `&mut` walks. **Shapes come in material
//! runs**, sorted by layer and then material, so a GPU presenter draws a
//! run as one instanced call: a draw an item cost a hundred times as much
//! (presentation-spike.md, sections 4 and 5).
//!
//! **A delta flow comes beside this one, never inside it.** For large sets
//! that rarely change (a tilemap, sleeping bodies) a later `DrawDelta`
//! flow would carry what changed, for presenters that keep their own copy,
//! and a game would opt entities into it with a marker. Entities drawn
//! that way would leave this list, so it carries `retained`, how many
//! drawables it leaves out: 0 until then, and a presenter that reads only
//! this list can tell, and say so, rather than draw a frame that's
//! silently missing things.

use engine_api::Entity;

use crate::{Camera, Colour};

/// What an `Item` draws. A fieldless `repr(u32)`, so an item's bytes can
/// be uploaded as they are.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Shape {
    /// A rectangle `size` across, turned by `rot`.
    #[default]
    Rect = 0,
    /// A circle `size[0]` across.
    Circle = 1,
    /// A line `size[0]` long and `size[1]` thick, turned by `rot`: drawn
    /// as a rectangle, kept apart so a text presenter can draw it as one.
    Line = 2,
}

/// One shape, in world units. `repr(C)` with only 4-byte fields, so a
/// presenter can upload a run of them as instances without reshaping
/// (the spike's vertex layout).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Item {
    /// Centre.
    pub pos: [f32; 2],
    /// Full width and height.
    pub size: [f32; 2],
    /// The cosine and sine of its angle.
    pub rot: [f32; 2],
    pub colour: Colour,
    pub shape: Shape,
    pub layer: i32,
    pub material: u32,
}

/// One line of text: `text` at `pos`, `size` tall, `anchor` as `Text`'s.
/// Upright: a place's turn doesn't turn its text, until a presenter can
/// lay text out turned.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TextItem {
    pub entity: Entity,
    pub text: String,
    pub pos: [f32; 2],
    pub size: f32,
    pub anchor: [f32; 2],
    pub colour: Colour,
    pub layer: i32,
}

/// A run of `items` that share a layer and a material.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Run {
    pub layer: i32,
    pub material: u32,
    pub start: u32,
    pub len: u32,
}

/// A labelled entity, as agents' views list it: its id, class and name,
/// and the box its shapes cover (axis-aligned, world units; a point at its
/// place when it has none).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Labelled {
    pub entity: Entity,
    /// An index into `DrawList::names`: classes repeat ("coin" a thousand
    /// times), so each is one string a frame.
    pub what: u32,
    pub name: String,
    /// The box's centre and full size.
    pub pos: [f32; 2],
    pub size: [f32; 2],
}

engine_api::flow! {
    /// The frame's drawables. Shapes are `items`, sorted into `runs`, and
    /// drawn in layer order, a layer's shapes before its text; `texts` are
    /// sorted by layer, in the order the extract found them within one.
    pub struct DrawList: "present::DrawList" {
        pub items: Vec<Item>,
        /// `items[i]`'s entity, for a presenter that reports what it drew.
        pub entities: Vec<Entity>,
        pub runs: Vec<Run>,
        pub texts: Vec<TextItem>,
        pub labels: Vec<Labelled>,
        /// The label classes `labels` name, in the order first met.
        pub names: Vec<String>,
        /// The world rectangle to show: the first `Camera`'s, or, with
        /// none (`cameras` 0), the box around every shape and label; `None`
        /// only when there's neither a camera nor anything to fit.
        pub view: Option<Camera>,
        /// How many cameras the world has. More than one is not yet
        /// meaningful (no split screens): the first is used.
        pub cameras: u32,
        /// Drawables a delta flow carries instead of this list (see the
        /// module's comment): 0, until there is one.
        pub retained: u32,
    }
}

impl DrawList {
    /// A run's items.
    pub fn run(&self, run: &Run) -> &[Item] {
        &self.items[run.start as usize..(run.start + run.len) as usize]
    }

    /// A label's class.
    pub fn what(&self, label: &Labelled) -> &str {
        &self.names[label.what as usize]
    }

    /// Whether the list keeps its promises: `entities` beside `items`,
    /// runs that cover the items in order, one per layer and material,
    /// sorted by them, and texts sorted by layer. What a presenter's
    /// debug checks and the extract's tests hold it to.
    pub fn check(&self) -> Result<(), String> {
        if self.entities.len() != self.items.len() {
            return Err(format!("{} items but {} entities", self.items.len(), self.entities.len()));
        }
        let mut next = 0u32;
        let mut last: Option<(i32, u32)> = None;
        for (i, run) in self.runs.iter().enumerate() {
            if run.start != next || run.len == 0 {
                return Err(format!("run {i} starts at {} with {} items, after {next} items", run.start, run.len));
            }
            let key = (run.layer, run.material);
            if last.is_some_and(|l| l >= key) {
                return Err(format!("run {i} (layer {}, material {}) is out of order", run.layer, run.material));
            }
            last = Some(key);
            if let Some(item) = self.run(run).iter().find(|item| (item.layer, item.material) != key) {
                return Err(format!(
                    "run {i} is layer {}, material {}, but holds an item of layer {}, material {}",
                    run.layer, run.material, item.layer, item.material
                ));
            }
            next = run.start + run.len;
        }
        if next as usize != self.items.len() {
            return Err(format!("the runs cover {next} of {} items", self.items.len()));
        }
        if !self.texts.is_sorted_by_key(|t| t.layer) {
            return Err("texts are out of layer order".into());
        }
        if let Some(l) = self.labels.iter().find(|l| l.what as usize >= self.names.len()) {
            return Err(format!("label of {:?} names class {} of {}", l.entity, l.what, self.names.len()));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(layer: i32, material: u32) -> Item {
        Item { layer, material, ..Item::default() }
    }

    fn list(items: &[Item], runs: &[(i32, u32, u32, u32)]) -> DrawList {
        DrawList {
            items: items.to_vec(),
            entities: vec![Entity::DEAD; items.len()],
            runs: runs.iter().map(|&(layer, material, start, len)| Run { layer, material, start, len }).collect(),
            ..DrawList::default()
        }
    }

    #[test]
    fn a_list_in_runs_passes() {
        let l = list(&[item(0, 0), item(0, 0), item(0, 1), item(2, 0)], &[(0, 0, 0, 2), (0, 1, 2, 1), (2, 0, 3, 1)]);
        assert_eq!(l.check(), Ok(()));
        assert_eq!(l.run(&l.runs[0]).len(), 2);
    }

    #[test]
    fn broken_lists_are_named() {
        let cases: [(DrawList, &str); 7] = [
            (list(&[item(0, 0), item(0, 1), item(0, 1)], &[(0, 0, 0, 1), (0, 1, 2, 1)]), "run 1 starts at 2 with 1 items, after 1 items"),
            // Two runs of one key: one would do.
            (list(&[item(0, 0), item(0, 0)], &[(0, 0, 0, 1), (0, 0, 1, 1)]), "run 1 (layer 0, material 0) is out of order"),
            (list(&[item(0, 1), item(0, 0)], &[(0, 1, 0, 1), (0, 0, 1, 1)]), "run 1 (layer 0, material 0) is out of order"),
            (list(&[item(0, 0), item(1, 0)], &[(0, 0, 0, 2)]), "run 0 is layer 0, material 0, but holds an item of layer 1, material 0"),
            (list(&[item(0, 0), item(0, 0)], &[(0, 0, 0, 1)]), "the runs cover 1 of 2 items"),
            (list(&[item(0, 0)], &[(0, 0, 0, 0)]), "run 0 starts at 0 with 0 items, after 0 items"),
            (
                DrawList { items: vec![item(0, 0)], runs: vec![Run { layer: 0, material: 0, start: 0, len: 1 }], ..DrawList::default() },
                "1 items but 0 entities",
            ),
        ];
        for (l, why) in cases {
            assert_eq!(l.check(), Err(why.to_string()));
        }
    }

    #[test]
    fn texts_and_labels_are_checked() {
        let mut l = list(&[], &[]);
        l.texts = vec![TextItem { layer: 1, ..TextItem::default() }, TextItem { layer: 0, ..TextItem::default() }];
        assert_eq!(l.check(), Err("texts are out of layer order".into()));
        l.texts.reverse();
        l.labels = vec![Labelled { what: 0, ..Labelled::default() }];
        assert!(l.check().unwrap_err().contains("names class 0 of 0"));
        l.names.push("ball".into());
        assert_eq!(l.check(), Ok(()));
        assert_eq!(l.what(&l.labels[0]), "ball");
    }

    #[test]
    fn an_item_is_ten_words_with_no_padding() {
        // What an instance upload copies: a layout change is a presenter's
        // shader change too.
        assert_eq!(size_of::<Item>(), 40);
        assert_eq!(std::mem::offset_of!(Item, colour), 24);
        assert_eq!(std::mem::offset_of!(Item, material), 36);
    }

    #[test]
    fn colours_read_as_written() {
        assert_eq!(Colour::rgb(0x3cd2f0), Colour { r: 0x3c, g: 0xd2, b: 0xf0, a: 0xff });
        assert_eq!(Colour::WHITE.with_alpha(0x80).a, 0x80);
    }
}
