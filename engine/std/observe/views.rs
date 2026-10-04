//! The views' contents, and how each is drawn or written: the rules an
//! observer follows and an agent can rely on.

use engine_api::Entity;
use play::{ActionState, Metrics, Playable};

use crate::json;

engine_api::field_struct! {
    /// A labelled entity as a view reports it: its id, class and name, the
    /// centre and size of its box, and its velocity, in world units (per
    /// second).
    #[derive(Debug, Default, PartialEq)]
    pub struct Seen {
        pub id: Entity,
        pub what: String,
        pub name: String,
        pub x: f32,
        pub y: f32,
        pub w: f32,
        pub h: f32,
        pub vx: f32,
        pub vy: f32,
    }
}

impl Seen {
    fn to_json(&self) -> String {
        let mut o = json::Object::new();
        o.string("id", &crate::id(self.id));
        o.string("what", &self.what);
        if !self.name.is_empty() {
            o.string("name", &self.name);
        }
        for (k, v) in [("x", self.x), ("y", self.y), ("w", self.w), ("h", self.h), ("vx", self.vx), ("vy", self.vy)] {
            o.float(k, v);
        }
        o.finish()
    }

    fn line(&self) -> String {
        let name = if self.name.is_empty() { String::new() } else { format!(" {}", self.name) };
        format!(
            "{} {}{name} at ({}, {}) size {} x {} moving ({}, {})",
            crate::id(self.id),
            self.what,
            short(self.x),
            short(self.y),
            short(self.w),
            short(self.h),
            short(self.vx),
            short(self.vy)
        )
    }
}

/// At most two decimals, no trailing zeros: what a list read by a model
/// needs, where 19.999998 is noise that costs tokens and attention.
fn short(v: f32) -> String {
    let s = format!("{v:.2}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

engine_api::field_struct! {
    /// A glyph of the text grid, and the label class it stands for.
    #[derive(Debug, Default, PartialEq)]
    pub struct Legend {
        pub glyph: String,
        pub what: String,
    }
}

impl Legend {
    /// Glyphs for classes, in order: each class's first letter or digit
    /// not yet taken, then its uppercase, then any letter or digit, so
    /// "ball" is `b` and "paddle" is `p` and a reader can guess most
    /// without the legend. `?` once all 62 are taken.
    pub fn assign(classes: &[&str]) -> Vec<Legend> {
        let mut taken: Vec<char> = Vec::new();
        classes
            .iter()
            .map(|&what| {
                let own = what.chars().filter(char::is_ascii_alphanumeric);
                let upper = what.chars().filter(char::is_ascii_alphabetic).map(|c| c.to_ascii_uppercase());
                let any = ('a'..='z').chain('A'..='Z').chain('0'..='9');
                let glyph = own.chain(upper).chain(any).find(|c| !taken.contains(c)).unwrap_or('?');
                taken.push(glyph);
                Legend { glyph: glyph.to_string(), what: what.into() }
            })
            .collect()
    }
}

engine_api::field_struct! {
    /// The text view: a grid of `rows`, column c covering x from `left +
    /// c * cell_w`, row r y from `top + r * cell_h`; the legend; and the
    /// entities drawn, with exact coordinates.
    #[derive(Debug, Default, PartialEq)]
    pub struct TextView {
        pub left: f32,
        pub top: f32,
        pub cell_w: f32,
        pub cell_h: f32,
        pub rows: Vec<String>,
        pub legend: Vec<Legend>,
        pub entities: Vec<Seen>,
    }
}

/// The most columns or rows a text view draws: a grid bigger than this
/// costs a model more than it can use. A region that would need more is
/// cut at the right and the bottom.
pub const MAX_CELLS: usize = 256;

impl TextView {
    /// The grid of `seen` over `region` (x, y, w, h) in cells of `cell`.
    /// An entity marks every cell whose centre its box covers (its low
    /// edges in, its high edges out); one smaller
    /// than a cell marks the cell its centre is in, so a ball always shows.
    /// Where two overlap, the later in `seen` is drawn. Entities outside the
    /// region are still listed, since the list is the exact account.
    pub fn draw(region: [f32; 4], cell: [f32; 2], seen: Vec<Seen>) -> TextView {
        let [left, top, w, h] = region;
        let cols = ((w / cell[0]).ceil() as usize).clamp(1, MAX_CELLS);
        let rows = ((h / cell[1]).ceil() as usize).clamp(1, MAX_CELLS);
        let mut classes: Vec<&str> = Vec::new();
        for s in &seen {
            if !classes.contains(&s.what.as_str()) {
                classes.push(&s.what);
            }
        }
        let legend = Legend::assign(&classes);
        let mut grid = vec![vec![' '; cols]; rows];
        for s in &seen {
            let glyph = legend.iter().find(|l| l.what == s.what).and_then(|l| l.glyph.chars().next()).unwrap_or('?');
            // The cells whose centres fall inside the box, by index: from
            // its low edge, up to but not on its high edge, so a box one
            // cell wide that lies along the grid marks one cell, not two.
            let first = |min: f32, origin: f32, size: f32| ((min - origin) / size - 0.5).ceil();
            let last = |max: f32, origin: f32, size: f32| ((max - origin) / size - 0.5).ceil() - 1.0;
            let (c0, c1) = (first(s.x - s.w / 2.0, left, cell[0]), last(s.x + s.w / 2.0, left, cell[0]));
            let (r0, r1) = (first(s.y - s.h / 2.0, top, cell[1]), last(s.y + s.h / 2.0, top, cell[1]));
            let (cs, rs) = if c0 <= c1 && r0 <= r1 {
                ((c0, c1), (r0, r1))
            } else {
                let c = ((s.x - left) / cell[0]).floor();
                let r = ((s.y - top) / cell[1]).floor();
                ((c, c), (r, r))
            };
            for r in rs.0.max(0.0) as i64..=rs.1.min(rows as f32 - 1.0) as i64 {
                for c in cs.0.max(0.0) as i64..=cs.1.min(cols as f32 - 1.0) as i64 {
                    grid[r as usize][c as usize] = glyph;
                }
            }
        }
        TextView {
            left,
            top,
            cell_w: cell[0],
            cell_h: cell[1],
            rows: grid.into_iter().map(|r| r.into_iter().collect()).collect(),
            legend,
            entities: seen,
        }
    }

    /// As an agent reads it: what a cell is, column numbers above, row
    /// numbers left, a border, the legend, then the entities.
    pub fn render(&self) -> String {
        let cols = self.rows.first().map_or(0, |r| r.chars().count());
        let margin = (self.rows.len().saturating_sub(1)).to_string().len();
        let pad = " ".repeat(margin);
        let mut out = format!(
            "column c is x {} + {} c, row r is y {} + {} r\n",
            short(self.left),
            short(self.cell_w),
            short(self.top),
            short(self.cell_h)
        );
        if cols > 10 {
            let tens: String = (0..cols).map(|c| if c % 10 == 0 { char::from(b'0' + (c / 10 % 10) as u8) } else { ' ' }).collect();
            out += &format!("{pad} {}\n", tens.trim_end());
        }
        let units: String = (0..cols).map(|c| char::from(b'0' + (c % 10) as u8)).collect();
        out += &format!("{pad} {units}\n");
        let border = format!("{pad}+{}+\n", "-".repeat(cols));
        out += &border;
        for (r, row) in self.rows.iter().enumerate() {
            out += &format!("{r:>margin$}|{row}|\n");
        }
        out += &border;
        let legend: Vec<String> = self.legend.iter().map(|l| format!("{} {}", l.glyph, l.what)).collect();
        out += &format!("legend: {}\n", if legend.is_empty() { "nothing labelled".into() } else { legend.join(", ") });
        out += "entities:\n";
        for s in &self.entities {
            out += &format!("  {}\n", s.line());
        }
        out
    }
}

engine_api::field_struct! {
    /// A seat as the JSON view reports it: its held actions and its
    /// metrics, by name.
    #[derive(Debug, Default, PartialEq)]
    pub struct SeatView {
        pub name: String,
        pub actions: Vec<(String, f32)>,
        pub metrics: Vec<(String, f32)>,
    }
}

impl SeatView {
    /// Seat `seat` of `playable`, from its components. A per-frame action
    /// reads 0, since it holds nothing.
    pub fn of(playable: &Playable, seat: u32, actions: &ActionState, metrics: &Metrics) -> SeatView {
        SeatView {
            name: playable.players.get(seat as usize).cloned().unwrap_or_default(),
            actions: playable.actions.iter().enumerate().map(|(i, a)| (a.name.clone(), actions.get(i as u32))).collect(),
            metrics: playable
                .metrics
                .iter()
                .enumerate()
                .map(|(i, m)| (m.name.clone(), metrics.values.get(i).copied().unwrap_or(m.min)))
                .collect(),
        }
    }
}

engine_api::field_struct! {
    /// The JSON view: the world rectangle shown (x, y, w, h), its labelled
    /// entities and the seats.
    #[derive(Debug, Default, PartialEq)]
    pub struct Structured {
        pub view: [f32; 4],
        pub entities: Vec<Seen>,
        pub seats: Vec<SeatView>,
    }
}

impl Structured {
    pub(crate) fn write(&self, o: &mut json::Object) {
        let mut view = json::Object::new();
        for (k, v) in ["x", "y", "w", "h"].into_iter().zip(self.view) {
            view.float(k, v);
        }
        o.raw("view", &view.finish());
        o.raw("entities", &json::array(self.entities.iter().map(Seen::to_json)));
        let seat = |s: &SeatView| {
            let mut so = json::Object::new();
            so.string("name", &s.name);
            for (key, pairs) in [("actions", &s.actions), ("metrics", &s.metrics)] {
                let mut p = json::Object::new();
                for (k, v) in pairs {
                    p.float(k, *v);
                }
                so.raw(key, &p.finish());
            }
            so.finish()
        };
        o.raw("seats", &json::array(self.seats.iter().map(seat)));
    }
}

engine_api::field_struct! {
    /// The vector view: `values` named by `names`, which is the game's
    /// `VectorSchema`: the same length, in the same order, every frame.
    #[derive(Debug, Default, PartialEq)]
    pub struct VectorView {
        pub names: Vec<String>,
        pub values: Vec<f32>,
    }
}

impl VectorView {
    pub(crate) fn write(&self, o: &mut json::Object) {
        o.raw("names", &json::array(self.names.iter().map(|n| json::string(n))));
        o.raw("values", &json::array(self.values.iter().map(|&v| json::float(v))));
    }
}

/// Each slot's numbers, after its `present`.
pub const SLOT_FIELDS: [&str; 6] = ["x", "y", "vx", "vy", "w", "h"];

/// What the vector view holds, from the game's declaration: each seat's
/// metrics, then each declared slot's entities, `present` and
/// `SLOT_FIELDS` for each, as RL wants it (TowerMind and EA's agents took
/// observations this way).
///
/// Every number is scaled to the view, so a policy sees the same numbers
/// whatever the game's units: positions from 0 to 1 across the shown
/// rectangle, sizes and velocities in view widths and heights (per
/// second), metrics from 0 at their `min` to 1 at their `max`. Nothing is
/// clamped: a ball outside the view reads outside 0 to 1, and is still
/// seen.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorSchema {
    pub names: Vec<String>,
}

impl VectorSchema {
    pub fn of(playable: &Playable) -> VectorSchema {
        let mut names = Vec::new();
        for seat in &playable.players {
            for m in &playable.metrics {
                names.push(format!("{seat}.{}", m.name));
            }
        }
        for slot in &playable.observed {
            for i in 0..slot.count {
                names.push(format!("{}{i}.present", slot.what));
                for f in SLOT_FIELDS {
                    names.push(format!("{}{i}.{f}", slot.what));
                }
            }
        }
        VectorSchema { names }
    }

    /// The values for one frame. `metrics` is each seat's, in seat order;
    /// `view` the rectangle shown. A class's entities fill its slots in
    /// order of name, then id, so "left" is always before "right" and a
    /// slot keeps its entity while the others come and go; ones beyond the
    /// slots are left out, and empty slots are all 0.
    pub fn fill(playable: &Playable, view: [f32; 4], seen: &[Seen], metrics: &[Metrics]) -> VectorView {
        let mut values = Vec::new();
        for i in 0..playable.players.len() {
            for (j, m) in playable.metrics.iter().enumerate() {
                let v = metrics.get(i).and_then(|ms| ms.values.get(j)).copied().unwrap_or(m.min);
                values.push((v - m.min) / (m.max - m.min));
            }
        }
        let [x0, y0, w, h] = view;
        for slot in &playable.observed {
            let mut of: Vec<&Seen> = seen.iter().filter(|s| s.what == slot.what).collect();
            of.sort_by(|a, b| (&a.name, a.id).cmp(&(&b.name, b.id)));
            for i in 0..slot.count as usize {
                match of.get(i) {
                    Some(s) => values.extend([1.0, (s.x - x0) / w, (s.y - y0) / h, s.vx / w, s.vy / h, s.w / w, s.h / h]),
                    None => values.extend([0.0; 1 + SLOT_FIELDS.len()]),
                }
            }
        }
        VectorView { names: VectorSchema::of(playable).names, values }
    }
}
