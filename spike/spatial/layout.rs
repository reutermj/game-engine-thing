//! What every layout stores and answers. A row is one body: its shape, its
//! motion, and a payload standing for the other components a real row
//! carries, so moving a row costs what it would in the engine.

use physics::{Aabb, Vec2};

/// The other components' bytes, as `u64`s: about a hundred bytes, a body
/// with a few game components.
pub const PAYLOAD: usize = 12;

/// Rows per page in the spatial layouts: small, since a page is now a
/// neighborhood, not only a unit of borrowing.
pub const PAGE: usize = 64;

/// A body bigger than this (its larger half extent) is "big": walls, floors.
pub const BIG: f32 = 2.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Row {
    pub id: u32,
    pub pos: Vec2,
    pub half: Vec2,
    pub circle: bool,
    pub vel: Vec2,
    /// 0 for statics.
    pub inv_mass: f32,
    pub payload: [u64; PAYLOAD],
}

impl Row {
    pub fn aabb(&self) -> Aabb {
        Aabb { min: self.pos - self.half, max: self.pos + self.half }
    }

    pub fn grown(&self, by: f32) -> Aabb {
        let g = Vec2::new(by, by);
        Aabb { min: self.pos - self.half - g, max: self.pos + self.half + g }
    }

    pub fn moves(&self) -> bool {
        self.inv_mass > 0.0
    }

    pub fn big(&self) -> bool {
        self.half.x.max(self.half.y) > BIG
    }
}

/// What an upkeep did: rows moved to another place in storage.
#[derive(Clone, Copy, Debug, Default)]
pub struct Upkeep {
    pub moved: usize,
}

/// What a query or broadphase looked at.
#[derive(Clone, Copy, Debug, Default)]
pub struct Visit {
    pub pages: usize,
    pub rows: usize,
}

impl std::ops::AddAssign for Visit {
    fn add_assign(&mut self, o: Visit) {
        self.pages += o.pages;
        self.rows += o.rows;
    }
}

pub fn union(a: Aabb, b: Aabb) -> Aabb {
    Aabb { min: Vec2::new(a.min.x.min(b.min.x), a.min.y.min(b.min.y)), max: Vec2::new(a.max.x.max(b.max.x), a.max.y.max(b.max.y)) }
}

pub const EMPTY: Aabb = Aabb {
    min: Vec2 { x: f32::INFINITY, y: f32::INFINITY },
    max: Vec2 { x: f32::NEG_INFINITY, y: f32::NEG_INFINITY },
};

/// A table of bodies laid out one way. Ids are dense, `0..n`.
pub trait Layout {
    fn name(&self) -> String;
    fn get(&self, id: u32) -> &Row;
    /// A write to one row, which may move it: what a game's respawn is.
    fn get_mut(&mut self, id: u32) -> &mut Row;
    /// A write to every row's velocity, which moves nothing: gravity.
    fn each_velocity(&mut self, f: &mut dyn FnMut(&mut Row));
    /// Puts rows whose writes moved them where they now belong. What an
    /// apply node after a writer would run; a no-op for the baseline, which
    /// rebuilds its index here instead.
    fn upkeep(&mut self) -> Upkeep;
    /// The ids whose boxes overlap `region`, in any order.
    fn query(&self, region: &Aabb, out: &mut Vec<u32>) -> Visit;
    /// Every pair `(a, b)`, `a < b`, at least one moving, whose boxes grown
    /// by `grow` overlap; sorted, each once. May include no others.
    fn pairs(&self, grow: f32, out: &mut Vec<(u32, u32)>) -> Visit;
    fn pages(&self) -> usize;
}

/// The pairs a correct broadphase finds, by testing every pair.
pub fn brute_pairs(rows: &[Row], grow: f32) -> Vec<(u32, u32)> {
    let mut out = Vec::new();
    for (i, a) in rows.iter().enumerate() {
        for b in &rows[i + 1..] {
            if (a.moves() || b.moves()) && a.grown(grow).overlaps(&b.grown(grow)) {
                out.push((a.id.min(b.id), a.id.max(b.id)));
            }
        }
    }
    out.sort_unstable();
    out
}

/// Tests a pair and keeps it if it counts, for the layouts' broadphases.
pub fn keep(a: &Row, b: &Row, grow: f32, out: &mut Vec<(u32, u32)>) {
    if (a.moves() || b.moves()) && a.grown(grow).overlaps(&b.grown(grow)) {
        out.push((a.id.min(b.id), a.id.max(b.id)));
    }
}
