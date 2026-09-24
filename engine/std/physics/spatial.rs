//! Spatial queries: a `Query` whose entities come from the spatial index,
//! which the physics step publishes into the world. See physics.md,
//! "Spatial queries".

use engine_api::engine_ecs::{Changes, Data, Declare, Filter, FrameCx, Param, ParamDecl};
use engine_api::{Entity, Query, Row, component, field_struct};

use crate::shapes::{Aabb, Hit, Placed, Ray, Shape, Vec2, overlaps, raycast};
use crate::{BOX, CIRCLE};

field_struct! {
    /// A collider as the index saw it at the end of the last step.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Indexed {
        pub entity: Entity,
        pub shape: u8,
        pub x: f32,
        pub y: f32,
        pub hx: f32,
        pub hy: f32,
    }
}

field_struct! {
    /// One grid cell a collider covers: `body` indexes `SpatialIndex::bodies`.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Cell {
        pub cx: i32,
        pub cy: i32,
        pub body: u32,
    }
}

component! {
    /// Every collider's shape at the end of the last step, in a uniform grid:
    /// what spatial queries search. One entity has it, written by the step.
    #[derive(Debug, Default, PartialEq)]
    pub struct SpatialIndex: "physics::SpatialIndex" {
        pub cell: f32,
        pub bodies: Vec<Indexed>,
        /// Sorted by cell, then body.
        pub cells: Vec<Cell>,
    }
}

impl Indexed {
    pub fn placed(&self) -> Placed {
        let shape = if self.shape == CIRCLE { Shape::Circle(self.hx) } else { Shape::Box(Vec2::new(self.hx, self.hy)) };
        Placed { shape, at: Vec2::new(self.x, self.y) }
    }
}

impl SpatialIndex {
    /// The cells `aabb` covers, as an inclusive range per axis.
    pub fn cell_range(cell: f32, aabb: &Aabb) -> ((i32, i32), (i32, i32)) {
        let c = |v: f32| (v / cell).floor() as i32;
        ((c(aabb.min.x), c(aabb.max.x)), (c(aabb.min.y), c(aabb.max.y)))
    }

    /// The bodies whose cells `aabb` touches, each once, in index order:
    /// candidates, to be tested exactly.
    pub fn candidates(&self, aabb: &Aabb) -> Vec<u32> {
        if self.cell <= 0.0 {
            return Vec::new();
        }
        let ((x0, x1), (y0, y1)) = Self::cell_range(self.cell, aabb);
        let mut out = Vec::new();
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                let start = self.cells.partition_point(|c| (c.cx, c.cy) < (cx, cy));
                out.extend(self.cells[start..].iter().take_while(|c| (c.cx, c.cy) == (cx, cy)).map(|c| c.body));
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// A query over what's somewhere: `Data`, `Filter` and `Changes` as for
/// [`Query`], with entities found through the spatial index. Shapes are as
/// of the last physics step; the items are current.
pub struct Spatial<'w, D: Data, F = (), C = ()> {
    query: Query<'w, D, F, C>,
    index: Query<'w, &'static SpatialIndex>,
}

impl<D: Data + 'static, F: Filter, C: Changes> Param for Spatial<'static, D, F, C> {
    type Item<'w> = Spatial<'w, D, F, C>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        <(Query<'static, D, F, C>, Query<'static, &'static SpatialIndex>)>::declare(d)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Spatial<'w, D, F, C> {
        let (query, index) = <(Query<'static, D, F, C>, Query<'static, &'static SpatialIndex>)>::fetch(cx, decl);
        Spatial { query, index }
    }
}

impl<D: Data, F, C> Spatial<'_, D, F, C> {
    /// The indexed colliders `keep` accepts among those near `aabb`, copied
    /// out so the index's guard isn't held while the caller runs.
    fn search(&mut self, aabb: Aabb, mut keep: impl FnMut(&Indexed) -> bool) -> Vec<Indexed> {
        self.index
            .single(|_, index| index.candidates(&aabb).into_iter().map(|i| index.bodies[i as usize]).filter(|b| keep(b)).collect())
            .unwrap_or_default()
    }

    /// Every entity the query matches whose collider overlaps `probe`, in
    /// entity order, as `for_each` would hand it.
    pub fn overlapping(&mut self, probe: Placed, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let found = self.search(probe.aabb(), |b| overlaps(&b.placed(), &probe));
        let mut entities: Vec<Entity> = found.iter().map(|b| b.entity).collect();
        entities.sort();
        for e in entities {
            self.query.with(e, &mut f);
        }
    }

    /// Whether any entity the query matches has a collider at `point`.
    pub fn any_at(&mut self, point: Vec2) -> bool {
        let mut any = false;
        self.overlapping(Placed { shape: Shape::Box(Vec2::ZERO), at: point }, |_, _| any = true);
        any
    }

    /// The entities the query matches along `ray`, nearest first, each
    /// with where it was hit, until `f` returns `Some`.
    pub fn cast<R>(&mut self, ray: Ray, mut f: impl FnMut(Hit, Row<'_>, D::Items<'_>) -> Option<R>) -> Option<R> {
        let found = self.search(ray.aabb(), |_| true);
        let mut hits: Vec<(Hit, Entity)> =
            found.iter().filter_map(|b| raycast(&ray, &b.placed()).map(|h| (h, b.entity))).collect();
        // Ties by entity, so the order doesn't depend on the index's.
        hits.sort_by(|(a, ea), (b, eb)| a.t.total_cmp(&b.t).then(ea.cmp(eb)));
        for (hit, e) in hits {
            if let Some(Some(r)) = self.query.with(e, |row, items| f(hit, row, items)) {
                return Some(r);
            }
        }
        None
    }
}

/// Builds the index from colliders' current shapes: the step's last act.
pub fn build_index(cell: f32, colliders: impl IntoIterator<Item = (Entity, Placed)>, into: &mut SpatialIndex) {
    into.cell = cell;
    into.bodies.clear();
    into.cells.clear();
    for (entity, placed) in colliders {
        let (shape, h) = match placed.shape {
            Shape::Box(h) => (BOX, h),
            Shape::Circle(r) => (CIRCLE, Vec2::new(r, r)),
        };
        let body = into.bodies.len() as u32;
        into.bodies.push(Indexed { entity, shape, x: placed.at.x, y: placed.at.y, hx: h.x, hy: h.y });
        let ((x0, x1), (y0, y1)) = SpatialIndex::cell_range(cell, &placed.aabb());
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                into.cells.push(Cell { cx, cy, body });
            }
        }
    }
    into.cells.sort_unstable_by_key(|c| (c.cx, c.cy, c.body));
}
