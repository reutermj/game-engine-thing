//! Spatial queries: a `Query` over what's somewhere, found through the
//! spatial order the ECS keeps positions in, then tested against colliders'
//! exact shapes. See physics.md, "Spatial queries", and spatial-storage.md.

use engine_api::engine_ecs::{Changes, Data, Declare, Filter, FrameCx, Param, ParamDecl};
use engine_api::{Bounds, Entity, Query, Row};

use crate::shapes::{Aabb, Hit, Placed, Ray, Shape, Vec2, overlaps, raycast};
use crate::{Collider, Position};

/// A query over what's somewhere: `Data`, `Filter` and `Changes` as for
/// [`Query`], with entities found by where their colliders are. Shapes are
/// as of the last re-sort, after whoever last moved them; the items are
/// current. `Data` can't write `Position` or `Collider`, which the shapes
/// are read from: move things with a query of your own, and its apply node
/// puts them where a later `Spatial` finds them.
pub struct Spatial<'w, D: Data, F = (), C = ()> {
    query: Query<'w, D, F, C>,
    shapes: Query<'w, (&'static Position, &'static Collider)>,
}

impl<D: Data + 'static, F: Filter, C: Changes> Param for Spatial<'static, D, F, C> {
    type Item<'w> = Spatial<'w, D, F, C>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        <(Query<'static, D, F, C>, Query<'static, (&'static Position, &'static Collider)>)>::declare(d)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Spatial<'w, D, F, C> {
        let (query, shapes) = <(Query<'static, D, F, C>, Query<'static, (&'static Position, &'static Collider)>)>::fetch(cx, decl);
        Spatial { query, shapes }
    }
}

fn bounds(a: &Aabb) -> Bounds {
    Bounds::new([a.min.x, a.min.y], [a.max.x, a.max.y])
}

fn placed(p: &Position, c: &Collider) -> Placed {
    Placed { shape: Shape::of(c), at: Vec2::new(p.x, p.y) }
}

impl<D: Data, F, C> Spatial<'_, D, F, C> {
    /// The colliders near `aabb` that `keep` accepts, with their shapes,
    /// copied out so the shapes' guards aren't held while the caller runs.
    fn search(&mut self, aabb: Aabb, mut keep: impl FnMut(&Placed) -> bool) -> Vec<(Entity, Placed)> {
        let mut out = Vec::new();
        self.shapes.in_region(bounds(&aabb), |row, (p, c)| {
            let shape = placed(p, c);
            if keep(&shape) {
                out.push((row.entity(), shape));
            }
        });
        out
    }

    /// Every entity the query matches whose collider overlaps `probe`, in
    /// entity order, as `for_each` would hand it.
    pub fn overlapping(&mut self, probe: Placed, mut f: impl FnMut(Row<'_>, D::Items<'_>)) {
        let mut entities: Vec<Entity> = self.search(probe.aabb(), |s| overlaps(s, &probe)).into_iter().map(|(e, _)| e).collect();
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
        let mut hits: Vec<(Hit, Entity)> = found.iter().filter_map(|(e, s)| raycast(&ray, s).map(|h| (h, *e))).collect();
        // Ties by entity, so the order doesn't depend on storage's.
        hits.sort_by(|(a, ea), (b, eb)| a.t.total_cmp(&b.t).then(ea.cmp(eb)));
        for (hit, e) in hits {
            if let Some(Some(r)) = self.query.with(e, |row, items| f(hit, row, items)) {
                return Some(r);
            }
        }
        None
    }
}
