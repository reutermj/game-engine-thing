//! A frame as a dependency graph: each system, then (if it can change the
//! world, or send events) its apply node, in plan order. A node may start
//! once every earlier node in the plan that it overlaps has finished. There
//! are no phase barriers: that rule alone makes any execution produce the
//! sequential frame. This module decides overlap; a scheduler decides when
//! to run what. See docs/architecture/storage.md.
//!
//! Overlap is decided from footprints:
//! - a system's comes from its parameters: the tables its queries match,
//!   the columns they read and write there, and its sparse sets. Queries are
//!   predicates, so a table an earlier apply is about to make counts too;
//! - an apply node's is a set of table *shapes*: before its system has run,
//!   bounded by the changing queries' tables (each shape a range, from the
//!   source minus what's removed up to the source plus what's added); after,
//!   exactly the tables its log touches.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};
use std::sync::Arc;

use crate::component::{Entity, Storage};
use crate::query::{Change, ParamDecl, QueryDecl};
use crate::world::{ComponentId, TableId, World};

/// What the graph needs to know of a system: its parameters, and whose it
/// is (a mod's systems all borrow its state mutably).
#[derive(Clone, Copy)]
pub struct SystemView<'a> {
    pub owner: usize,
    pub params: &'a [ParamDecl],
}

impl SystemView<'_> {
    fn queries(&self) -> impl Iterator<Item = &QueryDecl> {
        ParamDecl::leaves(self.params).into_iter().filter_map(ParamDecl::query)
    }

    fn events(&self) -> Vec<(usize, bool)> {
        ParamDecl::leaves(self.params)
            .into_iter()
            .filter_map(|p| match p {
                ParamDecl::Events { queue, write } => Some((*queue, *write)),
                _ => None,
            })
            .collect()
    }
}

/// The tables rows may be in: any set `s` with `lower ⊆ s ⊆ upper`. A
/// concrete table is `lower == upper`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Shape {
    pub lower: Vec<ComponentId>,
    pub upper: Vec<ComponentId>,
}

fn sorted(mut set: Vec<ComponentId>) -> Vec<ComponentId> {
    set.sort();
    set.dedup();
    set
}

impl Shape {
    pub fn exactly(set: Vec<ComponentId>) -> Shape {
        let set = sorted(set);
        Shape { lower: set.clone(), upper: set }
    }

    /// Whether some table of this shape is one `q` reads. The smallest
    /// table a match needs is `lower` plus `q`'s required components: it
    /// must fit under `upper`, and `lower` must avoid `q`'s exclusions.
    fn meets(&self, world: &World, q: &QueryDecl) -> bool {
        q.uses_tables(world)
            && q.table_with(world).iter().all(|c| self.upper.contains(c))
            && !q.table_without(world).iter().any(|c| self.lower.contains(c))
    }

    /// Whether some table is of both shapes.
    fn meets_shape(&self, other: &Shape) -> bool {
        let fits = |c: &ComponentId| self.upper.contains(c) && other.upper.contains(c);
        self.lower.iter().chain(&other.lower).all(fits)
    }
}

/// What an apply node may touch.
#[derive(Clone, Debug, Default)]
pub struct Footprint {
    /// Tables rows are moved out of, dropped from or land in.
    pub tables: Vec<Shape>,
    pub sparse: Vec<(ComponentId, bool)>,
    /// Event queues it publishes to.
    pub events: Vec<(usize, bool)>,
    /// Kills entities, which a query iterating a sparse set observes.
    pub kills: bool,
}

impl Footprint {
    fn add(&mut self, shape: Shape) {
        // Few distinct shapes per footprint, however many rows move.
        if !self.tables.contains(&shape) {
            self.tables.push(shape);
        }
    }

    fn add_sparse(&mut self, c: ComponentId) {
        if !self.sparse.contains(&(c, true)) {
            self.sparse.push((c, true));
        }
    }

    fn add_events(&mut self, q: usize) {
        if !self.events.contains(&(q, true)) {
            self.events.push((q, true));
        }
    }
}

fn table_only(world: &World, ids: &[ComponentId]) -> Vec<ComponentId> {
    ids.iter().copied().filter(|c| world.storage(*c) == Storage::Table).collect()
}

/// An apply node's footprint before its system has run: every table its
/// changing queries match, with any subset of their changes.
pub fn bound(world: &World, params: &[ParamDecl]) -> Footprint {
    let mut fp = Footprint::default();
    for p in ParamDecl::leaves(params) {
        // A reorder rearranges the whole of each spatial or ordered table
        // it matched.
        if let ParamDecl::Query(q) = p
            && q.reorders
        {
            for t in world.tables().filter(|t| (t.spatial.is_some() || t.ordered.is_some()) && q.matches(world, &t.components)) {
                fp.add(Shape::exactly(t.components.clone()));
            }
        }
        match p {
            ParamDecl::Query(q) if !q.changes.is_empty() => {
                for &c in q.changes.adds.iter().chain(&q.changes.removes) {
                    if world.storage(c) == Storage::Sparse {
                        fp.add_sparse(c);
                    }
                }
                fp.kills |= q.changes.despawns;
                let (adds, removes) = (table_only(world, &q.changes.adds), table_only(world, &q.changes.removes));
                if adds.is_empty() && removes.is_empty() && !q.changes.despawns {
                    continue;
                }
                // A sparse-driven query with no table filters could yield an
                // entity in any table.
                let anywhere = !q.uses_tables(world);
                for t in world.tables().filter(|t| anywhere || q.matches(world, &t.components)) {
                    fp.add(Shape::exactly(t.components.clone()));
                    if !adds.is_empty() || !removes.is_empty() {
                        let lower = t.components.iter().copied().filter(|c| !removes.contains(c)).collect();
                        let upper = t.components.iter().chain(&adds).copied().collect();
                        fp.add(Shape { lower: sorted(lower), upper: sorted(upper) });
                    }
                }
            }
            ParamDecl::Spawner { components } => {
                fp.add(Shape::exactly(table_only(world, components)));
                for &c in components.iter().filter(|c| world.storage(**c) == Storage::Sparse) {
                    fp.add_sparse(c);
                }
            }
            ParamDecl::Events { queue, write: true } => fp.add_events(*queue),
            ParamDecl::Query(_) | ParamDecl::Events { .. } | ParamDecl::Dt => {}
            ParamDecl::Group(_) => unreachable!("leaves are flattened"),
        }
    }
    fp
}

/// An apply node's footprint once its system has run: the tables its log
/// moves each entity through, replayed in order. Linear in the log with
/// small constants: it runs before every apply, and a log can hold a
/// thousand spawns.
pub fn exact(world: &World, log: &[Change]) -> Footprint {
    let mut fp = Footprint::default();
    // The table sets entities have been given so far, and, for each entity
    // the log has spawned or moved, its set now. Spawns from one spawner
    // share one set.
    let mut sets: Vec<Vec<ComponentId>> = Vec::new();
    let mut now: HashMap<Entity, usize, BuildHasherDefault<EntityHasher>> = HashMap::default();
    // Tables already in `fp`, by id, so a log of despawns adds each shape
    // once without comparing component lists.
    let mut seen: Vec<TableId> = Vec::new();
    let mut table = |fp: &mut Footprint, t: TableId| {
        if !seen.contains(&t) {
            seen.push(t);
            fp.add(Shape::exactly(world.table(t).components.clone()));
        }
    };
    let mut last_spawn: Option<(&Arc<[ComponentId]>, usize)> = None;
    for change in log {
        match change {
            Change::Insert { e, c, .. } | Change::Remove { e, c } => {
                if world.storage(*c) == Storage::Sparse {
                    fp.add_sparse(*c);
                    continue;
                }
                let i = match now.get(e) {
                    Some(&i) => i,
                    None => {
                        let Some(at) = world.entities.location(*e) else { continue };
                        table(&mut fp, at.table);
                        sets.push(world.table(at.table).components.clone());
                        sets.len() - 1
                    }
                };
                let set = &sets[i];
                let moved: Vec<ComponentId> = match change {
                    Change::Insert { .. } if !set.contains(c) => set.iter().chain([c]).copied().collect(),
                    Change::Remove { .. } if set.contains(c) => set.iter().copied().filter(|x| x != c).collect(),
                    _ => {
                        now.insert(*e, i);
                        continue;
                    }
                };
                let moved = sorted(moved);
                fp.add(Shape::exactly(moved.clone()));
                sets.push(moved);
                now.insert(*e, sets.len() - 1);
            }
            Change::Despawn(e) => {
                fp.kills = true;
                // An entity the log spawned or moved is in a table already
                // added; a dead one is in none.
                if !now.contains_key(e)
                    && let Some(at) = world.entities.location(*e)
                {
                    table(&mut fp, at.table);
                }
            }
            Change::Event { queue, .. } => fp.add_events(*queue),
            Change::Reorder(t) => table(&mut fp, *t),
            Change::Spawn { e, components, .. } => {
                let i = match last_spawn {
                    Some((last, i)) if Arc::ptr_eq(last, components) => i,
                    _ => {
                        let set = sorted(table_only(world, components));
                        fp.add(Shape::exactly(set.clone()));
                        for &c in components.iter().filter(|c| world.storage(**c) == Storage::Sparse) {
                            fp.add_sparse(c);
                        }
                        sets.push(set);
                        last_spawn = Some((components, sets.len() - 1));
                        sets.len() - 1
                    }
                };
                // Later changes in the log start from the spawn's table: the
                // entity has no location until the apply places it.
                now.insert(*e, i);
            }
        }
    }
    fp
}

/// Hashes entities for `exact`'s map: one multiply per word. The standard
/// hasher, keyed against flooding, was most of a spawn's footprint cost,
/// and entity ids aren't attacker-chosen.
#[derive(Default)]
pub struct EntityHasher(u64);

impl Hasher for EntityHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(b as u64);
        }
    }
    fn write_u32(&mut self, n: u32) {
        self.write_u64(n as u64);
    }
    fn write_u64(&mut self, n: u64) {
        self.0 = (self.0.rotate_left(26) ^ n).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    }
    fn finish(&self) -> u64 {
        self.0
    }
}

/// The columns `decl`'s queries touch in a table with exactly `set`, with
/// whether each is written.
fn touched(world: &World, decl: SystemView<'_>, set: &[ComponentId]) -> Vec<(ComponentId, bool)> {
    let mut out: Vec<(ComponentId, bool)> = Vec::new();
    for q in decl.queries().filter(|q| q.matches(world, set)) {
        for &(c, write) in q.terms.iter().filter(|(c, _)| world.storage(*c) == Storage::Table) {
            match out.iter_mut().find(|(x, _)| *x == c) {
                Some((_, w)) => *w |= write,
                None => out.push((c, write)),
            }
        }
    }
    out
}

fn sparse_of(world: &World, decl: SystemView<'_>) -> Vec<(ComponentId, bool)> {
    decl.queries().flat_map(|q| q.sparse(world)).collect()
}

fn sparse_conflict<K: PartialEq + Copy>(a: &[(K, bool)], b: &[(K, bool)]) -> bool {
    a.iter().any(|&(c, wa)| b.iter().any(|&(d, wb)| c == d && (wa || wb)))
}

pub fn systems_overlap(world: &World, x: SystemView<'_>, y: SystemView<'_>) -> bool {
    // Both borrow the mod's state mutably.
    if x.owner == y.owner {
        return true;
    }
    if sparse_conflict(&sparse_of(world, x), &sparse_of(world, y)) || sparse_conflict(&x.events(), &y.events()) {
        return true;
    }
    world.tables().any(|t| {
        let (a, b) = (touched(world, x, &t.components), touched(world, y, &t.components));
        a.iter().any(|&(c, wa)| b.iter().any(|&(d, wb)| c == d && (wa || wb)))
    })
}

pub fn system_apply_overlap(world: &World, x: SystemView<'_>, fp: &Footprint) -> bool {
    let observes_deaths = x.queries().any(|q| q.sparse_driven(world));
    if sparse_conflict(&sparse_of(world, x), &fp.sparse) || sparse_conflict(&x.events(), &fp.events) || (fp.kills && observes_deaths) {
        return true;
    }
    // Moving rows rewrites every column of both tables, and the rows: any
    // query reading either conflicts, reads included.
    fp.tables.iter().any(|shape| x.queries().any(|q| shape.meets(world, q)))
}

pub fn applies_overlap(a: &Footprint, b: &Footprint) -> bool {
    sparse_conflict(&a.sparse, &b.sparse)
        || sparse_conflict(&a.events, &b.events)
        || a.tables.iter().any(|s| b.tables.iter().any(|t| s.meets_shape(t)))
}
