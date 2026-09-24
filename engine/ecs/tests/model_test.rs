//! Differential test of the storage: random structural changes applied to
//! the world and to a trivially correct model (a map from entity to
//! components), compared after every step. Covers table moves between
//! pages, heap values (drops), a zero-sized tag and a sparse component.

use std::collections::BTreeMap;

use engine_ecs::world::PAGE_ROWS;
use engine_ecs::{Build, Component, ComponentId, Entity, Structural, World, component};

component! {
    #[derive(Debug, Default, PartialEq)]
    struct A: "test::A" { n: u32 }
}

component! {
    #[derive(Debug, Default, PartialEq)]
    struct B: "test::B" { s: String }
}

component! {
    #[derive(Debug, Default, PartialEq)]
    struct Tag: "test::Tag" {}
}

component! {
    #[derive(Debug, Default, PartialEq)]
    struct S: "test::S", storage = sparse { n: u64 }
}

#[derive(Clone, Debug, Default, PartialEq)]
struct Model {
    a: Option<A>,
    b: Option<B>,
    tag: bool,
    s: Option<S>,
}

/// xorshift64*: deterministic, and enough for choosing operations.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

struct Ids {
    a: ComponentId,
    b: ComponentId,
    tag: ComponentId,
    s: ComponentId,
}

fn world() -> (World, Ids) {
    let w = World::new();
    let ids = {
        let m = w.between_frames(Build::default()).unwrap();
        Ids { a: m.id::<A>(), b: m.id::<B>(), tag: m.id::<Tag>(), s: m.id::<S>() }
    };
    (w, ids)
}

fn id<T: Component>(w: &World) -> ComponentId {
    w.id(T::NAME).unwrap()
}

/// The world's contents, as the model would hold them.
fn contents(w: &World) -> BTreeMap<Entity, Model> {
    let mut out: BTreeMap<Entity, Model> = BTreeMap::new();
    let (ia, ib, itag) = (id::<A>(w), id::<B>(w), id::<Tag>(w));
    for t in w.tables() {
        let rows = t.rows.read().unwrap();
        let cols: Vec<_> = t.columns.iter().map(|c| c.read().unwrap()).collect();
        assert_eq!(rows.len(), cols.first().map_or(rows.len(), |c| c.len()), "every column has the table's pages");
        for (p, page) in rows.iter().enumerate() {
            assert!(page.len() <= PAGE_ROWS);
            for col in &cols {
                assert_eq!(col[p].len(), page.len(), "every column has the page's rows");
            }
            for (r, &e) in page.iter().enumerate() {
                let loc = w.entities.location(e).expect("a row's entity is placed");
                assert_eq!((loc.table, loc.page as usize, loc.row as usize), (t.id, p, r), "location matches row");
                let mut m = Model::default();
                for (i, &c) in t.components.iter().enumerate() {
                    if c == ia {
                        m.a = Some(cols[i][p].as_slice::<A>()[r].clone());
                    } else if c == ib {
                        m.b = Some(cols[i][p].as_slice::<B>()[r].clone());
                    } else if c == itag {
                        m.tag = true;
                    }
                }
                assert!(out.insert(e, m).is_none(), "an entity in two rows");
            }
        }
    }
    let set = w.sparse_set(id::<S>(w)).read().unwrap();
    for (e, m) in out.iter_mut() {
        m.s = set.get::<S>(*e).cloned();
    }
    out
}

fn run(seed: u64, steps: usize) {
    let (w, ids) = world();
    let mut model: BTreeMap<Entity, Model> = BTreeMap::new();
    let mut dead: Vec<Entity> = Vec::new();
    let mut rng = Rng(seed | 1);
    for step in 0..steps {
        let live: Vec<Entity> = model.keys().copied().collect();
        let pick = |rng: &mut Rng| -> Entity {
            // Sometimes a dead entity, which every operation must ignore.
            if !dead.is_empty() && (live.is_empty() || rng.below(8) == 0) {
                dead[rng.below(dead.len() as u64) as usize]
            } else {
                live[rng.below(live.len() as u64) as usize]
            }
        };
        // Sparse entries of dead entities are only purged when a set is
        // written with purging; half the time, leave them, as a frame would.
        let mut s = Structural::new(&w);
        let tables: Vec<_> = w.tables().map(|t| t.id).collect();
        for t in tables {
            s.lock_table(t);
        }
        s.lock_sparse_with(ids.s, rng.below(2) == 0);
        let op = if live.is_empty() && dead.is_empty() { 0 } else { rng.below(10) };
        match op {
            0 | 1 => {
                // Reuses dead entities' indices, so a new entity can meet a
                // dead one's sparse entry.
                let e = w.entities.reserve();
                let n = rng.below(1000) as u32;
                s.spawn(e, (A { n }, B { s: format!("b{n}") }), &[ids.a, ids.b]);
                model.insert(e, Model { a: Some(A { n }), b: Some(B { s: format!("b{n}") }), ..Model::default() });
            }
            2 => {
                let e = pick(&mut rng);
                let n = rng.below(1000) as u32;
                s.insert_id(e, ids.a, A { n });
                if let Some(m) = model.get_mut(&e) {
                    m.a = Some(A { n });
                }
            }
            3 => {
                let e = pick(&mut rng);
                s.insert_id(e, ids.b, B { s: format!("x{step}") });
                if let Some(m) = model.get_mut(&e) {
                    m.b = Some(B { s: format!("x{step}") });
                }
            }
            4 => {
                let e = pick(&mut rng);
                s.insert_id(e, ids.tag, Tag {});
                if let Some(m) = model.get_mut(&e) {
                    m.tag = true;
                }
            }
            5 => {
                let e = pick(&mut rng);
                s.insert_id(e, ids.s, S { n: step as u64 });
                if let Some(m) = model.get_mut(&e) {
                    m.s = Some(S { n: step as u64 });
                }
            }
            6 => {
                let e = pick(&mut rng);
                match rng.below(4) {
                    0 => {
                        s.remove_id(e, ids.a);
                        model.get_mut(&e).map(|m| m.a = None);
                    }
                    1 => {
                        s.remove_id(e, ids.b);
                        model.get_mut(&e).map(|m| m.b = None);
                    }
                    2 => {
                        s.remove_id(e, ids.tag);
                        model.get_mut(&e).map(|m| m.tag = false);
                    }
                    _ => {
                        s.remove_id(e, ids.s);
                        model.get_mut(&e).map(|m| m.s = None);
                    }
                }
            }
            _ => {
                let e = pick(&mut rng);
                s.despawn(e);
                if model.remove(&e).is_some() {
                    dead.push(e);
                }
            }
        }
        drop(s);
        let got = contents(&w);
        assert_eq!(got, model, "seed {seed}, step {step}, op {op}");
    }
}

#[test]
fn random_changes_match_the_model() {
    for seed in 1..=20 {
        run(seed, 600);
    }
}

#[test]
fn long_runs_fill_and_empty_several_pages() {
    run(0xfeed, 5000);
}
