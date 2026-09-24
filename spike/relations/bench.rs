//! Numbers for the relationships design (get-emj.18): each contact model's
//! stages over a 1000-body pile, falling and settled; then structural links.
//! `./bazel run -c opt //spike/relations:bench`.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule, SystemDecl};
use engine_ecs::{Build, Despawns, Query, Spawner, With, Without, World};
use relations::cases::{entity_one_way, table_one_way};
use relations::common::{Body, Found, Grounded, Player, Pos, Shape, Vel, detect, has, pile};
use relations::entities::{self, ContactIndex, ContactOf, Impulse, Manifold, Response};
use relations::links::*;
use relations::table::{self, ContactTable, Contacts};

const BODIES: usize = 1000;

fn table_hook_all(_: &mut Cx, mut contacts: Contacts<(), With<Body>>) {
    contacts.for_each(|c, _, ()| c.set_restitution(0.2));
}

fn entity_hook_all(_: &mut Cx, mut contacts: Query<(&ContactOf, &mut Response)>, mut bodies: Query<(), With<Body>>) {
    contacts.for_each(|_, (pair, mut r)| {
        if has(&mut bodies, pair.a) || has(&mut bodies, pair.b) {
            r.restitution = 0.2;
        }
    });
}

const STAGES: [&str; 6] = ["gravity", "detect", "upkeep", "hook: all", "hook: one", "solve"];

/// Each stage its own schedule, so each can be timed.
fn stages(w: &World, entity: bool) -> Vec<Schedule> {
    let one = |d: SystemDecl| Schedule { systems: vec![d] };
    // Detection hands its contacts to the model's merge, so the two are
    // timed apart: detection is the same for both, and the slow part.
    let found = Arc::new(Mutex::new(Vec::<Found>::new()));
    let (into, from) = (found.clone(), found);
    let detect = (move |_: &mut Cx, mut bodies: Query<(&Pos, &Shape, &Vel, &Body)>, mut statics: Query<(&Pos, &Shape), Without<Body>>| {
        *into.lock().unwrap() = detect(&mut bodies, &mut statics);
    })
    .system(w, "detect");
    if entity {
        entities::setup(w);
        vec![
            one(entities::gravity_system.system(w, "gravity")),
            one(detect),
            one((move |_: &mut Cx, index: Query<&mut ContactIndex>, contacts: Query<(&mut Manifold, &mut Response), (), Despawns>, spawner: Spawner<(ContactOf, Manifold, Impulse, Response)>| {
                entities::merge(&from.lock().unwrap(), index, contacts, spawner)
            })
            .system(w, "merge")),
            one(entity_hook_all.system(w, "hook_all")),
            one(entity_one_way.system(w, "hook_one")),
            one(entities::solve.system(w, "solve")),
        ]
    } else {
        table::setup(w);
        vec![
            one(table::gravity_system.system(w, "gravity")),
            one(detect),
            one((move |_: &mut Cx, t: Query<&mut ContactTable>| table::merge(&from.lock().unwrap(), t)).system(w, "merge")),
            one(table_hook_all.system(w, "hook_all")),
            one(table_one_way.system(w, "hook_one")),
            one(table::solve.system(w, "solve")),
        ]
    }
}

fn contacts(w: &World, entity: bool) -> usize {
    if entity {
        w.values::<Manifold>().unwrap().len()
    } else {
        w.values::<ContactTable>().unwrap()[0].1.rows.len()
    }
}

fn began(w: &World, entity: bool) -> usize {
    if entity {
        w.values::<Manifold>().unwrap().iter().filter(|(_, m)| m.began).count()
    } else {
        w.values::<ContactTable>().unwrap()[0].1.rows.iter().filter(|r| r.began).count()
    }
}

fn contact_models() {
    println!("## Contacts: {BODIES}-body pile, µs a frame (60 frames)\n");
    println!("| model | scene | contacts | began | {} |", STAGES.join(" | "));
    println!("|---|---|---|---|{}", "---|".repeat(STAGES.len()));
    for (scene, skip) in [("falling", 0), ("settled", 400)] {
        for entity in [false, true] {
            let w = World::new();
            pile(&w, BODIES, scene == "falling");
            {
                // One player, so a hook for one entity has one to find.
                let mut m = w.between_frames(Build::default()).unwrap();
                let e = m.world().values::<Body>().unwrap()[0].0;
                m.insert(e, Player {});
                m.insert(e, Grounded::default());
            }
            let s = stages(&w, entity);
            for _ in 0..skip {
                s.iter().for_each(|s| drop(s.run_sequential(&w)));
            }
            let (frames, mut times, mut count, mut new) = (60, vec![0.0; STAGES.len()], 0, 0);
            for _ in 0..frames {
                for (i, s) in s.iter().enumerate() {
                    let t = Instant::now();
                    s.run_sequential(&w);
                    times[i] += t.elapsed().as_secs_f64() * 1e6;
                }
                count += contacts(&w, entity);
                new += began(&w, entity);
            }
            let row: Vec<String> = times.iter().map(|t| format!("{:.0}", t / frames as f64)).collect();
            let model = if entity { "entities" } else { "table" };
            println!("| {model} | {scene} | {} | {} | {} |", count / frames, new / frames, row.join(" | "));
        }
    }
}

/// The storage's cost when every contact is new: a settled pile's 1000
/// contacts, alternated each frame with as many other pairs, so each merge
/// ends every contact and begins as many. Merge only, µs a frame.
fn churn() {
    let w = World::new();
    pile(&w, BODIES, false);
    let found = {
        let s = stages(&w, false);
        for _ in 0..400 {
            s.iter().for_each(|s| drop(s.run_sequential(&w)));
        }
        let t = &w.values::<ContactTable>().unwrap()[0].1;
        t.rows.iter().map(|r| Found { a: r.a, b: r.b, normal: physics::Vec2::new(r.nx, r.ny), depth: r.depth }).collect::<Vec<_>>()
    };
    // Different pairs, still sorted: the same `a`, `b` shifted past every live entity.
    let other: Vec<Found> = found.iter().map(|f| Found { b: engine_ecs::Entity { index: f.b.index + 100_000, ..f.b }, ..*f }).collect();
    let lists = Arc::new([found, other]);
    let frame = Arc::new(Mutex::new(0usize));
    let mut row = Vec::new();
    for entity in [false, true] {
        let w = World::new();
        let (lists, frame) = (lists.clone(), frame.clone());
        let merge = if entity {
            entities::setup(&w);
            (move |_: &mut Cx, index: Query<&mut ContactIndex>, contacts: Query<(&mut Manifold, &mut Response), (), Despawns>, spawner: Spawner<(ContactOf, Manifold, Impulse, Response)>| {
                let mut f = frame.lock().unwrap();
                *f += 1;
                entities::merge(&lists[*f % 2], index, contacts, spawner)
            })
            .system(&w, "merge")
        } else {
            table::setup(&w);
            (move |_: &mut Cx, t: Query<&mut ContactTable>| {
                let mut f = frame.lock().unwrap();
                *f += 1;
                table::merge(&lists[*f % 2], t)
            })
            .system(&w, "merge")
        };
        row.push(format!("{:.0}", time(&w, &Schedule { systems: vec![merge] }, 200)));
    }
    println!("\nFull churn, {} contacts ending and as many beginning a frame, upkeep µs: table {}, entities {}.", lists[0].len(), row[0], row[1]);
}

/// Median µs of `runs` runs of `s`.
fn time(w: &World, s: &Schedule, runs: usize) -> f64 {
    let mut t: Vec<f64> = (0..runs)
        .map(|_| {
            let at = Instant::now();
            s.run_sequential(w);
            at.elapsed().as_secs_f64() * 1e6
        })
        .collect();
    t.sort_by(f64::total_cmp);
    t[runs / 2]
}

fn hierarchies() {
    let (parents, per) = (1000, 8);
    println!("\n## Hierarchy: {parents} parents × {per} children, µs to propagate (median of 200)\n");
    println!("| children stored | lookup per child | `Children` on parent | runs of siblings |");
    println!("|---|---|---|---|");
    for shuffled in [false, true] {
        let mut row = Vec::new();
        for way in 0..3 {
            let w = World::new();
            hierarchy(&w, parents, per, shuffled);
            let step = match way {
                0 => (|_: &mut Cx, mut c: Query<(&ChildOf, &Local, &mut Global)>, mut r: Query<&Global, Without<ChildOf>>| by_lookup(&mut c, &mut r)).system(&w, "p"),
                1 => (|_: &mut Cx, mut r: Query<(&Global, &Children), Without<ChildOf>>, mut c: Query<(&Local, &mut Global), With<ChildOf>>| by_children(&mut r, &mut c)).system(&w, "p"),
                _ => (|_: &mut Cx, mut c: Query<(&ChildOf, &Local, &mut Global)>, mut r: Query<&Global, Without<ChildOf>>| by_runs(&mut c, &mut r)).system(&w, "p"),
            };
            row.push(format!("{:.0}", time(&w, &Schedule { systems: vec![step] }, 200)));
        }
        println!("| {} | {} |", if shuffled { "shuffled" } else { "by parent" }, row.join(" | "));
    }
    // What the loop would cost with no link at all: children's own rows.
    let w = World::new();
    hierarchy(&w, parents, per, false);
    let plain = (|_: &mut Cx, mut c: Query<(&Local, &mut Global), With<ChildOf>>| {
        c.for_each(|_, (l, mut g)| *g = Global { x: l.x, y: l.y + 1.0 })
    })
    .system(&w, "plain");
    // The ideal lookup: parents' values in a vector by entity index.
    let dense = (|_: &mut Cx, mut c: Query<(&ChildOf, &Local, &mut Global)>, mut r: Query<&Global, Without<ChildOf>>| {
        let mut dense = vec![Global::default(); 10_000];
        r.for_each(|row, g| dense[row.entity().index as usize] = *g);
        c.for_each(|_, (of, l, mut g)| {
            let p = dense[of.parent.index as usize];
            *g = Global { x: p.x + l.x, y: p.y + l.y }
        })
    })
    .system(&w, "dense");
    println!("\nParents copied into a vector by entity index first: {:.0} µs.", time(&w, &Schedule { systems: vec![dense] }, 200));
    println!("\nNo parent read at all: {:.0} µs.", time(&w, &Schedule { systems: vec![plain] }, 200));
}

fn colliders() {
    let (n, per) = (1000, 2);
    let w = World::new();
    bodies(&w, n, per);
    let out = Arc::new(Mutex::new(Vec::new()));
    let (a, b) = (out.clone(), out.clone());
    let compound = (move |_: &mut Cx, mut q: Query<(&Global, &Parts)>| compound_boxes(&mut q, &mut a.lock().unwrap())).system(&w, "a");
    let entities = (move |_: &mut Cx, mut c: Query<&ColliderOf>, mut q: Query<&Global, With<Parts>>| collider_boxes(&mut c, &mut q, &mut b.lock().unwrap())).system(&w, "b");
    println!("\n## Colliders: {n} bodies × {per}, µs to place every box (median of 200)\n");
    println!("| on the body (`Vec`) | an entity each |");
    println!("|---|---|");
    println!(
        "| {:.0} | {:.0} |",
        time(&w, &Schedule { systems: vec![compound] }, 200),
        time(&w, &Schedule { systems: vec![entities] }, 200)
    );
}

fn fragmentation() {
    let n = 8000;
    println!("\n## Fragmentation: {n} rows over N tables, µs for one query over all (median of 200)\n");
    println!("| tables | µs |");
    println!("|---|---|");
    for tables in [1, 16, 64, 256] {
        let w = World::new();
        fragmented(&w, n, tables);
        let s = Schedule { systems: vec![(|_: &mut Cx, mut q: Query<(&mut Global, &Vel2)>| integrate(&mut q)).system(&w, "i")] };
        println!("| {tables} | {:.1} |", time(&w, &s, 200));
    }
    // Same rows, one per table: what a relation to 1000 distinct targets
    // (a table per parent) would make of 8 children each, lower bound.
    println!("\n(256 tables at {n} rows is ~31 rows a table; a table per parent at 8 children is 8.)");
}

fn main() {
    contact_models();
    churn();
    hierarchies();
    colliders();
    fragmentation();
}
