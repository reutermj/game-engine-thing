//! The walkthrough's scenario (docs/architecture/storage.md, "Walkthrough:
//! walkers that catch fire"), for the tests and the benchmark, in plan order:
//!
//!   ignite  (reads Position, Health, Lava; adds Burning)
//!   burn    (writes Health, Burning; removes Burning)
//!   reap    (reads Health; despawns the dead)
//!   spawn   (spawns walkers)
//!   physics (writes Position, reads Velocity)
//!   ui      (writes Label, on entities of its own)
//!
//! `Burning` comes in two storages, to compare: `SparseBurning` and
//! `TableBurning`. With `anywhere`, ignite adds `Burning` through a query
//! matching every entity rather than through the walkers it iterates.

use std::sync::atomic::{AtomicU32, Ordering};

use engine_ecs::harness::{Cx, IntoSystem, Schedule, SystemDecl};
use engine_ecs::{Adds, Build, Component, Despawns, Entity, Query, Removes, Spawner, Without, World, component};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Position: "game::Position" { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Velocity: "game::Velocity" { pub x: f32, pub y: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Health: "game::Health" { pub hp: f32 }
}

component! {
    /// A lava pool: everything with x in `[from, to)` catches fire.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Lava: "game::Lava" { pub from: f32, pub to: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq)]
    pub struct Label: "game::Label" { pub text: String }
}

pub trait Burning: Component + Clone + std::fmt::Debug {
    fn new() -> Self;
    /// Burns for one frame; the damage done, and whether it's out.
    fn tick(&mut self) -> (f32, bool);
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct SparseBurning: "game::SparseBurning", storage = sparse { pub left: u32 }
}
impl Burning for SparseBurning {
    fn new() -> Self {
        SparseBurning { left: 3 }
    }
    fn tick(&mut self) -> (f32, bool) {
        self.left -= 1;
        (5.0, self.left == 0)
    }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct TableBurning: "game::TableBurning" { pub left: u32 }
}
impl Burning for TableBurning {
    fn new() -> Self {
        TableBurning { left: 3 }
    }
    fn tick(&mut self) -> (f32, bool) {
        self.left -= 1;
        (5.0, self.left == 0)
    }
}

pub fn world<B: Burning>() -> World {
    let w = World::new();
    let m = w.between_frames(Build::default()).unwrap();
    // Installs the components up front, so every test's tables are laid
    // out the same.
    m.id::<Position>();
    m.id::<Velocity>();
    m.id::<Health>();
    m.id::<Lava>();
    m.id::<Label>();
    m.id::<B>();
    w
}

/// Walkers spread over `[0, 100)`, a pool over `[40, 60)`, and labels.
pub fn populate(w: &World, walkers: usize, labels: usize) {
    let mut m = w.between_frames(Build::default()).unwrap();
    m.spawn((Lava { from: 40.0, to: 60.0 },));
    for i in 0..walkers {
        let x = (i * 37 % 100) as f32;
        let vx = if i % 2 == 0 { 1.5 } else { -1.5 };
        m.spawn((Position { x, y: 0.0 }, Velocity { x: vx, y: 0.0 }, Health { hp: 20.0 }));
    }
    for i in 0..labels {
        m.spawn((Label { text: format!("label {i}") },));
    }
}

static WORK: AtomicU32 = AtomicU32::new(0);

/// Busy work per entity walked, so a system's cost scales with what it walks.
pub fn set_work(n: u32) {
    WORK.store(n, Ordering::Relaxed);
}

fn work() {
    let mut x = 1.0f32;
    for i in 0..WORK.load(Ordering::Relaxed) {
        x = (x * 1.0001 + i as f32).sqrt();
    }
    std::hint::black_box(x);
}

fn pools(lava: &mut Query<&Lava>) -> Vec<Lava> {
    let mut pools = Vec::new();
    lava.for_each(|_, l| pools.push(*l));
    pools
}

fn ignite<B: Burning>(
    _: &mut Cx,
    mut lava: Query<&Lava>,
    mut walkers: Query<(&Position, &Health), Without<B>, Adds<B>>,
) {
    let pools = pools(&mut lava);
    walkers.for_each(|row, (p, _)| {
        work();
        if pools.iter().any(|l| (l.from..l.to).contains(&p.x)) {
            row.insert(B::new());
        }
    });
}

/// The same, adding `Burning` through a query that matches everything: what
/// a system inserting onto entities it didn't iterate would write.
fn ignite_anywhere<B: Burning>(
    _: &mut Cx,
    mut lava: Query<&Lava>,
    mut walkers: Query<(&Position, &Health), Without<B>>,
    mut anything: Query<(), (), Adds<B>>,
) {
    let pools = pools(&mut lava);
    let mut lit: Vec<Entity> = Vec::new();
    walkers.for_each(|row, (p, _)| {
        work();
        if pools.iter().any(|l| (l.from..l.to).contains(&p.x)) {
            lit.push(row.entity());
        }
    });
    for e in lit {
        if let Some(row) = anything.get(e) {
            row.insert(B::new());
        }
    }
}

fn burn<B: Burning>(_: &mut Cx, mut fires: Query<(&mut Health, &mut B), (), Removes<B>>) {
    fires.for_each(|row, (mut h, mut fire)| {
        work();
        let (damage, done) = fire.tick();
        h.hp -= damage;
        if done {
            row.remove::<B>();
        }
    });
}

fn reap(cx: &mut Cx, mut living: Query<&Health, (), Despawns>) {
    living.for_each(|row, h| {
        if h.hp <= 0.0 {
            row.despawn();
            cx.log("a walker burned up");
        }
    });
}

fn spawn(_: &mut Cx, new: Spawner<(Position, Velocity, Health)>) {
    for dir in [1.0, -1.0] {
        let x = if dir > 0.0 { 0.0 } else { 99.0 };
        new.spawn((Position { x, y: 0.0 }, Velocity { x: dir * 1.5, y: 0.0 }, Health { hp: 20.0 }));
    }
}

fn physics(_: &mut Cx, mut moving: Query<(&mut Position, &Velocity)>) {
    moving.for_each(|_, (mut p, v)| {
        work();
        p.x = (p.x + v.x).rem_euclid(100.0);
    });
}

fn ui(_: &mut Cx, mut labels: Query<&mut Label>) {
    labels.for_each(|_, mut l| {
        work();
        if l.text.len() > 64 {
            l.text.clear();
        }
        l.text.push('.');
    });
}

pub struct Options {
    /// Add `Burning` through a query matching everything.
    pub anywhere: bool,
}

pub fn schedule<B: Burning>(w: &World, o: Options) -> Schedule {
    let ignite: SystemDecl = if o.anywhere {
        ignite_anywhere::<B>.system(w, "ignite")
    } else {
        ignite::<B>.system(w, "ignite")
    };
    Schedule {
        systems: vec![
            ignite,
            burn::<B>.system(w, "burn"),
            reap.system(w, "reap"),
            spawn.system(w, "spawn"),
            physics.system(w, "physics"),
            ui.system(w, "ui"),
        ],
    }
}

/// Everything in the world, without entity ids (which differ between runs
/// whose spawns interleave differently), sorted: two worlds that ran the
/// same frames compare equal.
pub fn snapshot<B: Burning>(w: &World) -> Vec<String> {
    use std::collections::BTreeMap;
    let mut by: BTreeMap<Entity, String> = BTreeMap::new();
    fn add<T: Component + Clone + std::fmt::Debug>(w: &World, by: &mut BTreeMap<Entity, String>) {
        for (e, v) in w.values::<T>().unwrap_or_default() {
            *by.entry(e).or_default() += &format!("{v:?} ");
        }
    }
    add::<Position>(w, &mut by);
    add::<Velocity>(w, &mut by);
    add::<Health>(w, &mut by);
    add::<Lava>(w, &mut by);
    add::<Label>(w, &mut by);
    add::<B>(w, &mut by);
    let mut out: Vec<String> = by.into_values().collect();
    out.sort();
    out
}
