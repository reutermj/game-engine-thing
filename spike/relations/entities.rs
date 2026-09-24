//! Option 3: each contact an entity, with `ContactOf { a, b }`, its
//! manifold, impulses and what the game made of it. A step updates the
//! contacts that persist in place, spawns the ones that began and
//! despawns the ones that ended; an index finds a pair's entity. Hooks are
//! ordinary queries over contact entities.

use engine_ecs::harness::Cx;
use engine_ecs::{Despawns, Entity, Query, Spawner, Without, component, field_struct};
use physics::Vec2;

use crate::common::{Body, Found, Pos, Shape, Solvable, Vel, detect, gravity, solve_step};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ContactOf: "rel::ContactOf" { pub a: Entity, pub b: Entity }
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Manifold: "rel::Manifold" { pub nx: f32, pub ny: f32, pub depth: f32, pub began: bool }
}
component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Impulse: "rel::Impulse" { pub jn: f32, pub jt: f32 }
}
component! {
    /// What the game made of the contact this step. A field, not a
    /// `Disabled` marker: a marker would be a structural change per hook.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Response: "rel::Response" { pub restitution: f32, pub friction: f32, pub disabled: bool }
}

field_struct! {
    #[derive(Debug, Default, Copy, PartialEq)]
    pub struct Indexed { pub a: Entity, pub b: Entity, pub contact: Entity }
}

component! {
    /// Each live contact's entity, by pair, sorted: one entity has it.
    #[derive(Debug, Default, PartialEq)]
    pub struct ContactIndex: "rel::ContactIndex" { pub entries: Vec<Indexed> }
}

pub fn setup(w: &engine_ecs::World) {
    w.between_frames(Default::default()).unwrap().spawn((ContactIndex::default(),));
}

pub fn gravity_system(_: &mut Cx, mut bodies: Query<(&Body, &mut Vel)>) {
    gravity(&mut bodies);
}

const FRESH: Response = Response { restitution: 0.1, friction: 0.4, disabled: false };

#[allow(clippy::type_complexity)]
pub fn find(
    _: &mut Cx,
    mut bodies: Query<(&Pos, &Shape, &Vel, &Body)>,
    mut statics: Query<(&Pos, &Shape), Without<Body>>,
    index: Query<&mut ContactIndex>,
    contacts: Query<(&mut Manifold, &mut Response), (), Despawns>,
    spawner: Spawner<(ContactOf, Manifold, Impulse, Response)>,
) {
    merge(&detect(&mut bodies, &mut statics), index, contacts, spawner);
}

/// The storage's half of `find`: contacts that persist updated in place,
/// ended ones despawned, new ones spawned.
#[allow(clippy::type_complexity)]
pub fn merge(
    found: &[Found],
    mut index: Query<&mut ContactIndex>,
    mut contacts: Query<(&mut Manifold, &mut Response), (), Despawns>,
    spawner: Spawner<(ContactOf, Manifold, Impulse, Response)>,
) {
    index.single(|_, mut idx| {
        let old = std::mem::take(&mut idx.entries);
        let mut entries = Vec::with_capacity(found.len());
        let (mut i, mut j) = (0, 0);
        while i < old.len() || j < found.len() {
            let ok = old.get(i).map(|o| (o.a, o.b));
            let fk = found.get(j).map(|f| (f.a, f.b));
            match (ok, fk) {
                (Some(o), Some(f)) if o == f => {
                    let (e, f) = (old[i].contact, found[j]);
                    contacts.with(e, |_, (mut m, mut r)| {
                        *m = Manifold { nx: f.normal.x, ny: f.normal.y, depth: f.depth, began: false };
                        *r = FRESH;
                    });
                    entries.push(old[i]);
                    i += 1;
                    j += 1;
                }
                (Some(o), f) if f.is_none_or(|f| o < f) => {
                    if let Some(row) = contacts.get(old[i].contact) {
                        row.despawn();
                    }
                    i += 1;
                }
                _ => {
                    let f = found[j];
                    let m = Manifold { nx: f.normal.x, ny: f.normal.y, depth: f.depth, began: true };
                    let contact = spawner.spawn((ContactOf { a: f.a, b: f.b }, m, Impulse::default(), FRESH));
                    entries.push(Indexed { a: f.a, b: f.b, contact });
                    j += 1;
                }
            }
        }
        idx.entries = entries;
    });
}

pub fn solve(
    _: &mut Cx,
    mut bodies: Query<(&Body, &mut Vel, &mut Pos)>,
    mut contacts: Query<(&ContactOf, &Manifold, &mut Impulse, &Response)>,
) {
    let mut gathered: Vec<(Entity, Solvable)> = Vec::new();
    contacts.for_each(|row, (pair, m, j, r)| {
        if !r.disabled {
            let s = Solvable {
                a: pair.a,
                b: pair.b,
                normal: Vec2::new(m.nx, m.ny),
                depth: m.depth,
                restitution: r.restitution,
                friction: r.friction,
                jn: j.jn,
                jt: j.jt,
            };
            gathered.push((row.entity(), s));
        }
    });
    // Storage order depends on history; solving in pair order doesn't.
    gathered.sort_by_key(|(_, s)| (s.a, s.b));
    let solvable: Vec<Solvable> = gathered.iter().map(|(_, s)| *s).collect();
    let impulses = solve_step(&mut bodies, &solvable);
    for ((e, _), (jn, jt)) in gathered.iter().zip(impulses) {
        contacts.with(*e, |_, (_, _, mut j, _)| (j.jn, j.jt) = (jn, jt));
    }
}

