//! Option 3: each contact an entity, with `ContactOf { a, b }`, its
//! manifold, impulses and what the game made of it. A step updates the
//! contacts that persist in place, spawns the ones that began and
//! despawns the ones that ended. `ContactOf` is an ordered key, so contacts
//! are stored in pair order: the step's merge and the solver's gather are
//! one pass each, with no index and no sort. Hooks are ordinary queries over
//! contact entities.

use engine_ecs::harness::Cx;
use engine_ecs::{Despawns, Entity, OrderKey, Query, Spawner, Without, component, pair_key};
use physics::Vec2;

use crate::common::{Body, Found, Pos, Shape, Solvable, Vel, detect, gravity, solve_step};

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct ContactOf: "rel::ContactOf", order = key { pub a: Entity, pub b: Entity }
}

impl OrderKey for ContactOf {
    fn key(&self) -> u128 {
        pair_key(self.a, self.b)
    }
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

pub fn gravity_system(_: &mut Cx, mut bodies: Query<(&Body, &mut Vel)>) {
    gravity(&mut bodies);
}

const FRESH: Response = Response { restitution: 0.1, friction: 0.4, disabled: false };

#[allow(clippy::type_complexity)]
pub fn find(
    _: &mut Cx,
    mut bodies: Query<(&Pos, &Shape, &Vel, &Body)>,
    mut statics: Query<(&Pos, &Shape), Without<Body>>,
    contacts: Query<(&ContactOf, &mut Manifold, &mut Response), (), Despawns>,
    spawner: Spawner<(ContactOf, Manifold, Impulse, Response)>,
) {
    merge(&detect(&mut bodies, &mut statics), contacts, spawner);
}

/// The storage's half of `find`, one pass over both lists in pair order:
/// contacts that persist updated in place, ended ones despawned, new ones
/// spawned.
#[allow(clippy::type_complexity)]
pub fn merge(
    found: &[Found],
    mut contacts: Query<(&ContactOf, &mut Manifold, &mut Response), (), Despawns>,
    spawner: Spawner<(ContactOf, Manifold, Impulse, Response)>,
) {
    let spawn = |f: &Found| {
        let m = Manifold { nx: f.normal.x, ny: f.normal.y, depth: f.depth, began: true };
        spawner.spawn((ContactOf { a: f.a, b: f.b }, m, Impulse::default(), FRESH));
    };
    let mut next = 0;
    contacts.for_each_ordered(|row, (pair, mut m, mut r)| {
        while found.get(next).is_some_and(|f| (f.a, f.b) < (pair.a, pair.b)) {
            spawn(&found[next]);
            next += 1;
        }
        match found.get(next) {
            Some(f) if (f.a, f.b) == (pair.a, pair.b) => {
                *m = Manifold { nx: f.normal.x, ny: f.normal.y, depth: f.depth, began: false };
                *r = FRESH;
                next += 1;
            }
            _ => row.despawn(),
        }
    });
    found[next..].iter().for_each(spawn);
}

pub fn solve(
    _: &mut Cx,
    mut bodies: Query<(&Body, &mut Vel, &mut Pos)>,
    mut contacts: Query<(&ContactOf, &Manifold, &mut Impulse, &Response)>,
) {
    // In pair order already: storage keeps it.
    let mut solvable: Vec<Solvable> = Vec::new();
    contacts.for_each_ordered(|_, (pair, m, j, r)| {
        if !r.disabled {
            solvable.push(Solvable {
                a: pair.a,
                b: pair.b,
                normal: Vec2::new(m.nx, m.ny),
                depth: m.depth,
                restitution: r.restitution,
                friction: r.friction,
                jn: j.jn,
                jt: j.jt,
            });
        }
    });
    let mut impulses = solve_step(&mut bodies, &solvable).into_iter();
    contacts.for_each_ordered(|_, (_, _, mut j, r)| {
        if !r.disabled {
            (j.jn, j.jt) = impulses.next().expect("an impulse per contact solved");
        }
    });
}
