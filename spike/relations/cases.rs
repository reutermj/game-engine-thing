//! The cases a game writes against each contact model: a one-way platform,
//! "what am I standing on", and a bounce pad. Each is a system between
//! finding contacts and solving them, and each is tested by the behavior it
//! should cause. Read side by side, they are the ergonomics data.

use engine_ecs::harness::Cx;
use engine_ecs::{Query, With, Without};
use physics::Vec2;

use crate::common::{Bouncy, Grounded, OneWay, Player, Vel, has};
use crate::entities::{ContactOf, Manifold, Response};
use crate::table::Contacts;

// ---- The table (option 4) ----

/// Solid from above only: a contact with a one-way platform is dropped
/// unless the player is landing on it.
pub fn table_one_way(_: &mut Cx, mut contacts: Contacts<&Vel, With<Player>>, mut platforms: Query<(), With<OneWay>>) {
    contacts.for_each(|c, _, v| {
        if has(&mut platforms, c.other()) && (c.normal().y < 0.7 || v.y < 0.0) {
            c.disable();
        }
    });
}

pub fn table_standing_on(_: &mut Cx, mut contacts: Contacts<&mut Grounded, With<Player>>) {
    contacts.for_each(|c, _, mut g| {
        if c.normal().y > 0.7 {
            *g = Grounded { on: c.other(), grounded: true };
        }
    });
}

pub fn table_bounce(_: &mut Cx, mut contacts: Contacts<(), With<Bouncy>>) {
    contacts.for_each(|c, _, ()| c.set_restitution(1.0));
}

// ---- Entities (option 3) ----

pub fn entity_one_way(
    _: &mut Cx,
    mut contacts: Query<(&ContactOf, &Manifold, &mut Response)>,
    mut platforms: Query<(), With<OneWay>>,
    mut players: Query<&Vel, With<Player>>,
) {
    contacts.for_each(|_, (pair, m, mut r)| {
        let n = Vec2::new(m.nx, m.ny);
        // Which end is the platform decides which way the normal points.
        let (player, normal) = if has(&mut platforms, pair.b) {
            (pair.a, n)
        } else if has(&mut platforms, pair.a) {
            (pair.b, -n)
        } else {
            return;
        };
        let Some(vy) = players.with(player, |_, v| v.y) else { return };
        if normal.y < 0.7 || vy < 0.0 {
            r.disabled = true;
        }
    });
}

pub fn entity_standing_on(
    _: &mut Cx,
    mut contacts: Query<(&ContactOf, &Manifold)>,
    mut players: Query<&mut Grounded, (With<Player>, Without<ContactOf>)>,
) {
    contacts.for_each(|_, (pair, m)| {
        for (me, other, down) in [(pair.a, pair.b, m.ny), (pair.b, pair.a, -m.ny)] {
            if down > 0.7 {
                players.with(me, |_, mut g| *g = Grounded { on: other, grounded: true });
            }
        }
    });
}

pub fn entity_bounce(_: &mut Cx, mut contacts: Query<(&ContactOf, &mut Response)>, mut pads: Query<(), With<Bouncy>>) {
    contacts.for_each(|_, (pair, mut r)| {
        if has(&mut pads, pair.a) || has(&mut pads, pair.b) {
            r.restitution = 1.0;
        }
    });
}

#[cfg(test)]
mod tests {
    use engine_ecs::harness::{IntoSystem, Schedule, SystemDecl};
    use engine_ecs::{Build, Bundle, Entity, World};

    use super::*;
    use crate::common::{Body, Pos, Shape};
    use crate::{entities, table};

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Model {
        Table,
        Entities,
    }

    /// The step for `model`, with `hook` between finding and solving.
    fn schedule(w: &World, model: Model, hook: Option<SystemDecl>) -> Schedule {
        let mut systems = Vec::new();
        match model {
            Model::Table => {
                table::setup(w);
                systems.push(table::gravity_system.system(w, "gravity"));
                systems.push(table::find.system(w, "find"));
            }
            Model::Entities => {
                entities::setup(w);
                systems.push(entities::gravity_system.system(w, "gravity"));
                systems.push(entities::find.system(w, "find"));
            }
        }
        systems.extend(hook);
        systems.push(match model {
            Model::Table => table::solve.system(w, "solve"),
            Model::Entities => entities::solve.system(w, "solve"),
        });
        Schedule { systems }
    }

    fn player(w: &World, x: f32, y: f32, vy: f32) -> Entity {
        let mut m = w.between_frames(Build::default()).unwrap();
        m.spawn((
            Pos { x, y },
            Vel { x: 0.0, y: vy },
            Body { inv_mass: 1.0, kinematic: false },
            Shape { hx: 0.45, hy: 0.45, circle: false },
            Player {},
            Grounded::default(),
        ))
    }

    /// Spawns the player and the thing it meets in either order, so the
    /// player is either side of their contact: `(player, other)`.
    fn meet(w: &World, player_first: bool, player: impl FnOnce(&World) -> Entity, other: impl Bundle) -> (Entity, Entity) {
        let spawn_other = |w: &World| w.between_frames(Build::default()).unwrap().spawn(other);
        if player_first {
            let p = player(w);
            (p, spawn_other(w))
        } else {
            let o = spawn_other(w);
            (player(w), o)
        }
    }

    const ORDERS: [(Model, bool); 4] = [(Model::Table, true), (Model::Table, false), (Model::Entities, true), (Model::Entities, false)];

    fn y(w: &World, e: Entity) -> f32 {
        w.values::<Pos>().unwrap().into_iter().find(|(x, _)| *x == e).unwrap().1.y
    }

    /// A player jumping up from under a one-way platform (y grows down).
    fn one_way(model: Model, first: bool, hook: bool) -> f32 {
        let w = World::new();
        let platform = (Pos { x: 0.0, y: 5.0 }, Shape { hx: 3.0, hy: 0.25, circle: false }, OneWay {});
        let (p, _) = meet(&w, first, |w| player(w, 0.0, 8.0, -16.0), platform);
        let hook = hook.then(|| match model {
            Model::Table => table_one_way.system(&w, "one_way"),
            Model::Entities => entity_one_way.system(&w, "one_way"),
        });
        let s = schedule(&w, model, hook);
        for _ in 0..120 {
            s.run_sequential(&w);
        }
        y(&w, p)
    }

    #[test]
    fn a_one_way_platform_is_jumped_through_and_landed_on() {
        for (model, first) in ORDERS {
            // On top: the platform's top is 4.75, the player's half 0.45.
            let on_top = one_way(model, first, true);
            assert!((on_top - 4.3).abs() < 0.05, "{model:?}, player first {first}: at {on_top}");
            // Without the hook it's solid: the jump stops under it.
            assert!(one_way(model, first, false) > 5.0, "{model:?}");
        }
    }

    #[test]
    fn a_player_knows_what_it_stands_on() {
        for (model, first) in ORDERS {
            let w = World::new();
            let floor = (Pos { x: 0.0, y: 5.5 }, Shape { hx: 3.0, hy: 0.5, circle: false });
            let (_, floor) = meet(&w, first, |w| player(w, 0.0, 4.5, 0.0), floor);
            let hook = match model {
                Model::Table => table_standing_on.system(&w, "standing_on"),
                Model::Entities => entity_standing_on.system(&w, "standing_on"),
            };
            let s = schedule(&w, model, Some(hook));
            for _ in 0..10 {
                s.run_sequential(&w);
            }
            let g = w.values::<Grounded>().unwrap()[0].1;
            assert_eq!((g.grounded, g.on), (true, floor), "{model:?}, player first: {first}");
        }
    }

    /// Starts touching: bouncing off a speculative contact is broken in the
    /// solver itself (get-emj.19), and this case is about the hook.
    #[test]
    fn a_bounce_pad_throws_back_what_lands_on_it() {
        for (model, first) in ORDERS {
            let run = |hook: bool| {
                let w = World::new();
                let pad = (Pos { x: 0.0, y: 6.0 }, Shape { hx: 3.0, hy: 0.5, circle: false }, Bouncy {});
                let (ball, _) = meet(&w, first, |w| player(w, 0.0, 5.06, 10.0), pad);
                let hook = hook.then(|| match model {
                    Model::Table => table_bounce.system(&w, "bounce"),
                    Model::Entities => entity_bounce.system(&w, "bounce"),
                });
                schedule(&w, model, hook).run_sequential(&w);
                w.values::<Vel>().unwrap().into_iter().find(|(e, _)| *e == ball).unwrap().1.y
            };
            let (bounced, landed) = (run(true), run(false));
            assert!(bounced < -10.0, "{model:?}: thrown back at its speed, got {bounced}");
            assert!(landed > -1.5, "{model:?}: without the pad it barely bounces, got {landed}");
        }
    }
}

