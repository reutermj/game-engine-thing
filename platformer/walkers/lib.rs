//! Walker behavior: pace, turn at a wall or a ledge, and meet the player.
//! Landing on a walker from above kills it and bounces the player; any other
//! touch kills the player. Both are `Bounce` and `Hurt` events to the
//! platformer's rules, which own what happens to the player.
//!
//! A walker is a physics body that collides with tiles only and senses the
//! player: physics reports their overlap (an `Overlap` entity) rather than
//! pushing them apart, so a stomp and a touch can be told apart before
//! either pushes the other.

use engine_api::{Cx, Despawns, EventWriter, Mod, Query, Systems, With, Without, export_mod, phase};
use physics::{Overlap, Position, Spatial, Touching, Vec2, Velocity, rect};
use platformer::{Bounce, Hurt, PLAYER_HEIGHT, Player, SOLID, Tile};
use walkers::{STOMP_BOUNCE, WALK_SPEED, Walker};

engine_api::mod_state! {
    #[derive(Default)]
    struct Walkers {}
}

/// What meeting the player came to this frame.
enum Meeting {
    None,
    Stomped,
    Hurt,
}

impl Walkers {
    fn walk(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut walkers: Query<(&Position, &mut Velocity, &Touching), With<Walker>>,
        mut tiles: Spatial<&Tile>,
    ) {
        walkers.for_each(|_, (w, mut v, touching)| {
            let dir = if v.x < 0.0 { -1.0 } else { 1.0 };
            // Just past its leading edge, and just below its feet.
            let ahead = Vec2::new(w.x + dir * 0.55, w.y + 0.6);
            let mut ground = false;
            tiles.overlapping(rect(ahead, Vec2::ZERO), |_, t| ground |= t.kind == SOLID);
            let wall = if dir > 0.0 { touching.right } else { touching.left };
            v.x = if wall || !ground { -dir * WALK_SPEED } else { dir * WALK_SPEED };
        });
    }

    /// The player overlapping a walker, as physics found it this step: run
    /// right after `find_contacts`, so it's as of where everything is now
    /// (a respawn included), not last step.
    fn meet(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut walkers: Query<&Position, With<Walker>, Despawns>,
        mut players: Query<(&Position, &Velocity), (With<Player>, Without<Walker>)>,
        mut overlaps: Query<&Overlap>,
        hurts: EventWriter<Hurt>,
        bounces: EventWriter<Bounce>,
    ) {
        let mut meeting = Meeting::None;
        overlaps.for_each(|_, o| {
            for (walker, other) in [(o.a, o.b), (o.b, o.a)] {
                let Some((p, vy)) = players.with(other, |_, (p, v)| (*p, v.y)) else { continue };
                walkers.with(walker, |row, w| {
                    // From above: falling, with the player's feet in the
                    // walker's top half.
                    if vy > 0.0 && p.y + PLAYER_HEIGHT / 2.0 < w.y {
                        row.despawn();
                        meeting = Meeting::Stomped;
                    } else if !matches!(meeting, Meeting::Stomped) {
                        meeting = Meeting::Hurt;
                    }
                });
            }
        });
        match meeting {
            Meeting::None => {}
            Meeting::Stomped => bounces.send(Bounce { speed: STOMP_BOUNCE }),
            Meeting::Hurt => hurts.send(Hurt {}),
        }
    }
}

impl Mod for Walkers {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("walk", Self::walk).phase(phase::SIMULATE).after("platformer::play");
        s.add("meet", Self::meet).phase("physics::step").after("physics::find_contacts").before("physics::solve");
    }
}

export_mod!(Walkers);
