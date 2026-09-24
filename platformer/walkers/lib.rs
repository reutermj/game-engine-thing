//! Walker behavior: pace, turn at a wall or a ledge, and meet the player.
//! Landing on a walker from above kills it and bounces the player; any other
//! touch kills the player. Both are `Bounce` and `Hurt` events to the
//! platformer's rules, which own what happens to the player.

use std::collections::HashSet;

use clock::Clock;
use engine_api::{Cx, Despawns, EventWriter, Mod, Query, Systems, export_mod, phase};
use platformer::{Bounce, Hurt, PLAYER_HEIGHT, PLAYER_WIDTH, Player, SOLID, Tile};
use walkers::{STOMP_BOUNCE, WALK_SPEED, Walker};

engine_api::mod_state! {
    #[derive(Default)]
    struct Walkers {}
}

fn overlaps(p: &Player, w: &Walker) -> bool {
    p.x < w.x + 1.0 && w.x < p.x + PLAYER_WIDTH && p.y < w.y + 1.0 && w.y < p.y + PLAYER_HEIGHT
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
        mut clocks: Query<&Clock>,
        mut tiles: Query<&Tile>,
        mut players: Query<&Player>,
        mut walkers: Query<&mut Walker, (), Despawns>,
        hurts: EventWriter<Hurt>,
        bounces: EventWriter<Bounce>,
    ) {
        let Some(dt) = clocks.single(|_, c| c.dt) else { return };
        let mut solid = HashSet::new();
        tiles.for_each(|_, t| {
            if t.kind == SOLID {
                solid.insert((t.x, t.y));
            }
        });
        let player = players.single(|_, p| *p);

        let mut meeting = Meeting::None;
        walkers.for_each(|walker, w| {
            if w.vx == 0.0 {
                w.vx = WALK_SPEED;
            }
            // Turn instead of walking into a wall or off a ledge.
            let ahead = w.x + w.vx * dt + if w.vx > 0.0 { 1.0 } else { 0.0 };
            let (col, row) = (ahead.floor() as i32, w.y.floor() as i32);
            if solid.contains(&(col, row)) || !solid.contains(&(col, row + 1)) {
                w.vx = -w.vx;
            } else {
                w.x += w.vx * dt;
            }

            if let Some(p) = player.filter(|p| overlaps(p, w)) {
                // From above: falling, with the player's feet in the walker's
                // top half.
                if p.vy > 0.0 && p.y + PLAYER_HEIGHT < w.y + 0.5 {
                    walker.despawn();
                    meeting = Meeting::Stomped;
                } else if !matches!(meeting, Meeting::Stomped) {
                    meeting = Meeting::Hurt;
                }
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
    }
}

export_mod!(Walkers);
