//! Walker behavior: pace, turn at a wall or a ledge, and meet the player.
//! Landing on a walker from above kills it and bounces the player; any other
//! touch kills the player. Both go through the platformer's `Rules` service,
//! which owns what happens to the player.

use std::collections::HashSet;

use clock::Clock;
use engine_api::{Cx, Mod, Status, export_mod};
use platformer::{PLAYER_HEIGHT, PLAYER_WIDTH, Player, SOLID, Tile};
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

impl Mod for Walkers {
    type Transient = ();

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        let mut world = cx.world();
        let Some(Clock { dt, .. }) = clock::now(&mut world) else {
            return Status::OK;
        };
        let solid: HashSet<(i32, i32)> =
            world.query::<Tile>().filter(|(_, t)| t.kind == SOLID).map(|(_, t)| (t.x, t.y)).collect();
        let player = world.query::<Player>().next().map(|(_, p)| *p);

        let mut stomped = Vec::new();
        let mut meeting = Meeting::None;
        for (e, w) in world.query::<Walker>() {
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
                    stomped.push(e);
                    meeting = Meeting::Stomped;
                } else if !matches!(meeting, Meeting::Stomped) {
                    meeting = Meeting::Hurt;
                }
            }
        }
        for e in stomped {
            world.despawn(e);
        }

        // After the query: a call can change the world, so it can't happen
        // while one is open.
        let result = match meeting {
            Meeting::None => Ok(()),
            Meeting::Stomped => platformer::bounce(cx, STOMP_BOUNCE),
            Meeting::Hurt => platformer::hurt(cx),
        };
        if let Err(e) = result {
            cx.log(format!("couldn't reach the rules: {e}"));
        }
        Status::OK
    }
}

export_mod!(Walkers);
