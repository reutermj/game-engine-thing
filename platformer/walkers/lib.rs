//! Walker behavior: pace, turn at a wall or a ledge, and meet the player.
//! Landing on a walker from above kills it and bounces the player; any other
//! touch sets `Player::hurt`, which the platformer's rules act on.

use std::collections::HashSet;

use clock::Clock;
use engine_api::{Cx, Mod, Status, export_mod};
use platformer::{PLAYER_HEIGHT, PLAYER_WIDTH, Player, SOLID, Tile};
use walkers::{STOMP_BOUNCE, WALK_SPEED, Walker};

#[derive(Default)]
struct Walkers;

fn overlaps(p: &Player, w: &Walker) -> bool {
    p.x < w.x + 1.0 && w.x < p.x + PLAYER_WIDTH && p.y < w.y + 1.0 && w.y < p.y + PLAYER_HEIGHT
}

impl Mod for Walkers {
    fn step(&mut self, cx: &mut Cx) -> Status {
        let mut world = cx.world();
        let Some(Clock { dt, .. }) = clock::now(&mut world) else {
            return Status::OK;
        };
        let solid: HashSet<(i32, i32)> =
            world.query::<Tile>().filter(|(_, t)| t.kind == SOLID).map(|(_, t)| (t.x, t.y)).collect();
        let mut player = world.query::<Player>().next().map(|(e, p)| (e, *p));

        let mut stomped = Vec::new();
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

            if let Some((_, p)) = player.as_mut().filter(|(_, p)| overlaps(p, w)) {
                // From above: falling, with the player's feet in the walker's
                // top half.
                if p.vy > 0.0 && p.y + PLAYER_HEIGHT < w.y + 0.5 {
                    stomped.push(e);
                    p.vy = -STOMP_BOUNCE;
                } else {
                    p.hurt = true;
                }
            }
        }
        for e in stomped {
            world.despawn(e);
        }
        if let Some((e, p)) = player {
            world.insert(e, p);
        }
        Status::OK
    }
}

export_mod!(Walkers);
