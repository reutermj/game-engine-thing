//! Walker behavior: pace, turn at a wall or a ledge, and meet the player.
//! Landing on a walker from above kills it and bounces the player; any other
//! touch kills the player. Both are `Bounce` and `Hurt` events to the
//! platformer's rules, which own what happens to the player.
//!
//! A walker is a physics body that collides with tiles only; it meets the
//! player by overlap, not collision, so a stomp and a touch can be told
//! apart before either pushes the other.

use engine_api::{Cx, Despawns, EventWriter, Mod, Query, Systems, With, Without, export_mod, phase};
use physics::{Position, Spatial, Touching, Vec2, Velocity, rect};
use platformer::{Bounce, Hurt, PLAYER_HEIGHT, PLAYER_WIDTH, Player, SOLID, Tile};
use walkers::{STOMP_BOUNCE, WALK_SPEED, Walker};

engine_api::mod_state! {
    #[derive(Default)]
    struct Walkers {}
}

/// The player, as a walker meets it.
#[derive(Clone, Copy)]
struct Seen {
    x: f32,
    y: f32,
    vy: f32,
}

/// Whether the player's box overlaps a walker's, both by their centers.
fn overlaps(p: &Seen, w: &Position) -> bool {
    (p.x - w.x).abs() < PLAYER_WIDTH / 2.0 + 0.5 && (p.y - w.y).abs() < PLAYER_HEIGHT / 2.0 + 0.5
}

/// What meeting the player came to this frame.
enum Meeting {
    None,
    Stomped,
    Hurt,
}

impl Walkers {
    #[allow(clippy::too_many_arguments)]
    fn walk(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut walkers: Query<(&Position, &mut Velocity, &Touching), With<Walker>, Despawns>,
        // Not a walker, whose velocity `walkers` writes.
        mut players: Query<(&Position, &Velocity), (With<Player>, Without<Walker>)>,
        mut tiles: Spatial<&Tile>,
        hurts: EventWriter<Hurt>,
        bounces: EventWriter<Bounce>,
    ) {
        let player = players.single(|_, (p, v)| Seen { x: p.x, y: p.y, vy: v.y });
        let mut meeting = Meeting::None;
        walkers.for_each(|walker, (w, v, touching)| {
            if v.x == 0.0 {
                // A new walker sets off right, without looking: on the first
                // frame the spatial index is empty, since no physics step has
                // published it yet, and it would see a ledge everywhere.
                v.x = WALK_SPEED;
                return;
            }
            let dir = if v.x < 0.0 { -1.0 } else { 1.0 };
            // Just past its leading edge, and just below its feet.
            let ahead = Vec2::new(w.x + dir * 0.55, w.y + 0.6);
            let mut ground = false;
            tiles.overlapping(rect(ahead, Vec2::ZERO), |_, t| ground |= t.kind == SOLID);
            let wall = if dir > 0.0 { touching.right } else { touching.left };
            v.x = if wall || !ground { -dir * WALK_SPEED } else { dir * WALK_SPEED };

            if let Some(p) = player.filter(|p| overlaps(p, w)) {
                // From above: falling, with the player's feet in the walker's
                // top half.
                if p.vy > 0.0 && p.y + PLAYER_HEIGHT / 2.0 < w.y {
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
