//! The platformer's rules, as systems over physics: `steer` turns `Run` and
//! `Jump` events into the player's `Input` (in `input`); `play` turns it
//! into velocity, and spawns the player once there's a level (in
//! `simulate`, before physics moves anything); `take_hits` applies what the
//! step and other mods reported: coins, spikes and the goal as sensors'
//! triggers, `Hurt` and `Bounce` from the walkers (in `late`).
//!
//! The player is spawned once a level exists (a `LevelInfo`) and there's no
//! player yet, so this mod doesn't care whether it loads before or after the
//! level, or reloads.

use engine_api::{Cx, Despawns, EventReader, Mod, Query, Spawner, Systems, With, export_mod, phase};
use physics::{Body, Collider, Gravity, Position, Touching, Trigger, Velocity};
use platformer::{
    Bounce, Coin, GOAL, GRAVITY, Hurt, Input, JUMP_SPEED, Jump, LevelInfo, MAX_FALL, PLAYER, PLAYER_HEIGHT,
    PLAYER_WIDTH, Player, RUN_SPEED, Run, SENSORS, SPIKE, TILES, Tile,
};

engine_api::mod_state! {
    #[derive(Default)]
    struct Core {}
}

/// Where the player's body starts: the level's spawn is its box's corner.
fn start(info: &LevelInfo) -> Position {
    Position { x: info.spawn_x + PLAYER_WIDTH / 2.0, y: info.spawn_y + PLAYER_HEIGHT / 2.0 }
}

fn respawn(player: &mut Player, p: &mut Position, v: &mut Velocity, info: &LevelInfo) {
    (*p, *v) = (start(info), Velocity::default());
    player.deaths += 1;
}

impl Core {
    fn steer(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut runs: EventReader<Run>,
        mut jumps: EventReader<Jump>,
        mut inputs: Query<&mut Input, With<Player>>,
    ) {
        let dir = runs.read().last().map(|r| r.dir);
        let jump = !jumps.read().is_empty();
        inputs.for_each(|_, input| {
            if let Some(dir) = dir {
                input.dir = dir;
            }
            input.jump |= jump;
        });
    }

    fn play(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut levels: Query<&LevelInfo>,
        mut players: Query<(&mut Input, &Touching, &mut Velocity), With<Player>>,
        spawner: Spawner<(Player, Input, Position, Velocity, Body, Collider, Touching)>,
    ) {
        let Some(info) = levels.single(|_, i| *i) else { return };
        let played = players.single(|_, (input, touching, v)| {
            v.x = input.dir.clamp(-1.0, 1.0) * RUN_SPEED;
            // A jump request is used up by the next frame, whether or not
            // the player was standing on something to jump from.
            if std::mem::take(&mut input.jump) && touching.below {
                v.y = -JUMP_SPEED;
            }
            v.y = v.y.min(MAX_FALL);
        });
        if played.is_none() {
            // Seen by the systems after this one: physics moves it this frame.
            let body = Body { friction: 0.0, ..Body::default() };
            let collider = Collider::rect(PLAYER_WIDTH / 2.0, PLAYER_HEIGHT / 2.0).on(PLAYER, TILES | SENSORS);
            spawner.spawn((
                Player::default(),
                Input::default(),
                start(&info),
                Velocity::default(),
                body,
                collider,
                Touching::default(),
            ));
        }
    }

    /// What happened to the player this frame. In `late`, after physics, so
    /// it sees every trigger and every `simulate` system's events the same
    /// frame.
    #[allow(clippy::too_many_arguments)]
    fn take_hits(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut triggers: EventReader<Trigger>,
        mut hurts: EventReader<Hurt>,
        mut bounces: EventReader<Bounce>,
        mut levels: Query<&LevelInfo>,
        mut players: Query<(&mut Player, &mut Position, &mut Velocity)>,
        mut tiles: Query<&Tile>,
        mut coins: Query<&Coin, (), Despawns>,
    ) {
        let Some(info) = levels.single(|_, i| *i) else { return };
        let mut hurt = !hurts.read().is_empty();
        let bounce = bounces.read().last().map(|b| b.speed);
        let (mut won, mut collected) = (false, 0);
        for t in triggers.read() {
            match tiles.with(t.sensor, |_, tile| tile.kind) {
                Some(SPIKE) => hurt = true,
                Some(GOAL) => won = true,
                _ => {
                    if let Some(coin) = coins.get(t.sensor) {
                        coin.despawn();
                        collected += 1;
                    }
                }
            }
        }
        players.for_each(|_, (player, p, v)| {
            player.coins += collected;
            player.won |= won;
            if let Some(speed) = bounce {
                v.y = -speed;
            }
            if hurt || p.y > info.height as f32 + 2.0 {
                respawn(player, p, v, &info);
            }
        });
    }
}

impl Mod for Core {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("steer", Self::steer).phase(phase::INPUT);
        s.add("play", Self::play).phase(phase::SIMULATE);
        s.add("take_hits", Self::take_hits).phase(phase::LATE);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        if world.single::<&Gravity, ()>(|_, _| ()).is_none() {
            world.spawn((Gravity { x: 0.0, y: GRAVITY },));
        }
    }
}

export_mod!(Core);
