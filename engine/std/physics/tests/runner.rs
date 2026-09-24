//! A platformer in miniature, for the tests: a floor of one-unit tiles with
//! a pit, a player who runs and jumps on it, and a walker that turns at
//! ledges with a spatial query.
//!
//!   go | stop | jump   the player runs right, stops, jumps once when grounded
//!   trace              the player each frame so far: x y vx vy below
//!   walker             the walker: x y vx
//!   events             what the player met: coin triggers, contacts begun;
//!                      and every trigger sent at all
//!   contacts           the player's pressed contacts, as entities: the other
//!                      end and the normal away from the player, each
//!   overlaps           how many overlaps the player is in, as entities
//!   ghost              from now on, the player's contacts are disabled
//!                      between finding and solving them: it falls through

use engine_api::{
    Cx, Entity, EventReader, Mod, Query, Systems, With, Without, component, export_mod, field_struct, phase,
};
use physics::{
    Body, Collider, Contact, ContactPair, Gravity, Manifold, Overlap, Position, Response, Spatial, Touching, Trigger, Vec2,
    Velocity,
};

pub const FLOOR_Y: f32 = 10.0;
/// Columns with no tile.
pub const PIT: [i32; 2] = [15, 16];
pub const RUN: f32 = 7.0;
pub const JUMP: f32 = 12.0;
pub const WALK: f32 = 3.0;
/// A coin the running player passes through.
pub const COIN_X: f32 = 8.5;

const TILES: u32 = 1;
const PLAYER: u32 = 2;
const WALKER: u32 = 4;
const COIN: u32 = 8;

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Runner: "test::Runner" {}
}

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Walker: "test::Walker" {}
}

field_struct! {
    #[derive(Debug, Default, Copy)]
    struct Sample {
        x: f32,
        y: f32,
        vx: f32,
        vy: f32,
        below: bool,
    }
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Scene {
        built: bool,
        running: bool,
        jump: bool,
        trace: Vec<Sample>,
        walker: Option<Entity>,
        player: Option<Entity>,
        coins: u32,
        triggers: u32,
        /// Contacts begun between the player and a tile, and the fastest.
        landings: u32,
        hardest: f32,
        ghost: bool,
    }
}

impl Scene {
    fn run(&mut self, _: &mut (), _: &mut Cx, mut players: Query<(&Touching, &mut Velocity), With<Runner>>) {
        players.for_each(|_, (touching, mut v)| {
            v.x = if self.running { RUN } else { 0.0 };
            if self.jump && touching.below {
                v.y = -JUMP;
                self.jump = false;
            }
        });
    }

    fn walk(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut walkers: Query<(&Position, &mut Velocity), With<Walker>>,
        mut ground: Spatial<(), Without<Body>>,
    ) {
        walkers.for_each(|_, (p, mut v)| {
            let dir = if v.x < 0.0 { -1.0 } else { 1.0 };
            // Just past its leading edge, just below its feet.
            let ahead = Vec2::new(p.x + dir * 0.55, p.y + 0.6);
            v.x = if ground.any_at(ahead) { dir * WALK } else { -dir * WALK };
        });
    }

    fn record(&mut self, _: &mut (), _: &mut Cx, mut players: Query<(&Position, &Velocity, &Touching), With<Runner>>) {
        players.for_each(|_, (p, v, t)| {
            self.trace.push(Sample { x: p.x, y: p.y, vx: v.x, vy: v.y, below: t.below });
        });
    }

    fn meet(&mut self, _: &mut (), _: &mut Cx, mut triggers: EventReader<Trigger>, mut contacts: EventReader<Contact>) {
        let player = self.player;
        let triggers = triggers.read();
        self.triggers += triggers.len() as u32;
        self.coins += triggers.iter().filter(|t| Some(t.other) == player).count() as u32;
        for c in contacts.read().iter().filter(|c| Some(c.a) == player || Some(c.b) == player) {
            self.landings += 1;
            self.hardest = self.hardest.max(c.speed);
        }
    }

    /// Between finding contacts and solving them: a pre-solve hook.
    fn pass_through(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut contacts: Query<(&ContactPair, &mut Response)>,
        mut runners: Query<(), With<Runner>>,
    ) {
        if !self.ghost {
            return;
        }
        contacts.for_each(|_, (pair, mut r)| {
            if runners.with(pair.a, |_, _| ()).is_some() || runners.with(pair.b, |_, _| ()).is_some() {
                r.disabled = true;
            }
        });
    }
}

impl Mod for Scene {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("run", Self::run);
        s.add("walk", Self::walk);
        s.add("record", Self::record).phase(phase::LATE);
        s.add("meet", Self::meet).phase(phase::LATE);
        s.add("pass_through", Self::pass_through)
            .phase("physics::step")
            .after("physics::find_contacts")
            .before("physics::solve");
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        if self.built {
            return;
        }
        self.built = true;
        let mut world = cx.world();
        world.spawn((Gravity { x: 0.0, y: 40.0 },));
        for x in (0..30).filter(|x| !PIT.contains(x)) {
            let tile = Position { x: x as f32 + 0.5, y: FLOOR_Y + 0.5 };
            world.spawn((tile, Collider::rect(0.5, 0.5).on(TILES, u32::MAX)));
        }
        let player = Body { friction: 0.0, ..Body::default() };
        self.player = Some(world.spawn((
            Runner {},
            Position { x: 2.5, y: FLOOR_Y - 0.5 },
            Velocity::default(),
            player,
            Collider::rect(0.4, 0.475).on(PLAYER, TILES | COIN),
            Touching::default(),
        )));
        let coin = Position { x: COIN_X, y: FLOOR_Y - 0.5 };
        world.spawn((coin, Collider::rect(0.3, 0.3).on(COIN, PLAYER).sensor()));
        // A sensor sunk into the floor, far from the player's run, that
        // collides with tiles: static in static, which is no event.
        let buried = Position { x: 28.5, y: FLOOR_Y + 0.5 };
        world.spawn((buried, Collider::rect(0.3, 0.3).on(COIN, TILES).sensor()));
        let walker = Body { friction: 0.0, ..Body::default() };
        self.walker = Some(world.spawn((
            Walker {},
            Position { x: 20.5, y: FLOOR_Y - 0.5 },
            Velocity { x: -WALK, y: 0.0 },
            walker,
            Collider::rect(0.45, 0.45).on(WALKER, TILES),
        )));
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        match message.trim() {
            "go" => self.running = true,
            "stop" => self.running = false,
            "jump" => self.jump = true,
            "ghost" => self.ghost = true,
            "overlaps" => {
                let player = self.player.ok_or("no player")?;
                let mut n = 0;
                cx.world().for_each::<&Overlap>(|_, o| n += o.other(player).is_some() as u32);
                return Ok(n.to_string());
            }
            "contacts" => {
                let player = self.player.ok_or("no player")?;
                let mut out = Vec::new();
                cx.world().for_each::<(&ContactPair, &Manifold)>(|_, (pair, m)| {
                    if let Some((other, sign)) = pair.seen_from(player).filter(|_| m.pressed) {
                        out.push(format!("{} {:.2} {:.2}", other.index, m.nx * sign, m.ny * sign));
                    }
                });
                return Ok(out.join("\n"));
            }
            "trace" => {
                let lines: Vec<String> = self
                    .trace
                    .iter()
                    .map(|s| format!("{:.4} {:.4} {:.4} {:.4} {}", s.x, s.y, s.vx, s.vy, s.below))
                    .collect();
                return Ok(lines.join("\n"));
            }
            "walker" => {
                let e = self.walker.ok_or("no walker")?;
                let world = cx.world();
                let p = world.get::<Position>(e).ok_or("the walker is gone")?;
                let v = world.get::<Velocity>(e).ok_or("the walker is gone")?;
                return Ok(format!("{:.4} {:.4} {:.4}", p.x, p.y, v.x));
            }
            "events" => {
                let (c, l, h, t) = (self.coins, self.landings, self.hardest, self.triggers);
                return Ok(format!("coins {c} landings {l} hardest {h:.3} triggers {t}"));
            }
            other => return Err(format!("unknown command {other:?}")),
        }
        Ok("ok".into())
    }
}

export_mod!(Scene);
