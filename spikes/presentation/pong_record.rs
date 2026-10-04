//! SPIKE (get-3hd.1): `spike_pong_record`, pong's side of the session
//! recorder. At the end of every frame (in `late`, after pong's `rebound`
//! has scored and served) it fills the `spike_record::Watch` the windowed
//! lockstep bootstrap keeps on its clock entity: the `pong::Steer` events
//! the frame applied, the score, the ball and the paddles. The bootstrap
//! reads it after the frame and writes what changed to the session log.
//!
//! It writes only the `Watch`, so a game with it is the same game. The
//! `Watch` lives on the bootstrap's clock entity rather than one of its
//! own so that recording spawns nothing: entity indices, and anything that
//! could depend on them, stay what they are without it.

use clock::Clock;
use engine_api::{Cx, EventReader, Mod, Query, Systems, With, export_mod, phase};
use physics2d::{Position, Velocity};
use pong::{Ball, Paddle, Score, Steer, WIDTH};
use spike_record::Watch;

engine_api::mod_state! {
    #[derive(Default)]
    struct PongRecord {}
}

impl PongRecord {
    fn watch(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut steers: EventReader<Steer>,
        mut clocks: Query<&Clock>,
        mut balls: Query<(&Position, &Velocity), With<Ball>>,
        mut paddles: Query<(&Paddle, &Position)>,
        mut scores: Query<&Score>,
        mut watches: Query<&mut Watch>,
    ) {
        let read = steers.read();
        let (count, last) = (read.len() as u32, read.last().map_or(0.0, |s| s.intent));
        let frame = clocks.single(|_, c| c.frame).unwrap_or(0);
        let ball = balls.single(|_, (p, v)| [p.x, p.y, v.x, v.y]).unwrap_or_default();
        let mut ys = [0.0; 2];
        paddles.for_each(|_, (p, at)| ys[usize::from(p.face > WIDTH / 2.0)] = at.y);
        let score = scores.single(|_, s| [s.left, s.right]).unwrap_or_default();
        watches.for_each(|_, mut w| {
            *w = Watch { frame, steers: count, steer: last, score, ball, paddles: ys };
        });
    }
}

impl Mod for PongRecord {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("watch", Self::watch).phase(phase::LATE).after("pong::rebound");
    }
}

export_mod!(PongRecord);
