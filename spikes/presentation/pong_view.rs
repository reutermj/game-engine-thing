//! SPIKE (get-3hd.1): `spike_pong_view`, pong drawn in the spike's
//! vocabulary. A stage on the frame's `DrawList` (`Pass`, after
//! `spike_draw`'s extract, which finds no `Place`s in a pong world and so
//! makes an empty list): it reads pong's own components (the ball, the
//! paddles, the walls and goals as physics colliders, the `Score`) and the
//! `Clock`, and adds rects for them on a fixed 1280x720 canvas, which the
//! presenter letterboxes into the window. Pong's code doesn't change and
//! this writes nothing to the world, so a game with it is the same game.
//!
//! Colours: you (the left paddle, `pong_text`'s) cyan, the AI orange; the
//! goal strips behind the paddles in the colour of the side that loses a
//! point there. The score is seven-segment digits, yours left of centre;
//! the frame number is small, top left.
//!
//! With turns (`spike_turns::Turn` in the world), the score band also shows
//! the barrier (`band`): per side a lamp, lit once it has submitted, and
//! while it hasn't, the whole seconds it has been waited on; in the centre
//! a green play triangle while a turn's frames run, dim pause bars while
//! the turn is open; the turn number top right; at game over, the winner's
//! half framed in its colour. The band changes while no frames run, so it
//! is also drawn between frames, through `spike_draw::Restage`, whenever
//! the bootstrap asks for a redraw. The wait timer's start is an `Instant`
//! in this build's transient part, noted when the band first sees the
//! barrier's status change: wall-clock time reaches the pixels only, never
//! the world. (A reload of this mod restarts the timer.)

use std::time::Instant;

use clock::Clock;
use engine_api::{Cx, Mod, Pass, Query, Systems, With, Without, export_mod, phase};
use physics2d::{CIRCLE, Collider, Position};
use pong::{Ball, Goal, HEIGHT, Paddle, Score, WIDTH};
use spike_draw::{DrawList, Drawn, Item};
use spike_turns::{Outcome, Turn, TurnInfo};

engine_api::mod_state! {
    #[derive(Default)]
    struct PongView {}
}

/// The barrier as the band last drew it, and since when.
#[derive(Default)]
pub struct Band {
    /// Turn, who has submitted, playing, over.
    shown: Option<(u64, [bool; 2], bool, bool)>,
    since: Option<Instant>,
}

/// Pixels a court cell: the court (40 by 20 cells) is 1120 by 560.
const CELL: f32 = 28.0;
/// The canvas: the court with 32 px each side, a 96 px band above for the
/// score, 24 px below. pong_window opens its window at exactly this size
/// (`SPIKE_WINDOW` in BUILD.bazel), so a cell is 28 screen pixels.
const CANVAS: (f32, f32) = (WIDTH * CELL + 64.0, HEIGHT * CELL + 120.0);
const ORIGIN: (f32, f32) = (32.0, 96.0);
/// How far outside the court walls and goals are drawn, in cells: they
/// reach 10 cells out, which would cover the score.
const MARGIN: f32 = 0.75;
/// The band's height: down to where the top wall's margin starts, so a
/// restaged band (which starts with its own background) never covers it.
const BAND_H: f32 = ORIGIN.1 - MARGIN * CELL;

const fn rgb(r: u32, g: u32, b: u32) -> u32 {
    r | g << 8 | b << 16 | 0xff << 24
}
// The surface is sRGB, so these are linear values and the dark ones come
// out lighter than their numbers suggest.
const BACKGROUND: u32 = rgb(3, 3, 6);
const WALL: u32 = rgb(140, 140, 150);
const MIDLINE: u32 = rgb(14, 15, 22);
const BALL: u32 = rgb(250, 250, 250);
const YOU: u32 = rgb(60, 210, 240);
const AI: u32 = rgb(250, 150, 40);
const FRAME: u32 = rgb(110, 110, 125);
/// A side's lamp before it submits.
const YOU_DIM: u32 = rgb(6, 30, 38);
const AI_DIM: u32 = rgb(40, 18, 4);
const PLAYING: u32 = rgb(70, 235, 100);
const PAUSED: u32 = rgb(30, 30, 40);

fn rect(out: &mut Vec<Item>, x0: f32, y0: f32, x1: f32, y1: f32, colour: u32, shape: u32) {
    out.push(Item { pos: [(x0 + x1) / 2.0, (y0 + y1) / 2.0], size: [x1 - x0, y1 - y0], colour, shape, material: 0, layer: 0.5 });
}

/// A box in court cells (centre and half extents), clipped to the court
/// and its margin, onto the canvas.
fn court_box(out: &mut Vec<Item>, at: &Position, c: &Collider, colour: u32) {
    let clip = |v: f32, hi: f32| v.clamp(-MARGIN, hi + MARGIN);
    let (x0, x1) = (clip(at.x - c.hx, WIDTH), clip(at.x + c.hx, WIDTH));
    let (y0, y1) = (clip(at.y - c.hy, HEIGHT), clip(at.y + c.hy, HEIGHT));
    if x1 <= x0 || y1 <= y0 {
        return;
    }
    let shape = if c.shape == CIRCLE { 1 } else { 0 };
    let px = |x: f32| ORIGIN.0 + x * CELL;
    let py = |y: f32| ORIGIN.1 + y * CELL;
    rect(out, px(x0), py(y0), px(x1), py(y1), colour, shape);
}

/// `n` in seven-segment digits, each `w` by `h` with strokes `t` thick,
/// starting at `x` (or ending there, `right_aligned`).
fn number(out: &mut Vec<Item>, n: u64, x: f32, y: f32, w: f32, h: f32, t: f32, colour: u32, right_aligned: bool) {
    // Segments a to g, as bits: the usual seven-segment table.
    const DIGITS: [u8; 10] = [0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f];
    let text = n.to_string();
    let gap = w * 0.35;
    let total = text.len() as f32 * (w + gap) - gap;
    let mut left = if right_aligned { x - total } else { x };
    let mid = y + (h - t) / 2.0;
    for d in text.bytes() {
        let bits = DIGITS[(d - b'0') as usize];
        let segments = [
            (left, y, left + w, y + t),           // a, top
            (left + w - t, y, left + w, mid + t), // b, top right
            (left + w - t, mid, left + w, y + h), // c, bottom right
            (left, y + h - t, left + w, y + h),   // d, bottom
            (left, mid, left + t, y + h),         // e, bottom left
            (left, y, left + t, mid + t),         // f, top left
            (left, mid, left + w, mid + t),       // g, middle
        ];
        for (i, &(x0, y0, x1, y1)) in segments.iter().enumerate() {
            if bits & (1 << i) != 0 {
                rect(out, x0, y0, x1, y1, colour, 0);
            }
        }
        left += w + gap;
    }
}

/// The score band: the score, the frame number and, with turns, the
/// barrier. Starts with the band's own background, so drawn again over a
/// stale band it replaces it.
fn band(out: &mut Vec<Item>, b: &mut Band, score: Score, frame: Option<u64>, turn: Option<TurnInfo>, outcome: Option<Outcome>) {
    let mid = ORIGIN.0 + WIDTH / 2.0 * CELL;
    rect(out, 0.0, 0.0, CANVAS.0, BAND_H, BACKGROUND, 0);
    if let Some(o) = outcome {
        // The winner's half of the band, framed.
        let (x0, x1) = if o.winner == 0 { (6.0, mid - 6.0) } else { (mid + 6.0, CANVAS.0 - 6.0) };
        let (y0, y1, t) = (4.0, BAND_H - 4.0, 4.0);
        let colour = if o.winner == 0 { YOU } else { AI };
        for (a, b, c, d) in [(x0, y0, x1, y0 + t), (x0, y1 - t, x1, y1), (x0, y0, x0 + t, y1), (x1 - t, y0, x1, y1)] {
            rect(out, a, b, c, d, colour, 0);
        }
    }
    number(out, score.left as u64, mid - 60.0, 14.0, 28.0, 52.0, 7.0, YOU, true);
    number(out, score.right as u64, mid + 60.0, 14.0, 28.0, 52.0, 7.0, AI, false);
    if let Some(frame) = frame {
        number(out, frame, 16.0, 16.0, 12.0, 22.0, 3.0, FRAME, false);
    }
    let Some(t) = turn else { return };
    let over = outcome.is_some();
    let shown = (t.turn, t.submitted, t.playing, over);
    if b.shown != Some(shown) {
        (b.shown, b.since) = (Some(shown), Some(Instant::now()));
    }
    number(out, t.turn, CANVAS.0 - 16.0, 16.0, 12.0, 22.0, 3.0, FRAME, true);
    // Lamps 32 px square, 24 px outside a two-digit score; timers 16 px
    // beyond them. Mirrored about the centre line.
    let waited = b.since.map_or(0, |s| s.elapsed().as_secs());
    for (i, (lit, dim)) in [(YOU, YOU_DIM), (AI, AI_DIM)].into_iter().enumerate() {
        let on = match outcome {
            Some(o) => o.winner as usize == i,
            None => t.playing || t.submitted[i],
        };
        let (x0, x1) = if i == 0 { (mid - 182.0, mid - 150.0) } else { (mid + 150.0, mid + 182.0) };
        rect(out, x0, 24.0, x1, 56.0, if on { lit } else { dim }, 0);
        if !over && !t.playing && !t.submitted[i] {
            let (x, right) = if i == 0 { (mid - 198.0, true) } else { (mid + 198.0, false) };
            number(out, waited, x, 26.0, 16.0, 28.0, 4.0, lit, right);
        }
    }
    if over {
        return;
    }
    if t.playing {
        // A triangle pointing right, in 4 px slices.
        for k in 0..8 {
            let (y0, y1) = (24.0 + k as f32 * 4.0, 28.0 + k as f32 * 4.0);
            let w = 28.0 * (1.0 - ((y0 + y1) / 2.0 - 40.0).abs() / 16.0);
            rect(out, mid - 12.0, y0, mid - 12.0 + w, y1, PLAYING, 0);
        }
    } else {
        rect(out, mid - 12.0, 26.0, mid - 4.0, 54.0, PAUSED, 0);
        rect(out, mid + 4.0, 26.0, mid + 12.0, 54.0, PAUSED, 0);
    }
}

impl PongView {
    fn draw(
        &mut self,
        b: &mut Band,
        _: &mut Cx,
        mut list: Pass<DrawList>,
        mut balls: Query<(&Position, &Collider), With<Ball>>,
        mut paddles: Query<(&Position, &Collider, &Paddle)>,
        mut goals: Query<(&Position, &Collider, &Goal)>,
        mut walls: Query<(&Position, &Collider), (Without<Ball>, Without<Paddle>, Without<Goal>)>,
        (mut scores, mut clocks): (Query<&Score>, Query<&Clock>),
        (mut turns, mut outcomes): (Query<&Turn>, Query<&Outcome>),
    ) {
        let out = &mut list.items;
        rect(out, 0.0, 0.0, CANVAS.0, CANVAS.1, BACKGROUND, 0);
        let mid = ORIGIN.0 + WIDTH / 2.0 * CELL;
        for i in 0..(HEIGHT as u32) {
            let y = ORIGIN.1 + i as f32 * CELL;
            rect(out, mid - 2.0, y + CELL * 0.25, mid + 2.0, y + CELL * 0.75, MIDLINE, 0);
        }
        goals.for_each(|_, (at, c, g)| court_box(out, at, c, if g.side < 0.0 { YOU } else { AI }));
        walls.for_each(|_, (at, c)| court_box(out, at, c, WALL));
        paddles.for_each(|_, (at, c, p)| court_box(out, at, c, if p.face < WIDTH / 2.0 { YOU } else { AI }));
        balls.for_each(|_, (at, c)| court_box(out, at, c, BALL));
        let score = scores.single(|_, s| *s).unwrap_or_default();
        let frame = clocks.single(|_, c| c.frame);
        band(out, b, score, frame, turns.single(|_, t| t.now), outcomes.single(|_, o| *o));
        list.len = list.items.len() as u32;
        (list.canvas_w, list.canvas_h) = CANVAS;
    }
}

impl spike_draw::Restage for PongView {
    fn restage(&mut self, b: &mut Band, cx: &mut Cx) -> Vec<Drawn> {
        let mut world = cx.world();
        let score = world.single::<&Score, _>(|_, s| *s).unwrap_or_default();
        let frame = clock::now(&mut world).map(|c| c.frame);
        let (turn, outcome) = (spike_turns::turn(&mut world), spike_turns::outcome(&mut world));
        let mut out = Vec::new();
        band(&mut out, b, score, frame, turn, outcome);
        out.iter().map(Drawn::from).collect()
    }
}

impl Mod for PongView {
    type Transient = Band;

    fn systems(s: &mut Systems<Self>) {
        s.add("draw", Self::draw).phase(phase::RENDER).after("spike_draw::extract").before("spike_present::present");
    }
}

export_mod!(PongView, provides = [spike_draw::Restage]);
