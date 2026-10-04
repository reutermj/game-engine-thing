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

use clock::Clock;
use engine_api::{Cx, Mod, Pass, Query, Systems, With, Without, export_mod, phase};
use physics2d::{CIRCLE, Collider, Position};
use pong::{Ball, Goal, HEIGHT, Paddle, Score, WIDTH};
use spike_draw::{DrawList, Item};

engine_api::mod_state! {
    #[derive(Default)]
    struct PongView {}
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

impl PongView {
    fn draw(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut list: Pass<DrawList>,
        mut balls: Query<(&Position, &Collider), With<Ball>>,
        mut paddles: Query<(&Position, &Collider, &Paddle)>,
        mut goals: Query<(&Position, &Collider, &Goal)>,
        mut walls: Query<(&Position, &Collider), (Without<Ball>, Without<Paddle>, Without<Goal>)>,
        mut scores: Query<&Score>,
        mut clocks: Query<&Clock>,
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
        number(out, score.left as u64, mid - 60.0, 14.0, 28.0, 52.0, 7.0, YOU, true);
        number(out, score.right as u64, mid + 60.0, 14.0, 28.0, 52.0, 7.0, AI, false);
        if let Some(frame) = clocks.single(|_, c| c.frame) {
            number(out, frame, 16.0, 16.0, 12.0, 22.0, 3.0, FRAME, false);
        }
        list.len = list.items.len() as u32;
        (list.canvas_w, list.canvas_h) = CANVAS;
    }
}

impl Mod for PongView {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("draw", Self::draw).phase(phase::RENDER).after("spike_draw::extract").before("spike_present::present");
    }
}

export_mod!(PongView);
