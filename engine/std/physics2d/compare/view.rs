//! The debug view: a 2D scene drawn at chosen steps, for every engine side
//! by side, so a scene can be looked at rather than trusted from its
//! numbers. Bodies are drawn from what every engine reports (`Sim::bodies`,
//! positions and angles), contacts from each engine's own (`Sim::marks`:
//! ours from the arrays or from the world's contacts, Box2D's and
//! Rapier's manifolds), and statics from the scene. Two forms: an SVG grid
//! (a row a step, a column an engine), and text, a character grid, for
//! whoever reads rather than sees. Test and bench code only: nothing here
//! is in a mod. How to run it: runbook 005, "The debug view".

use std::fmt::Write;

use physics2d::{Rot, Vec2};

use crate::Dyn;
use crate::scene::{Scene, Spec};
use crate::sim::Mark;

/// One engine at one step.
pub struct Panel {
    pub label: String,
    pub bodies: Vec<Dyn>,
    pub marks: Vec<Mark>,
    pub sleeping: Vec<bool>,
}

/// What every panel shows of the world: the dynamic bodies wherever they
/// go in any panel, and a margin, so statics as wide as a floor don't
/// shrink the bodies to nothing.
fn frame(rows: &[(u32, Vec<Panel>)]) -> (Vec2, Vec2) {
    let (mut lo, mut hi) = (Vec2::new(f32::MAX, f32::MAX), Vec2::new(f32::MIN, f32::MIN));
    for p in rows.iter().flat_map(|(_, ps)| ps) {
        for b in &p.bodies {
            let r = b.hx.hypot(b.hy);
            (lo.x, lo.y) = (lo.x.min(b.x - r), lo.y.min(b.y - r));
            (hi.x, hi.y) = (hi.x.max(b.x + r), hi.y.max(b.y + r));
        }
    }
    let pad = 0.1 * (hi.x - lo.x).max(hi.y - lo.y) + 1.0;
    (Vec2::new(lo.x - pad, lo.y - pad), Vec2::new(hi.x + pad, hi.y + pad))
}

fn corners(x: f32, y: f32, hx: f32, hy: f32, angle: f32) -> [Vec2; 4] {
    let q = Rot::from_angle(angle);
    [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)].map(|(u, v)| Vec2::new(x, y) + q.rotate(Vec2::new(u, v)))
}

fn shape(out: &mut String, circle: bool, x: f32, y: f32, hx: f32, hy: f32, angle: f32, fill: &str) {
    if circle {
        let (c, s) = (angle.cos(), angle.sin());
        let _ = write!(out, "<circle cx='{x}' cy='{y}' r='{hx}' fill='{fill}' stroke='#333'/>");
        // The radius line shows how far it has turned.
        let _ = write!(out, "<line x1='{x}' y1='{y}' x2='{}' y2='{}' stroke='#333'/>", x + hx * c, y + hx * s);
    } else {
        let pts: Vec<String> = corners(x, y, hx, hy, angle).iter().map(|p| format!("{},{}", p.x, p.y)).collect();
        let _ = write!(out, "<polygon points='{}' fill='{fill}' stroke='#333'/>", pts.join(" "));
    }
}

/// The scene at each step for each engine, as one SVG: a row a step, a
/// column an engine. Dynamic bodies blue, sleeping ones grey, statics dark;
/// contact points red (hollow orange where held within the speculative
/// margin, not pressed; a square where ours keeps no point and it is
/// estimated), each with its normal.
pub fn svg(scene: &Scene, rows: &[(u32, Vec<Panel>)]) -> String {
    let (lo, hi) = frame(rows);
    let size = hi - lo;
    // Panels about 360 across, whatever the scene's size.
    let scale = 360.0 / size.x.max(size.y);
    let (pw, ph) = (size.x * scale, size.y * scale);
    let cols = rows.iter().map(|(_, ps)| ps.len()).max().unwrap_or(1);
    let (w, h) = (cols as f32 * (pw + 10.0) + 10.0, rows.len() as f32 * (ph + 30.0) + 30.0);
    let mut out = String::new();
    let _ = write!(out, "<svg xmlns='http://www.w3.org/2000/svg' width='{w}' height='{h}' font-family='monospace' font-size='12'>");
    let _ = write!(out, "<rect width='{w}' height='{h}' fill='white'/><text x='10' y='18'>{}</text>", scene.text());
    let statics: Vec<Spec> = scene.build().into_iter().filter(|s| !s.dynamic).collect();
    // Normals a twentieth of the frame long, points a hundredth across.
    let (normal, dot) = (0.05 * size.x.max(size.y), 0.01 * size.x.max(size.y));
    for (r, (step, panels)) in rows.iter().enumerate() {
        for (c, p) in panels.iter().enumerate() {
            let (ox, oy) = (10.0 + c as f32 * (pw + 10.0), 30.0 + r as f32 * (ph + 30.0));
            let _ = write!(out, "<text x='{ox}' y='{}'>{}, step {step}</text>", oy + 12.0, p.label);
            let _ = write!(
                out,
                "<svg x='{ox}' y='{}' width='{pw}' height='{ph}' viewBox='{} {} {} {}'>",
                oy + 16.0,
                lo.x,
                lo.y,
                size.x,
                size.y
            );
            let _ = write!(out, "<rect x='{}' y='{}' width='{}' height='{}' fill='#f8f8f8' stroke='#ccc'/>", lo.x, lo.y, size.x, size.y);
            // A pixel wide, in world units: the panel is scaled by its viewBox.
            let _ = write!(out, "<g stroke-width='{}'>", 1.0 / scale);
            for s in &statics {
                shape(&mut out, s.circle, s.x, s.y, s.hx, s.hy, s.angle, "#555");
            }
            for (i, b) in p.bodies.iter().enumerate() {
                let fill = if p.sleeping.get(i).copied().unwrap_or(false) { "#aaa" } else { "#8cf" };
                shape(&mut out, b.circle, b.x, b.y, b.hx, b.hy, b.angle, fill);
            }
            for m in &p.marks {
                let colour = if m.speculative { "orange" } else { "red" };
                let (x2, y2) = (m.x + normal * m.nx, m.y + normal * m.ny);
                let _ = write!(out, "<line x1='{}' y1='{}' x2='{x2}' y2='{y2}' stroke='{colour}'/>", m.x, m.y);
                let fill = if m.speculative { "none" } else { colour };
                if m.estimated {
                    let _ = write!(
                        out,
                        "<rect x='{}' y='{}' width='{}' height='{}' fill='{fill}' stroke='{colour}'/>",
                        m.x - dot,
                        m.y - dot,
                        2.0 * dot,
                        2.0 * dot
                    );
                } else {
                    let _ = write!(out, "<circle cx='{}' cy='{}' r='{dot}' fill='{fill}' stroke='{colour}'/>", m.x, m.y);
                }
            }
            out.push_str("</g></svg>");
        }
    }
    out.push_str("</svg>\n");
    out
}

fn inside(p: Vec2, circle: bool, x: f32, y: f32, hx: f32, hy: f32, angle: f32) -> bool {
    let d = p - Vec2::new(x, y);
    if circle {
        return d.len() <= hx;
    }
    let l = Rot::from_angle(angle).unrotate(d);
    l.x.abs() <= hx && l.y.abs() <= hy
}

/// Every point along a shape's outline, `step` apart: what marks a shape
/// thinner than a character (a card, a thin wall), whose inside no
/// character's middle may fall in.
fn outline(circle: bool, x: f32, y: f32, hx: f32, hy: f32, angle: f32, step: f32) -> Vec<Vec2> {
    let c = Vec2::new(x, y);
    if circle {
        let n = ((std::f32::consts::TAU * hx / step).ceil() as usize).max(8);
        return (0..n).map(|k| c + Rot::from_angle(k as f32 * std::f32::consts::TAU / n as f32).rotate(Vec2::new(hx, 0.0))).collect();
    }
    let p = corners(x, y, hx, hy, angle);
    let mut out = Vec::new();
    for k in 0..4 {
        let (a, b) = (p[k], p[(k + 1) % 4]);
        let n = (((b - a).len() / step).ceil() as usize).max(1);
        out.extend((0..=n).map(|i| a + (b - a) * (i as f32 / n as f32)));
    }
    out
}

/// The same, as text `cols` characters wide, a character twice as tall as
/// wide: statics `#`, each dynamic body a letter in turn (a to z, then A to
/// Z), a sleeping one `.`, and on the statics and in the gaps, a pressed
/// contact point `*` and a held one `+` (inside a body the letter wins).
/// Coarse, but enough to see a stack lean, a pile in columns or a body
/// through a wall.
pub fn text(scene: &Scene, rows: &[(u32, Vec<Panel>)], cols: usize) -> String {
    let (lo, hi) = frame(rows);
    let size = hi - lo;
    let cell = size.x / cols as f32;
    let lines = ((size.y / (2.0 * cell)).ceil() as usize).max(1);
    let statics: Vec<Spec> = scene.build().into_iter().filter(|s| !s.dynamic).collect();
    let letters: Vec<char> = ('a'..='z').chain('A'..='Z').collect();
    let mut out = String::new();
    for (step, panels) in rows {
        for p in panels {
            let _ = writeln!(out, "{}, {}, step {step} ({:.2} a character)", scene.text(), p.label, cell);
            let mut grid = vec![vec![' '; cols]; lines];
            let mark = |p: Vec2, ch: char, grid: &mut Vec<Vec<char>>| {
                let (c, r) = ((p.x - lo.x) / cell, (p.y - lo.y) / (2.0 * cell));
                if c >= 0.0 && r >= 0.0 && (r as usize) < lines && (c as usize) < cols {
                    grid[r as usize][c as usize] = ch;
                }
            };
            for s in &statics {
                outline(s.circle, s.x, s.y, s.hx, s.hy, s.angle, 0.5 * cell).into_iter().for_each(|p| mark(p, '#', &mut grid));
            }
            for (r, row) in grid.iter_mut().enumerate() {
                for (c, ch) in row.iter_mut().enumerate() {
                    let at = Vec2::new(lo.x + (c as f32 + 0.5) * cell, lo.y + (r as f32 + 0.5) * 2.0 * cell);
                    if statics.iter().any(|s| inside(at, s.circle, s.x, s.y, s.hx, s.hy, s.angle)) {
                        *ch = '#';
                    }
                    for (i, b) in p.bodies.iter().enumerate() {
                        if inside(at, b.circle, b.x, b.y, b.hx, b.hy, b.angle) {
                            *ch = if p.sleeping.get(i).copied().unwrap_or(false) { '.' } else { letters[i % letters.len()] };
                        }
                    }
                }
            }
            for (i, b) in p.bodies.iter().enumerate() {
                let ch = if p.sleeping.get(i).copied().unwrap_or(false) { '.' } else { letters[i % letters.len()] };
                outline(b.circle, b.x, b.y, b.hx, b.hy, b.angle, 0.5 * cell).into_iter().for_each(|p| mark(p, ch, &mut grid));
            }
            for m in &p.marks {
                let (c, r) = (((m.x - lo.x) / cell) as usize, ((m.y - lo.y) / (2.0 * cell)) as usize);
                if r < lines && c < cols && (grid[r][c] == ' ' || grid[r][c] == '#') {
                    grid[r][c] = if m.speculative { '+' } else { '*' };
                }
            }
            for row in grid {
                let _ = writeln!(out, "|{}|", row.into_iter().collect::<String>());
            }
        }
    }
    out
}
