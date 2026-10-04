//! SPIKE (get-3hd.1): replays a recorded pong session headless, checks it
//! reproduces, and writes what a reviewer needs to investigate it: probes
//! (the ball, the paddles and the ball's contacts, every frame), flags
//! (frames that look wrong, with why) and, on request, frames as PNGs.
//! REVIEW.md says how to run it and read what it writes.
//!
//!     ./bazel run //spikes/presentation:pong_replay -- <session.jsonl> [options]
//!
//!     --out <dir>        where to write (default: `replay/` beside the log)
//!     --frames <n>..<m>  render frames n to m (inclusive) as PNGs
//!     --scale <s>        PNG scale of the 1184x680 canvas (default 0.5)
//!     --depth <cells>    deep_penetration's threshold (default 0.1)
//!     --window <frames>  frames after a paddle hit for vx to turn (default 6)
//!     --plain            one player's session on plain pong: no recorder,
//!                        view or presenter, the standard lockstep bootstrap
//!
//! The game is the recorded one (its `game` line names it) loaded into an
//! `Engine` in this process, as the integration tests do: the same
//! bootstrap (`lockstep_window.rs`) and mods, with `spike_capture` for the
//! window's presenter and no display. Frames are run one at a time through
//! the bootstrap's own messages (`step 1`, or `turn frame` in turns), the
//! recorded inputs sent before the frame they took effect in, so between
//! frames this tool reads the world directly: it isn't a mod.
//!
//! Exit status: 0 if the replay matched every check and point in the log,
//! 1 if it didn't (it still writes everything), 2 if it couldn't run.

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use engine_loader::engine::Engine;
use physics2d::{Collider, Contact, ContactPair, Impulse, Manifold, Position, Response, Velocity};
use pong::{BALL_RADIUS, Ball, HEIGHT, LEFT_FACE, Paddle, RIGHT_FACE, Score, WIDTH};
use spike_record::Watch;

// ---- JSON, as much as the session log needs ----

#[derive(Clone, Debug)]
enum J {
    Null,
    Bool(bool),
    /// Kept as text, so an f32 parses back to the bits that were written.
    Num(String),
    Str(String),
    Arr(Vec<J>),
    Obj(Vec<(String, J)>),
}

impl J {
    fn get(&self, key: &str) -> Option<&J> {
        match self {
            J::Obj(fields) => fields.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
    fn str(&self, key: &str) -> Option<&str> {
        match self.get(key)? {
            J::Str(s) => Some(s),
            _ => None,
        }
    }
    fn u64(&self, key: &str) -> Option<u64> {
        match self.get(key)? {
            J::Num(n) => n.parse().ok(),
            _ => None,
        }
    }
    fn f32s(&self, key: &str) -> Option<Vec<f32>> {
        match self.get(key)? {
            J::Arr(v) => v.iter().map(|x| if let J::Num(n) = x { n.parse().ok() } else { Some(f32::NAN) }).collect(),
            _ => None,
        }
    }
    fn u32s(&self, key: &str) -> Option<Vec<u32>> {
        match self.get(key)? {
            J::Arr(v) => v.iter().map(|x| if let J::Num(n) = x { n.parse().ok() } else { None }).collect(),
            _ => None,
        }
    }
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn ws(&mut self) {
        while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
            self.i += 1;
        }
    }
    fn eat(&mut self, c: u8) -> Result<(), String> {
        self.ws();
        if self.s.get(self.i) == Some(&c) {
            self.i += 1;
            Ok(())
        } else {
            Err(format!("expected {:?} at {}", c as char, self.i))
        }
    }
    fn value(&mut self) -> Result<J, String> {
        self.ws();
        match self.s.get(self.i) {
            Some(b'{') => {
                self.i += 1;
                let mut fields = Vec::new();
                self.ws();
                if self.s.get(self.i) == Some(&b'}') {
                    self.i += 1;
                    return Ok(J::Obj(fields));
                }
                loop {
                    self.ws();
                    let J::Str(k) = self.value()? else { return Err("a key must be a string".into()) };
                    self.eat(b':')?;
                    fields.push((k, self.value()?));
                    self.ws();
                    match self.s.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b'}') => {
                            self.i += 1;
                            return Ok(J::Obj(fields));
                        }
                        _ => return Err(format!("expected , or }} at {}", self.i)),
                    }
                }
            }
            Some(b'[') => {
                self.i += 1;
                let mut items = Vec::new();
                self.ws();
                if self.s.get(self.i) == Some(&b']') {
                    self.i += 1;
                    return Ok(J::Arr(items));
                }
                loop {
                    items.push(self.value()?);
                    self.ws();
                    match self.s.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b']') => {
                            self.i += 1;
                            return Ok(J::Arr(items));
                        }
                        _ => return Err(format!("expected , or ] at {}", self.i)),
                    }
                }
            }
            Some(b'"') => {
                self.i += 1;
                let mut out = String::new();
                loop {
                    let rest = std::str::from_utf8(&self.s[self.i..]).map_err(|e| e.to_string())?;
                    let mut chars = rest.char_indices();
                    let Some((_, c)) = chars.next() else { return Err("unterminated string".into()) };
                    self.i += c.len_utf8();
                    match c {
                        '"' => return Ok(J::Str(out)),
                        '\\' => {
                            let e = *self.s.get(self.i).ok_or("unterminated escape")?;
                            self.i += 1;
                            match e {
                                b'n' => out.push('\n'),
                                b't' => out.push('\t'),
                                b'r' => out.push('\r'),
                                b'u' => {
                                    let hex = std::str::from_utf8(self.s.get(self.i..self.i + 4).ok_or("short \\u")?)
                                        .map_err(|e| e.to_string())?;
                                    let code = u32::from_str_radix(hex, 16).map_err(|e| e.to_string())?;
                                    out.push(char::from_u32(code).unwrap_or('?'));
                                    self.i += 4;
                                }
                                other => out.push(other as char),
                            }
                        }
                        c => out.push(c),
                    }
                }
            }
            Some(b't') if self.s[self.i..].starts_with(b"true") => {
                self.i += 4;
                Ok(J::Bool(true))
            }
            Some(b'f') if self.s[self.i..].starts_with(b"false") => {
                self.i += 5;
                Ok(J::Bool(false))
            }
            Some(b'n') if self.s[self.i..].starts_with(b"null") => {
                self.i += 4;
                Ok(J::Null)
            }
            Some(_) => {
                let start = self.i;
                while self.i < self.s.len() && matches!(self.s[self.i], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9') {
                    self.i += 1;
                }
                if start == self.i {
                    return Err(format!("unexpected {:?} at {}", self.s[self.i] as char, self.i));
                }
                Ok(J::Num(String::from_utf8_lossy(&self.s[start..self.i]).into()))
            }
            None => Err("unexpected end".into()),
        }
    }
}

fn parse_json(line: &str) -> Result<J, String> {
    Parser { s: line.as_bytes(), i: 0 }.value()
}

/// An f32 as JSON, as the bootstrap writes it.
fn num(v: f32) -> String {
    if v.is_finite() { format!("{v}") } else { "null".into() }
}

fn nums(vs: &[f32]) -> String {
    format!("[{}]", vs.iter().map(|&v| num(v)).collect::<Vec<_>>().join(","))
}

// ---- The session ----

struct Step {
    from: u64,
    to: u64,
    fps: Option<String>,
}

struct TurnRec {
    turn: u64,
    from: u64,
    frames: u64,
    left: String,
    right: String,
}

struct Check {
    line: usize,
    watched: u64,
    ball: Vec<f32>,
    paddles: Vec<f32>,
    score: Vec<u32>,
}

#[derive(Default)]
struct Session {
    game: String,
    commit: String,
    /// Recorded with uncommitted changes: the commit alone may not be
    /// what was played.
    dirty: bool,
    turns: u32,
    steps: Vec<Step>,
    turn_recs: Vec<TurnRec>,
    /// Frame: the message, to `pong_text`.
    inputs: BTreeMap<u64, (usize, String)>,
    /// Frame: (line, score).
    points: BTreeMap<u64, (usize, Vec<u32>)>,
    checks: BTreeMap<u64, Check>,
    notes: Vec<(u64, String)>,
    over: Option<u64>,
    end: u64,
}

fn read_session(path: &Path) -> Result<Session, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("reading {}: {e}", path.display()))?;
    let mut s = Session::default();
    for (n, line) in text.lines().enumerate().filter(|(_, l)| !l.trim().is_empty()) {
        let n = n + 1;
        let r = parse_json(line).map_err(|e| format!("line {n}: {e}"))?;
        let bad = |what: &str| format!("line {n}: {what}: {line}");
        let frame = r.u64("frame");
        match r.str("t").ok_or_else(|| bad("no \"t\""))? {
            "meta" => {
                s.game = r.str("game").ok_or_else(|| bad("no game"))?.into();
                s.commit = r.str("commit").unwrap_or("unknown").into();
                s.dirty = matches!(r.get("dirty"), Some(J::Bool(true)));
            }
            "start" => s.turns = r.u64("turns").unwrap_or(0) as u32,
            "step" => {
                let (from, to) = (r.u64("from").ok_or_else(|| bad("no from"))?, r.u64("to").ok_or_else(|| bad("no to"))?);
                s.steps.push(Step { from, to, fps: r.str("fps").map(String::from) });
                s.end = s.end.max(to);
            }
            "turn" => {
                let t = TurnRec {
                    turn: r.u64("turn").ok_or_else(|| bad("no turn"))?,
                    from: r.u64("from").ok_or_else(|| bad("no from"))?,
                    frames: r.u64("frames").ok_or_else(|| bad("no frames"))?,
                    left: r.str("left").ok_or_else(|| bad("no left"))?.into(),
                    right: r.str("right").ok_or_else(|| bad("no right"))?.into(),
                };
                s.turn_recs.push(t);
            }
            "input" => {
                let msg = r.str("msg").ok_or_else(|| bad("no msg"))?;
                s.inputs.insert(frame.ok_or_else(|| bad("no frame"))?, (n, msg.into()));
            }
            "point" => {
                s.points.insert(frame.ok_or_else(|| bad("no frame"))?, (n, r.u32s("score").ok_or_else(|| bad("no score"))?));
            }
            "check" => {
                let f = frame.ok_or_else(|| bad("no frame"))?;
                let check = Check {
                    line: n,
                    watched: r.u64("watched").unwrap_or(0),
                    ball: r.f32s("ball").ok_or_else(|| bad("no ball"))?,
                    paddles: r.f32s("paddles").ok_or_else(|| bad("no paddles"))?,
                    score: r.u32s("score").ok_or_else(|| bad("no score"))?,
                };
                s.checks.insert(f, check);
                s.end = s.end.max(f);
            }
            "note" => s.notes.push((frame.unwrap_or(0), r.str("text").unwrap_or("").into())),
            "over" => s.over = frame,
            "end" | "submit" => {}
            other => return Err(bad(&format!("unknown record {other:?}"))),
        }
    }
    if s.game.is_empty() {
        return Err("no meta line naming the game".into());
    }
    Ok(s)
}

// ---- The world, as probes see it ----

#[derive(Clone, Copy, Default, Debug)]
struct Body2 {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
}

impl Body2 {
    fn speed(&self) -> f32 {
        self.vx.hypot(self.vy)
    }
}

#[derive(Clone, Debug)]
struct BallContact {
    /// What the ball touches: `left`, `right` (the paddles), `top`,
    /// `bottom` (the walls) or `other`.
    with: String,
    depth: f32,
    /// The normal pointing from the ball to the other body.
    n: [f32; 2],
    pressed: bool,
    was_pressed: bool,
    disabled: bool,
    impulse: [f32; 2],
}

#[derive(Clone, Debug)]
struct Began {
    with: String,
    n: [f32; 2],
    speed: f32,
}

#[derive(Clone, Debug, Default)]
struct Probe {
    frame: u64,
    turn: u64,
    ball: Body2,
    /// Left, right: y, vy, intent.
    paddles: [(f32, f32, f32); 2],
    score: [u32; 2],
    contacts: Vec<BallContact>,
    began: Vec<Began>,
    /// How far the ball overlaps each paddle's box (negative: the gap).
    overlap: [f32; 2],
    watch: Option<Watch>,
}

type Key = (u32, u32);

fn key(e: engine_api::Entity) -> Key {
    (e.index, e.generation)
}

fn by_entity<T: engine_api::Component + Clone>(e: &Engine) -> HashMap<Key, T> {
    e.world().values::<T>().unwrap_or_default().into_iter().map(|(e, v)| (key(e), v)).collect()
}

/// How far a circle overlaps a box (negative: the gap between them).
fn overlap(cx: f32, cy: f32, r: f32, bx: f32, by: f32, hx: f32, hy: f32) -> f32 {
    let (dx, dy) = ((cx - bx).abs() - hx, (cy - by).abs() - hy);
    if dx <= 0.0 && dy <= 0.0 { r + (-dx).min(-dy) } else { r - dx.max(0.0).hypot(dy.max(0.0)) }
}

struct Reader {
    next_contact: u64,
}

impl Reader {
    fn probe(&mut self, e: &Engine, frame: u64, turn: u64) -> Result<Probe, String> {
        let balls = e.world().values::<Ball>().unwrap_or_default();
        let ball = key(balls.first().ok_or("no ball in the world")?.0);
        let positions = by_entity::<Position>(e);
        let velocities = by_entity::<Velocity>(e);
        let colliders = by_entity::<Collider>(e);
        let paddles = e.world().values::<Paddle>().unwrap_or_default();
        let at = |k: Key| positions.get(&k).copied().unwrap_or_default();
        let v = |k: Key| velocities.get(&k).copied().unwrap_or_default();
        let (p, bv) = (at(ball), v(ball));
        let mut probe = Probe { frame, turn, ball: Body2 { x: p.x, y: p.y, vx: bv.x, vy: bv.y }, ..Probe::default() };
        let mut names: HashMap<Key, String> = HashMap::new();
        probe.overlap = [f32::NEG_INFINITY; 2];
        for (pe, paddle) in &paddles {
            let side = usize::from(paddle.face > WIDTH / 2.0);
            let k = key(*pe);
            names.insert(k, ["left", "right"][side].into());
            let (pp, pv) = (at(k), v(k));
            probe.paddles[side] = (pp.y, pv.y, paddle.intent);
            if let Some(c) = colliders.get(&k) {
                probe.overlap[side] = overlap(p.x, p.y, BALL_RADIUS, pp.x, pp.y, c.hx, c.hy);
            }
        }
        let name = |k: Key| -> String {
            if let Some(n) = names.get(&k) {
                return n.clone();
            }
            match positions.get(&k) {
                Some(p) if p.y < 0.0 && p.x > 0.0 && p.x < WIDTH => "top".into(),
                Some(p) if p.y > HEIGHT && p.x > 0.0 && p.x < WIDTH => "bottom".into(),
                _ => "other".into(),
            }
        };
        probe.score = e.world().values::<Score>().unwrap_or_default().first().map_or([0, 0], |(_, s)| [s.left, s.right]);
        let manifolds = by_entity::<Manifold>(e);
        let impulses = by_entity::<Impulse>(e);
        let responses = by_entity::<Response>(e);
        for (ce, pair) in e.world().values::<ContactPair>().unwrap_or_default() {
            let Some((other, sign)) = pair.seen_from(balls[0].0) else { continue };
            let k = key(ce);
            let m = manifolds.get(&k).copied().unwrap_or_default();
            let i = impulses.get(&k).copied().unwrap_or_default();
            probe.contacts.push(BallContact {
                with: name(key(other)),
                depth: m.depth,
                n: [m.nx * sign, m.ny * sign],
                pressed: m.pressed,
                was_pressed: m.was_pressed,
                disabled: responses.get(&k).is_some_and(|r| r.disabled),
                impulse: [i.normal, i.tangent],
            });
        }
        if let Some((events, _)) = e.world().events_of::<Contact>() {
            for (seq, _, c) in events {
                if seq < self.next_contact {
                    continue;
                }
                self.next_contact = seq + 1;
                let (other, sign) = if key(c.a) == ball {
                    (c.b, 1.0)
                } else if key(c.b) == ball {
                    (c.a, -1.0)
                } else {
                    continue;
                };
                probe.began.push(Began { with: name(key(other)), n: [c.nx * sign, c.ny * sign], speed: c.speed });
            }
        }
        probe.watch = e.world().values::<Watch>().unwrap_or_default().first().map(|(_, w)| *w);
        Ok(probe)
    }
}

fn probe_json(p: &Probe) -> String {
    let b = &p.ball;
    let mut out = format!(
        "{{\"frame\":{},\"turn\":{},\"ball\":{{\"x\":{},\"y\":{},\"vx\":{},\"vy\":{},\"speed\":{}}}",
        p.frame,
        p.turn,
        num(b.x),
        num(b.y),
        num(b.vx),
        num(b.vy),
        num(b.speed())
    );
    for (i, side) in ["left", "right"].iter().enumerate() {
        let (y, vy, intent) = p.paddles[i];
        let _ =
            write!(out, ",\"{side}\":{{\"y\":{},\"vy\":{},\"intent\":{},\"overlap\":{}}}", num(y), num(vy), num(intent), num(p.overlap[i]));
    }
    let _ = write!(out, ",\"score\":[{},{}],\"contacts\":[", p.score[0], p.score[1]);
    for (i, c) in p.contacts.iter().enumerate() {
        let _ = write!(
            out,
            "{}{{\"with\":\"{}\",\"depth\":{},\"n\":{},\"pressed\":{},\"was_pressed\":{},\"disabled\":{},\"impulse\":{}}}",
            if i > 0 { "," } else { "" },
            c.with,
            num(c.depth),
            nums(&c.n),
            c.pressed,
            c.was_pressed,
            c.disabled,
            nums(&c.impulse)
        );
    }
    out += "],\"began\":[";
    for (i, c) in p.began.iter().enumerate() {
        let _ = write!(out, "{}{{\"with\":\"{}\",\"n\":{},\"speed\":{}}}", if i > 0 { "," } else { "" }, c.with, nums(&c.n), num(c.speed));
    }
    out + "]}"
}

const CSV_HEADER: &str = "frame,turn,ball_x,ball_y,ball_vx,ball_vy,ball_speed,left_y,left_vy,left_intent,right_y,right_vy,right_intent,\
                          score_left,score_right,left_overlap,right_overlap,contacts,began";

fn probe_csv(p: &Probe) -> String {
    let b = &p.ball;
    let contacts: Vec<String> = p.contacts.iter().map(|c| format!("{}:{}", c.with, num(c.depth))).collect();
    let began: Vec<&str> = p.began.iter().map(|c| c.with.as_str()).collect();
    format!(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        p.frame,
        p.turn,
        num(b.x),
        num(b.y),
        num(b.vx),
        num(b.vy),
        num(b.speed()),
        num(p.paddles[0].0),
        num(p.paddles[0].1),
        num(p.paddles[0].2),
        num(p.paddles[1].0),
        num(p.paddles[1].1),
        num(p.paddles[1].2),
        p.score[0],
        p.score[1],
        num(p.overlap[0]),
        num(p.overlap[1]),
        contacts.join(" "),
        began.join(" ")
    )
}

// ---- Detectors ----

struct Settings {
    depth: f32,
    window: u64,
}

/// A paddle hit waiting to see whether the ball turns.
struct Hit {
    frame: u64,
    turn: u64,
    side: usize,
    why: &'static str,
    before: Body2,
    at: Body2,
    paddle_y: f32,
    contact: Option<BallContact>,
    began: Option<Began>,
}

/// A run of frames something holds over, reported as one flag.
struct Run {
    first: u64,
    turn: u64,
    last: u64,
    worst: f32,
    worst_frame: u64,
    detail: String,
}

#[derive(Default)]
struct Detectors {
    flags: Vec<String>,
    counts: BTreeMap<&'static str, u32>,
    hits: Vec<Hit>,
    beyond: Option<Run>,
    deep: [Option<Run>; 2],
    /// Hits seen, and how many turned the ball.
    paddle_hits: u32,
}

fn body(b: &Body2) -> String {
    format!("{{\"x\":{},\"y\":{},\"vx\":{},\"vy\":{}}}", num(b.x), num(b.y), num(b.vx), num(b.vy))
}

impl Detectors {
    fn flag(&mut self, kind: &'static str, frame: u64, turn: u64, fields: String) {
        *self.counts.entry(kind).or_default() += 1;
        self.flags.push(format!("{{\"flag\":\"{kind}\",\"frame\":{frame},\"turn\":{turn},{fields}}}"));
    }

    fn end_run(&mut self, kind: &'static str, run: Option<Run>) {
        if let Some(r) = run {
            let fields =
                format!("\"frames\":[{},{}],\"worst\":{},\"worst_frame\":{},{}", r.first, r.last, num(r.worst), r.worst_frame, r.detail);
            self.flag(kind, r.first, r.turn, fields);
        }
    }

    fn see(&mut self, s: &Settings, prev: &Probe, p: &Probe) {
        let b = p.ball;
        // beyond_wall: the ball's centre outside where the walls turn it.
        let (lo, hi) = (BALL_RADIUS, HEIGHT - BALL_RADIUS);
        let out = if b.y < lo {
            lo - b.y
        } else if b.y > hi {
            b.y - hi
        } else {
            0.0
        };
        // A thousandth of a cell is float noise at a wall, not a bug.
        if out > 1e-3 {
            // The walls' colliders stand a radius outside the court
            // (pong's `set_up`), so the ball's centre turns at y = 0 and
            // y = HEIGHT, not at the limits above: `wall_overlap` is how far
            // the ball is inside a wall's box, the part no bounce explains.
            let wall_overlap = (-b.y).max(b.y - HEIGHT).max(0.0);
            let detail = format!(
                "\"y\":{},\"vy\":{},\"outward\":{},\"limits\":[{},{}],\"wall_overlap\":{},\"ball\":{}",
                num(b.y),
                num(b.vy),
                (b.y < lo && b.vy < 0.0) || (b.y > hi && b.vy > 0.0),
                num(lo),
                num(hi),
                num(wall_overlap),
                body(&b)
            );
            match &mut self.beyond {
                Some(r) if r.last + 1 == p.frame => {
                    r.last = p.frame;
                    if out > r.worst {
                        (r.worst, r.worst_frame, r.detail) = (out, p.frame, detail);
                    }
                }
                _ => {
                    let old = self.beyond.take();
                    self.end_run("beyond_wall", old);
                    self.beyond = Some(Run { first: p.frame, turn: p.turn, last: p.frame, worst: out, worst_frame: p.frame, detail });
                }
            }
        } else {
            let old = self.beyond.take();
            self.end_run("beyond_wall", old);
        }
        // deep_penetration: the ball inside a paddle's box.
        for side in 0..2 {
            let o = p.overlap[side];
            if o > s.depth {
                let contact = p.contacts.iter().find(|c| c.with == ["left", "right"][side]);
                let face = [LEFT_FACE, RIGHT_FACE][side];
                let detail = format!(
                    "\"side\":\"{}\",\"overlap\":{},\"past_face\":{},\"ball\":{},\"paddle_y\":{},\"contact_depth\":{}",
                    ["left", "right"][side],
                    num(o),
                    num(if side == 0 { face - b.x } else { b.x - face }),
                    body(&b),
                    num(p.paddles[side].0),
                    contact.map_or("null".into(), |c| num(c.depth))
                );
                match &mut self.deep[side] {
                    Some(r) if r.last + 1 == p.frame => {
                        r.last = p.frame;
                        if o > r.worst {
                            (r.worst, r.worst_frame, r.detail) = (o, p.frame, detail);
                        }
                    }
                    _ => {
                        let old = self.deep[side].take();
                        self.end_run("deep_penetration", old);
                        self.deep[side] = Some(Run { first: p.frame, turn: p.turn, last: p.frame, worst: o, worst_frame: p.frame, detail });
                    }
                }
            } else {
                let old = self.deep[side].take();
                self.end_run("deep_penetration", old);
            }
        }
        // A paddle hit: pong's rebound kicks the ball on a contact's first
        // frame (`physics2d::Contact`), so that is the trigger; a vertical
        // kick near a paddle without one is a trigger too, in case the two
        // ever part.
        for side in 0..2 {
            let name = ["left", "right"][side];
            let began = p.began.iter().find(|c| c.with == name).cloned();
            let near = if side == 0 { b.x < LEFT_FACE + 3.0 } else { b.x > RIGHT_FACE - 3.0 };
            let kick = (b.vy - prev.ball.vy).abs() > 0.05
                && !p.began.iter().any(|c| c.with == "top" || c.with == "bottom")
                && p.score == prev.score
                && near;
            let why = match (&began, kick) {
                (Some(_), _) => "contact",
                (None, true) => "kick",
                _ => continue,
            };
            if self.hits.iter().any(|h| h.side == side && p.frame <= h.frame + s.window) {
                continue;
            }
            self.paddle_hits += 1;
            let contact = p.contacts.iter().find(|c| c.with == name).cloned();
            self.hits.push(Hit {
                frame: p.frame,
                turn: p.turn,
                side,
                why,
                before: prev.ball,
                at: b,
                paddle_y: p.paddles[side].0,
                contact,
                began,
            });
        }
        // Each pending hit: did vx turn away from the paddle in time?
        for h in std::mem::take(&mut self.hits) {
            let away = if h.side == 0 { b.vx > 0.0 } else { b.vx < 0.0 };
            let incoming = if h.side == 0 { h.before.vx < 0.0 } else { h.before.vx > 0.0 };
            let scored = p.score != prev.score;
            let expired = p.frame >= h.frame + s.window;
            if !(away || scored || expired) {
                self.hits.push(h);
                continue;
            }
            // speed_loss: |v| after the hit against before it.
            let (before, after) = (h.before.speed(), b.speed());
            if away && after < 0.6 * before {
                let fields = format!(
                    "\"side\":\"{}\",\"speed_before\":{},\"speed_after\":{},\"loss\":{},\"before\":{},\"after\":{},\"hit_frame\":{}",
                    ["left", "right"][h.side],
                    num(before),
                    num(after),
                    num(1.0 - after / before),
                    body(&h.before),
                    body(&b),
                    h.frame
                );
                self.flag("speed_loss", h.frame, h.turn, fields);
            }
            if away && incoming {
                continue;
            }
            let kind = if incoming { "hit_no_bounce" } else { "hit_receding" };
            let fields = format!(
                "\"side\":\"{}\",\"trigger\":\"{}\",\"before\":{},\"at\":{},\"then\":{},\"then_frame\":{},\"scored\":{},\"paddle_y\":{},\
                 \"offset\":{},\"reach\":{},\"contact\":{},\"began\":{}",
                ["left", "right"][h.side],
                h.why,
                body(&h.before),
                body(&h.at),
                body(&b),
                p.frame,
                scored,
                num(h.paddle_y),
                num(h.at.y - h.paddle_y),
                num(pong::PADDLE_HEIGHT / 2.0 + BALL_RADIUS),
                h.contact.as_ref().map_or("null".into(), |c| format!(
                    "{{\"depth\":{},\"n\":{},\"pressed\":{},\"disabled\":{},\"impulse\":{}}}",
                    num(c.depth),
                    nums(&c.n),
                    c.pressed,
                    c.disabled,
                    nums(&c.impulse)
                )),
                h.began.as_ref().map_or("null".into(), |c| format!("{{\"n\":{},\"speed\":{}}}", nums(&c.n), num(c.speed)))
            );
            self.flag(kind, h.frame, h.turn, fields);
        }
    }

    fn finish(&mut self) {
        let old = self.beyond.take();
        self.end_run("beyond_wall", old);
        for side in 0..2 {
            let old = self.deep[side].take();
            self.end_run("deep_penetration", old);
        }
    }
}

// ---- Frames: the draw list, rasterised ----

/// The sRGB encoding of a linear 0..255 channel: the window's surface is
/// sRGB, so the presenter's colours are linear and the GPU encodes them;
/// a PNG holds encoded values.
fn srgb_table() -> [u8; 256] {
    let mut t = [0u8; 256];
    for (i, v) in t.iter_mut().enumerate() {
        let l = i as f64 / 255.0;
        let s = if l <= 0.003_130_8 { 12.92 * l } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 };
        *v = (s * 255.0).round() as u8;
    }
    t
}

fn render(items: &str, scale: f32, srgb: &[u8; 256]) -> Result<tiny_skia::Pixmap, String> {
    use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, Rect, Transform};
    let mut lines = items.lines();
    let canvas: Vec<f32> = lines.next().ok_or("no canvas line")?.split_whitespace().skip(1).filter_map(|v| v.parse().ok()).collect();
    let (cw, ch) = match canvas[..] {
        [w, h] if w > 0.0 && h > 0.0 => (w, h),
        _ => (1184.0, 680.0),
    };
    let (w, h) = ((cw * scale).round() as u32, (ch * scale).round() as u32);
    let mut pixmap = Pixmap::new(w, h).ok_or("a pixmap")?;
    pixmap.fill(Color::BLACK);
    let mut paint = Paint { anti_alias: true, ..Paint::default() };
    let t = Transform::from_scale(scale, scale);
    for line in lines {
        let v: Vec<&str> = line.split_whitespace().collect();
        let [x, y, iw, ih, colour, shape] = v[..] else { return Err(format!("bad item {line:?}")) };
        let f = |s: &str| s.parse::<f32>().map_err(|e| format!("{s}: {e}"));
        let (x, y, iw, ih) = (f(x)?, f(y)?, f(iw)?, f(ih)?);
        let colour: u32 = colour.parse().map_err(|e| format!("{colour}: {e}"))?;
        let [r, g, b, a] = colour.to_le_bytes();
        paint.set_color_rgba8(srgb[r as usize], srgb[g as usize], srgb[b as usize], a);
        if shape == "1" {
            if let Some(path) = PathBuilder::from_circle(x, y, iw.min(ih) / 2.0) {
                pixmap.fill_path(&path, &paint, FillRule::Winding, t, None);
            }
        } else if let Some(rect) = Rect::from_xywh(x - iw / 2.0, y - ih / 2.0, iw, ih) {
            pixmap.fill_rect(rect, &paint, t, None);
        }
    }
    Ok(pixmap)
}

/// A PNG of an opaque pixmap, written by hand: tiny-skia's encoder is
/// behind a feature (and the `png` crate) this spike doesn't build, and
/// "no new dependencies". RGB, each row Sub-filtered, deflated with fixed
/// Huffman codes and only two kinds of match (a run of one byte, and the
/// row above): flat colour and repeated rows are all a pong frame is.
fn png(pixmap: &tiny_skia::Pixmap) -> Vec<u8> {
    let (w, h) = (pixmap.width() as usize, pixmap.height() as usize);
    let stride = w * 3 + 1;
    let mut raw = Vec::with_capacity(stride * h);
    let data = pixmap.data();
    for y in 0..h {
        raw.push(1); // Sub
        for x in 0..w {
            for c in 0..3 {
                let v = data[(y * w + x) * 4 + c];
                let left = if x > 0 { data[(y * w + x - 1) * 4 + c] } else { 0 };
                raw.push(v.wrapping_sub(left));
            }
        }
    }
    let mut z = vec![0x78, 0x01];
    z.extend(deflate(&raw, stride));
    z.extend(adler32(&raw).to_be_bytes());
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut ihdr = Vec::new();
    ihdr.extend((w as u32).to_be_bytes());
    ihdr.extend((h as u32).to_be_bytes());
    ihdr.extend([8, 2, 0, 0, 0]);
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend((data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend(kind);
    out.extend(data);
    let crc = crc32(&out[start..]);
    out.extend(crc.to_be_bytes());
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &b in bytes {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

fn adler32(bytes: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &x in bytes {
        a = (a + x as u32) % 65_521;
        b = (b + a) % 65_521;
    }
    (b << 16) | a
}

struct Bits {
    out: Vec<u8>,
    acc: u64,
    n: u32,
}

impl Bits {
    /// `count` bits of `v`, least significant first (deflate's order for
    /// everything but Huffman codes).
    fn put(&mut self, v: u32, count: u32) {
        self.acc |= (v as u64) << self.n;
        self.n += count;
        while self.n >= 8 {
            self.out.push(self.acc as u8);
            self.acc >>= 8;
            self.n -= 8;
        }
    }
    /// A Huffman code, most significant bit first.
    fn code(&mut self, code: u32, len: u32) {
        let mut rev = 0;
        for i in 0..len {
            rev |= ((code >> i) & 1) << (len - 1 - i);
        }
        self.put(rev, len);
    }
    fn literal(&mut self, sym: u32) {
        match sym {
            0..=143 => self.code(0x30 + sym, 8),
            144..=255 => self.code(0x190 + sym - 144, 9),
            256..=279 => self.code(sym - 256, 7),
            _ => self.code(0xc0 + sym - 280, 8),
        }
    }
    fn matched(&mut self, len: usize, dist: usize) {
        const LEN_BASE: [usize; 29] =
            [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
        const LEN_EXTRA: [u32; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
        const DIST_BASE: [usize; 30] = [
            1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289,
            16385, 24577,
        ];
        const DIST_EXTRA: [u32; 30] = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];
        let l = LEN_BASE.iter().rposition(|&b| b <= len).expect("a match is 3 or longer");
        self.literal(257 + l as u32);
        self.put((len - LEN_BASE[l]) as u32, LEN_EXTRA[l]);
        let d = DIST_BASE.iter().rposition(|&b| b <= dist).expect("a distance is 1 or more");
        self.code(d as u32, 5);
        self.put((dist - DIST_BASE[d]) as u32, DIST_EXTRA[d]);
    }
}

fn deflate(data: &[u8], stride: usize) -> Vec<u8> {
    let mut bits = Bits { out: Vec::new(), acc: 0, n: 0 };
    bits.put(1, 1); // the last block
    bits.put(1, 2); // fixed Huffman codes
    let longest = |at: usize, dist: usize| -> usize {
        if dist > at || dist > 32_768 {
            return 0;
        }
        let mut n = 0;
        while n < 258 && at + n < data.len() && data[at + n] == data[at + n - dist] {
            n += 1;
        }
        n
    };
    let mut i = 0;
    while i < data.len() {
        let (run, up) = (longest(i, 1), longest(i, stride));
        let (len, dist) = if up >= run { (up, stride) } else { (run, 1) };
        if len >= 3 {
            bits.matched(len, dist);
            i += len;
        } else {
            bits.literal(data[i] as u32);
            i += 1;
        }
    }
    bits.literal(256);
    bits.put(0, 7); // flush
    bits.out
}

// ---- The replay ----

fn send(e: &Engine, name: &str, message: &str) -> Result<String, String> {
    e.send(name, message).map_err(|err| format!("{name} {message:?}: {err}"))
}

struct Args {
    session: PathBuf,
    out: PathBuf,
    frames: Option<(u64, u64)>,
    scale: f32,
    settings: Settings,
    /// Replay one player's session on plain `//pong` (`pong_plain_replay`):
    /// the standard lockstep bootstrap, no recorder, view or presenter. It
    /// reproducing too shows the spike's additions change nothing.
    plain: bool,
}

fn args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut session = None;
    let (mut out, mut frames, mut scale, mut plain) = (None, None, 0.5, false);
    let mut settings = Settings { depth: 0.1, window: 6 };
    // `bazel run` runs this in its runfiles: relative paths are the user's.
    let cwd = std::env::var("BUILD_WORKING_DIRECTORY").map(PathBuf::from).unwrap_or_default();
    let path = |p: String| cwd.join(p);
    while let Some(a) = it.next() {
        let mut value = || it.next().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--out" => out = Some(path(value()?)),
            "--frames" => {
                let v = value()?;
                let (n, m) = v.split_once("..").ok_or("--frames <n>..<m>")?;
                frames = Some((n.parse().map_err(|_| "--frames <n>..<m>")?, m.parse().map_err(|_| "--frames <n>..<m>")?));
            }
            "--scale" => scale = value()?.parse().map_err(|_| "--scale <number>")?,
            "--depth" => settings.depth = value()?.parse().map_err(|_| "--depth <cells>")?,
            "--window" => settings.window = value()?.parse().map_err(|_| "--window <frames>")?,
            "--plain" => plain = true,
            "-h" | "--help" => {
                return Err("usage: pong_replay <session.jsonl> [--out <dir>] [--frames <n>..<m>] [--scale <s>] [--depth <cells>] \
                            [--window <frames>] [--plain]"
                    .into());
            }
            _ if session.is_none() && !a.starts_with('-') => session = Some(path(a)),
            _ => return Err(format!("unknown argument {a:?}")),
        }
    }
    let session: PathBuf = session.ok_or("usage: pong_replay <session.jsonl> [options] (--help)")?;
    let dir = if plain { "replay-plain" } else { "replay" };
    let out = out.unwrap_or_else(|| session.parent().unwrap_or(Path::new(".")).join(dir));
    if !(scale > 0.0 && scale <= 4.0) {
        return Err("--scale is between 0 and 4".into());
    }
    Ok(Args { session, out, frames, scale, settings, plain })
}

fn main() -> ExitCode {
    // No window: `spike_platform` makes one if it finds a display, and the
    // bootstrap paces a step's frames while one is open. Removing the
    // variable from this process's environment is `unsafe` in edition 2024,
    // so this runs itself again without them.
    if std::env::var_os("DISPLAY").is_some() || std::env::var_os("WAYLAND_DISPLAY").is_some() {
        let argv: Vec<String> = std::env::args().collect();
        let status = std::process::Command::new(&argv[0]).args(&argv[1..]).env_remove("DISPLAY").env_remove("WAYLAND_DISPLAY").status();
        return match status {
            Ok(s) => ExitCode::from(s.code().unwrap_or(2) as u8),
            Err(e) => {
                eprintln!("pong_replay: running without a display: {e}");
                ExitCode::from(2)
            }
        };
    }
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("pong_replay: {e}");
            ExitCode::from(2)
        }
    }
}

/// Which turn (versus) or step (one player) frame `f` was played in.
fn turn_of(s: &Session, f: u64) -> u64 {
    if s.turns > 0 {
        s.turn_recs.iter().rev().find(|t| t.from <= f).map_or(0, |t| t.turn)
    } else {
        s.steps.iter().position(|st| st.from <= f && f <= st.to).map_or(0, |i| i as u64 + 1)
    }
}

fn run() -> Result<bool, String> {
    let a = args()?;
    let s = read_session(&a.session)?;
    let manifest_var = match (s.game.as_str(), a.plain) {
        ("pong_window", false) => "PONG_WINDOW_REPLAY",
        ("pong_window", true) if a.frames.is_some() => return Err("--plain has no draw list: no --frames".into()),
        ("pong_window", true) => "PONG_PLAIN_REPLAY",
        ("pong_versus", false) => "PONG_VERSUS_REPLAY",
        ("pong_versus", true) => return Err("--plain is one player only: turns need the spike's bootstrap".into()),
        (other, _) => return Err(format!("no replay game for {other:?}")),
    };
    let rlocation = std::env::var(manifest_var).map_err(|_| format!("{manifest_var} isn't set: run me with ./bazel run"))?;
    let manifest = engine_control::read_manifest(&rlocation)?;
    std::fs::create_dir_all(&a.out).map_err(|e| format!("making {}: {e}", a.out.display()))?;
    let staging = std::env::temp_dir().join(format!("pong-replay-{}", std::process::id()));
    let engine = Engine::new(manifest.bootstrap, staging.clone());
    engine.load_batch(&manifest.mods).map_err(|e| format!("loading the game: {e}"))?;
    let t = std::time::Instant::now();
    let result = replay(&engine, &a, &s);
    println!("replayed in {:.2} s", t.elapsed().as_secs_f64());
    engine.shutdown();
    let _ = std::fs::remove_dir_all(&staging);
    result
}

fn write_file(path: &Path, text: &str) -> Result<(), String> {
    std::fs::write(path, text).map_err(|e| format!("writing {}: {e}", path.display()))
}

fn replay(e: &Engine, a: &Args, s: &Session) -> Result<bool, String> {
    if !a.plain {
        send(e, "lockstep", "pace off")?;
    }
    if s.turns > 0 {
        send(e, "lockstep", &format!("turns {}", s.turns))?;
        // The replay stops where the log does; a point limit set during the
        // session isn't in the log, and needn't be.
        send(e, "pong_versus", "first-to off")?;
    }
    let end = s.end.max(s.over.unwrap_or(0));
    println!(
        "recorded at commit {}{}",
        s.commit,
        if s.dirty { " with uncommitted changes: if this doesn't reproduce, the code may differ" } else { "" }
    );
    println!(
        "replaying {} ({}): {end} frames, {} {}, {} inputs, {} points, {} checks",
        a.session.display(),
        s.game,
        if s.turns > 0 { s.turn_recs.len() } else { s.steps.len() },
        if s.turns > 0 { "turns" } else { "steps" },
        if s.turns > 0 { s.turn_recs.len() * 2 } else { s.inputs.len() },
        s.points.len(),
        s.checks.len()
    );
    let mut probes = std::io::BufWriter::new(std::fs::File::create(a.out.join("probes.jsonl")).map_err(|e| format!("probes.jsonl: {e}"))?);
    let mut csv = String::from(CSV_HEADER);
    csv.push('\n');
    let mut mismatches: Vec<String> = Vec::new();
    let mut reader = Reader { next_contact: 0 };
    let mut detectors = Detectors::default();
    let srgb = srgb_table();
    if a.frames.is_some() {
        std::fs::create_dir_all(a.out.join("frames")).map_err(|e| e.to_string())?;
    }
    let mut prev = reader.probe(e, 0, 0)?;
    let mut score = [0u32; 2];
    let mut turns = s.turn_recs.iter().peekable();
    let (mut checked, mut pointed) = (0, 0);
    let mut rendered = 0;
    for f in 1..=end {
        if s.turns > 0 {
            if let Some(t) = turns.next_if(|t| t.from == f) {
                send(e, "lockstep", &format!("turn length {}", t.frames))?;
                send(e, "lockstep", &format!("turn left {}", t.left))?;
                send(e, "lockstep", &format!("turn right {}", t.right))?;
            }
            send(e, "lockstep", "turn frame").map_err(|err| format!("frame {f}: {err} (is the log's turn record missing?)"))?;
        } else {
            if let Some((_, msg)) = s.inputs.get(&f) {
                send(e, "pong_text", msg)?;
            }
            let step = s.steps.iter().find(|st| st.from <= f && f <= st.to);
            match step.and_then(|st| st.fps.as_deref()) {
                Some(fps) => send(e, "lockstep", &format!("step 1 at {fps}"))?,
                None => send(e, "lockstep", "step 1")?,
            };
        }
        let turn = turn_of(s, f);
        let p = reader.probe(e, f, turn)?;
        // The recorder's `Watch`, where it's loaded (not with --plain): it
        // should be this frame's, and show each logged input applied.
        if let Some(w) = p.watch {
            if w.frame != f {
                mismatches.push(format!("frame {f}: the Watch says frame {}", w.frame));
            }
            if let Some((line, msg)) = s.inputs.get(&f)
                && w.steers == 0
            {
                mismatches.push(format!("frame {f}: log line {line} input {msg:?} wasn't applied"));
            }
        }
        if p.score != score {
            match s.points.get(&f) {
                Some((_, want)) if want[..] == p.score[..] => pointed += 1,
                Some((line, want)) => mismatches.push(format!("frame {f}: score {:?}, log line {line} says {want:?}", p.score)),
                None => mismatches.push(format!("frame {f}: a point ({:?}) the log doesn't have", p.score)),
            }
            score = p.score;
        } else if let Some((line, want)) = s.points.get(&f) {
            mismatches.push(format!("frame {f}: no point, log line {line} says {want:?}"));
        }
        // Checked against the world as this tool reads it, not the `Watch`,
        // so --plain (no recorder) is checked the same way.
        if let Some(c) = s.checks.get(&f) {
            let got_ball = vec![p.ball.x, p.ball.y, p.ball.vx, p.ball.vy];
            let got_paddles = vec![p.paddles[0].0, p.paddles[1].0];
            let bits = |v: &[f32]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
            if bits(&got_ball) != bits(&c.ball) || bits(&got_paddles) != bits(&c.paddles) || p.score[..] != c.score[..] || c.watched != f {
                mismatches.push(format!(
                    "frame {f}: log line {} check differs: ball {} paddles {} score {:?} (logged ball {} paddles {} score {:?})",
                    c.line,
                    nums(&got_ball),
                    nums(&got_paddles),
                    p.score,
                    nums(&c.ball),
                    nums(&c.paddles),
                    c.score
                ));
            } else {
                checked += 1;
            }
        }
        writeln!(probes, "{}", probe_json(&p)).map_err(|e| e.to_string())?;
        csv += &probe_csv(&p);
        csv.push('\n');
        detectors.see(&a.settings, &prev, &p);
        if let Some((n, m)) = a.frames
            && (n..=m).contains(&f)
        {
            let items = send(e, "spike_capture", "items")?;
            let pixmap = render(&items, a.scale, &srgb)?;
            let path = a.out.join("frames").join(format!("frame-{f:06}.png"));
            std::fs::write(&path, png(&pixmap)).map_err(|e| format!("writing {}: {e}", path.display()))?;
            rendered += 1;
        }
        prev = p;
    }
    detectors.finish();
    probes.flush().map_err(|e| e.to_string())?;
    write_file(&a.out.join("probes.csv"), &csv)?;
    let mut flags = String::new();
    for f in &detectors.flags {
        flags += f;
        flags.push('\n');
    }
    write_file(&a.out.join("flags.jsonl"), &flags)?;

    let ok = mismatches.is_empty() && checked == s.checks.len() && pointed == s.points.len();
    let mut report = String::new();
    let _ = writeln!(
        report,
        "{}: {checked} of {} checks and {pointed} of {} points matched, {} mismatches",
        if ok { "REPRODUCED" } else { "NOT REPRODUCED" },
        s.checks.len(),
        s.points.len(),
        mismatches.len()
    );
    for m in mismatches.iter().take(10) {
        let _ = writeln!(report, "  {m}");
    }
    if mismatches.len() > 10 {
        let _ = writeln!(report, "  ... and {} more", mismatches.len() - 10);
    }
    let _ = writeln!(report, "final: frame {end}, score {:?}, ball {}", prev.score, body(&prev.ball));
    let counts: Vec<String> = detectors.counts.iter().map(|(k, n)| format!("{k} {n}")).collect();
    let _ = writeln!(
        report,
        "flags: {} ({}); paddle hits seen {}",
        detectors.flags.len(),
        if counts.is_empty() { "none".into() } else { counts.join(", ") },
        detectors.paddle_hits
    );
    for (f, text) in &s.notes {
        let _ = writeln!(report, "note at frame {f}: {text}");
    }
    let _ = writeln!(
        report,
        "wrote {}/probes.jsonl, probes.csv, flags.jsonl{}",
        a.out.display(),
        if rendered > 0 { format!(", {rendered} frames in frames/") } else { String::new() }
    );
    write_file(&a.out.join("verify.txt"), &report)?;
    print!("{report}");
    Ok(ok)
}
