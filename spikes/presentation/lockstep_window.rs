//! SPIKE (get-3hd.1): `spike_lockstep_window`, the lockstep bootstrap
//! (`//engine/std/lockstep`) with a spectator window. Its mod name is
//! `lockstep`, so an agent drives it exactly as it drives plain lockstep:
//! `modctl send lockstep step 30`. Three changes:
//!
//! - **The window stays pumped while idle.** Plain lockstep blocks in
//!   `pump_loader` for up to a second. This waits at most `IDLE`, then
//!   pumps the platform's window; when the pump saw events (an expose, a
//!   resize), it asks the presenter to draw the last frame again
//!   (`spike_platform::redraw`), since no frame will. With turns it also
//!   redraws when the barrier's status changes and every `TICK` while a
//!   turn is open, so the view's status band (who has submitted, a wait
//!   timer) stays current. The close button or Escape quits, between steps
//!   or during one.
//! - **Pacing.** With a window and pacing on (the default), a step's frames
//!   are spread out in real time, each `dt / speed` seconds apart, so a
//!   spectator sees `step 30` played out rather than its last frame. It
//!   only sleeps between frames: every frame covers the same `dt` either
//!   way, so pacing changes when frames run, never what they compute.
//!   `pace off` runs steps as fast as they compute, as plain lockstep does.
//! - **Turns, for two players** (on when the engine starts with
//!   `SPIKE_TURNS=<frames>`). `step` is refused; instead each side submits
//!   an action for the open turn (`turn left up`), and once both have, the
//!   turn's frames run, paced as a step's are, and the next turn opens. The
//!   barrier is here, not in a game mod, because only the bootstrap makes
//!   time move: a reloadable mod's message handler can't run a frame.
//!   Requests never wait for a turn: a submit replies at once, and the
//!   turn's frames run one per pass of the loop below, between pumps, so
//!   `state` polls are answered while it plays. What a turn means to the
//!   game is the game's: this publishes a `spike_turns::Turn` (the actions
//!   in force, who has submitted), which `pong_versus` steers by, and stops
//!   when the game spawns a `spike_turns::Outcome`.
//!
//! - **Recording** (on when the engine starts with `SPIKE_RECORD=<file>`,
//!   which the launcher sets). Every frame's inputs, the turns, points and a
//!   check of the ball, paddles and score at the end of each step or turn
//!   go to a JSONL session log, enough to replay the session exactly
//!   (`pong_replay.rs`; the format is in REVIEW.md). The game's side is a
//!   `spike_record::Watch` that `spike_pong_record` fills each frame and
//!   this reads after it; the `Watch` sits on the clock entity whether or
//!   not a log is written, so a replay's world is the recorded one. Files
//!   are written here because this build is resident: it is never
//!   unloaded, so nothing it holds outlives its code (as with the window).
//!
//! Messages: `step [frames] [at fps]` and `frame`, as lockstep;
//! `pace on|off|<speed>` (a speed of 0.5 is half speed); `pace` reports;
//! `note <text>` (a line in the session log); with turns, `turn
//! <left|right> <up|down|stay>`, `turn` (reports) and `turn length
//! <frames>`. For the replay tool, which drives this without its loop:
//! `turns <frames>` (what `SPIKE_TURNS` does at start) and `turn frame`
//! (one frame of the playing turn, as the loop plays it).

use std::fmt::Write as _;
use std::io::Write as _;
use std::time::{Duration, Instant};

use clock::Clock;
use engine_api::{Bootstrap, Cx, Entity, Mod, Pumped, Status, export_mod};
use spike_record::Watch;
use spike_turns::{SIDES, Turn, TurnInfo};

const DT: f32 = 1.0 / 60.0;
const MAX_STEPS: u64 = 100_000;
/// The longest the window goes unpumped while no requests come: a frame
/// at 60 Hz, so a drag or a close feels immediate.
const IDLE: Duration = Duration::from_millis(16);
/// A turn's frames when `SPIKE_TURNS` isn't a number: 0.1 s.
const TURN_FRAMES: u32 = 6;
/// How often an open turn is redrawn while it waits, so the view's wait
/// timer (whole seconds) ticks: a quarter second keeps it at most that
/// late without the bootstrap knowing when the view started counting.
const TICK: Duration = Duration::from_millis(250);

engine_api::mod_state! {
    #[derive(Default)]
    struct Lockstep {
        frame: u64,
        clock: Option<Entity>,
        /// Pacing is on unless this is set: the default state is the
        /// spectator's.
        unpaced: bool,
        /// Real-time speed while paced; 0 means 1.
        speed: f32,
        /// The window was closed during a step: quit once it's replied.
        quit: bool,
        /// Turns are on: `step` is refused, `turn` plays.
        turns: bool,
        /// The open (or playing) turn and its progress, as published.
        turn: TurnInfo,
        turn_slot: Option<Entity>,
        /// Frames the next turn to start runs; `turn length` sets it, and
        /// a turn already playing keeps its own.
        next_frames: u32,
        /// What each side submitted for the open turn: kept here, not in
        /// `turn`, so neither sees the other's move before it plays.
        pending: [f32; 2],
        /// The game spawned an `Outcome`: no more turns.
        over: bool,
    }
}

/// When the last frame ran, which the next paced frame waits a frame
/// after. Transient, not state: an `Instant` isn't a `FieldType`, and a
/// resident mod is never carried to another build anyway.
#[derive(Default)]
pub struct Pace {
    last_frame: Option<Instant>,
    /// The barrier's status (`Lockstep::shown`) at the last idle redraw,
    /// and when that was.
    shown: Option<(u64, [bool; 2], bool, bool, u32)>,
    redrawn: Option<Instant>,
    log: SessionLog,
}

/// The session log, if `SPIKE_RECORD` names one: JSONL, a line per record,
/// each written whole as it happens (no buffer), so a crash or a kill
/// keeps everything up to it.
#[derive(Default)]
pub struct SessionLog {
    file: Option<std::fs::File>,
    opened: Option<Instant>,
    /// The score last recorded, to record points as it changes.
    score: [u32; 2],
}

/// An f32 as JSON: Rust's shortest text that parses back to the same bits,
/// which is what lets the replay compare exactly. JSON has no NaN.
fn num(v: f32) -> String {
    if v.is_finite() { format!("{v}") } else { "null".into() }
}

fn nums(vs: &[f32]) -> String {
    format!("[{}]", vs.iter().map(|&v| num(v)).collect::<Vec<_>>().join(","))
}

fn json_str(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out + "\""
}

impl SessionLog {
    fn open(&mut self, cx: &Cx) {
        let Ok(path) = std::env::var("SPIKE_RECORD") else { return };
        match std::fs::OpenOptions::new().create(true).append(true).open(&path) {
            Ok(f) => {
                self.file = Some(f);
                self.opened = Some(Instant::now());
                cx.log(format!("recording the session to {path}"));
            }
            Err(e) => cx.log(format!("not recording: opening {path}: {e}")),
        }
    }

    /// Writes `{"t":"<kind>",<fields>,"ms":<since open>}`. A failed write
    /// stops recording rather than the game.
    fn line(&mut self, cx: &Cx, kind: &str, fields: &str) {
        let Some(f) = &mut self.file else { return };
        let ms = self.opened.map_or(0, |t| t.elapsed().as_millis());
        let sep = if fields.is_empty() { "" } else { "," };
        if let Err(e) = writeln!(f, "{{\"t\":\"{kind}\"{sep}{fields},\"ms\":{ms}}}") {
            cx.log(format!("recording stopped: {e}"));
            self.file = None;
        }
    }

    fn check(&mut self, cx: &mut Cx, frame: u64) {
        if self.file.is_none() {
            return;
        }
        let Some(w) = spike_record::watch(&mut cx.world()) else { return };
        let fields = format!(
            "\"frame\":{frame},\"watched\":{},\"ball\":{},\"paddles\":{},\"score\":[{},{}]",
            w.frame,
            nums(&w.ball),
            nums(&w.paddles),
            w.score[0],
            w.score[1]
        );
        self.line(cx, "check", &fields);
    }

    /// After frame `frame`: its inputs and any point, from the `Watch`.
    fn frame(&mut self, cx: &mut Cx, frame: u64) {
        if self.file.is_none() {
            return;
        }
        let Some(w) = spike_record::watch(&mut cx.world()) else { return };
        if w.frame != frame {
            return;
        }
        if w.steers > 0 {
            let fields =
                format!("\"frame\":{frame},\"to\":\"pong_text\",\"msg\":\"{}\",\"steers\":{}", spike_turns::action(w.steer), w.steers);
            self.line(cx, "input", &fields);
        }
        if w.score != self.score {
            let by = if w.score[0] > self.score[0] { "left" } else { "right" };
            self.score = w.score;
            let fields = format!("\"frame\":{frame},\"by\":\"{by}\",\"score\":[{},{}],\"ball\":{}", w.score[0], w.score[1], nums(&w.ball));
            self.line(cx, "point", &fields);
        }
    }
}

/// What a pump of the window found.
enum Window {
    /// No platform, or no window: nothing to pace or keep drawn.
    None,
    Open {
        events: u32,
    },
    Closed,
}

fn pump_window(cx: &mut Cx) -> Window {
    match spike_platform::pump(cx) {
        Ok(p) if p.close || p.keys.iter().any(|k| k.contains("Escape")) => Window::Closed,
        Ok(p) if p.width > 0 => Window::Open { events: p.events },
        _ => Window::None,
    }
}

impl Lockstep {
    fn run_frame(&mut self, pace: &mut Pace, cx: &mut Cx, dt: f32) {
        self.frame += 1;
        {
            let mut world = cx.world();
            clock::publish(&mut world, &mut self.clock, Clock { frame: self.frame, dt });
            // Whether or not a log is written, so a recorded world and its
            // replay are the same world.
            if let Some(e) = self.clock
                && world.get::<Watch>(e).is_none()
            {
                world.insert(e, Watch::default());
            }
        }
        cx.run_frame_for(dt);
        pace.log.frame(cx, self.frame);
    }

    fn speed(&self) -> f32 {
        if self.speed > 0.0 { self.speed } else { 1.0 }
    }

    fn gap(&self, dt: f32) -> Duration {
        Duration::from_secs_f32(dt / self.speed())
    }

    /// `fps` is the step's `at <fps>`, as given, for the log: the replay
    /// sends it back, so its `dt` is parsed from the same text.
    fn step(&mut self, pace: &mut Pace, cx: &mut Cx, n: u64, dt: f32, fps: Option<&str>) -> String {
        let from = self.frame + 1;
        let reply = self.step_frames(pace, cx, n, dt);
        if self.frame >= from {
            let fps = fps.map_or("null".into(), json_str);
            let fields = format!("\"from\":{from},\"to\":{},\"fps\":{fps},\"dt\":{}", self.frame, num(dt));
            pace.log.line(cx, "step", &fields);
            pace.log.check(cx, self.frame);
        }
        reply
    }

    fn step_frames(&mut self, pace: &mut Pace, cx: &mut Cx, n: u64, dt: f32) -> String {
        let window = pump_window(cx);
        if let Window::Closed = window {
            self.quit = true;
            return format!("frame {} (the window was closed: quitting)", self.frame);
        }
        let paced = !self.unpaced && matches!(window, Window::Open { .. });
        let gap = self.gap(dt);
        for i in 0..n {
            // A frame a gap after the last one, the last step's included, so
            // an agent that sends `step 6` ten times a second shows the same
            // pace as one `step 60`. A slower agent's next step starts at
            // once: the wait is only ever up to a frame.
            if paced && let Some(last) = pace.last_frame {
                let next = last + gap;
                let now = Instant::now();
                if next > now {
                    std::thread::sleep(next - now);
                }
                if let Window::Closed = pump_window(cx) {
                    self.quit = true;
                    return format!("frame {} (the window was closed after {i} of {n} frames: quitting)", self.frame);
                }
            }
            // Taken as the frame starts, so a slow frame isn't paid twice.
            pace.last_frame = Some(Instant::now());
            self.run_frame(pace, cx, dt);
        }
        format!("frame {}", self.frame)
    }

    fn publish_turn(&mut self, cx: &mut Cx) {
        spike_turns::publish(&mut cx.world(), &mut self.turn_slot, Turn { now: self.turn });
    }

    fn start_turns(&mut self, cx: &mut Cx, frames: u32) {
        self.turns = true;
        self.next_frames = frames;
        self.turn = TurnInfo { turn: 1, frames, ..TurnInfo::default() };
        self.publish_turn(cx);
    }

    /// What a spectator sees of the barrier: an idle redraw follows any
    /// change to it.
    fn shown(&self) -> (u64, [bool; 2], bool, bool, u32) {
        (self.turn.turn, self.turn.submitted, self.turn.playing, self.over, self.turn.frames)
    }

    fn waiting_for(&self) -> String {
        let waiting: Vec<&str> = SIDES.iter().zip(self.turn.submitted).filter(|(_, s)| !s).map(|(side, _)| *side).collect();
        waiting.join(" and ")
    }

    fn turn_status(&self) -> String {
        let t = &self.turn;
        if self.over {
            format!("game over after turn {}, frame {}", t.turn, self.frame)
        } else if t.playing {
            format!("turn {} playing, frame {} of {} (frame {})", t.turn, t.played, t.frames, self.frame)
        } else {
            format!("turn {} open, {} frames, waiting for {} (frame {})", t.turn, t.frames, self.waiting_for(), self.frame)
        }
    }

    /// A side's action for the open turn. Replies at once: the turn plays
    /// from the loop, a frame a pass.
    fn submit(&mut self, pace: &mut Pace, cx: &mut Cx, side: &str, action: &str) -> Result<String, String> {
        let Some(i) = SIDES.iter().position(|s| *s == side) else {
            return Err(format!("no side {side:?}: left or right"));
        };
        let intent = spike_turns::intent(action).ok_or_else(|| format!("no action {action:?}: up, down or stay"))?;
        if self.over {
            return Err(self.turn_status());
        }
        if self.turn.playing {
            return Err(format!("{}; turn {} opens when it ends", self.turn_status(), self.turn.turn + 1));
        }
        // A second submit in the same turn replaces the first.
        self.pending[i] = intent;
        self.turn.submitted[i] = true;
        let n = self.turn.turn;
        pace.log.line(cx, "submit", &format!("\"turn\":{n},\"side\":\"{side}\",\"action\":\"{action}\""));
        if self.turn.submitted.iter().all(|&s| s) {
            self.turn.playing = true;
            self.turn.played = 0;
            self.turn.intents = self.pending;
            self.publish_turn(cx);
            // What the replay applies: the actions, from the frame after this one.
            let fields = format!(
                "\"turn\":{n},\"from\":{},\"frames\":{},\"left\":\"{}\",\"right\":\"{}\"",
                self.frame + 1,
                self.turn.frames,
                spike_turns::action(self.pending[0]),
                spike_turns::action(self.pending[1])
            );
            pace.log.line(cx, "turn", &fields);
            Ok(format!("submitted for turn {n}: both in, playing {} frames", self.turn.frames))
        } else {
            self.publish_turn(cx);
            Ok(format!("submitted for turn {n}, waiting for {}", self.waiting_for()))
        }
    }

    /// One frame of the playing turn, and the next turn opened after its
    /// last one, or the game's end.
    fn play_frame(&mut self, pace: &mut Pace, cx: &mut Cx) {
        pace.last_frame = Some(Instant::now());
        self.run_frame(pace, cx, DT);
        self.turn.played += 1;
        if let Some(outcome) = spike_turns::outcome(&mut cx.world()) {
            self.over = true;
            self.turn.playing = false;
            cx.log(format!("game over at frame {}", self.frame));
            let winner = SIDES.get(outcome.winner as usize).copied().unwrap_or("?");
            pace.log.line(cx, "over", &format!("\"frame\":{},\"turn\":{},\"winner\":\"{winner}\"", self.frame, self.turn.turn));
            pace.log.check(cx, self.frame);
        } else if self.turn.played >= self.turn.frames {
            pace.log.check(cx, self.frame);
            let n = self.turn.turn + 1;
            self.turn = TurnInfo { turn: n, frames: self.next_frames, intents: self.turn.intents, ..TurnInfo::default() };
        }
        self.publish_turn(cx);
    }

    fn handle(&mut self, pace: &mut Pace, cx: &mut Cx, message: &str) -> Result<String, String> {
        if let Some(text) = message.trim_start().strip_prefix("note")
            && (text.is_empty() || text.starts_with(char::is_whitespace))
        {
            if pace.log.file.is_none() {
                return Err("not recording: notes go to the session log (SPIKE_RECORD)".into());
            }
            let fields = format!("\"frame\":{},\"text\":{}", self.frame, json_str(text.trim()));
            pace.log.line(cx, "note", &fields);
            return Ok(format!("noted at frame {}", self.frame));
        }
        let mut words = message.split_whitespace();
        match (words.next(), words.next(), words.next(), words.next()) {
            (Some("step"), ..) if self.turns => {
                Err("this game plays in turns: `turn <left|right> <up|down|stay>`; `turn` says whose move it is".into())
            }
            (Some("step"), n, at, fps) => {
                let n: u64 = match n {
                    None => 1,
                    Some(n) => n.parse().map_err(|_| format!("not a frame count: {n:?}"))?,
                };
                let dt = match (at, fps) {
                    (None, None) => DT,
                    (Some("at"), Some(fps)) => {
                        let fps: f32 = fps.parse().map_err(|_| format!("not a frame rate: {fps:?}"))?;
                        if fps <= 0.0 {
                            return Err("a frame rate is frames per second".into());
                        }
                        1.0 / fps
                    }
                    _ => return Err(USAGE.into()),
                };
                if n > MAX_STEPS {
                    return Err(format!("at most {MAX_STEPS} frames per step"));
                }
                Ok(self.step(pace, cx, n, dt, fps))
            }
            (Some("frame"), None, None, None) => Ok(format!("frame {}", self.frame)),
            (Some("turn"), ..) if !self.turns => Err("turns are off: start the engine with SPIKE_TURNS=<frames>".into()),
            (Some("turn"), None, None, None) => Ok(self.turn_status()),
            (Some("turn"), Some("length"), Some(n), None) => {
                let n: u32 = n.parse().ok().filter(|&n| n > 0 && u64::from(n) <= MAX_STEPS).ok_or("a turn is 1 or more frames")?;
                self.next_frames = n;
                if !self.turn.playing && !self.over {
                    self.turn.frames = n;
                    self.publish_turn(cx);
                }
                Ok(format!("turns are {n} frames from turn {}", if self.turn.playing { self.turn.turn + 1 } else { self.turn.turn }))
            }
            (Some("turn"), Some("frame"), None, None) => {
                if !self.turn.playing {
                    return Err(self.turn_status());
                }
                self.play_frame(pace, cx);
                Ok(self.turn_status())
            }
            (Some("turn"), Some(side), Some(action), None) => self.submit(pace, cx, side, action),
            (Some("turns"), Some(n), None, None) => {
                if self.turns {
                    return Err("turns are already on".into());
                }
                let n: u32 = n.parse().ok().filter(|&n| n > 0 && u64::from(n) <= MAX_STEPS).ok_or("a turn is 1 or more frames")?;
                self.start_turns(cx, n);
                Ok(format!("turns of {n} frames"))
            }
            (Some("pace"), arg, None, None) => {
                match arg {
                    None => {}
                    Some("on") => self.unpaced = false,
                    Some("off") => self.unpaced = true,
                    Some(speed) => {
                        let speed: f32 = speed.parse().map_err(|_| USAGE.to_string())?;
                        if !(speed > 0.0 && speed.is_finite()) {
                            return Err("a speed is a positive number (1 is real time)".into());
                        }
                        (self.unpaced, self.speed) = (false, speed);
                    }
                }
                Ok(if self.unpaced {
                    "pacing off: steps run as fast as they compute".into()
                } else {
                    format!("pacing on at {}x real time", self.speed())
                })
            }
            _ => Err(USAGE.into()),
        }
    }
}

const USAGE: &str =
    "usage: step [frames] [at fps] | frame | pace [on|off|<speed>] | note <text> | turn [<left|right> <up|down|stay> | length <frames>]";

impl Mod for Lockstep {
    type Transient = Pace;

    fn message(&mut self, pace: &mut Pace, cx: &mut Cx, message: &str) -> Result<String, String> {
        self.handle(pace, cx, message)
    }

    fn close(&mut self, _: &mut Pace, cx: &mut Cx) {
        for e in [self.clock.take(), self.turn_slot.take()].into_iter().flatten() {
            cx.world().despawn(e);
        }
    }
}

impl Bootstrap for Lockstep {
    fn run(&mut self, pace: &mut Pace, cx: &mut Cx) -> Status {
        if let Ok(frames) = std::env::var("SPIKE_TURNS") {
            let frames = frames.parse().ok().filter(|&n| n > 0).unwrap_or(TURN_FRAMES);
            self.start_turns(cx, frames);
            cx.log(format!("lockstep in turns of {frames} frames, with a spectator window: waiting for both sides"));
        } else {
            cx.log("lockstep with a spectator window: waiting for `step`");
        }
        pace.log.open(cx);
        let turns = if self.turns { self.turn.frames } else { 0 };
        pace.log.line(cx, "start", &format!("\"turns\":{turns},\"dt\":{}", num(DT)));
        let status = self.run_loop(pace, cx);
        pace.log.line(cx, "end", &format!("\"frame\":{}", self.frame));
        pace.log.check(cx, self.frame);
        status
    }
}

impl Lockstep {
    fn run_loop(&mut self, pace: &mut Pace, cx: &mut Cx) -> Status {
        let mut window_open = false;
        loop {
            // While a turn plays, wait only until its next frame is due, so
            // requests are still served between its frames.
            let paced = !self.unpaced && window_open;
            let due = pace.last_frame.map(|last| last + self.gap(DT));
            let wait = match (self.turn.playing, paced, due) {
                (false, ..) => IDLE,
                (true, true, Some(due)) => due.saturating_duration_since(Instant::now()).min(IDLE),
                (true, ..) => Duration::ZERO,
            };
            match cx.pump_loader(wait, |cx, message| self.handle(pace, cx, message)) {
                Pumped::Continue => {}
                Pumped::Quit => return Status::QUIT,
                Pumped::Refused => {
                    cx.log("the loader refused to pump; is this build resident?");
                    return Status::ERROR;
                }
            }
            if self.quit {
                cx.log("the window was closed: quitting");
                return Status::QUIT;
            }
            match pump_window(cx) {
                Window::Closed => {
                    cx.log("the window was closed: quitting");
                    return Status::QUIT;
                }
                Window::Open { events } => {
                    window_open = true;
                    // While a turn plays its frames draw; otherwise redraw on
                    // an expose, a change to the barrier (a submit, the turn
                    // ending, the game's end), and every `TICK` of an open
                    // turn, for the view's wait timer. A redraw reads the
                    // world and writes nothing, so none of this touches what
                    // a frame computes.
                    let now = Instant::now();
                    let changed = pace.shown != Some(self.shown());
                    let tick = self.turns && !self.over && pace.redrawn.is_none_or(|t| now >= t + TICK);
                    if !self.turn.playing && (events > 0 || changed || tick) {
                        // Not loaded, or failed: the window just stays as it was.
                        let _ = spike_platform::redraw(cx);
                        (pace.shown, pace.redrawn) = (Some(self.shown()), Some(now));
                    }
                }
                Window::None => window_open = false,
            }
            if self.turn.playing {
                let paced = !self.unpaced && window_open;
                if !paced || pace.last_frame.is_none_or(|last| Instant::now() >= last + self.gap(DT)) {
                    self.play_frame(pace, cx);
                }
            }
        }
    }
}

export_mod!(Lockstep, bootstrap);
