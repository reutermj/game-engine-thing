//! "A reload is invisible", as a property of a real game: replay a route
//! through the game's text interface once without reloads, then again while
//! reloading its mods every few frames, and require every frame of the
//! second to be the first's, bit for bit.
//!
//! Each reload swaps a mod between its build and its twin
//! (`engine_mod(twin = True)`): the same sources under another crate name
//! and build label, so the library is another file with the same code and
//! layouts. The loader skips a build whose file matches the running one's,
//! and `dlopen` would hand back an image it already has, so reloading a
//! build onto itself would prove nothing; a twin is always re-mapped, and
//! the running build is asked its label after every reload
//! (`Engine::build_of`) to prove it.
//!
//! What's compared at every frame boundary is what the game's `snapshot`
//! gathers: the text interfaces' reports and the world's values, in
//! storage order (so a table re-sorted or a row moved differently shows),
//! with each event queue's contents and readers' cursors and every change
//! tick change detection reads. The snapshot at a
//! boundary where the reloading run reloaded is taken after the reload, so
//! it also says the reload itself changed nothing.

use std::fmt::{Debug, Write};
use std::path::PathBuf;

use engine_ecs::{ChildOf, Component, Event};
use engine_loader::engine::Engine;
use physics::{
    Asleep, Body, Collider, Contact, ContactPair, Gravity, Impulse, Manifold, Overlap, Position, Response, Resting, Sleep, Touching,
    Trigger, Velocity,
};
use runfiles::Runfiles;

/// One entry of a route: a message to a mod, or frames to run.
#[derive(Clone, Copy, Debug)]
pub enum Input<'a> {
    Send(&'a str, &'a str),
    Frames(u32),
}

/// Which mods each reload swaps.
#[derive(Clone, Copy, Debug)]
pub enum Batch {
    /// Every mod with a twin, in one batch: a game reload.
    Whole,
    /// One mod per reload, taking turns: each mod reloaded alone, under the
    /// others' running builds.
    EachInTurn,
    /// A pseudo-random nonempty subset per reload, from this seed.
    Mixed(u64),
}

/// Reload before every `every`-th frame, starting before the first, for
/// the first `frames` of the route, or all of it. Every reload of a mod
/// costs a copy, a hash and a `dlopen` of a debug build (about 18 ms for
/// each of pong's five), so the heaviest plans stop early; `REPLAY_FULL`
/// in the environment plays every plan to the end (the `_full` targets).
#[derive(Clone, Copy, Debug)]
pub struct Reloads {
    pub every: u32,
    pub batch: Batch,
    pub frames: Option<u32>,
}

pub struct Game<'a> {
    /// The env var holding the game's manifest's rlocation.
    pub manifest: &'a str,
    pub route: &'a [Input<'a>],
    /// What a frame sends the bootstrap: `step 1`, or `step 1 at <fps>` for
    /// fixed-rate phases that don't take one step a frame.
    pub frame: &'a str,
    /// Everything compared at a frame boundary.
    pub snapshot: fn(&Engine) -> String,
}

/// A mod's two builds, and which is running.
struct Twin {
    name: String,
    paths: [PathBuf; 2],
    running: usize,
    generation: u64,
}

fn engine(game: &Game, test: &str) -> Box<Engine> {
    let manifest = engine_control::read_manifest(&std::env::var(game.manifest).unwrap()).unwrap();
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(test);
    let e = Engine::new(manifest.bootstrap, dir);
    e.load_batch(&manifest.mods).expect("loading the game");
    e
}

/// The twins, from `$TWINS` (`name=rlocation;...`), with each mod's build
/// the one the game loaded. Every mod not resident must have one: a mod
/// added to the game without a twin would otherwise go unreloaded here, and
/// this test would stop meaning what it says.
fn twins(game: &Game, e: &Engine) -> Vec<Twin> {
    let runfiles = Runfiles::create().unwrap();
    let manifest = engine_control::read_manifest(&std::env::var(game.manifest).unwrap()).unwrap();
    let mut twins: Vec<Twin> = std::env::var("TWINS")
        .expect("$TWINS")
        .split(';')
        .map(|entry| {
            let (name, rlocation) = entry.split_once('=').expect("name=rlocation");
            let twin = runfiles.rlocation(rlocation).unwrap_or_else(|| panic!("{rlocation} is not in runfiles"));
            let (_, build) = manifest.mods.iter().find(|(n, _)| n == name).unwrap_or_else(|| panic!("{name} isn't in the game"));
            Twin { name: name.into(), paths: [build.clone(), twin], running: 0, generation: 0 }
        })
        .collect();
    twins.sort_by(|a, b| a.name.cmp(&b.name));
    let mut reloadable: Vec<String> = e
        .list()
        .lines()
        .skip(1)
        .filter(|l| l.starts_with("  ") && !l.contains("[resident]"))
        .map(|l| l.split_whitespace().next().unwrap().to_string())
        .collect();
    reloadable.sort();
    let named: Vec<&str> = twins.iter().map(|t| t.name.as_str()).collect();
    assert_eq!(named, reloadable, "every mod that isn't resident needs a twin in $TWINS");
    twins
}

/// Swaps `which` of `twins` to their other builds in one batch, and checks
/// each was reloaded as it is, into the image asked for.
fn reload(e: &Engine, twins: &mut [Twin], which: &[usize]) {
    let batch: Vec<(String, PathBuf)> =
        which.iter().map(|&i| (twins[i].name.clone(), twins[i].paths[1 - twins[i].running].clone())).collect();
    let reply = e.load_batch(&batch).unwrap_or_else(|err| panic!("reloading {batch:?}: {err}"));
    let mut said: Vec<&str> = reply.split("; ").collect();
    said.sort();
    let mut expected = Vec::new();
    for &i in which {
        let t = &mut twins[i];
        let label = e.build_of(&t.name).unwrap_or_else(|| panic!("{} doesn't say its build", t.name));
        // The twin's label is the build's with `_twin` on the end.
        assert_eq!(label.ends_with("_twin"), t.running == 0, "{} is running {label} after the reload", t.name);
        t.running = 1 - t.running;
        t.generation += 1;
        // Nothing after the generation: no state reset or migrated.
        expected.push(format!("reloaded {} (generation {})", t.name, t.generation));
    }
    expected.sort();
    assert_eq!(said, expected, "{reply}");
}

/// A splitmix64 step: the mixed batches' choices, the same every run.
fn next(seed: &mut u64) -> u64 {
    *seed = seed.wrapping_add(0x9e3779b97f4a7c15);
    let mut z = *seed;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
    z ^ (z >> 31)
}

/// Plays the route as far as `frames`, calling `check` with the snapshot at
/// every frame boundary (before the first frame and after each), reloading
/// as `reloads` says. Returns how many reloads each mod took.
fn play(game: &Game, test: &str, reloads: Option<Reloads>, mut check: impl FnMut(usize, &str)) -> Vec<(String, u64)> {
    let e = engine(game, test);
    let mut twins = twins(game, &e);
    let limit = match reloads {
        Some(r) if std::env::var_os("REPLAY_FULL").is_none() => r.frames.unwrap_or(u32::MAX),
        _ => u32::MAX,
    };
    let (mut frame, mut turn) = (0u32, 0usize);
    let mut seed = match reloads {
        Some(Reloads { batch: Batch::Mixed(seed), .. }) => seed,
        _ => 0,
    };
    let mut boundary = |e: &Engine, frame: u32, twins: &mut [Twin]| {
        if let Some(r) = reloads.filter(|r| frame.is_multiple_of(r.every)) {
            let which: Vec<usize> = match r.batch {
                Batch::Whole => (0..twins.len()).collect(),
                Batch::EachInTurn => {
                    turn += 1;
                    vec![(turn - 1) % twins.len()]
                }
                Batch::Mixed(_) => {
                    let mask = next(&mut seed) % ((1 << twins.len()) - 1) + 1;
                    (0..twins.len()).filter(|i| mask & (1 << i) != 0).collect()
                }
            };
            reload(e, twins, &which);
        }
        check(frame as usize, &(game.snapshot)(e));
    };
    // A message is sent between boundaries, so a boundary's reload comes
    // after the message and before the frame that acts on it: what it sent
    // is in flight across the reload.
    boundary(&e, 0, &mut twins);
    'route: for input in game.route {
        match *input {
            Input::Send(to, message) => {
                e.send(to, message).unwrap_or_else(|err| panic!("{to} {message:?}: {err}"));
            }
            Input::Frames(n) => {
                for _ in 0..n {
                    if frame == limit {
                        break 'route;
                    }
                    e.send("lockstep", game.frame).unwrap();
                    frame += 1;
                    boundary(&e, frame, &mut twins);
                }
            }
        }
    }
    twins.iter().map(|t| (t.name.clone(), t.generation)).collect()
}

/// Plays `game`'s route without reloads, then under each of `plans`, and
/// fails at the first frame boundary that differs, showing the lines that
/// do. Returns every snapshot of the run without reloads, for the test to
/// check the route went somewhere worth comparing.
pub fn assert_invisible(game: &Game, test: &str, plans: &[Reloads]) -> Vec<String> {
    let mut baseline = Vec::new();
    play(game, &format!("{test}_baseline"), None, |_, s| baseline.push(s.to_string()));
    for (k, &plan) in plans.iter().enumerate() {
        let reloads = play(game, &format!("{test}_{k}"), Some(plan), |frame, s| {
            if s != baseline[frame] {
                let diff: Vec<String> = baseline[frame]
                    .lines()
                    .zip(s.lines())
                    .filter(|(a, b)| a != b)
                    .take(12)
                    .map(|(a, b)| format!("  without: {a}\n  with:    {b}"))
                    .collect();
                panic!("frame {frame} differs under {plan:?}:\n{}", diff.join("\n"));
            }
        });
        // Every mod really was swapped, and more than once, under every plan.
        for (name, generation) in reloads {
            assert!(generation >= 2, "{name} was reloaded {generation} time(s) under {plan:?}");
        }
    }
    baseline
}

/// The frames a route runs.
pub fn frames(route: &[Input]) -> u32 {
    route.iter().map(|i| if let Input::Frames(n) = i { *n } else { 0 }).sum()
}

/// Appends `T`'s values in storage order. Floats print as Rust's `Debug`
/// does, which round-trips, so equal text is equal bits (every NaN aside).
pub fn values<T: Component + Clone + Debug>(e: &Engine, out: &mut String) {
    let values = e.world().values::<T>();
    writeln!(out, "{}: {values:?}", T::NAME).unwrap();
}

/// Appends `E`'s queued events and its readers' cursors.
pub fn events<E: Event + Clone + Debug>(e: &Engine, out: &mut String) {
    writeln!(out, "{} events: {:?}", E::NAME, e.world().events_of::<E>()).unwrap();
}

/// What physics and the ECS keep for any game: every physics component and
/// event, `ChildOf`, the world's counts, and what physics reports of its
/// state (steps run and bodies asleep; not its timings, which are wall time).
pub fn physics(e: &Engine, out: &mut String) {
    writeln!(out, "{}", e.world().summary()).unwrap();
    let stats = e.send("physics", "stats").unwrap();
    let steps: String = stats.split(" us/step").next().unwrap().split_once(' ').unwrap().1.split_once(' ').unwrap().1.into();
    writeln!(out, "physics {steps}, {}", e.send("physics", "sleeping").unwrap()).unwrap();
    values::<Position>(e, out);
    values::<Velocity>(e, out);
    values::<Body>(e, out);
    values::<Collider>(e, out);
    values::<ContactPair>(e, out);
    values::<Manifold>(e, out);
    values::<Response>(e, out);
    values::<Impulse>(e, out);
    values::<Overlap>(e, out);
    values::<Gravity>(e, out);
    values::<Sleep>(e, out);
    values::<Asleep>(e, out);
    values::<Resting>(e, out);
    values::<Touching>(e, out);
    values::<ChildOf>(e, out);
    events::<Contact>(e, out);
    events::<Trigger>(e, out);
    ticks(e, out);
}

/// Appends every table's change ticks, by component, in storage order, and
/// the world's tick: what change detection reads (physics wakes a sleeping
/// body a game wrote to by them), so a reload that rewrote them shows
/// before anything they decide does.
fn ticks(e: &Engine, out: &mut String) {
    let w = e.world();
    writeln!(out, "tick {}", w.current_tick()).unwrap();
    for t in w.tables() {
        for (i, &c) in t.components.iter().enumerate() {
            let pages = t.columns[i].read().unwrap();
            let ticks: Vec<u32> = pages.iter().flat_map(|p| p.ticks().iter().copied()).collect();
            writeln!(out, "ticks {}: {ticks:?}", w.name(c)).unwrap();
        }
    }
}
