//! The platformer replayed while its mods reload under it: every frame of
//! a run with reloads is the run without them, bit for bit. See
//! //engine/tests:replay.rs for how, and how the reloads are made real.
//!
//! The level mod rebuilds the level when its map changes, so a reload
//! that lost its state (the map it built) would respawn every tile and
//! its `ChildOf` link, which shows as new entities.

use engine_loader::engine::Engine;
use platformer::{Bounce, Coin, Hurt, Jump, LevelInfo, Player, Run, Tile};
use reload_replay::{Batch, Game, Input, Reloads, assert_invisible, events, physics, values};
use walkers::Walker;

/// `platformer_test`'s routes, typed as an agent does: `step N` to the bootstrap,
/// the rest to the text interface.
fn route(script: &[&'static str]) -> Vec<Input<'static>> {
    script
        .iter()
        .map(|c| match c.strip_prefix("step ") {
            Some(n) => Input::Frames(n.parse().unwrap()),
            None => Input::Send("platformer_text", c),
        })
        .collect()
}

/// Over the pit, collecting the coin, up the platforms to the goal; into the
/// corner, standing still long enough to fall asleep; and a jump onto the
/// walker there, stomping it.
const STOMP: &[&str] = &[
    "step 5", "right", "step 29", "jump", "step 41", "stop", "step 2",
    "right", "step 23", "jump", "step 41", "stop", "step 2",
    "right", "step 45", "jump", "step 66", "stop", "step 1",
    "right", "step 60", "stop", "step 1",
    "step 31", "jump", "step 60",
];

/// Standing at the start until asleep (about frame 40), then a jump from asleep.
const NAP: &[&str] = &["step 60", "jump", "step 40"];

fn snapshot(e: &Engine) -> String {
    // Before the first frame there's no player, which the text says.
    let text = |m| e.send("platformer_text", m).unwrap_or_else(|before| before);
    let mut out = text("state") + "\n" + &text("show") + "\n";
    physics(e, &mut out);
    values::<Tile>(e, &mut out);
    values::<Coin>(e, &mut out);
    values::<LevelInfo>(e, &mut out);
    values::<Player>(e, &mut out);
    values::<platformer::Input>(e, &mut out);
    values::<Walker>(e, &mut out);
    events::<Run>(e, &mut out);
    events::<Jump>(e, &mut out);
    events::<Hurt>(e, &mut out);
    events::<Bounce>(e, &mut out);
    out
}

fn game<'a>(route: &'a [Input<'static>], frame: &'static str) -> Game<'a> {
    Game { manifest: "MANIFEST", route, frame, snapshot }
}

fn every(every: u32, batch: Batch) -> Reloads {
    Reloads { every, batch, frames: None }
}

/// How many frames of `seen` end with a body asleep. The player sleeps a
/// frame at a time: the rules write its velocity every frame, which wakes
/// it (see `platformer_test`).
fn asleep(seen: &[String]) -> usize {
    seen.iter().filter(|s| !s.contains("asleep 0")).count()
}

#[test]
fn reloading_the_whole_game_every_frame_is_invisible_asleep_and_awake() {
    let nap = route(NAP);
    let seen = assert_invisible(&game(&nap, "step 1"), "whole", &[every(1, Batch::Whole)]);
    assert!(asleep(&seen) > 0, "the player never fell asleep");
    assert!(seen.last().unwrap().contains("asleep 0"), "the jump didn't wake it:\n{}", seen.last().unwrap());
}

#[test]
fn reloading_one_mod_a_frame_is_invisible() {
    let stomp = route(STOMP);
    let seen = assert_invisible(&game(&stomp, "step 1"), "each", &[every(1, Batch::EachInTurn)]);
    // The run won, slept in the corner and stomped the walker.
    let last = seen.last().unwrap();
    assert!(last.contains("YOU WIN") && last.contains("deaths 0") && !last.split("world:").next().unwrap().contains("walker"), "{last}");
    assert!(asleep(&seen) > 0, "the player never fell asleep");
    assert!(seen.iter().any(|s| s.contains("platformer::Bounce events: Some(([(")), "no stomp");
}

#[test]
fn reloading_a_few_mods_together_now_and_then_is_invisible() {
    let stomp = route(STOMP);
    assert_invisible(
        &game(&stomp, "step 1"),
        "mixed",
        &[every(3, Batch::Mixed(5)), every(13, Batch::Whole), every(29, Batch::EachInTurn)],
    );
}

/// At 45 frames a second the 60 Hz phases take one step some frames and
/// two others, and the reload between them must keep the time owed.
#[test]
fn reloading_between_uneven_fixed_rate_steps_is_invisible() {
    let nap = route(NAP);
    let seen = assert_invisible(&game(&nap, "step 1 at 45"), "at_45", &[every(1, Batch::Whole), every(2, Batch::Mixed(6))]);
    assert!(asleep(&seen) > 0, "the player never fell asleep");
}
