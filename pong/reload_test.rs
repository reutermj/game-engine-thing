//! Pong replayed while its mods reload under it: every frame of a game with
//! reloads is the game without them, bit for bit. See
//! //engine/tests:replay.rs for how, and how the reloads are made real.

use engine_loader::engine::Engine;
use pong::{Ball, Goal, Opponent, Paddle, Player, Score, Steer};
use reload_replay::{Batch, Game, Input, Reloads, assert_invisible, events, physics, values};

/// `pong_test`'s replay: the serve returned (frame 68), off the bottom wall
/// (about 105) and onto the AI's paddle (about 195).
const RALLY: &[Input] = &[
    Input::Send("pong_text", "down"),
    Input::Frames(24),
    Input::Send("pong_text", "stay"),
    Input::Frames(46),
    Input::Send("pong_text", "up"),
    Input::Frames(25),
    Input::Send("pong_text", "stay"),
    Input::Frames(400),
    Input::Send("pong_text", "down"),
    Input::Frames(13),
];

/// The serve missed: through the player's goal line, a point, and served again.
const MISS: &[Input] = &[Input::Send("pong_text", "stay"), Input::Frames(150)];

fn snapshot(e: &Engine) -> String {
    let mut out = e.send("pong_text", "state").unwrap() + "\n" + &e.send("pong_text", "show").unwrap() + "\n";
    physics(e, &mut out);
    values::<Ball>(e, &mut out);
    values::<Paddle>(e, &mut out);
    values::<Player>(e, &mut out);
    values::<Opponent>(e, &mut out);
    values::<Goal>(e, &mut out);
    values::<Score>(e, &mut out);
    events::<Steer>(e, &mut out);
    out
}

fn game(route: &'static [Input<'static>], frame: &'static str) -> Game<'static> {
    Game { manifest: "PONG_MANIFEST", route, frame, snapshot }
}

fn every(every: u32, batch: Batch) -> Reloads {
    Reloads { every, batch, frames: None }
}

#[test]
fn reloading_the_whole_game_every_frame_is_invisible() {
    let plan = Reloads { frames: Some(210), ..every(1, Batch::Whole) };
    let seen = assert_invisible(&game(RALLY, "step 1"), "whole", &[plan]);
    // Something to see in the frames compared: contacts with both paddles,
    // and the wall between.
    let contacts = seen[..210].iter().filter(|s| s.contains("physics::ContactPair: Some([(")).count();
    assert!(contacts > 3, "{contacts} frames with contacts");
    assert!(seen[100].contains("vx 16.80"), "returned:\n{}", seen[100]);
}

#[test]
fn reloading_one_mod_a_frame_is_invisible() {
    assert_invisible(&game(RALLY, "step 1"), "each", &[every(1, Batch::EachInTurn)]);
}

#[test]
fn reloading_a_few_mods_together_now_and_then_is_invisible() {
    assert_invisible(
        &game(RALLY, "step 1"),
        "mixed",
        &[every(3, Batch::Mixed(1)), every(7, Batch::Mixed(2)), every(16, Batch::Whole), every(41, Batch::EachInTurn)],
    );
}

/// A point: the goal's sensor, its `Trigger`, the score and the next serve.
#[test]
fn reloading_around_a_point_is_invisible() {
    let seen = assert_invisible(&game(MISS, "step 1"), "point", &[every(2, Batch::Mixed(4)), every(5, Batch::Whole)]);
    assert!(seen.last().unwrap().contains("score you 0 ai 1"), "{}", seen.last().unwrap());
    assert!(seen.iter().any(|s| s.contains("physics::Trigger events: Some(([(")), "the goal line never triggered");
}

/// At 45 frames a second, the 60 Hz phases take one step some frames and
/// two others: a reload between them must keep the time the next step is
/// owed.
#[test]
fn reloading_between_uneven_fixed_rate_steps_is_invisible() {
    let plan = Reloads { frames: Some(100), ..every(1, Batch::Whole) };
    let seen = assert_invisible(&game(MISS, "step 1 at 45"), "at_45", &[plan, every(4, Batch::Mixed(3))]);
    assert!(seen.last().unwrap().contains("score you 0 ai 2"), "{}", seen.last().unwrap());
}
