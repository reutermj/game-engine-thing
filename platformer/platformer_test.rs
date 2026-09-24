//! The platformer played through its text interface, against the real game:
//! the mods in //platformer's manifest, driven by the same messages an agent
//! sends. The scripts are the ones an agent played by hand, so each is a
//! route known to work.

use std::path::PathBuf;

use engine_loader::engine::Engine;
use runfiles::Runfiles;

fn game(test: &str) -> Box<Engine> {
    let manifest = engine_control::read_manifest(&std::env::var("MANIFEST").unwrap()).unwrap();
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(test);
    let engine = Engine::new(manifest.bootstrap, dir);
    engine.load_batch(&manifest.mods).expect("loading the platformer");
    engine
}

/// Runs commands as typed: `step N` goes to the lockstep bootstrap, anything
/// else to the text interface.
fn play(e: &Engine, script: &[&str]) {
    for &command in script {
        let (to, message) = if command.starts_with("step") { ("lockstep", command) } else { ("platformer_text", command) };
        e.send(to, message).unwrap_or_else(|err| panic!("{command:?}: {err}"));
    }
}

fn state(e: &Engine) -> String {
    e.send("platformer_text", "state").unwrap()
}

/// The number after `key` on the `player` line of `state`.
fn player(e: &Engine, key: &str) -> f32 {
    let state = state(e);
    let line = state.lines().find(|l| l.starts_with("player")).expect(&state);
    let mut words = line.split_whitespace();
    words.find(|w| *w == key).expect(line);
    words.next().and_then(|v| v.parse().ok()).expect(line)
}

/// Over the pit onto the low platform (collecting the coin above it), up to
/// the middle platform, then up to the goal.
const WINNING_RUN: &[&str] = &[
    "step 5", "right", "step 29", "jump", "step 41", "stop", "step 2", // the low platform
    "right", "step 23", "jump", "step 41", "stop", "step 2", // the middle platform
    "right", "step 45", "jump", "step 66", "stop", "step 1", // the goal
];

/// From the goal: drop into the right-hand corner and wait for the walker.
const TO_THE_CORNER: &[&str] = &["right", "step 60", "stop", "step 1"];

#[test]
fn the_winning_run_wins() {
    let e = game("win");
    play(&e, WINNING_RUN);
    let state = state(&e);
    assert!(state.starts_with("frame 255   coins 1 (1 left)   deaths 0   YOU WIN"), "{state}");
}

#[test]
fn the_same_inputs_replay_the_same_run() {
    let run = |test| {
        let e = game(test);
        play(&e, WINNING_RUN);
        (state(&e), e.send("platformer_text", "show").unwrap())
    };
    assert_eq!(run("replay_a"), run("replay_b"));
}

#[test]
fn landing_on_a_walker_stomps_it() {
    let e = game("stomp");
    play(&e, WINNING_RUN);
    play(&e, TO_THE_CORNER);
    assert!(state(&e).contains("walker"), "{}", state(&e));
    // Timed to come down on it as it turns at the wall.
    play(&e, &["step 31", "jump"]);
    let stomped = (0..60).find(|_| {
        play(&e, &["step 1"]);
        !state(&e).contains("walker")
    });
    assert!(stomped.is_some(), "{}", state(&e));
    // The frame it's stomped, the player bounces off it.
    assert!(player(&e, "vy") < 0.0, "{}", state(&e));
    play(&e, &["step 60"]);
    assert!(state(&e).contains("deaths 0"), "{}", state(&e));
}

#[test]
fn a_walker_that_reaches_the_player_kills_it() {
    let e = game("hurt");
    play(&e, WINNING_RUN);
    play(&e, TO_THE_CORNER);
    play(&e, &["step 91"]);
    let state = state(&e);
    assert!(state.contains("deaths 1"), "{state}");
    assert!(state.contains("walker"), "the walker survives:\n{state}");
}

#[test]
fn a_walker_turns_back_at_the_pit_rather_than_fall_in() {
    let e = game("ledge");
    // Right to the far wall (from 13 to 33) and back to the pit's edge at
    // 10, with the player out of reach on the pit's other side.
    let mut xs = Vec::new();
    for _ in 0..40 {
        play(&e, &["step 30"]);
        let state = state(&e);
        let line = state.lines().find(|l| l.starts_with("walker")).unwrap_or_else(|| panic!("no walker:\n{state}"));
        let n: Vec<f32> = line.split_whitespace().filter_map(|w| w.parse().ok()).collect();
        assert!((n[1] - 11.0).abs() < 0.05, "fell: {line}");
        xs.push(n[0]);
    }
    let min = xs.iter().copied().fold(f32::INFINITY, f32::min);
    // Sampled every 30 frames, so up to 1.5 short of where it turned.
    assert!((9.9..11.6).contains(&min), "turned at {min}, not the pit's edge: {xs:?}");
}

#[test]
fn falling_onto_the_spikes_respawns_the_player() {
    let e = game("pit");
    play(&e, &["step 5", "right", "step 60"]);
    assert!(state(&e).contains("deaths 1"), "{}", state(&e));
    assert!(player(&e, "x") < 7.0, "back at the start:\n{}", state(&e));
}

/// The level mod's own builds, loaded over the running one like
/// `./bazel run //platformer/level` after an edit.
fn level_build(var: &str) -> PathBuf {
    Runfiles::create().unwrap().rlocation(std::env::var(var).unwrap()).unwrap()
}

#[test]
fn a_map_edit_rebuilds_the_level_and_moves_the_player_to_the_new_start() {
    let e = game("map_edit");
    play(&e, &WINNING_RUN[..7]);
    assert!(state(&e).contains("coins 1 (1 left)"), "{}", state(&e));

    let reply = e.load("level", &level_build("LEVEL_ALT_MAP")).unwrap();
    assert_eq!(reply, "reloaded level (generation 1)");
    play(&e, &["step 1"]);
    let state = state(&e);
    // The new map's coins and no walker; the player keeps its count.
    assert!(state.contains("coins 1 (2 left)"), "{state}");
    assert!(state.contains("coin at 20 10") && !state.contains("walker"), "{state}");
    assert!((player(&e, "x") - 4.1).abs() < 0.01, "at the new start:\n{state}");
}

#[test]
fn a_code_only_reload_of_the_level_keeps_it_as_played() {
    let e = game("code_edit");
    play(&e, &WINNING_RUN[..7]);
    let (before, x) = (state(&e), player(&e, "x"));

    assert_eq!(e.load("level", &level_build("LEVEL_ALT_CODE")).unwrap(), "reloaded level (generation 1)");
    assert!(state(&e).contains("coins 1 (1 left)"), "the collected coin must stay collected:\n{}", state(&e));
    assert_eq!(player(&e, "x"), x, "{before}\n---\n{}", state(&e));
}

#[test]
fn input_comes_first_then_the_rules_the_walkers_physics_and_what_they_did() {
    let e = game("schedule");
    assert_eq!(
        e.schedule().unwrap(),
        "input: platformer::steer\n\
         simulate (60 Hz): platformer::play, walkers::walk\n\
         physics::step (60 Hz): physics::integrate_velocities, physics::find_contacts, walkers::meet, physics::solve\n\
         late: platformer::take_hits"
    );
}

