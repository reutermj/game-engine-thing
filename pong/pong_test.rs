//! Pong played through its text interface, against the real game: the mods
//! in //pong's manifest, loaded into an `Engine` and driven by messages, as
//! an agent plays it over `modctl send`. Lockstep time makes every run of a
//! test the same game.

use std::path::PathBuf;

use engine_loader::engine::Engine;

fn game(test: &str) -> Box<Engine> {
    let manifest = engine_control::read_manifest(&std::env::var("PONG_MANIFEST").unwrap()).unwrap();
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(test);
    let engine = Engine::new(manifest.bootstrap, dir);
    engine.load_batch(&manifest.mods).expect("loading pong");
    engine
}

fn send(e: &Engine, name: &str, message: &str) -> String {
    e.send(name, message).unwrap_or_else(|err| panic!("{name} {message:?}: {err}"))
}

/// Sets the player's paddle moving, then runs `frames` frames.
fn play(e: &Engine, command: &str, frames: u32) {
    send(e, "pong_text", command);
    send(e, "lockstep", &format!("step {frames}"));
}

fn state(e: &Engine) -> String {
    send(e, "pong_text", "state")
}

/// The number after `key` on the `ball` line of `state`.
fn ball(e: &Engine, key: &str) -> f32 {
    let state = state(e);
    let line = state.lines().find(|l| l.starts_with("ball")).expect(&state);
    let mut words = line.split_whitespace();
    words.find(|w| *w == key).expect(line);
    words.next().and_then(|v| v.parse().ok()).expect(line)
}

#[test]
fn an_unreturned_serve_scores_for_the_ai() {
    let e = game("unreturned");
    // The first serve comes to the player, low; standing still at the center
    // misses it.
    play(&e, "stay", 120);
    assert!(state(&e).contains("score you 0 ai 1"), "{}", state(&e));
}

#[test]
fn meeting_the_serve_returns_it() {
    let e = game("returned");
    assert!(ball(&e, "vx") < 0.0, "the first serve goes to the player");
    // The serve lands near y = 16.3 at frame 68; the paddle reaches it in 24.
    play(&e, "down", 24);
    play(&e, "stay", 46);
    assert!(ball(&e, "vx") > 0.0, "the ball should be heading back:\n{}", state(&e));
    assert!(state(&e).contains("score you 0 ai 0"), "{}", state(&e));
}

#[test]
fn the_same_inputs_replay_the_same_game() {
    let script = [("down", 24), ("stay", 46), ("up", 25), ("stay", 400), ("down", 13)];
    let run = |test| {
        let e = game(test);
        for (command, frames) in script {
            play(&e, command, frames);
        }
        (state(&e), send(&e, "pong_text", "show"))
    };
    let first = run("replay_a");
    assert_eq!(first, run("replay_b"));
    assert!(first.0.starts_with("frame 508"), "{}", first.0);
}

#[test]
fn the_court_is_drawn_to_size() {
    let e = game("drawn");
    let show = send(&e, "pong_text", "show");
    let lines: Vec<&str> = show.lines().collect();
    // A header, then the court between two borders.
    assert_eq!(lines.len(), 1 + 1 + 20 + 1, "{show}");
    assert!(lines[1..].iter().all(|l| l.chars().count() == 42), "{show}");
    let court = lines[1..].concat();
    assert_eq!(court.matches('o').count(), 1, "{show}");
    assert_eq!(court.matches('#').count(), 8, "two paddles of four:\n{show}");
}

#[test]
fn unknown_commands_are_refused_with_the_command_list() {
    let e = game("unknown");
    let err = e.send("pong_text", "jump").unwrap_err();
    assert!(err.contains("commands: up | down | stay | show | state"), "{err}");
}
