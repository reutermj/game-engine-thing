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
    // Served again, to the player, who lost the point.
    assert!((0.0..40.0).contains(&ball(&e, "x")) && ball(&e, "vx") < 0.0, "{}", state(&e));
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
    // Faster by the speed-up, and spun by where it met the paddle: the serve
    // came in at 5.6 down.
    assert!((ball(&e, "vx") - 16.0 * 1.05).abs() < 0.01, "{}", state(&e));
    assert!((ball(&e, "vy") - 5.6).abs() > 0.05, "no spin:\n{}", state(&e));
}

/// The returned serve bounces off the bottom wall and the AI's paddle at
/// full speed. A contact found a step early (speculative) used to stop the
/// ball at the surface and bounce it from what speed was left, so at the
/// AI's paddle (frame 200) it stopped dead and crawled along it (get-az6,
/// get-emj.19). Restitution now comes from the speed the ball came in at.
#[test]
fn the_ai_returns_the_ball_at_full_speed() {
    let e = game("ai_returns");
    play(&e, "down", 24);
    play(&e, "stay", 46);
    play(&e, "up", 25);
    // Off the bottom wall, about frame 105: coming down at 5.72, it leaves
    // going up at 5.72.
    play(&e, "stay", 15);
    assert!((ball(&e, "vy") + 5.72).abs() < 0.01, "{}", state(&e));
    play(&e, "stay", 100);
    assert!(ball(&e, "vx") < -16.0, "the AI's return:\n{}", state(&e));
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

/// Physics sleeps by default, and nothing in pong ever should: the ball
/// never goes slower than a serve, and the paddles are kinematic, which
/// don't sleep however still they stand.
#[test]
fn nothing_in_pong_falls_asleep() {
    let e = game("awake");
    for (command, frames) in [("down", 24), ("stay", 46), ("up", 25), ("stay", 400), ("down", 13)] {
        send(&e, "pong_text", command);
        for _ in 0..frames {
            send(&e, "lockstep", "step 1");
            assert_eq!(send(&e, "physics", "sleeping"), "asleep 0", "{}", state(&e));
        }
    }
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

#[test]
fn input_and_the_ai_come_before_the_paddles_move() {
    // The pong retrospective's complaint: in load order, the AI moved on the
    // previous frame's ball.
    let e = game("schedule");
    assert_eq!(
        e.schedule().unwrap(),
        "input: pong::steer\n\
         update: pong_ai::think\n\
         simulate (60 Hz): pong::play\n\
         physics::step (60 Hz): physics::integrate_velocities, physics::find_contacts, physics::solve\n\
         late: pong::rebound"
    );
}
