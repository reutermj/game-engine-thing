//! The physics mod in a running engine: a pile that settles, a solver
//! reloaded under it, and a platformer in miniature, all on the lockstep
//! bootstrap so every run is the same.

use std::path::PathBuf;

use engine_loader::engine::Engine;
use physics::Position;
use runfiles::Runfiles;

fn path(var: &str) -> PathBuf {
    Runfiles::create().unwrap().rlocation(std::env::var(var).unwrap()).unwrap()
}

fn game(var: &str, test: &str) -> Box<Engine> {
    let manifest = engine_control::read_manifest(&std::env::var(var).unwrap()).unwrap();
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(test);
    let engine = Engine::new(manifest.bootstrap, dir);
    engine.load_batch(&manifest.mods).expect("loading the game");
    engine
}

fn send(e: &Engine, to: &str, message: &str) -> String {
    e.send(to, message).unwrap_or_else(|err| panic!("{to} {message}: {err}"))
}

fn step(e: &Engine, n: u32) {
    send(e, "lockstep", &format!("step {n}"));
}

/// The number after `key` in `text`.
fn field(text: &str, key: &str) -> f32 {
    let mut words = text.split_whitespace();
    words.find(|w| *w == key).unwrap_or_else(|| panic!("no {key} in {text}"));
    words.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| panic!("no number after {key} in {text}"))
}

fn positions(e: &Engine) -> Vec<(f32, f32)> {
    let mut all: Vec<_> = e.world().values::<Position>().unwrap().into_iter().map(|(_, p)| (p.x, p.y)).collect();
    all.sort_by(|a, b| a.partial_cmp(b).unwrap());
    all
}

mod pile {
    use super::*;

    fn settled(e: &Engine) {
        let stats = send(e, "pile", "stats");
        assert_eq!(field(&stats, "resting"), field(&stats, "bodies"), "{stats}");
        assert!(field(&stats, "deepest") < 0.05, "{stats}");
        assert_eq!(field(&stats, "escaped"), 0.0, "{stats}");
    }

    #[test]
    fn a_pile_of_bodies_comes_to_rest_without_sinking_into_each_other() {
        let e = game("PILE", "settle");
        send(&e, "pile", "drop 300");
        step(&e, 600);
        settled(&e);
    }

    #[test]
    fn the_same_drop_settles_the_same_way() {
        let run = |test| {
            let e = game("PILE", test);
            send(&e, "pile", "drop 200");
            step(&e, 200);
            positions(&e)
        };
        assert_eq!(run("same_a"), run("same_b"));
    }

    #[test]
    fn the_same_pile_settles_the_same_at_any_frame_rate() {
        // Two seconds of frames at each rate: 120 physics steps each.
        let run = |test, frames: u32, fps: u32| {
            let e = game("PILE", test);
            send(&e, "pile", "drop 200");
            send(&e, "lockstep", &format!("step {frames} at {fps}"));
            assert_eq!(field(&send(&e, "physics", "stats"), "steps"), 120.0, "at {fps} fps");
            positions(&e)
        };
        let at_60 = run("rate_60", 120, 60);
        assert_eq!(run("rate_30", 60, 30), at_60);
        assert_eq!(run("rate_20", 40, 20), at_60);
        assert_eq!(run("rate_120", 240, 120), at_60);
    }

    #[test]
    fn the_solver_reloads_under_a_moving_pile() {
        let e = game("PILE", "reload");
        send(&e, "pile", "drop 300");
        step(&e, 60);
        let before = send(&e, "physics", "stats");
        assert!(before.starts_with("build v1"), "{before}");
        assert!(field(&before, "contacts") > 100.0, "{before}");
        let at = positions(&e);

        assert_eq!(e.load("physics", &path("PHYSICS_V2")).unwrap(), "reloaded physics (generation 1)");
        let after = send(&e, "physics", "stats");
        assert!(after.starts_with("build v2"), "{after}");
        assert_eq!(field(&after, "contacts"), field(&before, "contacts"), "the contact cache carried over");
        assert_eq!(positions(&e), at, "the reload moved nothing");

        step(&e, 540);
        settled(&e);
        assert_eq!(field(&send(&e, "physics", "stats"), "steps"), 600.0);
    }
}

mod runner {
    use super::*;

    const FLOOR: f32 = 10.0;

    /// (x, y, vx, vy, below) per frame.
    fn trace(e: &Engine) -> Vec<(f32, f32, f32, f32, bool)> {
        send(e, "runner", "trace")
            .lines()
            .map(|l| {
                let w: Vec<&str> = l.split_whitespace().collect();
                let f = |i: usize| w[i].parse::<f32>().unwrap();
                (f(0), f(1), f(2), f(3), w[4] == "true")
            })
            .collect()
    }

    #[test]
    fn running_across_a_floor_of_tiles_never_snags_on_a_seam() {
        let e = game("RUNNER", "run");
        step(&e, 10);
        send(&e, "runner", "go");
        step(&e, 80);
        let t = trace(&e);
        let standing = FLOOR - 0.475;
        for (frame, &(x, y, vx, _, below)) in t.iter().enumerate().skip(12) {
            assert!(below, "frame {frame}: airborne at x {x} y {y}");
            assert!((vx - 7.0).abs() < 1e-3, "frame {frame}: snagged at x {x}: vx {vx}");
            assert!((y - standing).abs() < 0.02, "frame {frame}: y {y}");
        }
        assert!(t.last().unwrap().0 > 10.0, "{:?}", t.last());
    }

    #[test]
    fn a_sensor_triggers_once_as_the_player_passes_and_a_landing_is_a_contact() {
        let e = game("RUNNER", "events");
        step(&e, 10);
        let landed = send(&e, "runner", "events");
        assert!(landed.starts_with("coins 0 landings 1 "), "landed from its spawn height: {landed}");
        assert_eq!(field(&landed, "triggers"), 0.0, "a static sensor in a static floor is quiet: {landed}");
        assert!(field(&landed, "hardest") > 0.5, "{landed}");
        send(&e, "runner", "go");
        step(&e, 80);
        // Through the coin (at 8.5) once. Each tile stepped onto is a new
        // pair, so a new contact: `Contact` is per pair, and "landed" is
        // `Touching::below` becoming true.
        let end = trace(&e).last().unwrap().0;
        assert!(end > 10.0);
        let events = send(&e, "runner", "events");
        assert!(events.starts_with("coins 1 "), "{events}");
        let tiles_crossed = (end + 0.4).floor() - (2.5f32 + 0.4).floor();
        assert_eq!(field(&events, "landings"), 1.0 + tiles_crossed, "{events}");
    }

    #[test]
    fn a_jump_leaves_the_ground_and_lands_again() {
        let e = game("RUNNER", "jump");
        step(&e, 10);
        send(&e, "runner", "jump");
        step(&e, 60);
        let t = trace(&e);
        let top = t.iter().map(|s| s.1).fold(f32::INFINITY, f32::min);
        // 12 up against 40 of gravity peaks at 1.8.
        assert!((FLOOR - 0.475 - top - 1.8).abs() < 0.15, "peak at {top}");
        assert!(t[11..40].iter().any(|s| !s.4), "left the ground");
        assert!(t.last().unwrap().4, "landed: {:?}", t.last());
    }

    #[test]
    fn a_walker_turns_at_the_ledges_it_finds_with_a_spatial_query() {
        let e = game("RUNNER", "walker");
        let mut directions = Vec::new();
        for _ in 0..40 {
            step(&e, 15);
            let w = send(&e, "runner", "walker");
            let n: Vec<f32> = w.split_whitespace().map(|v| v.parse().unwrap()).collect();
            let (x, y, vx) = (n[0], n[1], n[2]);
            assert!((y - (FLOOR - 0.45)).abs() < 0.05, "fell: {w}");
            assert!((16.9..=30.1).contains(&x), "left its platform: {w}");
            directions.push(vx.signum());
        }
        directions.dedup();
        assert!(directions.len() >= 3, "turned at both ends: {directions:?}");
    }
}

