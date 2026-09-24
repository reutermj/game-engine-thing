//! The physics mod in a running engine: a pile that settles, a solver
//! reloaded under it, and a platformer in miniature, all on the lockstep
//! bootstrap so every run is the same.

use std::path::PathBuf;

use engine_loader::engine::Engine;
use physics::{Position, Velocity};
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

    /// Physics gathers colliders with a body and a velocity, with one and
    /// not the other, and with neither, apart: a shelf of either odd kind
    /// has to hold what falls on it.
    #[test]
    fn a_shelf_without_a_velocity_or_without_a_body_holds_what_lands_on_it() {
        let e = game("PILE", "shelves");
        send(&e, "pile", "shelves");
        step(&e, 120);
        // Dropped from 9, they fall to rest on the shelves' tops at 10: a
        // step that stopped (physics failing on a collider it didn't
        // gather) would leave them where they started.
        let resting = positions(&e).into_iter().filter(|&(x, y)| (0.0..40.0).contains(&x) && (y - 9.55).abs() < 0.02);
        assert_eq!(resting.count(), 38, "every body dropped on a shelf rests on it");
    }

    /// Rows the last spatial re-sort of the bodies' tables re-bounded: the
    /// statics' are re-sorted only when they change.
    fn rebounded(e: &Engine) -> usize {
        let w = e.world();
        w.tables()
            .filter(|t| t.components.iter().any(|&c| w.name(c) == "physics::Body"))
            .filter_map(|t| t.spatial.as_ref())
            .map(|s| s.pages.read().unwrap().rebounded)
            .sum()
    }

    /// A pile at rest comes to rest bit for bit, and then the solver
    /// writes no position, so the re-sort after it re-bounds nothing.
    #[test]
    fn a_pile_at_rest_is_not_re_sorted() {
        let e = game("PILE", "at_rest");
        send(&e, "pile", "drop 200");
        let bits = |e: &Engine| {
            let mut all: Vec<_> = e.world().values::<Position>().unwrap().into_iter().map(|(e, p)| (e, p.x.to_bits(), p.y.to_bits())).collect();
            all.sort();
            all
        };
        let mut steps = 0;
        let mut before = bits(&e);
        loop {
            step(&e, 50);
            steps += 50;
            let now = bits(&e);
            if now == before {
                break;
            }
            assert!(steps < 5000, "still moving after {steps} steps");
            before = now;
        }
        step(&e, 1);
        assert_eq!(bits(&e), before);
        assert_eq!(rebounded(&e), 0, "at rest after {steps} steps");
    }

    fn asleep(e: &Engine) -> f32 {
        field(&send(e, "physics", "sleeping"), "asleep")
    }

    /// Steps until `n` bodies are asleep, or panics after `limit` steps.
    fn until_asleep(e: &Engine, n: f32, limit: u32) -> u32 {
        let mut steps = 0;
        while asleep(e) < n {
            assert!(steps < limit, "{} of {n} asleep after {steps} steps", asleep(e));
            step(e, 10);
            steps += 10;
        }
        steps
    }

    #[test]
    fn nothing_sleeps_unless_asked() {
        let e = game("PILE", "no_sleep");
        send(&e, "pile", "drop 200");
        step(&e, 600);
        assert_eq!(asleep(&e), 0.0);
    }

    /// With sleeping on, a settled pile falls asleep and stays put, and a
    /// body dropped on it wakes what it lands on, which settles and sleeps
    /// again without sinking in.
    #[test]
    fn a_sleeping_pile_wakes_where_something_lands_on_it() {
        let e = game("PILE", "sleep");
        send(&e, "pile", "drop 200");
        send(&e, "pile", "sleep 0.05 0.5");
        until_asleep(&e, 200.0, 2000);
        let (at, before) = (positions(&e), send(&e, "physics", "stats"));
        step(&e, 100);
        assert_eq!(positions(&e), at, "asleep, nothing moved");
        assert_eq!(rebounded(&e), 0);
        // Kept as they were, impulses and all, though not looked for.
        assert!(field(&before, "contacts") >= 200.0, "{before}");
        assert_eq!(field(&send(&e, "physics", "stats"), "contacts"), field(&before, "contacts"), "contacts kept");
        let still = e.world().values::<Velocity>().unwrap().into_iter().all(|(_, v)| v == Velocity::default());
        assert!(still, "asleep, every body stopped");

        // Onto the top of the pile, from a row above it.
        send(&e, "pile", "drop 20");
        let mut least = asleep(&e);
        for _ in 0..30 {
            step(&e, 5);
            least = least.min(asleep(&e));
        }
        assert!(least < 150.0, "landing woke the bodies under it: at least {least} of 200 stayed asleep");
        until_asleep(&e, 220.0, 3000);
        settled(&e);
        send(&e, "physics", "wake");
        assert_eq!(asleep(&e), 0.0);
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
        // In the coin (at 8.5) at about 8.45, an overlap while it lasts.
        step(&e, 51);
        assert_eq!(send(&e, "runner", "overlaps"), "1");
        step(&e, 29);
        assert_eq!(send(&e, "runner", "overlaps"), "0", "past the coin");
        // Through the coin once. Each tile stepped onto is a new
        // pair, so a new contact: `Contact` is per pair, and "landed" is
        // `Touching::below` becoming true.
        let end = trace(&e).last().unwrap().0;
        assert!(end > 10.0);
        let events = send(&e, "runner", "events");
        assert!(events.starts_with("coins 1 "), "{events}");
        let tiles_crossed = (end + 0.4).floor() - (2.5f32 + 0.4).floor();
        assert_eq!(field(&events, "landings"), 1.0 + tiles_crossed, "{events}");
    }

    /// Standing, the player presses on the tile under it and nothing else:
    /// a contact entity whose normal, seen from the player, points down.
    #[test]
    fn a_player_standing_still_has_one_pressed_contact_with_the_tile_under_it() {
        let e = game("RUNNER", "contacts");
        step(&e, 20);
        let contacts = send(&e, "runner", "contacts");
        let lines: Vec<Vec<f32>> =
            contacts.lines().map(|l| l.split_whitespace().map(|v| v.parse().unwrap()).collect()).collect();
        // At x 2.5, only the tile across 2..3 is under it.
        assert_eq!(lines.len(), 1, "{contacts}");
        assert_eq!((lines[0][1], lines[0][2]), (0.0, 1.0), "{contacts}");
    }

    /// A system between finding contacts and solving them disables the
    /// player's: nothing holds it up.
    #[test]
    fn a_contact_disabled_before_the_solve_lets_the_player_fall_through() {
        let e = game("RUNNER", "ghost");
        step(&e, 20);
        assert!(trace(&e).last().unwrap().4, "standing first");
        send(&e, "runner", "ghost");
        step(&e, 20);
        let y = trace(&e).last().unwrap().1;
        assert!(y > FLOOR + 1.0, "through the floor: y {y}");
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

