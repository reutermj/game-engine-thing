//! The physics mod in a running engine: a pile that settles, a solver
//! reloaded under it, and a platformer in miniature, all on the lockstep
//! bootstrap so every run is the same.

use std::path::PathBuf;

use engine_loader::engine::Engine;
use physics::{Asleep, Body, ContactPair, DYNAMIC, Overlap, Position, Resting, Touching, Velocity};
use runfiles::Runfiles;

#[path = "pool.rs"]
mod pool;

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

    /// In columns (40 wide) and as a real pile (41 wide): see `SCENES`.
    #[test]
    fn a_pile_of_bodies_comes_to_rest_without_sinking_into_each_other() {
        for (width, n) in SCENES {
            let e = game("PILE", &format!("settle_{width}"));
            send(&e, "pile", &format!("widen {width}"));
            send(&e, "pile", &format!("drop {n}"));
            step(&e, 1200);
            settled(&e);
        }
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
            let mut all: Vec<_> =
                e.world().values::<Position>().unwrap().into_iter().map(|(e, p)| (e, p.x.to_bits(), p.y.to_bits())).collect();
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

    /// When each entity's `component` was last written, by entity: what
    /// change detection sees.
    fn ticks(e: &Engine, component: &str) -> Vec<(u32, u32, u32)> {
        let w = e.world();
        let mut all = Vec::new();
        for t in w.tables() {
            let Some(c) = t.components.iter().position(|&c| w.name(c) == component) else { continue };
            let (rows, pages) = (t.rows.read().unwrap(), t.columns[c].read().unwrap());
            for (p, page) in rows.iter().enumerate() {
                all.extend(page.iter().zip(pages[p].ticks()).map(|(e, &tick)| (e.index, e.generation, tick)));
            }
        }
        all.sort();
        all
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

    /// The piles sleeping is tested on: how wide the box, and how many
    /// bodies. At 40 wide a row holds an odd count, so the pile stands in
    /// columns of alternating circles and boxes that don't touch: a contact
    /// per body, and an island per column. At 41 the columns are all circles
    /// or all boxes, which fall into a real pile, bodies resting on two
    /// below, but only once they're tall enough to: 200 or 300 still stand
    /// in columns, 500 have 1.1 contacts a body, 1000 have 1.5 (2026-09-25; docs/lore).
    const SCENES: [(f32, u32); 2] = [(40.0, 200), (41.0, 1000)];

    /// Physics sleeps by default; the pile, the benchmarks' scene, turns
    /// it off (`Sleep::OFF`) unless asked.
    #[test]
    fn sleeping_is_on_by_default_and_the_pile_turns_it_off() {
        let e = game("PILE", "no_sleep");
        send(&e, "pile", "drop 200");
        step(&e, 600);
        assert_eq!(asleep(&e), 0.0);
        send(&e, "pile", "sleep default");
        until_asleep(&e, 200.0, 2000);
    }

    /// With sleeping on, a settled pile falls asleep and stays put, and a
    /// body dropped on it wakes what it lands on, which settles and sleeps
    /// again without sinking in.
    #[test]
    fn a_sleeping_pile_wakes_where_something_lands_on_it() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("sleep", (width, n), &[]);
            let (at, before) = (positions(&e), send(&e, "physics", "stats"));
            let kept = ["physics::Position", "physics::Velocity", "physics::Manifold", "physics::Impulse"];
            let written = kept.map(|c| ticks(&e, c));
            step(&e, 100);
            assert_eq!(positions(&e), at, "{width} wide: asleep, nothing moved");
            // Nor was anything written: change detection, and the spatial
            // re-sort, leave sleeping bodies and their contacts alone.
            assert!(written == kept.map(|c| ticks(&e, c)), "{width} wide: asleep, nothing written");
            // Kept as they were, impulses and all, though not looked for.
            assert!(field(&before, "contacts") >= all, "{before}");
            assert_eq!(field(&send(&e, "physics", "stats"), "contacts"), field(&before, "contacts"), "contacts kept");
            let still = e.world().values::<Velocity>().unwrap().into_iter().all(|(_, v)| v == Velocity::default());
            assert!(still, "{width} wide: asleep, every body stopped");

            // Onto the top of the pile, from a row above it.
            send(&e, "pile", "drop 20");
            let mut least = asleep(&e);
            for _ in 0..30 {
                step(&e, 5);
                least = least.min(asleep(&e));
            }
            assert!(least < 0.75 * all, "{width} wide: landing woke the bodies under it: at least {least} of {n} stayed asleep");
            until_asleep(&e, all + 20.0, 3000);
            settled(&e);
            send(&e, "physics", "wake");
            assert_eq!(asleep(&e), 0.0);
            assert_eq!(in_sleeping_tables(&e), (0, 0), "woken, out of the sleeping tables");
        }
    }

    /// Sleeping bodies, and contacts at rest, as the world has them: how
    /// many have `Asleep`, and `Resting`.
    fn in_sleeping_tables(e: &Engine) -> (usize, usize) {
        let w = e.world();
        (w.values::<Asleep>().unwrap_or_default().len(), w.values::<Resting>().unwrap_or_default().len())
    }

    /// A pile of `n` in a box `width` wide, asleep, with sleeping on as
    /// `a_sleeping_pile_wakes_..` has it.
    fn asleep_pile(test: &str, scene: (f32, u32), then: &[&str]) -> Box<Engine> {
        asleep_pile_with("PILE", test, scene, then)
    }

    /// `asleep_pile` in the game `var` names: the pile, or the pile with a
    /// pre-solve hook (`PILE_HOOK`).
    fn asleep_pile_with(var: &str, test: &str, (width, n): (f32, u32), then: &[&str]) -> Box<Engine> {
        let e = game(var, &format!("{test}_{width}"));
        send(&e, "pile", &format!("widen {width}"));
        send(&e, "pile", &format!("drop {n}"));
        for message in then {
            send(&e, "pile", message);
        }
        send(&e, "pile", "sleep 0.05 0.5");
        until_asleep(&e, n as f32, 3000);
        e
    }

    /// Asleep, a pile is as deep in itself as the same pile awake at the
    /// same step, and stays so: sleeping stops bodies where they are, and a
    /// body's weight on its sleeping neighbors isn't solved, so it can't
    /// push them into each other either.
    #[test]
    fn a_sleeping_pile_sinks_no_deeper_than_one_awake() {
        for (width, n) in SCENES {
            let e = asleep_pile("deep", (width, n), &[]);
            let deepest = |e: &Engine| field(&send(e, "pile", "stats"), "deepest");
            let at = deepest(&e);
            let steps = field(&send(&e, "physics", "stats"), "steps") as u32;
            step(&e, 300);
            assert_eq!(deepest(&e), at, "{width} wide: asleep, as deep as it fell asleep");
            let awake = game("PILE", &format!("deep_awake_{width}"));
            send(&awake, "pile", &format!("widen {width}"));
            send(&awake, "pile", &format!("drop {n}"));
            step(&awake, steps + 300);
            let (awake, asleep) = (deepest(&awake), deepest(&e));
            assert!(asleep <= awake + 0.005 && asleep < 0.02, "{width} wide: {asleep} deep asleep, {awake} awake");
        }
    }

    /// Asleep is storage: every sleeping body has left the tables the step
    /// walks for awake ones, and every contact between sleeping bodies (or
    /// a sleeping body and a wall) the ones it merges and solves.
    #[test]
    fn sleeping_bodies_and_their_contacts_have_tables_of_their_own() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("tables", (width, n), &["sensing"]);
            let contacts = e.world().values::<ContactPair>().unwrap().len();
            assert_eq!(in_sleeping_tables(&e), (n as usize, contacts));
            // The scenes are what `SCENES` says: columns about a contact a
            // body, a pile half again as many.
            let per_body = contacts as f32 / all;
            assert!(if width == 41.0 { per_body > 1.4 } else { per_body < 1.2 }, "{width} wide: {contacts} contacts");
            // Overlaps between sleeping bodies aren't looked for either, and
            // last while they sleep.
            let overlaps = e.world().values::<Overlap>().unwrap().len();
            assert!(overlaps > 100, "{overlaps} overlaps");
            step(&e, 100);
            assert_eq!(e.world().values::<Overlap>().unwrap().len(), overlaps);
            let w = e.world();
            for t in w.tables() {
                let named = |name: &str| t.components.iter().any(|&c| w.name(c) == name);
                if named("physics::Body") && !named("physics::Asleep") {
                    assert!(t.is_empty(), "an awake body's table has rows");
                }
            }
        }
    }

    /// Physics leaves a sleeping body alone; a game writing its velocity
    /// (a jump, a shove) wakes it, and it moves. So does changing its shape.
    #[test]
    fn a_game_setting_a_sleeping_bodys_velocity_wakes_it() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("kick", (width, n), &[]);
            let before = positions(&e);
            send(&e, "pile", "kick 6 -6");
            step(&e, 1);
            assert!(asleep(&e) < all, "{width} wide: kicked, its island woke");
            step(&e, 10);
            assert_ne!(positions(&e), before, "and it moved");

            // In the step after its island fell asleep, when it's among the
            // rows new to the sleeping tables, as one a game put to sleep
            // would be.
            let e = game("PILE", &format!("kick_newest_{width}"));
            send(&e, "pile", &format!("widen {width}"));
            send(&e, "pile", &format!("drop {n}"));
            send(&e, "pile", "sleep 0.05 0.5");
            let mut steps = 0;
            while asleep(&e) < all {
                assert!(steps < 3000, "{} of {n} asleep after {steps} steps", asleep(&e));
                step(&e, 1);
                steps += 1;
            }
            send(&e, "pile", "kick 6 -6 newest");
            step(&e, 1);
            assert!(asleep(&e) < all, "{width} wide: kicked just asleep, its island woke");
            assert_eq!(in_sleeping_tables(&e).0 as f32, asleep(&e), "{width} wide: the world and physics agree");

            // From a system between finding contacts and solving them, as a
            // pre-solve hook would.
            let e = asleep_pile_with("PILE_HOOK", "kick_between", (width, n), &[]);
            send(&e, "pile_hook", "kick 6 -6");
            step(&e, 2);
            assert!(asleep(&e) < all, "{width} wide: kicked between finding contacts and solving, its island woke");

            let e = asleep_pile("grow", (width, n), &[]);
            send(&e, "pile", "grow 0.6");
            step(&e, 1);
            assert!(asleep(&e) < all, "{width} wide: grown into its neighbors, its island woke");
        }
    }

    /// A game wakes a body by taking its `Asleep` off: the rest of its
    /// island wakes with it, as they rest on each other. Putting it back
    /// puts it to sleep again, as physics takes what a game says.
    #[test]
    fn a_game_removing_a_sleeping_bodys_asleep_wakes_its_island() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("unsleep", (width, n), &[]);
            send(&e, "pile", "unsleep");
            step(&e, 1);
            let awake = asleep(&e);
            assert!(awake < all - 1.0, "{width} wide: its island woke too: {awake} asleep");
            assert_eq!(in_sleeping_tables(&e).0 as f32, awake, "the world and physics agree");
            send(&e, "pile", "resleep");
            step(&e, 1);
            assert_eq!(in_sleeping_tables(&e).0 as f32, asleep(&e), "{width} wide: the world and physics agree");
            // Given `Asleep` and nothing else new: a body the solve doesn't
            // write, spawned steps before.
            send(&e, "pile", "post");
            step(&e, 5);
            let before = asleep(&e);
            send(&e, "pile", "resleep post");
            step(&e, 1);
            assert_eq!(asleep(&e), before + 1.0, "{width} wide: put to sleep");
            assert_eq!(in_sleeping_tables(&e).0 as f32, asleep(&e), "{width} wide: the world and physics agree");
        }
    }

    /// Every body's height (y grows down), not the walls'.
    fn heights(e: &Engine) -> Vec<f32> {
        let w = e.world();
        let bodies: std::collections::HashSet<_> = w.values::<Velocity>().unwrap().into_iter().map(|(e, _)| e).collect();
        w.values::<Position>().unwrap().into_iter().filter(|(e, _)| bodies.contains(e)).map(|(_, p)| p.y).collect()
    }

    /// The lowest body's center.
    fn lowest(e: &Engine) -> f32 {
        heights(e).into_iter().fold(f32::NEG_INFINITY, f32::max)
    }

    fn highest(e: &Engine) -> f32 {
        heights(e).into_iter().fold(f32::INFINITY, f32::min)
    }

    /// Statics are passive to the broadphase like sleeping bodies, so pairs
    /// between them aren't looked for: a floor taken away or moved has to
    /// wake what rests on it some other way.
    #[test]
    fn a_sleeping_pile_falls_when_its_floor_goes() {
        for (width, n) in SCENES {
            let e = asleep_pile("floor_off", (width, n), &[]);
            let at = lowest(&e);
            send(&e, "pile", "floor off");
            step(&e, 1);
            // Woken as the floor is found gone (in `find_contacts`, after
            // gravity), the bottom row falls in that step as the same pile
            // awake does: gravity, and the contacts above it solved.
            let bottom = |e: &Engine, at: f32| {
                let heights: std::collections::HashMap<_, _> =
                    e.world().values::<Position>().unwrap().into_iter().map(|(b, p)| (b, p.y)).collect();
                let mut falling: Vec<f32> =
                    e.world().values::<Velocity>().unwrap().into_iter().filter(|(b, _)| heights[b] > at - 0.3).map(|(_, v)| v.y).collect();
                falling.sort_by(f32::total_cmp);
                falling
            };
            let awake = game("PILE", &format!("floor_off_awake_{width}"));
            send(&awake, "pile", &format!("widen {width}"));
            send(&awake, "pile", &format!("drop {n}"));
            step(&awake, field(&send(&e, "physics", "stats"), "steps") as u32 - 1);
            let awake_at = lowest(&awake);
            send(&awake, "pile", "floor off");
            step(&awake, 1);
            let (asleep, awake) = (bottom(&e, at), bottom(&awake, awake_at));
            let near = asleep.len() == awake.len() && asleep.iter().zip(&awake).all(|(a, b)| (a - b).abs() < 0.15);
            assert!(near && asleep[0] > 0.1, "{width} wide: the bottom row as it falls, asleep before {asleep:?}, awake {awake:?}");
            let w = e.world();
            let there: std::collections::HashSet<_> = w.values::<Position>().unwrap().into_iter().map(|(e, _)| e).collect();
            let dangling =
                w.values::<ContactPair>().unwrap().into_iter().filter(|(_, p)| !there.contains(&p.a) || !there.contains(&p.b)).count();
            assert_eq!(dangling, 0, "{width} wide: no contact with the floor left for the solve to push on");
            step(&e, 29);
            assert!(lowest(&e) > at + 1.0, "{width} wide: fell through where the floor was: {} from {at}", lowest(&e));

            // A static despawned and another spawned in the same step: as
            // many statics as there were, and a floor gone.
            let e = asleep_pile("floor_swap", (width, n), &[]);
            let at = lowest(&e);
            send(&e, "pile", "floor swap");
            step(&e, 30);
            assert!(lowest(&e) > at + 1.0, "{width} wide: fell through where the floor was: {} from {at}", lowest(&e));

            let e = asleep_pile("floor_down", (width, n), &[]);
            let at = lowest(&e);
            send(&e, "pile", "floor 3");
            step(&e, 120);
            assert!((lowest(&e) - (at + 3.0)).abs() < 0.1, "{width} wide: fell onto the floor 3 lower: {} from {at}", lowest(&e));

            // A floor made a body falls, and the pile with it: its contacts no
            // longer rest, since one end moves.
            let e = asleep_pile("floor_falls", (width, n), &[]);
            let at = lowest(&e);
            send(&e, "pile", "floor falls");
            let once = |e: &Engine| {
                let contacts = e.world().values::<ContactPair>().unwrap();
                let mut pairs: Vec<_> = contacts.iter().map(|(_, p)| (p.a, p.b)).collect();
                pairs.sort();
                pairs.dedup();
                assert_eq!(pairs.len(), contacts.len(), "a contact per pair");
            };
            for _ in 0..3 {
                step(&e, 1);
                once(&e);
            }
            step(&e, 27);
            assert!(lowest(&e) > at + 1.0, "{width} wide: fell with the floor: {} from {at}", lowest(&e));
        }
    }

    /// A static a game moves into sleeping bodies wakes them: no contact
    /// joined them before it moved. So does one it spawns there, whose
    /// values are written as it's spawned. Found as contacts are, and out
    /// of the sleeping tables the same step.
    #[test]
    fn a_static_moved_or_spawned_into_a_sleeping_pile_wakes_what_it_meets() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("block", (width, n), &["block 20 -10"]);
            send(&e, "pile", "block 20 28");
            step(&e, 1);
            assert!(asleep(&e) < all, "{width} wide: what the block moved into woke");
            assert!(in_sleeping_tables(&e).0 < n as usize, "{width} wide: and left the sleeping tables in the same step");

            let e = asleep_pile("block_spawned", (width, n), &[]);
            send(&e, "pile", "block 20 28");
            step(&e, 1);
            assert!(asleep(&e) < all, "{width} wide: what the block was spawned into woke");
            assert!(in_sleeping_tables(&e).0 < n as usize, "{width} wide: and left the sleeping tables in the same step");
        }
    }

    /// A kinematic body pushes and is never pushed: moving into sleeping
    /// bodies, it wakes them rather than passing through.
    #[test]
    fn a_kinematic_body_moving_into_a_sleeping_pile_wakes_it() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("pusher", (width, n), &[]);
            // From above the pile's top, down at 3 a second.
            send(&e, "pile", &format!("pusher 20 {} 0 3", highest(&e) - 2.0));
            step(&e, 120);
            assert!(asleep(&e) < all, "{width} wide: what it pressed on woke");
        }
    }

    /// A contact pressed on by a sleeping body that ends (what it rested on
    /// was despawned, or moved away) wakes it.
    #[test]
    fn a_body_despawned_from_under_sleeping_ones_wakes_them() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("despawn", (width, n), &[]);
            send(&e, "pile", "despawn");
            step(&e, 1);
            assert!(asleep(&e) < all - 1.0, "{width} wide: its island woke: {} asleep", asleep(&e));
            step(&e, 120);
            until_asleep(&e, all - 1.0, 2000);
            assert_eq!(in_sleeping_tables(&e).0, n as usize - 1);

            // From a system between finding contacts and solving them.
            let e = asleep_pile_with("PILE_HOOK", "despawn_between", (width, n), &[]);
            send(&e, "pile_hook", "despawn");
            step(&e, 2);
            assert!(
                asleep(&e) < all - 1.0,
                "{width} wide: despawned between finding contacts and solving, its island woke: {} asleep",
                asleep(&e)
            );
        }
    }

    /// pile.rs's `NAP`: where `despawn nap` spawns its sleeping body.
    const NAP: f32 = -5.0;

    /// A game may put a body to sleep itself (here spawning it with
    /// `Asleep`, above the box), which physics takes as it is: in the same
    /// step as a sleeping body is despawned, whose island wakes all the
    /// same, though there are as many bodies asleep as before.
    #[test]
    fn a_body_put_to_sleep_by_a_game_as_another_is_despawned() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("nap", (width, n), &[]);
            send(&e, "pile", "despawn nap");
            step(&e, 1);
            assert!(asleep(&e) < all - 1.0, "{width} wide: the despawned body's island woke: {} asleep", asleep(&e));
            assert_eq!(in_sleeping_tables(&e).0 as f32, asleep(&e), "the world and physics agree");
            let napping = |e: &Engine| {
                let w = e.world();
                let asleep: std::collections::HashSet<_> = w.values::<Asleep>().unwrap().into_iter().map(|(e, _)| e).collect();
                w.values::<Position>().unwrap().into_iter().filter(|(e, p)| asleep.contains(e) && p.y == NAP).count()
            };
            assert_eq!(napping(&e), 1, "{width} wide: spawned asleep, and kept so");
            // Asleep before it's a body, then made one: asleep too.
            send(&e, "pile", "nap later");
            step(&e, 1);
            send(&e, "pile", "nap body");
            step(&e, 1);
            assert_eq!(in_sleeping_tables(&e).0 as f32, asleep(&e), "the world and physics agree");
            step(&e, 60);
            assert_eq!(napping(&e), 2, "{width} wide: asleep in the air, where they were spawned");

            // Woken by a game (its velocity written, to nothing), one falls
            // in that step, as a body awake at rest there would: gravity,
            // which the step gave awake bodies before it woke.
            send(&e, "pile", "kick 0 0 naps");
            step(&e, 1);
            let at: std::collections::HashMap<_, _> = e.world().values::<Position>().unwrap().into_iter().collect();
            let near_nap = |b: &engine_ecs::Entity| at.get(b).is_some_and(|p: &Position| (p.y - NAP).abs() < 0.1);
            let fell: Vec<_> =
                e.world().values::<Velocity>().unwrap().into_iter().filter(|(b, _)| near_nap(b)).map(|(b, v)| (v.y, at[&b].y)).collect();
            assert!(
                fell.len() == 2 && fell.iter().all(|(vy, y)| *vy > 0.3 && *y > NAP),
                "{width} wide: woken, they fell in the step they woke: {fell:?}"
            );
        }
    }

    /// A shelf that starts moving under bodies asleep on it wakes them as
    /// it's found moving (`find_contacts`), and they move with it in the
    /// same step: out of their sleeping tables before the solve, their
    /// resting contacts solved again. Waking only from the next step, the
    /// shelf rose into them for a step, as they stood immovable.
    #[test]
    fn bodies_asleep_on_a_shelf_that_starts_moving_rise_with_it_in_the_same_step() {
        let e = game("PILE", "lift");
        send(&e, "pile", "shelves");
        send(&e, "pile", "sleep 0.05 0.5");
        until_asleep(&e, 38.0, 2000);
        let on_shelf = |e: &Engine| {
            let bodies: std::collections::HashSet<_> =
                e.world().values::<Body>().unwrap().into_iter().filter(|(_, b)| b.kind == DYNAMIC).map(|(e, _)| e).collect();
            let mut all: Vec<f32> = e
                .world()
                .values::<Position>()
                .unwrap()
                .into_iter()
                .filter(|(en, p)| bodies.contains(en) && p.x < 20.0)
                .map(|(_, p)| p.y)
                .collect();
            all.sort_by(f32::total_cmp);
            all
        };
        let before = on_shelf(&e);
        assert_eq!(before.len(), 19, "a row on the kinematic shelf");
        // Up at 3 a second: 0.05 a step.
        send(&e, "pile", "lift 3");
        step(&e, 1);
        let after = on_shelf(&e);
        for (b, a) in before.iter().zip(&after) {
            assert!(b - a > 0.04, "risen with the shelf in the step it woke: from {b} to {a}");
        }
    }

    /// A sleeping body's `Touching` is as it fell asleep, not reset: its
    /// contacts aren't solved, but it still stands on what it stood on.
    #[test]
    fn touching_is_kept_while_asleep() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("touching", (width, n), &["touching"]);
            step(&e, 100);
            let below = e.world().values::<Touching>().unwrap().into_iter().filter(|(_, t)| t.below).count();
            // Every body but a few wedged between others by their sides.
            assert!(below as f32 > 0.9 * all, "{width} wide: {below} of {n} stand on something");
        }
    }

    /// Who sleeps is in the world, so a new build of physics finds them
    /// asleep, and takes none of its own writes for a game's.
    #[test]
    fn a_reload_keeps_a_sleeping_pile_asleep() {
        for (width, n) in SCENES {
            let all = n as f32;
            let e = asleep_pile("reload_asleep", (width, n), &[]);
            let written = ticks(&e, "physics::Position");
            assert_eq!(e.load("physics", &path("PHYSICS_V2")).unwrap(), "reloaded physics (generation 1)");
            assert_eq!(asleep(&e), all, "asleep as the new build loads");
            step(&e, 100);
            assert_eq!(asleep(&e), all);
            assert!(ticks(&e, "physics::Position") == written, "nothing moved or was written");
        }
    }

    /// How long each awake body has been still goes to the new build too,
    /// so a pile due to fall asleep does so the same step with a reload in
    /// between as without. Rebuilt from the world, the time restarted, and
    /// the pile slept `Sleep::time` late (2026-09-25).
    #[test]
    fn a_reload_keeps_how_long_awake_bodies_have_been_still() {
        let pile = |test| {
            let e = game("PILE", test);
            send(&e, "pile", "drop 200");
            send(&e, "pile", "sleep 0.05 0.5");
            e
        };
        let e = pile("still_a");
        let mut frames = 0;
        while asleep(&e) == 0.0 {
            assert!(frames < 3000, "never asleep");
            step(&e, 1);
            frames += 1;
        }
        let (slept, at) = (asleep(&e), positions(&e));
        // A third of `Sleep::time` before: every body that falls asleep
        // then has been still for twenty steps.
        let e = pile("still_b");
        step(&e, frames - 10);
        assert_eq!(e.load("physics", &path("PHYSICS_V2")).unwrap(), "reloaded physics (generation 1)");
        step(&e, 9);
        assert_eq!(asleep(&e), 0.0);
        step(&e, 1);
        assert_eq!(asleep(&e), slept, "asleep at step {frames} without the reload");
        assert_eq!(positions(&e), at);
    }

    /// The shelves (a body with no velocity, a velocity with no body) are
    /// on the broadphase's awake side, as colliders that aren't statics, but
    /// don't move: their contacts with bodies asleep on them rest too, found
    /// by the broadphase each step and left alone.
    #[test]
    fn bodies_asleep_on_shelves_keep_their_contacts_as_they_are() {
        let e = game("PILE", "shelves_asleep");
        // Sensing too, so every pair is also tested for an overlap, and a
        // resting contact is found again unless it's left out as one.
        send(&e, "pile", "shelves sensing");
        send(&e, "pile", "sleep 0.05 0.5");
        until_asleep(&e, 38.0, 2000);
        let contacts = |e: &Engine| {
            let mut all: Vec<_> = e.world().values::<ContactPair>().unwrap().into_iter().map(|(_, p)| (p.a, p.b)).collect();
            all.sort();
            all
        };
        let before = contacts(&e);
        let mut once = before.clone();
        once.dedup();
        assert_eq!(once.len(), before.len(), "a contact per pair");
        step(&e, 100);
        assert_eq!(contacts(&e), before, "the same contacts");
        assert_eq!(in_sleeping_tables(&e), (38, before.len()));
    }

    #[test]
    fn turning_sleeping_off_wakes_everything() {
        for (width, n) in SCENES {
            let e = asleep_pile("sleep_off", (width, n), &[]);
            send(&e, "pile", "sleep off");
            step(&e, 1);
            assert_eq!(asleep(&e), 0.0);
            assert_eq!(in_sleeping_tables(&e), (0, 0));
        }
    }

    /// Lockstep with sleeping on: the same commands at the same steps end
    /// the same, bit for bit, falling asleep, woken by each kind of thing
    /// and asleep again, at 60 frames a second or at 30.
    #[test]
    fn a_sleeping_pile_replays_the_same() {
        let run = |test: &str, fps: u32| {
            let e = game("PILE", test);
            // Physics steps, as frames at `fps`.
            let steps = |e: &Engine, n: u32| send(e, "lockstep", &format!("step {} at {fps}", n * fps / 60));
            send(&e, "pile", "widen 41");
            send(&e, "pile", "drop 1000");
            send(&e, "pile", "sleep 0.05 0.5");
            let mut seen = Vec::new();
            let mut look = |e: &Engine| {
                let mut bits: Vec<_> =
                    e.world().values::<Position>().unwrap().into_iter().map(|(en, p)| (en, p.x.to_bits(), p.y.to_bits())).collect();
                bits.sort_unstable();
                seen.push((bits, asleep(e).to_bits(), in_sleeping_tables(e)));
            };
            steps(&e, 300);
            for then in ["drop 20", "kick 3 -3", "block 20 26", "floor 1", "despawn", "unsleep"] {
                steps(&e, 300);
                look(&e);
                send(&e, "pile", then);
            }
            steps(&e, 300);
            look(&e);
            seen
        };
        let once = run("replay_a", 60);
        // Asleep, woken, and asleep again: sleeping had a part in it.
        let counts: Vec<f32> = once.iter().map(|s| f32::from_bits(s.1)).collect();
        assert!(
            counts.iter().filter(|&&c| c >= 1000.0).count() >= 2 && counts.iter().any(|&c| c < 1000.0),
            "asleep at each look: {counts:?}"
        );
        assert!(once == run("replay_b", 60), "replayed, the same");
        assert!(once == run("replay_30", 30), "at 30 frames a second, the same");
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
        let lines: Vec<Vec<f32>> = contacts.lines().map(|l| l.split_whitespace().map(|v| v.parse().unwrap()).collect()).collect();
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

/// Host threads running physics's tasks: its parallel stages, on threads
/// that aren't the mod's (docs/architecture/physics.md, "Parallelism").
mod threads {
    use std::path::Path;
    use std::sync::Arc;

    use engine_ecs::{Executor, Scoped};

    use super::*;

    /// How many of the staged builds under `dir` the process has mapped:
    /// each load stages a copy of its library there.
    fn images(dir: &Path) -> usize {
        let maps = std::fs::read_to_string("/proc/self/maps").unwrap();
        let dir = dir.to_string_lossy();
        let mut paths: Vec<&str> =
            maps.lines().filter_map(|l| l.splitn(6, ' ').nth(5)).map(str::trim_start).filter(|p| p.starts_with(&*dir)).collect();
        paths.sort_unstable();
        paths.dedup();
        paths.len()
    }

    /// Threads that ran a build's tasks don't keep it mapped once it's
    /// reloaded, nor after its engine is dropped, kept between runs (a
    /// pool) or spawned for each: the same builds are mapped at each point
    /// as with no threads at all. Tasks run only inside the system that
    /// made them, and physics's leave nothing on a thread (a thread-local
    /// with a destructor would keep the build mapped until the thread
    /// exits: docs/lore/a-mod-that-spawns-a-thread-is-never-unmapped.md).
    #[test]
    fn threads_that_ran_a_builds_tasks_do_not_keep_it_mapped() {
        let executors: [(&str, Option<Arc<dyn Executor>>); 3] =
            [("none", None), ("spawned", Some(Arc::new(Scoped(4)))), ("kept", Some(Arc::new(super::pool::Pool::new(4))))];
        let mut seen = Vec::new();
        for (name, executor) in executors {
            let test = format!("images_{name}");
            let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(&test);
            let e = game("PILE", &test);
            e.world().set_executor(executor.clone());
            send(&e, "pile", "drop 400");
            step(&e, 30);
            let loaded = images(&dir);
            assert_eq!(e.load("physics", &path("PHYSICS_V2")).unwrap(), "reloaded physics (generation 1)");
            step(&e, 30);
            let reloaded = images(&dir);
            drop(e);
            // The threads are still there, kept or not.
            seen.push((name, loaded, reloaded, images(&dir)));
            drop(executor);
        }
        eprintln!("builds mapped (loaded, reloaded, engine dropped): {seen:?}");
        let (_, loaded, reloaded, dropped) = seen[0];
        assert_eq!(dropped, 0, "with no threads, nothing is left mapped");
        for s in &seen[1..] {
            assert_eq!((s.1, s.2, s.3), (loaded, reloaded, dropped), "{}: {seen:?}", s.0);
        }
    }

    /// The pile at four threads is where it is at one, bit for bit, over
    /// its fall and settling, with every body sensing the others (an
    /// `Overlap` each) and asking what it touches: `:tax -- parallel`
    /// checks the plain pile at 10 000 bodies, this in every test run.
    #[test]
    fn a_pile_on_four_threads_lands_where_it_does_on_one() {
        let run = |executor: Option<Arc<dyn Executor>>, test: &str| {
            let e = game("PILE", test);
            e.world().set_executor(executor);
            send(&e, "pile", "widen 41");
            send(&e, "pile", "drop 600");
            send(&e, "pile", "sensing");
            send(&e, "pile", "touching");
            step(&e, 200);
            let w = e.world();
            let mut all: Vec<_> = w.values::<Position>().unwrap().into_iter().map(|(en, p)| (en, p.x.to_bits(), p.y.to_bits())).collect();
            all.sort_unstable();
            let mut contacts: Vec<_> = w.values::<ContactPair>().unwrap().into_iter().map(|(en, p)| (en, p.a, p.b)).collect();
            contacts.sort_unstable();
            let mut overlaps: Vec<_> = w.values::<Overlap>().unwrap().into_iter().map(|(en, o)| (en, o.a, o.b)).collect();
            overlaps.sort_unstable();
            let mut touching: Vec<_> =
                w.values::<Touching>().unwrap().into_iter().map(|(en, t)| (en, [t.below, t.above, t.left, t.right])).collect();
            touching.sort_unstable();
            (all, contacts, overlaps, touching)
        };
        let one = run(None, "one_thread");
        let four = run(Some(Arc::new(super::pool::Pool::new(4))), "four_threads");
        assert!(one.1.len() > 700, "{} contacts: a pile", one.1.len());
        assert!(one.2.len() > 500 && one.3.iter().filter(|t| t.1[0]).count() > 400, "overlaps, and bodies standing on something");
        assert!(one.0 == four.0 && one.1 == four.1, "the same bodies where they were, and the same contacts, entities and all");
        assert!(one.2 == four.2 && one.3 == four.3, "the same overlaps, and sides touched");
    }
}
