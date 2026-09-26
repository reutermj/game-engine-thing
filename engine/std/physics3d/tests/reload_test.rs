//! A 3D pile replayed while physics3d (and the scene, and the scheduler)
//! reload under it: every frame with reloads is the pile without them, bit
//! for bit, contacts, their impulses and manifolds, and the settings the
//! step reads from the world included. See //engine/tests:replay.rs for
//! how, and how the reloads are made real.

use engine_loader::engine::Engine;
use physics3d::{AngularVelocity, Body, Collider, ContactPair, Gravity, Impulse, Manifold, Position, Rotation, Static, Tuning, Velocity};
use reload_replay::{Batch, Game, Input, Reloads, assert_invisible, ticks, values};

/// Boxes dropped in a walled box, with the step tuned away from its
/// defaults (so a build that forgot the world's `Tuning` shows), then a
/// sphere spun onto the pile, so something rolls on what has settled.
const PILE: &[Input] = &[
    Input::Send("pile3d", "tune relax=3,warm=nearest"),
    Input::Send("pile3d", "build boxes 64"),
    Input::Frames(60),
    Input::Send("pile3d", "body sphere 0.5 at 0.3 6 0.2 v 1 0 0 w 0 0 -8"),
    Input::Frames(100),
];

fn snapshot(e: &Engine) -> String {
    let mut out = e.world().summary() + "\n";
    // What physics3d and the scene report of their state, not the
    // timings, which are wall time.
    let stats = e.send("physics3d", "stats").unwrap();
    out += stats.split(" us/step").next().unwrap();
    let stages = e.send("physics3d", "stages").unwrap();
    out += &format!("\nfound {}\n", &stages[stages.find("pairs").expect("counts in stages")..]);
    out += &(e.send("pile3d", "stats").unwrap() + "\n");
    values::<Position>(e, &mut out);
    values::<Rotation>(e, &mut out);
    values::<Velocity>(e, &mut out);
    values::<AngularVelocity>(e, &mut out);
    values::<Body>(e, &mut out);
    values::<Collider>(e, &mut out);
    values::<Static>(e, &mut out);
    values::<ContactPair>(e, &mut out);
    values::<Manifold>(e, &mut out);
    values::<Impulse>(e, &mut out);
    values::<Gravity>(e, &mut out);
    values::<Tuning>(e, &mut out);
    ticks(e, &mut out);
    out
}

fn game() -> Game<'static> {
    Game { manifest: "PILE3D", route: PILE, frame: "step 1", snapshot }
}

fn every(every: u32, batch: Batch) -> Reloads {
    Reloads { every, batch, frames: None }
}

#[test]
fn reloading_every_frame_is_invisible() {
    let seen = assert_invisible(&game(), "whole", &[every(1, Batch::Whole)]);
    // Something to see in the frames compared: the pile landed and holds
    // contacts of four points, warm-started, and the sphere came.
    let last = seen.last().unwrap();
    assert!(last.contains("physics3d::ContactPair: Some([("), "no contacts:\n{last}");
    assert!(last.contains("count: 4"), "no contact of four points:\n{last}");
    assert!(seen[30].contains("warm: 1"), "the tuning isn't in the world:\n{}", seen[30]);
    let bodies = |s: &str| s.split("bodies ").nth(1).and_then(|w| w.split_whitespace().next()).map(str::to_string);
    assert_eq!(bodies(last).as_deref(), Some("65"), "{last}");
}

#[test]
fn reloading_one_mod_a_frame_is_invisible() {
    assert_invisible(&game(), "each", &[every(1, Batch::EachInTurn)]);
}

#[test]
fn reloading_now_and_then_is_invisible() {
    assert_invisible(&game(), "mixed", &[every(3, Batch::Mixed(1)), every(7, Batch::Whole), every(20, Batch::Mixed(2))]);
}
