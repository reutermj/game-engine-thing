//! Spatial queries over positions kept in spatial order, run by the ECS
//! harness: what they find, in what order, and that they follow the `Query`
//! rules (rows, changes, conflicts).

use std::sync::Mutex;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{Despawns, Entity, Query, With, World};
use physics::{Collider, Position, Ray, Spatial, Vec2, circle, rect};

engine_api::component! {
    #[derive(Debug, Default, Copy)]
    struct Lava: "test::Lava" {}
}

/// A row of unit tiles centered at x = 0.5 .. 5.5, y = 0.5, the third one
/// lava. Positions are a spatial key, so spawning them sorts them in.
fn world() -> (World, Vec<Entity>) {
    let w = World::new();
    let mut m = w.between_frames(Default::default()).unwrap();
    let tiles: Vec<Entity> = (0..6)
        .map(|x| {
            let at = Position { x: x as f32 + 0.5, y: 0.5 };
            if x == 2 { m.spawn((at, Collider::rect(0.5, 0.5), Lava {})) } else { m.spawn((at, Collider::rect(0.5, 0.5))) }
        })
        .collect();
    drop(m);
    (w, tiles)
}

static SEEN: Mutex<Vec<String>> = Mutex::new(Vec::new());

fn seen() -> Vec<String> {
    std::mem::take(&mut *SEEN.lock().unwrap_or_else(|e| e.into_inner()))
}

fn say(s: impl Into<String>) {
    SEEN.lock().unwrap_or_else(|e| e.into_inner()).push(s.into());
}

fn run<P>(w: &World, s: impl IntoSystem<P>, name: &str) {
    Schedule { systems: vec![s.system(w, name)] }.run_sequential(w);
}

// One test at a time touches SEEN.
static SERIAL: Mutex<()> = Mutex::new(());

#[test]
fn overlapping_finds_what_overlaps_and_the_filter_keeps() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let (w, _) = world();
    fn probe(_: &mut Cx, mut all: Spatial<&Position>, mut lava: Spatial<&Position, With<Lava>>) {
        let mut xs = Vec::new();
        all.overlapping(rect(Vec2::new(2.0, 0.5), Vec2::new(0.9, 0.1)), |_, p| xs.push(p.x));
        say(format!("all {xs:?}"));
        let mut xs = Vec::new();
        lava.overlapping(rect(Vec2::new(3.0, 0.5), Vec2::new(10.0, 10.0)), |_, p| xs.push(p.x));
        say(format!("lava {xs:?}"));
    }
    run(&w, probe, "probe");
    assert_eq!(seen(), ["all [1.5, 2.5]", "lava [2.5]"]);
}

#[test]
fn a_point_on_an_edge_is_at_the_tile_and_an_empty_one_is_not() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let (w, _) = world();
    fn probe(_: &mut Cx, mut tiles: Spatial<()>) {
        say(format!("{} {} {}", tiles.any_at(Vec2::new(1.0, 0.0)), tiles.any_at(Vec2::new(3.2, 0.9)), tiles.any_at(Vec2::new(3.0, 2.0))));
    }
    run(&w, probe, "probe");
    assert_eq!(seen(), ["true true false"]);
}

#[test]
fn a_cast_visits_hits_nearest_first_and_stops_when_asked() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let (w, tiles) = world();
    fn probe(_: &mut Cx, mut tiles: Spatial<&Position>) {
        let ray = Ray::new(Vec2::new(-5.0, 0.5), Vec2::new(1.0, 0.0), 8.0);
        let mut order = Vec::new();
        let none: Option<()> = tiles.cast(ray, |hit, _, p| {
            order.push((p.x, hit.t));
            None
        });
        assert!(none.is_none());
        say(format!("{order:?}"));
        let first = tiles.cast(ray, |hit, row, _| Some((row.entity(), hit.t, hit.normal.x, hit.normal.y == 0.0)));
        say(format!("{first:?}"));
    }
    run(&w, probe, "probe");
    let expect_first = format!("{:?}", Some((tiles[0], 5.0f32, -1.0f32, true)));
    // Through tiles 0..=2; the ray ends at x = 3, the fourth tile's face.
    assert_eq!(seen(), ["[(0.5, 5.0), (1.5, 6.0), (2.5, 7.0), (3.5, 8.0)]".to_string(), expect_first]);
}

#[test]
fn rows_from_a_spatial_query_change_the_world_after_the_system() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let (w, tiles) = world();
    fn blast(_: &mut Cx, mut hit: Spatial<(), (), Despawns>) {
        hit.overlapping(circle(Vec2::new(3.0, 0.5), 0.6), |row, ()| row.despawn());
    }
    run(&w, blast, "blast");
    let alive: Vec<bool> = tiles.iter().map(|&e| w.entities.is_alive(e)).collect();
    assert_eq!(alive, [true, true, false, false, true, true]);
}

#[test]
fn a_move_between_frames_is_where_the_next_query_looks() {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let (w, tiles) = world();
    // Re-sorted as it's written: there's no index to rebuild first.
    w.between_frames(Default::default()).unwrap().insert(tiles[0], Position { x: 50.0, y: 50.0 });
    fn probe(_: &mut Cx, mut all: Spatial<&Position>) {
        let (mut old, mut new) = (Vec::new(), Vec::new());
        all.overlapping(rect(Vec2::new(0.5, 0.5), Vec2::new(0.1, 0.1)), |_, p| old.push(p.x));
        all.overlapping(rect(Vec2::new(50.0, 50.0), Vec2::new(0.1, 0.1)), |_, p| new.push(p.x));
        say(format!("{old:?} {new:?}"));
    }
    run(&w, probe, "probe");
    assert_eq!(seen(), ["[] [50.0]"]);
}

#[test]
#[should_panic(expected = "two queries access physics::Position and one writes it")]
fn a_spatial_query_conflicts_like_the_query_it_is() {
    let (w, _) = world();
    fn both(_: &mut Cx, _s: Spatial<&mut Position>, _q: Query<&Position>) {}
    both.system(&w, "both");
}
