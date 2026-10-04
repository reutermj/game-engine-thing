//! The extract on worlds of its own, run by the ECS harness: what each
//! component becomes in the draw list, the runs, the labels' boxes and
//! classes, and the view.

use std::f32::consts::FRAC_PI_2;
use std::sync::Mutex;

use engine_api::{Entity, Make, Query, See};
use engine_ecs::World;
use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use present::{Camera, Circle, Colour, DrawList, Item, Label, Labelled, Line, Look, Place, Rect, Run, Shape, TEXT_ADVANCE, Text, TextItem};

#[path = "../extract.rs"]
mod extract;

/// What a frame's list held, copied out by a reader after the extract.
#[derive(Debug)]
struct Seen {
    items: Vec<Item>,
    entities: Vec<Entity>,
    runs: Vec<Run>,
    texts: Vec<TextItem>,
    labels: Vec<Labelled>,
    names: Vec<String>,
    view: Option<Camera>,
    cameras: u32,
    check: Result<(), String>,
}

static SCRATCH: Mutex<Option<extract::Scratch>> = Mutex::new(None);
static SEEN: Mutex<Vec<Seen>> = Mutex::new(Vec::new());
// One test at a time touches the statics.
static SERIAL: Mutex<()> = Mutex::new(());

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn extract_system(
    _: &mut Cx,
    (mut rects, mut circles, mut lines): (Query<(&Place, &Rect, &Look)>, Query<(&Place, &Circle, &Look)>, Query<(&Place, &Line, &Look)>),
    mut texts: Query<(&Place, &Text, &Look)>,
    mut labels: Query<(&Place, &Label)>,
    mut cameras: Query<&Camera>,
    mut out: Make<DrawList>,
) {
    let mut scratch = lock(&SCRATCH);
    let scratch = scratch.get_or_insert_with(Default::default);
    extract::extract(&mut out, scratch, (&mut rects, &mut circles, &mut lines), &mut texts, &mut labels, &mut cameras);
}

fn read(_: &mut Cx, list: See<DrawList>) {
    lock(&SEEN).push(Seen {
        items: list.items.clone(),
        entities: list.entities.clone(),
        runs: list.runs.clone(),
        texts: list.texts.clone(),
        labels: list.labels.clone(),
        names: list.names.clone(),
        view: list.view,
        cameras: list.cameras,
        check: list.check(),
    });
}

/// Runs `frames` frames of the extract and a reader over `w`.
fn frames(w: &World, frames: usize) -> Vec<Seen> {
    let schedule = Schedule { systems: vec![extract_system.system(w, "present::extract"), read.system(w, "reader")] };
    for _ in 0..frames {
        schedule.run_sequential(w);
    }
    std::mem::take(&mut *lock(&SEEN))
}

fn frame(w: &World) -> Seen {
    let mut seen = frames(w, 1);
    let seen = seen.pop().unwrap();
    assert_eq!(seen.check, Ok(()), "the list breaks its promises");
    seen
}

fn spawn<B: engine_ecs::Bundle>(w: &World, bundle: B) -> Entity {
    w.between_frames(Default::default()).unwrap().spawn(bundle)
}

const RED: Colour = Colour::rgb(0xff0000);

fn close(a: [f32; 2], b: [f32; 2]) -> bool {
    (a[0] - b[0]).abs() < 1e-5 && (a[1] - b[1]).abs() < 1e-5
}

#[test]
fn each_shape_becomes_an_item() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let look = Look::colour(RED).layer(3).material(7);
    let r = spawn(&w, (Place::at(1.0, 2.0), Rect { w: 4.0, h: 2.0 }, look));
    let c = spawn(&w, (Place::at(5.0, 6.0), Circle { radius: 0.5 }, look));
    let l = spawn(&w, (Place::at(10.0, 0.0), Line { dx: 2.0, dy: 0.0, width: 0.1 }, look));
    let seen = frame(&w);
    let item = |pos, size, rot, shape| Item { pos, size, rot, colour: RED, shape, layer: 3, material: 7 };
    assert_eq!(
        seen.items,
        [
            item([1.0, 2.0], [4.0, 2.0], [1.0, 0.0], Shape::Rect),
            item([5.0, 6.0], [1.0, 1.0], [1.0, 0.0], Shape::Circle),
            item([11.0, 0.0], [2.0, 0.1], [1.0, 0.0], Shape::Line),
        ]
    );
    assert_eq!(seen.entities, [r, c, l]);
    assert_eq!(seen.runs, [Run { layer: 3, material: 7, start: 0, len: 3 }]);
}

#[test]
fn a_turned_place_turns_its_rect_and_its_line() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let at = Place::at(1.0, 1.0).turned(FRAC_PI_2);
    spawn(&w, (at, Rect { w: 4.0, h: 2.0 }, Look::default()));
    spawn(&w, (at, Line { dx: 2.0, dy: 0.0, width: 0.5 }, Look::default()));
    let seen = frame(&w);
    assert!(close(seen.items[0].rot, [0.0, 1.0]), "{:?}", seen.items[0]);
    assert_eq!(seen.items[0].pos, [1.0, 1.0]);
    // The line points down from its place (y grows downward), its centre
    // halfway along.
    assert!(close(seen.items[1].pos, [1.0, 2.0]), "{:?}", seen.items[1]);
    assert!(close(seen.items[1].rot, [0.0, 1.0]));
    assert!(close(seen.items[1].size, [2.0, 0.5]));
}

#[test]
fn a_line_points_where_it_goes_not_where_its_place_faces() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    spawn(&w, (Place::at(0.0, 0.0), Line { dx: 0.0, dy: -3.0, width: 0.5 }, Look::default()));
    let seen = frame(&w);
    assert!(close(seen.items[0].rot, [0.0, -1.0]), "{:?}", seen.items[0]);
    assert!(close(seen.items[0].pos, [0.0, -1.5]));
    assert!(close(seen.items[0].size, [3.0, 0.5]));
}

#[test]
fn a_line_of_no_length_keeps_its_places_turn() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    spawn(&w, (Place::at(0.0, 0.0).turned(FRAC_PI_2), Line { dx: 0.0, dy: 0.0, width: 1.0 }, Look::default()));
    let seen = frame(&w);
    assert!(close(seen.items[0].rot, [0.0, 1.0]), "{:?}", seen.items[0]);
    assert_eq!(seen.items[0].size, [0.0, 1.0]);
}

#[test]
fn items_come_in_runs_by_layer_then_material_keeping_world_order_within_one() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    // (layer, material), spawned out of order.
    let keys = [(1, 0), (0, 2), (1, 0), (0, 1), (0, 2), (-1, 9)];
    let entities: Vec<Entity> = keys
        .iter()
        .map(|&(layer, material)| spawn(&w, (Place::default(), Rect::default(), Look::default().layer(layer).material(material))))
        .collect();
    let seen = frame(&w);
    let run = |layer, material, start, len| Run { layer, material, start, len };
    assert_eq!(seen.runs, [run(-1, 9, 0, 1), run(0, 1, 1, 1), run(0, 2, 2, 2), run(1, 0, 4, 2)]);
    // Each item still beside its entity, and equal keys in spawn order.
    let by: Vec<Entity> = [5, 3, 1, 4, 0, 2].iter().map(|&i| entities[i]).collect();
    assert_eq!(seen.entities, by);
    for (item, e) in seen.items.iter().zip(&seen.entities) {
        let i = entities.iter().position(|x| x == e).unwrap();
        assert_eq!((item.layer, item.material), keys[i]);
    }
}

#[test]
fn texts_carry_their_text_and_are_sorted_by_layer() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let text = |s: &str| Text { text: s.into(), size: 2.0, anchor: [0.5, 0.0] };
    let a = spawn(&w, (Place::at(1.0, 1.0), text("top"), Look::colour(RED).layer(2)));
    let b = spawn(&w, (Place::at(2.0, 2.0), text("under"), Look::default().layer(0)));
    let c = spawn(&w, (Place::at(3.0, 3.0), text("also top"), Look::default().layer(2)));
    let seen = frame(&w);
    let order: Vec<(Entity, &str)> = seen.texts.iter().map(|t| (t.entity, t.text.as_str())).collect();
    assert_eq!(order, [(b, "under"), (a, "top"), (c, "also top")]);
    assert_eq!(
        seen.texts[1],
        TextItem { entity: a, text: "top".into(), pos: [1.0, 1.0], size: 2.0, anchor: [0.5, 0.0], colour: RED, layer: 2 }
    );
    assert!(seen.items.is_empty());
}

#[test]
fn a_label_covers_all_its_entitys_shapes() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    // A 4 by 2 rect at (0, 0), and a circle of radius 3 on the same place:
    // the circle's 6 by 6 box covers the rect's.
    let e =
        spawn(&w, (Place::at(0.0, 0.0), Rect { w: 4.0, h: 2.0 }, Circle { radius: 3.0 }, Look::default(), Label::named("ship", "mine")));
    // A rect 10 by 2 turned a quarter: 2 by 10.
    let t = spawn(&w, (Place::at(20.0, 0.0).turned(FRAC_PI_2), Rect { w: 10.0, h: 2.0 }, Look::default(), Label::what("pillar")));
    let seen = frame(&w);
    assert_eq!(seen.labels[0], Labelled { entity: e, what: 0, name: "mine".into(), pos: [0.0, 0.0], size: [6.0, 6.0] });
    assert_eq!(seen.labels[1].entity, t);
    assert!(close(seen.labels[1].size, [2.0, 10.0]), "{:?}", seen.labels[1]);
    assert!(close(seen.labels[1].pos, [20.0, 0.0]));
}

#[test]
fn a_rect_and_a_line_on_one_entity_join_their_boxes() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    // A unit square at (0, 0) and a line from it to (10, 0), 1 thick.
    spawn(&w, (Place::at(0.0, 0.0), Rect { w: 1.0, h: 1.0 }, Line { dx: 10.0, dy: 0.0, width: 1.0 }, Look::default(), Label::what("flag")));
    let seen = frame(&w);
    assert!(close(seen.labels[0].pos, [4.75, 0.0]), "{:?}", seen.labels[0]);
    assert!(close(seen.labels[0].size, [10.5, 1.0]));
}

#[test]
fn a_labelled_text_is_measured_by_its_advance_and_anchor() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    // "12" at size 2 is 2 * 2 * TEXT_ADVANCE wide, its top centre on (10, 4).
    spawn(&w, (Place::at(10.0, 4.0), Text { text: "12".into(), size: 2.0, anchor: [0.5, 0.0] }, Look::default(), Label::what("score")));
    let seen = frame(&w);
    let width = 4.0 * TEXT_ADVANCE;
    assert!(close(seen.labels[0].size, [width, 2.0]), "{:?}", seen.labels[0]);
    assert!(close(seen.labels[0].pos, [10.0, 5.0]));
}

#[test]
fn a_label_with_nothing_drawn_is_a_point_and_classes_are_shared() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    spawn(&w, (Place::at(1.0, 2.0), Label::what("coin")));
    spawn(&w, (Place::at(3.0, 4.0), Label::what("spike")));
    spawn(&w, (Place::at(5.0, 6.0), Label::named("coin", "last")));
    let seen = frame(&w);
    assert_eq!(seen.names, ["coin", "spike"]);
    let classes: Vec<u32> = seen.labels.iter().map(|l| l.what).collect();
    assert_eq!(classes, [0, 1, 0]);
    assert_eq!((seen.labels[0].pos, seen.labels[0].size), ([1.0, 2.0], [0.0, 0.0]));
    assert_eq!(seen.labels[2].name, "last");
}

#[test]
fn the_view_is_the_first_camera() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let camera = Camera { x: -1.0, y: -2.0, w: 42.0, h: 24.0, clear: Colour::BLACK };
    spawn(&w, (camera,));
    spawn(&w, (Camera { x: 100.0, ..camera },));
    spawn(&w, (Place::at(500.0, 500.0), Rect { w: 1.0, h: 1.0 }, Look::default()));
    let seen = frame(&w);
    assert_eq!(seen.view, Some(camera));
    assert_eq!(seen.cameras, 2);
}

#[test]
fn with_no_camera_the_view_fits_everything_drawn_and_labelled() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    spawn(&w, (Place::at(0.0, 0.0), Rect { w: 2.0, h: 2.0 }, Look::default()));
    spawn(&w, (Place::at(10.0, 5.0), Circle { radius: 1.0 }, Look::default()));
    spawn(&w, (Place::at(-3.0, 20.0), Label::what("marker")));
    spawn(&w, (Place::at(4.0, -6.0), Text { text: "ab".into(), size: 1.0, anchor: [0.0, 0.0] }, Look::default()));
    let seen = frame(&w);
    let view = seen.view.unwrap();
    assert_eq!((seen.cameras, view.x, view.y), (0, -3.0, -6.0));
    assert!(close([view.w, view.h], [14.0, 26.0]), "{view:?}");
}

#[test]
fn an_empty_world_has_no_view() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let seen = frame(&w);
    assert_eq!((seen.view, seen.items.len(), seen.runs.len()), (None, 0, 0));
}

#[test]
fn each_frame_is_rebuilt_whole() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let e = spawn(&w, (Place::at(0.0, 0.0), Rect { w: 1.0, h: 1.0 }, Look::default().layer(1), Label::what("a")));
    spawn(&w, (Place::at(1.0, 0.0), Rect { w: 1.0, h: 1.0 }, Look::default(), Label::what("b")));
    let seen = frames(&w, 3);
    for s in &seen {
        assert_eq!((s.items.len(), s.runs.len(), s.labels.len()), (2, 2, 2));
        assert_eq!(s.names, ["a", "b"]);
        assert_eq!(s.check, Ok(()));
    }
    w.between_frames(Default::default()).unwrap().insert(e, Place::at(5.0, 5.0));
    let seen = frame(&w);
    assert_eq!(seen.items[1].pos, [5.0, 5.0]);
}

/// Pong as presentation.md's example describes it: the 40 by 20 court in
/// its own cells, a band above it for the score.
#[test]
fn pong_in_the_vocabulary() {
    let _serial = lock(&SERIAL);
    let w = World::new();
    let grey = Look::colour(Colour::rgb(0x8c8c96));
    spawn(&w, (Camera { x: -1.0, y: -4.0, w: 42.0, h: 25.0, clear: Colour::rgb(0x030306) },));
    for y in [-0.25, 20.25] {
        spawn(&w, (Place::at(20.0, y), Rect { w: 40.0, h: 0.5 }, grey, Label::what("wall")));
    }
    // The net: under everything else.
    spawn(&w, (Place::at(20.0, 0.0), Line { dx: 0.0, dy: 20.0, width: 0.1 }, Look::colour(Colour::rgb(0x0e0f16)).layer(-1)));
    let left =
        spawn(&w, (Place::at(1.5, 10.0), Rect { w: 1.0, h: 4.0 }, Look::colour(Colour::rgb(0x3cd2f0)), Label::named("paddle", "left")));
    spawn(&w, (Place::at(38.5, 10.0), Rect { w: 1.0, h: 4.0 }, Look::colour(Colour::rgb(0xfa9628)), Label::named("paddle", "right")));
    let ball = spawn(&w, (Place::at(20.0, 10.0), Circle { radius: 0.25 }, Look::default(), Label::what("ball")));
    let score = Text { text: "0 : 0".into(), size: 2.0, anchor: [0.5, 0.5] };
    spawn(&w, (Place::at(20.0, -2.0), score, Look::default(), Label::what("score")));
    let seen = frame(&w);
    // The net its own run below; walls, paddles and ball one run.
    assert_eq!(seen.runs, [Run { layer: -1, material: 0, start: 0, len: 1 }, Run { layer: 0, material: 0, start: 1, len: 5 }]);
    assert_eq!(seen.names, ["wall", "paddle", "ball", "score"]);
    let paddle = seen.labels.iter().find(|l| l.entity == left).unwrap();
    assert_eq!((paddle.name.as_str(), paddle.pos, paddle.size), ("left", [1.5, 10.0], [1.0, 4.0]));
    assert!(seen.labels.iter().any(|l| l.entity == ball && l.size == [0.5, 0.5]));
    assert_eq!(seen.texts[0].text, "0 : 0");
    assert_eq!(seen.view.map(|v| (v.w, v.h)), Some((42.0, 25.0)));
}
