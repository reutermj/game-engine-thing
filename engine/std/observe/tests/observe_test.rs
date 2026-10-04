//! Requests as agents type them, and each view as it's drawn and written.

use engine_api::Entity;
use observe::{Happened, Legend, Observation, Request, SeatView, Seen, Structured, TextView, VectorSchema, View, id};
use play::{Action, ActionState, GameEvent, Metric, Metrics, Playable};

fn parse(r: &str) -> Request {
    Request::parse(r).unwrap_or_else(|e| panic!("{r}: {e}"))
}

#[test]
fn requests_read_as_typed() {
    let text = |cell, region| View::Text { cell, region };
    assert_eq!(parse("text"), Request { view: text([1.0, 1.0], None), seat: None });
    assert_eq!(parse("text cell 2").view, text([2.0, 2.0], None));
    assert_eq!(parse("text cell 2 0.5").view, text([2.0, 0.5], None));
    assert_eq!(parse("text region 0 -1 40 20 cell 2").view, text([2.0, 2.0], Some([0.0, -1.0, 40.0, 20.0])));
    assert_eq!(parse("text cell 2 region 0 0 4 4").view, text([2.0, 2.0], Some([0.0, 0.0, 4.0, 4.0])));
    assert_eq!(parse("json as left"), Request { view: View::Json, seat: Some("left".into()) });
    assert_eq!(parse(" vector ").view, View::Vector);
    assert_eq!(parse("pixels 1008 756 overlay as right").view, View::Pixels { width: 1008, height: 756, overlay: true });
    assert_eq!(parse("pixels 84 84").view, View::Pixels { width: 84, height: 84, overlay: false });
}

#[test]
fn a_bad_request_says_why_and_how() {
    let why = |r: &str| Request::parse(r).unwrap_err();
    assert!(why("").starts_with("no view named. views: text"), "{}", why(""));
    assert!(why("picture").starts_with("can't read `picture`."));
    assert!(why("json please").starts_with("can't read `json please`."));
    assert!(why("text cell 0").starts_with("a cell needs a width and height above 0."));
    assert!(why("text cell -1 1").starts_with("a cell needs a width and height above 0."));
    assert!(why("text cell big").starts_with("`big` isn't a number."));
    assert!(why("text cell inf").starts_with("`inf` isn't a number."));
    assert!(why("text region 0 0 0 4").starts_with("a region needs a width and height above 0."));
    assert!(why("text region 0 0 4 -4").starts_with("a region needs a width and height above 0."));
    assert!(why("text region 0 0 4").starts_with("can't read `region 0 0 4`."));
    assert!(why("pixels 0 10").starts_with("pixels are 1 to 4096 a side, not 0 by 10."));
    assert!(why("pixels 10 4097").starts_with("pixels are 1 to 4096 a side, not 10 by 4097."));
    assert!(why("pixels 10 10 shiny").starts_with("can't read `shiny`."));
    assert!(why("json as").starts_with("`as` takes one seat name, at the end."));
    assert!(why("json as left now").starts_with("`as` takes one seat name, at the end."));
}

#[test]
fn glyphs_are_the_classes_own_letters_where_free() {
    let glyphs = |classes: &[&str]| Legend::assign(classes).into_iter().map(|l| l.glyph).collect::<Vec<_>>().join("");
    assert_eq!(glyphs(&["ball", "paddle", "wall"]), "bpw");
    assert_eq!(glyphs(&["ball", "bat", "bat2"]), "bat");
    assert_eq!(glyphs(&["b", "b2", "bb"]), "b2B");
    assert_eq!(glyphs(&["x", "x"]), "xX");
    assert_eq!(glyphs(&["-", "_"]), "ab");
    let many: Vec<String> = (0..63).map(|i| format!("k{i}")).collect();
    let many: Vec<&str> = many.iter().map(String::as_str).collect();
    assert!(glyphs(&many).ends_with('?'));
    assert!(!glyphs(&many[..62]).contains('?'));
}

fn seen(what: &str, name: &str, x: f32, y: f32, w: f32, h: f32) -> Seen {
    Seen { id: Entity { index: 0, generation: 0 }, what: what.into(), name: name.into(), x, y, w, h, vx: 0.0, vy: 0.0 }
}

#[test]
fn a_box_marks_the_cells_whose_centres_it_covers() {
    // A 1 by 2 paddle lying along the grid: column 1, rows 1 and 2.
    let paddle = seen("paddle", "", 1.5, 2.0, 1.0, 2.0);
    // A ball smaller than a cell: the cell its centre is in.
    let ball = seen("ball", "", 5.2, 1.7, 0.5, 0.5);
    // A wall over the top row, cut at the region's edges.
    let wall = seen("wall", "", 5.0, 0.5, 20.0, 1.0);
    // Outside the region: listed, not drawn.
    let far = seen("ball", "far", 50.0, 50.0, 0.5, 0.5);
    let t = TextView::draw([0.0, 0.0, 8.0, 4.0], [1.0, 1.0], vec![paddle, ball, wall, far]);
    assert_eq!(t.rows, ["wwwwwwww", " p   b  ", " p      ", "        "]);
    assert_eq!(t.entities.len(), 4);
    let legend: Vec<(&str, &str)> = t.legend.iter().map(|l| (l.glyph.as_str(), l.what.as_str())).collect();
    assert_eq!(legend, [("p", "paddle"), ("b", "ball"), ("w", "wall")]);
}

#[test]
fn cells_follow_the_region_and_their_size_and_later_entities_cover_earlier() {
    let under = seen("floor", "", 2.0, 2.0, 4.0, 4.0);
    let over = seen("coin", "", 3.0, 3.0, 2.0, 2.0);
    // Cells 2 by 2 from (0, 0): a 2 by 2 grid.
    let t = TextView::draw([0.0, 0.0, 4.0, 4.0], [2.0, 2.0], vec![under.clone(), over.clone()]);
    assert_eq!(t.rows, ["ff", "fc"]);
    // The same, the region moved a cell right: the floor's right column only.
    let t = TextView::draw([2.0, 0.0, 4.0, 4.0], [2.0, 2.0], vec![under, over]);
    assert_eq!(t.rows, ["f ", "c "]);
    // A region of no whole cells still has one.
    let t = TextView::draw([0.0, 0.0, 0.5, 0.5], [1.0, 1.0], vec![]);
    assert_eq!(t.rows, [" "]);
    let huge = TextView::draw([0.0, 0.0, 1e6, 3.0], [1.0, 1.0], vec![]);
    assert_eq!(huge.rows[0].len(), observe::MAX_CELLS);
}

#[test]
fn the_text_view_reads_with_its_coordinates_legend_and_list() {
    let mut ball = seen("ball", "", 11.25, 1.5, 0.5, 0.5);
    (ball.id, ball.vx, ball.vy) = (Entity { index: 3, generation: 0 }, 16.0, -5.6000004);
    let mut paddle = seen("paddle", "left", 1.5, 1.0, 1.0, 2.0);
    paddle.id = Entity { index: 7, generation: 2 };
    let t = TextView::draw([0.0, 0.0, 12.0, 3.0], [1.0, 1.0], vec![ball, paddle]);
    let expected = "\
column c is x 0 + 1 c, row r is y 0 + 1 r
  0         1
  012345678901
 +------------+
0| p          |
1| p         b|
2|            |
 +------------+
legend: b ball, p paddle
entities:
  3 ball at (11.25, 1.5) size 0.5 x 0.5 moving (16, -5.6)
  7.2 paddle left at (1.5, 1) size 1 x 2 moving (0, 0)
";
    assert_eq!(t.render(), expected);
}

#[test]
fn rows_are_numbered_to_the_widest_and_an_empty_view_says_so() {
    let t = TextView::draw([0.0, 0.0, 2.0, 11.0], [1.0, 1.0], vec![]);
    let r = t.render();
    assert!(r.contains("\n 0|  |\n"), "{r}");
    assert!(r.contains("\n10|  |\n"), "{r}");
    assert!(r.contains("legend: nothing labelled\n"), "{r}");
    assert!(!r.contains("  0         1"), "no tens line under ten columns: {r}");
}

fn pong() -> Playable {
    Playable::new("first to 5")
        .players(&["left", "right"])
        .action(Action::axis("paddle", "moves").held().directions("up", "stay", "down"))
        .action(Action::button("serve", "serves"))
        .event("hit", "a hit")
        .metric(Metric::new("points", "your points", 0.0, 5.0))
        .observe("ball", 1)
        .observe("paddle", 2)
}

#[test]
fn the_vector_schema_is_metrics_then_slots() {
    let names = VectorSchema::of(&pong()).names;
    assert_eq!(names.len(), 2 + 3 * 7);
    assert_eq!(names[..4], ["left.points", "right.points", "ball0.present", "ball0.x"]);
    assert_eq!(names[8], "ball0.h");
    assert_eq!(names[9], "paddle0.present");
    assert_eq!(names[22], "paddle1.h");
}

#[test]
fn vector_values_are_scaled_to_the_view_and_slots_kept_in_name_order() {
    let p = pong();
    let mut ball = seen("ball", "", 20.0, 10.0, 0.5, 0.5);
    (ball.vx, ball.vy) = (16.0, -8.0);
    let right = seen("paddle", "right", 38.5, 5.0, 1.0, 4.0);
    let left = seen("paddle", "left", 1.5, 15.0, 1.0, 4.0);
    let extra = seen("paddle", "zzz", 0.0, 0.0, 1.0, 1.0);
    let metrics = [Metrics { values: vec![2.0] }, Metrics { values: vec![5.0] }];
    let v = VectorSchema::fill(&p, [0.0, 0.0, 40.0, 20.0], &[right, ball, extra, left], &metrics);
    assert_eq!(v.names, VectorSchema::of(&p).names);
    #[rustfmt::skip]
    let expected = [
        0.4, 1.0,
        1.0, 0.5, 0.5, 0.4, -0.4, 0.0125, 0.025,
        1.0, 1.5 / 40.0, 0.75, 0.0, 0.0, 0.025, 0.2,
        1.0, 38.5 / 40.0, 0.25, 0.0, 0.0, 0.025, 0.2,
    ];
    assert_eq!(v.values, expected);
    // With no ball and a seat's metrics missing: zeros and the minimum.
    let v = VectorSchema::fill(&p, [10.0, 0.0, 40.0, 20.0], &[], &metrics[..1]);
    assert_eq!(v.values[..2], [0.4, 0.0]);
    assert!(v.values[2..].iter().all(|&x| x == 0.0));
    let ball = seen("ball", "", 50.0, 10.0, 0.5, 0.5);
    let v = VectorSchema::fill(&p, [10.0, 0.0, 40.0, 20.0], &[ball], &metrics);
    assert_eq!(v.values[3], 1.0, "unclamped, and from the view's left");
}

#[test]
fn a_seat_reports_its_held_actions_and_metrics_by_name() {
    let p = pong();
    let s = SeatView::of(&p, 1, &ActionState { held: vec![-1.0, 0.0] }, &Metrics { values: vec![] });
    assert_eq!(s.name, "right");
    assert_eq!(s.actions, [("paddle".to_string(), -1.0), ("serve".to_string(), 0.0)]);
    assert_eq!(s.metrics, [("points".to_string(), 0.0)]);
}

#[test]
fn events_carry_their_seat_by_name() {
    let e = GameEvent::new("hit").seat(1).subject(Entity { index: 4, generation: 0 }).value(16.5).datum("spin", -2.0).note("an end");
    let h = Happened::of(120, &e, &pong());
    assert_eq!((h.frame, h.kind.as_str(), h.seat.as_str(), h.value), (120, "hit", "right", 16.5));
    assert_eq!(h.other, Entity::DEAD);
    assert_eq!(Happened::of(1, &GameEvent::new("hit").seat(9), &pong()).seat, "");
}

#[test]
fn the_json_view_writes_entities_seats_and_events() {
    let p = pong();
    let mut ball = seen("ball", "", 20.0, 10.5, 0.5, 0.5);
    ball.id = Entity { index: 3, generation: 0 };
    let o = Observation {
        frame: 7,
        json: Some(Structured {
            view: [0.0, 0.0, 40.0, 20.0],
            entities: vec![ball],
            seats: vec![SeatView::of(&p, 0, &ActionState { held: vec![1.0, 0.0] }, &Metrics { values: vec![3.0] })],
        }),
        events: vec![Happened::of(6, &GameEvent::new("hit").seat(0).datum("spin", 0.1).note("say \"hi\"\n"), &p)],
        ..Observation::default()
    };
    let expected = concat!(
        r#"{"frame":7,"view":{"x":0,"y":0,"w":40,"h":20},"#,
        r#""entities":[{"id":"3","what":"ball","x":20,"y":10.5,"w":0.5,"h":0.5,"vx":0,"vy":0}],"#,
        r#""seats":[{"name":"left","actions":{"paddle":1,"serve":0},"metrics":{"points":3}}],"#,
        r#""events":[{"frame":6,"kind":"hit","seat":"left","value":0,"data":{"spin":0.1},"note":"say \"hi\"\n"}]}"#
    );
    assert_eq!(o.render(), expected);
}

#[test]
fn the_vector_view_writes_names_and_values_and_non_numbers_as_null() {
    let o = Observation {
        frame: 1,
        vector: Some(observe::VectorView { names: vec!["a".into(), "b".into()], values: vec![0.5, f32::NAN] }),
        ..Observation::default()
    };
    assert_eq!(o.render(), r#"{"frame":1,"names":["a","b"],"values":[0.5,null],"events":[]}"#);
}

#[test]
fn the_text_observation_ends_with_its_events_and_an_error_is_all_there_is() {
    let p = pong();
    let mut o = Observation { frame: 3, text: Some(TextView::draw([0.0, 0.0, 1.0, 1.0], [1.0, 1.0], vec![])), ..Observation::default() };
    assert!(o.render().starts_with("frame 3\ncolumn c is"));
    assert!(o.render().ends_with("entities:\nevents since last observed: none\n"), "{}", o.render());
    let point = GameEvent::new("point_won").seat(0).subject(Entity { index: 3, generation: 1 }).value(2.0).datum("rally", 4.0);
    o.events = vec![Happened::of(2, &point, &p)];
    assert!(
        o.render().ends_with("events since last observed:\n  frame 2 point_won seat left subject 3.1 value 2 rally 4\n"),
        "{}",
        o.render()
    );
    assert_eq!(Observation::failed(3, "no observer").render(), "error: no observer");
}

#[test]
fn ids_read_short_until_reused() {
    assert_eq!(id(Entity { index: 12, generation: 0 }), "12");
    assert_eq!(id(Entity { index: 12, generation: 3 }), "12.3");
}
