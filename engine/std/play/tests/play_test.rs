//! The declaration's rules, the discrete set and commands derived from
//! it, and declaring and acting on a world of its own.

use engine_ecs::World;
use play::{
    Act, Action, ActionError, ActionState, Bound, Choice, GameEvent, Goal, Invalid, Metric, Metrics, Playable, Seat, Setting, act,
    act_command, declare, declared, source,
};

/// Pong, as presentation.md's example declares it.
fn pong() -> Playable {
    Playable::new("Return the ball past the other paddle; first to 5 points wins.")
        .players(&["left", "right"])
        .action(
            Action::axis("paddle", "moves your paddle: -1 up, 1 down, 0 stops it")
                .held()
                .directions("up", "stay", "down")
                .key("KeyW", -1.0)
                .key("KeyS", 1.0)
                .key_for(1, "ArrowUp", -1.0)
                .key_for(1, "ArrowDown", 1.0)
                .pad("DPadUp", -1.0)
                .pad("DPadDown", 1.0)
                .pad("LeftStickY", -1.0),
        )
        .event("hit", "the ball bounced off a paddle: subject the ball, other the paddle")
        .goal(Goal::success("point_won", "the ball passed the other paddle"))
        .goal(Goal::failure("point_lost", "the ball passed your paddle"))
        .goal(Goal::success("match_won", "you reached 5 points first").ends().reward(10.0))
        .goal(Goal::failure("match_lost", "the other side reached 5 points first").ends().reward(-10.0))
        .metric(Metric::new("points", "your points", 0.0, 5.0))
        .observe("ball", 1)
        .observe("paddle", 2)
        .stall_after(3600)
}

fn names(choices: &[Choice]) -> Vec<&str> {
    choices.iter().map(|c| c.name.as_str()).collect()
}

fn set(action: u32, value: f32) -> Setting {
    Setting { action, value }
}

#[test]
fn pong_declares_cleanly_and_its_choices_are_its_words() {
    let p = pong();
    assert_eq!(p.validate(), Ok(()));
    assert_eq!(names(&p.choices()), ["noop", "up", "stay", "down"]);
    assert_eq!(p.choices()[1].setting, Some(set(0, -1.0)));
    assert_eq!(p.choices()[0].setting, None);
    assert_eq!(p.options(0), [-1.0, 0.0, 1.0]);
    assert_eq!(p.options(1), [] as [f32; 0]);
}

#[test]
fn every_kind_of_action_has_its_choices() {
    let p = Playable::new("test")
        .action(Action::button("jump", "jumps").key("Space", 1.0))
        .action(Action::button("crouch", "crouches while held").held())
        .action(Action::axis("steer", "turns"));
    assert_eq!(p.validate(), Ok(()));
    assert_eq!(names(&p.choices()), ["noop", "jump", "crouch", "crouch=0", "steer=-1", "steer=0", "steer=1"]);
    assert_eq!(p.choices()[3].setting, Some(set(1, 0.0)));
    assert_eq!(p.choices()[6].setting, Some(set(2, 1.0)));
    assert_eq!(p.options(0), [0.0, 1.0]);
}

#[test]
fn commands_set_choices_and_values() {
    let p = pong();
    assert_eq!(p.parse("up"), Ok(vec![set(0, -1.0)]));
    assert_eq!(p.parse("  down "), Ok(vec![set(0, 1.0)]));
    assert_eq!(p.parse("paddle=-0.5"), Ok(vec![set(0, -0.5)]));
    assert_eq!(p.parse("noop"), Ok(vec![]));
    let two = Playable::new("test").action(Action::button("jump", "jumps")).action(Action::axis("steer", "turns"));
    assert_eq!(two.parse("jump, steer=1"), Ok(vec![set(0, 1.0), set(1, 1.0)]));
    assert_eq!(two.parse("steer=0 jump=0"), Ok(vec![set(1, 0.0), set(0, 0.0)]));
}

#[test]
fn a_bad_command_is_answered_with_what_would_do() {
    let p = pong();
    let err = |c: &str| p.parse(c).unwrap_err().to_string();
    assert_eq!(err("jump"), "unknown action `jump`: choose from noop, up, stay, down");
    assert_eq!(err("speed=1"), "unknown action `speed=1`: choose from noop, up, stay, down");
    assert_eq!(err("paddle=fast"), "`paddle=fast`: the value isn't a number");
    assert_eq!(err("paddle=1.5"), "`paddle` is an axis, from -1 to 1, not 1.5");
    assert_eq!(err("paddle=NaN"), "`paddle` is an axis, from -1 to 1, not NaN");
    assert_eq!(err("up down"), "`paddle` is set twice");
    assert_eq!(err("up paddle=-1"), "`paddle` is set twice", "even to one value: a command says each thing once");
    assert_eq!(err(" , "), "no action given: send `noop` to change nothing");
    let jump = Playable::new("test").action(Action::button("jump", "jumps"));
    assert_eq!(jump.parse("jump=0.5").unwrap_err().to_string(), "`jump` is a button, 0 or 1, not 0.5");
    assert_eq!(jump.check(1, 1.0).unwrap_err(), ActionError::Unknown { token: "#1".into(), choices: vec!["noop".into(), "jump".into()] });
    assert_eq!(jump.check(0, 1.0), Ok(1.0));
    assert_eq!(jump.check(0, 0.0), Ok(0.0));
    assert_eq!(p.check(0, -1.0), Ok(-1.0));
}

#[test]
fn controls_find_what_they_drive() {
    let p = pong();
    let bound = |seat, value| Some(Bound { seat, action: 0, value });
    assert_eq!(p.bound(platform::KEYBOARD, 0, "KeyW"), bound(0, -1.0));
    assert_eq!(p.bound(platform::KEYBOARD, 0, "ArrowDown"), bound(1, 1.0));
    // A pad drives the seat of its number, whatever pad is asked about.
    assert_eq!(p.bound(platform::GAMEPAD, 1, "DPadUp"), bound(1, -1.0));
    assert_eq!(p.bound(platform::GAMEPAD, 0, "LeftStickY"), bound(0, -1.0));
    assert_eq!(p.bound(platform::GAMEPAD, 2, "DPadUp"), None, "pong has two seats");
    assert_eq!(p.bound(platform::KEYBOARD, 0, "KeyQ"), None);
    assert_eq!(p.bound(platform::GAMEPAD, 0, "KeyW"), None);
    assert_eq!(p.bound(0, 0, "KeyW"), None);
}

/// Validates `p` changed by `change`, expecting exactly `why`.
fn refused(change: impl FnOnce(&mut Playable), why: &[&str]) {
    let mut p = pong();
    change(&mut p);
    let found: Vec<String> = p.validate().err().unwrap_or_default().iter().map(Invalid::to_string).collect();
    assert_eq!(found, why);
}

#[test]
fn the_game_itself_is_checked() {
    refused(|p| p.objective = " ".into(), &["objective: is empty: say, in prose, what a player is trying to do"]);
    refused(
        |p| p.players.clear(),
        &[
            "players: is empty: a game has at least one seat",
            "actions[0] `paddle`, key `ArrowUp`: drives seat 1, of 0",
            "actions[0] `paddle`, key `ArrowDown`: drives seat 1, of 0",
        ],
    );
    refused(|p| p.players[1] = "left".into(), &["players[1]: `left` is already players[0]"]);
    refused(
        |p| p.players[0] = "Left".into(),
        &["players[0]: `Left` isn't a name: lowercase letters, digits and `_`, from a letter, at most 32"],
    );
    refused(|p| p.observed[1].count = 0, &["observed[1]: has room for no `paddle`"]);
    refused(|p| p.observed[1].what = "ball".into(), &["observed[1]: `ball` is already observed[0]"]);
    refused(|p| p.observed[0].what.clear(), &["observed[0]: names no label class"]);
}

#[test]
fn names_are_identifiers() {
    for (bad, ok) in [("9lives", false), ("_x", false), ("a-b", false), ("", false), ("a b", false), ("ok_9", true), ("x", true)] {
        let p = Playable::new("test").event(bad, "it happened");
        assert_eq!(p.validate().is_ok(), ok, "`{bad}`");
    }
    let long = "a".repeat(33);
    assert!(Playable::new("test").event(&long, "it happened").validate().is_err());
    assert!(Playable::new("test").event(&long[..32], "it happened").validate().is_ok());
}

#[test]
fn actions_are_checked() {
    let at = "actions[0] `paddle`";
    refused(|p| p.actions[0].meaning.clear(), &[&format!("{at}: says nothing of what it means")]);
    refused(|p| p.actions[0].neutral.clear(), &[&format!("{at}: names some of its directions: name all three (-1, 0, 1) or none")]);
    refused(|p| p.actions[0].positive = "up".into(), &[&format!("a direction of {at}: `up` is already a direction of {at}")]);
    refused(|p| p.actions[0].positive = "noop".into(), &[&format!("a direction of {at}: `noop` is already the choice `noop`")]);
    refused(|p| p.actions.push(Action::button("up", "jumps")), &[&format!("actions[1] `up`: `up` is already a direction of {at}")]);
    refused(
        |p| p.actions[0].negative = "Up".into(),
        &[&format!("{at}, direction `Up`: `Up` isn't a name: lowercase letters, digits and `_`, from a letter, at most 32")],
    );
    refused(
        |p| p.actions.push(Action::button("fire", "fires").directions("a", "b", "c")),
        &["actions[1] `fire`: is a button, and only an axis names its directions"],
    );
}

#[test]
fn bindings_are_checked() {
    let at = "actions[0] `paddle`";
    refused(
        |p| p.actions[0].keys[0].control = "w".into(),
        &[&format!("{at}, key `w`: isn't a control (platform::KEYS, PAD_BUTTONS, PAD_AXES)")],
    );
    refused(
        |p| p.actions[0].keys[0].control = "South".into(),
        &[&format!("{at}, key `South`: isn't a control (platform::KEYS, PAD_BUTTONS, PAD_AXES)")],
    );
    refused(
        |p| p.actions[0].pad[0].control = "KeyW".into(),
        &[&format!("{at}, pad `KeyW`: isn't a control (platform::KEYS, PAD_BUTTONS, PAD_AXES)")],
    );
    refused(|p| p.actions[0].keys[0].value = 0.0, &[&format!("{at}, key `KeyW`: gives 0: an axis's binding gives -1 to 1, not 0")]);
    refused(|p| p.actions[0].keys[0].value = -1.5, &[&format!("{at}, key `KeyW`: gives -1.5: an axis's binding gives -1 to 1, not 0")]);
    refused(|p| p.actions[0].pad[0].seat = 1, &[&format!("{at}, pad `DPadUp`: names a seat: gamepad n drives seat n")]);
    refused(|p| p.actions[0].keys[0].seat = 2, &[&format!("{at}, key `KeyW`: drives seat 2, of 2")]);
    refused(|p| p.actions[0].keys[1].control = "KeyW".into(), &[&format!("{at}'s key 1: `key `KeyW` of seat 0` is already {at}'s key 0")]);
    // The same key for two seats is two bindings.
    refused(|p| p.actions[0].keys[2].control = "KeyW".into(), &[]);
    let button = |b: Action| Playable::new("test").action(b).validate().err().unwrap_or_default();
    assert_eq!(
        button(Action::button("fire", "fires").key("Space", 0.5))[0].to_string(),
        "actions[0] `fire`, key `Space`: gives 0.5: a button's binding gives 1"
    );
    assert_eq!(
        button(Action::button("fire", "fires").pad("LeftStickX", 1.0))[0].to_string(),
        "actions[0] `fire`, pad `LeftStickX`: is a stick, which only an axis can take"
    );
    assert_eq!(button(Action::button("fire", "fires").pad("South", 1.0)), []);
}

#[test]
fn events_goals_and_metrics_are_checked() {
    refused(|p| p.events[0].meaning = "".into(), &["events[0] `hit`: says nothing of what it means"]);
    refused(|p| p.goals[0].name = "hit".into(), &["goals[0] `hit`: `hit` is already events[0] `hit`"]);
    refused(|p| p.goals[1].meaning = "".into(), &["goals[1] `point_lost`: says nothing of what it means"]);
    refused(|p| p.goals[0].reward = -1.0, &["goals[0] `point_won`: is a success worth -1: a success is worth 0 or more"]);
    refused(|p| p.goals[1].reward = 0.5, &["goals[1] `point_lost`: is a failure worth 0.5: a failure is worth 0 or less"]);
    refused(|p| p.goals[1].reward = f32::NEG_INFINITY, &["goals[1] `point_lost`: is worth -inf"]);
    refused(|p| p.goals[0].reward = 0.0, &[]);
    refused(|p| p.metrics[0].max = 0.0, &["metrics[0] `points`: runs from 0 to 0: it needs a finite range, low to high"]);
    refused(|p| p.metrics[0].min = f32::NAN, &["metrics[0] `points`: runs from NaN to 5: it needs a finite range, low to high"]);
    refused(
        |p| p.metrics.push(Metric::new("points", "again", 0.0, 1.0)),
        &["metrics[1] `points`: `points` is already metrics[0] `points`"],
    );
    refused(|p| p.metrics[0].meaning.clear(), &["metrics[0] `points`: says nothing of what it means"]);
}

#[test]
fn events_are_checked_against_the_declaration() {
    let p = pong();
    assert_eq!(p.check_event(&GameEvent::new("hit")), Ok(()));
    assert_eq!(p.check_event(&GameEvent::new("point_won").seat(1)), Ok(()));
    assert_eq!(p.check_event(&GameEvent::new("goal")), Err("event `goal` isn't declared".into()));
    assert_eq!(p.check_event(&GameEvent::new("point_won").seat(2)), Err("event `point_won` is for seat 2, of 2".into()));
}

// ---- On a world ----

fn world_with(p: Playable) -> World {
    let w = World::new();
    declare(&mut w.between_frames(Default::default()).unwrap(), p).unwrap();
    w
}

/// Each seat's (index, name, held, metrics), by index.
fn seats(w: &World) -> Vec<(u32, String, Vec<f32>, Vec<f32>)> {
    let mut out = Vec::new();
    w.between_frames(Default::default())
        .unwrap()
        .for_each::<(&Seat, &ActionState, &Metrics)>(|_, (s, a, m)| out.push((s.index, s.name.clone(), a.held.clone(), m.values.clone())));
    out.sort_by_key(|s| s.0);
    out
}

fn acts(w: &World) -> Vec<Act> {
    w.events_of::<Act>().map(|(events, _)| events.into_iter().map(|(_, _, e)| e).collect()).unwrap_or_default()
}

#[test]
fn declaring_makes_a_seat_for_each_player() {
    let w = world_with(pong());
    assert_eq!(seats(&w), [(0, "left".into(), vec![0.0], vec![0.0]), (1, "right".into(), vec![0.0], vec![0.0])]);
    assert_eq!(declared(&mut w.between_frames(Default::default()).unwrap()), Some(pong()));
}

#[test]
fn an_invalid_declaration_puts_nothing_in_the_world() {
    let w = World::new();
    let mut m = w.between_frames(Default::default()).unwrap();
    let bad = pong().players(&[]);
    assert!(declare(&mut m, bad).is_err());
    assert_eq!(declared(&mut m), None);
    drop(m);
    assert_eq!(seats(&w), []);
}

#[test]
fn acting_holds_held_actions_and_sends_every_one_with_its_source() {
    let p = pong().action(Action::button("serve", "serves the ball"));
    let w = world_with(p);
    let mut m = w.between_frames(Default::default()).unwrap();
    assert_eq!(act_command(&mut m, "right", "up serve", source::AGENT), Ok(vec![set(0, -1.0), set(1, 1.0)]));
    drop(m);
    // The per-frame `serve` isn't held: it's only an `Act`.
    assert_eq!(seats(&w)[1].2, [-1.0, 0.0]);
    assert_eq!(seats(&w)[0].2, [0.0, 0.0]);
    assert_eq!(
        acts(&w),
        [Act { seat: 1, action: 0, value: -1.0, source: source::AGENT }, Act { seat: 1, action: 1, value: 1.0, source: source::AGENT }]
    );
    act(&mut w.between_frames(Default::default()).unwrap(), 0, &[set(0, 0.25)], source::GAMEPAD).unwrap();
    assert_eq!(seats(&w)[0].2, [0.25, 0.0]);
    assert_eq!(acts(&w).last(), Some(&Act { seat: 0, action: 0, value: 0.25, source: source::GAMEPAD }));
}

#[test]
fn a_bad_act_changes_nothing() {
    let w = world_with(pong());
    let mut m = w.between_frames(Default::default()).unwrap();
    assert_eq!(
        act(&mut m, 0, &[set(0, -1.0), set(0, 3.0)], source::AGENT),
        Err(ActionError::OutOfRange { action: "paddle".into(), value: 3.0 })
    );
    assert_eq!(
        act(&mut m, 2, &[set(0, 1.0)], source::AGENT),
        Err(ActionError::UnknownSeat { seat: "#2".into(), seats: vec!["left".into(), "right".into()] })
    );
    assert_eq!(act(&mut m, 0, &[set(0, 1.0)], 0), Err(ActionError::BadSource { source: 0 }));
    assert_eq!(act(&mut m, 0, &[set(0, 1.0)], 6), Err(ActionError::BadSource { source: 6 }));
    assert_eq!(act_command(&mut m, "middle", "up", source::AGENT).unwrap_err().to_string(), "no seat `middle`: the seats are left, right");
    assert!(act_command(&mut m, "left", "up sideways", source::AGENT).is_err());
    drop(m);
    assert_eq!(seats(&w)[0].2, [0.0]);
    assert_eq!(acts(&w), []);
    let empty = World::new();
    assert_eq!(act(&mut empty.between_frames(Default::default()).unwrap(), 0, &[], source::AGENT), Err(ActionError::NotDeclared));
}

#[test]
fn declaring_again_keeps_each_seats_values_by_name() {
    let w = world_with(pong().metric(Metric::new("rally", "hits this rally", 0.0, 100.0)));
    let mut m = w.between_frames(Default::default()).unwrap();
    act_command(&mut m, "left", "down", source::KEYBOARD).unwrap();
    m.for_each::<(&Seat, &mut Metrics)>(|_, (s, mut ms)| ms.values = vec![s.index as f32 + 1.0, 7.0]);
    // A reload: an action and a metric before the old ones, `right` gone,
    // `third` new.
    let again = pong().players(&["left", "third"]).metric(Metric::new("rally", "hits this rally", 0.0, 100.0)).metric(Metric::new(
        "lives",
        "lives left",
        1.0,
        3.0,
    ));
    let mut again = again;
    again.actions.insert(0, Action::button("serve", "serves").held());
    again.metrics.reverse();
    declare(&mut m, again).unwrap();
    drop(m);
    assert_eq!(
        seats(&w),
        [
            // serve released, paddle kept; lives at its minimum, rally and
            // points kept.
            (0, "left".into(), vec![0.0, 1.0], vec![1.0, 7.0, 1.0]),
            (1, "third".into(), vec![0.0, 0.0], vec![1.0, 0.0, 0.0]),
        ]
    );
}
