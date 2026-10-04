# Reviewing a recorded pong session (for a reviewer agent)

SPIKE (get-3hd.1): part of the presentation spike, deleted with it.

Every game started with `:pong_window_launch` (AGENT.md) or
`:pong_versus_launch` (AGENT_VERSUS.md) is recorded. Pong is bit-for-bit
deterministic, so the log's inputs reproduce every frame, and a replay can
answer questions nobody thought to ask while it was played. Your job is to
find out whether what the players reported happened, and why: the tool
shows you the frames; it doesn't diagnose them.

## Where the sessions are

    /run/user/1000/pong-sessions/<YYYYMMDD-HHMMSS>-<game>/session.jsonl

UTC time the game started; `<game>` is `pong_window` (one agent against
`pong_ai`) or `pong_versus` (two agents in turns). The launcher prints the
path and the replay command when it starts. `PONG_SESSIONS=<dir>` puts them
elsewhere and `PONG_RECORD=0` turns recording off. `/run/user` is a tmpfs:
copy a session you want to keep.

## Replay it

From the repository root:

    ./bazel run //spikes/presentation:pong_replay -- <session.jsonl>

It loads the recorded game into an engine in its own process (no window,
no pacing: about half a second for 5000 frames, plus Bazel's start),
sends the recorded inputs before
the frames they took effect in, and checks the result against the log. It
writes, beside the log in `replay/` (or `--out <dir>`):

| file | what |
|---|---|
| `verify.txt` | `REPRODUCED` or `NOT REPRODUCED`, the first mismatches, the final state, a count of flags, the players' notes |
| `flags.jsonl` | frames that look wrong, one JSON object a line, with why |
| `probes.jsonl` | every frame: the ball, both paddles, the score, the ball's contacts |
| `probes.csv` | the same numbers as a table (contacts summarised) |
| `frames/frame-NNNNNN.png` | with `--frames`, the frames as the window drew them |

Options:

- `--frames <n>..<m>`: render frames n to m (inclusive) as PNGs. The
  window's canvas is 1184x680; `--scale` (default 0.5) sizes it, so
  `--scale 1` is what the spectator saw, pixel for pixel. The pixels are
  deterministic: the same frame renders to the same bytes every time.
- `--depth <cells>`: `deep_penetration`'s threshold (default 0.1).
- `--window <frames>`: how many frames after a paddle hit the ball has to
  turn before `hit_no_bounce` (default 6).
- `--plain` (one-player sessions only): replay on plain `//pong`, without
  the recorder, the view or the window's bootstrap. If that reproduces too,
  nothing the spike added changed the game.

Exit status 0 means it reproduced; 1 that it ran but didn't (everything is
still written, but frames after the first mismatch aren't the session's);
2 that it couldn't run.

**Check `verify.txt` first.** If it says `NOT REPRODUCED`, the probes and
flags describe some other game. The usual cause is code that changed since
the session: the log's first line has the commit (and `"dirty":true` if the
tree had uncommitted changes then; the tool says so). Check out that commit
and replay again. A mod reloaded during the session (`bazel run //...`)
also isn't in the log.

## What happened when: frames and turns

A frame is 1/60 s; frame 1 is the first that ran. Every record names a
frame. `turn` in the probes and flags is the versus turn the frame was
played in (what the players call turn 327); in a one-player session it is
the number of the agent's `step` command that ran it, counting from 1. In
versus, turn N starts at frame `(N - 1) * 6 + 1` only while turns are 6
frames; the log's `turn` records have each turn's first frame (`from`).

## The court's geometry, for reading the numbers

Units are cells, y grows downward (AGENT.md). What the colliders are, which
the numbers below are measured against (pong's `set_up`):

- **Walls**: boxes whose inner faces are at y = -0.25 and y = 20.25, a
  radius outside the court. So the ball's centre turns at **y = 0 and
  y = 20**, which is where AGENT.md tells players to fold (it said 0.25
  and 19.75 before get-c3s's fix). `beyond_wall` uses 0.25..19.75; its `wall_overlap` is how
  far the ball is inside a wall's box (centre below 0 or above 20), which
  is the part no bounce explains.
- **Paddles**: boxes 1 wide and 4.5 tall (half extents 0.5 and 2.25),
  their faces at x = 1.75 and 38.25, so the ball's centre meets a face
  at x = 2 or 38 (`LEFT_FACE`, `RIGHT_FACE`) within 2.25 of the paddle's
  centre in y. Past 2.25 (up to 2.5) it meets the box's end, a corner or
  the end cap, which pushes it vertically.
- **Ball**: a circle of radius 0.25.
- **Pong's rule on a hit** (`rebound`, `pong/core/rules.rs`): on the
  first frame of a contact between the ball and a paddle's *face*
  (`physics2d::Contact` whose normal, from the ball, points into the
  paddle from the court more than along it), it multiplies `vx` by 1.05
  and adds `3 * (ball y - paddle y)` to `vy`, then scales the velocity
  down to a speed of 40 if it's faster. A contact with an end or the
  back is left to physics, which bounces the ball as off a wall
  (get-c3s). Recordings from before that fix (commit b732529 and older)
  were played under the old rule: any paddle contact got the kick, and
  `vx` and `vy` were clamped to ±40 apart (get-7la).

## Flags

One JSON object a line, `{"flag": kind, "frame", "turn", ...}`. Positions
and velocities are `{"x","y","vx","vy"}`; numbers are f32s as Rust prints
them, exact.

**`hit_no_bounce`**: a ball-paddle contact began (`trigger: "contact"`), or
`vy` jumped near a paddle without one (`"kick"`), while the ball was coming
toward that paddle, and `vx` didn't turn within `--window` frames.

- `before` is the ball the frame before the hit, `at` its frame, `then`
  where it was when the check ended (`then_frame`); `scored` says it went
  in.
- `offset` is the ball's y minus the paddle's centre at the hit; `reach`
  is 2.25. An offset beyond the reach is a hit on the paddle's end.
- `contact`: the contact at the end of the hit frame, `depth` (positive
  is overlap), `n` the normal from the ball to the paddle, `pressed`,
  `disabled`, `impulse` [normal, tangent]. `began`: the `Contact` event
  (normal and closing speed).

Example, from a scripted versus session (the paddle's top end, moving up,
meets a 40-cell/s ball; pong's kick turns `vy` to -40, `vx` stays 40, and
three frames later it's a point):

    {"flag":"hit_no_bounce","frame":2181,"turn":631,"side":"right","trigger":"contact",
     "before":{"x":38.29985,"y":0.56519014,"vx":40,"vy":17.985834},
     "at":{"x":38.96652,"y":0.250739,"vx":40,"vy":-40},
     "then":{"x":20,"y":10,"vx":16,"vy":-9.6},"then_frame":2184,"scored":true,
     "paddle_y":2.5333333,"offset":-2.2825942,"reach":2.25,
     "contact":{"depth":0.26519012,"n":[-0,1],"pressed":true,"disabled":false,"impulse":[67.971664,0]},
     "began":{"n":[-0,1],"speed":33.985832}}

**`hit_receding`**: the same, but the ball was already moving away from
the paddle (the paddle caught up with it). Pong applies its kick only if
the contact was with the face.

**`beyond_wall`**: the ball's centre outside 0.25..19.75, one flag for a
run of frames: `frames` [first, last], `worst` how far outside the limits
at `worst_frame`, and that frame's `y`, `vy`, `outward` (still moving
out), `wall_overlap` (above) and `ball`. With `wall_overlap` 0 it's an
ordinary bounce, since the centre turns at 0 and 20.

**`deep_penetration`**: the ball overlapping a paddle's box by more than
`--depth`, one flag a run: `overlap` at the worst frame, `past_face` (how
far the centre is behind the face line, x = 2 or 38), the `ball`, the
`paddle_y`, and `contact_depth`, the contact's own depth if physics has
one at that frame (`null`: it hasn't found the contact yet).

**`speed_loss`**: at a paddle hit that did turn the ball, `|v|` after it
under 60% of `|v|` before: `speed_before`, `speed_after`, `loss`, both
states and `hit_frame`.

## Probes

`probes.jsonl`, a line a frame, the world after that frame ran:

    {"frame":2181,"turn":631,
     "ball":{"x":38.96652,"y":0.250739,"vx":40,"vy":-40,"speed":56.568542},
     "left":{"y":10.000008,"vy":0,"intent":0,"overlap":-37.714565},
     "right":{"y":2.5333333,"vy":-15.999998,"intent":-1,"overlap":0.2174058},
     "score":[0,0],
     "contacts":[{"with":"right","depth":0.26519012,"n":[-0,1],"pressed":true,
                  "was_pressed":false,"disabled":false,"impulse":[67.971664,0]}],
     "began":[{"with":"right","n":[-0,1],"speed":33.985832}]}

- `left`/`right`: the paddle's centre `y`, its `vy` (±16 when moving),
  `intent` (-1 up, 1 down, 0), and `overlap`, how far the ball overlaps its
  box (negative: the gap).
- `contacts`: every contact physics holds for the ball at the end of the
  frame, `with` one of `left`, `right` (paddles), `top`, `bottom` (walls):
  `depth` (positive is overlap, negative a gap a speculative contact is
  held across), `n` from the ball to the other body, `pressed` (pushing at
  the end of the step), `was_pressed` (the step before), `disabled`, and
  `impulse` [normal, tangent] from the solve.
- `began`: `physics2d::Contact` events this frame: contacts that began,
  which are what pong's `rebound` acts on.

`probes.csv` has the same per-frame numbers in columns (`contacts` as
`with:depth`, space separated), for plotting or a quick `awk`.

## A way to work

1. Read `verify.txt`: reproduced? how many of each flag?
2. For a reported moment ("turn 327"), find its frames:
   `grep '"turn":327,' replay/probes.jsonl`. Look a few frames either side.
3. Read the flags near it, and the probes for the frames they name: where
   the ball was, which contacts physics held, when the `Contact` began and
   what `rebound` did to `vx` and `vy` after it.
4. Render the moment and look at it:
   `--frames <first-10>..<last+5> --scale 1`.
5. Say what you found with frames and numbers, and what you couldn't tell.

## The log's format

`session.jsonl`, one record a line, each with `t` (its kind) and `ms`
(wall-clock milliseconds since the engine started recording, which never
reaches the game). The launcher writes the first line; the bootstrap
(`lockstep_window.rs`) the rest, each as it happens.

| `t` | fields | what |
|---|---|---|
| `meta` | `game`, `target`, `commit`, `dirty`, `turn_frames`, `started`, `format` | what was played |
| `start` | `turns` (frames a turn, 0 for one player), `dt` | the bootstrap began |
| `input` | `frame`, `to`, `msg`, `steers` | `pong_text up/down/stay` took effect in `frame` (the last of `steers` sent before it) |
| `step` | `from`, `to`, `fps`, `dt` | a `lockstep step` ran frames `from` to `to` (`fps` from `at <fps>`, else null) |
| `submit` | `turn`, `side`, `action` | a versus submission, when it came |
| `turn` | `turn`, `from`, `frames`, `left`, `right` | a turn started playing at frame `from` with those actions |
| `point` | `frame`, `by`, `score`, `ball` | the score changed in `frame` (the ball is already served again) |
| `check` | `frame`, `watched`, `ball` [x,y,vx,vy], `paddles` [left y, right y], `score` | the state after a step, a turn, the game's end or the session's |
| `over` | `frame`, `turn`, `winner` | versus: a side reached the point limit |
| `note` | `frame`, `text` | `lockstep note <text>`, from a player or the person watching |
| `end` | `frame` | the engine quit |

What isn't in it: `pace` (it changes when frames run, not what they
compute), `state` and `show` (they read), `pong_versus first-to` (the
replay plays with no point limit and stops where the log does), and
reloads.
