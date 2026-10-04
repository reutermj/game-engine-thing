# Playing pong against another agent (for an AI agent)

SPIKE (get-3hd.1): part of the presentation spike, deleted with it.

Two agents play pong against each other over modctl while a person
watches in a window. One plays `left`, the other `right`. Time moves in
**turns**: each turn, both sides submit an action, and only when both
have does the game run the turn's frames (6 by default, a tenth of a
second), played out in the window at real-time speed. A fast thinker gets
no more turns than a slow one.

## Start it

From the repository root (a person usually does this):

    ./bazel run //spikes/presentation:pong_versus_launch

The engine listens on `/run/user/1000/pong-versus/control.sock`. Both
players use it:

    export ENGINE_SOCKET=/run/user/1000/pong-versus/control.sock
    M=bazel-bin/engine/modctl/modctl

The window is the same as AGENT.md's (1184x680, floats under i3). `left`
is cyan, `right` orange. There is no AI: both paddles are yours.

### The score band

The strip above the court shows the turn barrier, so a spectator can
tell a side that is thinking from a game that has hung. Mirrored about
the centre line, from the centre out:

    frame   [left timer] [left lamp]  [left score]  [marker]  [right score]  [right lamp] [right timer]   turn

- **Scores**, large, either side of the centre.
- **Lamps**, a square outside each score in the side's colour: dim while
  that side has yet to submit for the open turn, lit once it has (both
  lit while the turn plays).
- **Wait timers**, beside each lamp that is still dim: whole seconds that
  side has been waited on, counted from when the turn opened or, once the
  other side is in, from that submit. They tick while nothing else moves.
- **Marker**, between the scores: two dim pause bars while the turn is
  open, a green play triangle while its frames run.
- **Frame number** small at top left, **turn number** small at top right.
- **Game over**: the winner's half of the band is framed in its colour
  and its lamp lit; no marker, no timers.

Before the first turn plays no frame has run, so the court is blank and
only the band shows. The timers are the window's alone (wall-clock time
never enters the game), and restart if the view mod is reloaded.

## The commands

`SIDE` is `left` or `right`, whichever you were told you play.

| what | command | reply |
|---|---|---|
| observe | `$M send pong_versus state` | below |
| observe, as a picture | `$M send pong_versus show` | the turn line, then a 40x20 text grid, `#` paddles, `o` ball |
| submit your action | `$M send lockstep turn SIDE up` (or `down`, `stay`) | `submitted for turn 41, waiting for right` |
| whose move it is, short | `$M send lockstep turn` | `turn 41 open, 6 frames, waiting for right (frame 240)` |
| note something for the reviewer | `$M send lockstep note <text>` | `noted at frame 240` |

The game is recorded (REVIEW.md): a `note` saying what looked wrong, with
the turn, lands in the log at the current frame for whoever replays it.

Submitting **never waits**: it replies at once. If you are the second to
submit, the reply is `submitted for turn 41: both in, playing 6 frames`,
and the turn then plays out over the next 0.1 s while you poll.

- A second submit in the same turn replaces your first, until the other
  side's submit starts the turn.
- A submit while a turn plays is refused (`turn 41 playing, frame 3 of 6;
  turn 42 opens when it ends`): wait and poll.
- You can't see what the other side submitted until the turn plays: the
  `state` shows only *whether* it has.
- `$M send lockstep step` is refused in this game: only the barrier moves
  time.

Commands a player shouldn't send (they change the game for both; leave
them to the person): `$M send lockstep turn length N` (frames a turn, from
the next turn on), `$M send pong_versus first-to N` (or `off`), `$M send
lockstep pace off|on|<speed>`, `$M quit`. `lockstep turn frame` and
`lockstep turns N` are the replay tool's (REVIEW.md), not a player's.

## What `state` means

Both players read the same text; the sides are named, never "you".

    turn 41 open, 6 frames: submitted left, waiting for right (first to 5)
    frame 240
    ball x 18.00 y 2.70 vx 20.42 vy 8.10
    left paddle face 2 y 10.00 last up
    right paddle face 38 y 2.21 last stay
    score left 0 right 2

- The first line is the turn: its number, `open` (taking submissions) or
  `playing, frame 3 of 6`, who has submitted and who is still to, and the
  point limit. When a side reaches it, the line is
  `game over: left wins 5 to 3 (first to 5)` and no more turns are taken.
- `frame`: frames run so far. A frame is 1/60 s; turn N starts at frame
  `(N - 1) * 6` while turns are 6 frames.
- Units are court cells. The court is 40 wide (x 0 to 40, left to right)
  and 20 tall (**y 0 at the top, growing downward**).
- `ball`: its centre and velocity in cells a second. `vx < 0` is heading
  to `left`, `vx > 0` to `right`. Its radius is 0.25.
- `left` paddle's face is at x = 2, `right`'s at x = 38. `y` is a
  paddle's centre; a paddle is 4 tall, so it covers `y - 2` to `y + 2` (the
  ball bounces off it up to 2.25 from its centre).
- `last`: the action in force, from the last played turn (`none` before
  the first). It **keeps applying** for the whole turn: `up` and `down`
  move a paddle 16 cells a second (1.6 cells over a 6-frame turn) until it
  stops at the court's edge.
- `score`: points for each side.

## How a point is scored

As in AGENT.md: the ball bounces off the top and bottom walls; past a
paddle to the goal line behind it (x below 0 on the left, above 40 on the
right), it's a point to the other side, and the ball is served from the
centre toward the side that lost it. A hit on a paddle's face (within 2.25
of its centre) bounces the ball back, speeds it up by 5% and adds 3 cells
a second of vertical speed per cell it struck off the paddle's centre: hit
near the edge of the face to send it steeply. Its speed is capped at 40
cells a second, direction kept. A ball already past the face line that
meets a paddle's top or bottom end only glances off it, as off a wall (no
speed-up, no spin), and goes in.

To meet the ball, predict where it reaches your face (x = 2 for left, 38
for right): `t = (face - x) / vx` seconds away, at `y + vy * t`, folded
back off the walls at y = 0 and y = 20 (the ball's centre turns there:
the walls stand a radius outside the court). Your paddle covers 1.6
cells a turn, so start moving early.

## A loop

    1. state                         # wait for a turn line saying `open`
       - `game over`: stop.
       - `playing`, or your side already submitted: poll again (a few ms
         apart is plenty; a turn takes 0.1 s to play).
    2. decide from the ball and your paddle
    3. turn SIDE <up|down|stay>
    4. go to 1: poll `state` (or the shorter `lockstep turn`) until the
       turn number goes up, then observe the new turn's state.

Note the turn number when you submit; your move for that turn is in once
the reply says so, and the next thing to do is wait for `turn <n+1> open`.
