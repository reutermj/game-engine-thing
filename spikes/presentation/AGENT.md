# Playing pong over modctl (for an AI agent)

SPIKE (get-3hd.1): part of the presentation spike, deleted with it.

A person watches the game in a window while you play it through text
commands. Time only moves when you say so; the window shows the frames
you ask for at real-time speed.

For two agents playing each other, in turns, see AGENT_VERSUS.md.

## Start it

From the repository root (a person usually does this):

    ./bazel run //spikes/presentation:pong_window_launch

The engine listens on `/run/user/1000/pong-window/control.sock`. Every
command below needs that socket:

    export ENGINE_SOCKET=/run/user/1000/pong-window/control.sock
    M=bazel-bin/engine/modctl/modctl

The window is fixed at 1184x680, typed a dialog, so i3 and other tiling
window managers float it. Its `WM_CLASS` class is `game-engine-thing`
(instance `pong`), for a rule such as i3's
`for_window [class="game-engine-thing"] floating enable`.

## The commands

| what | command | reply |
|---|---|---|
| observe, as numbers | `$M send pong_text state` | below |
| observe, as a picture | `$M send pong_text show` | a 40x20 text grid, `#` paddles, `o` ball |
| move your paddle up | `$M send pong_text up` | `moving up` |
| move it down | `$M send pong_text down` | `moving down` |
| stop it | `$M send pong_text stay` | `staying` |
| advance time | `$M send lockstep step 6` | `frame 606` |
| pacing | `$M send lockstep pace off` (or `on`, or a speed such as `0.5`) | what it is now |
| quit | `$M quit` | `quitting` |

A frame is 1/60 s. `step N` runs N frames and replies when they are done:
with pacing on (the default) that takes N/60 s, so the spectator sees them;
with it off, a few ms. Pacing never changes the game: the same commands give
the same `state` either way. While a step runs, other commands wait for it.

`up`, `down` and `stay` set your paddle moving and it **keeps moving** until
the next one, taking effect from the next frame. A paddle moves 16 cells a
second (about 0.27 a frame) and stops at the court's edges.

A good loop: `state`, decide, one of `up`/`down`/`stay`, `step 6` (0.1 s).

## What `state` means

    frame 606
    ball x 18.00 y 2.70 vx 20.42 vy 8.10
    you paddle face 2 y 10.00 intent 0.0
    ai paddle face 38 y 2.21 intent 0.8
    score you 0 ai 0

- Units are court cells. The court is 40 wide (x 0 to 40, left to right)
  and 20 tall (**y 0 at the top, growing downward**).
- `ball`: its centre, and its velocity in cells a second (`vx` < 0 is
  coming toward you). Its radius is 0.25.
- **You are the left paddle** (`you`, face at x = 2, cyan in the window).
  The AI is the right one (face at x = 38, orange). `y` is a paddle's
  centre; a paddle is 4 tall, so it covers `y - 2` to `y + 2` (the ball
  bounces off it up to 2.25 from its centre). `intent` is -1 moving up,
  1 down, 0 still.
- `score`: your points and the AI's.

## How a point is scored

The ball bounces off the top and bottom walls. If it gets past a paddle to
the goal line behind it (x below 0 on your side, above 40 on the AI's), the
other side scores, and the ball is served again from the centre toward the
side that lost the point.

On a paddle hit the ball speeds up by 5% (up to 40 cells a second) and
gains vertical speed by how far off the paddle's centre it struck (3 cells
a second per cell off centre): hit it with the paddle's edge to send it
steeply. The AI follows the ball at 80% of paddle speed, so steep, fast
returns beat it.

To meet the ball, predict where it reaches x = 2: `t = (x - 2) / -vx`
seconds away, at `y + vy * t`, folded back off the walls at y = 0.25 and
y = 19.75.
