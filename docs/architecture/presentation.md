# Presentation, input and playtesting

**Status: sketch** (2026-10-04). Nothing here is built, and nothing is
decided past the direction agreed in conversation. It becomes a proposal
once the spike and the decision-models research below have reported.

The engine's own description says rendering, input and the game itself are
mods the loader reloads. Today neither game has a window: pong and the
platformer are played through messages, pong drawn by a hand-written
`text` mod. This doc is how they get a window, a keyboard and a GPU,
without losing what the text mode gave them: **every game can be played by
an AI agent**, in lockstep, through text, data or pixels, and watched in a
window while it plays.

## Requirements

1. **Everything is a mod.** The renderer reloads while a game runs; the
   window and GPU device, which can't, live in something resident.
2. **Every game is agent-playtestable with no per-game glue.** An agent
   observes, acts and steps any game through one interface.
3. **Watchable.** A window can attach to an agent's lockstep run and show
   it, without driving time.
4. **Deterministic.** Presentation never feeds back into the simulation,
   so lockstep runs and replays stay bit for bit, and there is a way to
   get the same pixels on every machine for tests and agents.
5. **Fast.** No cost of the abstraction that design can't remove: the
   extract is incremental, the GPU presenter batches and instances, and
   rendering overlaps the next frame's simulation.
6. **Swappable.** Backends are mods behind interfaces: a game never names
   wgpu, and a presenter never names a game.

## The shape

```text
game mods ──(presentation components)──► extract ──► draw list ──► window presenter (wgpu)
                                                         ├────────► pixel presenter (tiny-skia)
                                                         ├────────► text presenter
                                                         └────────► structured presenter (data)
keyboard / gamepad mod ─┐
agent mod (MCP) ────────┼──► action events ──► game mods
replay mod ─────────────┘
platform service (one provider): window, event loop, GPU device
    wgpu + winit | headless | none
```

## Decisions, as agreed so far

**D1. Games describe; presenters draw.** A game attaches presentation
components to its entities: what to show (shapes, sprites, text, a
camera, layers) and a semantic label for agents ("player", "ball",
"coin"). The vocabulary lives in an interface crate and says *what*, never
*how*: a wgpu-shaped interface would make every other presenter
second-class. Backend-only features (custom shaders, say) are optional
extensions the other presenters ignore.

**D2. The draw list is extracted incrementally.** Each frame a
presentation pipeline turns the components into a draw list, a flow
([flows.md](flows.md)). It is kept between frames and updated from the
world's change ticks, as `Live` keeps the broadphase's pairs
([live.md](live.md)), so its cost follows what changed, not what exists.
Bevy's per-frame extract of everything is the cost this avoids. Items
carry a material or mesh identity, so the GPU presenter can batch and
instance.

**D3. Presenters read the draw list.** A flow has any number of readers,
so presenters run side by side: a window for a human, text and structured
views for an agent, CPU pixels for a test, all from one frame. Each is a
mod; loading or unloading one changes nothing in the game.

- **window** (wgpu): retains GPU buffers per entity and uploads what
  changed;
- **pixels** (tiny-skia): deterministic CPU raster at a fixed size, for
  golden-image tests and vision agents, since GPU output differs across
  drivers;
- **text**: a character grid, what pong's `text` mod does by hand,
  generalised;
- **structured**: entities, positions, labels and state as data.

**D4. Input is actions.** A game reads its own action vocabulary
(`move_left`, `jump`, `serve`), declared with what each means, never a
device. Sources are mods writing action events: keyboard and gamepad
(gilrs) from the window, an agent, a replay. Every frame's actions are
recorded, which gives replays, playtests turned into regression tests (as
`platformer_test` does today), and the declared action space a decision
model needs.

**D5. Time.** The bootstraps stay as they are: realtime for a human,
lockstep for an agent (act, step n, observe). A window can attach to a
lockstep run as a **spectator**: it presents what's stepped and drives
nothing.

**D6. The platform is a service with one provider.** The window, the
event loop and the GPU device are long-lived state outside the world (the
open epic get-y5t, registered resources and resident mods). The provider
is wgpu + winit, a headless one, or none ([mod-deps.md](mod-deps.md),
"Calls between mods", one provider per service). Whether that provider is
a windowed bootstrap, since the window's event loop and the frame loop
both want the loop, is an open question below.

**D7. One agent interface.** On the control socket: `observe(view)` (text,
structured or pixels), `act(actions)`, `step(n)`, `actions()` (the action
space and its meanings), and `reset` or loading a state. An MCP server on
top, so Claude and other agents playtest any game with no glue. A
playtest harness runs sessions headless, records them, and turns good runs
into replay tests.

**D8. Rendering overlaps the next frame's simulation** (scheduling.md,
step 4, "through an extract step or double-buffering, decided with the
renderer spike"). Through the extract: the draw list is the hand-off, so
the presenter draws frame N while the scheduler simulates N+1. That needs
a value that outlives its frame (flows are emptied at frame end today) and
the scheduler running presentation nodes beside the next frame's systems,
machinery that system parallelism (get-znt.5) shares. A reload drains the
pipeline first. It adds a frame of latency, so a low-latency setting turns
it off. Lockstep has nothing to overlap: the agent waits for each
observation. Copy-on-write world versions (storage.md, "Later") stay a
later option, since presenters reading the world directly would break D1.

**D9. GPU state never crosses mods.** Each mod is its own library with its
own copy of wgpu's statics, and `Crossing<'call>` lets only owned data
cross a call. So all GPU work lives in the window presenter, and a game
wanting a GPU effect goes through that presenter's extension interface.
Reloading the presenter rebuilds its pipelines: a pause during development,
never in a game.

**D10. The stack.** wgpu (Vulkan on Linux) and winit, as Bevy uses, with
gilrs for gamepads and tiny-skia for deterministic pixels. Linux only.

## Open questions

- **The vocabulary.** What the draw list holds, 2D first: shapes, sprites,
  text, camera, layers, labels. How 3D (pile3d) extends it later. This
  and the action format are the stable contract; everything behind them
  is free to change.
- **Actions.** Buttons and analog axes; how a game declares its actions'
  meanings for an agent; how actions from several sources combine.
- **Who owns the loop.** winit's event loop wants to run the program; the
  bootstrap runs the session and calls `pump_loader` once a frame. A
  windowed bootstrap that pumps winit, or a platform service the
  bootstrap calls, decided by the spike.
- **What survives a renderer reload.** Whether any GPU object can be kept
  across one, given per-library statics, or the presenter rebuilds
  everything.
- **Assets.** Sprites need images and text needs fonts, and the engine has
  no asset loading or asset hot reload yet. Probably its own design, after
  shapes and text.
- **Observations.** The structured view's schema; pixel size and rate; how
  the text grid maps a world to characters for a game it's never seen.
- **Several providers of one service.** mod-deps.md's open question;
  most of this works without it.

## Plan

1. **Spike** (wgpu and winit), to test the riskiest assumptions before
   the interface is fixed:
   - they build under Bazel, and what the Vulkan and display libraries
     need at runtime;
   - who owns the loop, and a window showing a reloadable mod's frame;
   - per-library statics: what of wgpu can live in a resident mod and be
     used from a reloadable one, and what a presenter reload costs;
   - extract cost at 1k, 10k and 100k drawables, full against incremental;
   - draw throughput, plain against instanced, at those counts;
   - that a draw list handed between mods costs only the copy itself.
2. **Research** into decision models for playtesting, in parallel: what
   current game-playing and vision-language-action models expect
   (observations, action formats, rates), what's usable locally or by API,
   and how they've been used to playtest rather than to play.
3. **This doc becomes a proposal**, with the spike's numbers and the
   research's answers.
4. **Milestones**, each a game you can play and an agent can play:
   - **M1:** pong in a window, drawn from shapes and text, played with the
     keyboard; text and structured observations over the control socket.
   - **M2:** the MCP server, the playtest harness, the spectator window.
   - **M3:** rendering pipelined with simulation (D8).
   - **M4:** the platformer, with sprites and assets.
   - **M5:** 3D, pile3d in a window.

## What stays out

Audio, a UI toolkit, an editor, networking, and platforms other than Linux.
