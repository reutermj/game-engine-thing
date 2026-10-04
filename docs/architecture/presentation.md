# Presentation, input and playtesting

**Status: proposal** (2026-10-04, epic get-3hd). Nothing here is built
outside the spike. This replaces the sketch of the same date[^sketch] with
what three pieces of evidence showed:

- the spike, [presentation-spike.md](presentation-spike.md): wgpu, winit
  and tiny-skia under Bazel, the loop, what crosses mods, the extract and
  draw costs, and pong in a window played by agents;
- the survey, [playtesting-research.md](playtesting-research.md):
  decision models and AI playtesting, with its sources;
- the playtest loop, presentation-spike.md section 7: recorded agent
  matches, replayed and probed, which found three real bugs (get-c3s,
  get-7la, get-lye), fixed with regression scenes.

The engine's own description says rendering, input and the game itself
are mods the loader reloads. This is how games get a window, a keyboard
and a GPU, and how **every game becomes playable, and reviewable, by AI
agents**: in lockstep, through text, data or pixels, watched in a window,
and recorded so any session can be replayed and questioned afterwards.

## Requirements

1. **Everything is a mod.** The renderer reloads while a game runs; the
   window, which can't, lives in a resident mod.
2. **Every game is agent-playtestable with no per-game glue.** One
   interface observes, acts and steps any game.
3. **Watchable.** A window can attach to an agent's lockstep run, show it
   at real-time pace, and show what the agents are doing (whose turn,
   how long they've been thinking), so a long think never looks like a
   hang.
4. **Deterministic.** Presentation never feeds back into the simulation.
   Lockstep runs and replays are bit for bit; tests and vision agents can
   get the same pixels on every machine.
5. **Recorded and reviewable.** Every session can be replayed headless,
   verified, probed with questions nobody asked while it was played, and
   turned into findings and regression tests.
6. **Fast.** No cost of the abstraction that design can't remove.
7. **Swappable.** Backends are mods behind interfaces: a game never names
   wgpu, a presenter never names a game.

## The shape

```text
game mods ──(presentation components)──► extract ──► draw list ──► window presenter (wgpu)
                                                         ├────────► pixel presenter (tiny-skia)
                                                         ├────────► text presenter
                                                         └────────► structured presenter (data)
keyboard / gamepad mod ─┐
agent (control socket, MCP) ─┼──► actions ──► game mods
replay ─────────────────┘
platform service (one provider, resident, pumped by the bootstrap): window, event loop, input
    winit | headless | none
recorder: inputs at their frames, events, notes ──► session log ──► replay, probes, detectors, review
```

## Decisions

**D1. Games describe; presenters draw.** A game attaches presentation
components: what to show (shapes, sprites, text, a camera, layers), a
**semantic label** ("player", "ball", "coin") and a stable id an agent can
follow across frames. The vocabulary lives in an interface crate and says
*what*, never *how*. Items carry a material identity, so a GPU presenter
can batch. Backend-only features are optional extensions others ignore.
This vocabulary and the action declaration (D4) are the stable contract;
everything behind them may change.

**D2. The draw list is rebuilt each frame by default.** A full extract
costs 1.1 ns a drawable (110 µs at 100k on one thread), emitted in material
runs. Incremental extraction from change ticks was 3 to 4 times *slower*
whenever games move things through `&mut` walks, since such a walk marks
every page it visits written (get-7oo), and won only below about 5% of
entities changing, written one by one (spike, section 4). So incremental is
an option, a delta flow for large sets that rarely change (a tilemap,
sleeping bodies), not the default. A kept list lives in the extract's
transient state, since flows last one frame.

**D3. Presenters read the draw list,** side by side, each a mod:

- **window** (wgpu): instanced by material. At 100k items, 0.45 ms on the
  RTX 4090 against 47.6 ms one draw an item (spike, section 5);
- **pixels** (tiny-skia): deterministic RGB8 at a size the caller picks,
  letterboxed (presets 256², 192², 84², and around 1024×768 for vision
  LLMs, in multiples of 28, the size of Claude's image patch[^claude-vision]);
  about 2 µs a shape at 720p, the same bytes every run. An optional
  overlay draws labels and ids on the image, as the Gemini harness
  did[^gemini];
- **text**: a character grid with axis coordinates and a legend from the
  labels, *and* the same entities as a list with exact coordinates, since
  models confuse rows and columns on grids alone[^spatial];
- **structured**: ids, labels, positions, velocities, sizes and the game's
  declared state, as JSON for LLMs and as a fixed-length vector with a
  schema for RL (as TowerMind and EA did[^ea]).

**Every view carries the events since the last observation** (points,
hits, deaths, spawns). An agent stepping many frames misses them
otherwise, and in the pong matches both players inferred points from
score changes and wanted hit events.

**D4. Input is declared actions.** A game reads its own actions, never a
device. Each action declares a name, what it means, a kind (button or axis
in [-1, 1]), and whether it is **held until changed** (pong's
`up`/`down`/`stay`) or per frame. From that the engine derives a discrete
set for LLMs and RL, and the declaration carries default keyboard and
gamepad bindings, so device-trained models drive any game the way a human
does. An invalid action is answered with an error and does nothing.
Sources (keyboard and gamepad through gilrs, agents, replays) are mods
writing actions; each action is recorded with its source and frame.

**D5. Time.** The bootstraps stay: realtime for a human, lockstep for an
agent.

- **`step(n)` is the frame skip:** held actions repeat for n frames, as in
  the Arcade Learning Environment[^ale].
- **`step until`:** run until an event or condition, at most n frames
  ("until the ball crosses my half"). The agents in the pong matches spent
  most of their calls on turns where nothing needed deciding.
- **Step and observe in one call.** Every player asked for it; it halves
  the calls a turn.
- **A spectator window** shows a lockstep run at real-time pace (pacing
  only delays; paced and unpaced runs end in the same state) and stays
  responsive while the game waits.
- **Optional sticky actions**, seeded and off by default, so RL policies
  can't memorise a deterministic game[^ale].
- **Headless speed:** no presenter but the view asked for, no sleeping;
  thousands of frames a second in-process for pong-sized games, and
  several engine processes in parallel for search and RL.

**D6. The platform is the window, the event loop and input.** Not the GPU
device: D11 forbids GPU state crossing mods, and the spike crashed the
engine when it did (spike, section 3). It is resident, because winit sets a
process-wide X11 error handler that points into the library that made it.
It is a one-provider service the bootstrap pumps once a frame (75 to 85 µs),
never running the loop itself, and a game picks its provider (winit,
headless or none) in `engine_game`, as it picks threads and the scheduler.
The window reaches the presenter as plain numbers (X11 handles), from
which the presenter makes its own surface.

**D7. One agent interface.** On the control socket, and an MCP server over
it so Claude and other agents playtest any game with no glue:

- `observe(view)`, `act(actions)`, `step(n | until)`, `actions()` (the
  declared actions and their meanings);
- `reset(seed)` returning an observation and info, and `step` returning
  observation, reward, terminated, truncated and info: Gymnasium's
  shape[^gym], so standard RL and search tools attach through a thin
  client, not the engine;
- `save` and `load` (D10), and `note`, `report(finding)` and
  `export_replay`, so a run's findings travel with its recording (D9);
- **every way a session ends is a reason**: game over, window closed,
  engine stopped, opponent gone. In the matches an agent only ever saw
  "can't reach the engine".

**Games declare goals and metrics** beside their actions: named success
and failure conditions, progress measures and an objective in prose.
Reward and termination are derived from them, so the engine hard-codes no
reward; "no progress for N frames" truncates a run and flags a softlock;
fuzzy goals are scored afterwards by an LLM judge against a rubric, as
SIMA 2's evaluation did[^sima2]. Games may also declare **agent helpers**,
values an agent would otherwise compute badly by hand. Both pong players
wrote scripts to predict the ball's crossing point after their mental
arithmetic failed.

**D8. Several agents share time by a policy, per session.** The spike's
**turn barrier** (each side submits, the turn plays when all have)
is fair but runs at the slowest thinker: in the first two-agent match a
0.1-second turn waited tens of seconds. A **deadline** (a side's last
action stands if it hasn't submitted in time) keeps time moving and
penalises slow thinkers. **Longer turns** and plans ("move to y 12") cut
decisions. The session picks; the spectator shows which side is awaited
and for how long.

**D9. Every session is recorded and reviewable.** This is the loop the
spike built and the matches proved:

- **record**: the build, the inputs with the frame each took effect,
  events, periodic state checks and agents' notes; a few KB a game;
- **replay** headless, verified against the recorded checks (a log with
  one input moved a frame is caught), about 0.1 ms a frame;
- **probe**: per-frame data read from the world by the replay tool,
  including questions nobody asked during play;
- **detect**: flags with a frame and numbers (a hit that didn't bounce,
  penetration, a ball beyond a wall, speed lost);
- **review**: an agent replays to each flag, renders deterministic frames
  and clips, and reports with evidence;
- **close the loop**: a finding becomes a bead with its recording, and a
  regression scene where the code that's wrong is tested.

The matches showed why. A person saw the ball sink into the paddles, an
agent suspected its points were a collision bug, its opponent blamed its
own play, and neither could be checked until a recording existed. Replayed,
the detectors found pong's end-hit rule (get-c3s), its speed cap (get-7la)
and the physics' missing contact (get-lye), all three fixed.

**A recording replays exactly only while the code it ran on is unchanged**:
after a fix, the same inputs make a different game, since the agents
reacted to the old one. So a corpus diagnoses before a fix and re-measures
after; a finding becomes permanent through a regression scene or a snapshot
(D10).

**D10. Snapshot and restore of the world.** The one new engine capability
the research found playtesting needs[^goexplore]: reachability search
(Go-Explore covered a 1.5 km map in 10 hours on one machine), starting a
playtest from a saved state, RL resets, and pinning a finding across code
changes. The world already migrates its values as bytes across reloads,
which may make it tractable. Its own design.

**D11. GPU state never crosses mods.** Each mod is its own library with its
own copy of wgpu's statics, and `Crossing<'call>` lets only owned data
cross a call. A reloadable mod *can* drive a resident mod's device, until
a callback it registered outlives it (demonstrated, a crash), parking_lot
deadlocks across copies, or error scopes are lost (spike, section 3). So
all GPU work lives in the window presenter, and nothing GPU-side survives
its reload: about 0.2 s on the 4090, 0.05 s on lavapipe, development only.

**D12. Overlap after measuring.** The GPU already runs beside the next
frame once work is submitted. What pipelining (scheduling step 4) would
hide is the presenter's CPU time, 0.2 to 0.4 ms at 10k items and about
1 ms at 100k with the upload, so it waits for M1's measurement of a real
game. If built, it hands the draw list on through `Take::into_inner`
rather than a world in two versions, which would break D1.

**D13. The stack.** wgpu (Vulkan only) and winit (X11 only), as Bevy uses,
gilrs for gamepads, tiny-skia for deterministic pixels. Linux only. They
build under Bazel unpatched, and every system library is loaded at run
time, so mods link only glibc. Each mod linking wgpu is about 8.5 MB and
rebuilds in about half a second. Presenters carry their dependencies'
licence texts, as the threads mod does (CREDITS.md).

## Open questions

- **The vocabulary**, 2D first: shapes, text, sprites, camera, layers,
  labels. 3D extends it later.
- **Combining actions** from several sources at once, and analog axes.
- **Change ticks** (get-7oo): pages marked only on a real write would make
  incremental extraction, and every system like it, pay. It touches the
  unsafe core, so it's the user's decision.
- **A presenter's library stays mapped after each reload** in our builds,
  since debug assertions are on (get-3hd.9); harmless for a session, it
  grows over a long one.
- **x11-dl's build script asks the host's pkg-config** (get-3hd.8): the
  same behaviour, a different compiled crate per machine.
- **Which multi-agent policy is the default** (D8).
- **Snapshot and restore** (D10), its own design.
- **Several providers of one service** (mod-deps.md); the window plus a
  headless capture would want it.
- **Assets**: sprites need images and text needs fonts, and the engine has
  no asset loading or asset hot reload. Before the platformer's sprites.

## Milestones

Each is a game a person can play and an agent can play and review.

1. **M1. Pong, built for real** (get-3hd.3): the presentation interface
   crate and its 2D vocabulary; the extract; the wgpu presenter; the
   platform service with winit and headless providers; pong's actions
   declared, with keyboard input; the text and structured views with
   events on the control socket; recording on by default, and the replay
   tool. Measure the presenter's CPU share (D12). The spike's code is
   deleted as each piece lands.
2. **M2. The agent interface** (get-3hd.4): `step until`, step and
   observe, Gymnasium's shape, goals and metrics, the MCP server, the
   spectator status, the multi-agent policies, and the review workflow.
   Then the research's first experiments: Claude playing pong with each
   view and step size, headless throughput, and a random agent checking
   pong's declared goals[^experiments].
3. **M3. Snapshot and restore** (D10), and a reachability search over the
   platformer's levels: can each be completed, where do players stick or
   fall out.
4. **M4. The platformer in a window** (get-3hd.6), with sprites and assets.
5. **M5. Overlap** (get-3hd.5), if M1's measurement asks for it.
6. **M6. The pixel presenter** for vision models, and a zero-shot trial of
   an open pixel-to-action model on pong[^nitrogen].
7. **M7. 3D**, pile3d in a window (get-3hd.7).

## What stays out

Audio, a UI toolkit, an editor, networking, platforms other than Linux,
and world models or robotics models as playtesters (we have the
simulator; playtesting-research.md, section 5).

[^sketch]: *(History, 2026-10-04.)* The sketch put the GPU device in the
    platform (D6), made the extract incremental by default (D2) and
    planned overlap through the extract up front (D8). The spike reversed
    all three: the device can't cross mods, a full rebuild is cheaper in
    most cases, and the GPU already overlaps. It also closed two open
    questions: the bootstrap owns the loop, and nothing GPU-side survives
    a presenter reload. The sketch's text is in git, at commit `16be7f4`.
[^claude-vision]: Claude's image cost is ⌈w/28⌉·⌈h/28⌉ tokens:
    <https://platform.claude.com/docs/en/build-with-claude/vision>.
[^gemini]: Gemini Plays Pokémon's harness:
    <https://blog.jcz.dev/the-making-of-gemini-plays-pokemon>.
[^spatial]: Spatial reasoning in LLM game agents:
    <https://arxiv.org/abs/2607.22732>; and TowerMind,
    <https://arxiv.org/html/2601.05899>.
[^ea]: EA SEED, Augmenting Automated Game Testing with Deep RL:
    <https://arxiv.org/abs/2103.15819>.
[^ale]: Machado et al., Revisiting the Arcade Learning Environment (frame
    skip, sticky actions): <https://arxiv.org/abs/1709.06009>.
[^gym]: Gymnasium's `Env` API: <https://gymnasium.farama.org/api/env/>.
[^sima2]: SIMA 2: <https://arxiv.org/abs/2512.04797>.
[^goexplore]: Go-Explore for automated reachability testing:
    <https://arxiv.org/abs/2209.00570>.
[^experiments]: playtesting-research.md, section 5, experiments E1 to E3.
[^nitrogen]: NVIDIA NitroGen: <https://huggingface.co/nvidia/NitroGen>;
    playtesting-research.md, section 5, experiment E4.
