# Decision models and AI playtesting: a survey

**Status: research** (2026-10-04, get-3hd.2). Evidence for
[presentation.md](presentation.md), plan step 2. Sources are linked where
they're used; nothing here was verified by running a model.


## Summary

Several kinds of model can now play games, but only two of them are worth
building for first.

1. **LLM agents through tools (Claude and others, over MCP).** These are the
   best general playtesters available today. They play unseen games with no
   training, explain what they did, and file bug reports. They need the game
   to pause while they think, and they work best from text and structured
   state, with screenshots added for visual checks. The studies disagree on
   how much screenshots help. They are slow (one to several seconds per
   decision), so they need macro steps (`step n`, or step until something
   happens) and a summary of what happened since they last looked. Claude
   Fable 5 finished Pokémon FireRed in June 2026 from screenshots alone, so
   pixel play by LLMs is now real. A year earlier it needed text maps and
   navigation tools.
2. **Search and small per-game RL on the deterministic simulation.** This is
   the industry's proven playtesting tool, for reachability, stuck spots,
   exploits and difficulty: EA SEED's RL testers, Go-Explore reachability
   testing, King's Candy Crush difficulty bots. It needs fast headless
   stepping, save and restore of a state, structured observations and a
   reward or goal signal. Bit-for-bit determinism makes this cheap for us
   and makes every finding replayable.

Generalist pixel-to-action game models (NVIDIA NitroGen, Elefant Open P2P)
are new, open, and small enough for an RTX 4090, but their own authors say
they are fast reflex policies without planning. They are trained on
commercial-game footage, and their zero-shot play on unseen games is weak.
They are a cheap experiment, not a pillar. The strongest generalists (SIMA 2,
Game-TARS, Lumine) are not available to us: early access, a closed model, or
weights that are unclear. Video world models (Genie 3, Muse/WHAM,
Matrix-Game, Dreamer 4) stand in for a simulator. We have a better simulator
than any of them, so they are irrelevant for testing our own games.
Robotics VLAs (π0, GR00T) need fine-tuning for each embodiment and add
nothing that a per-game policy wouldn't do better.

**The design implications in one paragraph.** The plan in presentation.md is
mostly right. Most of what needs adding is in the step and observe contract:

- `step(n)` as frame skip with held actions, plus "step until an event or
  n frames";
- an events-since-last-observation log;
- world **snapshot and restore** (or fork), the one new engine capability
  the rest depends on;
- game-declared **goals and metrics** for success, failure and reward;
- an action vocabulary that is device-free but carries default keyboard and
  gamepad bindings, plus a discrete projection;
- pixel views at a caller-chosen fixed size (square 192 or 256 for local
  models, around 1024×768 for Claude);
- a Gymnasium-shaped client, so standard RL and search tooling plugs in.

## 1. Decision models that act in games (as of October 2026)

"4090 fit" is my estimate from parameter count and the published VRAM
figures, unless a source states it. Where the information is thin, the
table says so.

| Model | Who, when | Input | Output | Availability | 4090 fit | Generality |
|---|---|---|---|---|---|---|
| **NitroGen** | NVIDIA + MineDojo, Dec 2025 (CVPR 2026) | One 256×256 RGB frame (more context didn't help) | Gamepad: 17 buttons plus 2 sticks, chunks of 16 actions, flow-matching DiT | Open weights (0.49B), NVIDIA non-commercial licence; the play script targets Windows games | Yes, easily (0.5B). Latency not published | Trained on 40k h, 1,000+ games. Zero-shot is "non-trivial"; the README says it can't plan or play wholly unseen games well. Fine-tuning gives up to 52% relative gain. Best on gamepad games (action, platformer) |
| **Open P2P** (Pixel2Play) | Elefant AI, 2026 | 192×192 frames plus a text instruction | Keyboard and mouse | Open, MIT, 150M to 1.2B; trained on 8.3k h across 40+ 3D games | Probably. Authors tested RTX 5080/5090 at under 50 ms per step | Real-time behaviour cloning; no unseen-game results in the README (thin) |
| **UI-TARS-1.5** | ByteDance Seed, Apr 2025 | Screenshot plus text | GUI actions (click, key), as text | 7B open, Apache-2.0; the larger game-tuned model is closed | 7B in bf16 is about 16 GB: fits | Large model: 100% on 14 Poki web games, Minecraft SOTA. The 7B is "not specifically optimized for game-based scenarios" |
| **Game-TARS** | ByteDance Seed, Oct 2025 | Screenshots 640×360 to 1280×720, actions every 50–100 ms | `mouseMove(dx,dy)`, `mouseClick`, `keyPress`, plus Think and No-Op | **Closed** (built on proprietary Seed-VL-1.5) | – | Near fresh-human generality on unseen web 3D games. Web games were **time-paused** for model latency |
| **Lumine** | ByteDance Seed, Nov 2025 | 720p every 200 ms (5 Hz) | Keyboard and mouse at 30 Hz as text action chunks | "Open recipe"; weights not confirmed (thin) | 7B (Qwen2-VL base) | Trained on Genshin Impact; zero-shot it completed hours of Wuthering Waves and Honkai: Star Rail |
| **SIMA 2** | Google DeepMind, Nov 2025 | 720p RGB frames plus language and images | 96 keys, mouse clicks, discretised relative mouse moves, emitted as structured text; also dialogue | Research preview: "early access to a small cohort of academics and game developers" | – | Gemini-based. 62% of tasks vs SIMA 1's 31% (humans about 70%). Generalises to held-out games and Genie 3 worlds. **Self-improves using Gemini as task setter and as a 0–100 reward judge** |
| **VPT** | OpenAI, 2022 | 128×128-ish frames from 360p at 20 Hz (Minecraft) | Keyboard and mouse | Open weights, MIT | Yes | Minecraft only |
| **Claude (Fable 5, Opus 5.x), GPT-5.x, Gemini 3** | Frontier LLMs | Text, JSON, screenshots (Claude: 28×28-pixel visual tokens, up to 2576 px long edge) | Tool calls: whatever actions the harness exposes | API | – (cloud) | Zero-shot on anything with a harness. Fable 5 beat FireRed vision-only; GPT-5 finished Pokémon Red in 6,470 steps; Gemini 3 Pro finished Crystal. Slow: about 1–5 s or more per decision |
| **Computer-use tools** (Claude computer use, Gemini 2.5 Computer Use, OpenAI CUA) | 2024–2026 | Screenshots (Claude recommends about 1024×768 or 1280×720; Gemini 1440×900) | Click, key, hold_key, drag, wait | API | – | Anthropic's docs: "not suitable for real-time games", and they recommend game APIs and direct state access instead |
| **World models**: Genie 3, Muse/WHAM, Matrix-Game 2/3, Dreamer 4, GameNGen | DeepMind, Microsoft, Skywork, DeepMind | Frames plus actions | Generated frames (WHAM also generates controller actions) | Genie 3: Google AI Ultra product only. WHAM: open, research licence, 300×180 at 10 Hz, "too slow for real-time". Matrix-Game: open. Dreamer 4: paper plus unofficial reimplementations | WHAM needs 4.5 GB; the others vary | They *replace* a simulator (Dreamer 4 trains agents inside a learned Minecraft). We have the real, deterministic one, so they are irrelevant for testing our games |
| **Robotics VLAs**: π0/π0.5 (openpi), GR00T N1.6 | Physical Intelligence, NVIDIA | Camera images, proprioceptive state, language | Continuous action chunks (GR00T at 76–80 Hz on an RTX 5090) | Open (Apache-2.0 / NVIDIA) | π0 inference needs more than 8 GB and LoRA fine-tuning more than 22.5 GB, both "RTX 4090". GR00T needs 16 GB or more | **A new embodiment needs fine-tuning**. Not a playtesting tool; the useful idea for us is action chunking |

**On the sources.** One 2026 survey ("Towards Generalist Game Players",
arXiv 2605.09965) marks SIMA 2 and Game-TARS as "open". Their primary
sources contradict it: the SIMA 2 paper says early access only, and the
Game-TARS paper releases no weights. I went with the primary sources.

## 2. AI playtesting, as opposed to playing

**Reachability, stuck spots and exploits, using search or RL on internal
state.** This is the most mature line of work, and it works.

- **EA SEED, 2020.** RL agents on a gamepad-like continuous action space,
  with action repeat 3. Observations were a vector of relative goal
  position, velocity, rotation, contact flags and 12 raycasts, normalised to
  [-1, 1]. The reward was progress toward a goal. The agents found a missing
  collision mesh (walking through a wall), five stuck spots, and areas the
  navmesh couldn't reach. Throughput was 10,000 interactions a second on
  four machines, "equivalent to 3000 human players".
- **EA SEED, 2021.** Curiosity rewards for state coverage, with heatmap
  visualisations so designers can see where the agents went.
- **EA SEED, AAA deployment (Battlefield 2042, Dead Space).**
  - State vectors, not pixels: "rendering viewports for all of them is
    intractable".
  - Running faster than real time broke physics: parked helicopters fell
    through the ground.
  - Inference had to take under 100 µs, so models were small.
  - RL complemented the scripted bots rather than replacing them.
  - Recommendations to engine makers: fast-forward with stable physics,
    and cheap access to full game state.
- **Go-Explore for reachability testing (Microsoft Research, 2022).** It
  saves checkpoints of distinct "cells" and restores them to explore
  further. It covered a 1.5 km × 1.5 km map in 10 hours on one machine and
  beat curiosity-driven RL by orders of magnitude in coverage. Its key need
  is **saving and restoring state**.

**Difficulty and balance.** King trained a CNN on player data to imitate
human moves in Candy Crush. Measuring a level's difficulty went from a week
to minutes, and the estimates correlated better with real players than MCTS
did. Politowski et al. (2023) compared difficulty between versions of a
platform game, and skill against luck, with autonomous agents. RuleSmith
(2026) uses multi-agent LLMs for balancing.

**LLM agents as testers.**

- **TITAN (2025).** An LLM agent for MMORPG testing that abstracts game state
  into a structured form, prioritises actions, and uses LLM oracles for
  logic bugs. It reached 95% task success, found 4 unknown bugs, and is
  "deployed in eight real-world game QA pipelines".
- **GBQA (2026).** 30 games and 124 verified bugs. The best model
  (Claude 4.6 Opus, thinking) found only 48% of them. Autonomous bug-finding
  remains hard.
- **PlaytestArena / Play2Code (2026).** A GUI agent judges generated games
  against rubrics of expected in-play behaviour. Feeding its reports back
  into the code gave a 66.8% rubric pass rate.

**Industry (Naavik, July 2026).** Black-box, vision-based QA with no
integration dominates commercial adoption (modl.ai, nunu.ai). The proven
wins are repetitive, low-judgment work: regression checks, triage, test
generation. "Whether a game is good" stays human. The vendor numbers (30%
cost cuts, EA's "85%") are vendor or secondhand claims.

**What worked, and what it needed.**

- Structured state beat pixels whenever the tester was inside the engine.
- Snapshot and restore enabled exploration.
- Many fast headless instances enabled RL.
- A goal or reward signal plus telemetry (positions visited, events) turned
  runs into findings.
- Human-readable reports and reproduction steps made findings actionable.

Our deterministic replays give reproduction for free. Black-box vision QA
exists because studios can't change their engines; we can.

## 3. LLM agents playing and testing through tools

**Harnesses that worked.**

- **Claude Plays Pokémon.** Button-press tools plus a `wait`, a knowledge
  base for notes, and screenshots. Earlier Claude models needed text
  navigation, tile labels and coordinate tracking. Claude Fable 5 finished
  FireRed from "raw game screenshots — with no maps, navigation aids, or
  extra game-state information" (Anthropic, June 2026); Anthropic also
  showed it playing Slay the Spire and Factorio.
- **Gemini Plays Pokémon.** Screenshots overlaid with extra information,
  RAM text, a fog-of-war "mental map", notepad and markers, code execution
  and custom pathfinding tools, and summarisation every 100 actions.
  Lesson: tools built on wrong assumptions (pathfinding that ignored
  off-screen NPCs) trapped the agent, and leaning on RAM text reduced its
  use of vision.
- **lmgame-Bench.** A perception module (a grid turned into a text table of
  coordinates) helps spatial games. A memory module (the last N states plus
  reflections) helps long-horizon ones. Both together are best.
- **PokeAgent (NeurIPS 2025).** "A harness … is a prerequisite for
  progress", and RL specialists still beat LLMs in competitive play.

**Observation formats: the evidence is mixed.**

- **BALROG.** Adding images *hurt* GPT-4o (32% to 23%) and Llama 3.2. Claude
  3.5 Sonnet and Gemini 1.5 Pro held steady.
- **TowerMind.** Adding vision *helped* most models.
- **GVGAI-LLM.** ASCII grids plus coordinate tags helped only slightly. The
  failures were coordinate confusion (rows and columns flipped), wrongly
  judged distances, and no algorithmic path planning.
- **Insights into Opus 4.5.** Inattentional blindness, and "hallucinating …
  objects … if he really wants it to be there". Explicit labels and
  coordinates help.

My conclusion: give **structured state as the primary view, a text grid for
layout, and pixels optionally** for visual bugs and as models improve. Give
labelled coordinates, a legend, and an explicit list of events.

**Latency.** Real-time play fails: VideoGameBench's best models completed
0.48% real-time and 1.6% with the game paused. Game-TARS paused its web
games, and NitroGen's wrapper "intercepts the game engine's system clock"
for frame-by-frame play. Every serious harness pauses. That validates D5's
lockstep.

**Tool shape.** BALROG answers invalid actions with feedback and a no-op
fallback. Claude's computer use costs about 1,000–1,800 tokens per
screenshot and advises keeping 20 or fewer images in context. Its vision
docs warn that images under 200 px are unreliable. Godot already has
several MCP servers (screenshot, simulated input, scene-tree inspection,
assertions), so the engine-side MCP pattern is established.

## 4. What this asks of our design (mapped onto D3, D4, D5, D7)

### D3: views

- **Structured.**
  - Contents: stable entity ids across frames; the semantic label (D1);
    position, velocity, size or shape; and game-declared state (score,
    lives, goals).
  - Two encodings: JSON for LLMs, and a fixed-length float vector (with a
    schema) for RL. TowerMind exposes both, and EA used normalised vectors.
- **Text grid.**
  - Axis coordinates on the border, plus a legend generated from the
    labels.
  - Below the grid, the same entities as a list with exact coordinates. The
    grid gives layout; the list fixes the row and column errors.
  - The caller chooses the cell size and region (camera, or whole level).
- **Pixels.**
  - Deterministic RGB8 (no alpha) at a **caller-chosen size**, letterboxed
    to keep the camera's aspect.
  - Presets: 256×256 (NitroGen), 192×192 (P2P), 84×84 or 64×64 greyscale
    (classic RL), about 1024×768 or 1280×720 for VLMs. Claude's cost is
    ⌈w/28⌉·⌈h/28⌉ tokens, so multiples of 28 waste nothing.
  - Encodings: raw bytes for local models, PNG for LLMs (never lossy).
  - An optional **label overlay** that draws the D1 labels and ids on the
    image, as the Gemini harness did. It costs nothing in the presenter and
    helps VLMs ground what they see.
- **Events since the last observation**, available in every view: score
  changed, collision, death, level change, entity spawned or despawned. A
  model stepping 30 frames at a time misses things otherwise.

### D4: actions

- **Each action declares:** a name, a description of what it means, and a
  kind. The kind is either a button (pressed or held) or an axis (a
  continuous value in [-1, 1]). The declaration also says whether the
  action is **held until changed** (like pong's `up`, `down`, `stay`) or
  per-frame.
- **A discrete projection,** generated from the declaration (button
  combinations, axes bucketed). LLMs and most RL libraries want a small
  discrete set.
- **Default device bindings** (keyboard and a standard gamepad layout) in
  the declaration. A game still never reads a device. Device-trained models
  (NitroGen's 17 buttons and 2 sticks, P2P's keyboard and mouse) then drive
  any game through the same bindings a human uses.
- **Invalid actions are answered, not ignored:** an error plus a no-op.
- **The recording** keeps the source of each action (keyboard, agent,
  replay) and, for agents, their notes and findings, so a replay is also a
  bug report.

### D5: time

- **`step(n)` is the frame skip:** held actions repeat for n frames, which
  is exactly ALE frame skip or EA's action repeat.
- **Add `step(until = event | condition, max = n)`.** This is the macro step
  that saves LLM turns ("run until the ball crosses my half or 120 frames").
- **Optional sticky actions** (with probability p, repeat the previous
  action), as in Machado et al. 2018. They stop RL policies memorising a
  deterministic game. Off by default, and seeded, so runs stay replayable.
- **Headless speed.** Lockstep with no presenters except the requested
  view, and no sleeping.
  - Targets: in-process, thousands of frames a second for pong-sized games.
    Over the socket, round-trips for 10 or more environments in parallel
    (several engine processes).
  - Reference points: PufferLib environments reach 1M steps/s, and EA got
    10k interactions/s.
  - Lockstep must never mean slower physics: AAA testers found
    faster-than-real-time stepping broke physics, and our fixed-step
    determinism should already rule that out. Pin it with a test.

### D7: the agent interface

- **Snapshot and restore of the world (and fork).** Go-Explore, savestate
  playtests ("start at the boss"), RL resets to curriculum states, and
  "try both choices" all need it. It is the one new engine capability here.
  The machinery for bytes-level state migration across reloads may make it
  tractable.
- **`reset(seed, options)`** returns an observation and an info record.
  `step` returns `(obs, reward, terminated, truncated, info)`.
  Gymnasium's shape is the lingua franca: NitroGen's wrapper and most RL
  and search libraries speak it. A thin Gymnasium client over the control
  socket is an adapter, not part of the engine.
- **Goals and metrics, declared by the game:** named success and failure
  predicates (`level_complete`, `player_dead`), progress measures (score,
  distance to goal) and an objective in prose.
  - The harness derives reward and termination from them, so no reward is
    hard-coded in the engine.
  - **Truncation for no progress** (no metric change for N frames) is a
    cheap softlock detector.
  - For fuzzy goals, an LLM judge scores the recorded run against a rubric,
    as SIMA 2 (Gemini, 0–100) and PlaytestArena do.
- **Telemetry.** Positions visited and events, collected per run, for
  coverage heatmaps (EA 2021).
- **MCP layer:**
  - the tools: `observe`, `act`, `step`, `actions`, `reset`, `save`,
    `load`;
  - plus `note` and `report(finding)`, so findings land in the recording;
  - plus `export_replay`, to turn a run into a regression test, as
    `platformer_test` does by hand today.

## 5. Recommendation

**Support first, in this order.**

1. **LLM agents over MCP with structured and text views (M1/M2).** These
   give zero-shot playtesting of any game, and they extend what pong and the
   platformer already do by hand. They need D7 as sketched, plus held
   actions, `step until`, and the events log.
2. **Snapshot and restore, plus fast headless stepping**, then a
   **Go-Explore-style reachability search** over structured-state cells for
   the platformer: completability, stuck spots, out-of-bounds. This is the
   highest value for the cost. Industry has proven it, and determinism makes
   every finding a replay.
3. **The Gymnasium client and small per-game RL** for balance and difficulty
   (pong: win rate against paddle speed). Off-the-shelf PPO runs on the
   4090.
4. **Pixel views** at fixed sizes, for VLM visual checks (rendering bugs,
   readability) and as frontier vision improves (Fable 5 already plays from
   pixels).

**Defer or skip.** World models: we have the simulator. Robotics VLAs: each
embodiment needs fine-tuning. Computer-use agents: our own observe and act
is strictly better than screen-scraping our own window. SIMA 2, Game-TARS
and Lumine: not obtainable.

**Small experiments once M1 exists** (pong in a window, text and structured
views on the control socket):

- **E1. Claude plays pong** through a minimal MCP shim. Compare the text
  view, the structured view, and both. Vary `step n` (or `until`). Measure
  return rate, points won, tokens and turns per point. This answers which
  view to make primary and what step granularity LLMs need.
- **E2. Throughput.** Headless frames per second in-process and over the
  socket, and how many engine processes run in parallel on this machine.
  This decides whether RL and search go through the socket or need an
  in-process batch path.
- **E3. A random and scripted baseline with goal metrics.** Declare pong's
  goals (point won, point lost) and run a random agent through the
  Gymnasium client. This validates reward and termination before any RL.
- **E4** (once the pixel presenter exists). **NitroGen zero-shot** on pong
  at 256×256, with the paddle axis bound to the left stick's Y. Then a short
  fine-tune on recorded human and agent play. Expect weak zero-shot play,
  since flat shapes are far from its training data. The point is to test
  the device-binding path cheaply: a 0.5B model on the 4090.

## Sources

Models
- SIMA 2 paper: https://arxiv.org/abs/2512.04797 (HTML: https://arxiv.org/html/2512.04797)
- SIMA 1 blog: https://deepmind.google/blog/sima-generalist-ai-agent-for-3d-virtual-environments/
- NitroGen model card: https://huggingface.co/nvidia/NitroGen ; repo: https://github.com/MineDojo/NitroGen ; paper: https://arxiv.org/abs/2601.02427 (HTML: https://arxiv.org/html/2601.02427) ; CVPR 2026: https://openaccess.thecvf.com/content/CVPR2026/html/Magne_NitroGen_An_Open_Foundation_Model_for_Generalist_Gaming_Agents_CVPR_2026_paper.html
- Game-TARS: https://arxiv.org/abs/2510.23691 (HTML: https://arxiv.org/html/2510.23691)
- UI-TARS-1.5-7B: https://huggingface.co/ByteDance-Seed/UI-TARS-1.5-7B ; https://github.com/bytedance/ui-tars
- Lumine: https://arxiv.org/abs/2511.08892
- Open P2P: https://github.com/elefant-ai/open-p2p/ ; https://huggingface.co/elefantai/open-p2p ; dataset https://huggingface.co/datasets/elefantai/p2p-full-data
- VPT: https://github.com/openai/Video-Pre-Training
- Muse / WHAM: https://huggingface.co/microsoft/wham ; https://www.microsoft.com/en-us/research/publication/world-and-human-action-models-towards-gameplay-ideation/
- Genie 3: https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/ ; availability (secondary): https://en.wikipedia.org/wiki/World_model_(artificial_intelligence)
- Dreamer 4: https://arxiv.org/abs/2509.24527 ; unofficial code e.g. https://github.com/nicklashansen/dreamer4
- Matrix-Game 2.0/3.0: https://arxiv.org/abs/2508.13009 ; https://github.com/SkyworkAI/Matrix-Game
- openpi (π0, π0.5): https://github.com/Physical-Intelligence/openpi
- GR00T N1.6: https://huggingface.co/nvidia/GR00T-N1.6-3B ; latency thread: https://forums.developer.nvidia.com/t/real-time-inference-on-thor-rtx-pi0-5-gr00t-n1-6-1-7-thor-23-hz-rtx-5090-50-80hz/368788
- Survey, 2026: https://arxiv.org/html/2605.09965v1 ; AI for Games in the Foundation Model Era: https://arxiv.org/abs/2609.16679

LLMs playing games
- Claude Fable 5 announcement (FireRed vision-only, Slay the Spire, Factorio): https://www.anthropic.com/news/claude-fable-5-mythos-5 ; coverage: https://www.shacknews.com/article/149574/claude-ai-pokemon-firered-fable-5
- Claude Plays Pokémon: https://x.com/AnthropicAI/status/1894419011569344978 ; https://techcrunch.com/2025/02/25/anthropics-claude-ai-is-playing-pokemon-on-twitch-slowly ; harness write-up: https://michaelyliu6.github.io/posts/claude-plays-pokemon/
- Insights into Claude Opus 4.5 from Pokémon: https://www.lesswrong.com/posts/u6Lacc7wx4yYkBQ3r/insights-into-claude-opus-4-5-from-pokemon
- Gemini Plays Pokémon: https://blog.jcz.dev/the-making-of-gemini-plays-pokemon ; https://blog.jcz.dev/gemini-3-pro-vs-25-pro-in-pokemon-crystal ; https://www.dbreunig.com/2025/06/17/an-agentic-case-study-playing-pok%C3%A9mon-with-gemini.html
- GPT-5 Pokémon: https://x.com/OpenAIDevs/status/1953551715543744547 ; https://www.techradar.com/ai-platforms-assistants/chatgpt/gpt-5-just-completed-pokemon-red-in-a-new-world-record-time-claude-gemini-and-chatgpt-o3-arent-even-close
- PokeAgent Challenge: https://arxiv.org/html/2603.15563v1
- VideoGameBench: https://arxiv.org/abs/2505.18134
- lmgame-Bench: https://arxiv.org/abs/2505.15146
- BALROG: https://arxiv.org/abs/2411.13543 (HTML: https://arxiv.org/html/2411.13543v1)
- TowerMind: https://arxiv.org/html/2601.05899
- GVGAI-LLM: https://arxiv.org/html/2508.08501v2
- Spatial reasoning in LLM game agents: https://arxiv.org/abs/2607.22732
- Claude computer use tool: https://platform.claude.com/docs/en/agents-and-tools/tool-use/computer-use-tool ; vision limits and token cost: https://platform.claude.com/docs/en/build-with-claude/vision
- Gemini Computer Use: https://ai.google.dev/gemini-api/docs/computer-use
- Godot MCP servers: https://github.com/beckettlab/beckett-godot-mcp ; https://mcpservers.org/servers/erodenn/godot-mcp-runtime

Playtesting
- EA SEED, Augmenting Automated Game Testing with Deep RL (2020): https://arxiv.org/abs/2103.15819 ; https://www.ea.com/seed/news/automated-game-testing-deep-reinforcement-learning
- EA SEED, Curiosity-driven playtesting coverage (2021): https://arxiv.org/abs/2103.13798
- EA SEED, Technical Challenges of Deploying RL Agents for Game Testing in AAA Games: https://arxiv.org/abs/2307.11105
- Go-Explore for reachability testing: https://arxiv.org/abs/2209.00570 ; https://www.microsoft.com/en-us/research/publication/go-explore-complex-3d-game-environments-for-automated-reachability-testing/
- King, Human-Like Playtesting with Deep Learning: https://dl.acm.org/doi/10.1109/CIG.2018.8490442 (PDF: https://gwern.net/doc/reinforcement-learning/imitation-learning/2018-gudmundsson.pdf)
- Politowski et al., Assessing Video Game Balance using Autonomous Agents: https://arxiv.org/abs/2304.08699
- RuleSmith: https://arxiv.org/abs/2602.06232
- TITAN, LLM agents for MMORPG testing: https://arxiv.org/abs/2509.22170
- GBQA: https://arxiv.org/abs/2604.02648
- GUI agents for continual game generation (PlaytestArena): https://arxiv.org/abs/2605.28258
- Naavik, The State of AI for Game QA (July 2026): https://naavik.co/ai-gaming/the-state-of-ai-for-game-qa/

Interfaces and conventions
- Machado et al., Revisiting the ALE (sticky actions, frame skip): https://arxiv.org/abs/1709.06009
- Gymnasium Env API: https://gymnasium.farama.org/api/env/
- PufferLib 2.0: https://rlj.cs.umass.edu/2025/papers/Paper151.html

## Where information is thin or unverified

- Lumine's weights: the paper says "open recipe", but I found no confirmed
  weight release.
- Open P2P: no unseen-game results in its README, and no 4090 numbers.
- NitroGen: no published inference latency.
- SIMA 2: the frame rate isn't stated.
- TITAN: the games aren't named.
- Commercial QA figures (nunu.ai, modl.ai, EA's 85%): vendor or secondhand.
- Genie 3's availability: from secondary sources.
- Claude Fable 5's FireRed run: Anthropic's announcement gives no duration
  or step count, and no harness details beyond "minimal, vision-only".
