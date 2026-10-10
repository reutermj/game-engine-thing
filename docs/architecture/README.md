# Architecture docs

Design documentation for the engine's major components and decisions. Each
file covers one area; keep them focused rather than growing a single
monolithic design doc.

## Index

- [overview.md](overview.md) — everything is a mod: what the loader does,
  what the bootstrap mod does, and why Bazel is the reload trigger
- [hot-reload.md](hot-reload.md) — the mod ABI, who owns state, the reload
  sequence, and the control protocol
- [ecs.md](ecs.md) — the world every mod shares: why an ECS fits hot reload,
  why the loader owns it, and what a component layout change does
- [mod-deps.md](mod-deps.md) — how a mod uses another mod's components, how
  the engine knows, and why interface changes reload through the game
- [scheduling.md](scheduling.md) — systems, phases and their order, what a
  system's parameters declare, events, and the steps toward multithreading
- [storage.md](storage.md) — archetype tables in pages, sparse sets, and
  structural change and events without stop-the-world points
- [spatial-storage.md](spatial-storage.md) — tables kept in
  spatial order, so pages are neighborhoods and region queries are queries
- [live.md](live.md) — live relations, derived results storage keeps
  current between frames: `Proximity` declared, taken as `Live<R>`, the
  broadphase's pairs, what it costs storage, and how it could grow
- [relationships.md](relationships.md) — how entities refer to each
  other: contacts, colliders and hierarchy, measured
- [presentation.md](presentation.md) — rendering, input and AI
  playtesting as mods: games describe and presenters draw (window, CPU
  pixels, text, data), input as declared actions, one agent interface,
  every session recorded and reviewable, and the milestones (proposal)
- [playtesting-research.md](playtesting-research.md) — decision models
  and AI playtesting, surveyed with sources: what LLM agents, search and
  per-game RL, and pixel-to-action game models need from a game, and what
  that asks of presentation.md (research)
- [presentation-spike.md](presentation-spike.md) — wgpu, winit and
  tiny-skia under Bazel, a window pumped by the bootstrap through a
  resident platform mod, what of wgpu can cross mods (and the crash when
  it does), the extract full against incremental, draw throughput plain
  against instanced, and the hand-off's cost, measured (spike results)
- [parallel-relations.md](parallel-relations.md) — systems whose
  relation rows write the entities they name, across threads in colors:
  who wants it, what other engines keep in and out of the ECS, colors as
  world state, and why the solve still copies (proposed)
- [working-sets.md](working-sets.md) — whether storage should keep a
  dense working set for a system, as the solver's per-step copy: the copy
  measured eight ways, how Box2D, Rapier, Jolt and Flecs keep theirs,
  hierarchy propagation as a second user, and why an order matters more
  than a kept copy (proposed)
- [flows-spike.md](flows-spike.md) — flows, values systems hand one
  another within a frame as stages of a pipeline: how Bevy, Flecs, Unity
  and dataflow systems pass values, the mechanism built on the ECS's own
  graph, physics's solve ported onto it bit for bit with a generic colored
  primitive, hierarchy propagation as a second user (spike results;
  designed and built as [flows.md](flows.md))
- [flows.md](flows.md) — flows designed: `Make`, `See`, `Pass` and
  `Take` and their rules, edges in the graph, the world's store and its
  recycling bins across reloads, the plan check and its errors, fixed-rate
  groups, and parallel shapes (`ParMap`, `Reduce`, `Passes`) declared as
  parameters and run by the scheduler (built: `Passes` across threads)
- [dispatch-spike.md](dispatch-spike.md) — how a scheduler hands a
  `Passes` program's stages out across threads: how Box2D, Box3D, Jolt
  and Rapier dispatch staged work, five protocols run over both physics
  solves bit for bit against `run_across`, where idle time goes, two
  programs in one dispatch, and what the pool must provide (spike results;
  built as [threads.md](threads.md))
- [threads.md](threads.md) — the scheduler's threads: a resident mod's
  rayon pool on one CCD as the world's executor, why it lives there, the
  task graph `Passes` runs across it, warmth, panics and hot reload, and
  what it measures in the running engine (built)
- [contiguous-columns.md](contiguous-columns.md) — why storage can't be
  indexed as a plain array, and whether it could: a table column in one
  block or in reserved address space against today's pages, measured for
  storage and for a solve in place, and what it means for parallel
  relations' phase 2 (deferred)
- [physics.md](physics.md) — the 2D and 3D physics mods as they are, by
  topic, with every piece of state in the world; the measurements and
  decisions behind them are the dated
  [physics log](../retrospectives/2026-10-04-physics-log.md)
- [physics-sharing.md](physics-sharing.md) — the 2D and 3D physics mods
  side by side: a parity table from the code, what each piece is
  (dimension-independent, the same algorithm over other math, or its own),
  how Rapier, Box2D and Box3D, Jolt and Avian share 2D and 3D, and a shared
  implementation crate in phases (accepted; phases 1 and 2 built)
- [physics-testing.md](physics-testing.md) — how physics is kept from
  regressing: equivalence, physical law, the reference floor, and a
  baseline of our own accepted results

## Conventions

- **How to write one:** [writing.md](writing.md), the order a doc goes
  in, who it's for, and what stays out of the explanation.
- These are living design docs, not a decision log. If a design changes,
  update the doc in place rather than appending "UPDATE:" notes. An
  abandoned approach worth remembering goes in the doc's "Alternatives
  considered" (writing.md), or in [docs/lore/](../lore/) if the reason
  it failed is the valuable part.
- Mark undecided design questions as `**Open question:**` so they're easy
  to grep for. Much of the design is deliberately deferred until hot reload
  is proven; the open questions are the list of what was deferred.
