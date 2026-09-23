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
- [scheduling.md](scheduling.md) — systems, phases and their order, checked
  access, commands and events, and the steps toward multithreading

## Conventions

- These are living design docs, not a decision log. If a design changes,
  update the doc in place rather than appending "UPDATE:" notes. An
  abandoned approach worth remembering goes in a dated footnote, or in
  [docs/lore/](../lore/) if the reason it failed is the valuable part.
- Mark undecided design questions as `**Open question:**` so they're easy
  to grep for. Much of the design is deliberately deferred until hot reload
  is proven; the open questions are the list of what was deferred.
