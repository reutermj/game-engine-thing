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
- [parallel-relations.md](parallel-relations.md) — systems whose
  relation rows write the entities they name, across threads in colors:
  who wants it, what other engines keep in and out of the ECS, colors as
  world state, and why the solve still copies (proposed)
- [working-sets.md](working-sets.md) — whether storage should keep a
  dense working set for a system, as the solver's per-step copy: the copy
  measured eight ways, how Box2D, Rapier, Jolt and Flecs keep theirs,
  hierarchy propagation as a second user, and why an order matters more
  than a kept copy (proposed)
- [contiguous-columns.md](contiguous-columns.md) — why storage can't be
  indexed as a plain array, and whether it could: a table column in one
  block or in reserved address space against today's pages, measured for
  storage and for a solve in place, and what it means for parallel
  relations' phase 2 (proposed)
- [physics.md](physics.md) — 2D rigid bodies as an engine mod,
  with every piece of state in the world
- [physics-sharing.md](physics-sharing.md) — the 2D and 3D physics mods
  side by side: a parity table from the code, what each piece is
  (dimension-independent, the same algorithm over other math, or its own),
  how Rapier, Box2D and Box3D, Jolt and Avian share 2D and 3D, and a shared
  implementation crate in phases (proposed)
- [physics-testing.md](physics-testing.md) — how physics is kept from
  regressing: equivalence, physical law, the reference floor, and a
  baseline of our own accepted results

## Conventions

- These are living design docs, not a decision log. If a design changes,
  update the doc in place rather than appending "UPDATE:" notes. An
  abandoned approach worth remembering goes in a dated footnote, or in
  [docs/lore/](../lore/) if the reason it failed is the valuable part.
- Mark undecided design questions as `**Open question:**` so they're easy
  to grep for. Much of the design is deliberately deferred until hot reload
  is proven; the open questions are the list of what was deferred.
