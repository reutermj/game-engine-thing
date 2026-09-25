# An empty HashMap points into the build that made it

An empty `std::collections::HashMap` allocates nothing: its control bytes
point at a static in hashbrown (`Group::static_empty`), and every mod links
its own copy of std, so that static is in the image of whichever build ran
`HashMap::new()` (or `Default`). Walking the map reads it, and so does
`entry`. An empty map made by a build and walked after that build was
unmapped segfaults, though it holds nothing and its type says it points
nowhere. (`get` and `len` don't read it: `get` checks for an empty table
first.)

Found by the reload fuzzer's seeded driver (2026-09-25), in the world
itself, and measured with the fuzzer's scripts (`reload_ops::script`):

| session | result |
|---|---|
| herald_a loaded, unloaded, loaded again; hearer_a loaded; one frame | segfault in `hearer::early`'s first `EventReader::read` |
| the same, with the event queue's cursors in a `BTreeMap` | passes |
| keeper_a's `Item` given an empty `HashMap` field; an item spawned; keeper_b loaded (taking the component over, so keeper_a is unmapped); keeper_b walks the item's map | segfault (`get` on it passes) |
| keeper_a's state given an empty `HashMap` field; keeper_b loaded, the same state layout | `reloaded keeper (generation 1, state held pointers into the old build, so it was reset)` |

The first row: an event queue is made by the first build to declare the
event (declarations run the ECS code linked into the mod), and its reader
cursors were a `HashMap`, which the reader's `entry` walks. The first herald
build stayed mapped while the queue's keepalive was its; its second build's
load took the queue over and unmapped it.

A `BTreeMap` (an `Option` root when empty), a `Vec` (dangling, never read)
and a `String` point at nothing.

## What it means

- **World structures that a mod's code can create hold no std hash maps.**
  The queue's cursors are a `BTreeMap` now
  (`engine/tests/fuzz/scripts/event-cursors-outlive-the-build-that-made-them.txt`
  replays the crash). The world's own maps are made by the loader in
  `World::new`, and a map allocates on the shared heap once something is
  inserted.
- **`HashMap` is a `FieldType`, and it isn't safe as one.** An empty map in
  a component, made by a build's `Default`, crashes the first walk after
  that build is gone; in a mod's state, the loader's scan for pointers into
  the old image finds it and resets the state on every reload. Not fixed:
  whether to drop `HashMap` from `FieldType`, or give it a representation
  that owns its empty table, is a design decision (see the open question in
  [ecs.md](../architecture/ecs.md#components)).
- It's the one-compiler rule's blind spot: the rule makes std types'
  layouts agree between builds, but a value can still point into the image
  that made it without holding any code.
