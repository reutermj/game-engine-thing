# An empty HashMap points into the build that made it

An empty `std::collections::HashMap` allocates nothing: its control bytes
point at one static, all-empty control group in hashbrown
(`Group::static_empty`), and every mod links its own copy of std, so that
static is in the image of whichever build ran `HashMap::new()` (or
`Default`). Walking the map reads it, and so does `entry` (and so an
insert). An empty map made by a build and walked after that build was
unmapped segfaults, though it holds nothing and its type says it points
nowhere. (`get` and `len` don't read it: `get` checks for an empty table
first.) A map that has held something has its table on the shared heap,
which is why the games, whose maps fill on their first frame, never hit it.

A `BTreeMap` (an `Option` root when empty), a `Vec` (a dangling, aligned
address, never read) and a `String` point at nothing.

## Where it bit

Each event queue kept its readers' cursors in a `HashMap`. The queue is made
by the first build to declare the event (declarations run the ECS code
linked into the mod), so the map was that build's. It was found twice on
the same day (2026-09-25), independently:

- **pong's reload replay** reloads every mod before the first frame, so the
  build that made the queue was unmapped before anything read it, and the
  new build's first read segfaulted. Pinned by
  `a_queue_outlives_the_build_that_made_it_before_anything_read_it`
  (`//engine/tests:reload_test`).
- **The reload fuzzer's seeded driver**, in the sessions below, measured
  with its scripts (`reload_ops::script`;
  `engine/tests/fuzz/scripts/event-cursors-outlive-the-build-that-made-them.txt`
  replays the first row in `//engine/tests/fuzz:replay_test`).

| session | result |
|---|---|
| herald_a loaded, unloaded, loaded again; hearer_a loaded; one frame | segfault in `hearer::early`'s first `EventReader::read` |
| the same, with the event queue's cursors in a `BTreeMap` | passes |
| keeper_a's `Item` given an empty `HashMap` field; an item spawned; keeper_b loaded (taking the component over, so keeper_a is unmapped); keeper_b walks the item's map | segfault (`get` on it passes) |
| keeper_a's state given an empty `HashMap` field; keeper_b loaded, the same state layout | `reloaded keeper (generation 1, state held pointers into the old build, so it was reset)` |

In the first row the first herald build stayed mapped while the queue's
keepalive was its; its second build's load took the queue over and
unmapped it. The last two rows were measured while `HashMap` was still a
`FieldType`, and are why it isn't one now.

## What it means

- **Anything a mod's code makes that outlives the build is data only if
  its empty value holds no pointer into static memory.** `Vec`, `String`,
  `Box` and `BTreeMap` qualify; `HashMap` and `HashSet` don't. The queue's
  cursors are a `BTreeMap` now.
- **So `HashMap` is not a `FieldType`.** In a component, an empty map made
  by a build's `Default` crashes the first walk after that build is gone
  (third row). In a mod's state, the loader's scan for pointers into the
  old image finds one at the top level and resets the state on every
  reload (fourth row); one inside a `Vec` it can't see, and that would
  crash like the component. Giving it a representation that owns its empty
  table was the alternative; removing it closes the class, and `BTreeMap`
  does the same job.
- **World structures the loader makes may use `HashMap`.** `World`'s own
  maps are made in `World::new`, by the loader, whose image never goes. One
  made on first use may be made by a mod: check which side runs `new`.
- It's the one-compiler rule's blind spot: the rule makes std types'
  layouts agree between builds, but a value can still point into the image
  that made it without holding any code.
- "It didn't crash" means nothing unless the maker was really unmapped
  first (see [dlclose-unmaps-a-mod-nothing-holds.md](dlclose-unmaps-a-mod-nothing-holds.md)).
