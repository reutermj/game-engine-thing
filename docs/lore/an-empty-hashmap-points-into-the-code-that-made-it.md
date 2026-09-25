# An empty HashMap points into the code that made it

A `HashMap` that has never held anything owns no heap memory: its table
points at one static, shared, all-empty control group in the binary whose
code made it (hashbrown's `Group::static_empty`; std is linked into every
mod, so every mod has its own). Once that mod's build is unmapped, the map
is a pointer into nothing, and the first lookup or insert, which reads the
control bytes, crashes.

## Where it bit

Each event queue kept its readers' cursors in a `HashMap`. The queue is made
by the first build to declare the event, from that mod's copy of
`engine_ecs`; if that build was reloaded before anything read the queue
(pong's reload replay reloads every mod before the first frame), the first
read of the new build segfaulted. Measured: the integration test
`a_queue_outlives_the_build_that_made_it_before_anything_read_it`
(//engine/tests:reload_test) dies with a segfault with the cursors in a
`HashMap`, and passes with them in a `BTreeMap`, whose empty value is a
null root and points nowhere. A map that has held something has its table
on the heap, which is why the games, reloading after their first frame,
never hit it.

## What it means

- Anything a mod's code makes that outlives the build is data only if its
  empty value holds no pointer into static memory. `Vec`, `String`, `Box`
  and `BTreeMap` qualify (an empty `Vec` is a dangling, aligned address,
  not an address in the image); `HashMap` and `HashSet` don't. So
  `HashMap` is no longer a `FieldType`: in a component or a mod's state it
  would be the same trap (the state's pointer scan would catch one at the
  top level and reset the state; one inside a `Vec`, or in a component,
  nothing would).
- Structures the world keeps (`World`'s own maps) are made by the loader,
  whose image never goes, so they may use `HashMap`. One made on first
  use may be made by a mod: check which side runs `new`.
- "It didn't crash" means nothing unless the maker was really unmapped
  first (see [dlclose-unmaps-a-mod-nothing-holds.md](dlclose-unmaps-a-mod-nothing-holds.md)).
=====END
