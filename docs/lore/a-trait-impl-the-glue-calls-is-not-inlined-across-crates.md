# A trait impl the glue calls is not inlined across crates

Found 2026-09-24 reading the disassembly of `libphysics_mod.so`
(`objdump -d -C`), and measured with `./bazel run -c opt
//engine/std/physics:tax`.

`engine_ecs`'s spatial glue, `__bounds::<Position>`, loops over a page's
rows calling `<Position as SpatialKey>::bounds` for each. The glue is
generic, so it's compiled in the mod's crate; `bounds` is an ordinary
method of `physics`'s interface crate, a different crate, and not tiny
(a match on the collider's shape and a box), so rustc didn't inline it
across the crate boundary. Worse, in the mod's shared library it was
called through the GOT (`call *...(%rip)`): one real call per row, with
the loop's state saved around it, for every body re-bounded every step.

`#[inline]` on the impl's `bounds` put it in the loop (no calls left in
the glue). At 10 000 bodies, what `:tax` counts outside the systems (almost
all the re-sort) went from 134 to 123 µs a step falling and from 84 to 74
settled (medians of three runs).

The same holds for `OrderKey::key`, which the ordered tables' glue calls
per row, and for any small trait method an engine loop calls generically:
the trait's own crate can't force it inline, so the implementing crate has
to mark it. `SpatialKey` and `OrderKey` say so in their docs. Rust inlines
only trivial functions across crates by itself; a function with a branch
in it isn't one.
