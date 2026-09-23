# ECS

The world is an entity/component store that every mod shares and no reload
touches. The code is in `engine/api/ecs.rs` (the ABI and the typed API mods
use) and `engine/loader/world.rs` (the storage).

## Why an ECS fits hot reload

Hot reload works when code and data are separate: code can be swapped, and
data has to survive the swap. An ECS is that separation as a design rule.
Components are plain data with no behavior, and systems are functions over
them with little or no state of their own. So in this engine:

- **Components live in the loader's world.** They outlive every build of
  every mod, and every mod sees the same entities.
- **Systems are mods.** `mods/physics` has no state at all; reloading it
  changes how entities move, and nothing else.
- **Per-mod state (`Mod`) remains** for data genuinely private to one mod,
  such as `mods/spawner`'s list of the entities it owns.

Changing a type is also handled better than with per-mod state. Every
component carries a field-level schema, so a layout change migrates its
values field by field instead of resetting a whole mod's state (see
[Layout changes](#layout-changes)).

## Why the loader owns it

The world has to be shared by every mod and survive any of them reloading, so
it can't live in one mod's state. Until mods can call each other, only the
loader can hold it. It is a host service like `step_mods`: the loader stores
bytes and knows nothing about the game.

Mods reach it only through `WorldApi`, a table of `extern "C"` functions.
The alternative, a Rust `World` type linked into every mod and operating on
shared memory, would need every mod and the loader to agree on that type's
layout. That holds only until one of them is rebuilt against a changed ECS
crate and reloaded alone, and then it fails silently. Through the C ABI, the
storage code exists in one place, and `API_VERSION` covers the contract.

**Open question:** whether the world should move into a mod once mods can
call each other. That would make the ECS itself reloadable, at the cost of
the world's lifetime depending on one mod.

## Components

A component is declared with `component!`, which gives it a stable name and
a schema:

```rust
component! {
    #[derive(Debug, Default)]
    pub struct Position: "game::Position" {
        pub x: f32,
        pub y: f32,
    }
}
```

The macro adds `Clone` and `Copy` and implements `Component`, recording each
field's name, kind and offset (`offset_of!`) in `Component::FIELDS`. Every
field must be a `FieldType`: the integer and float types, `bool` and
`Entity`. It is a `macro_rules!` macro rather than a derive, so it needs no
proc-macro crate; the price is its fixed `struct Name: "id" { ... }` syntax.
`Component` can still be implemented by hand, with no schema.

The name is the identity, not the Rust type. `TypeId` can differ between
builds, and two mods compiled separately each have their own copy of the
type. A component shared by several mods is declared in one mod's interface
and used by the others through `mod_deps` (see [mod-deps.md](mod-deps.md)).

The trait requires `Copy` and `Default`, and its safety contract requires
plain data. A component's value outlives the build that wrote it, so it can't
hold a reference, raw pointer or function pointer into a mod (a `&'static str`
or `fn` points into a library a reload unmaps). `Copy` also rules out drop
glue, which would be code in an unmapped library. The type system can't check
the pointer rule, which is why the trait is `unsafe`; `component!` makes it
safe by accepting only `FieldType` fields.

**Open question:** components that own heap data (`Vec`, `String`). They need
a drop that doesn't live in a mod, perhaps by keeping the allocation in loader
memory. Today such data goes in `Mod` state.

## Storage

Each component has a sparse set: values packed densely alongside their
entities, plus a map from entity index to slot. A query walks one component's
dense column, and `query2` looks the second component up per entity, so it
should be given the rarer component first. Removal swaps the last value into
the gap, so iteration order is not stable.

Entities are generational indices: despawning bumps the index's generation,
so an old handle never reaches the entity that reuses its index.

The loader borrows the store only for the length of one call, and pointers
it hands out are valid until the next structural change. The typed API makes
that a borrow-checker rule: `insert`, `remove` and `despawn` take `&mut self`,
and so do queries, for as long as their references are used.

**Open question:** structural changes during a query (spawning from inside a
loop). The borrow checker currently forbids them. A command buffer applied
after the query is the usual answer.

**Open question:** archetypes. Sparse sets are the simplest store that works.
Packing entities with the same components together makes multi-component
queries faster, but moves data on every add and remove.

## Layout changes

A component's size, alignment, `VERSION` and schema are checked on every
access. When two builds disagree, the one loaded more recently wins, because
that is the one the developer just changed. "More recently" means
`ModContext::loaded_at`, a counter the loader increments on every load across
all mods.

**A newer build migrates the values** to its layout. Each registration
carries the build's `Default` value along with the schema, and the loader
rebuilds every value from it, matching fields by name:

| change | result |
|---|---|
| field added | takes the new build's `Default` (not zero) |
| field removed | dropped |
| fields reordered | kept (matched by name, not offset) |
| numeric type changed (`f32` to `f64`, `i32` to `u8`) | converted with `as` semantics |
| type changed across kinds (`f32` to `bool`) | reset to the default |
| field renamed | dropped and re-added as a default |

The loader logs what happened, e.g.
`migrated 3 value(s) of game::Position to physics's layout: kept x, kept y, added z (default)`.

**Bumping `VERSION` clears the values instead** (`component!` takes it as
`pub struct Position: "game::Position", version = 1 { ... }`). It is for a
change a schema can't see: a field that now means something else, such as
different units. A component with no schema is also cleared, since there is
nothing to match fields by.

**An older build** (still built against the previous layout) gets
`ComponentId::INVALID`, which makes every operation on that component a no-op,
with one warning per build. It works again once it is reloaded. For mods
built with `engine_mod` this doesn't arise: the engine refuses a reload that
would leave a dependent on the old layout, and a game reload swaps the
component's owner and its dependents together (see
[mod-deps.md](mod-deps.md)). The check remains for libraries that record no
dependencies.

**Open question:** renames. A rename is indistinguishable from a removal
plus an addition. An attribute naming the old field
(`#[renamed_from = "pos_x"]`) would let migration carry the value over.

**Open question:** conversions a schema can't express, such as splitting a
field or changing units. They need code from the new build, given the old
values, and a hook for it would sit beside the version bump.

**Open question:** nested types and arrays (`[f32; 3]`, a struct field). The
schema has only scalar kinds, so `component!` rejects them, and a component
that needs one has to be implemented by hand, without a schema. Nesting
schemas would handle both.
