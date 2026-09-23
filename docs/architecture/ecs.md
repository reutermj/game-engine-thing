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
    pub struct Inventory: "rpg::Inventory" {
        pub owner: String,
        pub items: Vec<Item>,
        pub gold: u32,
    }
}

field_struct! {
    #[derive(Debug)]
    pub struct Item {
        pub name: String,
        pub weight: f32,
    }
}
```

The macro adds `Clone` (add `Copy` yourself if every field is) and implements
`Component`, recording each field's name, kind, offset, size, layout
fingerprint and drop function in `Component::FIELDS`. Every field must be a
`FieldType`: the integer and float types, `bool`, `Entity`, `String`, `Vec`,
`Option`, `Box`, arrays, `HashMap`, `BTreeMap`, and structs declared with
`field_struct!`. It is a `macro_rules!` macro rather than a derive, so it
needs no proc-macro crate; the price is its fixed `struct Name: "id" { ... }`
syntax. `Component` can still be implemented by hand, with no schema.

The name is the identity, not the Rust type. `TypeId` can differ between
builds, and two mods compiled separately each have their own copy of the
type. A component shared by several mods is declared in one mod's interface
and used by the others through `mod_deps` (see [mod-deps.md](mod-deps.md)).

A component's value outlives the build that wrote it, so it can't hold
anything that points into a mod's image: a reference, a function pointer, a
trait object (its vtable is in the mod) or a `&'static str` (its bytes are).
Heap memory is fine: every mod and the loader share libc's allocator. The type
system can't check the image rule, which is why `Component` and `FieldType`
are `unsafe` traits; `component!` makes a component safe by accepting only
`FieldType` fields, and `FieldType` is implemented only for types that obey
it.

### Heap data and whose code runs

The loader moves values around as bytes, but freeing a `String` or making a
`Default` takes code, and only mods have it. So every registration hands the
loader that code: a drop function for the whole value, one per field, and a
`Default` constructor, all compiled into the registering build. The world
keeps the newest registered build's functions, and a reference to that
build's library, which keeps it mapped: an older build's library is unmapped
once a newer build has taken over every component it provided code for,
normally on that build's first access. Values therefore survive their mod
being reloaded or even unloaded, and are dropped with valid code when they're
removed, despawned, cleared or the world goes away. (`dlclose` really does
unmap a mod whose code nothing holds; see
[lore](../lore/dlclose-unmaps-a-mod-nothing-holds.md).)

`World::insert` moves the value in: the world owns it from then on and drops
the value it replaces. A value the world refuses (dead entity, stale layout)
is dropped by the caller.

### One compiler per session

`String`, `Vec` and `HashMap` have no guaranteed layout; Rust only keeps it
the same between builds of one compiler. Since a value written by one build is
used by others' code, every mod and the loader in one running engine must come
from the same rustc. The loader enforces it: rustc records its version in the
`.comment` section of everything it compiles, and a mod whose version differs
from the engine's is refused ("restart the engine to switch compilers"). The
workspace pins one toolchain, so this only matters when it changes while an
engine is running.

**Open question:** mods built outside this workspace would need the game's
exact rustc. Only components holding std types strictly need it (scalar
fields have fixed layouts), so the check could be narrowed to those, or
engine-owned `#[repr(C)]` containers could lift it.

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
carries the build's `Default` constructor along with the schema, and the
loader rebuilds every value from it, matching fields by name:

| change | result |
|---|---|
| field added | takes the new build's `Default` (not zero) |
| field removed | dropped, with the old build's drop code |
| fields reordered | kept (matched by name, not offset) |
| numeric type changed (`f32` to `f64`, `i32` to `u8`) | converted with `as` semantics |
| non-scalar field, same type (`Vec<String>`) | moved as is: the heap buffer carries over |
| non-scalar type changed (`Vec<u32>` to `Vec<u64>`, or an `Item` inside gained a field) | old value dropped, reset to the default |
| type changed across kinds (`f32` to `bool`) | reset to the default |
| field renamed | dropped and re-added as a default |

"Same type" is decided by a layout fingerprint `component!` computes for each
field type, recursively: `Vec<Item>` covers `Item`'s own fields, so changing
`Item` changes every field that holds one, and its values are never moved
under a layout they weren't built for. Each migrated value starts as a fresh
`Default` from the new build, so a default that owns heap memory isn't
shared between values.

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
