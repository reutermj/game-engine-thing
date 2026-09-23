# Hot reload

How a mod's code is swapped while its state stays live. The code lives in
`engine/api/lib.rs` (the ABI), `engine/loader/engine.rs` (the sequence) and
`engine/defs.bzl` (how a mod is built so the sequence works).

## The ABI

A mod is a `cdylib` exporting two `extern "C"` symbols:

- `engine_mod_info() -> ModInfo`: the mod API version it was built against,
  and the size, alignment and version of its state. The loader calls this
  before committing to a new build.
- `engine_mod_main(ctx, op) -> Status`: the single entry point, as in cr.h.
  `op` is `LOAD`, `STEP`, `UNLOAD` or `CLOSE`.

Everything that crosses the boundary is `#[repr(C)]`, and `Op`/`Status` are
integer newtypes rather than Rust enums, so a value one side doesn't know is
an unknown number rather than undefined behavior. Mods don't write any of
this: they implement the `Mod` trait and call `export_mod!`, which also
catches panics at the boundary (unwinding out of an `extern "C"` function
aborts the process).

The ABI is C, not Rust's, and not `abi_stable`'s, because `abi_stable`
deliberately never unloads a library (see
[lore](../lore/abi-stable-never-unloads-a-library.md)).

**Open question:** whether to use `abi_stable`'s FFI-safe types (`RString`,
`RVec`) inside the C ABI once mods exchange richer data. Its loader is out,
but its types don't depend on it.

## Who owns state

There are two kinds. Data shared between mods, or that belongs to the game
rather than to one mod, lives in the world as components (see
[ecs.md](ecs.md)). This section is about the other kind: a mod's own state.

The loader allocates each mod's state (zeroed, sized and aligned per
`ModInfo`) and passes it in `ModContext`. The mod type *is* the state: the
`Mod` value lives in that memory, initialized from `Default` the first time
the mod is loaded. Because the memory belongs to the loader, it outlives any
one build of the mod.

What survives a reload is exactly the state struct. A mod's `static`s do not,
since every reload maps a fresh copy of the library. Heap allocations the
state points to do survive, because every mod and the loader share the same
system allocator.

The loader decides whether the old state can be handed to the new build by
comparing size, alignment and `Mod::STATE_VERSION`. A mod bumps
`STATE_VERSION` when a field's meaning changes but the layout doesn't.

**Open question:** migration. An incompatible layout currently resets the
state (the old build drops it, the new build starts from `Default`). A
migrate hook that sees the old state would let a layout change keep data.

**Open question:** `thread_local!` in mods. glibc won't unmap a library
while a thread still has TLS destructors registered in it, and for the main
thread that means until exit: `dlclose` quietly keeps the old image. Reload
still works, because each load uses a fresh path, but memory grows with every
reload. Whether to ban, lint or accept it is undecided.

## The reload sequence

Every load is a batch: `load <name> <path>` is a batch of one, and a game
reload sends every mod in the game. For each mod in the batch:

1. **Skip it if unchanged.** A library whose contents match the running
   build's is left alone, so a batch reloads only what an edit affected.
2. **Stage.** Copy the library to a unique path under
   `$XDG_RUNTIME_DIR/game-engine-thing/libs/`. `dlopen` returns the already
   loaded image for a path, or a file, it has seen, so reloading from the
   Bazel output path would silently change nothing (see
   [lore](../lore/dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md)).
   The copy is deleted right after `dlopen`; the mapping doesn't need it.
3. **Open and validate the new build** while the old one keeps running:
   `dlopen`, resolve both symbols, check `API_VERSION`. Mods are linked with
   `-z now`, so unresolved symbols fail here too.

Then, for the batch as a whole:

4. **Check dependencies.** Every mod must end up running against the
   interfaces it was built against (see [mod-deps.md](mod-deps.md)). Any
   failure in steps 2–4 refuses the whole batch and leaves every running
   build untouched.
5. **Retire the old builds**, dependents first. If a mod's state is
   compatible, send the old build `UNLOAD`. Otherwise send it `CLOSE` so it
   drops its own state (only the old code knows how), then allocate fresh
   zeroed state.
6. **Swap**, dependencies first. Drop the old library, bump `generation`,
   and send the new build `LOAD`. A fresh state is initialized from
   `Default` as part of that `LOAD`.

A mod that returns `Status::ERROR` (including a caught panic) is marked
failed and not stepped again until a reload clears it.

**Open question:** crash rollback. A panic is caught; a segfault in a freshly
loaded mod kills the engine. cr.h recovers from this with signal handlers
and a rollback to the previous build, which would mean keeping the previous
library open.

### Why reloads are safe to apply between frames

Mod code runs only while the loader holds a shared borrow of the mod list
(stepping). Loading and unloading take it mutably, and the control socket is
polled only between bootstrap steps, so no request ever swaps code that is on
the stack. A mod that calls `step_mods` while the list is being modified gets
`Status::ERROR` rather than a panic, since a panic inside a host callback
would unwind through `extern "C"`.

## The control protocol

The engine listens on a Unix domain socket (`$XDG_RUNTIME_DIR/game-engine-thing/control.sock`,
or `$ENGINE_SOCKET`). One request per connection, ended by the client
shutting down its side; the reply runs to EOF and its first word is `ok` or
`err`:

```
load <name> <path>   ok loaded <name> | ok reloaded <name> (generation N) | ok <name> unchanged | err <why>
batch                the same per mod, joined with "; " (all or nothing)
<name> <path>        ...one line per mod after `batch`
unload <name>        ok unloaded <name>
list                 ok <count> mod(s) loaded, then one line per mod
quit                 ok quitting
```

The path is the rest of the line, so it may contain spaces. `modctl` is the
client; `engine_mod` targets are `modctl` with `ENGINE_MOD_NAME` and
`ENGINE_MOD_RLOCATION` set, and a game's reload target is `modctl` with
`ENGINE_BATCH_MANIFEST`. It resolves runfiles paths to absolute ones before
sending, since the engine's working directory is not the client's.

## How a mod is built

`engine_mod` wraps `rust_shared_library` with two settings the reload
sequence depends on:

- `cc_runtime_linkage = "static"`. Otherwise libc++, libc++abi and libunwind
  are linked dynamically through an `$ORIGIN`-relative RUNPATH, which stops
  resolving once the library is copied to the staging directory (see
  [lore](../lore/rust-shared-libraries-link-the-cxx-runtime-dynamically.md)).
- `-Clink-arg=-Wl,-z,now`, which is what makes step 2 above reject a build
  with unresolved symbols. The `llvm` toolchain already passes it; it is
  pinned so the loader doesn't depend on a default (see
  [lore](../lore/mods-are-linked-bind-now.md)).
