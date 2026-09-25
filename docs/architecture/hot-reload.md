# Hot reload

How a mod's code is swapped while its state stays live. The code lives in
`engine/api/lib.rs` (the ABI), `engine/loader/engine.rs` (the sequence) and
`engine/defs.bzl` (how a mod is built so the sequence works).

## The ABI

A mod is a `cdylib` exporting two `extern "C"` symbols:

- `engine_mod_info() -> ModInfo`: the mod API version it was built against,
  the size, alignment and version of its state, and a function declaring
  its systems. The loader calls this before committing to a new build.
- `engine_mod_main(ctx, op) -> Status`: the lifecycle entry point, as in
  cr.h. `op` is `LOAD`, `UNLOAD`, `CLOSE`, `MESSAGE`, or, for a bootstrap,
  `RUN` (the session). A frame's work doesn't go through it: the loader
  calls each system directly (see [scheduling.md](scheduling.md)).

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

Data shared between mods, or that belongs to the game rather than to one mod,
lives in the world as components (see [ecs.md](ecs.md)). A mod's own data has
three homes, by how long it has to live:

| lifetime | holds | where |
|---|---|---|
| one build | closures, trait objects, caches, crate objects | the mod's `Transient` |
| across reloads | the mod's own game state | the mod's state (`mod_state!`) |
| the process | windows, devices, threads | registered resources, resident mods (get-y5t) |

**The state** is the `Mod` type itself, declared with `mod_state!`. The loader
allocates it, makes it from the build's `Default` on first load, and hands it
to every later build. So it follows the rule components do: every field must
be a `FieldType`, which rules out anything pointing into a build's code (a
closure, a trait object, a function pointer, a `&'static str`). A reload would
unmap that code and leave the next build calling through a dangling pointer.
Heap data is fine: every mod and the loader share one allocator, and every
build comes from the same rustc (checked at load; see
[ecs.md](ecs.md#one-compiler-per-session)), so std types keep their layout.

**The transient part** (`Mod::Transient`) is everything else. Each build makes
its own, from `Default`, before its `load`, and drops it itself after its
`unload` or `close`, while its code is still mapped. It never reaches another
build, so it may hold anything, and a reload replaces it with the new build's:
a closure there runs the new code, which one kept in the state never could.
Most mods need none (`type Transient = ();`).

A mod's `static`s survive nothing: every reload maps a fresh copy of the
library.

When a build replaces another, the loader hands the state over by the same
rules a component migrates by:

- **Same layout and schema:** carried over as is.
- **Changed layout, same version, both with a schema:** migrated field by
  field (kept, converted, dropped with the old build's code, or added from the
  new build's `Default`), as in [ecs.md](ecs.md#layout-changes).
- **A version bump** (`mod_state! { struct S, version = 1 { .. } }`) **or no
  schema:** the old build drops its state (`CLOSE`) and the new one starts
  from `Default`.

**A net for state that bypassed the rule.** `mod_state!` enforces it, but a
hand-written `unsafe impl ModState` can hold anything. Before carrying state
over, the loader scans it for pointer-sized values inside the old build's
mapped image; if it finds one, it has the old build drop the state and the
new one start over, with a warning, instead of crashing. The scan is shallow
(a pointer inside a `Vec`'s buffer is invisible to it) and could in principle
be fooled by an integer that looks like an address, so it's a mitigation, not
a guarantee. It also finds an empty `HashMap`, which points at a static in
the image that made it, so a state holding one is reset on every reload
(see [lore](../lore/an-empty-hashmap-points-into-the-build-that-made-it.md)).

## Resident mods

`engine_mod(resident = True)` makes a mod **resident**: loaded once, and its
build never swapped for the rest of the session. Its code therefore stays
mapped until the engine exits, and so does its transient part, which it makes
once at load and drops at shutdown. That is what makes a resident mod the
place for things that point into their own code and can't be rebuilt on every
reload: windows, devices, and threads (the test mod `vault` owns one). Other
mods reach them through the resident mod's services, holding plain handles.

A new build of a resident mod takes a restart:

- **A per-mod reload** of a changed resident build is refused: "vault is
  resident: restart the engine to load its new build".
- **A game reload** keeps the resident build, reloads everything else that
  changed, and says which resident builds it kept. Other mods' new builds are
  checked against the resident build that stays, so one compiled against a
  resident mod's new interface is refused with the same advice.
- **Unloading a resident mod** is refused. It is closed when the engine
  shuts down, after the mods loaded after it, so it outlives what uses it.

**A resident mod may depend only on resident mods.** Residency flows
downward: resident mods form the stable base (a platform layer owning the
window and GPU), and reloadable mods sit on top, depending on them. If a
resident mod could depend on a reloadable one, that mod's interface would be
frozen too, since its resident dependent could never be rebuilt against a new
one, and "reloadable" would silently stop meaning it. `engine_mod` fails the
build for a resident mod with a non-resident dependency, and the loader
refuses one built some other way. Where a resident layer needs something
from gameplay, gameplay leaves it in the world (or sends an
[event](scheduling.md#events)) and the resident layer reads it.

Every bootstrap is resident: it runs the frame loop, so it's on the stack
whenever the loader swaps builds. `engine_game` fails the build for a
bootstrap that isn't, and so does `Engine::run_bootstrap` for one loaded
otherwise. The `clock` both bootstraps publish is resident because they
depend on it.

A resident mod is otherwise an ordinary mod: its state follows the same rules,
it provides and calls services, and `list` marks it `[resident]`.

Dropping an `Engine` closes every mod, whether or not `shutdown` was called,
so a resident mod's threads are stopped while its code is still mapped.

**Open question:** `thread_local!` in mods. glibc won't unmap a library
while a thread still has TLS destructors registered in it, and for the main
thread that means until exit: `dlclose` quietly keeps the old image. Reload
still works, because each load uses a fresh path, but memory grows with every
reload. This happens without any `thread_local!` of a mod's own: a mod that
spawns a std thread stays mapped after the engine drops it (see
[lore](../lore/a-mod-that-spawns-a-thread-is-never-unmapped.md)), so a
reloadable mod that spawns threads leaks its image on every reload. Resident
mods are the right home for threads anyway; whether to also lint or warn is
undecided.

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
5. **Retire the old builds**, dependents first. If a mod's state can be
   carried over or migrated, send the old build `UNLOAD` (it drops its
   transient part), then move or migrate the state. Otherwise send it `CLOSE`
   so it drops its transient part and state (only the old code knows how),
   and make the new build's `Default`.
6. **Swap**, dependencies first. Drop the old library, bump `generation`,
   and send the new build `LOAD`, which makes its transient part.

A mod that returns `Status::ERROR` (including a caught panic in a system)
is marked failed, and none of its systems run again until a reload clears
it.

**Open question:** crash rollback. A panic is caught; a segfault in a freshly
loaded mod kills the engine. cr.h recovers from this with signal handlers
and a rollback to the previous build, which would mean keeping the previous
library open.

### Why reloads are safe to apply between frames

Requests are served only when the bootstrap pumps the loader
(`cx.pump_loader`), and the loader accepts a pump only at a safe point: the
mods running are all resident, and no step, load or message delivery is
under way (the mod list isn't borrowed). Resident builds are never swapped,
so no request ever swaps code that is on the stack. Anywhere else the pump
answers `Pumped::Refused`. See
[overview.md](overview.md#who-runs-the-loop).

A mod that calls `step_mods` while the list is being modified gets
`Status::ERROR` rather than a panic, since a panic inside a host callback
would unwind through `extern "C"`.

## How reload is tested

Three ways, each finding what the others can't. `//engine/tests:reload_test`
pins each rule with two real builds and an exact expectation. The e2e test
covers the socket and the manifest. And a **fuzzer** composes the rules:
coverage-guided (libFuzzer), with a model of the rules checked after every
operation, over mods made for it (`engine/tests/fuzz/`, runbook
[003](../runbooks/003-fuzz-hot-reload.md)).

The mods are built several ways each, and each build reports which it is:

- `keeper`: a table component with heap fields and a sparse one, laid out
  three ways (fields reordered, widened, added, removed; a version bump),
  with its state migrating or resetting alongside; builds that panic in
  `load`, in `unload` and `close`; one that stores the sparse component in
  tables, which must be refused.
- `herald` and `hearer`: an event, sent between frames and from a system,
  read before and after it is sent; a service and its caller, from a hook
  and from messages; two interface versions, so a reload strands the other
  unless both are batched; a provider that panics.
- `ranker`: an ordered key whose glue changes (ascending, then descending)
  with its layout the same, and a layout change.
- `anchor` and `tether`: residency, built resident and not.
- `clock`, `lockstep` (frames run from a message) and `sequential` (frames
  scheduled through a service), as the engine ships them; libraries that
  aren't mods, or are from another mod API.

The operations are loads of any build under its name (and a second
provider's name), batches, unloads, the loader's own frames and lockstep's,
and messages that spawn, despawn, write and flag components, send and
queue events, call the service, or fail the mod. The model predicts each
from this document and [mod-deps.md](mod-deps.md): which loads are refused
and why (dependencies, interfaces, residency, one provider, a changed
storage, open failures), and the exact reply; how each mod's state is
handed over; how each component's values migrate or reset when a newer
build installs another layout; which events each reader sees, once; the
order ordered rows are in; which mods have failed. After every operation
the driver checks all of that against the engine, the world read through
its own mirror of each layout, and the builds mapped (`/proc/self/maps`),
which must be exactly the running builds and those the world keeps for its
values. Dropping the engine must unmap them all.

Six bugs planted in the loader and the ECS, from a wrong numeric
conversion to an event read twice across a reload, were each found within
15 minutes (runbook 003 has them). It found one real bug on its first
seeded runs: an event queue's cursors outlived the build that made them
(see [lore](../lore/an-empty-hashmap-points-into-the-build-that-made-it.md)).

**Not covered yet:** spatial keys; bootstraps running the session
(`run_bootstrap`, `pump_loader`); schedule refusals (cycles, a phase no mod
declares); fixed-rate phases; state that points into its build; builds
from another rustc; threads.

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
send <name> <text>   ok <the mod's reply> | err <why the mod declined>
list                 ok <count> mod(s) loaded, then one line per mod
schedule             ok <phase>: <system>, ... one line per phase with systems
quit                 ok quitting
```

The path is the rest of the line, so it may contain spaces. `modctl` is the
client; `engine_mod` targets are `modctl` with `ENGINE_MOD_NAME` and
`ENGINE_MOD_RLOCATION` set, and a game's reload target is `modctl` with
`ENGINE_BATCH_MANIFEST`. It resolves runfiles paths to absolute ones before
sending, since the engine's working directory is not the client's.

## Testing that a reload doesn't show

A reload of builds with the same code must leave a game exactly as it
would have been: every mod's state, every value in the world, every event
queue and every fixed-rate phase's time owed. The games hold the engine to
that (`//pong:reload_test`, `//platformer:reload_test`, over
`//engine/tests:replay.rs`): each replays a recorded route once without
reloads, then again reloading its mods (every one not resident) every
frame, a few frames or tens of frames, one mod at a time or several in a
batch, and requires every frame of the two runs to be the same, bit for bit.

The builds swapped have to be different files, or step 1 of the reload
sequence skips them
(and `dlopen` would hand back the image it has anyway). So a mod the
replays reload is built twice: `engine_mod(twin = True)` adds `<name>_twin`,
the same sources under another crate name, which changes its symbols and
so its file but not its code or layouts. Every build embeds the Bazel label
it was built as (`ENGINE_MOD_BUILD`), and `Engine::build_of` asks the
running build for it, which is how the replays know each reload mapped the
build they sent.

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
