# Overview

## Everything is a mod

The engine is layered so that the part that can't be reloaded is as small as
possible:

1. **The loader** (`engine/loader`) loads dynamic libraries, owns their state
   memory, serves the control socket, and calls one mod — the bootstrap mod —
   once per iteration. It has no frame loop, no timing and no game concepts.
2. **The bootstrap mod** (`mods/bootstrap`) owns the frame loop: it paces
   frames and steps every other mod through the `step_mods` host service. It
   is an ordinary mod, so it is reloadable like any other.
3. **Every other mod** does real work when stepped. Today that is `counter`
   and `hello`; eventually windowing, rendering, input and the game.

### Why the loader still has a loop

The bootstrap mod "owns the loop" in every sense except the literal `loop`,
which lives in `engine/loader/main.rs` as a trampoline: poll the socket, step
the bootstrap mod once, repeat. If the bootstrap mod ran the loop itself, its
code would always be on the stack and it could never be swapped out. With the
trampoline, each bootstrap step returns before the next socket poll, so a
reload always happens when no mod code is running. The bootstrap mod ends the
program by returning `Status::QUIT`.

Time is published as data: the bootstrap writes a `Clock` component (from
`mods/clock`) before each frame, and systems read `dt` from it. That lets a
game swap the real-time bootstrap for `mods/lockstep`, which runs frames only
when sent `step N` (see `modctl send`), without changing any system. Pong
does, so it can be played turn by turn.

**Open question:** what the bootstrap should own. It currently owns time
policy, publishing the clock and scheduling, and changing only the first
means replacing all three. See
[the pong retrospective](../retrospectives/2026-09-23-pong.md#the-bootstrap-mod).

**Open question:** ordering. `step_mods` steps mods in load order; a real
engine needs phases (input before simulation before rendering). Whether the
bootstrap mod should get per-mod stepping, or mods should declare a phase, is
undecided.

**Open question:** threading. Everything runs on the one thread that owns the
loop. A mod that spawns a thread makes unloading unsafe, because the thread
can still be running code in the library being unmapped. How mods get
concurrency without breaking reload is undecided.

## Bazel is the build and the reload trigger

Every artifact is built by Bazel with hermetic toolchains, and the developer
loop runs through `bazel run`:

- `./bazel run //game` starts the engine with the mods listed by an
  `engine_game` target, the startup list of mods, loaded in order after the
  bootstrap mod.
- `./bazel run //mods/<name>` builds that mod and sends it to the running
  engine, which loads it if it's new or hot-reloads it if it's running. A mod
  the engine has never heard of is just a first load.

This works because `bazel run` releases the Bazel server before it executes
the binary: the engine started by the first `bazel run` doesn't block the
second one from building.

Mods share data through the world, and a mod can depend on another mod's
components: see [mod-deps.md](mod-deps.md), which also covers
`./bazel run //game:reload` for changes that reach several mods.

**Open question:** calls between mods. A renderer mod that a game mod calls
into needs an interface of functions, not just components, and reloading the
provider then affects its consumers. The likely shape is a registry of C-ABI
vtables in the loader, re-fetched after a provider reloads.

**Open question:** distribution. Whether mods will ever be built outside this
workspace or shipped to players decides whether the ABI must be stable
across builds or only within one. Today it only has to hold within one build,
and `API_VERSION` exists to fail loudly when it doesn't.
