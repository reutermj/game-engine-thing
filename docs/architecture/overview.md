# Overview

## Everything is a mod

The engine is layered so that the part that can't be reloaded is as small as
possible:

1. **The loader** (`engine/loader`) loads dynamic libraries, owns their state
   memory, and serves the control socket when it's handed control. It has no
   loop, no timing and no game concepts: after loading the game, it calls the
   bootstrap mod once and that call is the session.
2. **The bootstrap mod** (`mods/bootstrap`) owns the frame loop: it paces
   frames, steps every other mod through the `step_mods` host service, and
   once a frame hands the loader control. It is
   [resident](hot-reload.md#resident-mods): loaded once, never swapped.
3. **Every other mod** does real work when stepped. Today that is `counter`
   and `hello`; eventually windowing, rendering, input and the game.

### Who runs the loop

The bootstrap does, literally: its `Bootstrap::run` is a `loop` that returns
`Status::QUIT` when the engine is asked to quit. The loader gets control back
through a host call, `cx.pump_loader(timeout, handler)`, which serves the
requests queued since the last one (loads, reloads, messages, `list`, `quit`)
and returns. The control socket has its own thread, which only reads each
request and queues it; nothing touches a mod until the bootstrap pumps.

A reload can happen inside that call even though the bootstrap's code is on
the stack, because the bootstrap is resident: its build is never swapped.
The loader serves a pump only at a **safe point**, where every mod running is
resident and no step, load or message delivery is under way; anywhere else,
such as a reloadable mod or a pump from inside `step_mods`, it answers
`Pumped::Refused` and does nothing. A message for the pumping bootstrap
itself goes to `handler` rather than its `Mod::message`, since the bootstrap
is already running. `mods/lockstep` is built on that: it blocks in the pump
until a request arrives, and runs frames when a message asks.

This is what a platform layer needs. Windowing libraries own the thread in
the same way (winit's `run_app`, a browser's `requestAnimationFrame`), calling
back once per event; a bootstrap on one pumps the loader from its per-frame
callback. The test bootstrap `engine/tests/mods/fake_os.rs` has that shape,
with a fake event loop in place of a window: the tests reload a mod, message
the bootstrap (feeding its event queue) and quit, all while its loop runs.

Decided 2026-09-23. Before, the loader's `main` looped over "poll the socket,
step the bootstrap once", so the bootstrap could be reloaded, but it couldn't
block (`lockstep` slept 2 ms a call to avoid spinning) and couldn't sit
under an event loop that won't return. A resident bootstrap costs a restart
to change it, which is rare next to changing gameplay.

Tests and tools can drive an `Engine` with no bootstrap: `Engine::step_all`
runs a frame, and `Engine::pump` serves requests.

Time is published as data: the bootstrap writes a `Clock` component (from
`mods/clock`) before each frame, and systems read `dt` from it. That lets a
game swap the real-time bootstrap for `mods/lockstep`, which runs frames only
when sent `step N` (see `modctl send`), without changing any system. Pong
does, so it can be played turn by turn.

**Open question:** what the bootstrap should own. It currently owns time
policy, publishing the clock and scheduling, and changing only the first
means replacing all three. See
[the pong retrospective](../retrospectives/2026-09-23-pong.md#the-bootstrap-mod).

**Ordering** is by systems and phases: each mod declares its systems, and
`step_mods` runs one frame of them in plan order. See
[scheduling.md](scheduling.md).

**Open question:** threading. Mods run on the one thread that owns the loop
(the control socket's thread runs only loader code). A mod that spawns a thread makes unloading unsafe, because the thread
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

Mods can also call each other through services (`service!`), resolved to the
provider's current build on every call, so a reloaded provider is simply
called in its new build: see
[mod-deps.md](mod-deps.md#calls-between-mods).

**Open question:** distribution. Whether mods will ever be built outside this
workspace or shipped to players decides whether the ABI must be stable
across builds or only within one. Today it only has to hold within one build,
and `API_VERSION` exists to fail loudly when it doesn't.
