# game-engine-thing

A mod-first engine prototype. The engine binary is only a mod loader. Everything
else, including the frame loop, is a mod (a `cdylib`), hot-reloaded cr.h-style: the
loader owns each mod's state, so it survives code swaps. The mod running the frame
loop is resident (loaded once) and hands the loader control between frames. Game data
lives in an ECS world the loader also owns: systems are mods and get reloaded,
components are data and don't.

## Try it

Nothing needs to be installed: `./bazel` fetches a pinned bazelisk, which
fetches the pinned Bazel, which fetches hermetic Rust and LLVM toolchains.

```sh
./bazel run //game                  # terminal 1: engine + the mods in game/BUILD.bazel
# edit mods/physics/lib.rs (say, add gravity), then:
./bazel run //mods/physics          # terminal 2: rebuild and hot-reload; entities keep moving
# edit mods/physics/components.rs, which spawner also uses, then:
./bazel run //game:reload           # reloads physics and spawner together, nothing else
# edit mods/counter/lib.rs, then:
./bazel run //mods/counter          # per-mod state carries over too
./bazel run //mods/hello            # load a mod the running game didn't ship with
./bazel run //engine/modctl -- list # or: schedule, unload <name>, quit
./bazel test //...                  # unit, integration and end-to-end tests
```

## Play pong

Pong runs on the lockstep bootstrap: time only moves when you step it, so
it's playable turn by turn (by an agent, say) and every game is
reproducible.

```sh
./bazel run //pong                                        # terminal 1
M=bazel-bin/engine/modctl/modctl                          # terminal 2
$M send pong_text show                                    # draw the court
$M send pong_text down                                    # up | down | stay
$M send lockstep step 30                                  # run 30 frames
$M send pong_text state                                   # exact positions
```

The platformer works the same way (`./bazel run //platformer`, then
`$M send platformer_text left | right | stop | jump | show | state`), and its
level is `platformer/level/map.txt`: edit it and
`./bazel run //platformer/level` swaps the new level in while you play.

## Layout

- `engine/api`: the C ABI (`Mod` trait + `export_mod!`, and the ECS `World`) shared
  by loader and mods
- `engine/loader`: the engine binary (libloading, state ownership, the ECS store,
  control socket)
- `engine/control`: the line protocol spoken over the Unix socket
- `engine/modctl`: the client; every `engine_mod` target is a symlink to it
- `engine/defs.bzl`: `engine_mod` and `engine_game`
- `mods/*`: `bootstrap` (frame loop), `counter` (per-mod state), `hello` (live
  load), `transform`/`physics`/`spawner`/`reporter` (ECS demo, and mods that
  depend on each other's components)
- `engine/tools`: build-time tooling for `engine_mod`

## How reload works

1. `./bazel run //mods/foo` builds `libfoo.so`, then runs `modctl`, which sends
   `load foo <path>` to `$XDG_RUNTIME_DIR/game-engine-thing/control.sock`.
2. Between frames, when the bootstrap mod hands it control, the engine copies the `.so` to a unique path, `dlopen`s it and
   checks its `ModInfo`. A bad build leaves the old code running.
3. The old build gets `UNLOAD`, the new one gets `LOAD` with the same state
   memory, migrated field by field if its layout changed. If its version changed,
   the old build gets `CLOSE` instead and the new one starts from `Default`.

A mod's state (declared with `mod_state!`) survives reloads, so it may only hold
plain data, as components do. Closures, trait objects and crate objects go in the
mod's `Transient`, which every build makes for itself. Mod `static`s don't survive
a reload. A panicking mod is disabled until it's reloaded.

## Docs

- [docs/architecture/](docs/architecture/): the design, including the open
  questions deliberately deferred until hot reload was proven
- [docs/lore/](docs/lore/): non-obvious things that cost real effort to work out
- [docs/runbooks/](docs/runbooks/): recurring maintenance procedures
- [docs/retrospectives/](docs/retrospectives/): what building on the engine
  showed: pong, then the platformer
- [CLAUDE.md](CLAUDE.md): conventions for working in the repo (for agents,
  and humans too)
