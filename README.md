# game-engine-thing

A mod-first engine prototype. The engine binary is only a mod loader. Everything
else, including the frame loop, is a hot-reloadable mod (a `cdylib`) that is reloaded
cr.h-style: the loader owns each mod's state, so it survives code swaps.

## Try it

```sh
bazel run //game                  # terminal 1: engine + bootstrap + counter
# edit mods/counter/lib.rs, then:
bazel run //mods/counter          # terminal 2: rebuild and hot-reload
bazel run //mods/hello            # load a mod the running game didn't ship with
bazel run //engine/modctl -- list # or: unload <name>, quit
```

## Layout

- `engine/api`: the C ABI (`Mod` trait + `export_mod!`) shared by loader and mods
- `engine/loader`: the engine binary (libloading, state ownership, control socket)
- `engine/control`: the line protocol spoken over the Unix socket
- `engine/modctl`: the client; every `engine_mod` target is a symlink to it
- `engine/defs.bzl`: `engine_mod` and `engine_game`
- `mods/*`: `bootstrap` (frame loop), `counter` (reload demo), `hello` (live-load demo)

## How reload works

1. `bazel run //mods/foo` builds `libfoo.so`, then runs `modctl`, which sends
   `load foo <path>` to `$XDG_RUNTIME_DIR/game-engine-thing/control.sock`.
2. Between frames, the engine copies the `.so` to a unique path, `dlopen`s it and
   checks its `ModInfo`. A bad build leaves the old code running.
3. The old build gets `UNLOAD`, the new one gets `LOAD` with the same state
   memory. If the state's size, alignment or `STATE_VERSION` changed, the old
   build gets `CLOSE` instead and the new one starts from `Default`.

Mod `static`s don't survive a reload; put anything that should persist in the mod
struct. A panicking mod is disabled until it's reloaded.
