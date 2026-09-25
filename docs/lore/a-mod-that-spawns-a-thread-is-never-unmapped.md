# A mod that spawns a thread is never unmapped

`dlclose` really does unmap a mod nothing holds (see
[dlclose-unmaps-a-mod-nothing-holds.md](dlclose-unmaps-a-mod-nothing-holds.md)),
but not one that has spawned a std thread. Measured in the resident-mod
tests, counting the staged library's lines in `/proc/self/maps` after the
engine was dropped:

| mod | what it did | mappings after the engine is dropped |
|---|---|---|
| `bag` | pushed strings into a component | 0 |
| `vault` | spawned a thread, and stopped and joined it on close | 4 |

The thread itself was gone (its name no longer in `/proc/self/task`). What
still pins the library is a TLS destructor the spawn registered on the
*spawning* thread, here the engine's main thread, in the mod's own copy of
std: `std::thread::spawn` touches the thread-local
`std::thread::spawnhook::SpawnHooks`. glibc won't unmap a library with TLS
destructors still registered, and a main-thread destructor only runs at
exit.

Traced 2026-09-25 (it was inferred before, and blamed on the spawning
thread's handle): the loader's poison mode with
`ENGINE_POISON_UNLOADED=strict` makes a build `dlclose` keeps mapped
`PROT_NONE` anyway (engine/loader/poison.rs). `reload_test`'s
`dropping_the_engine_stops_a_resident_mods_thread` then faults when its
test thread exits, at offset `0x3b170` of `libvault_v1_mod.so`, which
`llvm-symbolizer` names as
`std::sys::thread_local::native::eager::destroy::<Cell<SpawnHooks>>`.

## What it means

A reloadable mod that spawns threads leaks its whole image on every reload:
nothing crashes, and memory grows. It's another reason threads belong in
resident mods (see
[hot-reload.md](../architecture/hot-reload.md#resident-mods)), which are
never unloaded before exit anyway.

It also means "the library is unmapped" is not a usable test that a mod
stopped its threads. The resident-mod test checks the thread directly, by
name, instead.
