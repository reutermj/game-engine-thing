# A mod that spawns threads takes a TLS key each load

A process has 1024 pthread TLS keys, and each load of a mod that spawns
std threads uses one up for good. After about a thousand loads the next
thread a new copy starts aborts the whole process:

    fatal runtime error: out of TLS keys, aborting

Found 2026-10-03 (get-znt.34) running the 3D baseline's long suite
(`//engine/std/physics3d/compare:baseline -- --long`) once every game
loaded the `threads` mod (docs/architecture/threads.md): the harness
makes a fresh engine for each run, each engine loaded the thread host,
and each host spawned its pool's seven workers. **Measured:** the run
logged 1020 pools made, then aborted; the same suite without the thread
host (the 4daabfc tree) ran to the end.

**Why, read in std's source** (`library/std/src/sys/thread_local/guard/key.rs`,
the toolchain's rust-src): on Linux every copy of std schedules its
per-thread cleanup with one lazily made pthread key (`static DTORS:
LazyKey`), made the first time a thread in that copy needs it, which a
spawned thread running that copy's code does. A mod links its own copy of
std, so every load of the mod is a fresh copy and a fresh key, and nothing
calls `pthread_key_delete`: the copy stays mapped anyway (its TLS
destructor on the spawning thread keeps it there,
[lore](a-mod-that-spawns-a-thread-is-never-unmapped.md)). That a key is
made per copy is from the source; that the abort follows from it is
inferred from the count matching the limit, not traced.

## What it means

- A thread host is loaded once a session: resident, as `threads` is, a
  game uses one key.
- A harness that makes engine after engine in one process must not load
  the thread host each time. The physics comparisons load their scene
  games without it and install one pool for the process
  (`engine_threads::shared`), whose threads the test binary's own std
  spawned once.
- A test that loads a game a handful of times (`physics2d_test`, the
  replays) is far from the limit.
