# A thread-local a mod touches keeps its build mapped

Measured 2026-09-24 in `physics_test`
(`threads::threads_that_ran_a_builds_tasks_do_not_keep_it_mapped`): the
pile game with physics's parallel stages running on host threads, counting
the staged libraries under the engine's staging directory in
`/proc/self/maps` after loading, after reloading physics, and after
dropping the engine (the threads still alive):

| physics's tasks | threads | loaded | reloaded | engine dropped |
|---|---|---|---|---|
| as built | none | 5 | 5 | 0 |
| as built | spawned for each run (`Scoped`) | 5 | 5 | 0 |
| as built | kept between runs (a pool) | 5 | 5 | 0 |
| one touches a `thread_local!` holding a `Vec` | spawned for each run | 5 | 6 | 2 |
| one touches a `thread_local!` holding a `Vec` | kept between runs | 5 | 6 | 2 |

Five are the game's mods. As built, physics's code running on other threads
leaves nothing that keeps it mapped: the old build goes at the reload, and
everything when the engine drops. With one thread-local that has a
destructor touched in a task, the old build stays after the reload, and both
builds after the engine is gone, even with threads that exit after every
run: the calling thread runs tasks too, and it's the main thread, which
never exits. The first use of such a thread-local registers its destructor
with glibc (`__cxa_thread_atexit_impl`), which then keeps the library mapped
until that thread exits.

## What it means

Worker threads add no leak of their own: a task runs only inside the system
that made it, and a pool's idle loop is the host's code. The leak is the
open question in [hot-reload.md](../architecture/hot-reload.md) about
`thread_local!` in mods, now reachable from any task on any thread, and the
same as before on the main thread. Recycling kept workers at a reload
wouldn't fix it; not touching destructor-carrying thread-locals from mod
code would. Nothing checks that yet.

See also [a mod that spawns a thread is never
unmapped](a-mod-that-spawns-a-thread-is-never-unmapped.md), the same
mechanism through std's own thread-local.
