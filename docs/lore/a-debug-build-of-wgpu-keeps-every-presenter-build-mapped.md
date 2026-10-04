# A debug build of wgpu keeps every presenter build mapped

Measured 2026-10-04 in the presentation spike (get-3hd.1,
`//spikes/presentation:window_game` with `ENGINE_POISON_UNLOADED=1`;
[presentation-spike.md](../architecture/presentation-spike.md), section 1).

`spike_present` is a reloadable mod that links wgpu 30.0.1. In the
default build (`-c opt` with `-Cdebug-assertions=on`, `.bazelrc`), on the
4090 with its own device, both of its builds stayed mapped: the old one
at the reload, and the last one at shutdown. Poison mode reported each:

```
[poison] /run/user/1000/game-engine-thing/libs/spike_present-1104105-7.so stayed mapped after dlclose (a TLS destructor registered in it?); a stale pointer into it still works
```

That leaks 8.5 MB, the library's size, on every reload. Under
`--config=bench` (debug assertions off), nothing was reported: neither
at a reload on the 4090 (borrowing the platform's device), nor at a
reload on lavapipe with its own device, nor at that run's shutdown.

**Why (read in source, not isolated by experiment):** with
`debug_assertions`, wgpu-core turns on a `thread_local!` that traces
snatch-lock use (`wgpu-core/src/snatch.rs`, `SNATCH_LOCK_TRACE`). Its
first use on the main thread registers a TLS destructor in the
presenter's library, and glibc then keeps that library mapped until the
main thread exits. This is the mechanism of [a thread-local a mod touches
keeps its build mapped](a-thread-local-a-mod-touches-keeps-its-build-mapped.md).
It is the only debug-only thread-local the spike's audit found in wgpu.
parking_lot_core's `THREAD_DATA` would have the same effect if a wgpu lock
were ever contended, and nothing contends it on one thread.

**What it means:** in development builds a wgpu presenter's reloads
leak their images. Correctness is unaffected, since the old build stays
mapped rather than being unmapped under a pointer. Building the wgpu
crates without debug assertions would end it; so far that is only a
follow-up idea.
