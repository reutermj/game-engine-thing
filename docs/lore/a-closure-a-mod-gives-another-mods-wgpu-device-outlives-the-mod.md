# A closure a mod gives another mod's wgpu device outlives the mod

Measured 2026-10-04 in the presentation spike (get-3hd.1,
`//spikes/presentation:window_game`, `--config=bench`,
`ENGINE_POISON_UNLOADED=1`; [presentation-spike.md](../architecture/presentation-spike.md),
section 3).

The resident `spike_platform` made a wgpu `Device`. The reloadable
`spike_present` borrowed it (`SPIKE_SHARED=1`) and used it from its own
copy of wgpu. That works, and is surprising in the other direction:
wgpu's dispatch is an enum, not a trait object, so the borrower's own
wgpu and wgpu-core code runs on the platform's data. Only the hal layer
calls through the platform's vtables. Reloads of the presenter were
clean.

Then the presenter registered an empty `queue.on_submitted_work_done(||
{})` after every submit and skipped its wait at unload (`hazard on`). The
next reload faulted in the new build's first `surface.configure`, which
polls the device and fires the closures that are done:

```
[poison] fault at 0x7f01305ce100, inside an unloaded build of .../libspike_present_mod.so
```

The process exited with 139. The closure's code and vtable were in the
old build. The device, owned by a library that stays, kept them, and
whichever library next polled or submitted called them.

The same holds for everything a mod hands a device or queue to call
later (read in source, wgpu 30.0.1): `map_async` callbacks, the
device-lost callback, and `on_uncaptured_error` handlers, which stay
until replaced. Waiting for the GPU (`device.poll(Wait)`) before
unloading fires the first two kinds and drops them; the other two have to
be removed by hand.

**What it means:** a GPU object owned by one mod is not safe to use from
another unless the user drains everything it registered before it
unloads, and the engine can't check that. presentation.md's D9 (GPU state
never crosses mods) is the rule that avoids it.
