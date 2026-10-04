# Presentation: spike results

**Status: spike results** (2026-10-04, get-3hd.1). It tests the riskiest
assumptions behind [presentation.md](presentation.md) (a sketch) before its
decisions are fixed. Nothing in the engine changed apart from one
build-rule convenience: `engine_mod` and `engine_game` now take `tags`, so
a mod can be `manual`. The spike is one package of manual targets,
`//spikes/presentation`, to be deleted once presentation.md's decisions
are made (CLAUDE.md, "Spikes are throwaway"):[^spike-code]

- `:probe`, `:draw_bench` and `:skia_bench`: plain binaries. They list the
  Vulkan adapters, measure wgpu's draw throughput and tiny-skia's raster;
- `spike_platform`: a resident mod that owns winit's event loop and one
  window. It serves a `Platform` service (`pump`, `window`, and the
  experiment's `shared_gpu`);
- `spike_windowed`: the realtime bootstrap, with a call to `pump` once a
  frame;
- `spike_draw`, `spike_scene` (two builds) and `spike_present`, all
  reloadable. `spike_draw` declares the vocabulary (`Place`, `Look`) and
  the `DrawList` flow, and runs the extract. `spike_scene` is the game.
  `spike_present` is the wgpu presenter;
- `spike_sink`: a presenter without a GPU, which reads and copies the list;
- `:window_game`: the windowed game. `:extract_game` and `:extract_bench`
  cover the extract and the hand-off in the engine, in lockstep;
- `:pong_window` and its launcher `:pong_window_launch`: pong in lockstep
  in a spectator window, for an agent to play
  ([section 7](#7-pong-in-a-spectator-window)), with `spike_pong_view` and
  `spike_lockstep_window`.

Machine: an RTX 4090 (NVIDIA 590.48.01) and lavapipe (Mesa 22.3.6,
LLVM 15), on X11 (`DISPLAY=:0`), Debian 12, Vulkan loader 1.3.239. The AMD
iGPU's Vulkan driver lists no adapter. Times come from `--config=bench`
unless they say otherwise.

**In short:**

- **The stack builds hermetically.** wgpu 30.0.1, winit 0.30.13 and
  tiny-skia 0.12.0 build under `rules_rs` with no patches and no BUILD
  overrides. Every mod links glibc and nothing else. Vulkan, X11, xcb and
  xkbcommon are all loaded with `dlopen` at runtime. One build script
  probes the host (x11-dl runs `pkg-config`), but only for a fallback
  path.
- **The window doesn't need the loop.** A resident platform mod pumps
  winit once a frame for 75 to 85 µs, called by the bootstrap. The window
  showed a reloadable mod's frame through `bazel run` reloads of the scene
  and of the presenter, and closed cleanly from its close button. So:
  **a platform service the bootstrap calls**, not a windowed bootstrap.
- **D9 holds, and D6 contradicts it.** A resident mod's wgpu device can be
  used from a reloadable mod's copy of wgpu. But whatever the reloadable
  mod registers (callbacks, error handlers) stays in the device after the
  mod is unmapped. A reload with one callback outstanding crashed the
  engine in poison mode. Keeping the device would save about 150 ms of the
  presenter's 220 ms reload. D6 puts the GPU device in the platform, which
  D9 rules out.
- **D2's incremental extract loses to a full rebuild in most cases.** A
  full rebuild costs 1.1 ns a drawable (0.11 ms at 100k, one thread),
  whatever changed. Incremental from change ticks is 3 to 4 times slower
  when the game moves its things in a walk with `&mut`, which marks every
  page written. It is 9 times faster only when under about 5% change and
  the game writes them by entity. A flow lives one frame, so a kept list
  costs a full copy into the flow every frame (0.45 ns an item) unless
  the flow carries only the delta.
- **Batching by material is required.** On the 4090 a draw per item costs
  100 times as much as one instanced draw per material: 47.6 ms against
  0.45 ms at 100k. lavapipe draws 100k instanced in 44 ms and 10k in
  4.8 ms.
- **The hand-off costs the copy.** A draw list made by one mod and read
  through a flow by another cost its reader 47.5 to 51.5 µs at 100k, against
  44 to 46 µs for the copy alone. Nothing is serialised. The only lock is
  the flow slot's uncontended try-lock.
- **tiny-skia** rasterises 10k anti-aliased shapes at 1280x720 on one
  thread in 19.3 ms (9.9 ms aliased), the same pixels every run.

## 1. Build

`spikes/presentation/Cargo.toml` adds, per [runbook 001](../runbooks/001-regenerate-cargo-lock.md):

- `wgpu = "30"`, without default features and with only `std`,
  `parking_lot`, `vulkan` and `wgsl`. This leaves out GLES/EGL, DX12,
  Metal and WebGPU;
- `winit = "0.30"`, with only `x11` and `rwh_06`;
- `tiny-skia = "0.12"`, with `std` and `simd`;
- `pollster` and `bytemuck`.

`cargo update --workspace` added 170 packages and changed no existing
version. Many of the 170 are `windows-*` crates that Linux never builds.
Everything compiled under `rules_rs` on the first try.

**What they need at runtime, and how they find it.** The mods' and
binaries' `NEEDED` entries are glibc only (`libc`, `libm`, `libdl`,
`librt`, `libpthread`, `ld-linux`):

| library | loaded by | how |
|---|---|---|
| `libvulkan.so.1`, then the ICDs from `/etc/vulkan/icd.d` and `/usr/share/vulkan/icd.d` | ash's `Entry::load` (wgpu-hal) | `dlopen` when the instance is made |
| `libX11.so.6`, `libX11-xcb`, `libXcursor` and others | x11-dl | `dlopen` by soname, then a `libdir` baked in at build time |
| `libxcb.so.1` | x11rb (`dl-libxcb`) | `dlopen` |
| `libxkbcommon.so.0`, `libxkbcommon-x11.so.0` | xkbcommon-dl | `dlopen` |

So the build is hermetic, and a machine without a GPU or a display still
builds everything. At runtime a game needs a Vulkan loader, a driver (or
lavapipe) and an X server, like any Vulkan game.

**One host probe.** x11-dl's build script calls `pkg-config` for 16
libraries and writes each `libdir` into `config.rs`. The sandbox can see
the host's `/usr/bin/pkg-config`, so this machine's build has
`x11: Some("/usr/lib/x86_64-linux-gnu")` and the same for `gl` (read
from `bazel-out/.../x11-dl-2.21.0/_bs.out_dir/config.rs`). It is only a
fallback after the bare soname, so it changes no behaviour here. But the
rlib's bytes depend on the host, which costs cache hits between machines.
The audit found no other build script that probes the host:

- wgpu, wgpu-core, wgpu-hal, naga and winit use `cfg_aliases` only;
- ash would read `VULKAN_SDK` only with `linked`, which isn't used;
- parking_lot_core reads `CARGO_CFG_SANITIZE`.

**Build time and size:**

| | |
|---|---|
| clean `//game` (baseline) | 15.7 s wall |
| then every spike target on top | +22.7 s wall, 91.7 s summed over 260 actions, 120 of them Rust. Critical path: x11rb-protocol (9.8 s), then winit, then the platform mod |
| largest new crates (summed action time) | x11rb-protocol 9.8 s, ash 6.3 s, naga 5.2 s, wgpu-core 4.0 s, x11-dl 2.9 s, wgpu-hal 2.1 s, wgpu 1.8 s, winit 1.6 s, tiny-skia 0.9 s |
| rebuild of `spike_present` (links wgpu) after an edit | 0.55 s |
| rebuild of `spike_scene` after an edit | 0.35 s |
| `libspike_present_mod.so` (wgpu) | 8.5 MB |
| `libspike_platform_mod.so` (winit, and wgpu for the experiment) | 7.9 MB |
| a mod with neither (`spike_draw`, `spike_sink`, `sequential`) | 0.5 to 0.7 MB |
| `libphysics2d_mod.so`, for scale | 3.7 MB |

The hot-reload loop doesn't slow down: wgpu's API isn't generic, so its
code is compiled once into its rlibs and a presenter edit relinks in half
a second. Every mod that links wgpu carries its own 7 to 8 MB copy, as
each mod carries its own copy of every crate.

**Debug assertions change wgpu.** The default build keeps
`debug_assertions` on (CLAUDE.md, "Every build is optimized"), and that
changes wgpu in two ways:

- `InstanceFlags::default()` becomes `debugging()`, which turns on Vulkan
  validation if the Khronos layer is installed. It isn't installed here.
- wgpu-core turns on a thread-local, its snatch-lock trace, which by
  its source is the likely cause of a leak. In the default build the
  presenter's old build stayed mapped at its reload, and its last build
  at shutdown: poison mode reported "stayed mapped after dlclose (a TLS
  destructor registered in it?)", 8.5 MB a reload. Under
  `--config=bench` neither of two reloads, nor the shutdown, left a
  build mapped ([lore](../lore/a-debug-build-of-wgpu-keeps-every-presenter-build-mapped.md)).

A presenter should set its instance flags explicitly. Whether to build
the wgpu crates without debug assertions is a follow-up.

## 2. The loop

`spike_platform` is resident. Its `load` makes winit's `EventLoop`, and
one `pump_app_events(Some(Duration::ZERO), ..)` delivers `resumed`,
where it makes the window. It never calls `run`. `spike_windowed` is
`realtime` plus one call a frame, `spike_platform::pump(cx)`, made before
the frame and next to `pump_loader`. It quits when the pump reports
`CloseRequested` or Escape. The presenter makes its surface from the
window's X11 handles, which `spike_platform::window` returns as numbers
(the `Display *` as a `u64`, the screen, the window id). It calls that
from its system each frame to follow resizes. A service call from a
system is allowed.

What ran, on `./bazel run //spikes/presentation:window_game` with
`ENGINE_POISON_UNLOADED=1` and 10 000 drawables:

- **The window opened** in 9 to 11 ms, inside the platform's `load`.
- **`./bazel run //spikes/presentation:spike_scene_b`** reloaded the
  scene (generation 1): the drawables stopped drifting and started
  swirling, and the window carried on. Screenshots before and after (with
  `xwd`) show both.
- **`./bazel run //spikes/presentation:spike_present`**, after an edit to
  the clear colour, reloaded the presenter. Its old build dropped its
  surface and device, and the new build made its own on the same window.
- **The close button** (a `WM_DELETE_WINDOW` sent to the window, which is
  what the button sends) went: platform `CloseRequested`, bootstrap
  `Status::QUIT`, engine shutdown. The presenter dropped its GPU side
  before the platform closed the window and event loop. The process
  exited 0.
- **Costs, the mean over 300 frames:** the window pump 74 to 86 µs a
  frame on X11, an idle window included (0.5% of a 60 Hz frame); the
  loader pump 0 µs between reloads.

What winit requires, read in its source and confirmed by running it:

- **The main thread.** Making the event loop panics unless
  `gettid() == getpid()`. The engine loads mods and runs frames there, so
  the platform's `load` and the bootstrap's pump qualify. A scheduler that
  ran systems on other threads would need the pump kept off them, which
  is one reason the bootstrap, not a system, should pump.
- **Residency, for a reason beyond convenience.** winit's X11 connection
  calls `XSetErrorHandler` with a function in its own library, and that
  handler is process-global in libX11. The library that makes the event
  loop must never be unmapped. Its `Window` methods also read per-library
  caches (`SUPPORTED_HINTS`, `WM_NAME`) that only the event loop's copy
  fills, so a window must not be driven from another mod's copy of winit.
- **`pump_app_events` is "not portable"** (its own docs). X11, Wayland,
  Windows and macOS have it; the web and iOS don't. presentation.md is
  Linux only (D10), so that's acceptable.

**Which design.** The window is a resource pumped once a frame, not the
loop, so the evidence supports **a platform service the bootstrap
calls**:

- **Bootstraps.** The realtime bootstrap gains one call. Lockstep gains
  the same call, plus a shorter wait in `pump_loader` (about a frame,
  rather than 1 s), so a spectator window stays responsive while no steps
  arrive.
- **A headless game** is the same interface with another provider: a
  `platform` with no window, whose `pump` returns nothing.
- **Choosing a provider.** `mod_deps` would make the bootstrap require
  the provider, and one provider per service is the rule. So the game
  should pick the provider the way `engine_game` already picks `threads`
  and `scheduler`.
- **Input.** Input comes out of `pump`, between frames, where the provider
  may use `cx.world()` and so could send action events (D4) itself.

A windowed bootstrap would duplicate every bootstrap for each platform,
and gains nothing the service doesn't give.

## 3. Per-library statics: what of wgpu crosses mods

A source audit covered wgpu, wgpu-core, wgpu-hal (Vulkan), wgpu-types,
naga, ash, gpu-allocator, parking_lot, winit and the X11 crates. The
experiment then had the presenter borrow a device that the resident
platform made (`SPIKE_SHARED=1`; the platform's `shared_gpu` hands over
the address of its `SharedGpu` as a `u64`, since no wgpu object may
cross a call).

**How a call on another mod's wgpu object runs.** wgpu's dispatch is an
enum (`dispatch.rs`, `DispatchDevice::Core(Arc<CoreDevice>)`), not a
trait object. So when the presenter calls a method on the platform's
`Device`, **its own copy of wgpu, wgpu-core and naga runs**, on the
platform's data. That works because both libraries link the same rlibs,
under the one-compiler rule. Only the hal layer goes through the
creator's code: wgpu-core holds `Box<dyn hal::DynDevice>`, whose vtable
is in the platform. Raw resources (buffers, textures) are boxed there
too, so a buffer the presenter makes has the platform's drop code. wgpu-core keeps no global registry: its hub lives in
the instance's `Arc<Global>`.

**What isn't safe.** Each of these is a way for the presenter's code or
statics to outlive or diverge from the presenter:

- **Callbacks the reloadable mod registers stay in the resident device.**
  This covers `map_async` callbacks, `on_submitted_work_done`, the
  device-lost callback and an uncaptured-error handler. They are fired by
  whichever library next polls or submits, which can be the next build,
  after this one is unmapped.

  *Measured.* With `hazard on`, the presenter registers an empty
  work-done closure on every submit and skips its wait at unload. The
  reload then faulted in poison mode at the new build's
  `surface.configure`: "fault at 0x7f01305ce100, inside an unloaded build
  of .../libspike_present_mod.so". The engine exited with 139. Without
  the hazard, the same reloads were clean. ([lore](../lore/a-closure-a-mod-gives-another-mods-wgpu-device-outlives-the-mod.md))
- **parking_lot's parking table is a static.** A thread parked on a wgpu
  lock through one copy is never woken by an unlock through another copy,
  which is a permanent deadlock. This needs two threads using wgpu on one
  device at once. wgpu spawns no threads, so today nothing does that.
- **Error scopes are keyed by `std::thread::current().id()`,** and each
  copy of std numbers threads differently. A scope pushed through one
  copy is missed when popped through the other.
- **`log`:** a copy's wgpu log lines go to that copy's logger, so a mod
  that installs none drops them.

**What the device would save.** These are presenter loads and reloads;
"drop" is the old build's `unload` (a wait for the GPU, then dropping the
surface and the device), and "make" is the new build's `load`. The 4090
figures for its own device come from the default config, and the shared
ones from bench:

| | first load | reload: drop | reload: make |
|---|---|---|---|
| 4090, its own device | 228 ms: instance 52, adapter 22, device 97, surface 49, shader 0.1, pipelines 7.2 | 72 ms | 147 ms: instance 33, adapter 4.3, device 76, surface 33, pipelines 0.5 |
| 4090, the platform's device | 193 ms: the platform makes the device (141), surface 47, pipelines 5.0 | 24 to 28 ms | 44 ms: surface 43.5, pipelines 0.4 |
| lavapipe, its own device | 67 ms: instance 46, pipelines 11 | 9 ms | 45 ms: instance 34.5, device 4.8, surface 0.5, pipelines 1.4 |

Pipelines aren't the cost. NVIDIA's driver keeps its own pipeline cache
on disk, so 4 pipelines take 7 to 15 ms cold and 0.4 to 0.5 ms after
that. A shader module takes 0.1 ms. The device and the instance are
three quarters of a reload, and the surface (the swapchain) is most of
the rest. Even a kept device leaves a 70 ms reload, since the surface
would have to be kept too to save more. The first frame after a reload
took 0.34 ms on the 4090 and 8.6 ms on lavapipe (its JIT).

**A presenter that fails.** A validation error with wgpu's default
handler (`provoke`: 64 bytes into a 4-byte buffer) panicked in the
presenter's own copy of wgpu. That copy made the device, so the handler
was its own. The panic was caught at the mod boundary and the presenter
marked failed, while the game and the window carried on. Through a
borrowed device it would depend on which copy installed the handler.

**What this says.** Nothing of wgpu survives a presenter reload *safely*
unless the reloadable mod has drained before it goes. That means waiting
for the GPU, registering no callbacks or handlers (or removing them),
leaving no error scope open, and never using wgpu from two copies at
once. The engine can't check any of that, and getting it wrong is a
crash or a deadlock. The prize is about 150 ms of a 220 ms pause, during
development only (get-y5t: reloading is a development tool). D9 is right
to forbid it.

## 4. Extract cost

`spike_draw`'s `extract` (render phase) turns `(&Place, &Look)` into the
frame's `DrawList` in one of three modes:

- **full** walks every row into the flow;
- **incremental** keeps the list in its transient part, updates it from
  `for_each_written(since)`, and copies it whole into the flow, because a
  flow lives one frame;
- **delta** keeps the same list but puts only the changed `(slot, item)`
  pairs in the flow, and the reader keeps its own copy.

A row arriving or leaving rebuilds the list (`arrived_since`,
`left_since`), which happens once, at the first frame. `spike_scene`
moves 1%, 10% or 100% of its drawables a frame, a different share each
frame, in one of two ways:

- **spread:** every hundredth row (more, for more churn), found by a walk
  of all of them with `Query<&mut Place>`;
- **block:** one run of rows in a row, reached by entity with
  `Query::with`.

Times are µs a frame from the sequential scheduler's node timings
(`:extract_bench`, the median of 3 runs of 60 frames, one thread). The
full tables are in the bench's output. These are the 100k rows, with the
1k and 10k rows scaling linearly:

| moving | how | full | incremental | delta | reader (full list / delta) |
|---|---|---|---|---|---|
| 1% | spread, `&mut` walk | 109 | 387 | 340 | 48.7 / 1.7 |
| 10% | spread, `&mut` walk | 108 | 441 | 407 | 48.3 / 9.0 |
| 100% | spread, `&mut` walk | 108 | 919 | 997 | 48.9 / 57.0 |
| 1% | block, by entity | 105 | 57 | **11.8** | 48.3 / 0.6 |
| 10% | block, by entity | 106 | 137 | 101 | 47.6 / 5.7 |
| 100% | block, by entity | 108 | 918 | 994 | 48.2 / 55.7 |

- **A full rebuild costs 1.05 to 1.1 ns a drawable**, whatever changed:
  1.2 µs at 1k, 11 µs at 10k, 105 to 110 µs at 100k. That is a sixtieth
  of a 60 Hz frame at 100k, on one thread, before any `ParMap`.
- **Incremental pays per row walked, not per row changed, unless pages
  are skipped.** `for_each_written` skips a page whose tick is old, but
  any system that takes `&mut` over a page marks the whole page written,
  whether or not it writes (`ErasedColumn::as_mut_slice_ticked`). A game
  that walks its things with `&mut` therefore defeats page skipping, and
  the walk then costs about 3.4 ns a row against the full extract's 1.1
  ([lore](../lore/a-walk-with-mut-marks-every-page-written.md)). Physics
  writes every awake body each step in exactly that way.
- **With precise pages and few changes, delta wins nine times over:**
  11.8 µs against 105 at 1% of 100k. A changed row costs about 10 ns
  (`for_each_written`'s per-row fetch, the slot lookup and the push).
  Break-even is about 10% of rows changed, in pages that are clustered.
- **A flow can't keep the list.** Flows are emptied at the end of the
  frame (flows.md), so an incremental extract has to copy its kept list
  into the flow every frame: 45 µs at 100k, half the cost of a full
  extract. Only a delta in the flow, with each reader keeping its own
  copy, keeps the cost proportional to what changed. A reader loaded
  later then needs a full frame, which the `full` flag gives.

## 5. Draw throughput

`:draw_bench` renders N shapes to an offscreen 1280x720 RGBA8 target.
Each shape is 4 to 16 px, half rects and half circles (discarded in the
fragment shader), in four materials. A material is a pipeline with its
own blend state. A frame is uploading the items (`write_buffer`, 32
bytes an item), encoding, submitting, and waiting for the GPU. Times are
in ms, the median of 30 frames (15 on lavapipe), with encode alone in
brackets:

| items | adapter | a draw per item, materials interleaved | a draw per item, by material | instanced, a draw per material |
|---|---|---|---|---|
| 1k | 4090 | 0.54 (0.19) | 0.096 (0.048) | **0.050** (0.010) |
| 10k | 4090 | 5.1 (2.1) | 0.54 (0.43) | **0.092** (0.040) |
| 100k | 4090 | 47.6 (19.1) | 5.0 (4.2) | **0.45** (0.33) |
| 1k | lavapipe | 14.0 (0.2) | 0.91 (0.05) | **0.62** (0.01) |
| 10k | lavapipe | 90.6 (2.6) | 8.3 (0.9) | **4.7** (0.04) |
| 100k | lavapipe | 486 (10–17) | 79 (9.2) | **44.4** (0.29) |

- **Instancing by material is required, not an optimisation.** A draw
  costs about 50 ns to encode in wgpu, or about 200 ns with a pipeline
  switch. Per-item drawing at 10k is a third of a 60 Hz frame on a 4090.
  Instanced, 100k shapes take 0.45 ms, most of it the 3.2 MB upload.
- **So the extract must hand items over in material runs.** The scene
  spawned in material order, so the query's table order was already
  batched. An extract can't assume that. A material as an ordered key
  (relationships.md), or a counting sort into runs, would keep it.
- **Each adapter drew the same pixels in every run.** The instanced
  frame's hash matched across two runs on the 4090 and across two on
  lavapipe. The two adapters' hashes differ from each other, as D3
  expects. Whether lavapipe is stable across Mesa versions wasn't
  measured.
- **The windowed presenter** spent 0.16 to 0.36 ms of CPU a frame at 10k
  on the 4090 (upload, encode, acquire, present; not vsynced) and 5.4 ms
  on lavapipe.

## 6. The hand-off

`spike_draw` (one mod) makes the `DrawList` flow, and `spike_sink`
(another mod) takes `See<DrawList>` and copies the list into memory it
owns, as an upload would. "Copy alone" is the same bytes copied by
`extend_from_slice` in the bench's own process (µs):

| items | the reader's system, full list | copy alone |
|---|---|---|
| 1k | 0.5 | 0.4 to 0.8 |
| 10k | 4.4 to 4.8 | 4.0 to 4.3 |
| 100k | 47.5 to 51.5 | 44.3 to 46.6 |

The reader pays the copy plus 0.1 to 5 µs, which covers its system's
dispatch and the flow's one use. Nothing else appears:

- **No serialisation.** The flow's type comes from the interface crate
  both mods compile against, so `See` hands out a reference into the
  value the maker filled.
- **No other lock** than the flow slot's uncontended `try_*` per use
  (flows.md, "The store").
- **No copy of its own.** The one copy is the reader's upload.

A service would be no cheaper. `&[Item]` doesn't cross a call
(`Crossing`), and a `Vec<Item>` argument moves without a copy but takes
the list from its maker, so it serves one reader.

## 7. Pong in a spectator window

The spike's last question: can a person watch an agent play a real game,
in lockstep, in a window, with nothing in the game changed? `:pong_window`
is pong's mods (`//pong/ai`, `//pong/text`, unchanged) on a lockstep
bootstrap with a window, plus `spike_draw`, `spike_pong_view` and
`spike_present`. An agent plays it over modctl as pong_test does
(`spikes/presentation/AGENT.md`).

- **The adapter is a stage, not an extract.** `spike_pong_view` reads
  pong's components (`Ball`, `Paddle`, `Goal`, the walls as colliders,
  `Score`, and the `Clock`) and adds 40 to 50 rects to the frame's list
  with `Pass<DrawList>`, after `spike_draw`'s extract, which makes an
  empty list in a world without `Place`s. A flow has one maker, so a
  second producer is a `Pass`, as flows.md says. It draws on a fixed
  canvas (1184x680: the 40x20 court at 28 px a cell, a score band
  above); the list now carries its canvas size, and the presenter
  letterboxes it into the window.
- **Lockstep needs two things the realtime bootstrap doesn't.** The
  variant (`lockstep_window.rs`, mod name `lockstep`, so agents' commands
  don't change) waits in `pump_loader` for at most 16 ms rather than 1 s,
  then pumps the window. And when that pump saw events (an expose), it
  asks the presenter to draw the last frame again, since no frame runs
  while idle. The bootstrap is resident and so may depend only on resident
  mods (defs.bzl), so it can't name the presenter: the `Redraw` service is
  declared in the platform's interface and *provided* by the presenter,
  and the call resolves by name to whichever presenter build is loaded.
  A service whose interface lives with a resident mod but whose provider
  reloads is the shape a presenter-agnostic bootstrap needs. Measured: two
  exposes sent while idle (`XClearArea`) gave one redraw.
- **Pacing is the bootstrap's.** With a window and pacing on (the
  default; `lockstep pace on|off|<speed>`), each frame of a step starts a
  frame after the last one, the previous step's included, so `step 6`
  sent ten times a second paces like `step 60`. Every frame's `dt` is the
  same either way, so pacing changes when frames run, not what they
  compute. Measured: 1200 frames of an agent's commands (`step 6` at a
  time) took 20.16 s paced and 0.54 s unpaced, from fresh engines, and
  ended in the same `state` to the last digit. 600 paced frames took
  10.07 s (59.6 fps).
- **The window stays live.** Closing it while idle quits at once; closing
  it during `step 600` stopped the step after 283 frames, replied
  `frame 283 (the window was closed after 283 of 600 frames: quitting)`
  and shut the engine down with exit 0. `bazel run` of `spike_pong_view`
  (a ball recoloured) and of `spike_present` mid-game reloaded each,
  twice, under the running game. A reloaded presenter has no last frame
  until the next step, so an expose in between shows the clear colour.
- **Tiling window managers.** The platform's window is now fixed-size
  (min = max = the game's resolution, `SPIKE_WINDOW`), typed a dialog,
  with `WM_CLASS` `"pong", "game-engine-thing"`, which i3 floats. A real
  platform would take the resolution from the game, not from the
  environment.

### Two agents against each other, in turns

`:pong_versus` (`./bazel run //spikes/presentation:pong_versus_launch`,
socket `/run/user/1000/pong-versus/control.sock`) is the same window with
no `pong_ai` and no `pong_text`, for two agents playing `left` and
`right` (`spikes/presentation/AGENT_VERSUS.md`). In lockstep whoever sends
`step` moves time for both, so the faster thinker would get more turns:
the game needs a **turn barrier**. A turn is N frames (6 by default,
`turn length`); each side submits `lockstep turn <side> <up|down|stay>`,
and when both have, the turn's frames run, paced, and the next opens.

- **The barrier is the bootstrap's.** Only the bootstrap makes time move:
  a reloadable mod's message handler can't run a frame, and the barrier's
  state (who submitted what) has to survive any reload of the game. So
  `lockstep_window.rs` keeps it (on with `SPIKE_TURNS=<frames>`, set by the
  launcher; `step` is then refused), and publishes it as data, a
  `spike_turns::Turn` component, from a new resident interface mod
  (`spike_turns`, shaped like `clock`, since the resident bootstrap may
  depend only on resident mods). What a turn *means* is the game's:
  `pong_versus` (reloadable) sets both paddles' `intent` from it each
  frame in `update`, as `pong_ai` sets the right one, and spawns a
  `spike_turns::Outcome` when a side reaches the point limit (5 by
  default, `first-to`), which the bootstrap reads after every frame and
  stops on, mid-turn if need be. Pong's code doesn't change.
- **Requests never wait for a turn.** A submit replies at once. The
  completing submit starts the turn, and its frames run one per pass of
  the bootstrap's loop, between pumps that wait only until the next frame
  is due, so polls are answered while it plays. What the other side
  submitted stays in the bootstrap's state until the turn plays; the
  world shows only that it has.
- **One `state` for both sides.** `pong_versus state` names `left` and
  `right` (never "you"), with a turn line: number, open or playing (frame
  k of N), who has submitted, the point limit, or the winner.
- **The window shows the barrier.** Watching two agents, turns that
  waited long on a slow thinker looked like a hang, so the score band
  now shows it (AGENT_VERSUS.md, "The score band"): a lamp per side, lit
  once it has submitted; a wait timer in whole seconds beside each side
  still to; a play triangle while a turn runs, pause bars while it's
  open; the turn number; the winner framed at game over. That band
  changes while no frames run, which a redraw of the last frame's list
  can't show, so a stage can now *restage* between frames: a
  `spike_draw::Restage` service (provided by `spike_pong_view`, called by
  the presenter's `Redraw`) returns items drawn over the last list,
  starting with the band's own opaque background. Its provider reads the
  world with `cx.world()`, which a redraw, outside any frame, allows. The
  bootstrap redraws on a change to the barrier and every 250 ms while a
  turn is open. Wall-clock time lives only in the view's transient (when
  it first saw the barrier's status change), so it reaches pixels, never
  the world: the same 150-turn script with three 1.3 s waits ended in the
  same `state` paced (18.9 s) and unpaced (4.7 s). The general shape, a
  view whose model changes between frames, is one a real presenter
  will meet again (a pause menu, a loading screen): here, the stage that
  draws something also answers for it between frames.
- Measured, on the 4090 with the window up: with only `left` in, the frame
  stayed at 0 for 1 s; `right`'s submit replied in 1.2 ms, and the 6-frame
  turn played out in 85 ms with polls seeing frames 1 to 5 go by. Two
  threaded players (one modctl process a call, polling every 10 ms)
  played 600 turns in 60.27 s (11999 polls, 1198 submits): pacing, not
  the barrier, sets the rate. A fixed submission script gave the same
  final `state` to the last digit paced and unpaced, from fresh engines:
  to a 5-0 game over at frame 637 (mid-turn) in 10.66 s against 0.54 s,
  and with no point limit, 600 turns and 13 points in 60.31 s against
  2.87 s. Closing the window mid-turn (turn 640, frame 4 of 6) shut the
  engine down with exit 0.

### The playtest loop: record, replay, probe, flag

After a versus match, one agent reported both its lost points as a
collision bug, and other reports followed (the ball beyond the walls, past
the paddle faces, a return that lost most of its speed). Nothing was
recorded, so none of it could be checked. The windowed games now record
every session, and a tool replays one headless and flags frames for a
reviewer agent (`spikes/presentation/REVIEW.md`, playtesting-research.md's
record and replay).

- **Recording is inputs, not frames.** Physics is deterministic, so a
  session is its inputs at the frames they took effect, and the rest is
  checks: the launcher writes what was played (game, commit, dirty tree,
  turn length) and the bootstrap appends JSONL as it happens: steps,
  turns (and each submission's wall time), `pong_text` steers, points, a
  check of the ball, paddles and score after every step or turn, and
  agents' `lockstep note <text>`. The bootstrap writes the file because
  it is resident (never unloaded, like the window it owns), but it can't
  name pong's types, so a game mod (`spike_pong_record`) fills a
  `spike_record::Watch` at the end of each frame and the bootstrap reads
  it after. The `Watch` sits on the bootstrap's clock entity, so recording
  spawns nothing; it's there whether or not a log is written.
- **The replay is the game, in-process.** `pong_replay` loads the
  recorded game's mods into an `Engine` as the integration tests do (the
  same bootstrap and mods, `spike_capture` for the presenter, no display),
  runs it a frame at a time through the bootstrap's own messages (`step
  1`; in turns, `turns N` and `turn frame`, which plays one frame as the
  loop does), and reads the world between frames, since it isn't a mod.
  5062 and 5666 frames replayed in 0.40 and 0.48 s, probes and flags
  included.
- **Measured.** A one-player session (5062 frames, 769 steps, 250 steers,
  6 points, one paced step) and a versus match (5666 frames, 2076 turns of
  6 and 1 frames, 3 points, game over) each reproduced every check bit for
  bit. The one-player session also reproduced on plain `//pong` (the
  standard lockstep, no recorder, view or window; `--plain`), with every
  frame's probes byte-identical, so nothing the spike adds changes the
  game. One steer moved a frame later was caught at the next check (frame
  1050, the paddle 0.267 off), and one versus action changed at the check
  ending its turn; both exit 1. Frames rendered twice are the same bytes.
- **Frames come from the draw list.** `spike_capture` (a `See` after
  `spike_pong_view::draw`) keeps the last list and replies it as text;
  the tool rasterises it with tiny-skia, linear colours encoded to sRGB as
  the window's surface does, into PNGs it encodes itself (no `png` crate;
  tiny-skia's encoder is behind a feature this build leaves off).
- **What the detectors found on scripted play** (REVIEW.md says how to
  read them), reported, not diagnosed:
  - `hit_no_bounce`, 4 of 121 paddle hits, every one a point: the ball
    met the paddle's *end* (centre 2.28 to 2.60 off the paddle's centre,
    past its 2.25 face), physics pushed it vertically, and pong's `rebound`
    still applied its kick (`vy` to ±40 in one case) and speed-up, so `vx`
    kept its sign and the ball went in: the agent's turn-1036 point. Three
    came at 31 to 44 cells/s from a script aiming at the paddle's end; the
    fourth at 17 cells/s in one-player play.
  - `deep_penetration`, 80 runs: the ball inside a paddle's box by up to
    0.51 cells (at 35 to 40 cells/s), and at the deepest frame of every
    run physics held no contact yet; it is found, and the ball turned, a
    frame later.
  - `beyond_wall`, 79 runs. The walls stand a radius outside the court, so
    the ball's centre turns at y = 0 and 20, not at 0.25 and 19.75 as
    AGENT.md tells players: most runs are ordinary bounces. In 53 the ball
    was inside a wall's box too, by up to 0.42 cells (a 40-cell/s ball the
    paddle's end had just kicked upward).
  - `speed_loss`: none. The reported 37 to 12.9 return wasn't provoked.

## What it means for presentation.md

- **D2 (incremental extract).** The premise that cost follows what
  changed holds only for a few changes, clustered in pages, written by
  entity. Otherwise a full rebuild is cheaper, and at 1.1 ns a drawable it
  is cheap. Recommend: **a full rebuild by default**, and a delta flow as
  an option for large sets that rarely change (a tilemap, sleeping
  bodies), where its 9x shows. "Kept between frames" can't be a flow, so
  a kept list lives in the extract's transient part, and its readers keep
  their own copies. The ECS change that would make change ticks pay is
  pages marked only on a real write, which is its own decision.
- **D6 (the platform).** **Take the GPU device out of the platform.** D6
  and D9 contradict each other: the device can't live in the platform if
  GPU state never crosses mods. The platform owns the window, the event
  loop and input. It is resident, since winit's X11 error handler points
  into it. It provides a service that every bootstrap calls once a frame,
  with a headless provider, and the game picks the provider the way it
  picks threads and the scheduler. The window crosses as plain numbers
  (X11 handles), from which a presenter makes its own surface.
- **D8 (overlap).** The GPU already runs alongside the next frame: once
  `write_buffer` has copied the list into wgpu's staging, `submit`
  returns, and the flow can be emptied as it is today. What pipelining
  would hide is the presenter's CPU time: 0.2 to 0.4 ms at 10k on the
  4090, and about 1 ms at 100k counting the upload. That is worth
  measuring in a real game before building a value that outlives its
  frame. If it is built, `Take::into_inner` keeps a list past its frame
  without a copy, at the cost of the bin's allocation.
- **D9 (GPU state never crosses mods).** Confirmed: sharing works until it
  doesn't, and when it fails it's a crash or a deadlock (above). **Keep
  it**, and record the reload cost it implies: about 0.2 s on the 4090,
  0.05 s on lavapipe, development only.
- **"Who owns the loop":** the bootstrap. The platform is pumped, never
  run.
- **"What survives a renderer reload":** nothing GPU-side. Pipelines
  aren't worth keeping, since the driver caches them. The device could
  survive only under a drain discipline the engine can't check.
- **D3 (presenters).** tiny-skia costs about 2 µs an anti-aliased shape
  on one thread at 1280x720 (19 ms at 10k, 194 ms at 100k) and gives the
  same pixels every run: fine for golden images and for vision agents'
  observations at a few a second. lavapipe is faster (4.7 ms at 10k) but
  is a GPU path with its own pixels.
- **D10 (the stack).** It builds and runs as chosen. Add to it: Vulkan
  only, X11 only, and the instance flags set explicitly.

## Recommended changes to the sketch

1. **D6:** the platform is the window, the event loop and input, a
   resident one-provider service (winit, headless or none) that the
   bootstrap pumps once a frame. Picked per game in `engine_game`. No GPU
   device in it.
2. **D9:** keep it. Name the reload cost (about 0.2 s on the 4090) and
   the reasons: callbacks that outlive the mod (demonstrated), parking_lot
   across copies, error scopes across copies.
3. **D2:** a full rebuild by default, with material runs in its output.
   Incremental as a delta flow for large sets that rarely change, and
   only where the writers mark rows precisely. Drop "kept between frames"
   from the flow itself.
4. **D8:** measure the presenter's CPU share in M1 before building
   overlap. The GPU's own overlap is free.
5. **Open questions:** close "who owns the loop" (the bootstrap; the
   platform is a pumped service) and "what survives a renderer reload"
   (nothing; the numbers above). Add one: building wgpu without debug
   assertions, or living with a leaked presenter image a reload in
   development.
6. **M1:** the presenter carries its dependencies' license texts in its
   runfiles, as the `threads` mod does (docs/CREDITS.md).

## Where the spike's unsafe is

There are three blocks, all in `spikes/presentation/present.rs`:

- dereferencing the platform's `SharedGpu` address (the experiment, which
  is the point: it shows what D9 forbids);
- `create_surface_unsafe` from the window's raw handles;
- casting `&[Item]` to `&[DrawItem]`, the same `repr(C)` layout.

A real presenter needs the second, or a safe wrapper over the platform's
handles. The interface's `Item` would be `Pod`, if interfaces could take
crate dependencies (`engine_mod` gives an interface `//engine/api` only).

[^spike-code]: (2026-10-04) The spike's code is `spikes/presentation/`
    and its targets, on the branch that added this report; delete it in
    the change that lands presentation.md's decisions, and name the
    commit it last built at here.
