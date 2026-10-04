# Threads

**Status: built** (2026-10-03; get-znt.29, the design; get-znt.20, the
pool; get-znt.34, shapes run across it; get-znt.39, the one-thread
guard; get-znt.40, the solves' batches filled across it; get-znt.31,
every split on a shape or the ECS's own; get-znt.45, the write-back
across it). The running
engine has threads: a resident mod, `threads`
(`engine/std/threads`), keeps a rayon pool on one CCD and installs it as
the world's executor, and `Passes::run` hands a program's stages out
across it as a task graph (`engine_ecs::dispatch`), the dispatch spike's
`affine` protocol. Both physics solves run across it in `//game`, pong,
the platformer and pile3d, bit for bit what they are on one thread.

How the solve's passes came to be a shape the scheduler runs:
[flows.md](flows.md#parallel-shapes). How they were measured before the
engine had threads: [dispatch-spike.md](dispatch-spike.md). What the
solves are: [physics.md](physics.md#solving-across-threads).

**In short:**

- **The pool is a resident mod**, not the loader and not the bootstrap:
  its threads run rayon's code between frames, which only a resident
  library can hold, and a mod is where policy (how many, where, how warm)
  belongs. Mods reach it only through the world's `Executor`, a trait
  object; the loader is unchanged. The scheduler mod stays `sequential`,
  reloadable.
- **Rayon hosts the threads and nothing more.** A dispatch is one scope:
  a spawn a worker, the calling thread as worker 0, the protocol inside
  handing out the blocks. Never rayon's global pool, and never its work
  stealing for blocks.
- **One CCD, one thread a core,** found from the cache topology in sysfs
  and pinned with core_affinity in rayon's `start_handler`; 8 on this
  machine. The frame's thread, which dispatches and is worker 0, is
  pinned to the CCD's first core at its first dispatch: left to the OS,
  it made a turning step at 8 threads 1.7 to 2.0 times as long.
- **Warm for a millisecond:** after a dispatch a worker spins that long
  for the next before rayon parks it: at none, the settled pile's passes
  took 1.8 times as long at 8 threads, at 5 ms 3% less than at 1.
- **Every shape runs across threads**, on one dispatch: `Passes` a stage a
  color or pass, `ParMap` and `Reduce` one stage of blocks. A system has
  no other way to fan out (get-znt.31 removed the last); the ECS's own
  work inside a node (`near_pairs`, a `Live`'s broadphase, the re-sort's
  re-bounding) splits on the same dispatch, crate-private (`Split`).
- **Bit for bit** at 1, 2, 4 and 8 threads: both baselines, short and
  long, byte-identical to before; the 3D fingerprint; pong's and the
  platformer's replays.
- **What it bought**, the real engine on step_bench at 8 threads against
  one: 2D's turning pile of 10 000 and pyramid of 5050, the whole step 2.2
  to 2.4 times as fast (passes 4.8 and 5.2), the falling pile 1.3; 3D's
  10 000 boxes settled 2.3 to 2.5 (passes 4.4 to 5.5), falling 1.9,
  planks 2.1.
- **The fill in the passes** (get-znt.40): the batches filled in the
  program's first stage, `prepare` left with the coloring and seating
  (2D's 568 µs → 122 at 8 threads, 3D's 2360 → about 275). At 8 threads
  2D's settled solve 1818 µs against 2150-2240, 3D's settled boxes
  4246-4284 against 6655-7375; on one thread within 1% but 3D's falling
  boxes, +1.4 to 1.7% pooled ("Measured").

## Where the pool lives

Three places could own the threads. What each has to satisfy:

1. **Rayon's code is never swapped while a thread is in it.** A worker
   parked or spinning between dispatches is in rayon's code, in whichever
   library made the pool; unmapping that library under it is a crash.
2. **Mods reach the pool only through the world's executor**
   (`World::set_executor`, `Arc<dyn Executor>`), a trait object, so no
   mod links the pool or depends on whoever made it. And the world hands
   it to no one: a system runs on it only through a declared shape, and
   `World::executor` is crate-private, so a mod holding the world (a
   message handler, a hook, `WorldMut`) can't take it and run tasks
   undeclared (get-znt.47). Its owner removes it with
   `take_executor_if(&ours)`, which hands back nothing the owner didn't
   hold; `set_executor` returning the old one would have, to anyone, by
   swapping and restoring.
3. **The loader stays bare** (CLAUDE.md).

| home | 1: never swapped | 2: the world's executor | 3: the loader bare | as policy |
|---|---|---|---|---|
| a host service in `engine/api`, the loader's | yes: the binary | yes | no: the loader links rayon, reads the topology, owns a placement and warmth policy | fixed in the binary; a game can't replace it |
| **a resident mod (built)** | yes: resident builds are never swapped, and it is closed last | yes: it installs it at load, removes it at close | yes: nothing in the loader | a mod: a game picks another, or configures it |
| the bootstrap | yes: resident | yes | yes | tied to time policy: both bootstraps would carry it, and a game replacing its bootstrap would lose its threads |

**The resident mod** is where hot-reload.md already puts threads ("Resident
mods are the right home for threads"), and where scheduling.md said the
parallel scheduler's workers would be. It is its own mod rather than the
scheduler's, because a scheduler that owned threads would have to be
resident, and scheduling policy would stop hot-reloading; `sequential`
keeps reloading like gameplay. When systems run in parallel (get-znt.5),
a scheduler reaches the same pool through the world's executor, as
`Passes` does, and runs a node's work inside a dispatch that returns
before `run_frame` does, so it can stay reloadable too.

What being a mod needs, and has:

- **Load and close.** `load` makes the pool and installs it
  (`set_executor`, between frames). `close` takes it out of the world if
  it is still there (`take_executor_if`: a test may have put its own in;
  `threads_test` holds both), and its transient
  part, dropped after, joins the threads while the library is mapped: the
  pool spawns them itself (`spawn_handler`) to keep the handles, since a
  dropped rayon pool's threads leave on their own time.
- **First in, last out.** `engine_game`'s `threads` attribute (default
  `//engine/std/threads`, `None` for none) puts it first in the manifest,
  so it is closed after every mod whose shapes ran on it. It must be
  resident; `engine_game` fails the build otherwise.
- **Statics.** Every mod library has its own copy of every crate's
  statics, rayon's global registry included, so a global pool would be
  one a library. Only the `threads` mod links rayon, and its pool is an
  explicit `ThreadPool`.
- **Mapped until exit.** Spawning a thread registers a TLS destructor in
  the spawning library's std on the spawning thread, which keeps that
  library mapped until the thread exits
  ([lore](../lore/a-mod-that-spawns-a-thread-is-never-unmapped.md)). For
  a resident mod that changes nothing; tests that count mapped builds
  leave the thread host out (`physics2d_test`'s `images`).
- **A TLS key a load.** Each load's copy of std takes a pthread key for
  its threads' cleanup and never gives it back, and a process has 1024:
  the 3D long baseline, an engine a run, aborted after 1020 loads of the
  host ([lore](../lore/a-mod-that-spawns-threads-takes-a-tls-key-each-load.md)).
  A game loads it once. The physics harnesses, which make engine after
  engine, load their scene games without it and install one pool for the
  process instead (`engine_threads::shared`), sized by the same
  environment.

**Considered and not built:** the pool in `engine_ecs`, beside `Scoped`.
Every library links `engine_ecs`, so its code would be in whichever
library made the pool, a reloadable one as easily as not; and keeping
threads that run a borrowed closure is the unsafe step rayon's scope
already takes (`tests/pool.rs` took it by hand).

## Placement

A `Placement` (`engine_threads::Placement`) is the CPUs the threads go on,
one a thread, the calling thread's first:

- **`OneCcd`, the default:** the CPUs sharing the last-level cache with
  the lowest CPU the process may run on (`cache/index*/shared_cpu_list`,
  the highest `level`), without SMT siblings
  (`topology/thread_siblings_list`); CPUs 0 to 7 here. Spread over both
  CCDs, a solve's passes take 1.9 times as long; on a core and its sibling,
  5% less (dispatch-spike.md, "What the pool must provide").
- **`Cpus(list)`**, for a game that knows better.

Worker `i` is pinned to CPU `i + 1` in rayon's `start_handler`
(`core_affinity::set_for_current`, the crate's `sched_setaffinity`).
**The calling thread is pinned to the first CPU**, the one no worker has,
at its first dispatch (`Settings::pin_caller`). It is the bootstrap's,
running the frame, and the pool doesn't own it; but it is worker 0 of
every dispatch and runs the frame's serial work between them, and left to
the OS, on step_bench at 8 threads, a turning step took 1.7 to 2.0 times
as long and one not turning 1.2, the serial `prepare` included
("Measured"). A pool that many threads dispatch on
at once (a harness's `shared` one) pins no caller, which would pile
them on one core.

**Placement is read before anything is pinned**: the CPUs the process may
run on, as the first thread to ask found them. Asked from a thread a pool
has pinned, `get_core_ids` answers that thread's one CPU, and a second
pool made there would put every thread on it, its unpinned workers
inheriting the mask; and every load of the `threads` mod is a fresh copy
of the library's statics, so a mask of one CPU is taken for a pinned
thread's and the topology alone places the pool
([lore](../lore/a-pinned-thread-sees-one-cpu-and-its-new-threads-inherit-it.md)).
Measured: a second engine's pool in one process ran 3D's passes in 107
ms a step against 4.3 before this.

The thread count is the placement's unless set. A game changes the
defaults three ways:

- `ENGINE_THREADS=<n>`, `ENGINE_PIN=0`, `ENGINE_WARM_US=<µs>` in its
  environment, read at load;
- `modctl send threads threads 4` (or `warm`, `pin on|off`, `status`)
  between frames, each a new pool;
- its own resident mod over `//engine/std/threads:pool` with its own
  `Placement`, named in `engine_game(threads = ...)`.

**Tests aren't pinned**: `.bazelrc` sets `ENGINE_PIN=0` for every test, since
tests run side by side, many engines to a test process, and every pool
pinned to one CCD's cores would pile all of them there.

## Warmth

A worker that finishes a dispatch spins for the next one for `warm`
(default 1 ms), then returns to rayon, which yields a few dozen times and
parks it. It is a job of its own (`spawn_broadcast` after each dispatch,
watching a generation the next dispatch bumps), so rayon's code stays
untouched and the spin ends the moment the next dispatch starts.

The dispatch spike measured what cold costs (+79% on the passes after 16
ms idle). Within a step the gaps between dispatches are short but not
nothing: 2D's `prepare` ran on one thread for about 470 µs between the
narrowphase's dispatch and the passes', 3D's for about 2 ms (since
get-znt.40 filled the batches in the passes, about 120 and 290). Measured on
step_bench at 8 threads, the workers pinned, µs a step, the passes and
the whole step (2D: the median of 5 runs; 3D: of 3):

| warm | 2D pile settled | 2D pyramid | 2D pile falling | 3D boxes settled |
|---|---|---|---|---|
| 0 (rayon's own spin, then parked) | 1379 / 3684 | 833 / 2263 | 122 / 840 | 4463 / 10 457 |
| 100 µs | 1302 / 3515 | 487 / 1868 | 123 / 818 | – |
| **1 ms (the default)** | 761 / 2894 | 455 / 1796 | 119 / 811 | 2822 / 8666 |
| 5 ms | 738 / 2866 | 454 / 1786 | 118 / 815 | – |

- **Cold workers cost the passes 1.8 times** on the settled pile and 1.6
  in 3D: after `prepare`'s half a millisecond (2 ms in 3D) on one thread,
  the workers have parked, and come back late and slow, the spike's cold
  figure in miniature (that their cores clocked down is inferred from
  the spike's lore, not measured here).
- **A millisecond covers it**: 5 ms gains 3% more on the settled pile and
  nothing on the pyramid; the pile not turning, whose dispatches are the
  broadphase's and narrowphase's alone, is level at every setting. The
  cost is the spinning: up to 1 ms of seven cores after a step's last
  dispatch, of a frame's 16.

## Dispatch

`engine_ecs::dispatch` runs a `Plan`, stages of blocks each waiting for a
count of other stages, as one run of the executor:

- **Publishing.** The stages that wait for nothing are published first;
  the thread whose completion fills a stage lowers its successors' counts
  and publishes those that reach zero, on a ready list.
- **Claiming.** In a published stage a thread takes blocks from its own
  share (`first_block`, Box2D's `GetWorkerStartIndex`) forward until one is
  taken, then back, by raising each block's mark to one past the stage
  (`fetch_max`): a thread still in an older stage can't, and a stage's
  blocks are claimed once. Stages over the same blocks share marks, so a
  thread finds its blocks in its cache pass after pass.
- **Completion** is counted once a thread a stage, what it ran.
- **Waiting** is spinning, a yield every 1024 turns, until every stage is
  done.
- **No main thread.** The calling thread is worker 0, and every worker
  runs the same loop: one that comes late skips what is done, one that
  never comes holds up nobody, which is all `Executor::run` promises.

`Passes::run` turns a program into a plan: an `Items` stage is a stage a
color (the overflow one block, its items sharing states; a color's items
in blocks of at least 4, at most four a thread, Box2D's sizes), an `All`
stage one stage of every color's blocks together, handed out worker by
worker, each its share of every color (`all_order`, so a worker fills the
batches it then solves: 3% off the passes at 8 threads against item
order), an `Each` stage a stage of ranges of at least 32 states. Each
block's items sit
behind a lock only its taker takes (`try_lock`), which is how a kernel
gets `&mut` items in safe Rust, and which fails loudly if the protocol
ever handed a block out twice. The states are shared as `T::Shared` for
the run and written back after.

**Every kernel has returned when `run` does.** The executor's `run` returns
once every task has (rayon's scope waits for its spawns), and every task
is the protocol's loop, which leaves only when every stage is done. So no
mod code is on a worker's stack once the system has returned, which is
what lets the loader swap the mod at the next pump.

**One thread, plain.** Where the world has no executor, or one thread,
`run` runs the stages in order on the system's thread with the states
plain (stage 1's path, 8 to 18% faster than shared, dispatch-spike.md).

**The guard, get-znt.39.** `passes.serial(true)` runs that call on one
thread, plain. physics2d's `prepare` decides it each step
(`staged::shareable`): a still body carrying a negative zero is written
back by every batch of a color that touches it, the same value whoever
writes last but for `-0.0 - -0.0`, which is `0.0`, so across threads the
result could differ from one thread's. physics3d's `prepare` already
sends such a step to its sweep (`in_order`), on one thread.

### Panics

A kernel's panic is caught on the thread it happened on, ends the other
workers' waits, and is raised again on the calling thread when the
dispatch returns, where the system's mod catches it as it would any.
Every split is a dispatch, so the same holds for all of them. It never
unwinds through the
executor: the pool's code is another library's, with another copy of std,
and std's `catch_unwind` aborts the process on a panic from another copy
(read in std's source, `panic_unwind/src/gcc.rs`: an exception whose
canary isn't this copy's is a foreign exception, and
`__rust_foreign_exception` aborts; not provoked here).

### `ParMap` and `Reduce`

**`ParMap::for_each_mut` runs across threads** (get-znt.45, 2026-10-03):
a plan of one stage, the items in blocks of at least `min`, at most four
a thread (`blocks_of`, as `Passes` cuts a color), each block's items
behind a lock only its taker takes, on the same dispatch; one block, or
one thread, runs on the system's thread in order. Its first user is both
physics mods' write-back, whose items are parts of the mod's own making,
each a run of several arrays (physics.md, "The write-back across
threads"). An empty map's dispatch costs about 3 µs at 8 threads (an
extra one in physics2d's `finish`, the settled pile: 31 µs against 28).

**`map_into` and `Reduce` run across threads** (get-znt.31,
2026-10-03), each a plan of one stage on the same dispatch. `map_into`
cuts the items into blocks as `for_each_mut` does, each block's results
into a list of its own made on the system's thread with room for them
(memory a worker allocates is its thread's:
[lore](../lore/memory-a-task-allocates-is-its-threads.md)), moved into
`out` in block order after; `Reduce` maps its fixed chunks in blocks of
consecutive chunks, at most four a thread, and folds the results on the
system's thread in chunk order. Neither has a user yet: physics2d's
broadphase and narrowphase, the splits they were meant for, fill lists
the system makes before the run, each chunk an item of `for_each_mut`
with its own lists, which keeps their allocations on the system's thread
and their chunking what it was.

### The ECS's own splits

Work the ECS does inside a declared node splits across the same pool
through `Split` (`par.rs`, crate-private): `near_pairs` (and so a
`Live`'s broadphase when it finds its pairs afresh), a query's parallel
walks (which take the system's `ParMap`), and the spatial re-sort's
re-bounding at an apply node. Each is one stage of blocks, an item a
block, each thread taking its own share first, on the same dispatch
(get-znt.31; until then each a run of the executor, tasks from a shared
counter). They declare nothing: the scheduler seeing them is get-znt.5.
There is one pool for everything, as parallel-relations.md's "(d) The
host pool" asked: two would oversubscribe the cores.

## Hot reload

The rules that make a swap safe hold with threads:

- **Code on a worker's stack.** Mod code (kernels, glue) runs on a worker
  only inside a dispatch, which returns before the system or apply node
  that made it; the pump comes after the frame. Rayon's code, always on
  the workers' stacks, is the resident `threads` mod's.
- **Thread-locals.** A mod's code on a kept thread must leave no TLS
  destructor there, or the thread keeps that build mapped until it exits
  ([lore](../lore/a-thread-local-a-mod-touches-keeps-its-build-mapped.md)).
  The dispatch uses none; `physics2d_test`'s
  `threads_that_ran_a_builds_tasks_do_not_keep_it_mapped` checks the
  builds mapped after a reload with the game's pool, a pool installed
  directly, threads spawned per run, and none.
- **Poison mode** makes a stale call fault at once:
  `reloading_physics_under_the_pool_is_reloading_it_on_one_thread` runs a
  turning pile on four threads, reloads physics, and runs on, against the
  same on one thread, bit for bit; any worker still in the old build would
  fault in its unmapped span.
- **The replays** reload pong's, the platformer's and pile3d's mods every
  frame, every few frames and in batches, physics among them, with the
  game's pool live, and must be the run without reloads bit for bit; they
  pass at 1, 2, 4 and 8 threads. How much of them runs on the pool is
  small (pong's and the platformer's broadphase splits; pile3d's replay is
  64 boxes): the reload test above is the one that puts a threaded solve
  across the swap on purpose.

## Determinism

Shapes are bit for bit at any thread count by construction (flows.md,
"Determinism"): a color's blocks are independent, and kernels see the same
values in any order. Proved here by:

- **Unit:** `Passes` on threads that come at once, late and one by one, at
  1 to 8, against one thread (`flow_test`); on the real pool at 1, 2, 4
  and 8, warm and cold (`pool_test`); `ParMap::for_each_mut` and
  `map_into` and `Reduce` the same ways, every item (or chunk) once at
  its own index, results in order and a fold in chunk order, the calling
  thread among the takers; the dispatch's every block once and each stage
  after its predecessor (`dispatch.rs`'s tests).
- **The mods:** `quality_test`'s
  `the_mod_across_threads_is_the_arrays_bit_for_bit` (2D's mod on the pool
  at 1 to 16 threads and one by one, against the arrays' one-thread solve);
  `physics2d_test`'s piles at four threads; and every test that loads a
  game runs on its pool (the `threads` mod, as many threads as the
  placement gives, unpinned), or, in the physics harnesses, on the
  process's shared one.
- **The baselines and the fingerprint**, at `ENGINE_THREADS` 1, 2, 4 and 8:
  2D's and 3D's baselines, default and long (`baseline -- --all` and
  `-- --long --all`), byte-identical to the 4daabfc tree's output, every
  value (232, 229, 104 and 98 lines); physics3d's fingerprint (`exact`) as
  pinned. The default and long test suites pass, and the exact,
  equivalence and replay tests at each of the four counts
  (`--test_env=ENGINE_THREADS=<n>`).
- **The games:** pong's and the platformer's replays pass unchanged with
  the pool on, at every count. Little of their frames runs on it: pong's
  600 frames made 1200 dispatches (the broadphase's split; nothing turns,
  so no passes). pile3d's pile of 1000 boxes made one a step, its passes.

## Measured

`step_bench`, 2D (`//engine/std/physics2d/compare:step_bench`) and 3D
(`//engine/std/physics3d:step_bench`), `--config=bench`, the new tree
and the 4daabfc tree built side by side and run alternately, two rounds
each (both rounds below, or their range), the median of 5 runs (3D: 3),
each a fresh engine stepped to the window and timed over 30 steps. The
new runs without `taskset`: the pool pins itself. The old runs as it was
documented, under `taskset -c 0-7`, its `THREADS=8` a kept pool of its own
for the broadphase's and narrowphase's splits alone (its passes on one
thread). Other agents' builds
shared the machine (load averages 2 to 7 between runs); every number
below repeated within 6% across rounds but 3D's settled boxes at 8
threads, whose passes read 2822 and 3509.

**2D**, µs a step: the solve, its passes, the whole step.

| case | old, 1 | old, 8 | new, 1 | new, 8 | new 8 against new 1 |
|---|---|---|---|---|---|
| pile 10 000 turning, settled | 5100 / 3770 / 6711 | 5050-5148 / 3701-3777 / 6029-6116 | 5088-5091 / 3758-3760 / 6712-6715 | 2190 / 789 / 2922-3081 | solve 2.3×, passes 4.8×, step 2.2-2.3× |
| pyramid 5050 turning | 3143-3192 / 2351-2384 / 4281-4328 | 3179-3224 / 2364-2390 / 3785-3875 | 3180-3183 / 2383-2387 / 4320-4327 | 1318-1323 / 454-455 / 1788-1884 | solve 2.4×, passes 5.2×, step 2.3-2.4× |
| pile 10 000 turning, falling | 560-563 / 293-294 / 1066-1080 | 586-588 / 293-295 / 962-963 | 560-564 / 282-284 / 1084-1088 | 430 / 119 / 815-818 | solve 1.3×, passes 2.4×, step 1.3× |
| pile 10 000 not turning, settled | 2659-2709 / – / 3243-3310 | 2693-2750 / – / 3142-3188 | 2737 / – / 3337-3341 | 2762 / – / 3230-3236 | step 1.03× |

- **One thread is the old engine**: the same solve, passes and step
  within 1% (the plain path, unchanged), but for the pile not turning, 3%
  slower (2737 against 2659-2709 µs of solve; not chased).
- **Eight threads more than halve a turning step**, where the old
  engine's 8 gained only what those splits gave (10 to 12%): the passes run
  4.8 to 5.3 times as fast as one thread's, the spike's 5.5 (its affine
  passes 651 µs settled, 426 on the pyramid; here 789 and 455, the step's
  own cache traffic around them). The rest is serial: `prepare` (568 µs
  settled, unchanged from one thread), the gathers and write-backs.
- **Nothing turning gains nothing**: that step is solved one contact at a
  time in pair order, and its splits were already the old
  engine's (3230 against 3142-3188, the new pool's dispatch a little
  dearer than the bench's old one for these small splits).

**3D**, µs a step: the solve, its passes, the whole step (the old
tree's step_bench didn't time the passes apart).

| case | old (one thread) | new, 1 | new, 8 | new 8 against new 1 |
|---|---|---|---|---|
| boxes 10 000 settled | 17 976-18 443 / – / 20 770-21 389 | 18 362-18 734 / 15 325-15 604 / 21 216-21 688 | 5938-6611 / 2822-3509 / 8686-9346 | solve 2.8-3.2×, passes 4.4-5.5×, step 2.3-2.5× |
| boxes 10 000 falling | 17 563-18 018 / – / 23 044-23 854 | 18 310-18 435 / 15 176-15 350 / 24 045-24 289 | 7414-7461 / 4322-4332 / 12 872-13 234 | solve 2.5×, passes 3.5×, step 1.9× |
| planks 1000 settled | 2524-2617 / – / 2951-3051 | 2573-2597 / 2209-2227 / 3001-3032 | 970-983 / 556-569 / 1403-1410 | solve 2.6-2.7×, passes 3.9-4.0×, step 2.1× |

- **The colored passes gain 4.4 to 5.5 times** on 10 000 boxes settled,
  where the layout's ideal was 7.9 (physics.md, "Colouring the 3D solve")
  and the spike measured 4.9 to 5.2; the solve about 3 times, as the spike
  found: `prepare`, 2.3 ms on one thread whatever the count, is 40% of it
  at 8.
- **One thread is the old engine** within 2% on the solve (the timings now
  split it into `prepare`, `passes` and `finish`).

**The frame's thread.** The same 2D bench at 8 threads, before the pool
pinned it, with it left to the OS against everything under `taskset -c
0-7` (the workers pinned either way): the step took 1410-1426 against
813-823 µs falling, 5406-5430 against 2892-2929 settled, 3486-3512
against 1775-1798 on the pyramid, 3960-4013 against 3229-3236 not
turning, `prepare` on one thread included (1072-1080 against 564-566 µs
settled). Where the OS put it wasn't sampled; another CCD is the likely
reading. Pinned by the pool (`pin_caller`), the step is level with
`taskset` (2899-2927 against 2901-2913 settled, 1787-1797 against
1797-1805 on the pyramid).

### Every split on a shape (get-znt.31)

`step_bench`, 2D and 3D, `--config=bench`, the df23813 tree and this one
(their step_benches and `sequential` alike, the breakdown below added to
both) alternated over two rounds, the pool pinning itself, nothing else
building (load averages 2.8 to 9.5, falling, other agents' Bazel servers
idle). µs a step, both rounds; 2D the median of 5 runs, 3D of 3.

| case | old, 1 | new, 1 | old, 8 | new, 8 |
|---|---|---|---|---|
| 2D pile 10 000 turning, settled | 6786-6853 | 6831-6841 | 2428-2462 | 2370-2385 |
| 2D pyramid 5050 turning | 4383-4421 | 4379-4397 | 1486-1487 | 1446-1462 |
| 2D pile 10 000 turning, falling | 1081-1116 | 1107-1113 | 801-804 | 789-798 |
| 2D pile 10 000 not turning, settled | 3364-3367 | 3362-3377 | 3254-3271 | 3233-3257 |
| 3D boxes 10 000, settled | 21 421-21 599 | 21 519-21 948 | 6827-7731 | 6833-6837 |
| 3D boxes 10 000, falling | 23 728 | 23 199-24 309 | 11 285-11 401 | 9382-9448 |
| 3D planks 1000, settled | 3045-3072 | 3059-3074 | 1152-1165 | 1156 |

- **2D at 8 threads, 1 to 3% faster**, from the narrowphase (settled
  275-282 µs → 250-260, the pyramid 240-241 → 216-230) and the merge
  (33-36 → 30): the same chunks, each thread now taking its own share
  first (dispatch-spike.md) where `Executor::run` handed tasks out from a
  shared counter. The pile not turning, get-znt.44's case, gains 0.5%:
  the small dispatches cost no more than before.
- **3D's falling boxes, 17% faster at 8**: `Live`'s broadphase, found
  afresh while the pile falls, now splits across the threads as 2D's did
  (1164-1166 µs → 355-358), and the passes after it take 3740-3819
  against 4880-4919, the workers kept warm by the broadphase's dispatch
  where 4.6 ms of serial `find_contacts` had let them park (inferred from
  "Warmth", not measured apart). Settled, the pairs are kept, and nothing
  moves.
- **Bit for bit**, by the chunking kept: at `ENGINE_THREADS` 1, 2, 4 and
  8 both baselines, default and long (`baseline -- --all`, `-- --long
  --all`), byte-identical to df23813's (232, 229, 104 and 98 lines); the
  3D fingerprint as pinned; the exact and equivalence tests and pong's
  and the platformer's replays pass at each count.
- **One thread is level** within the runs' spread (2D's falling pile
  1081-1116 against 1107-1113, 3D's boxes 21 421-21 599 against
  21 519-21 948).
- **Unexplained, kept as measured:** 2D's `gather_contacts` 291-297 µs →
  267-270 on the settled pile at both counts, and `scatter_bodies`
  139-146 → 153-155 on one thread; neither's code changed.

### The whole step, stage by stage

The same runs, the new tree: each stage µs a step at 1 / 8 threads (its
share of the step at 8), the mean of the two rounds' medians, from the
mod's own timers (`stages`) inside `find_contacts` and the scheduler's
(`sequential`'s `time on`) for each node, apply nodes included. A
median of each stage, so the stages needn't add to the step exactly.

**2D**

| stage | pile settled | pyramid | pile falling | pile not turning | |
|---|---|---|---|---|---|
| gravity (`integrate_velocities`) | 27 / 23 (1%) | 14 / 16 (1%) | 25 / 21 (3%) | 24 / 23 (1%) | split (`ParMap` walk), no gain |
| colliders gathered | 87 / 92 (4%) | 39 / 49 (3%) | 85 / 86 (11%) | 55 / 64 (2%) | split (`ParMap` walks), no gain |
| broadphase: `Live` kept pairs | 185 / 182 (8%) | 75 / 77 (5%) | 155 / 106 (13%) | 114 / 114 (4%) | serial (afresh: split) |
| broadphase: pairs to slots | 68 / 19 (1%) | 39 / 11 (1%) | 3 / 3 (0%) | 44 / 17 (1%) | scales: `ParMap` |
| narrowphase | 942 / 255 (11%) | 824 / 223 (15%) | 39 / 18 (2%) | 208 / 131 (4%) | scales: `ParMap` |
| merge with the world | 133 / 30 (1%) | 83 / 22 (2%) | 9 / 11 (1%) | 62 / 28 (1%) | scales: a `ParMap` walk |
| rest of `find_contacts` (overlaps, waking) | 13 / 11 (0%) | 6 / 8 (1%) | 7 / 8 (1%) | 6 / 8 (0%) | serial |
| apply(find_contacts): contacts spawned, despawned | 47 / 46 (2%) | 0 / 0 (0%) | 43 / 43 (5%) | 0 / 0 (0%) | serial |
| gather_bodies † | 53 / 34 (2%) | 24 / 23 (2%) | 48 / 35 (5%) | 47 / 35 (1%) | scales: a `ParMap` walk, carved |
| gather_turning † | 98 / 37 (2%) | 39 / 23 (2%) | 73 / 34 (5%) | 1 / 1 (0%) | scales: a `ParMap` page walk, joined |
| gather_contacts † | 268 / 75 (4%) | 203 / 64 (5%) | 12 / 12 (2%) | 79 / 36 (1%) | scales: a `ParMap` page walk, copied across threads |
| prepare (coloring, seating) | 121 / 122 (5%) | 73 / 77 (5%) | 36 / 42 (5%) | 0 / 0 (0%) | serial |
| passes (fill and solve) | 4200 / 818 (34%) | 2702 / 507 (35%) | 297 / 126 (16%) | 0 / 0 (0%) | scales: `Passes` |
| finish (write-back; not turning: the whole solve) | 134 / 29 (1%) | 80 / 22 (1%) | 15 / 10 (1%) | 2509 / 2523 (78%) | scales: `ParMap` (not turning: serial) |
| scatter_contacts † | 154 / 35 (2%) | 92 / 26 (2%) | 12 / 12 (2%) | 80 / 43 (1%) | scales: a `ParMap` page walk writing rows |
| scatter_bodies † | 143 / 51 (3%) | 57 / 33 (3%) | 90 / 44 (6%) | 51 / 25 (1%) | scales: `ParMap` walks writing rows (sleeping: serial) |
| apply(scatter_bodies): the re-sort | 146 / 33 (1%) | 50 / 23 (2%) | 153 / 66 (8%) | 81 / 60 (2%) | re-bounding split, moves serial |
| everything else (other nodes, outside them) | 11 / 11 (0%) | 9 / 11 (1%) | 12 / 12 (2%) | 5 / 6 (0%) | serial |
| **whole step** | 6836 / 2377 (100%) | 4388 / 1454 (100%) | 1110 / 793 (100%) | 3370 / 3245 (100%) |  |

**3D**, the get-emj.101 tree, measured the same way in its own runs
(2026-10-03; physics.md, "The 3D narrowphase across threads", has them
against the tree before):

| stage | boxes settled | boxes falling | planks settled | |
|---|---|---|---|---|
| gravity (`integrate_velocities`) | 50 / 40 (1%) | 46 / 40 (1%) | 3 / 2 (0%) | serial |
| colliders gathered | 378 / 152 (3%) | 355 / 151 (3%) | 47 / 25 (3%) | scales: `ParMap` walks (statics, `Slots` serial) |
| broadphase (`Live`) | 311 / 296 (6%) | 1157 / 344 (6%) | 41 / 32 (4%) | kept: serial; afresh: split |
| narrowphase | 1867 / 257 (6%) | 2899 / 546 (10%) | 330 / 54 (6%) | scales: `ParMap` |
| merge with the world | 158 / 54 (1%) | 182 / 66 (1%) | 18 / 11 (1%) | scales: a `ParMap` walk |
| rest of `find_contacts` | 25 / 8 (0%) | 5 / 3 (0%) | 4 / 4 (0%) | serial |
| apply(find_contacts): contacts spawned, despawned | 18 / 18 (0%) | 704 / 675 (12%) | 0 / 0 (0%) | serial |
| gather_bodies | 116 / 87 (2%) | 125 / 96 (2%) | 7 / 8 (1%) | serial |
| gather_contacts | 311 / 300 (7%) | 288 / 300 (5%) | 46 / 51 (6%) | serial |
| prepare (coloring, seating) | 269 / 271 (6%) | 279 / 290 (5%) | 33 / 31 (4%) | serial |
| passes (fill and solve) | 17745 / 2801 (61%) | 16699 / 2686 (49%) | 2541 / 605 (69%) | scales: `Passes` |
| finish (write-back) | 128 / 27 (1%) | 116 / 27 (0%) | 21 / 7 (1%) | scales: `ParMap` |
| scatter_contacts | 55 / 54 (1%) | 50 / 51 (1%) | 11 / 6 (1%) | serial |
| scatter_bodies | 172 / 151 (3%) | 163 / 154 (3%) | 10 / 10 (1%) | serial |
| apply(scatter_bodies): the re-sort | 238 / 70 (2%) | 260 / 95 (2%) | 20 / 20 (2%) | re-bounding split, moves serial |
| **whole step** | 21865 / 4603 (100%) | 23354 / 5530 (100%) | 3136 / 870 (100%) |  |

- **2D's serial stages over 5% at 8 threads**: `Live`'s kept pairs
  (5-13%), `prepare` (5%); while falling also the contacts spawned at
  `find_contacts`' apply node (5%) and the re-sort's moves (8%). The
  solve's gathers and write-backs (†, get-emj.103-105) were 11-13% (the
  contacts gathered, turning), 6-7% and 4-12% (the contacts and the
  bodies written back), and while falling 6% and 9% (the bodies' and
  their turning's gathers); split, each is 1-6% of the step (below). The colliders' gathers and gravity split but don't gain:
  memory-bound walks whose chunks are joined by a copy on the system's
  thread.
- **3D's serial stages over 5% at 8 threads**, its `find_contacts` split
  (get-emj.101: its narrowphase had been a quarter to a third of the
  step, on one thread): the contacts gathered for the solve (5-7%),
  `prepare` (4-6%), `Live`'s kept pairs (4-6%), and while falling the
  contacts spawned at `find_contacts`' apply node (12%).
- **† The solve's gathers and write-backs across threads** (get-emj.103-105,
  2026-10-03; physics.md, "The 2D solve's gathers and write-backs across
  threads"): those rows re-measured on that tree, the same bench against
  70a4093's alternated over two rounds (load 2.5-3), each cell the mean
  of the rounds' medians and its share of the new step. The whole step at
  8 threads: settled pile 1891-1908 µs against 2331-2348 (-19%), pyramid
  1202-1211 against 1437-1451 (-17%), falling 678-695 against 785-800
  (-13%), not turning 2999-3000 against 3198 (-6%); at one thread level
  but for 0.2-0.6% (1102-1107 against 1093-1110 falling, 6718-6731
  against 6691-6731 settled, 4377-4399 against 4356-4359 on the pyramid,
  3336-3339 against 3321-3324 not turning). The bodies' gather gains
  where the colliders' don't because it's carved, not joined: every row
  is a body, so each chunk writes its own part of the lists.
- **The pile not turning is its solve** (78%), one contact at a time in
  pair order, which no shape splits without moving results.

**A busy core.** One process spinning on CPU 3, one of the pool's: the
pyramid's passes 455-459 → 536-543 µs pinned (+18%), the step 1774-1778
→ 1858-1903; left to the OS the pool is slower with or without it (965 to
1026 µs of passes). Two runs each, single runs of 30 steps. A pinned worker sharing its core is what a stage waits for; the
OS can't move it. On a machine running other work, `ENGINE_PIN=0` (or a
`Placement` avoiding the busy cores) is the escape.

### The fill in the passes (get-znt.40)

The same benches (`--config=bench`, the median of 5 runs in 2D and 3 in
3D), the ed0b68d tree and the get-znt.40 one alternated, two rounds each
(their range; 2026-10-03; load averages 1 to 7, but 17 at the start of
2D's first round). What moved, and why the rest didn't: physics.md, "The
fill as the passes' first stage".

**2D**, µs a step: the solver (`prepare`, `passes`, `finish`), and the
whole step.

| case | before, 1 | after, 1 | before, 8 | after, 8 |
|---|---|---|---|---|
| pile 10 000 turning, settled | 4399-4478 / 6775-6932 | 4419-4435 / 6774-6797 | 1436-1491 (559-582, 769-799, 108-111) / 2897-3031 | 1118-1121 (122-123, 816-820, 177-180) / 2554-2562 |
| pyramid 5050 turning | 2828-2835 / 4359-4397 | 2850-2860 / 4395-4405 | 947-951 (410-423, 464-470, 65-66) / 1794-1820 | 728-729 (75-76, 512, 141) / 1594-1597 |
| pile 10 000 turning, falling | 345-346 / 1076-1095 | 342-344 / 1105-1122 | 191-193 / 812-813 | 182-183 / 814-816 |
| pile 10 000 not turning, settled | 2494-2496 / 3348-3356 | 2491-2496 / 3352-3363 | 2509-2510 / 3239-3242 | 2504-2512 / 3250-3253 |

**3D**, µs a step: the solver (`prepare`, `passes`, `finish`), and the
whole step.

| case | before, 1 | after, 1 | before, 8 | after, 8 |
|---|---|---|---|---|
| boxes 10 000 settled | 17 756-17 994 / 21 329-21 683 | 17 900-18 138 / 21 443-21 693 | 6059-6792 (2362-2372, 3559-4290, 130-131) / 9437-10 126 | 3647-3661 (272-276, 3222-3231, 157-159) / 6987-7001 |
| boxes 10 000 falling | 16 882-17 362 / 23 051-23 640 | 17 701-17 826 / 24 164-24 307 | 6788-6832 / 12 859-12 928 | 5154-5279 / 11 270-11 420 |
| planks 1000 settled | 2550-2557 / 3067-3073 | 2563-2596 / 3071-3108 | 906-909 / 1405-1408 | 664-668 / 1161-1166 |
| stack 20 | 21-22 / 32-33 | 22 / 33 | 51 / 63 | 50-51 / 62-63 |

- **At 8 threads the turning solvers take a fifth to two fifths less**:
  2D's settled pile 22-25%, the pyramid 23%; 3D's settled boxes 40-46%,
  falling 22-25%, planks 26-27%. The serial `prepare` is 122 µs in 2D's
  settled pile and about 275 in 3D's boxes; the passes grew by the fill,
  20-50 µs in 2D and 370-520 in 3D's falling boxes (settled, the base's
  own passes ranged 3559 to 4290). Against one thread the solver is now
  4.0 times as fast at 8 in 2D's settled pile (3.1 before), and 4.9 in
  3D's settled boxes (2.6-3.0).
- **`finish` grew at 8 threads** (2D's settled 108 → 178 µs, the pyramid
  65 → 141, 3D's boxes 131 → 158): the contacts, points and lanes it
  writes and reads were last touched by the fill and the passes on other
  cores. It was the largest serial part left, until get-znt.45 shared it
  out (below).
- **One thread.** 2D level within the rounds' noise but for its settled
  pile, +0.7% of the solve over five more alternated rounds (4483 µs
  against 4450, the step +0.25%); 3D's settled boxes +0.8%, its falling
  boxes +1.4 to 1.7% pooled over every alternated run (17 702-17 779 µs
  mean against 17 408-17 531), a case whose base itself ranged 16 882 to
  17 944.

### The write-back across threads (get-znt.45)

The same benches, the 92f67c6 tree and the get-znt.45 one alternated,
two rounds each (their range; 2026-10-03; load averages 2 to 5 but 18 at
the start of 2D's first round). Why it's shaped as it is: physics.md,
"The write-back across threads".

**2D**, µs a step: the solver (`prepare`, `passes`, `finish`), and the
whole step.

| case | before, 1 | after, 1 | before, 8 | after, 8 |
|---|---|---|---|---|
| pile 10 000 turning, settled | 4407-4459 (121, 4160-4213, 126) / 6739-6792 | 4458-4471 (122-123, 4201-4213, 134) / 6811-6880 | 1117-1120 (123, 812-816, 178-180) / 2550-2559 | 964-984 (124-125, 812-833, 28-29) / 2426-2468 |
| pyramid 5050 turning | 2836-2890 (73-74, 2680-2733, 82) / 4371-4424 | 2848-2851 (74-75, 2694-2697, 79) / 4366-4370 | 725-727 (74-75, 510-511, 139-141) / 1592-1605 | 611-620 (77-78, 512-522, 21) / 1484-1485 |
| pile 10 000 turning, falling | 344-348 (finish 14-15) / 1138-1145 | 343-351 (14-15) / 1090-1106 | 183 (36, 123-124, 24) / 807-810 | 175-181 (43, 124-128, 9) / 794-819 |
| pile 10 000 not turning, settled | 2494-2498 / 3354-3366 | 2496-2498 / 3357-3360 | 2508-2525 / 3242-3263 | 2511-2516 / 3250-3255 |

**3D**, µs a step: the solver (`prepare`, `passes`, `finish`), and the
whole step.

| case | before, 1 | after, 1 | before, 8 | after, 8 |
|---|---|---|---|---|
| boxes 10 000 settled | 18 122-18 450 (finish 143-152) / 21 853-22 182 | 17 855-17 950 (134-135) / 21 521-21 691 | 3630-4386 (277-283, 3176-3953, 159-161) / 7002-7786 | 3455-3456 (271, 3157-3165, 27-28) / 6804-6824 |
| boxes 10 000 falling | 17 520-17 629 (138-141) / 23 914-23 986 | 16 878-16 948 (120) / 23 061-23 145 | 5239-5301 (155-156) / 11 358-11 383 | 4941-5318 (35-40) / 11 067-11 576 |
| planks 1000 settled | 2597-2626 (27-29) / 3112-3145 | 2557-2589 (20-21) / 3063-3106 | 671-675 (24-25) / 1170-1174 | 650-652 (7) / 1150-1164 |

- **`finish` at 8 threads takes a fifth to a sixth of what it did**: 2D's
  settled pile 178-180 → 28-29 µs, the pyramid 139-141 → 21, 3D's
  settled boxes 159-161 → 27-28, planks 24-25 → 7. The solver at 8 is
  12-14% faster in 2D's settled pile and 15% on the pyramid; the step
  3-5% and 7%. 3D's settled boxes gain the 130 µs (their passes vary
  more than that from run to run, 3157-3953 in the base).
- **One thread.** 2D's `finish` is 8 µs slower on the settled pile (134
  against 126; 134-138 against 125-131 in one build switching between
  the two at run time, so about 0.2% of the solver), level on the pyramid: the contacts are
  written in order now and the batches read wherever a contact's lane
  is, where it was the other way round. The settled pile's solver ranged
  +0.0 to +1.4% over the two rounds and the step +1.0 to +1.3%, the base
  itself ranging 1.2% between rounds. 3D's `finish` is faster on one
  thread (settled boxes 134-135 against 143-152, falling 120 against
  138-141): the base found each contact's seat by walking the coloring
  (`Coloring::seats`), where it is now read (`seat_of`).
- **Gathering looked dearer and isn't.** The new tree's 2D gather read
  about 25 µs more than the base's at both thread counts in the first
  runs (435 against 410 settled); one build switching between the old
  write-back and the new at run time gathered alike either way (411-412
  at 8 threads, 404-427 at 1), so it was the build's layout, not the
  contacts' lines being on other cores.

## What waits

- **System parallelism (get-znt.5).** `Plan` already takes several chains,
  each a program, side by side: the scheduler would collect a frame's (or
  a phase's) shapes into one plan and dispatch it once, so one program's
  thin stages fill with another's blocks (the spike measured 5 to 18% from
  that). What it needs that isn't here: running systems themselves on the
  pool, with the loader's bookkeeping thread-safe (scheduling.md, step 2),
  and a shape's `run` that hands its plan to the frame's graph rather than
  dispatching it.
- **Block-level pipelining (get-znt.23).** A block's dependencies are its
  stage's today: a stage waits for whole stages. Pipelining is waits per
  block (block `k` of the next stage after block `k` of this one, for
  row-local work), the same publish-on-completion with a count a block,
  inside the same dispatch.
- **Block sizes by work (get-znt.41)**: the plan's blocks are Box2D's
  sizes; a stage's blocks could be cut by points, or more of them.

(History, 2026-10-03: filling the batches in the first stage waited here
too, until get-znt.40 built it, and the write-back across threads, until
get-znt.45 did; "Measured".)

## Testing, and the mutants

The tiers (CLAUDE.md), each test where the bug it's for shows first:

- **Unit:** `engine_ecs`'s `dispatch.rs` (every block once, each stage
  after the one it waits for, a panic raised on the caller and the rest
  stopped, on threads at once, late and one by one, 1 to 8); `flow_test`
  (`Passes` on test executors against one thread, its items filled by an
  `All` stage from their seats, `serial`, a kernel's panic;
  `ParMap::for_each_mut`'s every item once at its index, the calling
  thread among its takers, a call's panic);
  `//engine/std/threads:pool_test` (every task once and `run`
  returning after the last, `Passes` on the real pool at 1, 2, 4 and 8
  warm and cold, `ParMap` on it the same, workers pinned where the placement says, a second pool
  made from a pinned thread placed as the first, the threads ended on
  drop, sysfs topology parsed).
- **Integration:** `physics2d_test`'s `threads` module: a reload of
  physics under four of the game's pool's threads, bit for bit the reload
  on one, in poison mode; the builds mapped after a reload with the
  game's pool, a pool of the test's, spawned threads and none; the
  get-znt.39 guard (a kinematic body at `-0.0` holds the step's passes on
  one thread, at `0.0` they go across, and the pile is the one thread's);
  piles at four threads against one. `quality_test`'s
  `the_mod_across_threads_is_the_arrays_bit_for_bit` (1 to 16 threads and
  one by one against the arrays' one-thread solve).
- **End to end:** nothing new but the manifest's first line: `e2e_test`
  checks a game reload's reply, which now names the thread host.

Every test that loads a game runs on its pool (as many threads as the
placement gives, unpinned), so the default suite is threaded throughout.

**Mutants**, each planted by hand, run, and reverted (2026-10-03):

| mutant | fails |
|---|---|
| a block run twice (stage 1, block 0) | `ecs_test`, `flow_test`, `pool_test`, `quality_test`, `physics2d_test`, `exact_test` |
| a stage released one block early | `ecs_test` (once its blocks took 5 µs each: with empty blocks the overlap was too short to see, and only the physics tests failed), `pool_test`, `quality_test`, `physics2d_test`, `exact_test` |
| the calling thread returning before the workers finish (`Pool::run` spawning its tasks `'static`, by an unsafe transmute in the mutant only, and not waiting) | `pool_test`, `quality_test`, `physics2d_test`, `exact_test`, each a segfault or a task found not run |
| the get-znt.39 guard removed (`serial(false)`) | `physics2d_test`'s `a_still_body_with_a_negative_zero_keeps_the_passes_on_one_thread` |
| placement read from the calling thread again | `pool_test`'s `a_pool_made_from_a_pinned_thread_is_placed_as_the_first` |

The third can't be written in safe Rust: rayon's scope borrows the tasks
until they return, which is the guarantee. The guard's own race, which the
mutant re-opens, needs particular impulses to show and wasn't seen; the
test holds the guard, not the race.

**The fill as a stage** (get-znt.40, 2026-10-03), its mutants the same
way, each run against `flow_test`, `pool_test` and both physics mods'
tests:

| mutant | fails |
|---|---|
| an `All` stage skips its first block | `flow_test`; `physics2d_test`, 2D's `quality_test` and `behaviour_test`; `exact_test`, `physics3d_test`, 3D's `reload_test`, `quality_test` and `smoke_test` |
| an `All` stage fills its first block twice | `flow_test` alone: both mods' fills write their batches whole, so a batch filled twice is the batch filled once, which no physics test can see, and needn't |
| the stage after the fill not waiting for it (`plan.chain()` after it) | every physics test above but `flow_test`, at first: its fill was over before a thread reached the next stage. Once its blocks took 20 µs, 10 runs in 10 |
| 2D's fill of a block skipping each batch's last lane | `physics2d_test`, `quality_test`, `behaviour_test` |
| 2D's fill of every batch (one thread) skipping the same | `physics2d_test`, `quality_test`, `behaviour_test` |
| 2D's fill reading the first step's seats, not this step's | `physics2d_test`, `quality_test`, `behaviour_test` |
| 2D's fill of every batch keeping no record of each batch's last lane (`LaneCell`) | `physics2d_test`, `quality_test`, `behaviour_test` |
| 3D's fill of a block skipping each batch's last lane | `exact_test`, `physics3d_test`, `quality_test`, `smoke_test` |
| 3D's fill of every batch (one thread) skipping the same | `exact_test` (the kernel's fingerprint), `smoke_test` |
| 3D's fill reading the first step's seats | `exact_test`, `physics3d_test`, `reload_test`, `quality_test`, `behaviour_test`, `smoke_test` |

`pool_test` failed none: it runs no `All` stage. The 2D rows were run
again once the lanes became `LaneCell`s, with the same results.

**The write-back across threads** (get-znt.45, 2026-10-03), each run
against `flow_test`, `pool_test` and both physics mods' tests:

| mutant | fails |
|---|---|
| a map's block run twice (block 0) | `flow_test`, `pool_test` alone: both mods' write-backs write each value whole, so a part written twice is the part written once, which no physics test can see, and needn't |
| a map's block skipped (block 0) | `flow_test`, `pool_test`; `physics2d_test`, 2D's `quality_test`; `exact_test`, `physics3d_test`, 3D's `quality_test` |
| 2D's write-back skipping each part's first contact | `physics2d_test`, `quality_test`, `behaviour_test` |
| 2D's write-back clearing a part's points again after writing them | `physics2d_test`, `quality_test`, `behaviour_test` |
| 2D's fill by batch writing no contact's seat (each read the step before's) | `physics2d_test`, `quality_test`, `behaviour_test` |
| 3D's write-back skipping each part's first contact | `exact_test`, `physics3d_test`, `quality_test`, `smoke_test` |
| the write-back reading the batches before the passes' last stage (2D: restitution left out of the program; 3D: the last substep's sums) | 2D's `quality_test`, `behaviour_test`; `exact_test`, `physics3d_test`, 3D's `quality_test` |
| a dispatch's threads leaving once all but its last stage are done (and its "every stage done" check gone) | `flow_test`, `pool_test`, every physics test above but 3D's `reload_test`, and 3D's `behaviour_test` |

The write-back can't run before the passes are done other than by such
a mutant: `finish` takes the graph the `passes` system passes on (the
flows order them), and a dispatch returns only once every stage is
complete and its threads have left. Left out of the 3D program,
restitution failed no default test (the mod's bounces are the long
suite's; the arrays' solve, which `exact_test` checks restitution on,
has its own program), so the 3D row drops the last sums instead. Why
(get-znt.46, 2026-10-04): a lone ball's one contact is too sparse for
the lanes (`Tuning::sparse_alone`), so every bounce scene, the long
suite's included, was solved one contact at a time, with restitution of
its own; the long suites pass with the stage gone. `physics3d_test`'s
`balls_rebound_to_about_e_squared_through_the_lanes` drops sixteen
balls side by side, enough to fill the batches, and is the one default
test that fails with the stage dropped from the mod's program alone
(their apexes 0.2397 of the drop, 0.0 without it). Dropped from
`Staged::program` itself, `exact_test` fails as well. Long
checks for it: the 2D and 3D long suites; not the reload fuzzer or Miri,
since neither the reload sequence nor the unsafe core changed.

**Long checks run for the pool** (get-znt.34): the reload fuzzer (runbook 003; the loader's reload
sequence is unchanged, and its mods declare no shapes, so it checks that
reloads still hold with the engine changed under them, not threads), and
the loader's sanitizer suite (runbook 004, `//engine/tests:asan`, whose
trigger includes the host threads; `physics2d_test_asan` runs the threads
module but its mapping test). Not Miri: the unsafe core is unchanged, and
the pool's only unsafe code is rayon's and core_affinity's own.
