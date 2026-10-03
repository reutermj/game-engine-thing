# Dispatching a program's stages: spike results

**Status: spike results** (2026-10-03, get-znt.30), **built** the same day
as [threads.md](threads.md): the `affine` protocol in `engine_ecs`
(`dispatch.rs`), on a rayon pool pinned to one CCD. Nothing in the engine
changed for the spike itself: no `engine_ecs` code, no physics code, no
baseline. The spike was four targets over the mods' own
kernels:[^spike-code]

- `//engine/std/physics2d/compare:dispatch_spike` (`dispatch.rs`,
  `dispatch_2d.rs`, `dispatch_spike.rs`): five dispatchers, each running
  physics2d's `Passes` program across threads, against
  `lanes::run_across` (`solver::solve_across`) and one thread;
- `//engine/std/physics3d:dispatch_spike3d` (`tests/dispatch_3d.rs`,
  `tests/dispatch_spike3d.rs`): the same dispatchers over physics3d's
  colored program;
- `:dispatch_test` and `:dispatch_test3d`, in the default suite: every
  dispatcher is the one-thread solve bit for bit, and the check sees a
  planted bug.

This is a report for the scheduler's design (get-znt.29) and flows stage 3
(get-znt.34), where both physics solves get threads in the running engine.

**In short:**

- **Yes: a task graph costs what `run_across`'s stage counter costs, if
  it claims blocks the way `run_across` does.** The graph (each stage a
  count of the stages it waits for, published by whoever finishes the
  last block before it, no main thread) with `run_across`'s claim inside
  a stage (`affine`) runs the 2D passes within 1 to 2% of the counter at
  8 threads (651 µs against 642 on the settled pile of 10 000), and 3D's
  within −1 to +4%. Its dependency counts and ready list cost about 0.02
  µs a stage over the counter's 0.36 (the dispatchers alone, no work).
- **What costs is losing a thread's blocks, and sleeping.** The same graph
  taking blocks from one shared counter (`graph`, as Jolt's `fetch_add`)
  is 17 to 27% slower on 2D's passes: its blocks themselves run 26 to 32%
  longer, landing on a different core each stage. A work-stealing pool's
  shape (`steal`, block by block from deques) is 1.7 to 2.5 times slower
  on 2D's passes at 8 threads (1.1 to 2.2 on 3D's); parking idle threads
  between stages (`park`) 1.5 to 1.7 (1.1 to 1.6). A stage's handoff,
  from its last block to the next stage's first, is 0.15 to 0.24 µs
  (median) spinning, 0.33 to 0.44 parked, 1.2 to 1.6 from deques.
- **Generality pays where the stages are thin.** Two steps' passes as one
  dispatch of two programs (`affine`) took 5 to 18% less than the same
  two concatenated in `run_across`'s fixed order (`counter`), the falling
  pile's small stages most. Idle thread time at 8 threads is 16% (3D
  boxes) to 39% (2D falling pile) of the passes: half to three quarters
  of it in stage tails, the rest handoffs and claims. The thin last
  colors are a small part of the tails (0.8 to 5.6% of the wall time);
  the rest is the imbalance of fat stages' last blocks.
- **The gap to `run_across` is the serial fill, not dispatch.** The whole
  2D solve with any of the fast protocols is 430 to 500 µs slower than
  `solve_across` at 8 threads (1304 µs against 873 on the settled pile),
  with the passes themselves no slower: the pipeline fills the batches in
  `prepare`, on one thread (about 470 µs), where `run_across` fills them
  in its first stage. 3D's `prepare` is about 2.3 ms of a 5.4 ms solve at
  8 threads.
- **The pool must keep threads on one CCD and warm; pinning each to a
  core and keeping them alive matter less.** On the settled pile at 8
  threads (passes, `counter`): kept on one CCD 650 µs; each pinned to its
  own core 654 (no gain over the CCD mask); spawned for each run 700
  (+8%); placed by the OS over both CCDs 1244 (+91%), and pinned to one
  CCD by the pool itself 648; cold, 16 ms idle before each solve as a
  frame would leave them, 1163 (+79%).
- **Recommendation:** stage 3's dispatch is a task graph of (stage,
  block) tasks with dependency counts, published by whoever completes a
  stage, claimed from each thread's own share by `fetch_max` marks,
  completions counted once a thread a stage, idle threads spinning while
  the frame's graph has work, the calling thread a worker. The pool keeps
  its threads, confines them to one CCD (an affinity mask or a pin each),
  and is kept warm or accepts the cold-frame cost; rayon would serve only
  as a thread host (its scope starting the workers, as Rapier uses it),
  never as the per-block scheduler.

## The question

In stage 3, `Passes::run` hands a program's (stage, block) tasks to the
scheduler's threads ([flows.md](flows.md#declared-and-run-by-the-scheduler)).
`lanes::run_across` does this today for one solve
([physics.md](physics.md#solving-across-threads)): a fixed list of stages,
every thread walking all of them, blocks claimed by raising a mark
(`fetch_max`), a count per stage, spinning with yields between stages, no
main thread. A scheduler that owns all parallelism (get-znt.28) needs
something more general:

- tasks with dependencies: a stage's blocks after the previous stage's;
- programs from more than one system in one graph (later, with system
  parallelism, get-znt.5);
- the calling thread joining in.

What does that generality cost a stage? The 2D solve has 136 to 157
stages a step on the settled scenes, about 4 to 5 µs each at 8 threads;
3D's colored solve 286 over batches (11 colors × 26 passes) and 10 over
bodies at 10 000 boxes, about 10 µs each.

## How others dispatch staged work

Read in the fetched source (`$(./bazel info output_base)/external/`:
`+http_archive+box2d`, `+http_archive+box3d`, `+http_archive+jolt`,
`rules_rs++crate+crates__rapier{2,3}d-0.36.0`). None of them states a
cost per stage in its source or comments.

| engine | who builds the task list | claim within a stage | wait between stages | main thread | cost a stage, stated |
|---|---|---|---|---|---|
| Box2D v3 (`solver.c`) | no list: `enqueueTask(b2SolverTask)` once per worker, the user's task system owns the threads | CAS on each block's `syncIndex`, from the worker's own start (`GetWorkerStartIndex`) forward then back | worker 0 spins (`b2Pause`) on the stage's `completionCount`; the others spin on `atomicSyncBits`, two pauses a turn, `sched_yield` after 5 | worker 0 publishes every stage and runs blocks too | none ("can waste significant time overall, but it is necessary", `solver.c:1113`) |
| Box3D (`solver.c`, `scheduler.c`) | as Box2D; its own optional scheduler is a flat task array, claimed by CAS, idle threads on a semaphore | as Box2D (`b3SyncBlock`; the rationale, per-block CAS against one contended counter and start offsets for cache affinity, in `solver.h:4-47`, "inspired by bepuphysics2") | as Box2D | the orchestrator's slot is raced for (`mainClaimed`) by the queued worker-0 task and the calling thread, which also runs `b3SolverTask` | none |
| Jolt (`JobSystemThreadPool`, `LargeIslandSplitter`) | `PhysicsSystem::Update` builds a job DAG (`CreateJob(..., numDependencies)`, `RemoveDependency`); `max_concurrency` long-lived solve jobs, not a job a stage | jobs: a 1024-slot ring, CAS on push, `exchange` on take; inside an island: `fetch_add(cBatchSize = 16)` on a packed iteration/split/item word | idle pool threads block on a semaphore; a solve job with nothing to take polls with `yield()`; the barrier between colour splits is the thread whose `fetch_add` completes a split storing the next into the status word, not a job dependency | the caller runs `WaitForJobs`, executing ready jobs, then sleeps on the barrier's semaphore | none |
| Rapier 0.36 (`staged_island_solver`) | `rayon::in_place_scope` spawns workers 1..N, worker 0 inline; a fresh `StageSync` each step; compiled here without `parallel`, so one thread | CAS on per-worker padded cursors packed as (stage, position): its own slice first, then the others' | spin on `published` (`spin_loop`, `yield_now` every 10 000); the thread whose completion fills the stage publishes the next; a late worker sees a newer stage and skips ahead | none: worker 0 is a worker | none ("tiny batches measurably throttle high worker counts") |

What they agree on, and the spike's starting point:

- **One long-lived task a worker, not a task a stage.** All four start
  their workers once per solve and run every stage inside them. Even
  Jolt, the one with a real job graph, gives the solve `max_concurrency`
  jobs that each pull work until the island is solved.
- **Stages are atomics, and waits are spins.** No engine wakes a thread
  between stages: each spins, yielding now and then. Sleeping is for
  between solves (Jolt's semaphore, Box3D's scheduler).
- **Claims keep a thread on its own blocks.** Box2D, Box3D and Rapier
  claim from the worker's own share first, which keeps a block's data in
  the cache that last touched it; Jolt's single `fetch_add` doesn't, and
  splits islands in a way the others don't.
- **What's not in the source:** any figure for what a stage costs. The
  spike measures it.
- Not read: Bepu (named by Box3D as the source), Unity DOTS's job
  scheduler, and Rapier with `parallel` enabled at run time (its
  `StageSync` was read, not run).

## What was built

`dispatch.rs` runs one or more programs, each a chain of stages of blocks
with a function to run block `b` of stage `t`, by one of five protocols.
`Prepared` lays a `Passes` program out as `Passes::run` defines it (the
overflow one block, each color's batches cut as `run_across` cuts them, 4
batches at least and at most 4 blocks a thread, states 32 at least), so
every protocol runs the same blocks; only the handing out differs.

| protocol | what it is | claim within a stage | completion | wait | several programs |
|---|---|---|---|---|---|
| `counter` | `run_across`'s, generalised to any program | `fetch_max` on the block's mark, from the thread's own share forward then back | one `fetch_add` a thread a stage | spin on the stage's count, `yield` every 1024 | concatenated: one fixed sequence |
| `graph` | a task graph: each stage a count of stages it waits for; whoever completes a stage's last block lowers its successors' counts and publishes those at zero on a ready list | `fetch_add` on the stage's next block | one `fetch_add` a thread a stage | spin over the ready list, `yield` every 1024 | yes: any ready stage of any program |
| `affine` | the graph, with `counter`'s claim | as `counter` | as `graph` | as `graph` | yes |
| `steal` | a work-stealing pool's shape (rayon's, Jolt's ring): a ready stage is one range task on its publisher's deque, split in halves as taken; idle threads steal the oldest half of another's | a deque pop, a block at a time | `fetch_add` a block | spin over the deques | yes |
| `park` | `graph`, idle threads parked on a condvar and woken at each publish | as `graph` | as `graph` | a futex wait | yes |

All of them run on the executor's threads as one run (one wake-up), with
the calling thread as worker 0, no main thread, and a thread that comes
late or never holding nobody up (the test runs each on threads that come
one at a time, the last first). `steal`'s deques are `Mutex<VecDeque>`,
not Chase-Lev deques: an upper bound on what a lock-free one costs, which
needs `unsafe` or crossbeam.

**Unsafe code.** The dispatchers have none: atomics, a `Mutex` a block
taken with `try_lock` (as `run_across`'s), and the states as relaxed
atomics (`Shareable`). The spike's only unsafe is pinning
(`sched_setaffinity`, one FFI call, for `PIN=1`) and what the kept pool
already had (`tests/pool.rs`, a closure's lifetime erased). A real version
needs exactly those two, which are get-znt.20's decision, and nothing for
the protocol itself. Giving up the block's `Mutex` for disjoint slices
behind an `UnsafeCell` would save one uncontended lock a block, about 4400
a step on the settled pile (an estimate, not measured: a few µs a thread);
not worth new unsafe code.

**Bit for bit.** Every protocol at 1, 2, 4 and 8 kept threads, on
spawned threads (`Scoped`) and on threads that come one at a time (the
variants' `Backwards`), gives the one-thread solve's every value: 2D on a
settled pile of 1000, a pyramid of 210 and a falling pile
(`:dispatch_test`), 3D on boxes and planks (`:dispatch_test3d`); and the
benches check every timed configuration on the scenes they time. The
one-thread reference is `Passes::run`'s plain path (a copy of
`engine_ecs`'s private `stages`), which is the mod's solve, itself
`solve_with` bit for bit in 2D and `solver::solve` in 3D (checked in both
benches). Mutation-checked: a planted block run twice fails the check in
every protocol at one thread and four, in 2D and 3D, and a stage released
one block early (`Plant::Early`) fails it at four threads (as a different
result, or the block's lock caught taken twice); both checked by
the tests themselves, every run. Releasing every stage early by hand in
`graph` failed both the solve check and the dispatchers' own check (every
block once, none before its stage's predecessor is done).

**get-znt.39.** A 2D step with a still body carrying a negative zero is
solved on one thread, plain, by every protocol (`lanes::shareable`'s
condition, copied, since it is private to `solver.rs`); the test sets one
and checks it. 3D's `Staged::prepare` checks its own and sends such a step
to `in_order`. Run unguarded at 8 threads, the settled pile with its still
body's velocity set to `-0.0` gave the one-thread result in 20 of 20
tries: the hazard needs a still body written by batches of one color with
impulses of both signs, which this scene doesn't hit. The guard stays: the
argument for it is by construction, not by a failure seen.

## Measurements

All with `--config=bench` on the Ryzen 9 7950X, `taskset -c 0-7` (one
CCD; CPUs 16 to 31 are 0 to 15's SMT siblings), kept threads
(`tests/pool.rs`) warmed for 300 ms just before each thread count's
measurements, the median of 9 runs on each of three captured steps, mean
over the steps. The methods are interleaved run by run, so a drift in the
clocks reaches them all alike. Every timed configuration was checked bit
for bit against the one-thread solve; all were.

    taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:dispatch_spike
    taskset -c 0-7 ./bazel run --config=bench //engine/std/physics3d:dispatch_spike3d

The scenes: 2D, step_bench's, every body turning: the pile of 10 000
settled (steps 401, 430, 460; 22 012 contacts in 7 colors, the last of 4
batches), the pyramid of 5050 (steps 601, 630, 660; 14 950 in 6 even
colors), the pile falling (steps 20, 25, 30; 842 to 2914 contacts in 3
colors). 3D, pile3d's, colored (the default): 10 000 boxes settled (steps
900 to 930; 18 739 contacts, 11 colors of 1243 batches down to 1),
1000 planks settled (steps 600 to 630; 2992 contacts, 12 colors), 10 000
boxes falling (steps 61 to 90). 3D's inputs are gathered from the world
between steps (`dispatch_3d.rs`), since 3D has no arrays to capture them
from.

### The dispatchers alone

150 stages of 4 blocks a thread, each block nothing or 1 µs of spinning;
µs a stage over the ideal (the run's time less 150 × 4 × the block's work,
divided by 150), the median of 21 runs:

| threads | work a block | counter | graph | affine | steal | park |
|---|---|---|---|---|---|---|
| 1 | 0 | 0.03 | 0.03 | 0.03 | 0.06 | 0.25 |
| 1 | 1 µs | 0.15 | 0.14 | 0.14 | 0.17 | 0.34 |
| 2 | 0 | 0.12 | 0.20 | 0.20 | 0.62 | 0.18 |
| 2 | 1 µs | 0.21 | 0.30 | 0.28 | 0.52 | 1.72 |
| 4 | 0 | 0.20 | 0.36 | 0.26 | 1.60 | 0.26 |
| 4 | 1 µs | 0.31 | 0.38 | 0.36 | 1.40 | 2.15 |
| 8 | 0 | 0.36 | 0.64 | 0.38 | 4.23 | 0.50 |
| 8 | 1 µs | 0.41 | 0.48 | 0.44 | 3.28 | 2.56 |

- **The counter and the affine graph cost the same**: 0.36 to 0.44 µs a
  stage at 8 threads, the 0.19 to 0.24 µs barrier physics.md measured on
  one CCD plus the claims. The ready list and dependency counts are within
  noise of the counter's fixed sequence.
- **One shared claim counter costs where blocks are tiny** (0.64 at no
  work), as eight threads take turns on one line; with work in the blocks
  it costs little here, and the solve shows what it costs instead (below).
- **Parking costs a wake-up a stage** once a thread has time to park:
  2.6 µs a stage at 8 threads with 1 µs blocks, where spinning costs 0.4.
- **Locked deques cost 3 to 4 µs a stage at 8 threads**: each block a
  lock, a pop, and splits pushed back. A lock-free deque would cost less;
  the solve's numbers say how much it would have to save.

### 2D: the solve against `run_across`

µs; the whole solve as the pipeline runs it (`prepare`, the passes,
`finish`), the passes alone in brackets. `solve_across` is the hand-tuned
solve, its batch fill in its first stage. One thread, the pipeline's solve
plain (as `Passes::run` runs it on one thread): settled 4062 (passes
3510), pyramid 2680 (2275), falling 504 (432).

Settled pile of 10 000:

| threads | `solve_across` | counter | graph | affine | steal | park |
|---|---|---|---|---|---|---|
| 1 | 4158 | 4363 (3799) | 4359 (3802) | 4366 (3811) | 4364 (3811) | 4416 (3863) |
| 2 | 2430 | 2664 (2060) | 2783 (2159) | 2694 (2065) | 2777 (2157) | 2993 (2369) |
| 4 | 1403 | 1777 (1133) | 1906 (1246) | 1832 (1139) | 1999 (1340) | 2158 (1469) |
| 8 | 873 | 1304 (642) | 1489 (818) | 1372 (651) | 1773 (1102) | 1741 (1022) |

Pyramid of 5050:

| threads | `solve_across` | counter | graph | affine | steal | park |
|---|---|---|---|---|---|---|
| 1 | 2772 | 2900 (2493) | 2900 (2494) | 2902 (2496) | 2913 (2505) | 2947 (2541) |
| 2 | 1612 | 1810 (1375) | 1855 (1410) | 1830 (1374) | 1861 (1418) | 2040 (1584) |
| 4 | 912 | 1208 (738) | 1304 (825) | 1237 (742) | 1412 (935) | 1530 (1036) |
| 8 | 574 | 898 (417) | 1016 (519) | 940 (426) | 1348 (850) | 1226 (716) |

Pile of 10 000, falling:

| threads | `solve_across` | counter | graph | affine | steal | park |
|---|---|---|---|---|---|---|
| 1 | 510 | 601 (526) | 596 (522) | 595 (521) | 599 (526) | 622 (548) |
| 2 | 368 | 407 (321) | 439 (354) | 413 (324) | 443 (358) | 549 (460) |
| 4 | 260 | 315 (225) | 337 (248) | 318 (226) | 416 (328) | 442 (351) |
| 8 | 215 | 271 (177) | 299 (207) | 274 (178) | 540 (449) | 359 (265) |

- **The passes at 8 threads are 5.5 times one thread's plain passes** on
  the settled pile and the pyramid (642 and 417 µs, `counter`), 2.4 on the
  falling pile, with the affine graph within 2% of them throughout.
- **`affine`'s whole solve read 5% over `counter`'s here** (1372 against
  1304); a run with the order of the methods changed put them level (1337
  against 1344, prepare 570 and 583), so it is the methods' order, not
  the protocol: the passes are what differ, and they don't.
- **One thread on shared states costs 8 to 10%** (3799 against 3510;
  3D 10 to 18%): flows.md's
  "On one thread" cost, which stage 1 avoids by handing kernels their
  states plain; a dispatcher at one thread should too.
- **Every protocol's solve is slower than `solve_across`** by the fill
  in `prepare` (above), at every count above one: the passes themselves
  at 8 threads (642) are well under `solve_across`'s whole (873).

### 3D: the colored solve against one thread

µs, the solve (the passes, and their speedup over one thread's plain
passes). One thread, plain: boxes settled 16 899 (passes 14 727), planks
2431 (2147), boxes falling 15 665 (13 621).

10 000 boxes, settled:

| threads | counter | graph | affine | steal | park |
|---|---|---|---|---|---|
| 1 | 18 489 (16 294, 0.90×) | 18 470 (16 285, 0.90×) | 18 484 (16 298, 0.90×) | 18 496 (16 309, 0.90×) | 18 583 (16 397, 0.90×) |
| 2 | 11 486 (9212, 1.60×) | 11 240 (8998, 1.64×) | 10 683 (8448, 1.74×) | 10 876 (8664, 1.70×) | 11 646 (9433, 1.56×) |
| 4 | 7270 (4931, 2.99×) | 7303 (4995, 2.95×) | 7041 (4721, 3.12×) | 7272 (5043, 2.92×) | 7613 (5363, 2.75×) |
| 8 | 5407 (2980, 4.94×) | 5313 (2928, 5.03×) | 5448 (3003, 4.90×) | 5703 (3353, 4.39×) | 5643 (3260, 4.52×) |

1000 planks, settled:

| threads | counter | graph | affine | steal | park |
|---|---|---|---|---|---|
| 1 | 2822 (2533, 0.85×) | 2827 (2537, 0.85×) | 2834 (2544, 0.84×) | 2845 (2556, 0.84×) | 2918 (2628, 0.82×) |
| 2 | 1797 (1469, 1.46×) | 1928 (1611, 1.33×) | 1814 (1485, 1.45×) | 1929 (1612, 1.33×) | 2214 (1883, 1.14×) |
| 4 | 1190 (853, 2.52×) | 1309 (979, 2.19×) | 1223 (886, 2.42×) | 1499 (1166, 1.84×) | 1647 (1309, 1.64×) |
| 8 | 921 (578, 3.72×) | 1013 (673, 3.19×) | 946 (602, 3.57×) | 1585 (1245, 1.72×) | 1275 (930, 2.31×) |

10 000 boxes, falling:

| threads | counter | graph | affine | steal | park |
|---|---|---|---|---|---|
| 1 | 17 291 (15 219, 0.90×) | 17 272 (15 216, 0.90×) | 17 232 (15 177, 0.90×) | 17 243 (15 188, 0.90×) | 17 345 (15 286, 0.89×) |
| 2 | 10 763 (8614, 1.58×) | 10 385 (8272, 1.65×) | 9982 (7876, 1.73×) | 10 132 (8042, 1.69×) | 10 853 (8765, 1.55×) |
| 4 | 6815 (4601, 2.96×) | 6902 (4715, 2.89×) | 6728 (4516, 3.02×) | 6774 (4638, 2.94×) | 7079 (4942, 2.76×) |
| 8 | 4956 (2664, 5.11×) | 5039 (2786, 4.89×) | 4944 (2644, 5.15×) | 5362 (3108, 4.38×) | 5342 (3090, 4.41×) |

- **3D's colored passes gain 4.9 to 5.2 times at 8 threads** on 10 000
  boxes with the counter and both graphs, where physics.md's estimate
  from the layout was 7.9 (the blocks on the critical path alone); planks
  3.2 to 3.7. Ten µs a stage leaves dispatch little to cost: on the boxes
  all five protocols are within 15%, and the shared claim counter is no
  worse than the rest.
- **The solve gains only 3.1 to 3.2 times on the boxes**: `prepare`
  (grouping, packing, filling the batches) is 2.1 ms on one thread and
  serial, about 40% of the solve at 8 threads. Get-znt.34's fill in the
  passes would be worth more in 3D than in 2D.

### The cost a stage, over the ideal

The passes at 8 threads less one thread's shared passes divided by 8, a
stage: dispatch, waits and imbalance together, µs (and against
`counter`):

| scene | stages | counter | graph | affine | steal | park |
|---|---|---|---|---|---|---|
| 2D pile settled | 157 | 1.06 | 2.19 (+27%) | 1.12 (+1%) | 3.99 (+72%) | 3.48 (+59%) |
| 2D pyramid | 136 | 0.77 | 1.52 (+24%) | 0.84 (+2%) | 3.96 (+104%) | 2.97 (+72%) |
| 2D pile falling | 73 | 1.52 | 1.93 (+17%) | 1.54 (+1%) | 5.25 (+154%) | 2.73 (+50%) |
| 3D boxes settled | 296 | 3.19 | 3.01 (−2%) | 3.26 (+1%) | 4.45 (+13%) | 4.13 (+9%) |
| 3D planks | 322 | 0.81 | 1.11 (+16%) | 0.89 (+4%) | 2.88 (+115%) | 1.90 (+61%) |
| 3D boxes falling | 244 | 3.12 | 3.62 (+5%) | 3.04 (−1%) | 4.94 (+17%) | 4.87 (+16%) |

A general scheduler's stage, as `affine` hands it out, costs what
`run_across`'s does: about 1 µs a stage on 2D's scenes at 8 threads, of
which the protocol's own share is the 0.4 the dispatchers alone cost and
the rest waits for a stage's slowest block.

### Where the threads' time goes

One traced run of each step's passes (clocks around each block), µs of
thread time a solve at 8 threads, mean over the steps: the wall time ×
8 is busy time plus idle; idle is each stage's tail (from a thread's last
block in it to the stage's last block's end; in brackets the part in
stages of fewer blocks than threads, the thin colors), the handoffs
(from a stage's last block to the next stage's first, every thread; in
brackets the median handoff, µs of wall time), and the rest (claims,
completions, the run's start and end).

| scene | protocol | passes wall | busy | idle | tails (thin) | handoffs (median) | rest |
|---|---|---|---|---|---|---|---|
| 2D pile settled | counter | 632 | 4184 | 875 (17%) | 421 (42) | 214 (0.16) | 240 |
| | graph | 803 | 5505 | 920 (14%) | 398 (41) | 296 (0.22) | 225 |
| | affine | 660 | 4332 | 945 (18%) | 433 (42) | 285 (0.23) | 227 |
| | steal | 1109 | 4963 | 3907 (44%) | 768 (41) | 2101 (1.55) | 1038 |
| | park | 1025 | 5299 | 2901 (35%) | 615 (42) | 807 (0.37) | 1479 |
| 2D pyramid | counter | 423 | 2737 | 644 (19%) | 213 (0) | 192 (0.18) | 238 |
| | affine | 435 | 2760 | 717 (21%) | 252 (0) | 262 (0.24) | 203 |
| 2D pile falling | counter | 183 | 895 | 568 (39%) | 295 (15) | 134 (0.16) | 139 |
| | affine | 187 | 878 | 617 (41%) | 310 (15) | 187 (0.22) | 121 |
| 3D boxes settled | counter | 2654 | 17 827 | 3403 (16%) | 2529 (369) | 483 (0.16) | 391 |
| | graph | 2704 | 19 014 | 2620 (12%) | 1696 (363) | 526 (0.20) | 398 |
| | affine | 2600 | 18 076 | 2721 (13%) | 1843 (382) | 559 (0.23) | 319 |
| | steal | 3394 | 19 247 | 7901 (29%) | 2712 (507) | 3493 (1.34) | 1696 |
| | park | 3305 | 19 994 | 6446 (24%) | 2491 (535) | 2023 (0.41) | 1932 |
| 3D planks | counter | 599 | 3194 | 1601 (33%) | 899 (270) | 407 (0.15) | 295 |
| | affine | 621 | 3206 | 1759 (35%) | 876 (273) | 644 (0.23) | 239 |
| 3D boxes falling | counter | 2518 | 17 510 | 2632 (13%) | 1902 (160) | 337 (0.17) | 393 |
| | affine | 2394 | 16 283 | 2865 (15%) | 2101 (156) | 454 (0.22) | 311 |

(The bench prints 2 and 4 threads too, and every protocol for every
scene.)

- **Busy time is where the shared counter loses**: the same blocks take
  26 to 32% longer under `graph` than under `counter` in 2D (5505 against
  4184 µs on the settled pile), 6 to 19% in 3D. Claimed from one counter,
  a block lands on whichever thread is free, a different core from the
  stage before, and brings its batches and bodies with it
  ([lore](../lore/blocks-claimed-from-one-counter-run-a-quarter-slower.md)).
  Its idle time is close to the counter's.
- **The thin colors' tails are small**: 42 µs of thread time on the
  settled pile (5 µs of wall time at 8 threads, 0.8%), 156 to 382 µs in
  3D (20 to 48 µs of wall: 0.8% on the falling boxes, 1.7% settled, 5.6%
  on the planks). That bounds what another system's tasks could fill in
  them once get-znt.5 exists. Tails as a whole are larger: 27 to 316 µs
  of wall time, 6 to 20% of the passes, mostly fat stages waiting for
  their slowest block (3D's batches carry 1 to 4 points, so blocks of
  equal batches differ).
- **Handoffs are 0.15 to 0.24 µs spinning** in the counter and both
  spinning graphs, so
  the 286 barriers of 3D's colored step cost about 50 to 70 µs of wall
  time at 8 threads, as physics.md estimated (0.05 ms); a futex wake makes
  them 0.33 to 0.44 µs at the median, and the parked threads' late starts
  ("rest") cost more than the handoffs themselves.

### Two programs in one dispatch

Two steps' passes (the first two captured steps of each scene, prepared
apart), µs: one dispatch after the other / both programs in one dispatch.
`counter` can only run the second program's stages after the first's;
the graphs run any ready stage of either.

| scene | threads | counter | graph | affine | steal | park |
|---|---|---|---|---|---|---|
| 2D pile settled | 4 | 2142 / 2113 | 2368 / 2228 | 2190 / 2055 | 2663 / 2189 | 2927 / 2613 |
| | 8 | 1229 / 1182 | 1658 / 1351 | 1252 / 1118 | 2243 / 1377 | 2046 / 1631 |
| 2D pyramid | 4 | 1400 / 1384 | 1628 / 1441 | 1448 / 1349 | 1836 / 1436 | 2067 / 1506 |
| | 8 | 814 / 786 | 1014 / 864 | 832 / 732 | 1703 / 945 | 1414 / 902 |
| 2D pile falling | 4 | 380 / 356 | 425 / 361 | 395 / 319 | 583 / 383 | 605 / 417 |
| | 8 | 336 / 284 | 377 / 287 | 328 / 234 | 819 / 422 | 471 / 321 |

- **Filling one program's waits with another's blocks is worth 5 to 18%**
  at 8 threads (`affine` together against `counter` together: 1118
  against 1182, 732 against 786, 234 against 284), most where stages are
  small. It is more than the thin colors' tails alone: the second
  program's blocks also fill the fat stages' tails and the handoffs.
  That is what a scheduler running several systems' shapes at once
  (get-znt.5) can expect from the protocol, and it needs the graph.
- **Two dispatches cost about one dispatch more** (`counter`, 47 µs on the
  settled pile): the second run's wake-up and its threads' climb back to
  full speed.

### What the pool must provide

The settled pile's passes at 8 and 4 threads (µs, `counter` and `affine`;
the whole solve in the bench's output), one process a configuration
(`PIN=1`, `POOL=scoped`, `COLD_MS=16`, `taskset` or not):

| pool | 4: counter | affine | 8: counter | affine | `solve_across` at 8 |
|---|---|---|---|---|---|
| kept, one CCD (`taskset -c 0-7`), warm | 1111 | 1121 | 650 | 660 | 886 |
| kept, each pinned to its own CPU of 0-7, warm | 1122 | 1111 | 654 | 657 | 881 |
| kept, placed by the OS over both CCDs (no taskset), warm | 1806 | 1859 | 1244 | 1321 | 1856 |
| kept, pinned to 0-7 by the pool, no taskset, warm | 1131 | 1138 | 648 | 653 | 883 |
| spawned for each run (`Scoped`), one CCD, warm | 1217 | 1315 | 700 | 708 | 1066 |
| spawned for each run, placed by the OS | 2120 | 2124 | 1572 | 1613 | 2211 |
| kept, one CCD, cold: 16 ms idle before each solve | 2033 | 2042 | 1163 | 1207 | 1712 |
| kept, pinned, cold | 2021 | 2039 | 1159 | 1184 | 1714 |

By the numbers, in order:

1. **One CCD: +91% without it.** Left to the OS, the threads spread over
   both CCDs and the passes take 1.9 times as long; a pool that pins its
   threads to one CCD recovers it fully (648 against 650). This is the
   one property that needs `sched_setaffinity` (FFI) or a crate
   (core_affinity), and it is the largest.
2. **Warm: +79% cold.** After 16 ms idle, as a frame's worker would be
   between solves, the cores run at the idle clock; the one-thread solve
   is 56% slower too (5591 against 3586 µs of passes), since the bench's
   calling thread sleeps as well. Pinning doesn't change it. Only load
   keeps a core's clock up (lore: idle cores run a parallel solve at half
   speed), so it is the frame's: other systems' work on the same
   threads, or spinning between frames, which costs power. Measured here
   for the first time against a frame's idle; the lore inferred it.
3. **Kept: +8% spawned** (+50 µs a dispatch at 8 threads, +180 µs for
   `solve_across`'s two runs). Spawning a thread a run is affordable for
   one dispatch a frame and not for several.
4. **Pinned each to its own core: nothing** over a mask of the CCD (654
   against 650). The CCD matters; which core within it doesn't.
5. **Spinning between stages, inside a dispatch: +11 to 38% without it**
   (`park` against `graph`). The pool has to let the dispatch spin; the
   pool's own idle policy between dispatches (spin 50 µs, then park) is
   physics.md's "doesn't matter".
6. **The calling thread joining in** costs nothing measurable: every
   protocol here runs it as worker 0.

For get-znt.20 (rayon plus core_affinity against alternatives): rayon
would give kept threads and a scope that runs a borrowed closure (the
unsafe step `tests/pool.rs` takes by hand), and `start_handler` with
core_affinity the CCD. Its work stealing must not be the dispatcher:
`steal`, its shape, is 1.7 to 2.5 times slower on 2D's passes even
though a lock-free deque would close some of that, because a block taken
by a thief runs on a cold cache. Started once per dispatch, as Rapier
starts its staged solver (`in_place_scope`, a spawn a worker), with the
protocol above inside, rayon would be a thread host. Its threads sleep
after a few rounds of spinning between jobs, which is the between-
dispatch policy the numbers say doesn't matter; warmth stays the frame's
problem whichever host it is.

## Recommendation

**The dispatch protocol (get-znt.29, stage 3).** Expand a frame's
declared shapes into a graph of stages, each a block count and a count of
stages it waits for (a `Passes` program's stages in a chain; several
systems' programs side by side once get-znt.5 lets them). Run it as one
dispatch: every worker, the calling thread included, loops over the
published stages; within a stage it takes blocks from its own share
forward and back by raising each block's mark (`fetch_max`, Box2D's
`syncIndex`), adds what it ran to the stage's count once, and the thread
whose add completes the stage lowers its successors' counts and publishes
those at zero. Idle workers spin, yielding every 1024 turns, until the
graph is done. No main thread, and a late thread skips what's done. This
is `affine`: within −1 to +4% of `run_across`'s protocol (`counter`) on
the passes, general, and
the only protocol measured that gains from a second program.

What it needs, beyond the spike:

- **Plain states on one thread**: at one worker the dispatcher should run
  stage 1's plain path (8 to 18% faster than shared).
- **The fill in the passes** (get-znt.34's open call): the remaining gap
  to `run_across` in 2D, and 40% of 3D's solve at 8 threads, is
  `prepare`'s serial batch fill. Filled in the first stage, as
  `run_across` does.
- **Block sizes.** Tails in fat stages are most of the idle time (6 to
  20% of the passes); four blocks a thread, Box2D's choice, leaves the
  slowest block's wait. Sizing blocks by their work (3D's points) or
  more blocks a thread is worth measuring before stage 3 fixes it.
- **No unsafe code in the protocol.** The kept pool's lifetime erasure
  and pinning are the only unsafe steps, both get-znt.20's.
- **get-znt.39** as built here: a step that isn't shareable runs on one
  thread, plain; the dispatcher needs the system's word for it (a flag on
  `Passes::run`), since only the kernels' owner knows the condition.

**The pool (get-znt.20).** Kept threads, confined to one CCD, kept busy
or accepted cold; it lets a dispatch spin, and it doesn't schedule
blocks. Rayon plus core_affinity meets that as a host; so does
`tests/pool.rs` with one `sched_setaffinity` call, at the price of its
unsafe lifetime erasure. Nothing measured here prefers one over the
other.

## Open questions

- **Warmth in a real frame.** The cold figure is a bench sleeping 16 ms;
  a game's frame keeps some threads busy with other systems. How warm a
  scheduler's workers are in a running frame needs the scheduler's
  workers (get-znt.5).
- **Several programs' order.** The graph runs any ready stage; which
  program's stage a thread prefers (the oldest, the one with the longest
  path left) wasn't varied. With two equal programs it didn't matter.
- **SMT.** All numbers are on 8 cores without their siblings; physics.md
  measured SMT at +5% for `run_across`.

[^spike-code]: *(History, 2026-10-03.)* The spike's code was removed once
    its findings were written here and the protocol landed in the engine
    (threads.md): spikes are built to answer a question and then thrown
    away. `dispatch.rs`, `dispatch_2d.rs`, `dispatch_spike.rs` and
    `dispatch_test.rs` (physics2d/compare), `tests/dispatch_3d.rs`,
    `tests/dispatch_spike3d.rs` and `tests/dispatch_test3d.rs` (physics3d),
    the kept pool they ran on (`physics2d/tests/pool.rs`) and the
    `solve_across` they were measured against all build and run at commit
    `4daabfc`, the last commit with the spike building.
