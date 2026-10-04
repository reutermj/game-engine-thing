# A stage loop without a main thread must let any thread take any block

Found 2026-09-29 building the colored solve across threads
(`solver::solve_across`, `lanes::run_across`), on Box2D v3.1.1's stage loop
(`b2SolverStage`, `b2ExecuteStage` in `solver.c`, read in its fetched
source): each worker starts at its share of a stage's blocks, takes blocks
forward, then backward, until one is taken already, adds what it ran to
the stage's count, and the stage is done when the count is its blocks.

Box2D has a main thread (worker 0) that runs every stage and publishes the
next; a worker whose index is past a stage's block count takes nothing
(`GetWorkerStartIndex` returns `B2_NULL_INDEX`) and waits on the main
thread. Ported without one, so that a solve depends on no thread in
particular (the host's executor promises that tasks run, not that they
run at once), it hung in `quality_test`'s
`the_colored_solve_across_threads_is_the_solve_on_one_bit_for_bit`. Two
holes, one of them the hang:

- **A thread past a small stage's blocks waited forever.** With an
  executor that runs the tasks one after another, last first
  (`variants.rs`, `Backwards`, 4 threads), task 3 came first, alone, to a
  stage of one block (the last color, or the overflow), had no start, and
  waited for a count nobody else would raise: the test timed out at 300 s
  with one thread spinning, and passes since the fix. Fix: a thread past
  the blocks starts at `w % n`, so every thread can take any block.
- **A thread peeking at a block made the stage skip it.** Blocks were
  first taken with `Mutex::try_lock` and a mark inside: a thread still in
  an older stage (late, on a busy machine) locked a block for a moment to
  read that its stage was done, and a thread of the current stage that
  found it held took that for "taken" and stopped scanning, so the block
  could be left for nobody. Found reading the protocol while chasing the
  hang, and fixed first; not seen to hang alone. Fix: take a block by raising an atomic mark
  to the stage's number plus one (`fetch_max`, taken if it was lower): a
  thread in an older stage can never raise it, so it never holds what the
  current stage needs. The lock is then only how the taker has the block's
  data, and never held by anyone else (`try_lock().expect(..)`, which a
  planted stage without its wait trips at once).

Rapier 0.36's staged solver (`staged_island_solver/sync.rs`) avoids both
the same way: stages advance on completed work, not on arrival, and a
claim carries its stage, so a straggler's fails.

*(History: 2026-10-04: `solver::solve_across`, `lanes::run_across`
and the quality test named above were removed on 2026-10-03 (ed0b68d,
get-emj.93). The protocol lives on in `engine/ecs/dispatch.rs`, which
runs `Passes` across the world's executor: `first_block` starts a
thread past a stage's blocks at `w % n`, blocks are claimed by
`fetch_max` on their marks, and a block's data is still taken with
`try_lock().expect(..)` (`engine/ecs/shape.rs`). Its unit test
`every_block_runs_once_after_the_stage_before` runs on the same
`Backwards` executor, among others. A stage there is published by
whichever thread completes the last stage it waits for
([threads.md, "Dispatch"](../architecture/threads.md#dispatch)).)*
