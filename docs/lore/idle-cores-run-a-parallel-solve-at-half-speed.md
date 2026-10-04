# Idle cores run a parallel solve at half speed

On this machine (Ryzen 9 7950X, `acpi-cpufreq` with the `schedutil`
governor) a core that has been idle runs at 3.0 GHz, not the 5+ it boosts
to, until it has been busy for tens of milliseconds. `schedutil` picks a
core's clock from its recent load (the scheduler's decaying average), so a
worker thread that wakes up for one short job does the job at the idle
clock. The main thread, busy all along, doesn't: so a parallel run looks
like the extra threads barely help.

Measured in `:parallel_solver`[^spike-code] (2026-09-24), the colored solve of a 10 000
pile, 41 runs back to back, each after a 2 ms spin on all its threads:

| threads | first 16 to 20 runs, µs | the rest, µs |
|---|---|---|
| 2 | about 1030 (the 1-thread time) | 543 |
| 4 | 582 | 320 |
| 8 | 350 | 205 |

The step is sudden and comes after about 40 to 55 ms of load on the
worker cores (the runs are 2.5 ms each). A 2 ms spin before each run
isn't enough; 300 ms before a batch of runs is. The first bench runs, round
robin over thread counts, spent most of their time cold and made 4 threads
look slower than 2.

## What it means

- **A benchmark of threads must warm them for longer than the governor's
  window**, not just wake them: `:parallel_solver`[^spike-code] kept each variant's
  threads busy 300 ms before timing it, and ran one variant's runs back to
  back rather than round robin.
- **A game's workers will be cold every frame** (inferred, not measured
  in a frame loop): physics on 8 threads for about 0.2 ms of a 16.7 ms
  frame keeps each worker's load near 1%. Unless the pool keeps them busy
  between jobs (spinning, which costs the cores it spins on) or the
  machine runs the `performance` governor, parallel physics would get
  about half the speedup the bench shows. Worth measuring when the
  scheduler's workers exist (get-znt.5). Measured since against a
  frame's idle, not yet in a frame loop (the dispatch spike, 2026-10-03,
  [dispatch-spike.md](../architecture/dispatch-spike.md)): with 16 ms of
  sleep before each solve, the 2D settled pile's passes at 8 kept threads
  took 1163 µs against 650 warm (+79%), pinned or not, and on one thread
  5591 against 3586, since the calling thread slept too. The workers
  exist now: the resident `threads` mod keeps them spinning for `warm`
  (1 ms by default) after each dispatch, and
  [threads.md, "Warmth"](../architecture/threads.md#warmth) has what that
  bought within a step. A whole frame loop's idle is still unmeasured.

Check the governor with
`cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor`, and a core's
clock with `scaling_cur_freq` next to it.

[^spike-code]: (2026-10-04) `//engine/std/physics2d:parallel_solver`
    deleted by get-emj.112; last built at 4018662.
