# Physics regression testing

**Status: proposed** (2026-09-28). How physics is kept from getting worse
as it changes, including the changes that are meant to improve it. Today's
tests are described as they stand. The proposal adds what they miss: a
record of our own accepted results, which every change is compared
against in both directions.

Physics is hard to regression-test for three reasons, all of which the
design has to answer:

- **Most changes are meant to change results.** A fix that makes a pile
  settle sooner is supposed to move the numbers. So "the same as before"
  can't be the test, or every improvement fails it.
- **Settling is chaotic.** Rounding alone moves the step a pile comes to
  rest by 100-200 steps from one pile size to the next, in every engine
  (docs/lore). A single run's number says little.
- **"Better" can be a bug.** A pile that settles twice as fast may be a
  solver that kills motion, or a measurement that reads zero.

## What a regression is

Every physics change should answer four questions, and each needs a
different kind of test:

1. **Did the plumbing change the physics?** A refactor or a speed change
   must give the same answer, exactly.
2. **Does it obey the physics we can calculate?** A box on a ramp slides
   at g (sin θ − μ cos θ), whatever else changes.
3. **Is it as good as engines people ship?** Box2D, Rapier, Box3D and
   Jolt on the same scenes set the floor.
4. **Did it change from what we last accepted, in either direction?**
   Getting worse within the floor, and getting suspiciously better, should
   both be seen and agreed to.

Today's tests answer the first three. Nothing answers the fourth.

## What exists today

| layer | what it compares | direction | targets | catches | misses |
|---|---|---|---|---|---|
| **Equivalence** | two implementations of the same step: the ECS against plain arrays (`:tax`), the mod against the arrays, the solver's four-lane path against one contact at a time, a replay after a reload or at another frame rate | exact, bit for bit | `//engine/std/physics:tax`, `compare:quality_test` (`the_mod_is_the_arrays_bit_for_bit`, `the_solve_by_level_…`), `physics_test` (replays), reload tests | the plumbing changing physics | nothing about quality: both sides change together |
| **Physical law** | a measured quantity against a formula | both ways, within a tolerance | `compare:behaviour_test`, `physics3d:behaviour_test` (landing with the card-house fix) | wrong friction, rolling, restitution; bleeding energy; sticky or slippery contacts | only the scenes with a formula |
| **Reference floor** | our settling measures against the references' values on the same scenes, measured once and dated, turned into bounds by fixed rules | upper bounds only: rest, depth, energy, top moved, lean | `compare:quality_test`, `quality_long_test`, `physics3d:quality_test`, `quality_long_test` | falling below what shipped engines do | drift within the bounds; anything too good |
| **Scene sanity** | contacts per body, islands, nothing escaped | lower and upper | inside the floor tests | a scene that isn't what we think (the columns pile) | – |
| **Unit behaviour** | single contacts and bodies against hand-checked outcomes | exact or tight | `physics:core_test`, `physics3d:core_test`, the solver's and narrowphase's unit tests | a landing that bounces, a friction limit, a feature id that jumps | whole-scene behaviour |
| **Game acceptance** | routes an agent played, replayed: "YOU WIN at frame 255, deaths 0" | exact outcome | `pong:pong_test`, `platformer:platformer_test`, their reload tests | a physics change that breaks a game | why: a changed outcome says nothing about better or worse |

How bounds are set is in [physics.md, "Quality as a test"](physics.md#quality-as-a-test),
and how to re-measure the references in runbook 005. The rule there is
kept: a bound we no longer meet is a finding (a bead and an ignored test),
never a looser bound.

**Every test is checked by planting the break it claims to catch**
(CLAUDE.md, "green has to be earned"): friction halved, the old warm start
restored, an inverse mass squared. That keeps tolerances honest.

## The gaps

1. **Drift within the floor is invisible.** If a change moves a pile's
   rest from step 150 to 210 against a bound of 220, everything stays
   green. Our own values are recorded only in comments beside each bound
   ("ours: at rest from 10, 20, 30"), which nothing checks, and which go
   stale.
2. **"Too good" passes the floor.** Every settling bound is an upper
   limit, so a solver that over-damps settles sooner and scores better.
   Only the physical-law scenes, which include free motion, would catch
   it.
3. **A broken measurement passes.** A metric that reads 0 meets every
   upper bound. Planting breaks in the physics doesn't test this, since
   the measurement is what's broken.
4. **Branches are measured on different code.** On 2026-09-28 the card
   house fell after merging two branches that each passed alone: one
   changed the solver's warm start, the other added the scene, measured on
   the old solver. It was caught only because the scene's bound was
   tight. A merge needs a comparison of the merged tree against both
   parents, not two green runs.
5. **Sleeping too early is untested.** The floor tests run with sleep off.
   Sleep is tested for taking everything eventually, not for never
   freezing a body that is still tipping.
6. **The default suite is slowing.** `compare:quality_test` went from 6.7
   s (as documented) to 34 s, and to 72 s with the pyramids in three
   orders. There's no budget, so each scene added is a small, permanent
   tax on every change.

## The design

Keep the four existing layers as they are. Add a fifth, the **baseline**,
and three smaller pieces.

### The baseline: our own accepted results

**A checked-in file of our measured values, per scene and metric**, and a
test that compares a run against it within a noise band, *in both
directions*. A change that moves a value past its band fails, and becomes
green again only when the file is regenerated in the same commit. So the
diff of that file *is* the record of what the change did to physics, and
review is where "worse but within the floor" or "suspiciously better"
gets an explicit yes or no.

- **Where:** `engine/std/physics/compare/baseline.txt` (2D) and
  `bench/physics3d/baseline.txt` (3D), next to the scenes. Two files per
  dimension: one for the default suite's scenes, one for the long suite's.
- **Format:** plain text, one line per scene and metric, sorted, so diffs
  read directly:

  ```
  # scene                      metric          value      band
  pile 400 41 turning          rest            200        steps 30
  pile 400 41 turning          depth_end       0.0137     rel 0.10
  pile 400 41 turning          energy_end      1.6e-8     log 10
  pyramid 5050 turning         rest            450        steps 50
  card_house 5                 fallen          0          exact
  piles 400-1200 turning       rest_median     220        steps 30
  ```
- **What's recorded:** the numbers the floor and law tests already
  compute, no new measures. For chaotic families (piles at several sizes)
  the median and the worst over the sizes are recorded, not each size.
  Those are what's stable.
- **Bands by kind of metric:**

  | kind | band | why |
  |---|---|---|
  | steps (rest) | absolute steps, plus a percentage for large values | rest is sampled every 10 steps, and moves in jumps |
  | lengths (depth, top moved) | relative, about 10% | smooth in the parameters |
  | energy | a factor (log scale), with values under `STILL` (1e-8) equal | energies span ten decades, and anything under `STILL` is rounding |
  | counts and flags (fallen, escaped, stands) | exact | a card falling is never noise |
  | analytic results (ramp acceleration) | tighter than the law test's tolerance | the law test is the floor; the baseline sees drift inside it |

  Band sizes start from what we measure: run each scene family under
  small perturbations that shouldn't matter (a reassociated sum, a
  different pile size in the family) and set each band just outside that
  spread.
- **Regenerating:** one command writes the file from a run:
  `./bazel run //engine/std/physics/compare:baseline -- --write`. Bazel
  runs it with `BUILD_WORKSPACE_DIRECTORY` set, so it can write into the
  source tree. Without `--write` it prints the comparison as a table: old,
  new, band, and *better* or *worse* for each value that moved. That table
  is the before-and-after every physics report already includes, made
  automatic.
- **The test:** `compare:baseline_test` (and 3D's) fails on any value past
  its band, listing all of them, not just the first. It's the same
  `Broken` pattern the floor tests use.
- **Improvements update it too.** A value that got better past its band
  also fails until the file is regenerated. That tightens the ratchet, so
  a later change can't quietly give the gain back, and it's what turns
  "too good" into something a person sees.

**Why a band and not exact values.** The runs are deterministic, so an
exact baseline is possible. But every solver change, even a reassociated
sum, would then rewrite hundreds of chaotic numbers. The file's diff would
be noise, and reviewers would learn to accept it unread. A band keeps the
diff to what actually moved.

**Why not tighten the floor to our own values instead.** The floor answers
"as good as shipped engines", and only moves when the references are
re-measured. The baseline answers "as we last accepted", and moves with
every deliberate change. Mixing them would lose the floor's meaning the
first time we accept a regression.

### Merging

After merging branches that touch physics, the baseline is regenerated on
the merged tree and compared against *both* parents' files. A conflict in
the file is never resolved by hand: it's regenerated. A value that differs
from both parents is a finding about the combination, which is exactly
the card-house case.

### Measurements that are themselves tested

Calibration scenes where every metric has a known value: a box resting
with a planted overlap (depth), a body moving at a known speed (energy), a
box turned by a known angle (lean), a layout with a known count of
contacts and islands, a body still from a known step (rest). A unit test
per metric in `quality.rs` and 3D's `measure.rs`. This closes gap 3,
cheaply.

### Scenes for the named gaps

- **Sleeping too early**, with sleep on: a box tipping slowly over an
  edge, and a disc rolling slowly down a shallow ramp, must not sleep
  while they move. Compared against Box2D's and Rapier's sleep, which the
  harness can turn on (`SLEEP=1`).
- Every new scene family gets scene-sanity checks, as the piles have.

### Families at the edge

**Built** (2026-09-28, get-emj.61). A scene at the edge of stability (a
card house, a stack near its buckling height, dominoes near their reach,
heavy boxes on light ones) goes either way on rounding, in every engine:
Box2D's 5-storey card house stands in Box2D, loses two cards in Rapier,
and in ours stood or fell as the warm start's last bits changed. A bound
on one such run is a coin toss that a correct change can lose. So:

- **Edge-of-stability scenes are judged as families, by a share bound,**
  as chaotic piles are judged by medians over their sizes. A family is a
  grid of one scene across what decides it (lean and friction, angle
  about the friction angle, height, spacing, mass ratio), each run a yes
  or a no by a fixed rule, and the family's share of yeses is bounded.
- **The bound is the less reliable reference's count on exactly that
  grid**, measured once and dated, as the floor's other bounds are. Not a
  coarser or neighbouring grid: shares move with where the grid sits
  against the edge. Where a reference fails most of a grid (Box2D sways
  stacks over), the bound is weak, and the family is a record of how far
  ours is from the other.
- **A short grid in the default suite, the full one in the long suite**
  (`compare:behaviour_test`, `behaviour_long_test`; `family.rs` holds the
  grids and the rule). The short grid is chosen to straddle the edge, so a
  planted break moves it: friction halved, the short card grid stands
  none of 12 against a bound of 7.
- A family whose every engine is right at every point (a box 0.1° either
  side of the friction angle, the ladder 0.005 from its friction) is a
  law test, not an edge: it stays, and would catch a friction model that
  isn't Coulomb's.

### A time budget for the default suite

- **Default suite:** each physics test target within about 30 s in
  fastbuild, and the physics targets together within about 2 minutes of
  wall time when run in parallel. A scene goes in the default suite if
  it's fast and covers something no other default scene does.
- **Long suite** (manual targets, `-c opt`): everything else, with its
  own baseline. It runs before merging any change to the solver, the
  narrowphase, sleep or the step, and its baseline diff goes in that
  commit.
- **Compiling physics tests at `-c opt`** would cut the default suite's
  time several-fold, since the scenes are compute-bound. Worth measuring:
  it costs a separate build configuration for those targets.

### Game acceptance tests

Pong and the platformer stay outcome-based: they answer "does the game
still work", which no metric does. When a deliberate physics change alters
a route's outcome, the route is re-played and re-recorded, and the commit
says why. A locked-world change (get-emj.55) is the first expected case.
Their physics numbers can also go in the baseline (ball speed after the
paddle, a jump's apex), so a change there shows up as a number before it
shows up as a lost game.

### What reports contain

The baseline's comparison table replaces the hand-made before-and-after
tables in agents' reports and commit messages. Every physics change states
which values moved, which way, and why that's accepted.

## Alternatives considered

- **Golden outputs** (a hash of every body's final position). Catches
  every change and explains none. Chaos makes every change a change.
- **Statistics over many seeds** (compare distributions of rest times
  across randomized piles). The right tool for chaotic metrics, but ten to
  a hundred times the runtime. A candidate for the long suite later, if
  medians over sizes prove too noisy.
- **Running the references in each test** instead of recording their
  values. Always current, but it multiplies test time, and a library bump
  would silently move every bound. Bumps should be deliberate
  (runbook 005).
- **Only reviewing reports.** What we do today; the card house shows it
  depends on someone happening to measure the right scene.

## Plan

1. The baseline for 2D's default suite: format, the `baseline` tool,
   `baseline_test`, bands set from measured spread. Then 3D, then the long
   suites.
2. Calibration tests for every measure.
3. The sleep scenes.
4. Move the default suite within budget, and measure `-c opt` for physics
   test targets.
5. Update runbook 005 and CLAUDE.md's testing conventions: when to
   regenerate the baseline, and the merge rule.

## Open questions

- **Band sizes.** The table above is a starting shape. The real sizes come
  from measuring spread under changes that shouldn't matter.
- **Other machines.** The runs are deterministic on one machine and build.
  Different CPUs or compiler versions could shift float results. That
  matters once there's CI, or a second developer.
- **Who reviews a baseline diff** when an agent makes the change? The
  proposal is that the agent's report leads with the diff's table, and
  merging needs your yes when anything moves the worse way.
