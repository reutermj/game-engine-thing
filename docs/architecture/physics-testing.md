# Physics regression testing

**Status: the baseline, the calibration tests and the wider families
built** (2026-09-28, get-emj.62), and every build optimized by default
(get-emj.66); the sleep scenes not yet (get-emj.68). How physics is kept from
getting worse as it changes, including the changes that are meant to
improve it: the layers of tests as they stand, and the one added, a
record of our own accepted results that every change is compared against
in both directions.

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

The first four layers below answer the first three; the baseline answers
the fourth.

## What exists today

| layer | what it compares | direction | targets | catches | misses |
|---|---|---|---|---|---|
| **Equivalence** | two implementations of the same step: the ECS against plain arrays (`:tax`), the mod against the arrays, the solver's colored four-lane path (the default) against its colors' order solved one contact at a time, which a parallel solve over the colors will have to pass too, and the level path (a variant) against pair order, a replay after a reload or at another frame rate | exact, bit for bit | `//engine/std/physics2d:tax`, `physics2d/compare:quality_test` (`the_mod_is_the_arrays_bit_for_bit`, `the_colored_solve_…`, `the_solve_by_level_…`), `physics2d_test` (replays), reload tests | the plumbing changing physics | nothing about quality: both sides change together |
| **Exact fingerprint** | 3D, which has no second implementation: the physics3d mod and its solver against their own pinned bits ("The exact fingerprint", below) | exact, bit for bit | `physics3d:exact_test` | any change to a bit of the 3D step, and which layer it was in | why: a pin says only that something moved |
| **Physical law** | a measured quantity against a formula | both ways, within a tolerance | `physics2d/compare:behaviour_test`, `physics3d/compare:behaviour_test` (landing with the card-house fix) | wrong friction, rolling, restitution; bleeding energy; sticky or slippery contacts | only the scenes with a formula |
| **Reference floor** | our settling measures against the references' values on the same scenes, measured once and dated, turned into bounds by fixed rules | upper bounds only: rest, depth, energy, top moved, lean | `physics2d/compare:quality_test`, `quality_long_test`, `physics3d/compare:quality_test`, `quality_long_test` | falling below what shipped engines do | drift within the bounds; anything too good |
| **Scene sanity** | contacts per body, islands, nothing escaped | lower and upper | inside the floor tests | a scene that isn't what we think (the columns pile) | – |
| **Unit behaviour** | single contacts and bodies against hand-checked outcomes | exact or tight | `physics2d:core_test`, `physics3d:core_test`, the solver's and narrowphase's unit tests | a landing that bounces, a friction limit, a feature id that jumps, a contact colored against Box2D's rule (which the equivalence tests can't see, solving the same order on both sides) | whole-scene behaviour |
| **Baseline** | our values against our last accepted ones, per scene and measure, a band each set from measured noise | both ways | the `baseline` test in `physics2d/compare:quality_test`, `behaviour_test` and 3D's, and `baseline_long` in the long ones | drift within the floor; a result suspiciously better | a pile family's median rest moving under 100 steps |
| **Calibration** | each measure on bodies placed where its value is known | exact | `quality.rs`'s and `settle.rs`'s tests (2D), `physics3d/compare:measure_test` | a measure that reads zero or the wrong thing | – |
| **Game acceptance** | routes an agent played, replayed: "YOU WIN at frame 255, deaths 0" | exact outcome | `pong:pong_test`, `platformer:platformer_test`, their reload tests | a physics change that breaks a game | why: a changed outcome says nothing about better or worse |

How bounds are set is in [physics.md, "Quality as a test"](physics.md#quality-as-a-test),
and how to re-measure the references in runbook 005. The rule there is
kept: a bound we no longer meet is a finding (a bead and an ignored test),
never a looser bound.

**Every test is checked by planting the break it claims to catch**
(CLAUDE.md, "green has to be earned"): friction halved, the old warm start
restored, an inverse mass squared. That keeps tolerances honest.

## Where the tests live

Each physics mod has its harness beside it, `compare/`: its scenes built
in every engine, its measures, its quality and behaviour tests, its
baseline files and tool, and the comparison binary that prints the
tables the bounds come from (runbook 005).

| package | what | reference engines |
|---|---|---|
| `//engine/std/physics2d/compare` | the 2D mod, and our 2D step on arrays (bit for bit the mod) | Box2D v3.1.1, Rapier 2D 0.36 |
| `//engine/std/physics3d/compare` | the physics3d mod in the engine, on `pile3d`'s scenes | Rapier 3D 0.36, Box3D 0.1, Jolt 5.6 |
| `//engine/std/physics_testkit` | what the two share, apart from any dimension | – |

**The test kit** (`physics_testkit`, 2026-09-29) is the part of testing
physics that doesn't depend on the dimension, taking plain values or
small traits, never a 2D or a 3D type:

- `baseline`: the file format, bands and comparison (below, "The
  baseline"); `tool`: the baseline tool's command line, which each
  harness's `baseline_main.rs` fills with its files, its commands, its
  offset and its record;
- `behaviour`: a run's named values (`Behaviour`, `NEVER`), and the two
  bounce measures that are the same arithmetic in either dimension once a
  harness says how high a body is and how fast it rises: a dropped ball's
  apexes, and a series' decay by the energy's height;
- `bounces`: the bounce families' statistics and their bands, over any
  bounce that says what it is (`Bounce`: restitution, speed, friction,
  round, flat, tipping); each harness keeps its grids and its threshold;
- `broken`: `Broken`, every bound a test broke, not the first, with
  `MATRIX=1` printing every bound met or not; `Named` and `Values` say
  what names a run and what it measured;
- `stats`: median, most and least, a family's statistics;
- `runs`: `Runs`, each scene run once a test binary whichever test asks
  first, and `par`, runs in threads, at most so many at once (2D 256, the
  threads a long grid spawns; 3D 64, the engines a process can map).

What stays in each harness is what differs in substance: the scenes and
their grids, the engines and how each is driven, the settling measures
(2D's `settle.rs` and `quality.rs` read bodies from arrays or the ECS,
3D's `measure.rs` reads engine states, and the two record different
things), what each baseline records and with which bands (`record.rs`:
the families differ, and so do the measured spreads), the bounds and the
references' values they come from (`PileBounds` and `StandBounds` have
different fields, set from different engines), and one bounce's own
measure (`behave::hit`): the same physics, but on 2D and 3D vectors in
their own arithmetic (a momentum is `hypot` in 2D and a three-component
root in 3D), and with its values found in a different order, so sharing
it would change bits and the comparison's tables. Each harness's
`runs.rs` keeps its own keys and wrappers, since what a run is (a variant
of ours on arrays; ours tuned, in the engine) is the harness's.

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
6. **The default suite is slowing.** `physics2d/compare:quality_test` went from 6.7
   s (as documented) to 34 s, and to 72 s with the pyramids in three
   orders. There's no budget, so each scene added is a small, permanent
   tax on every change.

Gaps 1 and 2 are closed by the baseline, 3 by the calibration tests, 4 by
the merge rule (runbook 005), within what the bands can see (a pile
family's median rest moving under 100 steps isn't seen: "What the spread
says"); 5 is get-emj.68; 6 is measured and has a way out (get-emj.66).

## The design

Keep the four existing layers as they are. Add a fifth, the **baseline**,
and three smaller pieces.

### The baseline: our own accepted results

**Built** (2026-09-28, get-emj.62). **A checked-in file of our measured
values, per scene and measure**, and a test that compares a run against
it within a noise band, *in both directions*. A change that moves a value
past its band fails, and becomes green again only when the file is
regenerated in the same commit. So the diff of that file *is* the record
of what the change did to physics, and review is where "worse but within
the floor" or "suspiciously better" gets an explicit yes or no.

- **Where:** four files, next to the scenes:
  `engine/std/physics2d/compare/baseline.txt` (2D, the default suite's
  scenes: 228 values, 27 of them the bounce families') and
  `baseline_long.txt` (the long suite's: 225),
  `engine/std/physics3d/compare/baseline.txt` (100) and `baseline_long.txt` (94).
  The format and the comparison are the test kit's (`baseline.rs`,
  and the tool's command line, `tool.rs`); `record.rs` in each harness
  says what is recorded and with which band; `runs.rs` in each runs every
  scene once per test binary, on the kit's cache.
- **Format:** plain text, one line per scene and measure, in the order
  the suite records them, columns separated by two spaces or more (a
  scene's name has single spaces):

  ```
  quality    piles 400-1200 41 turning  rest_median          220        steps 100 or 20%
  quality    piles 400-1200 41 turning  depth_end_median     0.01439    rel 0.45 or 1e-4
  quality    pyramid 25 turning         energy_tail          6.781e-12  log 3 under 1e-8
  quality    pile 1000 41 sleeping      asleep_at            240        steps 60
  behaviour  ratio 1000 5               escaped              4          exact
  behaviour  family cards               good                 11         within 1
  ```

  The first column is the group, the test binary that checks the line:
  `quality` the quality tests, `behaviour` the behaviour tests.
- **What's recorded:** the numbers the floor and law tests already
  compute, no new measures, and the ignored tests' too (a heavy box
  through the floor, a bounce over e², the bullets through a wall), so
  that fixing what they found shows as a value moving the better way. For
  a pile family (one scene at several sizes) its statistics over the
  sizes, never one size's value: the medians of its rest and of its first
  look at rest, of its depths at the end and while landing, of its
  energy; the worst of its depth and of its mean overlap; the least
  contacts a body; escapes. A family's worst rest isn't recorded, nor its
  worst energy, since one pile decides them (below). For a pyramid family,
  medians and worsts of rest, the top's move, depth, tilt, energy, and the
  count standing. For an edge family, its count of runs that did what
  they should.
- **Bands by kind of measure,** each set just outside the spread measured
  under changes that shouldn't matter (the next list):

  | kind | band | spread measured |
  |---|---|---|
  | a pile family's median rest (5 sizes or more) | 100 steps or 20% | 90 (161 wide, turning); 80 at the default's turning family at sizes 10% more |
  | its median first look at rest | 130 or 25% | 120 |
  | the same, three sizes (the big piles) | 250 or 30% | 240 |
  | a family's depths (median; the worst at the end) | rel 0.45 | 41% (21 wide, landing) |
  | its greatest mean overlap | rel 0.3 | 24% |
  | its median energy | a factor of 10, equal under `STILL` | 1.04 where it rests |
  | contacts a body | rel 0.03 | 1.6% |
  | a scene that stands (pyramid, stack; a pyramid family) | rest 10 steps (a look); top and depth rel 0.02; tilt rel 0.1 or 0.01°; energy a factor of 3 | rest 0; 0.2%; 4%; 1.13 |
  | a pile of 1000 all asleep | 60 steps | 50 |
  | behaviour: lengths and speeds; steps to rest or apart | rel 0.03; 10 steps | 1.2%; 0 |
  | a hand calculation (ramp acceleration); a bounce's apex | rel 0.002; rel 0.01 | 0; 0 |
  | an edge family's count | within one run, or 2% of a long grid | one (dominoes 3 → 4 of 4, cards 105 → 106 of 135) |
  | escapes, a heavy box standing, dominoes in order | exact | 0 |
  | a bounce family (2D and 3D): its shares of energy; its counts; what's kept of a speed or a height; the momentum lost | rel 0.05 or 1e-3; one run in 50 (3D: one); rel 0.01 or 1e-3; rel 0.5 or 1e-4 | not yet measured: set by the kind of value (get-emj.70) |
  | 3D: a pile family's median rest; depth; landing and mean; partners | 90 or 30%; rel 0.5; rel 0.35; rel 0.25 | 84; 41%; 31%; 20% |
  | 3D: one big pile (the long suite's 1000 and 10 000): depth, mean overlap, partners; energy | rel 0.6, 0.25, 0.15; a factor of 10 | 55%, 19%, 14%; 6.6 |
  | 3D: a stack: rest; top and depth; tilt; energy | 10 steps; rel 0.15; rel 0.1 or 0.05°; a factor of 3 | 0; 9%; 0.014° (the measure's resolution is 0.02°); 1.13 |

  Counts are exact except an edge family's: a single run at the edge of
  stability is flipped by rounding, which is why these are families (and
  a card count that moved by one was the first thing the planted sums
  showed).
- **How the spread was measured** (2026-09-28): every family at sizes 2,
  3, 5 and 10% either way (`--offset=<percent>`; in 3D each size's pile
  at the next seeds, `--offset=1` to `11`), recorded under the sizes'
  names, so that a family's statistics are read at its neighbours; and
  three sums reassociated in the 2D solver (the separation, `a + (b + c)`;
  the turning part of the normal speed; the locked contacts' normal
  impulse, `a - b - c` as `a - (b + c)`) and two in 3D's (the separation;
  the normal speed), each run and reverted. Then every one of those runs
  was compared against the written files, and each band widened until
  none failed.
- **What the spread says:**
  - *Stand scenes and behaviour scenes are steady:* no rest moved, no
    length by more than 1.2% (3D stacks 9%). Their bands are tight, and a
    real change shows.
  - *A pile family's statistics are not.* Its median rest moves by up to
    90 steps at neighbouring sizes; its worst is one pile's and flips
    (from 240 to 390, from 400 to 1050, from never to 1380). Where piles
    move again after resting (get-emj.63; the big turning piles in every
    engine), even the median does: the big turning piles' is 350, 690,
    740 or never at sizes 2-10% away, the locked 81-wide family's 260 or
    510. So those families record only their first look at rest. This is
    the lore on chaotic rest, measured on the families, and it is what the
    baseline can and can't see: a change that moves a pile family's
    median rest by less than 100 steps is not seen, and more sizes are the
    only way to see less (the long suite's families have 7-10).
  - *First measured bands were too tight.* Measured on offsets of 10-50
    bodies alone, the default family's median moved 30 and a band of 40
    looked enough; sizes 10% away moved it 80, and the wider families' by
    up to 250. The bands above hold every run of both.
- **Regenerating:** one command writes the file from a run:
  `./bazel run //engine/std/physics2d/compare:baseline -- --write`
  (`--long` for the long file, about a minute; 3D's is
  `//engine/std/physics3d/compare:baseline`). Bazel runs it with
  `BUILD_WORKSPACE_DIRECTORY` set, so it writes into the source tree.
  Without `--write` it prints the comparison as a table: old, new, band,
  and *better*, *worse* or *moved* for each value past its band (`--all`:
  every value). Unoptimized and optimized builds write the same file (checked: the
  steps are the same arithmetic), the default suite's in 51 s and 1 s.
  `SOLVER=<variant>` (3D: `TUNE=`) compares a variant against the
  baseline, as the tests take it.
- **The test:** each quality and behaviour test binary has a test
  `baseline` (and in the long ones `baseline_long`) that checks its
  group's lines and fails on any value past its band, listing all of
  them as the table, with the command that writes the file. They read the
  runs the other tests in the binary make (`runs.rs`: each scene run once,
  whichever test asks first), so the baseline costs the default suite no
  runs of its own: `:quality_test` went from 58 to 51 s with it, since the
  pyramids and the sleeping pile that two tests ran are now run once.
  `//engine/std/physics2d/compare:baseline_test` (and 3D's, and
  `baseline_long_test`) is a test suite of those binaries: a target of
  its own would run every scene a second time. The file is built in
  (`include_str!`), so a change to it reruns the tests.
- **Improvements update it too.** A value that got better past its band
  also fails until the file is regenerated. That tightens the ratchet, so
  a later change can't quietly give the gain back, and it's what turns
  "too good" into something a person sees.
- **Checked by planting** (2026-09-28, each in the source, the tests run,
  the source restored):
  - Friction halved in the 2D solver fails `:behaviour_test`'s
    `baseline` on 19 values (the ramps, the slipping disc, the ladders,
    the dominoes' wave, overlap recovery, a fifth light box through the
    floor, the card, ramp and ladder families) and `:quality_test`'s on
    16 (the piles' contacts a body, the turning pyramids' tops and tilts,
    the sleeping pile all asleep at 460 where it was 240). No pile
    family's rest moved past its band: those bands are wide (above).
  - The same change with the file written passes both.
  - Friction restored against that file, a change for the better, fails
    on the same values the other way (the sliding box back to 6.536
    where it was 8.268, the cards 11 where they were 0), as *better*,
    until written.
  - In 3D, friction halved fails `:behaviour_test`'s baseline (3 values)
    but not `:quality_test`'s: no pile family or stack moved past its
    band. One relax pass (`TUNE=relax=1`) fails it on 11; the separation
    to first order in the turn (`anchors=linear`) on none.

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

**Built** as a rule (runbook 005, "The baseline"; CLAUDE.md). After
merging branches that touch physics, the baseline is regenerated on
the merged tree and compared against *both* parents' files. A conflict in
the file is never resolved by hand: it's regenerated. A value that differs
from both parents is a finding about the combination, which is exactly
the card-house case.

### The exact fingerprint

**Built** (2026-10-02, get-emj.89), for 3D. 2D's step is held bit for
bit to a second implementation, the arrays; 3D has none, since its
comparison runs the mod itself. So a one-ulp change to 3D's gather or
solver kernel (the gravity given back, a substep's share) moved 35 of
the 100 baseline values, all inside their bands, and failed one quality
bound by chance (flows.md, "physics3d"); the reload replay can't see it,
comparing the same code with and without reloads. The other side of the
comparison is a pinned value instead: `//engine/std/physics3d:exact_test`
against `engine/std/physics3d/tests/exact.txt`, in two layers
(`tests/exact.rs`):

- **The mod**: pile3d's scene in the engine (a mixed pile of spheres and
  boxes free to turn, a plank and a cube thrown in spinning, a sphere
  rolling, a locked box; contacts of one to four points; 44 bodies, 120
  frames), one line a frame, a hash per component (position, rotation,
  velocity, angular velocity, manifold, impulse) of every value, by
  entity. A failure names the first frame and the components that moved
  there: a manifold first is the narrowphase, a position alone the
  write-back. The test also checks the scene is what it says (every
  count of points seen, the locked box unturned).
- **The kernel**: `solver::solve` alone, on bodies and contacts made from
  a seed (`kernel_inputs`: bounces, friction at its limit, overlaps past
  the push, gaps, a body that can't turn), three steps at each of twelve
  tunings, one line a tuning. Nothing of the narrowphase, the gather or
  the world is in it, so it moves when the arithmetic changes and holds
  when only the order of the solve does. Two more tests check its inputs
  exercise what they claim, and that reversing their order changes every
  tuning's result.

Values are hashed through `Debug`, which prints an `f32` as the shortest
text that reads back to its bits. Neither scene calls libm (no sines or
powers; `sqrt` is exact by IEEE 754), so the pin is the code, the compiler
and its flags: `--config=bench` gives the same file. It runs in well under
a second.

**What it caught**, each planted alone (2026-10-02):

| one-ulp change | `exact_test` | anything else in `//engine/std/physics3d/...` |
|---|---|---|
| the gravity given back (`gather_bodies`), larger | mod, from frame 21 | nothing |
| a substep's `share` (`solver.rs`), larger | mod, from frame 1; kernel, every tuning | `quality_test`'s planks bound and its `baseline` |
| a body's position written back (`scatter_bodies`) | mod, from frame 1 (position) | nothing |
| a contact point's position (box-box clipping, `narrow.rs`) | mod, from frame 21 (manifold first) | nothing |
| the last two contacts swapped around the solve (`pipeline.rs`) | mod, from frame 44; the kernel held | `behaviour_test`'s `baseline` |

**Updating it.** Like the baselines, never by hand: `./bazel run
//engine/std/physics3d:exact` says which lines differ, and `-- --write`
writes the file, only in a commit that changes results on purpose, whose
message says why (runbook 005, "The exact fingerprint"). A change that
claims to keep results (a refactor, a speed change, a lanes kernel in
pair order) leaves the file as it was; one that changes the order alone
(colouring) rewrites the mod's lines and leaves the kernel's. Since every
physics change rewrites it, the file's diff says nothing about better or
worse: that is the baseline's job, and why a band and not a pin is the
baseline (above). After merging physics branches, write it on the merged
tree, as the baselines are.

**For a kernel in lanes** (get-emj.52, built 2026-10-03): it must be the
solve one contact at a time bit for bit, so
`the_lanes_are_the_solve_one_contact_at_a_time_bit_for_bit` solves
`kernel_inputs`, and the same with a kinematic body, a contact neither end
of which moves and warm twist on ends that can't turn (`odd_inputs`), at
widths 1, 4 and 8 and at `lanes=0` (`solver::one_at_a_time`), at each
tuning and `int=exact`, and compares every body and contact after every
step as `Debug` prints them, no pin needed; the kernel's pinned lines,
solved at the default's four lanes, and the mod's, held as they were
(physics.md, "The solver in lanes", for what it caught). Colouring (get-emj.90) moves the mod's lines and not the
kernel's; its equivalence is the coloured solve against `solver::solve`
over the contacts in the colours' order, as 2D's
`the_colored_solve_is_its_order_solved_one_contact_at_a_time_bit_for_bit`.

### Measurements that are themselves tested

**Built** (2026-09-28). Every measure the floor and the baseline read is
tested on bodies placed where its value is known by hand, so a measure
that reads zero, or reads the wrong thing, fails there instead of passing
every upper bound on it (gap 3):

- **2D** (`quality.rs` and `settle.rs`, run by `:quality_test` and
  `:behaviour_test`): depth (boxes sunk 0.03 and 0.004 into a floor, one
  turned a quarter, a disc 0.008: the deepest, the mean, the count past
  0.01; touching is not overlapping); energy (a 3-4-5 motion, a box and a
  disc turning, by their inertias); tilt (turned 10°, -10°, 100°, 135°,
  90°; a disc doesn't lean); contacts a body and islands (two boxes side
  by side and one alone, and just past touching); escapes; rest (an
  engine scripted to move until step 130, again from 300 to 350, and to
  the end); a top's move (0.3 and 0.4 from where it began).
- **3D** (`measure.rs`, `//engine/std/physics3d/compare:measure_test`): depth (a cube
  sunk 0.03, one turned a quarter, a ball); energy (moving, and a cube and
  a ball turning); tilt; partners and columns (a cube on a cube, straight
  above and set 0.3 aside); escapes; rest (a scripted backend: at rest
  from step 40, from the start, never).
- **Checked by breaking each measure** (2026-09-28): the depth ignoring
  statics, the energy without its turning part, the tilt without the
  quarter turn (2D) or by `asin` (3D), islands never joined, a column
  judged at a unit off, rest never reset once a body moves again, rest a
  step early, a top's move sideways only, escapes by x alone. Each fails
  its test (thirteen breaks, thirteen failures).
- **What it found:** 3D's tilt reads 0.02° for a cube lying flat after a
  quarter turn, `acos` in f32 of a cosine within rounding of 1: the
  measure's resolution, far under any bound (1°) or band (0.05°), now
  pinned by its test.

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
  (`physics2d/compare:behaviour_test`, `behaviour_long_test`; `family.rs` holds the
  grids and the rule). The short grid is chosen to straddle the edge, so a
  planted break moves it: friction halved, the short card grid stands
  none of 12 against a bound of 7.
- A family whose every engine is right at every point (a box 0.1° either
  side of the friction angle, the ladder 0.005 from its friction) is a
  law test, not an edge: it stays, and would catch a friction model that
  isn't Coulomb's.

### Wider families

**Built** (2026-09-28). More data before tuning, so that no change is
fitted to a few scenes: in the long suites only, so the default suite
doesn't grow. Each family is measured in the references on exactly the
same grid (runbook 005) and bounded by the same rules as the default's
([physics.md, "Quality as a test"](physics.md#quality-as-a-test)):
medians over sizes for chaotic scenes, a share for edge-of-stability ones.
A bound we don't meet is an ignored test and a bead, never a looser bound.
The debug view was checked on the new scenes (a mixed pile is a pile in
ours and Box2D at steps 0 and 600).

**2D** (`record::WIDE_PILES`, `MIXED`, `PYRAMID_FAMILIES`;
`family::wide`): at rest from, median over the sizes (worst), ours against
Box2D and Rapier, bodies locked / turning:

| family | ours | Box2D | Rapier | bounds |
|---|---|---|---|---|
| piles 21 wide, 150-450 (7) | 200 (320) / 210 (350) | 160 (190) / 230 (410) | 180 (210) / 190 (310) | met / **energy**: the 450 keeps 2.9e-6 a body, bound 2.5e-7 (get-emj.63) |
| piles 41 wide, 300-1200 by 100 (10) | 210 (240) / 210 (300) | 170 (260) / 210 (700) | 160 (220) / 250 (400) | met / met |
| piles 81 wide, 800-2400 (9) | 260 (**never**) / **370** (650) | 190 (250) / 290 (never) | 220 (250) / 260 (750) | **locked piles move again after resting: 1380, never, 1150 at 1800-2200** / median past 362 (get-emj.63) |
| piles 161 wide, 2000-4800 (8) | 230 (**410**) / 380 (**never**) | 200 (230) / 350 (390) | 180 (260) / 780 (1470) | worst past 400 / the 4800 never at rest (get-emj.63) |
| piles 401 wide, 5000-12000 (8) | 230 (340) / 360 (2150) | 190 (230) / 1590 (2360) | 190 (210) / 2450 (never) | met / met |
| mixed shapes and materials, 41 wide, 400-1200 (9) | 250 (380) / 260 (480) | 210 (290) / 290 (360) | 200 (230) / 270 (never) | met / met |
| pyramids 30-60 (7) | 50 (80) / 110 (190) | 80 (90) / 60 (90) | 210 (260) / 160 (330) | met / met |
| pyramids 70-120 (8) | 150 (200) / 780 (1200) | 80 (110) / 160 (310) | 280 (never) / 1100 (1650) | met / met (get-emj.64) |

Ours sinks a quarter to a fifth as deep in every one of them (at the end,
0.011-0.028 where both references reach 0.046-0.14), as on the default
scenes. The mixed pile is circles and boxes of half extents 0.25-0.5, a
box's two apart, friction 0.1-0.9 and restitution 0-0.5, by index
(`Scene::Mixed`).

Edge families on wider or finer grids (`:behaviour_long_test`), runs that
did what they should:

| family | ours | Box2D | Rapier |
|---|---|---|---|
| card houses 3-7 storeys, lean 23-27°, friction 0.6-0.9 (100) | 69 | 72 | 48 |
| ten dominoes 0.8-1.45 apart by twentieths, friction 0.3 and 0.6 (30) | 25 | 25 | 26 |
| stacks 12-30 high, every height (19) | 9 (to 20) | 0 | 14 (to 25) |
| a box 10-1000 times as heavy on 1-6 (84) | 60 | 33 | 49 |
| a pyramid 20 wide at friction 0-0.8, finest near 0 (12) | 11 | 11 | 11 |

**3D** (`record::WIDE` in `//engine/std/physics3d/compare`), against Rapier and Box3D
(Jolt beside them): at rest from, median (worst):

| family | ours | Rapier | Box3D | Jolt | bounds |
|---|---|---|---|---|---|
| cubes turning, 200-1000 by 100 (9) | 189 (281) | 220 (364) | 215 (710) | 568 (never) | met |
| cubes locked (9) | 69 (never) | breathe from 400 | breathe from 400 | 116 (149) | depth only |
| planks turning (9) | 288 (397) | 267 (364) | 262 (823) | 907 (never) | met |
| mixed spheres and boxes, turning (9) | never | never | never | never | depth only: spheres roll on |
| mixed, locked (9) | 297 (425) | 285 (354) | 274 (347) | 308 (475) | met |
| cubes turning, 2000-5000 (4) | **402** (505) | 288 (711) | 292 (439) | never | median past 365 (get-emj.65) |
| planks turning, 2000-5000 (4) | 417 (546) | 376 (381) | 314 (340) | 1499 (never) | met |

The 3D mixed pile is one material, the pile's: the engines mix two
frictions by different rules (Rapier the mean, the others the geometric
mean), which a pile of mixed materials would measure instead of the
solvers.

**What the wider data says about get-emj.62's two drifts.**

- *The turning pile of 11 000 at rest from 1870* is not an outlier in
  its family, and not a trend at scale: of eight sizes from 5000 to
  12 000 two rest late (8000 from 2150, 11 000 from 1870) and the others
  from 250-690; the family's median is 360. It is the big turning piles'
  way in every engine: a body now and then moving over 0.05 long after
  the pile came to rest. Box2D's median over the same sizes is 1590 and
  Rapier's 2450, and every engine is first at rest by 320-400. The
  baseline records only the first look at rest for these families, since
  their rest from goes 350, 690, 740 or never with the sizes 2-10% away.
  No gap to the references: a bead isn't warranted.
- *The 5050 pyramid at rest from 780* is in line with its neighbours: the
  turning pyramids 70-120 wide rest from 290, 410, 560, 640, 780, 880, 980
  and 1200, about ten steps a unit of width, and none of them moved with
  a reassociated sum. So it is a trend with size, not noise. Against the
  references it sits between them (Box2D 90-310, Rapier 450-1650), four to
  five times Box2D's, while sinking a fifth as far (the top 0.26 lower,
  Box2D's 1.46): get-emj.64 records it, within the bound.
- *Where the wider data does show a gap* is elsewhere: piles 81 and 161
  wide, where ours comes to rest and then moves again, or doesn't rest,
  and neither reference does (get-emj.63: body 1476 of the locked pile
  of 2000, 81 wide, creeps at 0.07 mid-pile from step 1140 to the end),
  and 3D's turning cubes at 2000-5000 (get-emj.65, four sizes, just past
  the noise).

### Families of a law

**Built** (2026-09-29, get-emj.56 and .60: physics.md, "Bounces"). A
family is a grid for a behaviour with an analytic answer too, not only for
one at an edge: the bounce families put restitution over impact speed,
restitution, gravity, shape, angle, two free bodies, time, the step and
the substeps. What building them taught:

- **Measure the event from free flight, not from the engine.** Each
  engine's contact starts and ends at its own substep, margin and push-out;
  the steps where a body's velocity changed by more than gravity are the
  same test for all of them.
- **Normalize by what can turn into what.** Counted from its contact
  height, a box landing on a corner reads as gaining the energy of tipping
  flat; counted from where it lies flat, it doesn't. Counted from the
  highest step, an apex reads 3% low at gravity 80; counted by its energy
  height, it doesn't.
- **Allow for what the method gives back, and say so.** A soft contact's
  push-out lifts a body by its overlap at no cost to its speed, in every
  engine; a law of "no energy from nowhere" that doesn't allow it fails
  the references on every fast impact.
- **A worst over a grid is one mechanism's.** The first "worst gain" was a
  corner tipping, then a ball passing the margin, then a speculative
  contact catching a ball short: each a separate finding. Split a
  statistic by mechanism (square on, tipping) before bounding it, and look
  at the runs that decide it.
- **The references have the artefacts too** (Box2D and Rapier gain 9% on
  a corner, Box3D 11% on a slow edge, Rapier has no threshold), which is
  why a law's bound stays analytic even where they break it, and a
  reference bound is theirs on the same grid, not a guess at it.
- **A family can't choose for you.** Every option was measured on every
  grid; where none met every bound the table and the trade-off went to a
  person rather than one being fitted.
- **Mind the instances.** A 3D run of ours is an engine with its mods
  loaded; one process ran out of room to map them near its 2300th, so the
  3D grids are about 1500 runs and the runs go 64 at a time (2D's 256:
  9900 threads at once ran out).

**Chaotic scenes are bounded on their families alone** (2026-09-29).
Taking restitution's closing speed before the step's gravity moved every
pile a little (their bodies bounce at 0.1), and six per-size bounds of the
2D long suite broke: a pile of 700 at rest from 700 (bound 500), piles of
8000 and 10 000 401 wide never at rest or from 2500 (2450), one's energy,
the three big piles' median. They weren't a regression: every other
closing speed weighed broke a different set of them (two to eight), and
only the one they were set on passed them all, while the baseline's
family statistics moved within their bands. So the pile tests
(`quality_test.rs`, `quality_long.rs`, `PileBounds`) now bound each
family's median rest (the median first look at rest where piles move
again), and median energy by ten times the worse reference's median,
measured again on exactly the families' sizes. Per size they keep only
what didn't flip under any option: depth, contacts a body and islands,
nothing escaped. The worst rest is gone everywhere: one pile decides it,
in the references as in ours (lore, a reference engine's rest is as
chaotic as ours), and nothing measured showed any reference's worst
stable. The three-size families are judged by the eight-size ones around
them. Both defaults were run on the new bounds, so the shape wasn't
chosen for the new one: the old passes all but one family (81 wide,
turning, already ignored), the new all but two (below).

**The limit it exposed: a family bound from the references can sit
inside the chaos band.** The locked piles 401 wide are bounded at 237, a
quarter over the references' median of 190; ours were 230 and are 240,
where a family's median moves by up to 90 steps with rounding ("What the
spread says"). A bound 7 steps from ours can't tell a change from noise
(get-emj.73, its test ignored). The answer is more sizes or seeds per
family, so the median itself moves less (get-emj.67), not a looser rule.
The other family it moved, the locked piles 81 wide (median 310 against
275, 260 before), is more piles resting and moving again, get-emj.63's
own mechanism: a clue there, not noise.

### A time budget for the default suite

- **Default suite:** each physics test target within about 30 s in
  fastbuild, and the physics targets together within about 2 minutes of
  wall time when run in parallel. A scene goes in the default suite if
  it's fast and covers something no other default scene does. Measured
  (2026-09-28, each target alone): `physics2d/compare:quality_test` 49 s,
  `physics2d/compare:behaviour_test` 7.7 s, `physics3d/compare:quality_test` 17.5 s,
  `physics3d/compare:behaviour_test` 0.3 s, the same with the baseline tests
  skipped (it costs nothing: its runs are the other tests'). With the whole
  suite running beside it, `:quality_test` takes 66 s. It is over the
  budget, and was before this work (58 s beside the other physics
  targets).
- **Long suite** (manual targets): everything else, with its
  own baseline. It runs before merging any change to the solver, the
  narrowphase, sleep or the step, and its baseline diff goes in that
  commit. Measured: `physics2d/compare:quality_long_test` 80 s,
  `behaviour_long_test` 4 s, `physics3d/compare:quality_long_test` 86-130 s.
- **Every build is optimized** (`.bazelrc`, 2026-09-28, get-emj.66),
  with debug assertions and overflow checks kept on, so tests check what
  they did unoptimized. The default suite runs in about 36 s, a full
  rebuild included: `:quality_test` 1.0 s (from 49), `physics2d_test` 2.8 s
  (from 25), pong's reload test 3.4 s (from 32). The results are bit for
  bit the same (the baselines pass unchanged; rustc doesn't reorder float
  arithmetic). The cost is a few seconds per physics rebuild (the physics
  mod 1.2 s to 3.9 s after a solver edit, 2.1 s to 5.7 s after an ECS
  edit). One mode everywhere also means a hot reload never loads a mod
  built in another mode than the engine. Timings go through
  `--config=bench`, which turns the checks off: with them on, physics's
  stages run 3-14% slower (narrowphase 1052 against 920 µs on a turning
  pile of 10 000). This makes the budget above easy to keep, and long
  suites' scenes can move into the default suite.[^opt]

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
  every change and explains none. Chaos makes every change a change. Not
  as the baseline, then; but where there is no second implementation to
  be equal to, a pin is the only exact check, and 3D's is one ("The exact
  fingerprint"), a hash a frame and a component so it says where.
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

1. ~~The baseline, 2D and 3D, default and long suites, bands from
   measured spread.~~ Done (2026-09-28).
2. ~~Calibration tests for every measure.~~ Done.
3. ~~Wider families in the long suites, measured in the references.~~
   Done; they found get-emj.63 and get-emj.65, and settled get-emj.62.
4. The sleep scenes (get-emj.68).
5. ~~Move the default suite within budget~~: every build optimized
   (get-emj.66).
6. ~~Runbook 005 and CLAUDE.md: when to regenerate, the merge rule.~~
   Done.
7. Tighter pile bands from more sizes or seeds per family (get-emj.67).

## Open questions

- **Band sizes** are measured now ("The baseline"). Open: whether pile
  families' medians are worth a band 100 steps wide, or families need more
  members to say anything about rest (get-emj.67).
- **Other machines.** The runs are deterministic on one machine and build.
  Different CPUs or compiler versions could shift float results. That
  matters once there's CI, or a second developer.
- **Who reviews a baseline diff** when an agent makes the change? The
  proposal is that the agent's report leads with the diff's table, and
  merging needs your yes when anything moves the worse way.

[^opt]: 2026-09-28: before, builds were unoptimized (`fastbuild`, Bazel's
    default), and optimized ones were asked for with `-c opt`. A
    per-target transition for the physics tests alone was measured
    (about 40 lines of Starlark, a second build configuration) and
    passed over for one mode everywhere.
