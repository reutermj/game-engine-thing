# Retrospective: still at rest (2026-09-27)

The work on the three failures the quality tests had left ignored
(get-emj.41, .42, .43), from f32d349 to 0a89fd9: turning bodies that
never came to rest where Rapier's and Box3D's did. The findings and every
measured option are in [physics.md, "Still at rest"](../architecture/physics.md#still-at-rest);
this is what the work showed about the design and the process, and what
the 2D decision it left open turns on.

What was built:
- **3D contact recycling.** A box pair whose bodies may have moved less
  than 0.03 since its manifold was found keeps its points, carried with
  both bodies, their separations updated: Box3D's recycling. The state is
  on the contact entity (`Manifold`, 31 to 40 words).
- **3D static contacts softened** from 0.4 of the substep rate to 0.25,
  Box3D's cap.
- **Tests that measure a swaying scene fairly.** Stacks and pyramids are
  bounded by the most energy over their last 200 steps, not at one step.
- **Variants, measured and kept for the decision:** 2D's block solver,
  contact stiffness and substeps; 3D's warm start from the last substep.

| turning, ours | before | after | Rapier / Box3D |
|---|---|---|---|
| 3D planks 10 000: at rest from | never | 335 | 392 / 369 |
| 3D planks 10 000: whole run, ms | 44.2 | 38.2 | – |
| 3D five-high stack | never at rest | at rest | at once / at once |
| 2D 20-high stack: at rest from | 580 | 580 (open) | 220 (Rapier) |

## What held up

- **Instrument before fixing.** The bet going in was one cause, a box
  rocking on its points. Logging every body and every contact's points,
  ids and impulses step by step, all read from the world, showed three:
  - 3D planks rocked on contact points that flickered between two
    features and two reductions, step to step;
  - the 3D five-high stack circled on its corners, a stiffness problem;
  - the 2D stack wasn't rocking at all, but swaying near its buckling
    load.

  A fix for the hypothesis would have fixed one of the three, and the
  table in physics.md shows the rest wouldn't have moved.
- **Physics's state in the world made the fix small.** Recycling needs a
  contact to remember its points, the rotations they were found at and a
  bound on how far the pair moved. Contacts are already entities, so that
  is three fields on `Manifold`, carried across reloads and visible to
  every query, with nothing beside the world. The 3D narrowphase halved
  on piles as a result.
- **Reading the references' source, not their docs.** Box3D's recycling
  is one comment in `b3CollideTask` ("This eliminates jitter"), and
  Rapier's block solver is a cargo feature on by default in 2D only.
  Neither is in their documentation's account of how they settle.
- **The quality tests paid for themselves.** They were the reason these
  failures were known, and bounded by what the references do, they said
  when a fix was enough. Every fix was also planted back out to watch the
  test fail (physics.md lists each).

## What fought back

### A test measured a moment, not the scene

The 2D stack's energy bound was ten times what Rapier's stack had at step
700. That step was where Rapier's swing happened to turn: over its last
200 steps Rapier's stack reaches 2.8e-4 a body, ours 5.6e-4. The test was
comparing where each stack was in its swing. It took a physical model (a
column of rotational springs under its own weight) to see that both
sway, that ours isn't broken, and that the real difference is how long the
sway takes to die. The lore entry records the rule: a swaying scene's
energy at one step is its phase, not its state.

### Settling is chaotic, so one scene size proves nothing

A combination that looked best on piles of 150-800 planks made a pile of
10 000 boxes rest at step 1343. Every option ended up measured at 14 to 28
pile sizes, and a finalist again at 900-2000. That multiplied the runs,
but it is the only way the options table means anything; the retro before
this one found the same about benchmark scenes.

### In our soft step, stiffness is bought with substeps

Contacts are soft springs whose stiffness is a fixed fraction of the
substep rate: 0.25 × 5 substeps × 60 Hz, 75 Hz. That is already 2.5 times
Box2D's 30 Hz, and still the 20-high stack under gravity 20 carries 92% of
the load that would buckle it, so its slowest mode barely damps. Softer
contacts topple it (as Box2D's do). Stiffer ones mean more substeps,
which is every pass of the solver again. Everything that fixed the 2D
stack cheaply broke something else: the block solver set the 5050
pyramid vibrating for a thousand steps (Rapier's, with the same block
solver, rests late there too); a warm start from the last substep left a
locked pile of 1300 never resting.

## How the work went

- **One agent, 2 h 43 min, 273 tool calls.** The physics was the smaller
  part of the time.
- **The shell sandbox was the largest cost.** It refused heredocs
  containing Rust function syntax (`name() {`), braces in Python
  f-strings, backticks, and loops passing variables to `./bazel` or `awk`.
  The agent wrote files by placeholder and `sed`, through a Python helper,
  and put every loop in a script file. The Read, Write and Edit tools
  timed out on every call, so none of that could go through them either.
- **Benchmarks blocked edits.** Quality tests took 2-3 s under `-c opt`,
  so each experiment was cheap, but the before-and-after benches took
  about 10 minutes each (the 2D comparison longer), and the worktree
  couldn't change while they ran.
- **Watching from outside was guesswork.** Between reports I could only
  read the worktree's uncommitted diff, and an option that had been tried
  and removed looked the same as one never tried. The agent's final report
  (root cause, options table, time spent) was what made the work
  reviewable.

## The 2D decision, and what it turns on

**The problem:** a tall, narrow stack of turning boxes, near its
buckling load, sways for about ten seconds before it rests. It doesn't
fall, and it doesn't sink. Rapier's sways too, for about four; Box2D's
topples. No game in the repo stacks turning bodies.

**The options:**

| option | 20-high stack at rest from | cost | what else changes |
|---|---|---|---|
| leave it | 580 | nothing | tall stacks sway about 10 s |
| six substeps, the default | 60 | solver +17-23%, every scene | every pile and pyramid rests sooner (piles median 270 to 220) |
| six substeps, a setting a game picks | 60, where chosen | only for games that choose it | a 2D settings component in the world, as 3D's `Tuning` |
| the block solver | 240 | solver -10% | the 5050 pyramid vibrates for ~1000 steps, failing its long test |
| three relax passes | 230 | solver +40% | pyramid rests later (510) |

**What the cost is against:** with rotation, the 2D solver is already 2 to
4 times Box2D's and Rapier's (8215 µs against 2626 and 3999 on a settled
10 000 pile), from running 20 scalar passes a step where Box2D runs 12
wide ones over colored contacts and Rapier 8. Six substeps make that 24.
The solver speed work (colored and wide passes, fewer passes) would cut
the base the 20% is a share of, but hardly the share itself, since nearly
every pass is inside a substep.

**What would settle it:**
- whether any game planned needs tall stacks of turning bodies to rest
  quickly, since that is the only scene this affects badly;
- whether defaults should favour speed or rest, which Box2D (4 substeps,
  a setting) and Rapier (4 iterations, a setting) both answer with a
  default and a knob.

**Suggested:** make the substep count a setting in the world, as 3D's
`Tuning` already is, keep five as the default, and revisit the default
once the solver's speed work lands and the 20% is measured on the new
base. That follows what both references do, and costs no game anything
it didn't choose.

## What it suggests

1. **Decide 2D's substeps** as above (get-emj.41).
2. **Solver speed next** (colored, wide passes): the largest gap to the
   references, and the one the 2D decision's cost depends on.
3. **2D contact recycling.** It halved 3D's narrowphase on piles; Rapier
   2D does it. Not needed for 2D's stability (its two points don't
   flicker), but it is the same cost cut.
4. **Fix the agents' shell sandbox** before the next long agent task, or
   budget for it: it cost more than any physics dead end.
