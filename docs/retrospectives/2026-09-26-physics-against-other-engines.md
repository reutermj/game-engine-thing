# Retrospective: physics against other engines (2026-09-26)

The physics work after [physics in the ECS](2026-09-25-physics-in-the-ecs.md),
from 5d17ca5 to a0f9f3a. Until then everything had been measured against
our own step on plain arrays. This round measured it against engines
people ship: Box2D v3.1.1 and Rapier 2D 0.36 in 2D; Rapier 3D 0.36, Jolt
5.6 and Box3D 0.1 in 3D. The aim was to understand the gaps, not to win.
Nobody expected us to be competitive yet.

What was built:
- **Sleeping, finished and on by default.**
  - Wakes are seen by change detection, not by counts that could be fooled.
  - A wake found while finding contacts takes effect in the same step.
  - Tested on real piles, and neither game's recorded routes change.
- **The 2D comparison,** `//engine/std/physics/compare` (runbook 005):
  - the same scenes in every engine: a real pile, a pyramid and rain, at
    1k and 10k;
  - one thread, rotation locked in the references, sleeping off;
  - time by stage, and quality: penetration, settling, energy at rest.
- **Spatial storage generic over its dimensions,** 2 by default.
- **An experimental translation-only 3D step,** `//engine/std/physics3d`,
  with its comparison in `//bench/physics3d`.
- **Credits,** in [docs/CREDITS.md](../CREDITS.md), added mid-round at the
  user's request. Code implementing a Box2D or Bullet idea now names it.

Single-threaded with rotation locked: µs per step in 2D, ms in 3D:

| scene | ours | Box2D / Box3D | Rapier | Jolt |
|---|---|---|---|---|
| 2D pile 10k falling | 1361 | 2518 | 2374 | – |
| 2D pile 10k settled | 3370 | 3307 | 3861 | – |
| 2D pyramid 5050 | 2814 | 2846 | 2753 | – |
| 3D spheres 10k (ms) | 6.1 | 12.2 | 11.0 | 16.4 |
| 3D boxes 10k (ms) | 4.9 | 11.5 | 13.7 | 13.1 |

## What held up

- **The storage design generalized to 3D at no cost to 2D.**
  - A const dimension parameter on bounds, keys, lanes, pages and the
    broadphase covered it. Splits on blocks of the order, merging, change
    detection and the two-sided broadphase never look at an axis, so they
    didn't change.
  - 2D's `:tax` stayed within 2%, still bit for bit.
  - Contacts as entities carried over. Four contact points stored inline on
    a contact cost about 2% of a step; Box3D and Jolt cap at four too.
- **Speed is in the same range as shipped engines**, one thread, like for
  like. 2D is even with Box2D and Rapier settled, and ahead when everything
  falls. 3D is 2-3x ahead, but that isn't like for like: our step has no
  angular terms and no contact points, exactly where the others spend.
  Nothing measured suggests the ECS as such puts us out of range.
- **Sleeping as storage pays off.** A whole 10k pile asleep costs about 22
  µs a step, since sleeping bodies are tables no walk visits.
- **The discipline carried over.** Ours runs both as the mod and as the
  array step, and the two agree bit for bit on every scene without rain.
  Assertions caught engines not doing what we asked (below).
- **Third-party code came into the hermetic build easily.**
  - Box2D, Jolt and Box3D became `cc_library`s over checksum-pinned
    archives.
  - Rapier came through `rules_rs` unpatched, about 65 crates.
  - All built on the first try, with one fix each: Jolt's thread-safety
    warning flood from our toolchain, and `-O3` to match its release build.

## What fought back

### Our 2D solver never settles

On the 2D pile, Box2D and Rapier are at rest by step 400, with energy
about 1e-10. Ours still has bodies moving at 8.6 u/s then, and settles only
by step 4000. With sleeping on, that becomes the biggest time gap as well:
at step 400 theirs cost under 1 µs a step, ours about 3700, because creeping
bodies never fall asleep.

The cause was measured. The split impulse pushes bodies apart along
slightly tilted normals, and that sideways motion is motion friction
doesn't act on. Throwing the correction away settles the pile but lets it
sink. None of our own tests could see this: they asked whether a pile comes
to rest eventually, on a pile that turned out to stand in columns, and never
how soon compared with anything. The 3D step, the same solver without the
tilt, settles in 174-612 steps.

### The broadphase finds every pair afresh

`near_pairs` re-finds every pair every step.
- **When everything falls, that wins:** in 2D it takes 189 µs, against
  1110 for Box2D and 655 for Rapier.
- **When little moves, it loses:** a resting 2D pile costs us 450 µs and
  Box2D nothing, and in 3D ours is 3-10x Rapier's and 2-5x Box3D's.

Both keep their pairs between steps, over a tree of fattened boxes (read in
their fetched source). The last round tuned `near_pairs` against piles that
were all moving. Games spend most of their time with little moving, and
keeping pairs is what's missing.

### Like for like took more care than expected

- **Box2D quietly turned locked bodies.** `b2Body_SetMassData` ignores
  `fixedRotation`, so every body rotated. A no-turning assertion caught it,
  and the fix was to pass zero inertia (lore).
- **"A real pile" differs by engine.** `:tax`'s pile stands in columns in
  Box2D and Rapier, and collapses in ours. The 2D harness uses a staggered
  pile, checked for contacts per body and islands in every engine.
- **Defaults aren't equivalent.** Rapier's and Box3D's 3D box piles keep
  oscillating at their default iterations. Box2D sub-steps where we iterate.
  Our narrowphase is cheapest per pair only because it builds no contact
  points. There was no single point of matched quality, so both are
  reported.
- **Rain grew towers.** Locked boxes land flush and stay, so they piled
  above the spawn height and spawned inside each other. Rain became circles
  in a wider box.
- **Some numbers are soft.**
  - 10k in 3D is single runs on a shared machine.
  - Jolt is built for AVX2, where our code targets baseline x86-64.
  - Jolt exposes no stage timings.

### Structural changes between frames are slow

Spawning and despawning through `WorldMut` from a message costs about 10
µs per entity at 10k. Through a `Spawner` in a system it's effectively
free. Rain found it, and now spawns from a system.

### Tools cost every agent time

The file tools (Read, Write, Edit) timed out on every call in the agents'
worktrees, and a command checker there refused many shell commands. The
agents routed around both with shell scripts, losing about 10 minutes in
2D and 30 in 3D on runs of about two hours. That was the largest avoidable
sink in their time reports, which were new this round.

The coordinator's own `Write` then stalled the same way while the user was
away from the editor. The likely cause is the VS Code integration holding
file tools for the editor. It's avoided for now by using Bash for files;
not yet confirmed.

## How the work went

- **Two agents ran in parallel,** about two hours each. The 3D agent ran a
  helper for Rapier 3D and Jolt beside its storage work, which saved about
  an hour.
- **Time reports and progress logs were added mid-round,** at the user's
  request. The 3D agent's log gave the first view of an agent's phases while
  it worked. Both now go in every agent's brief.
- **The comparisons re-ranked the work.** Before, the next steps were the
  ordered-table splice and parallelism. After, settling and a persistent
  broadphase: both algorithm questions that the ECS doesn't stand in the
  way of.
- **A long check was started that wasn't needed.** The merge ran Miri
  because the spatial bounds glue, which is unsafe code, changed. Only its
  types had changed, not the unsafe operations; the long-checks rule is
  about the latter.

## What it suggests

1. **Make the solver settle**, reaching rest and sleep in a few hundred
   steps as Box2D and Rapier do. Options: friction on the pseudo-velocities,
   a smaller correction once a contact persists, or Box2D's soft step with
   relax. It's the biggest quality gap, and with sleeping on, the biggest
   time gap too.
2. **A broadphase that keeps its pairs**, in 2D and 3D: fattened boxes, and
   pairs kept between pages whose rows haven't left their fat boxes. It's
   the one storage-side gap both comparisons found.
3. **Quality as a test**: settling time, penetration and energy at rest on
   real piles, against bounds the reference engines meet. Then a creeping
   solver fails a test, not just a benchmark.
4. **Page size per table.** 3D is 13-19% faster with 32-row pages, while 2D
   loses with them at 1k. This is the open question in storage.md.
5. **Rotation, as its own investigation.** The 3D spike predicts it needs
   a spatial key that takes the transform (a body's box depends on its
   rotation), per-body solver data growing from 7 floats to about 20,
   contact feature ids for warm starting, and a clipping narrowphase. Every
   comparison is translation-only until then.
6. **Then the known costs:**
   - a colored, SIMD solve on one thread: about 400-500 µs at 10k in 2D;
   - ordered tables that splice (get-emj.31);
   - solver arrays kept between steps: about 170 µs;
   - cheaper `WorldMut` structural changes.
7. **Fix the file tools** before the next parallel round.
