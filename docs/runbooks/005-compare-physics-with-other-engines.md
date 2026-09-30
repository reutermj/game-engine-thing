# Runbook: compare the physics with Box2D and Rapier

- **Trigger:** a change to `//engine/std/physics2d`'s step (the solver, the
  narrowphase, the broadphase, the storage it walks) that claims to make it
  faster or better behaved; or bumping Box2D or Rapier.

## Gap

`:tax` compares the mod with our own step on arrays, bit for bit, so it
sees the ECS's cost and nothing about the algorithms. Whether a change
closes a gap to a tuned engine takes the same scenes in those engines,
measured the same way, and quality beside time, since each solver trades
one for the other differently.

## Resolution

Each mod has its harness beside it: `//engine/std/physics2d/compare` (2D,
against Box2D and Rapier 2D) and `//engine/std/physics3d/compare` (3D,
against Rapier 3D, Box3D and Jolt), each with its comparison binary, its
quality and behaviour tests and its baseline. What they share apart from
the dimension (the baseline format and tool, the bounds a test collects,
family statistics, the run cache, the bounce statistics) is
`//engine/std/physics_testkit` (physics-testing.md, "Where the tests
live").

```sh
./bazel run --config=bench //engine/std/physics2d/compare > compare.md
```

About 25 minutes with `LONG=1`, 12 without, on this machine. It prints,
per case, a table of µs per step (the median of `REPS` runs, 3 by
default, with the least and most) by stage, one of how well each engine
settled, each engine's own stages, and whether the arrays agreed with the
mod bit for bit (they must, but for rain). Narrower runs:

```sh
ONLY="pile 10000" REPS=5 ./bazel run --config=bench //engine/std/physics2d/compare
ENGINES=ecs,box2d VARIANTS=box2d:2,rapier:8 ./bazel run --config=bench //engine/std/physics2d/compare
SLEEP=1 ./bazel run --config=bench //engine/std/physics2d/compare   # each engine's default sleeping
SETTLE=1500 ./bazel run --config=bench //engine/std/physics2d/compare  # how soon each comes to rest
VARIANTS=arrays:split,arrays:soft/sub=4 ./bazel run --config=bench //engine/std/physics2d/compare
TURN=1 ./bazel run --config=bench //engine/std/physics2d/compare   # only bodies that turn
TURN=1 SETTLE=1500 ENGINES=box2d,rapier,rot VARIANTS=arrays:rot,arrays:rot/warm=0 ./bazel run --config=bench //engine/std/physics2d/compare
./bazel run --config=bench //engine/std/physics2d/compare:solver_bench   # the 2D solver alone, each way of solving
```

Every case runs twice by default: with rotation locked (every engine), as
the comparison was until rotation, and with bodies turning (`, turning`
in the case's name: Box2D and Rapier unlocked, each dynamic body given its
shape's inertia at mass 1, ours with a `Rotation` and a `Spin`). `TURN=0`
runs the locked cases only, `TURN=1` the turning ones, `TURN=2` the locked
ones with ours giving every body a `Rotation` and no `Spin` (what a lock by
a flag would leave). The `arrays:rot/...` variants change rotation's
choices (physics.md, "Rotation"): `sep`, `int`, `relax`, `sub`, `warm`,
`deepest`, listed in `variants.rs`. A settling run's steps to rest vary by
a hundred or more with rounding alone (physics.md, "Contact points"): run
more than one scene before reading a difference into them.

`SETTLE=<steps>` replaces the timing with a settling table per scene (not
rain): each engine stepped that far and looked at every 10 steps, the
step it was first at rest (every body under 0.05, the sleep threshold)
and the step it stayed at rest from, fastest body at 100, 200 and 400,
energy and deepest overlap at 400 and at the end, the deepest and mean
overlap at any look, contacts a body, islands, tilt, escapes, and for
pyramids and stacks how far the top box moved. `TRACE=1` with it names
the body that moved again. The
`arrays:` variants are other solvers on the arrays, listed in
`engine/std/physics2d/compare/variants.rs`: the split-impulse solver the
soft step replaced (`split`, with friction on its pseudo velocities or a
decaying correction), Jolt-style position iterations (`ngs`), and the
soft step with any constant changed (`soft/<key>=<value>/...`, e.g.
`soft/hz=30/relax=1` for Rapier's settings). What they measured:
physics.md, "Settling".

Check before trusting a run:

- `uptime`: the machine is shared. Runs of the same case more than about
  10% apart, in the [min–max] column, mean something else was running;
  rerun those cases.
- The quality tables are deterministic (every engine replays exactly), so
  a change in them is a change in behavior, never noise.
- "contacts a body" and "islands" say what a scene is: a real pile is
  about 1.5 and a handful of islands in every engine. Near 1.0 is columns
  (see the lore on the 41-wide pile).

Record what changed in physics.md, "Against other engines", with the date.

## The debug view

Look at a scene before trusting its numbers: a scene that isn't what it
says (a pile standing in columns, a body through a wall, a card house
built wrong) measures as confidently as one that is.

```sh
VIEW="pile 1000 41" VIEW_STEPS=0,100,400 VIEW_OUT=/tmp ./bazel run --config=bench //engine/std/physics2d/compare
VIEW="ratio 1000 5" VIEW_STEPS=0,60,120 VIEW_TEXT=60 ENGINES=arrays,box2d ./bazel run --config=bench //engine/std/physics2d/compare
TURN=1 VIEW="pyramid 20" VIEW_STEPS=600 VIEW_TEXT=100 ./bazel run --config=bench //engine/std/physics2d/compare
```

- `VIEW` is any scene `Scene::parse` reads (`scene.rs`): the settling
  scenes, and the behaviour scenes (`ramp 30 0.2 box`, `bounce 0.5`,
  `ratio 100 1`, `bigonsmall`, `overlap 4 0.25`, `bullet 40 0.25 1 0`,
  `cards 5`, `ladder 30 0.2`, `dominoes 15`), which are always drawn
  turning; `TURN=1` turns the others.
- It writes `<VIEW_OUT>/<scene>.svg` (the directory `bazel run` was run
  from if unset): a row a step of `VIEW_STEPS` (default 0, 60, 300), a
  column an engine of `ENGINES`. Dynamic bodies blue, sleeping ones grey,
  statics dark, a circle's radius line showing its turn; contact points
  red with their normals, hollow orange where held within the speculative
  margin, square where ours keeps no point and the view estimates one.
- `VIEW_TEXT=<columns>` also prints each panel as characters, for reading
  in a terminal or by an agent that can't open an image: statics `#`, each
  body a letter, sleeping `.`, contact points `*` (pressed) and `+` (held)
  where they fall outside a body. Shapes thinner than a character are drawn
  by their outlines. 60-100 columns suit a small scene; a pile wants the
  SVG.
- Ours is drawn from the arrays (`ours (arrays)`) and from the mod
  (`ours (ECS)`, the world's contacts), both; they should be identical.

## The behaviour scenes

```sh
BEHAVE=1 VARIANTS=rapier:ccd ./bazel run --config=bench //engine/std/physics2d/compare > behave.md
SCENES="bullet 50 0.25 1 0.5,ladder 30 0.26" BEHAVE=1 ./bazel run --config=bench //engine/std/physics2d/compare
FAMILIES=all ENGINES=arrays,box2d,rapier ./bazel run --config=bench //engine/std/physics2d/compare   # the edge-of-stability families, short grids
FAMILIES=cards FAMILY_LONG=1 ENGINES=arrays,box2d,rapier ./bazel run --config=bench //engine/std/physics2d/compare   # a long grid
./bazel run --config=bench //engine/std/physics3d/compare:bench -- ramp_hold,ramp_slide,ramp_roll 1 all --rotate --behave
./bazel run --config=bench //engine/std/physics3d/compare:bench -- bounce 25,50,75,100 all --rotate --behave
./bazel run --config=bench //engine/std/physics3d/compare:bench -- ratio 10,100,1000 all --rotate --behave
```

A table a scene of what each engine did (`behave.rs` in each), about a
minute in 2D and seconds in 3D. These are what `:behaviour_test` (2D and
3D) records beside its bounds; after bumping a library or changing a
scene, measure again and set each bound by the rules in physics.md,
"Quality beyond settling". As with the quality tests, a bound ours no
longer meets is a bead and an ignored test.

## The bounce families

```sh
BOUNCES=all BOUNCE_RUNS=1 ./bazel run //engine/std/physics2d/compare > bounces.txt              # short grids, every engine, every run
BOUNCES=drops,rates BOUNCE_LONG=1 ./bazel run //engine/std/physics2d/compare                    # long grids, statistics only
BOUNCES=all ENGINES=closing VARIANTS=arrays:rot/closing=1,arrays:rot/closing=3 ./bazel run //engine/std/physics2d/compare   # options of ours
VIEW="hit corner e=1 v=5 g=20" VIEW_STEPS=0,15,16,22 VIEW_TEXT=60 ENGINES=arrays,box2d ./bazel run //engine/std/physics2d/compare
./bazel run //engine/std/physics3d/compare:bench -- ours,rapier,box3d --bounces --each > bounces3d.txt    # 3D, short grids
./bazel run //engine/std/physics3d/compare:bench -- ours --bounces --long --tune=closing=before           # 3D, long grids, an option
```

Each family's statistics per engine (`bounces ... : excess worst ...`), and
with `BOUNCE_RUNS` (3D `--each`) every run's values, one line a run (`run
<family> <engine> | <scene> | <values>`), which is what to read when a
statistic moves: the families' worsts are one run's (physics-testing.md,
"Families of a law"). A bounce is a scene like any other (`hit <target>
e=.. v=.. g=..`; `Scene::Hit`), so `VIEW` and `SCENES` take it. The mod in
the engine is left out of `BOUNCES`, since it steps at 60 Hz only; the
arrays are it bit for bit. The 2D long grids take about two minutes with
every option, the 3D ones about five. The families' tests record the
references' statistics beside their bounds; after bumping a library,
measure again on exactly the tests' grids.

## The baseline

Our own accepted results, per scene and measure, in four files:
`engine/std/physics2d/compare/baseline.txt` and `baseline_long.txt` (2D),
`engine/std/physics3d/compare/baseline.txt` and `baseline_long.txt` (3D)
(physics-testing.md, "The baseline"). The quality and behaviour tests
check them, from their own runs (the test `baseline` in each, and
`baseline_long` in the long ones), both ways: a value past its band fails,
better or worse. What each records, and with which band, is its
`record.rs`; the format, the comparison and the tool's flags are the test
kit's (`//engine/std/physics_testkit`, `baseline.rs` and `tool.rs`), the
same for both.

- **Trigger:** a baseline test fails; or a change to the solver, the
  narrowphase, the broadphase, sleeping, the step, a scene or a measure,
  which may move values inside the bounds without failing anything.
- **Compare**, before and after a change, and put the table in the
  commit message or the report:

  ```sh
  ./bazel run //engine/std/physics2d/compare:baseline          # 2D default, 1 s
  ./bazel run //engine/std/physics2d/compare:baseline -- --long  # 2D long, about a minute
  ./bazel run //engine/std/physics3d/compare:baseline                    # 3D default, seconds
  ./bazel run //engine/std/physics3d/compare:baseline -- --long            # 3D long, about 90 s
  SOLVER=rot/carry=0 ./bazel run //engine/std/physics2d/compare:baseline   # a variant against the baseline
  ```

  Every value that moved past its band, with old, new, band and which
  way. `--all` lists every value. Fastbuild gives the same numbers (the
  steps are the same arithmetic), at about fifty times the time.
- **Regenerate** when the change is accepted: add `--write` to each of
  the four, in the same commit as the change. The file's diff is the
  record of what the change did; say in the commit why each value that
  moved the worse way is accepted. The long files are regenerated
  whenever the long suites are run for a change (before merging any
  change to the solver, the narrowphase, sleep or the step). Never edit
  a file by hand.
- **Merging** branches that touch physics: after the merge, run the four
  comparisons on the merged tree against each parent's files
  (`git show <parent>:<path> > <path>`, compare, restore), then write
  them on the merged tree. A conflict in a baseline file is never
  resolved by hand. A value that differs from both parents is a finding
  about the combination (the card house of 2026-09-28 was one), and is
  looked at before it's accepted.
- **Bands** are set from measured spread, not guessed. To measure again
  (after adding a family, or when a band fails on a change that
  shouldn't matter): `--all --offset=<n>` runs every pile family at its
  sizes moved by n percent (2D; 2 to 10 either way) or at another seed
  (3D, n = 1-11), under the sizes' names, and the table's moves are the
  spread over neighbouring sizes; then plant a reassociated sum in the
  solver (`a + b + c` as `a + (b + c)`, in the separation, the normal
  speed and the normal impulse), run `--all`, and revert. Set each band
  just outside the larger spread, and record it in physics-testing.md.

## Refreshing the quality tests' bounds

The quality tests (physics.md, "Quality as a test") hold ours to bounds
set from the references' values on the same scenes, recorded beside each
bound in `quality_test.rs` and `quality_long.rs` here and in
`//engine/std/physics3d/compare`. After bumping a library, or changing a scene, measure
them again and set each bound by the rules in physics.md:

```sh
# 2D, the default suite's scenes, and the long ones
ENGINES=arrays,box2d,rapier SETTLE=700 SCENES="pile 400 41,pile 600 41,pile 800 41,pile 1000 41,pile 1200 41,pyramid 15,pyramid 20,pyramid 25,stack 10,stack 20" ./bazel run --config=bench //engine/std/physics2d/compare
ENGINES=arrays,box2d,rapier SETTLE=2500 SCENES="pile 9000 401,pile 10000 401,pile 11000 401,pyramid 100" ./bazel run --config=bench //engine/std/physics2d/compare
# 3D
./bazel run --config=bench //engine/std/physics3d/compare:bench -- boxes,planks 200,300,400,500,1000 all --rotate --runs=1
./bazel run --config=bench //engine/std/physics3d/compare:bench -- boxes 200,300,400,500 all --runs=1
./bazel run --config=bench //engine/std/physics3d/compare:bench -- stack 5,10,15,20 all --rotate --runs=1
./bazel run --config=bench //engine/std/physics3d/compare:bench -- boxes,planks 10000 all --rotate --runs=1
```

The quality numbers are deterministic, so one run each; about 5 minutes
for 2D's small scenes, 15 for its long ones, and 25 for 3D's. A bound
that ours no longer meets is a finding: a bead and an ignored test, never
a looser bound.

## Bumping a library

- **Box2D:** a new release's archive URL, `strip_prefix` and `sha256`
  (`curl -sL <url> | sha256sum`) in `MODULE.bazel`. Then check the shim
  against the new `types.h`: `b2Profile` and `b2Counters` are copied by
  field order (`box2d.rs` asserts their sizes, not their order), and
  `b2Body_SetMassData` must still leave a fixed rotation locked (the
  bench asserts no body turned; see the lore).
- **Rapier:** the version in `engine/std/physics2d/compare/Cargo.toml`, then
  runbook 001 for `Cargo.lock`. The counters' names move between versions
  (see the lore on its broadphase timers), and so do the defaults the
  tables call "defaults": note them.
- Update the versions in docs/CREDITS.md, and the license copy in
  `engine/std/physics2d/compare/licenses/` from the new tag.
