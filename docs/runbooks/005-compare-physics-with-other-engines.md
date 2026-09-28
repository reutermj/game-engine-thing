# Runbook: compare the physics with Box2D and Rapier

- **Trigger:** a change to `//engine/std/physics`'s step (the solver, the
  narrowphase, the broadphase, the storage it walks) that claims to make it
  faster or better behaved; or bumping Box2D or Rapier.

## Gap

`:tax` compares the mod with our own step on arrays, bit for bit, so it
sees the ECS's cost and nothing about the algorithms. Whether a change
closes a gap to a tuned engine takes the same scenes in those engines,
measured the same way, and quality beside time, since each solver trades
one for the other differently.

## Resolution

```sh
./bazel run -c opt //engine/std/physics/compare > compare.md
```

About 25 minutes with `LONG=1`, 12 without, on this machine. It prints,
per case, a table of µs per step (the median of `REPS` runs, 3 by
default, with the least and most) by stage, one of how well each engine
settled, each engine's own stages, and whether the arrays agreed with the
mod bit for bit (they must, but for rain). Narrower runs:

```sh
ONLY="pile 10000" REPS=5 ./bazel run -c opt //engine/std/physics/compare
ENGINES=ecs,box2d VARIANTS=box2d:2,rapier:8 ./bazel run -c opt //engine/std/physics/compare
SLEEP=1 ./bazel run -c opt //engine/std/physics/compare   # each engine's default sleeping
SETTLE=1500 ./bazel run -c opt //engine/std/physics/compare  # how soon each comes to rest
VARIANTS=arrays:split,arrays:soft/sub=4 ./bazel run -c opt //engine/std/physics/compare
TURN=1 ./bazel run -c opt //engine/std/physics/compare   # only bodies that turn
TURN=1 SETTLE=1500 ENGINES=box2d,rapier,rot VARIANTS=arrays:rot,arrays:rot/warm=0 ./bazel run -c opt //engine/std/physics/compare
./bazel run -c opt //engine/std/physics/compare:solver_bench   # the 2D solver alone, each way of solving
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
`engine/std/physics/compare/variants.rs`: the split-impulse solver the
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
VIEW="pile 1000 41" VIEW_STEPS=0,100,400 VIEW_OUT=/tmp ./bazel run -c opt //engine/std/physics/compare
VIEW="ratio 1000 5" VIEW_STEPS=0,60,120 VIEW_TEXT=60 ENGINES=arrays,box2d ./bazel run -c opt //engine/std/physics/compare
TURN=1 VIEW="pyramid 20" VIEW_STEPS=600 VIEW_TEXT=100 ./bazel run -c opt //engine/std/physics/compare
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
BEHAVE=1 VARIANTS=rapier:ccd ./bazel run -c opt //engine/std/physics/compare > behave.md
SCENES="bullet 50 0.25 1 0.5,ladder 30 0.26" BEHAVE=1 ./bazel run -c opt //engine/std/physics/compare
./bazel run -c opt //bench/physics3d:bench -- ramp_hold,ramp_slide,ramp_roll 1 all --rotate --behave
./bazel run -c opt //bench/physics3d:bench -- bounce 25,50,75,100 all --rotate --behave
./bazel run -c opt //bench/physics3d:bench -- ratio 10,100,1000 all --rotate --behave
```

A table a scene of what each engine did (`behave.rs` in each), about a
minute in 2D and seconds in 3D. These are what `:behaviour_test` (2D and
3D) records beside its bounds; after bumping a library or changing a
scene, measure again and set each bound by the rules in physics.md,
"Quality beyond settling". As with the quality tests, a bound ours no
longer meets is a bead and an ignored test.

## Refreshing the quality tests' bounds

The quality tests (physics.md, "Quality as a test") hold ours to bounds
set from the references' values on the same scenes, recorded beside each
bound in `quality_test.rs` and `quality_long.rs` here and in
`//bench/physics3d`. After bumping a library, or changing a scene, measure
them again and set each bound by the rules in physics.md:

```sh
# 2D, the default suite's scenes, and the long ones
ENGINES=arrays,box2d,rapier SETTLE=700 SCENES="pile 400 41,pile 600 41,pile 800 41,pile 1000 41,pile 1200 41,pyramid 15,pyramid 20,pyramid 25,stack 10,stack 20" ./bazel run -c opt //engine/std/physics/compare
ENGINES=arrays,box2d,rapier SETTLE=2500 SCENES="pile 9000 401,pile 10000 401,pile 11000 401,pyramid 100" ./bazel run -c opt //engine/std/physics/compare
# 3D
./bazel run -c opt //bench/physics3d:bench -- boxes,planks 200,300,400,500,1000 all --rotate --runs=1
./bazel run -c opt //bench/physics3d:bench -- boxes 200,300,400,500 all --runs=1
./bazel run -c opt //bench/physics3d:bench -- stack 5,10,15,20 all --rotate --runs=1
./bazel run -c opt //bench/physics3d:bench -- boxes,planks 10000 all --rotate --runs=1
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
- **Rapier:** the version in `engine/std/physics/compare/Cargo.toml`, then
  runbook 001 for `Cargo.lock`. The counters' names move between versions
  (see the lore on its broadphase timers), and so do the defaults the
  tables call "defaults": note them.
- Update the versions in docs/CREDITS.md, and the license copy in
  `engine/std/physics/compare/licenses/` from the new tag.
