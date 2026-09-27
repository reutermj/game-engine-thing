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
