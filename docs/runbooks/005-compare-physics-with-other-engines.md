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
```

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
