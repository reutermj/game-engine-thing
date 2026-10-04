# A copied Bazel binary still loads its runfiles from bazel-out

Copying a built binary with its `.runfiles` directory (`cp -rL`) to keep a
build to run later doesn't freeze what it loads. The runfiles directory
holds a `MANIFEST` of absolute paths into `bazel-out`, and the runfiles
library resolves through it, so a mod or manifest the binary looks up at
run time comes from whatever the last build left there, not from the
copy, even with `RUNFILES_DIR` pointed at the copy. (The effect was
measured, below; that the `MANIFEST` is why is inferred, not traced.)

Found 2026-09-27 benchmarking the 2D solver before and after a change
(`//engine/std/physics/compare`): the copy built before timed its
arrays, compiled into the binary, at the old speed (8002 µs of solver on
a turning pile of 10 000) but the physics mod in the engine at the new
one (4414), since the new build had overwritten the mod in `bazel-out`.
The engine loads mods by the scene game's manifest, which it finds in
runfiles.

## What it means

- For a before-and-after that loads anything at run time (a mod, a
  game's manifest), run each from a fresh build, one after the other,
  rather than keeping copies. Or remove the copy's `MANIFEST` so the
  library resolves inside the directory: measured 2026-10-03
  (get-emj.101): a copy of physics3d's `step_bench` with timers put in,
  kept with its `MANIFEST`, printed none of them and a narrowphase of
  281 µs at 8 threads on 10 000 settled boxes, the split build then in
  `bazel-out` (the timed build's was serial, about 1800); the copy of the
  tree before, with its `MANIFEST` removed, ran its own serial
  narrowphase (295 µs on the planks at 8 threads) while `bazel-out` held
  the split one. Each copy's `libphysics3d_mod.so` checksum differs. A path the tool takes from
  the environment under `bazel run` then has to be given by hand:
  physics2d's baseline needs
  `SCENE_GAME=_main/engine/std/physics2d/compare/scene_game.manifest`
  (the `.manifest`, not the game's launcher), step_bench
  `PILE3D=_main/engine/std/physics3d/pile3d_game.manifest`.
- A bench whose parts disagree with each other (the arrays slow, the
  mod fast, where they are the same code) is the sign.
