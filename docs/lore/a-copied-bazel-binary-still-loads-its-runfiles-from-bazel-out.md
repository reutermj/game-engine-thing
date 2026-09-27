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
  library resolves inside the directory (not measured here).
- A bench whose parts disagree with each other (the arrays slow, the
  mod fast, where they are the same code) is the sign.
