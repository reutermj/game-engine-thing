# Jolt damps every body by default

Found 2026-09-28 running the 3D behaviour scenes (physics.md, "Quality
beyond settling"): a unit cube sliding down a ramp at 30°, friction 0.2,
should accelerate at g (sin θ − μ cos θ) = 3.2059. Rapier, Box3D and ours
meet it to six digits; Jolt 5.6 gave 3.0460, and a rolling sphere 3.3288
against 3.5036, both 5.0% slow.

`BodyCreationSettings` gives every body damping unless told otherwise:

```cpp
// Jolt/Physics/Body/BodyCreationSettings.h, 5.6
float mLinearDamping = 0.05f;  ///< Linear damping: dv/dt = -c * v.
float mAngularDamping = 0.05f; ///< Angular damping: dw/dt = -c * w.
```

**Measured**: over the steps the acceleration is taken from (30 to 90)
the cube averages about 3.2 a second, and 0.05 of that is 0.16, the whole
gap (3.2059 − 3.0460 = 0.160). Jolt's lossless bounce keeps 0.53 of its
height after 11 bounces where Rapier's keeps 0.96, the same drag.

**Resolution**: none in the bench, which runs each engine at its defaults
and so leaves Jolt damped; its behaviour numbers are recorded beside the
bounds, not used for them. Anything comparing Jolt on motion (not rest)
should set both to 0 or say it didn't.
