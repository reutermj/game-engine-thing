# A reference engine's step to rest is as chaotic as ours

Measured 2026-09-26, setting the quality tests' bounds (physics.md,
"Quality as a test") from the comparisons, each engine at its defaults,
one run a size, each size its own drop:

| scene | engine | at rest from, by size |
|---|---|---|
| 3D turning planks, 200 / 300 / 400 / 500 / 1000 | Box3D 0.1 | **823** / 209 / 254 / 302 / 262 |
| 3D turning cubes, the same sizes | Jolt 5.6 | 240 / **835** / 274 / never / 502 |
| 2D turning pile, 400 / 600 / 800 / 1000 / 1200 | Rapier 0.36 | 160 / **400** / 250 / 230 / 310 |
| 2D turning pile, 9000 / 10 000 / 11 000 | Box2D v3.1.1 | 750 / 1160 / **280** |

The lore entry on our own piles (a-piles-step-to-rest-moves-by-hundreds-with-rounding-alone.md)
found the same of ours: which body tips which way is in the last bits, and
a pile that tipped differently rests at another step. The shipped engines
are no different: one size in four or five lands three times later (or
sooner) than its neighbours, in every engine that was run at several.

So a bound taken from a reference's single run, or its worst, is set by
that run's luck: Box3D's 823 would have allowed ours four times its typical
rest on planks. The tests take each reference's **median** over the sizes,
and bound ours on its own worst and median against that, so neither side's
outlier sets the bound. Run several sizes of every engine before reading a
difference in rest steps into one scene, references included.
