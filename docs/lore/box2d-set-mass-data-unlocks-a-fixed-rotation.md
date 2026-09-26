# Box2D's SetMassData unlocks a fixed rotation

Found 2026-09-25 building `//engine/std/physics/compare`, where every Box2D
body has `b2BodyDef::fixedRotation` set and mass 1 whatever its shape.

Setting the mass with `b2Body_SetMassData(body, (b2MassData){1, {0, 0}, I})`
after `b2CreateBody` sets the inverse inertia from `I` and nothing else:

```c
// body.c, v3.1.1, b2Body_SetMassData
bodySim->invInertia = body->inertia > 0.0f ? 1.0f / body->inertia : 0.0f;
```

`fixedRotation` is only honored where Box2D computes the mass itself
(`b2UpdateBodyMassData`, `body.c` around line 592). So with `I = 1` a
"fixed rotation" body turns, and the comparison's 1000-body pile came out
with Box2D sinking 0.27 into itself, 930 overlaps over 0.01 and 1.59
contacts a body, against 0.035, 558 and 1.01 with rotation really locked:
a different scene, silently.

**Measured**: with `I = 1`, the bench's check that no body has turned
(`quality.rs`) failed for all 1000 bodies; with `I = 0` none turns.

**Resolution**: pass a rotational inertia of 0 (`box2d_shim.c`), or call
`b2Body_SetFixedRotation` after setting the mass. And check the angle of
every body you expect not to turn: nothing else fails.
