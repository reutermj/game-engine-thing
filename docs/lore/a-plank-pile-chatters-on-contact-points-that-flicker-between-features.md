# A plank pile chatters on contact points that flicker between features

Measured 2026-09-27 in `//bench/physics3d` (the physics3d mod, `-c opt`,
turning planks 1 x 0.25 x 0.5 dropped into a walled box, 1000 steps,
every contact's points, ids and impulses read from the world each step).
Piles of 150-800 planks came to rest under 0.05 m/s in time, but 9 of 14
kept over 1e-8 a body of kinetic energy (Rapier's and Box3D's keep under
1e-11), in bursts every 9-10 steps, and 5 of 14 moved past 0.05 again
after step 600.

It is not rocking of a stable manifold. The loaded points of resting
contacts change from one step to the next and back:

- a vertex of the incident face that lies on a side plane of the reference
  face (planks of one width stack with their edges aligned) is kept by
  Sutherland-Hodgman clipping one step and clipped the next: the same
  point, 0.00088 against 0.00087 deep, under the vertex's id one step and
  the side-and-plane id the next;
- a clipped polygon of more than four points reduced to four picks two
  different sets in turn: a point carrying 0.08 of load at x = -0.49 one
  step, at x = -0.28 the next, as the rock it causes moves which point is
  deepest (the reduction starts from the deepest).

Warm starting still found 1188-1191 of 1191 points a step, and matching by
the nearest last point in place of feature ids changed nothing: what feeds
the rock is the support moving, not impulses lost. Box3D's own reduction
(a first point chosen along a fixed tangent, and a pecking order in which a
later candidate must beat the best by 5%) halved the chattering piles.
What stilled all 28 sizes tried was Box3D's contact recycling: a pair whose
bodies moved less than 0.03 since its manifold was found keeps it, its
points carried with the bodies and their separations updated, so a resting
pair has no new manifold to flicker to (physics.md, "Still at rest").

So a narrowphase that finds each resting pair afresh every step can make a
pile chatter however stable its solver is, and warm-starting statistics
won't show it: look at whether the points move.
