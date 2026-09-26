# Clamped edge closest points make contacts between edges that don't touch

Measured 2026-09-26 in `//engine/std/physics3d` (box against box, the
separating axis test and clipping) and `//bench/physics3d` (`-c opt`, ours,
turning cubes dropped into a walled box).

When the separating axis test picks an edge pair, the contact point is
halfway between the closest points of the two edges. The usual code for
that is segment-to-segment distance: the closest points of the two lines,
each clamped to its segment. Clamped, a pair whose lines meet off the ends
of an edge still gets a point, on the edges' ends, with the depth the axis
gave; the two points can be most of a box apart, and the point halfway
between them is in neither box. Box3D rejects such a pair instead:
its edge contact takes the lines' closest points (`b3LineDistance`) and
gives up unless `b3IsWithinSegments` (`convex_manifold.c`).

| turning box pile, settled at step | 900 | 1000 | 1100 |
|---|---|---|---|
| closest points off the edges rejected | 272 | 180 | 198 |
| clamped onto the edges | never | 902 | never |

Box3D's other guard on edge pairs, the Gauss map test (only edges whose
arcs cross, so that they make a face of the Minkowski difference), changed
nothing measurable on these scenes with the rejection in place, nor
without it. The pile wobbles rather than explodes: the phantom contacts are
shallow and come and go. `narrow.rs`'s `every_point_is_where_both_boxes_are`
drops 4000 random pairs of turned cubes and checks every point is within
the margin of both boxes; it fails with the clamp.
