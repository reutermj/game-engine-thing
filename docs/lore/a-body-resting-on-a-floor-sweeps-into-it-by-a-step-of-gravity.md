# A body resting on a floor sweeps into it by a step of gravity

Measured 2026-10-04 with 2D's swept contacts (get-lye,
`narrow::collide_moving`), in `physics2d_test`'s
`running_across_a_floor_of_tiles_never_snags_on_a_seam` (a box 0.8 wide
running at 7 over unit tiles, gravity 40).

A contact is looked for with the velocities as the step finds them, and
by then the step's gravity is in them: a body resting on a floor has a
velocity of g dt into it (0.67 a second at gravity 40 and 60 Hz) that the
floor's contact takes back out during the solve. Swept over the step, that
velocity carries the body 0.011 into the floor, so across a seam it meets
the next tile's side as if it overlapped it by 0.016 rather than the
0.005 it rests sunk. That was past `narrow::FLUSH` (0.01), the overlap
under which `box_box`'s seam rule treats a face as one the body slides
along, so the sweep made a contact on the tile's side and the runner
stopped dead at x 11.6.

So a sweep can't judge "flush" only where the bodies would meet: the
motion it extrapolates includes what a resting contact is about to
cancel. `swept_boxes` leaves a pair to the seam rule if it is flush across
the face now, as well as if it would be when they meet (the second for a
body falling flush down a wall of tiles). Planted back to the meeting
alone, both the runner and `narrow`'s
`boxes_meet_on_the_face_they_cross_and_not_on_a_corner_they_slide_past_flush`
fail.
