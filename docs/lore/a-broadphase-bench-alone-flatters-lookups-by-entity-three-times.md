# A broadphase bench alone flatters lookups by entity three times

Measured 2026-09-27, with `KeptStats`' timings (three clock reads a call),
in `//bench/physics3d`'s 10 000 boxes settled and in
`//engine/ecs:kept_bench`'s lattice of 10 000 creeping, 3D, one thread.

The kept broadphase (`engine/ecs/kept.rs`) keeps a record per row by entity
index, and each step walks the changed pages' rows into their records and
tests each candidate pair's two records. Rows on a page are neighbors in
space, not in entity index, so both are loads at random into about 640 KB.

On the bench's lattice, which runs nothing but the broadphase, the walk
took 5.8 ns a row. In the physics step, on the same number of bodies, it
took 16 to 19. With the bench made to write 64 MB between frames
(`POLLUTE=1`), as the rest of a step does to the caches, 10.6. The
candidate tests went from 1.7 to 5 ns each the same way. So the records
are in the level-3 cache or memory by the next step, whatever the bench
says; a microbenchmark that runs one stage in a loop measures it with warm
caches, which the stage never has in a game.

What didn't help (in the step): touching a page's 16 records first, in a
loop of independent loads so their misses overlap (19 to 16 ns a row), and
keeping the fat box in the record rather than beside it (one line a row,
not two; within the noise). What did: doing less of it, by retesting only
the candidates of the few rows that changed when few did (a pile at rest:
60 µs to 2).

Measure a stage's per-item cost inside the step it runs in (the physics
benches' stage timings) before tuning it on a bench of its own.
