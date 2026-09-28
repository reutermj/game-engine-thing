# A pile family's median rest moves by a hundred steps, and its worst by anything

Measured 2026-09-28 setting the baseline's bands (physics-testing.md, "The
baseline"): every pile family (one pile at 5 to 10 sizes) run at its sizes
moved 2, 3, 5 and 10% either way (`:baseline -- --all --offset=<percent>`),
and with three sums in the solver reassociated (`a + b + c` as
`a + (b + c)`), each against the unchanged run:

| statistic over the family | moved by |
|---|---|
| median at rest from | up to 90 steps (80 on the default 400-1200 turning family) |
| median first look at rest | up to 120 |
| the same over three sizes | up to 240 |
| worst at rest from | 240 → 390, 400 → 1050, never → 1380, 260 → never |
| worst energy at the end | a factor of 9000 (one pile still moving) |
| median depth, median energy where it rests | 41%; 1.04 |
| a pyramid's or a stack's rest, top, depth | 0 steps; 0.2% |

Surprising twice. The lore's "rest moves by hundreds" was about one pile,
and a median over five was expected to be steady; over neighbouring
sizes it isn't, and a first measurement with offsets of 10-50 bodies
alone (at most 30 steps) set bands the 10% offsets broke. And a family's
worst is a single pile's value, which rounding moves from resting to
never: no band holds it. Where piles move again after resting (the big
turning piles in every engine, and ours 81 wide, get-emj.63) even the
median jumps by 250 or to never.

So: record medians, never worsts, of chaotic families; measure spread at
neighbouring sizes of a few percent, not a few bodies; and read a change
of a family's median rest under about 100 steps as noise. Stand scenes
(pyramids, stacks) are the opposite: nothing moved, and their bands can be
tight.
