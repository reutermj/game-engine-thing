# A closure called from two walks may not be inlined into either

Measured 2026-10-03 with `step_bench` (`--config=bench`, one thread, the
median of 3 runs, the 2D settled pile of 10 000), µs a step for
`gather_contacts`, which builds a `Constraint` (and the contact's
`Points`) for each of about 15 000 contacts:

| the row's code | µs |
|---|---|
| a closure, in the one walk (70a4093) | 265-268 |
| the same closure, called from the one-thread walk and from the walk across threads | 294 |
| the same body as an `#[inline(always)]` function, from both | 250-262 |

The runs were minutes apart, alternated with 70a4093's. When the
one-thread path walked through `par_for_each_ordered_page` too (one
chunk), with the row's closure called from the chunk's closure, it was
340-406 in some sessions and 279-287 in others; the inlined function
ended that.

The same with a per-row closure pushing through a chunk's struct of
lists (`gather_turning`, 2D falling pile): 87-89 µs against 69; walked by
page instead, each page taking the lists out of the chunk into locals,
73.

Why, inferred rather than read from the assembly: each walk is a
different instantiation (`for_each_ordered_page` and `par_pages`, each
generic over the closure it's handed), so a closure both call has two
call sites, and LLVM's inliner weighs a body with two callers as twice
the code; the row's closure is big enough (the points' loop) to stay a
call. A call per row costs its arguments and a 44-byte return through
memory, and keeps the lists' lengths and pointers from staying in
registers.

## What it means

Where a system has a one-thread walk and a walk across threads sharing
the row's code, put the row's code in an `#[inline(always)]` function
(attributes on closures aren't stable), or give it one call site: a
page's closure that takes the chunk's lists into locals for the page.
Then measure one thread against the tree before: the split's cost there
is easy to miss when the gain at 8 threads is large.
