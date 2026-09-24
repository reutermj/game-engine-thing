# An assert in a hot accessor can cost a quarter of the loop

`near_pairs` (`engine/ecs/query.rs`) calls a row's grown box once per row
it tests, inside loops over bit masks. When the row boxes moved into
`Lanes` (`engine/ecs/spatial.rs`), the accessor it called first was `get`,
which asserts `i < len` before indexing arrays that are already
bounds-checked against their fixed length. That alone took the pair loop
at 10 000 bodies from 178 to 232 µs, and the self-pairs part from 64 to 87
(measured with `Instant`, `-c opt`, the physics pile, several runs each).
An unchecked twin (`grown`, `#[inline(always)]`, indexing the arrays
directly) brought it back.

The cause is inferred, not read in the assembly: the assert's panic path
(formatting its message) sits in the middle of the loop, which plausibly
stops LLVM keeping the lanes in registers or hoisting loads around it.

## Resolution

Keep checked accessors for the cold paths (the re-sort, `check`) and an
unchecked one for inner loops whose indices come from a mask of real rows.
When a refactor makes a hot loop slower for no visible reason, look for a
new panic path in it before anything else.
