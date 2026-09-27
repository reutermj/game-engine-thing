# A query of a sparse component alone has no length

`Query::len` and `Query::is_empty` count the rows of the tables the query
matches: the length of a page walk. A query whose terms are all sparse,
with no table filter (`Query<&Still>`), matches no table at all, since
`for_each` walks the sparse set instead, so its `len` is always 0 and
`is_empty` always true, however many entities have the component. With a
table filter, `len` counts the filtered tables' rows, not the set's
members.

Found 2026-09-26 (get-emj.40): physics's `solve` cleared every `Still`
when sleeping was turned off, behind `if !stills.is_empty()`. The branch
never ran: turned off mid-settle, 62 bodies of 200 kept their `Still`
(`turning_sleeping_off_wakes_everything`, which caught it once it looked
at `Still`; a mutation that emptied the branch had survived, since it
was dead already). Walking unconditionally fixed it, and costs nothing
when the set is empty: `for_each` on a sparse-driven query walks the set.

## What it means

- For a sparse component, ask the set: walk it (`for_each`), or count in
  the walk. Don't gate a walk on `len` or `is_empty`.
- A check that "can't be wrong, it's only a shortcut" is a branch nothing
  tests: here the shortcut was the whole behavior.
