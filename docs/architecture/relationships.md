# Relationships

**Status: decided and built** (2026-09-24). Contacts and overlaps are
entities that physics keeps in pair order, in ordered tables
(`engine/ecs/ordered.rs`), and links between entities are plain entity
fields, such as `ChildOf { parent }`, kept in parent order. Spiked in
`spike/relations` (get-emj.18) and built in get-emj.20. Compound colliders
(get-emj.8) are still to build.

How entities refer to each other: a contact to its two bodies, a collider
to its body, a child to its parent. The [physics
retrospective](../retrospectives/2026-09-23-physics.md#options-for-the-model-underneath)
lists six options; this doc measures the plausible ones and records what
was chosen.

The question splits in two, with opposite data profiles:

- **Contacts** churn: hundreds a step, rebuilt every step, read by the
  solver in bulk and by game hooks between finding them and solving them.
- **Structural links** (colliders, hierarchy) barely change, and are read
  every frame by whatever propagates along them.

## The decision

- **Contacts are entities.** Each is a `ContactPair { a, b }` with its
  `Manifold`, `Response` and `Impulse`, and each sensor overlap an
  `Overlap { a, b }` ([physics.md](physics.md#the-step)). Both are ordered
  keys, so contacts are stored in pair order. The step brings them in line
  with what it found in one pass, and the solver reads them in order with
  no sort. A pre-solve hook is a system ordered between `find_contacts`
  and `solve` that queries contacts like any other entities.
- **Links are entity fields.** `ChildOf { parent }` is in `engine_ecs`, an
  ordered key, so a parent's children are one range of rows
  (`in_keys::<ChildOf>(children_of(p))`). There are no engine-kept
  back-references until something needs to walk down a hierarchy.
- **Colliders will be compound data on the body** (option 5, not built).

Contacts in a physics-owned table (option 4) measured best in the spike,
and it was the first recommendation.[^table] It sidestepped the ECS: its
rows were invisible to queries, it was one footprint for every hook, and it
was a second data model beside the world's. The work since was closing its
performance lead with ECS storage (below), and it's mostly closed.

## Ordered tables

A component declared `order = key`, implementing `OrderKey` (a `u128` per
value, from glue compiled with the component like spatial bounds), keeps
the tables that hold it sorted by that key, then by entity. The order is
kept like [spatial order](spatial-storage.md):
- anything that moves a row into or out of the table, or writes its key,
  marks it;
- it's re-sorted when the `Structural` drops;
- a system never sees its own rows move, and the systems after it do.

A re-sort keys the rows new to the table or whose key was written since
the last (by the values' ticks, as a spatial re-sort re-bounds only what
was written), or every row when a newer build's glue installed the key,
since it may key the same values otherwise; then it stops if the rows are
in order.[^keyed] Otherwise it sorts them (stable and adaptive: a few
appended rows and a few swapped by removals are close to a merge) and moves
every column into pages in that order (`ErasedColumn::gather`).

- `Query::for_each_ordered` walks in key order across all the query's
  ordered tables: one table walks as it is, several are merged.
- `Query::in_keys::<K>(range)` seeks in tables kept in `K`'s order. Tables
  that hold `K` but are kept in another order are scanned, and come out in
  their own order. The query must read `K`.
- **One order per table**, like a database's clustered index. A table
  holding a spatial key is in spatial order, and one holding two ordered
  keys is in the older one's. The spawner's bodies are `ChildOf` its box
  and have a `Position`, so a range of them is a scan. Correct, and O(n).

## Closing the gap

The spike's contact numbers at 1000 bodies, µs per frame, `-c opt`, one
thread (`spike/relations`'s bench[^removed]):

| contacts as | upkeep, settled | upkeep, full churn | solve, settled |
|---|---|---|---|
| a table (option 4) | 4 | 4 | 120 |
| entities, as first spiked | 18 | 546 | 141 |
| entities, cheaper structural changes | 18 | 184 | 141 |
| entities in ordered tables | 11 | 184 | 127 |

Full churn means all 1000 contacts end and 1000 begin every frame. A pile
barely churns (15 contacts begin a frame while falling), but fast bodies,
bullets and particles do.

**Cheaper structural changes**, in `engine_ecs`, for every game's spawns
and despawns (the numbers measured each fix alone):
- `Structural` keeps its locked tables in a vector by table id. A SipHash
  per lookup, three or four per spawn, had been a fifth of a spawn's cost.
  Despawn went from 53 to 21 ns.
- The exact footprint an apply node locks before it runs had been
  allocating, sorting and cloning component lists per spawn and hashing
  each entity with SipHash. Now a spawner's spawns share one list
  (`Arc`), the table set is resolved once per list, despawns only add
  their table's shape, and the per-entity map uses a cheap hash. This was
  most of the win: 373 → 182 µs of full churn.
- Spawns write each column in place, place components by position, not
  by comparing names, and reuse the last spawn's table. Freed ids return
  to the free list in one lock.

A spawn and despawn is now about 90 ns per contact, against 270. That's
the floor with a structural change per contact.

**Ordered tables** then removed the lookups: the merge updates each
persisting contact in the pass, not through `Query::with` by an index, and
the solver reads contacts in order instead of sorting them.

**In the physics mod**, the 1000-body pile's settled frame went from 0.348
to 0.38 ms (`./bazel run -c opt //engine/std/physics:bench`), on a
simulation that comes out the same bit for bit, as do both games'
replays:
- `find_contacts` is a little faster (172–182 → 165 µs), since its merge
  no longer binary-searches last step's list.
- `solve` is slower: gathering contacts 36 → 45 µs and writing them back
  11 → 19 µs, reading them from columns instead of one `Vec` of structs.

That 9% is the price of contacts in the world.

## Ergonomics

The one-way platform, with contacts in the spike's table and as entities:

```rust
// Table
fn one_way(_: &mut Cx, mut contacts: Contacts<&Vel, With<Player>>, mut platforms: Query<(), With<OneWay>>) {
    contacts.for_each(|c, _, v| {
        if has(&mut platforms, c.other()) && (c.normal().y < 0.7 || v.y < 0.0) {
            c.disable();
        }
    });
}

// Entities
fn one_way(_: &mut Cx, mut contacts: Query<(&ContactOf, &Manifold, &mut Response)>,
           mut platforms: Query<(), With<OneWay>>, mut players: Query<&Vel, With<Player>>) {
    contacts.for_each(|_, (pair, m, mut r)| {
        let n = Vec2::new(m.nx, m.ny);
        let (player, normal) = if has(&mut platforms, pair.b) { (pair.a, n) }
            else if has(&mut platforms, pair.a) { (pair.b, -n) } else { return };
        let Some(vy) = players.with(player, |_, v| v.y) else { return };
        if normal.y < 0.7 || vy < 0.0 { r.disabled = true; }
    });
}
```

- **Which side am I is the hard part.** The table answered it once, in its
  parameter. Every entity-model hook works out which end of the pair is its
  entity and flips the normal. The spike's tests at first always had the
  player as side `a`, and three of five mutations to side handling
  survived. Physics now offers `ContactPair::seen_from(me)` and
  `Overlap::other(me)`, tested once, but each hook still has to call them.
- **Entities get the whole query language.** A game can put its own
  components on a contact or filter contacts by them. Filtering by the
  *participants'* components still takes a lookup per contact.
- **The ECS's structural costs shape the idiom.** "Disable this contact"
  wants a `Disabled` marker, but a marker is a structural change each time
  a hook adds one, so it's a field in `Response`.
- **Facts are as of when they were found.** An overlap read a step late
  had the player killed twice, once before its respawn and once after. So
  the walkers' `meet` runs right after `find_contacts`
  ([physics.md](physics.md#the-step)): order a reader after its finder.

## Links

| hierarchy, 1000 parents × 8 children | µs |
|---|---|
| no parent read (the loop alone) | 15 |
| parents copied into a vector by entity index first | 33 |
| `ChildOf { parent }`, a `Query::with` per child | 80 |
| `Children { list }` on the parent, a lookup per child | 88 (95 shuffled) |
| `ChildOf`, siblings stored together, last parent kept | 86 (133 shuffled) |

| colliders, 1000 bodies × 2 | µs |
|---|---|
| a `Vec` of parts on the body | 3 |
| an entity per collider, looking up its body | 20 |

| one query over 8000 rows, spread over | 1 | 16 | 64 | 256 tables |
|---|---|---|---|---|
| µs | 16.7 | 17.1 | 19.9 | 31.2 |

1. **Links are paid for by lookups, not layout.** A `Query::with` costs
   about 6 ns: propagation through it is 80 µs, against 33 with parents in
   a vector by entity index. Neither a children list nor storing siblings
   together helps. Keeping the last parent should have cut lookups from
   8000 to 1000 (counted), but it isn't faster, and without a profiler I
   can't say why.
2. **Tables per target are out.** Spreading rows over 256 tables nearly
   doubles an unrelated query over them, and a table per parent (1000
   tables of 8) would be worse. That rules out relationship pairs stored
   in archetypes (option 1).
3. **Compound colliders are data.** Parts on the body are 7× cheaper than
   collider entities, and fit spatial storage's one extent per key: a
   body's box is the union of its parts.

Mod state held entity lists that are now in the world: the platformer's
level is the entity with its `LevelInfo`, and what it builds is `ChildOf`
it. The demo spawner's box works the same way, and pong's goal lines are
`Goal { side }`. Rebuilding a level, or a reload deciding whether to
spawn, asks the world.

## Open questions

- **One entity's contacts.** A hook for the one player visits every
  contact (18 µs at 1000 contacts). `in_keys(pairs_from(e))` finds only the
  contacts where `e` is `a`. Finding the `b` side too needs a second order
  or an adjacency list.
- **`Query::with` costs 6 ns** a call, and reads per call what the query
  could decide once.
- **Churn costs a structural change per contact**, about 90 ns. The table
  paid none.
- **A reload that changes an ordered key's glue** doesn't re-sort its
  tables until each next changes.
- **The walkers' `meet` checks both ends of an overlap**, but in the
  replays the walker is always `a`, so the other orientation is untested.

**Found on the way:** restitution is lost on speculative contacts
(get-emj.19). The solver slows a landing body to gap/dt the step before
it touches, and bounces it from that speed, so a ball landing at 10 with
restitution 1 comes back at 3.

[^table]: 2026-09-24. The spike's recommendation was contacts in a
    physics-owned table (`ContactTable`, one component holding every
    contact) with a `Contacts<Data, Filter>` parameter showing each hook the
    contact from its side. It measured best (upkeep 4 µs, even under full
    churn). It was set aside as outside the ECS.

[^removed]: 2026-09-25. `spike/relations` was removed once the physics mod
    ran on ordered tables and `//engine/std/physics:tax` measured it against
    arrays; the table model and its bench are in git history up to 5377205.

[^keyed]: *(History, 2026-09-24, get-emj.26.)* A re-sort keyed every row
    afresh, a glue call each, since keys are cheap: a dirty re-sort of 4800
    contacts took 17 µs, 6 keying only what changed. It's little in the
    physics pile (the contacts are re-sorted about a third of the steps it
    falls), and more in any ordered table that's large and mostly still.
