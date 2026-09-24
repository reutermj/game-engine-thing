# Relationships

**Status: spiked** (`spike/relations`, get-emj.18); the recommendation
below awaits a decision. It blocks get-emj.8 (several colliders per body)
and get-emj.10 (contact response).

How entities refer to each other: a contact to its two bodies, a collider
to its body, a child to its parent. The [physics
retrospective](../retrospectives/2026-09-23-physics.md#options-for-the-model-underneath)
lists six options; this doc measures the ones that are plausible here and
recommends a combination.

The question splits in two, with opposite data profiles:

- **Contacts** churn: hundreds a step, rebuilt every step, read by the
  solver in bulk and by game hooks between finding them and solving them.
- **Structural links** (colliders, hierarchy) barely change, and are read
  every frame by whatever propagates along them.

## The spike

One pile of bodies (`common.rs`: the physics mod's narrowphase and solver,
by path, and a grid broadphase), with contacts stored two ways:

- **A table** (`table.rs`, option 4): one `ContactTable` component owned
  by physics, rebuilt each step and warm-started by pair, and a
  `Contacts<Data, Filter>` parameter that shows a hook each contact from
  the side its query matches, with that side's components.
- **Entities** (`entities.rs`, option 3): each contact an entity with
  `ContactOf { a, b }`, `Manifold`, `Impulse` and `Response`, updated in
  place while it lasts, despawned when it ends, spawned when it begins,
  with an index by pair.

Game code against each (`cases.rs`, tested in both models, with the
player on either side of the contact): a one-way platform, "what am I
standing on", and a bounce pad. Structural links (`links.rs`): a hierarchy
propagated four ways, a body's colliders two ways, and one component's
rows fragmented over many tables. `./bazel run -c opt
//spike/relations:bench`.

## Ergonomics

The one-way platform, in each model:

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

- **Which side am I is the hard part**, and the table answers it once.
  Every entity-model hook works out which end of the pair is its entity
  and flips the normal to match. The tests at first spawned the player
  before what it touched, so it was always side `a`, and three of five
  mutations to side handling survived: two in entity hooks, one in the
  table's `Seen`. In the table model, getting that right is one function
  tested once. In the entity model, every hook has to get it right again.
- **Entities get the whole query language.** A game can put its own
  components on a contact or filter contacts by them. Neither model can
  filter by the *participants'* components without a lookup per contact.
- **The ECS's structural costs shape the entity idiom.** "Disable this
  contact" wants a `Disabled` marker, but a marker is a structural change
  each time a hook adds one, so it's a field instead, which is the
  table's shape anyway.
- **Neither finds one entity's contacts cheaply.** A hook for the one
  player visits every contact (12 to 19 µs at 1000 contacts). The table's
  parameter can fix that inside physics, with an adjacency list per
  entity driven from the query's side. The entity model needs a second
  index.

## Performance

Per frame, µs, `-c opt`, one thread, 1000-body pile, 60 frames (detection
is the spike's own grid, the same for both models, and not the subject):

| model | scene | contacts | began/frame | upkeep | hook: all | hook: one | solve |
|---|---|---|---|---|---|---|---|
| table | falling | 332 | 15 | 2 | 4 | 4 | 64 |
| entities | falling | 332 | 15 | 13 | 3 | 7 | 71 |
| table | settled | 1000 | 0 | 4 | 9 | 12 | 121 |
| entities | settled | 1000 | 0 | 18 | 7 | 19 | 140 |

With full churn (1000 contacts end and 1000 begin every frame), upkeep is
**4 µs for the table and 545 µs for entities**, about 270 ns per contact
spawned and despawned through the logs. A pile barely churns. Fast bodies,
bullets and particles do. The solver's gather costs entities 7 to 19 µs
more: sorting into pair order, since storage order depends on history,
and writing impulses back by lookup.

Structural links:

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

**What the numbers say:**

1. **Contacts belong in a table.** The table beats entities on upkeep
   (4 to 5×, and 100× under churn) and on the solver's gather, and is even
   on hooks.
2. **Links are paid for by lookups, not layout.** A `Query::with` costs
   about 6 ns: propagation through it is 80 µs, against 33 with parents in
   a vector by entity index. Neither a children list nor storing siblings
   together helps. Keeping the last parent should have cut lookups from
   8000 to 1000 (counted), but it isn't faster, and without a profiler I
   can't say why.
3. **Tables per target are out.** Spreading rows over 256 tables nearly
   doubles an unrelated query over them; a table per parent (1000 tables
   of 8) would be worse. This rules out relationship pairs stored in
   archetypes (option 1) for this engine's tables.
4. **Compound colliders are data.** Parts on the body are 7× cheaper than
   collider entities. They also fit [spatial
   storage](spatial-storage.md#how-general-it-is), which allows one extent
   per key: a body's box is the union of its parts.

## Recommendation

- **Contacts: a physics-owned table with a `Contacts<Data, Filter>`
  parameter** (option 4), with a per-entity adjacency so that a narrow
  query (one player) visits only its own contacts. Edge events come from
  `began` and from rows that disappear. Hooks are scheduled systems
  between finding contacts and solving them.
- **Colliders: compound data on the body** (option 5).
- **Hierarchy and `Transform`: plain entity fields** (`ChildOf { parent }`,
  option 2 without back-references at first). Engine-kept back-references
  come when something needs to walk down a hierarchy (despawning a subtree),
  not for propagation, which they don't speed up. The lever is a faster
  `Query::with`: it currently re-derives per call what the query could
  decide once.

The cost: contacts are physics's own structure, not entities, so a game
can't hang components on them, and the table-and-parameter pattern exists
twice (`Spatial`, `Contacts`) without being an engine feature. If a third
user appears, it becomes one.

**Found on the way:** restitution is lost on speculative contacts
(get-emj.19). The solver slows a landing body to gap/dt the step before
it touches, and bounces it from that speed, so a ball landing at 10 with
restitution 1 comes back at 3.

**Open question:** whether to take this recommendation, or relationship
pairs stored sparsely (option 1 without the fragmentation), which would
cover all three cases with one feature at the cost of the largest ECS
change.
