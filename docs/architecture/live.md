# Live relations

**Status: built for one kind of relation** (2026-09-27, get-emj.36, get-pmk):
`engine_ecs::Live<R>` (`engine/ecs/live.rs`), where `R` is a
`Proximity`, the pairs of a spatial key's rows near each other, kept
current between steps. Any number of relations, each its own kept set.
Other kinds of relation are open (get-pmk). This doc is the design's home:
the problem it solves, how it looks to the code that uses it, what it
costs storage, how it's tested, where the ideas come from, and how it
could grow. The measurements behind each of its choices are in
[spatial-storage.md, "Keeping pairs"](spatial-storage.md#keeping-pairs).

## The problem

Most of what an ECS answers is found fresh each time it's asked. A query
walks the tables, and a system builds whatever it needs from what it
finds. That is right when the answer is cheap, or truly different every
frame. It goes wrong when the answer is expensive and mostly the same as
last frame's: a *derived relation between entities*, a function of their
components that changes a little at a time.

"Which pairs of bodies are near each other" is the standard case, a
self-join of every row against every other by position. The same shape
turns up across a game:

- which entities are inside which trigger zones;
- what each AI agent can perceive;
- which entities each networked client should be sent (interest
  management);
- what each light touches, what each camera sees.

Before `Live`, a system had two ways to get such a relation, and both were
bad:

1. **Recompute it every frame.** Correct and simple, and it costs the full
   join even when nothing moved. That was `near_pairs`: about 400 µs a step
   for a 2D pile of 10 000 at rest, where Box2D spends nothing.
2. **Cache it in the system's own state.** Fast, and it's a hidden index
   beside the world:
   - the scheduler can't see who reads or writes it, so nothing orders its
     users;
   - two systems can't share it;
   - a reload can drop it, or keep a copy that's wrong for the new build;
   - it copies the world's data into a shape nothing else can use.

   Physics had exactly this (its `SpatialIndex`, then its contact cache),
   and removing it was the first physics retrospective's main lesson.

Databases know the good version as *incremental maintenance of a
materialized view*: store the derived result, and update it from what
changed rather than from everything. A *live relation* is that, inside
the ECS: declared once, kept by the world, and always exact.

## What using it looks like

A relation is declared once, as a type, and taken by a system as a
parameter, like `Query` or `EventWriter`. The declaration says *what* the
relation is; the parameter says *how* it's kept:

```rust
/// Pairs of colliders near each other, one of them awake and able to move.
struct Contacts;
impl Proximity for Contacts {
    type Key = Position;
    type Active = AnyOf<(
        (With<(Position, Collider, Body, Velocity)>, Without<Asleep>),
        (With<(Position, Collider, Body)>, Without<(Velocity, Asleep)>),
        (With<(Position, Collider, Velocity)>, Without<(Body, Asleep)>),
    )>;
    type Passive = AnyOf<((With<(Position, Collider)>, Without<(Body, Velocity)>), With<(Position, Collider, Body, Asleep)>)>;
    const GROW: f32 = narrow::MARGIN;
    const MARGIN: f32 = FAT; // optional: the engine's default is 0.02
}

fn find_contacts(_: &mut Cx, /* the queries it reads data through */ mut near: Live<Contacts>) {
    for &(a, b) in near.pairs() {
        // ... the narrowphase ...
    }
}
```

That is the 2D physics mod's relation (`engine/std/physics2d/lib.rs`), which
finds its pairs across workers: `near.pairs_with(&workers)`.

**The names.** The two halves are named apart on purpose. `Proximity` is
the *kind* of relation, and it is spatial: pairs of rows of one spatial
key whose boxes meet. `Live<R>` is the *mechanism*: a result kept current
as the data under it changes, and always exact. The category is *live
relations*, so another kind (a join over ordered keys, an aggregate:
sketch 5) would be declared by its own trait and taken as `Live<R>` too.
The name it replaced, `Kept<K>`, described the mechanism and named the key,
which is neither what a system wants nor what tells two uses apart.

**The contract:** `near.pairs()` returns exactly what
`near_pairs(active, passive, R::GROW)` would over the relation's sides, bit
for bit: every pair of rows whose boxes, grown by `GROW`, meet, with at
least one row active, lesser entity first, sorted. `MARGIN` can change
only the cost, never the answer. The result is borrowed from the kept
state, not copied.

**Sides are lists of filters.** A side is a filter (`With`, `Without` and
tuples of them, as a query's), or several as `AnyOf<(F1, F2, ..)>`, whose
tables are those matching any of them; `AnyOf<()>` is a side with none. A
spatial side already matches whole tables (spatial-storage.md, "Two
sides"), so a side is a set of tables, and a union of filters is the
smallest thing that expresses one: physics's active side was three queries
because a query's filter is a conjunction. The union is only a side:
`AnyOf` isn't a `Filter`, and a tuple of filters stays their conjunction
everywhere. A general `Or` in `Query` would also have done it, and was
rejected: every rule that reads a query's filter assumes it's one list of
required and one of excluded components (which tables it matches, the
footprints and overlaps in `graph.rs`, when two queries' guards can't
meet), so `Or` would reach all of them. A list of filters is a list of
those, each read as a query of the key, so every rule applies unchanged.

**What is refused at registration**, each with a test
(`live_relations_refuse_what_they_cant_keep`):

| declared | why it would go wrong |
|---|---|
| a side filter naming a sparse component | a row can join or leave the side with no page changing, so every call would be afresh, silently as slow as `near_pairs` |
| a filter that doesn't require the key | it could match tables without the key, which aren't in its order. The key isn't added for it, so a filter reads as exactly the tables it covers, as it would in a query |
| a key that isn't spatial | there are no pages to keep pairs of |
| one relation taken twice by one system | two write guards on one lock |
| a relation beside a query that writes its key | its reads of the key would meet the query's write (as two queries' would) |

What remains a panic is what registration can't see: a table holding two
spatial keys is in the first one's order, and a side that matches such a
table for the other key panics at the call, naming the relation.

**Scheduling.** Taking `Live<R>` is a *write* of `R`, and a read of
`R::Key` in each side's tables, declared as one query per filter. Two
systems that take one relation are ordered like two writers of one
component. Two relations on one key are independent, since both only read
the key; a system that only reads the key isn't held up; and a system that
writes the key is ordered with every relation on it, since the relation
reads what it writes. This is what a private cache couldn't give: the
dependency is visible, so the scheduler can order it and, later, run
around it.

**The old API**, and why it changed, is in a footnote.[^kept]

**What games see.** Nothing directly, today. Each physics mod declares
its `Contacts`. Games see its results as physics turns them into entities:
`ContactPair`/`Manifold` for contacts and `Overlap` for sensors, in
ordered tables that any system can query ([relationships.md](relationships.md)).
Sensor overlaps and solid contacts come from the same relation. A game
may declare relations of its own, on physics's `Position` or its own key:
each is kept apart from physics's.

**The cost model.** What a call costs depends on how much moved since the
last one (2D, 10 000 bodies, `-c opt`, one thread; from spatial-storage.md):

| state of the world | what the call does | broadphase µs (fresh was) |
|---|---|---|
| nothing changed | returns the last answer | ~0 |
| at rest, a few rows nudged inside their fat boxes | retests only their candidates | 2 (368) |
| settled, creeping inside the fat boxes | retests every candidate | 107 (383) |
| some rows leave their fat boxes (up to a tenth) | searches again for those rows only | grows with the rows moving |
| more than a tenth moving (falling) | finds fresh, then waits 1-16 calls before looking again | 197 (189) |

`near.stats()` says which of these a call took (`How::Same`, `Few`,
`Kept`, `Rebuilt`, `Afresh`), with counts and timings, for benches and
tests.

**What declaring it costs:** nothing a call can measure. `live_bench`
times the same lattice through `Live<R>` and through the state alone,
and the physics compare ran interleaved against the build before
`Live` (2026-09-27, 10 000, pairs call µs, settled 2D 106 against 106,
turning 155 against 156, falling and 3D the same). That took keeping
the kept state's update out of line (`#[inline(never)]`): inlined into
`find_contacts` it ran 4% slower, though the work is the same. Fetching
the parameter takes a query per filter, outside the call.

## How it works

Five ideas, each of which could be reused for another relation:

1. **The result lives in the world**, one kept set per relation, beside
   the spatial order it's derived from. Not in a system and not in a
   mod: storage owns it, a reload keeps it, and access to it is declared.
2. **Change detection says where to look.** Each spatial page records the
   world tick its rows last changed at (`SpatialPages::changed`), and each
   table the tick a row last left it. A call visits only pages changed
   since its own last call, so a world at rest costs one comparison per
   page.
3. **Slack keeps small changes from mattering.** Each row keeps a *fat
   box*, its box grown by `margin`, for as long as its box stays inside it
   (and within twice the margin; below). While a row stays inside its fat
   box, the set of rows it could possibly pair with can't change, so
   nothing about it needs searching for again. That turns "every row moved
   a hair" into "no row moved".
4. **Candidates, then an exact test.** What's kept is a superset, the
   *candidates* (pairs whose fat boxes meet). Each call tests them exactly,
   with the same operations `near_pairs` uses, so the answer is exact. The
   optimization can't change behaviour, which is why it can be verified
   against brute force and `near_pairs` on every frame of a test.
5. **Fall back when maintaining would cost more.** When more than a tenth
   of the active rows leave their fat boxes (`MOST`), searching for each
   costs more than finding everything fresh, so it does that, and backs off
   (1, 2, 4 … 16 calls) before trying to keep again.

**The second bound on a fat box**, which Box2D's and Rapier's don't have:
a row keeps its fat box only while the fat box is also within `2 * margin`
of the row's box. A shrinking box (a turned box squaring up) would
otherwise leave a fat box far bigger than itself. With the bound, a moved
row's new candidates are found among the spatial pages' own boxes, grown
by `grow + 2 * margin`, using `near_pairs`' own sweep. There's no second
index of fat boxes to search.

**Entries are by entity index, not by page.** The spatial re-sort moves
rows between pages every step (5% of a settled pile, 18% falling), and
records by entity don't notice.

## What it does to storage

**Added to every spatial table**, whether or not any relation uses it:

- one `u32` tick per page (`changed`), stamped by the re-sort when a page's
  boxes are re-bounded or a row arrives, and by `swap_remove` when a row
  leaves (a despawn marks no table for a re-sort, so it must stamp itself);
- one `u32` per table (`left`), the tick a row last left it, so a call asks
  which rows went only when some did.

That's 4 bytes on a page of 16 rows, plus the stamping. The page layout
and the re-sort are otherwise unchanged.

**Added to the world:** a slot per relation (up to 256), made when a
system taking it is first registered, by the relation's type name: an
empty `LivePairs` under an `RwLock`, which allocates only when first
called. One key's relations share its pages' ticks and nothing else: each
compares the ticks with its own last call. (Until 2026-09-27 the one set
a key had was in its `ComponentInfo`, in every component's.)

**Once a relation is in use**, for each one:

| what | size | at 10 000 bodies in 2D |
|---|---|---|
| a record per entity index, up to the highest index a covered table holds: the grown and fat boxes, generation, when it last moved and was seen, its side | 48 bytes (2D), 64 (3D) | ~480 KB |
| per page, the entity indices it held at the last walk | 68 bytes | ~42 KB |
| per candidate: its key (`u64`), whether it met, an index by its greater end | ~13 bytes | ~250 KB (19 314 candidates) |

The records are indexed by entity index, so their size follows the
*highest* index among the key's rows, not how many there are. A world
whose physics bodies have high indices among many other entities pays for
the gap. That's the same trade `Slots` makes in physics, and is fine at
today's scale, but it's worth knowing before a game spawns millions of
non-physics entities first.

**What storage must now guarantee.** A live relation is correct only if every
change to a row's box, and every arrival and departure, stamps the page
it's on. That's a new invariant of spatial storage: code that ever changes
a page's lanes without the re-sort (or `swap_remove`) would silently
break every relation. The mutations listed under Testing include removing each
stamp, and the tests catch it.

**Reload.** `LivePairs` is plain data of `engine_ecs`'s own types. The
loader and every mod link one `engine_ecs` under the one-compiler rule, and
changing it bumps `API_VERSION`. So a reload keeps it like the rest of the
world, and nothing mod-built is in it (no vtables, no closures). It is
found again by the relation's type name, which a mod's builds share under
the one-compiler rule. A new build that declares another `GROW`,
`MARGIN` or sides starts it over at its first call: correct, and a full
search that once.

**Why not somewhere else in storage** (measured, spatial-storage.md
decision 3):

- **As entities** (a table of pairs): each begun or ended candidate would
  be a structural change, about 100 ns each, thousands a step while a pile
  lands. And each candidate test would be two lookups through entity
  locations.
- **In the spatial pages:** readers of the tables would have to write the
  order, which serializes every region query. And a pair across two
  tables (a body and a static) has no single page to live in.

## How it's tested

**`//engine/ecs:live_test`**, in 2D and 3D:

- **A scripted world, checked every frame against brute force and
  `near_pairs` fresh.** The script covers:
  - at rest, and creeping;
  - a few rows nudged back and forth inside their fat boxes (the
    few-changed path, seen to make and end pairs);
  - everything wobbled once, and 2% flying;
  - extents written (boxes that grow and shrink without moving);
  - everything falling and shaken: fresh, then the back-off;
  - rows spawned, and despawned with their indices reused or not;
  - rows moved between tables, between the active and passive sides, and
    out of the spatial tables altogether;
  - despawns with nothing else re-sorting, so only `swap_remove`'s stamp
    can see them.
- **`LivePairs::check` (`Live::check`), after every call:**
  - every row's side, box and fat box against the tables (the fat box
    holding the box, and within `2 * margin` of it);
  - nothing kept of rows that are gone;
  - the candidates exactly the pairs whose fat boxes meet, by brute force;
  - whether each candidate meets.
- **How each frame was answered is asserted,** not just what. At rest it
  must be the last answer, nudged the few-changed path, falling fresh,
  then the wait. A fallback that silently always went fresh would pass
  every equality check; this catches it.
- **`live_relations_are_a_footprint_the_scheduler_sees`:** two takers of
  one relation are ordered, a reader of the key isn't held up, another
  relation on the same key or another key is independent, and a writer of
  the key waits for every relation on it.
- **`two_relations_on_one_key_are_kept_apart`:** two relations on one key,
  with other sides, grow and margin, called in turn in a frame and alone
  on frames of their own, through motion and churn: each is its own
  sides' `near_pairs` every call, checked, and answered as its own history
  says (the last answer at rest, kept while creeping), where one shared
  set would start over at every call.
- **`a_declared_margin_is_the_one_kept`:** every row slid by more than the
  default margin and less than a declared one leaves every fat box at the
  default and none at the declared.
- **`live_relations_refuse_what_they_cant_keep`:** each refusal in the
  table above, and two relations on one key taken by one system allowed.

**Checked by mutation** (CLAUDE.md, "green has to be earned"): 24
mutations of the kept state, the footprint rules in `graph.rs` and
`query.rs`, and the stamps in `spatial.rs` are caught, most by `check`.
One survives: the slack for rounding in the search, which only matters
for coordinates around 1e5. The declared relation's code (2026-09-27)
was checked the same way: 15
mutations of the declaration, the refusals, the keying by relation, the
footprint and the sides' split (`live.rs`, `graph.rs`, `world.rs`,
`query.rs`) are all caught by `live_test`, the shared-state one by the
two-relations test and the default-margin one by the margin test.

**Through physics,** where it runs every step:

- `:tax` still asserts the ECS step and the same step on plain arrays end
  bit for bit the same.
- The quality tests bound settling against Box2D, Rapier and Box3D.
- The physics reload and replay tests run with the relation live across
  a reload.

**Not covered yet:**

- **The world fuzzer** (`//engine/ecs/fuzz:world`) doesn't take a live
  relation. Adding one as an operation checked by `check` is the natural
  next step for coverage (get-emj.45).
- **A reload that changes the key's extent glue** (how boxes are computed)
  without moving any row. Whether the re-sort after the install re-bounds
  and stamps every page is untested for live relations (get-emj.46).
- **The kept path runs on one thread.** Only the fresh path splits across
  workers.
- **Miri isn't needed for its own sake:** `live.rs` has no unsafe code.
  `live_test` is in the Miri suite anyway (`//engine/ecs:miri`, sized
  down), for the spatial glue its spawns, despawns and table moves drive.

## Where it comes from

Nothing in a live relation is new. What's particular is the combination, and where
it sits: inside the world, as a declared parameter, with an exact answer.

| idea | where it comes from | in `Live` |
|---|---|---|
| fat boxes: slack so small motion changes nothing | Box2D's `B2_AABB_MARGIN` and its move buffer; Box3D; Rapier's loosened BVH leaves; Bullet's `btDbvt` broadphase | each row's fat box, plus a second bound (2 × margin) they don't have |
| a neighbour list with a skin, rebuilt only when something moved more than it | Verlet lists in molecular dynamics (Verlet, 1967): each particle's neighbours within cutoff + skin | the same idea per row: candidates within the fat boxes |
| exploiting frame-to-frame coherence for pairs | I-COLLIDE's incremental sort-and-sweep (Cohen, Lin, Manocha, Ponamgi, 1995); Bullet's `btAxisSweep3` and its persistent pair cache | pairs kept between calls, searched again only for rows that moved |
| look only at what changed, by a coarse change stamp | Unity DOTS's per-chunk change versions (`DidChange`); Bevy's change ticks; Flecs's per-table change detection; a database page's LSN | a tick per spatial page, compared with the call's last tick |
| re-examine only pairs next to what changed | Rapier's BVH update ("a pair can only stop overlapping if one side changed") | the few-changed path, a search from either end of the candidates |
| keep a query's result, maintained as the world changes | Flecs's cached queries (matched tables kept as tables appear); EnTT's groups (kept packed through construct and destroy signals) | kept pairs, a relation between entities rather than a set of them |
| store a derived result, update it from the changes | materialized views and their incremental maintenance (Gupta and Mumick, 1995); differential dataflow (McSherry et al., 2013); DBSP (Budiu et al., 2023) | the view is the pairs, the changes are page ticks, and a refresh beyond a threshold is a full recompute |
| a cheap filter, then an exact test | the filter-and-refine spatial join (Brinkhoff, Kriegel, Seeger, 1993), bounding boxes first | candidates by fat box, then the exact grown-box test |
| a region inside which an object can move without changing a query's answer | safe regions in moving-object databases (Prabhakar et al., 2002) | a fat box is a row's safe region |
| data physically in the order a query wants | a database's clustered index | spatial tables (spatial-storage.md), which `Live` searches instead of a tree |

**Where the ECSs we credit stop short.** Flecs's cached queries and EnTT's
groups keep *which entities match* a query, incrementally. Unity's and
Bevy's change stamps say *what changed*. None keeps a *relation between
entities derived from their values*: that stays the user's job, in a
system or a resource, beside the world. Flecs's relationships make pairs
first-class entities, which is the "pairs as entities" option we measured
and rejected for this churn rate. A live relation sits between the physics engines,
which keep pairs but aren't an ECS, and the ECSs, which keep matches but
not relations.

## How it could grow

Each sketch is a direction, not a plan. The one general rule, from the
problem statement above: whatever is kept lives in the world, is reached
through a declared parameter, and answers exactly what recomputing would.

### 1. More than one kept set per key (done)

**Built 2026-09-27** (get-pmk), as the relation declared by a type:

```rust
struct Perception;
impl Proximity for Perception {
    type Key = Position;
    type Active = With<(Position, Agent)>;
    type Passive = With<Position>;
    const GROW: f32 = SIGHT;
    const MARGIN: f32 = 0.5;
}
fn perceive(_: &mut Cx, mut seen: Live<Perception>) {
    for &(agent, thing) in seen.pairs() { /* ... */ }
}
```

The sketch had a label type beside the key (`Kept<Position, Perception>`),
and the sides, grow and margin still passed at each call; declaring them
with the label is what let registration check them. The world keeps a
slot per relation; the page ticks are shared, and any number of relations
compare their own last tick against them, which is why a tick by page was
chosen over Box2D's single-reader move buffer (decision 4).

### 2. Began and ended, as well as the pairs

The kept candidates already know when a pair starts or stops meeting (the
`met` flags). `Live` could return the difference too:

```rust
let changes = near.changes();
for &(a, b) in changes.began { /* ... */ }
for &(a, b) in changes.ended { /* ... */ }
```

That's what Rapier hands its narrowphase, and what get-emj.12 wants for
per-body began- and ended-touching events. The exactness contract carries
over: `began` and `ended` must be exactly the difference between this
call's `near_pairs` and the last's.

### 3. Sparse filters without going fresh

A side filtered by a sparse component would go fresh every call, because
a row can join or leave the side with no page changing; since 2026-09-27
it is refused at registration instead of allowed to ("What is refused",
above), which physics doesn't meet: its sides filter by table components
only. Giving sparse sets a change tick of their own, stamped on insert
and remove, would let a relation see those changes the way it sees pages,
and lift the refusal. It would matter for a side filtered by a marker
that comes and goes often, which is what sparse storage is for.

### 4. Region membership

Trigger zones, interest management and area effects are a relation
between a few *regions* and many *entities*: "which entities are inside
which zone". With zones as the passive side, that's already `near_pairs`,
and, with (1) built, coexists with physics's relation. A dedicated shape could
keep, per zone, its members, and return joins and leaves (2), which is
what a networking layer's interest management needs every tick.

### 5. Relations that aren't spatial

The same five ideas apply to any derived relation whose inputs have
change stamps:

- **A join over ordered keys:** each `ChildOf` child with its parent's
  `Transform`, kept while neither changed. Ordered tables are to key
  order what spatial tables are to space: a range per key, found by
  search.
- **An aggregate:** the total mass of each parent's children, or the count
  of each team's units. It's kept and adjusted by the rows that changed,
  falling back to a recount when many did.

A sketch of what a general form might look like:

```rust
/// A relation storage keeps between calls: found fresh by `all`, updated
/// from what changed by `update`, which must answer as `all` would.
trait Derived: Send + Sync + 'static {
    type Inputs: Param;   // what it reads: its footprint
    type Answer;
    fn all(inputs: &Self::Inputs) -> Self::Answer;
    fn update(&mut self, inputs: &Self::Inputs, since: Tick) -> Update<Self::Answer>;
}
// A system takes it as `Live<MyRelation>`, a write of the relation.
```

**The design question this raises, and the reason it's deferred:** where
does the derivation's *code* live? Spatial order, ordered keys and
`Proximity` are kinds built into `engine_ecs`, whose code the loader and
every mod share: a mod declares a proximity relation's key, sides and
grow, never its code. So what `Live` keeps is plain `engine_ecs` data that survives
reload. A relation defined by a mod is different:

- its code reloads with the mod;
- what it keeps must follow the component rule (only `FieldType` data,
  with a schema to migrate by) or be thrown away at a reload and rebuilt,
  which is correct but costs a full recompute;
- the check that `update` matches `all` would then be the mod author's
  test to write.

Until a second relation shows a need, adding kinds to `engine_ecs`, as
spatial and ordered were added, keeps the guarantees that make `Live`
trustworthy: another kind would be a trait beside `Proximity`, taken as
`Live<R>` too.

### 6. Visible to readers

Today a relation's pairs are reachable only by whoever takes `Live<R>`
for writing. A read-only parameter (`LiveView<R>`, a read lock) would let
other systems read the last answer without recomputing it, ordered after
the writer like any reader after a writer. For physics, today's answer is
the contact entities themselves; for sketches (1) and (4), a view may be
the natural way to consume the result.

### 7. Across threads

The walk over changed pages and the candidate tests both split by range,
as `near_pairs_with` already splits the fresh search. Not done, and part
of parallelism's open work (get-znt.5).

## What's left

- **Settled isn't free.** Every candidate is retested each step a pile
  creeps: about 20 000 in 2D and 31 000 in 3D at 10 000, each two records
  read at random. Box2D and Rapier show 0 there only because their
  narrowphase does the same test on every fat pair.
- **Falling pays a few percent** in the broadphase stage against fresh,
  from a fresh call's fixed cost; not found.
- **Sparse filters are refused** (sketch 3), and **no fuzzing yet**
  (Testing, get-emj.45).
- **Other kinds of relation:** get-pmk.

[^kept]: **The old API** (2026-09-27, replaced the same day). The world
    kept one set of pairs per spatial key, taken as `Kept<K>`, and the
    system passed the sides, the grow and the margin at every call:
    `kept.near_pairs(&(&moving, &held, &drifting), &(&statics, &asleep),
    narrow::MARGIN, FAT)`. Sides that differed from the last call's, or
    another grow or margin, silently started the set over, so a second
    broadphase over one key made both go fresh every call; a side
    filtered by a sparse component silently went fresh every call; and a
    side with another key's tables panicked at the call. The name
    described the mechanism and named the key. Declaring the relation
    (`Proximity`) and naming the mechanism apart (`Live`) moved each of
    those into a declaration registration can check, and made the state
    the relation's rather than the key's. The kept state itself, and every
    answer, is unchanged (`KeptPairs` is `LivePairs`, `KeptStats` is
    `LiveStats`, `ParamDecl::Kept` is `ParamDecl::Live`).
