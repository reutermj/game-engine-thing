# Parallel relations

**Status: a design study, resolved by later decisions** (2026-09-29,
get-znt.19; closed 2026-10-04). Of what it proposes: (d), the host pool,
is built, as the resident `threads` mod ([threads.md](threads.md),
2026-10-03); phase 1, contacts' colors kept in the world, is open
(get-emj.74); phase 2 was rejected and replaced by shapes a system
declares ([flows.md](flows.md#parallel-shapes), get-znt.28); phase 3 is
deferred and likely superseded by `Passes` over flows. Both physics
solves run across threads, bit for bit the same at any thread count
(physics.md, [Solving across threads](physics.md#solving-across-threads)),
on physics's own per-step copy of bodies and contacts, declared as flows
since 2026-10-02. This doc asks how the ECS could grow so that the same
kind of parallelism fits the world instead of living beside it. Its
claims are measured by a spike,
`//engine/std/physics2d/compare:colors_spike` (`colors_spike.rs`, a bench
target, not engine code).[^spike-code] Where it describes the engine, it
describes it as of 2026-09-29 unless it says otherwise.

**The recommendation, in short:**

- **Colors become world state.** Each contact keeps its color in the world,
  a byte in its row, kept in pair order. A contact is colored once, when it
  begins, and every so often all colors are packed again. This is phase 1,
  and it is physics's alone (get-emj.74). Measured with the solve handed
  kept colors at 8 threads: 5% faster (879 → 838 µs on the settled turning
  pile of 10 000, 585 → 556 on the 5050 pyramid). About 9% is projected
  once placing contacts in their batches runs across the threads too.
- **The staged run does not move into `engine_ecs` as proposed.** Phase 2
  would have handed any system the host's threads to run its own stages on
  a copy of world data. The user rejected that (2026-10-02): systems
  shouldn't take the pool to run parallel work the ECS can't see. The user
  then decided the rule (get-znt.28): **parallel work is declared and run
  by the scheduler, never by a system.** The staged run returns as a shape
  a system declares over a flow and the scheduler runs (get-znt.29); see
  [flows-spike.md](flows-spike.md).
- **The primitive over world storage is likely superseded** (phase 3). A
  colored iteration over flows (`Colored::passes`, flows-spike.md) needs
  no unsafe code, since the flow owns its memory. What phase 3 would still
  add is the one-pass relations, which write the world directly.
- **The solve keeps its copy.** Solving in pages as they are is still a
  no: measured again, pages cost 10% on the passes to save an 11 µs copy.
  Neither storage owning the copy ([working-sets.md](working-sets.md)) nor
  contiguous columns ([contiguous-columns.md](contiguous-columns.md)) beat
  it. The copy is the solver's own layout and order, and becomes declared
  flows.

## The problem

A **relation table** is a table whose rows each name some entities: a
contact names two bodies, a joint two bodies, a spring two particles, an
overlap a sensor and what it senses. Many systems over such a table do
more than read it. **Each row writes the entities it names.** A contact
changes both bodies' velocities; a spring pushes both ends; a hit takes
health from its target and gives heat to its source.

Running such a system across threads raises two problems that a query
over one table doesn't have:

1. **Rows collide.** Two rows that name one entity both write it. Split the
   rows over threads naively, and two threads write one body at once: a
   data race, or in safe Rust, code that doesn't compile.
2. **Order is part of the answer.** Sequential impulses (Gauss-Seidel) read
   what the row before wrote, so the result depends on the order rows are
   solved in. For the result to be the same at any thread count, that
   order can't depend on which thread took what.

**Graph coloring** answers both at once:

- Rows are grouped into colors so that no two rows of one color write the
  same entity. Entities that don't move don't count: many rows may read a
  static body.
- The colors run in turn, and within a color any split over threads
  computes the same thing, because a color's rows are independent.
- The result is then a function of the coloring (which rows are in which
  color, and the colors' order) alone, never of the threads. The solver's
  test held it bit for bit across 1 to 16 threads
  (`the_colored_solve_across_threads_is_the_solve_on_one_bit_for_bit`; since
  2026-10-03 the mod's passes on the `threads` mod's pool,
  `the_mod_across_threads_is_the_arrays_bit_for_bit`, threads.md), and
  the spike asserts it for colors kept across steps too.

The general shape: **a system over a relation table whose rows write the
entities they reference, run in parallel in colors, deterministically at
any thread count.**

### Who else wants it

Two classes, and they want different things:

- **Order-dependent** rows read what earlier rows wrote, as the solver does:
  the result is Gauss-Seidel's, and coloring is how it runs in parallel.
- **Accumulating** rows each add something to their ends (a force, damage,
  heat), computed from values as they were before the pass. That is Jacobi,
  and it needs no colors to be correct. It still needs a fixed order of
  addition to be the same at any thread count, since float addition isn't
  associative.

| relation | class | churn | passes a step | what it needs that physics doesn't |
|---|---|---|---|---|
| contacts (physics) | order-dependent | low settled, some falling | 20 (5 substeps) | – |
| joints, ragdolls, vehicles' suspension | order-dependent | almost none: made and broken by the game | as many as contacts, solved with them | rows of other shapes solved in the same colors: Box2D colors joints and contacts in one graph (`b2AssignJointColor`) |
| springs, ropes, cloth (position-based) | order-dependent | none | several iterations | many rows at each entity (a cloth vertex has 6 to 12): more colors, which Fratarcangeli and Pellacini's partitioning for PBD addresses |
| damage or heat between touching pairs | accumulating, or order-dependent if clamped (health at 0 kills, and later rows must see it) | follows contacts | one | **writes game components in the world**, in one cheap pass, where a copy would cost more than the pass |
| flocking, crowd avoidance | accumulating (forces summed) | high: neighbours from a `Live` proximity change every step | one | colors can't be kept (the relation is new each step), so a reduction by entity rather than colors |
| trigger responses | usually one end written (the thing in the zone) | follows overlaps | one | rows that write one end: group by that end, not color |
| hierarchy (`ChildOf`) propagation | writes one end (the child), reads the other | none | one | not coloring at all: levels of a tree, each level a plain parallel walk |

What they have in common with physics: rows in a table, ends by entity,
the ends' components written, determinism at any thread count. What
**they need that physics doesn't**:

- **Writing the world in place.** A game's damage pass is one pass over a
  few hundred or thousand rows, writing `Health`. Physics copies because it
  makes 20 passes a step ([The solve still copies](#c-solving-in-world-storage)).
  A one-pass relation would pay the copy for nothing.
- **Accumulation** as a first-class shape, with a reduction whose order is
  fixed.
- **Other shapes of row:** one end written, or more than two ends.

What **physics needs that they don't**:

- many passes over the same rows with the same colors;
- lanes of four for SIMD;
- stages with spinning barriers between them, 158 a solve;
- warm starting;
- rows that read a body many rows share (statics) without owning it.

None of them runs in parallel today, and none is at a scale where it would
need to: no game in the repo has more than a few hundred rows of any
relation but contacts. That, more than any design question, sets the
order of the phases below.

## What exists, and what's outside the ECS

**In the world**, declared and scheduled:

- the bodies (`Position`, `Velocity`, `Body`, `Spin`, `Rotation`), in
  spatial tables;
- the contacts, as entities in an ordered table in pair order
  (`ContactPair`, `Manifold`, `Response`, `Impulse`, `ContactPoints`;
  [relationships.md](relationships.md));
- the broadphase's pairs, a live relation (`Live<Contacts>`,
  [live.md](live.md));
- the threads' entry point: the shapes (`ParMap`, `Reduce`, `Passes`),
  parameters a system calls with its kernels, run across the executor the
  resident `threads` mod installs in the world (threads.md), with
  `par_for_each` and the page walks split over a `ParMap`.[^workers]

**Outside it, in the solve system's per-step copy** (`solver::solve_across`
when this was written; since 2026-10-03 the pipeline's flows, its passes on
`Passes` and the stages in `engine_ecs`'s dispatch, threads.md):

| what | where | why it is outside |
|---|---|---|
| bodies as dense states, by an index the step makes (`Slots`) | `Head::s`, then `lanes::Atom`s (each `f32`'s bits in an `AtomicU32`) | the passes want dense, stable indices; spatial pages move rows every step ([What the ECS costs](physics.md#what-the-ecs-costs)) |
| each contact's color | `Head::groups`, from `lanes::group`, greedy in pair order, **every step** | a pure function of the contacts: nothing to store, and replays need nothing stored |
| contacts in batches of four lanes, by color | `Batch<N>`, `Lane` | SIMD layout, a transpose of the contacts |
| the stages and their barriers | `lanes::run_across` when this was written: stages claimed by `fetch_max`, a count a stage, no main thread. Since 2026-10-03, `Passes`' program, its stages dispatched by `engine_ecs::dispatch` (threads.md, "Dispatch") | a protocol inside one run of the executor, which the ECS had no word for; now a shape it provides |
| the threads | the host's, through `Workers`, when this was written; since 2026-10-03 the `threads` mod's pool, through `Passes` | a reloadable mod can't own threads (physics.md, [Parallelism](physics.md#parallelism)) |

So the parallelism was three things the ECS doesn't see:

1. a **derived relation**, the coloring, recomputed every step;
2. a **sharing mechanism**, relaxed atomics over a copy;
3. an **execution protocol**, stages within one run.

The third is the ECS's since 2026-10-03: `Passes`. The second is split:
the shape shares the states across threads, in a form the mod defines
(`Shareable`; flows.md, "On one thread"). The first is still physics's.

**What the scheduler sees is enough.** The solve system declares that it
writes `Velocity` and `Position` in the bodies' tables and `Impulse` in the
contacts', and reads the rest. That is at the granularity of tables and
columns, as every footprint is (`graph.rs`). Which rows each thread
writes is the system's business, as it is for `par_for_each`'s chunks.
Nothing in the design so far needs the scheduler to know about colors.

**What it costs to leave it outside:** the serial part of the solve at 8
threads. The pile of 10 000's is about 140 µs of 871 (get-emj.74): the
states 20, the coloring 43, counting and placing 55, the atoms 15. That is
Amdahl's limit of about 7 times, and more than Box2D's whole serial part,
since Box2D keeps its colors.

## Prior art

Read in their fetched source (Box2D v3.1.1, Rapier 0.36 and Jolt, from
the comparisons' Bazel repos) or official repositories and docs (Unity
Physics, Flecs, Bevy, Avian, EnTT, PhysX), 2026-09-29. Marked where only
inferred.

| project | kept in the ECS | outside it | pairwise writes in parallel | colors across steps | same at any thread count |
|---|---|---|---|---|---|
| **Unity Physics** (DOTS) | `LocalTransform`, `PhysicsVelocity` | a `PhysicsWorld` built every step (`PhysicsWorldBuilder.cs`: `MotionDatas`, `MotionVelocities`) and written back (`PhysicsWorldExporter.cs`) | `DispatchPairSequencer`: pairs packed in a `ulong` and radix-sorted, then a single job assigns up to 64 phases by a mask per body (`kMaxNumPhases = 64`, the last serial); masks committed per batch of 8 pairs | no: "stateless… does not cache anything frame-to-frame" | "completely deterministic" (`design.md`); thread count implied by the serial phase build, not stated |
| **Avian** (Bevy) ≥ 0.4 | components | `SolverBodies`, a resource of dense bodies (after `b2BodyState`), and `ConstraintGraph`, a resource ported from Box2D (24 colors, overflow) | `par_for_each` over a color's constraints, with a `SAFETY` comment that the coloring keeps them apart: unsafe code relying on the coloring | yes, incremental | not stated |
| **Bevy** | everything | – | none: `par_iter_mut`, `get_many_mut` and `par_iter_many_unique_mut` are unique per entity; `iter_combinations_mut` is sequential; `ParallelCommands` defers | – | – |
| **bevy_rapier** | components | Rapier's sets, copied in and written back each step (`writeback_rigid_bodies`) | Rapier's | Rapier's | Rapier's |
| **Flecs** | everything; a pair `(Likes, Bob)` is a component id, one table per target | – | none documented: multithreaded systems split a table's rows over N threads; writes to other entities go through per-thread command queues merged at sync points | – | the split depends on N; merge order not verified |
| **EnTT** | everything; owning groups kept packed and sorted | – | none: "the entire registry is not thread safe"; no executor (`organizer` builds a graph and doesn't run it) | – | – |
| **Box2D v3** | no ECS: its own solver sets (static, awake, disabled, one a sleeping island; only the awake set has dense `b2BodyState`s) | – | a `b2GraphColor` per color owns its contacts' arrays and a bitset of bodies; stages cut into blocks claimed by atomics (`b2SolverStage`, `syncIndex`) | **yes**: `b2AddContactToGraph` when a contact begins touching, `b2RemoveContactFromGraph` when it stops; 12 colors in v3.1.1, 24 on main, then an overflow solved on one thread; statics not colored | yes, stated and tested: "two threads will give the same result as eight threads" |
| **Rapier** 0.35+ | no ECS: arenas (`RigidBodySet`) | – | a staged island solver: per-color flat arrays "maintained incrementally by the narrow phase… no per-step collect/sort" (`solver_contact_graph.rs`), `u128` masks per body, 120 dynamic colors and 8 kept for fixed bodies | **yes** | yes, since 0.35: `enhanced-determinism` with `parallel`, "bitwise identical for any thread-pool size", the split fixed at 8 reference workers (`LAYOUT_REF_WORKERS`) so it depends on the scene alone. (Its rayon solver, dropped at 0.18, came back this way.) |
| **Jolt** | no ECS | – | `LargeIslandSplitter`: islands of 128 or more constraints split into 32 by a `uint32` mask a body, the last serial | no: made each step | not stated for thread count |
| **PhysX 5** | no ECS | – | constraint partitions by a mask a body, 32 at first (`DyConstraintPartition.cpp`) | no (inferred) | not stated for thread count |

The shared source of the coloring: Chen et al., "High-Performance Physical
Simulations on Next-Generation Architecture with Many Cores" (Intel
Technology Journal), cited by Box2D, Jolt and Avian. For position-based
constraints, Fratarcangeli and Pellacini's partitioning (Eurographics
2015) and Vivace's randomized parallel coloring (SIGGRAPH Asia 2016)
(from memory, not fetched).

**What it says, honestly:**

- **Everyone copies out for the solve.** Every engine that runs pairwise
  writes in parallel does it on its own copy or its own storage: Unity,
  Avian and bevy_rapier copy in and write back each step, and Box2D,
  Rapier, Jolt and PhysX are their own storage. The reasons are the ones
  this repo measured:
  - dense, stable indices for masks and gathers;
  - SIMD lanes over packed body states;
  - many passes, so a copy in and out is cheap against them;
  - sleeping sets that keep what's solved small.
- **No general-purpose ECS parallelizes writes through a relation.** Flecs,
  Bevy and EnTT defer such writes or leave them to the user. Relations are
  first-class in Flecs, and parallel iteration is per table, but nothing
  connects the two.
- **The newest designs keep colors.** Box2D, Rapier 0.35 and Avian keep
  them across steps and change them incrementally. Unity, Jolt and PhysX,
  older or stateless by design, make them each step.
- **The nearest to "in the ECS" is Avian.** The ECS is authoritative, with
  a Box2D constraint graph kept beside it as a resource and a dense mirror
  of the bodies. Its parallel loop relies on unsafe code whose soundness
  depends on the coloring, which is exactly what this repo's storage rule
  ([storage.md](storage.md#where-the-unsafe-is-and-isnt)) rules out of
  `engine_ecs`.

What none of them has, and what this design can add: colors as world state
that a snapshot, a reload and a replay carry with everything else, reached
through a declared parameter, beside a solve that still copies.

## Candidate designs

### (a) Colors as world state

**The idea.** A contact's color stops being recomputed each step and
becomes part of the contact, as Box2D's, Rapier's and Avian's are. A
contact is colored when it begins, the lowest color free at both its ends
by the same rule as today (`lanes::group`: not color 0 at a static). Its
ends' taken colors are updated only as contacts begin and end.

**Why it's in spirit.** The first physics retrospective's lesson was that
state beside the world (an index, a contact cache) goes wrong: hidden from
the scheduler, dropped or kept wrong by a reload, unshareable. Kept colors
are **state, not a derived view**, and the measurements say which:

- The coloring as built is a pure function of the contacts in pair order.
  But it isn't stable under change: while the pile falls, **761 persisting
  contacts a step** (of about 4400) change color afresh, because one new
  contact early in pair order shifts every later one it meets. Settled
  it's 12 a step of 22 000. So it can't be kept incrementally *and* exact,
  which is what `Live` promises ([live.md](live.md), "The contract").
- A kept coloring is cheap to keep (below) and depends on history: on the
  order contacts began in, and on when colors were last packed.

State that depends on history belongs in the world, where a snapshot, a
reload and a replay carry it. That is the price get-emj.74 names: colors
become state a snapshot must carry. In the world, a snapshot carries them
for free.

**API.** A component of the contact, in physics's interface:

```rust
component! {
    /// The color the contact is solved in: taken when it begins, kept while
    /// it lasts, packed again every `Tuning::repack` steps. `NONE` until
    /// the first solve that sees it.
    pub struct SolveColor: "physics2d::SolveColor" { pub color: u8 }
}
```

The solve system reads and writes it like `Impulse`:

1. **Gather.** Its gather already walks the contacts in pair order, split
   over the workers. It now also ORs each kept contact's color into its
   moving ends' masks (a `u64` a body, `fetch_or` on an atomic). OR
   commutes, so the masks are the same at any thread count, and the masks
   are derived, not stored: the only state is the byte on the contact.
2. **Color the new ones.** On the calling thread, in pair order, each
   contact with `NONE` takes the lowest color free in both masks. That is
   0 to 300 contacts a step on the pile (2 µs falling, 0.0 settled).
3. **Repack now and then.** Every `repack` steps (60 in the spike),
   everything is colored afresh, as today.
4. **Place in parallel.** Placing contacts in their batches by color
   becomes a counting sort whose counts are made per chunk and summed in
   chunk order: two stages of the solve's run, the same slots at any
   thread count (`place_across` in the spike).
5. **Write back.** The colors go back with the impulses.

A contact that ends is despawned, taking its color with it, and its bits
are gone from the next step's masks with no clearing. A body that starts
or stops moving (made static, woken) has its contacts colored again, and
waking contacts from `Resting` come back with `NONE`. Box2D, the same way,
re-adds a woken island's contacts to its graph.

**Storage.** A byte a contact in the ordered table, which stays in **pair
order**, since the merge needs it. Keeping the table in color order
instead, with the color as the key's high bits, would put each color's
contacts in one run and make placing free. The spike measured what it
costs the merge: its walk of last step's contacts against this step's
becomes a merge of 9 runs, 17 µs → 212 µs on the settled pile, far more
than placing (41). A table per color is the same merge over tables, and
moves every row at a repack. Both are rejected.

**Scheduling.** Nothing new. `SolveColor` is a column the solve writes, as
`Impulse` is. A hook between `find_contacts` and `solve` could read it.

**Determinism.**

- **At any thread count:** unchanged. For any valid coloring, the solve
  across threads is the solve on one bit for bit. The spike asserts it for
  kept and repacked colors on every input it times.
- **Against today:** changed. The colors now depend on history, so the
  one-thread solve's order changes and every result moves once. That needs
  the quality suites and baselines rerun, and a decision, as the move to
  colored order had ([The decision: B
  colored](physics.md#the-decision-b-colored)).
- **Across a reload, snapshot or replay:** the colors are in the world, so
  they are carried. A repack by step count needs the count in the world
  too: physics's step counter, or `repack` read against a world tick.

**What it buys, measured** (solve handed the kept colors, so its coloring
is skipped but its placing isn't; 8 kept threads on one CCD, µs, the
median of 21):

| scene | own colors | kept (colors) | kept, repacked every 60 steps (colors) |
|---|---|---|---|
| pile 10 000, settled, step 430 | 879 (7) | 877 (9) | **838** (7) |
| pyramid 5050, step 630 | 585 (6) | **556** (6) | 556 (6) |
| pile 10 000, falling, step 31 | 273 (5) | 276 (6) | 275 (6) |

- **Kept alone gains nothing on the pile**, because the colors fragment.
  They grow from 7 to 9 with a tail of small ones (…678, 56, 4 contacts),
  and each color is 21 stages a solve, so 42 more stages, two of them one
  block each with seven threads waiting. That costs about what skipping
  the coloring saved.
- **Repacked, the colors stay the solver's 7** and the gain is the
  coloring: 41 µs, 4.7%. The pyramid never fragments, so it gains the same
  either way (29 µs, 5%).
- **Falling**, the solve is too small to show it.
- **Placing in parallel**, not built into the solve, would take most of
  the placing's 41 µs off the serial part too (the spike's `place_across`
  is 18.7 µs as two runs of its own; as two stages of the solve's run it
  would be under that). With the states and atoms built by the threads as
  well, the serial part goes from about 140 µs to a few. **Projected**
  (not measured): about 800 µs at 8 threads on the pile, 9% under today.

### (b) An ECS primitive: colored parallel iteration

The general form of (a) and the solve's loop: a parameter that walks a
relation's rows in colors, across the host's threads, and lets each row
write its ends' components in the world:

```rust
/// A relation's rows name the entities they write.
pub trait Relation: Component {
    fn ends(&self) -> [Entity; 2];
}

fn exchange_heat(
    workers: Workers,
    // The rows, and their colors: kept by the world, as `Live` keeps pairs.
    mut touching: Colored<'_, Touch>,
    // What the rows write at their ends.
    mut heat: Query<&mut Temperature>,
) {
    touching.par_for_each(&workers, &mut heat, |touch, [a, b]| {
        let flow = (a.get().kelvin - b.get().kelvin) * touch.conductance;
        a.update(|t| t.kelvin -= flow);
        b.update(|t| t.kelvin += flow);
    });
}
```

(The sketch predates shapes. `Workers` is gone (get-znt.31); a walk
today takes its threads from a `ParMap`.)

**Footprint.** `Colored<R>` declares:

- a read of `R`'s columns in its tables, a query;
- a write of the relation's kept colors, a slot like a live relation's,
  so two systems taking one are ordered (graph.rs's `live()` rule, reused).

The ends' writes are the `Query<&mut Temperature>`'s own declaration. So
**the scheduler needs nothing new.** Its overlap rules are per table and
column, and a colored walk touches the same tables and columns a plain
walk of the same parameters would. Which rows each thread writes is
decided inside the system, by the colors, as `par_for_each`'s chunks are.
Entity-level footprints would be needed only to run *two systems* at once
over disjoint rows of one table, and nothing asks for that.

**Storage.** The colors live where (a) puts them, generalized: a byte a
row that `engine_ecs` keeps for the relation's table, colored as rows
arrive and repacked on a schedule. It is like `Live`'s kept state, but it
is state, not an exact view.

**Execution.** Colors in turn, each cut into blocks, claimed by the host's
threads: the solve's staged run, which is safe code, kept inside the
primitive rather than offered to systems (phase 2 below, rejected).

**The hard part is sharing, not scheduling.** Two rows of one color write
two different bodies, which may be two rows of one page. `engine_ecs`
hands a system a page's column as `&mut [T]` under one guard
([storage.md](storage.md#archetype-tables-in-pages)), so two threads
writing two rows of it at once is aliasing `&mut`. The ways out:

| how | safe | cost | what it would take |
|---|---|---|---|
| split borrows trusted to the coloring | no, and its soundness depends on the coloring and on concurrency | none | exactly what storage.md rules out of `engine_ecs`; what Avian does |
| **atomic views of a column held for the walk** (each field as its bits in an atomic, relaxed, like `lanes::Atom`) | the view is a cast of memory the system holds exclusively, correct whatever the coloring. The colors then buy determinism, not soundness | about 3% on the solver (relaxed atomics against raw pointers: 871 against 849 µs at 8 threads) | `component!` glue for a per-field atomic view, and the cast: `AtomicU32::from_mut_slice` is unstable (`atomic_from_mut`; not checked against this toolchain), so a small unsafe cast, new to the core: **the user's decision**, with Miri over it |
| **deltas and a reduction** (each row writes only its own outputs; a second pass adds them into the ends in a fixed order) | yes, plain `par_for_each` | a second pass, and a way to find each end's rows (the table is in one order) | only the accumulating class: Jacobi, not Gauss-Seidel |

**Verdict: not yet.** Physics wouldn't use it, since its passes run on the
copy (c). No game has a relation at the scale where one pass over it needs
threads. And its sharing mechanism is a decision about unsafe code. It is
worth building when a second user appears, as `Live` waited for a second
relation. The accumulating class then likely wants the safe reduction
first.

### (c) Solving in world storage

The 2026-09-25 retrospective decided against it, as a transpose rather
than a patch over a storage flaw. The spike measured it again on the
colored order, since colors and threads might have changed the balance:
the same contact kernel (one point's normal impulse, bodies turning) over
the colored contacts, 20 passes, one thread, µs:

| bodies | pile 10 000 settled (22 012 contacts) | pyramid 5050 (14 950) | pile falling (3248) |
|---|---|---|---|
| copied: the passes | 2116 | 1403 | 348 |
| copied: gathering from pages and scattering back | 5.3 + 6.1 | 2.9 + 3.2 | 5.3 + 6.2 |
| **copied, all** | **2127** | **1410** | **360** |
| in pages of 16 (9.5 full), each contact's rows found once a step | 2342 (+10%) | 1541 (+9%) | 378 (+5%) |
| in pages, rows looked up at every touch | 2465 (+16%) | 1617 (+15%) | 395 (+10%) |

- **The copy is 11 µs; solving in pages costs 215.** Finding each
  contact's rows once a step alone is 31 µs, three times the copy, and it
  can't be skipped: the spatial re-sort moves 5% of a settled pile's rows
  between pages every step and 18% of a falling one's.
- **Neither (a) nor (b) changes it.** Kept colors don't give bodies dense,
  stable indices. Colored iteration in place would add, on top of the
  pages, the atomic views (3%) or the unsafe split borrows.
- **Across threads it's worse, not better.** In place, the passes would
  share world pages across cores for the whole solve. The copy is dense and
  fits one CCD's cache (4.6 MB of batches).
- **Still a no.** It is what every engine in [Prior art](#prior-art) does,
  for the same reasons. The kernel is a stand-in (one point, arms along
  the normal), but it has the solver's shape: two bodies gathered, a few
  dozen flops, two scattered.

### (d) The host pool

The threads are the host's (get-znt.20): kept, placed on one CCD first,
and kept warm. That was measured to be most of the solver's scaling (882 µs
kept and placed, 1797 left to the scheduler). Everything above runs on
them through the world's `Executor`, which the shapes dispatch onto, so
for this design the pool is a dependency, not a part:

- **One pool for everything:** systems at once, `par_for_each`, the
  broadphase's split, the solve's stages, and a colored walk. Two pools
  would oversubscribe the cores.
- **Stages need every task of a run to make progress** without waiting for
  a task to start: an executor promises that each task runs, not when. The
  solve already copes: any thread takes any block, and a late thread skips
  what's done. A colored walk would inherit that from the solve's protocol.
- **Placement and warmth** help every primitive alike, and the sticky
  affinity measured in [Parallelism](physics.md#parallelism) (chunk `k` on
  thread `k % n`) would help a relation's walks as it helped the broadphase.

Whether the pool is rayon's or our own, and how it pins, was the user's
decision (get-znt.20). **Built** (2026-10-03, [threads.md](threads.md)):
rayon as the thread host, pinned to one CCD with core_affinity, in a
resident mod that installs it as the world's executor; one pool for the
shapes and the ECS's own splits alike (threads.md, "The ECS's own
splits").

### (e) Others considered

- **Colors as a live relation** (`Live<Coloring<Contacts>>`, kept by
  storage and exact): rejected. An exact coloring is the afresh one, and it
  isn't stable under change (761 recolored a step falling), so keeping it
  exactly costs about what computing it does. A kept coloring is history,
  which `Live`'s contract forbids.
- **A color key in the table's order, or a table per color:** rejected by
  the merge's cost (above).
- **Islands:** rejected before ([Parallel solving](physics.md#parallel-solving)):
  a pile is one island.
- **Entity-level footprints in the scheduler:** not needed. Nothing runs two
  systems over disjoint rows of one table, and a scheduler that tracked
  rows would pay for it on every system.

## Spike results

`taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:colors_spike`,
2026-09-29, Ryzen 9 7950X, one CCD, the machine otherwise idle (load under
1.2), results within 1-2% over three runs. The spike:

- captures every step's contacts from the step on arrays (bit for bit the
  mod's), bodies turning, from the first step;
- replays them 5 times, colored afresh (the solver's own coloring,
  asserted to give `solver::order`'s order) and kept (Box2D's rule, each
  contact keyed by its ends, since every static is one body to the
  solver);
- checks every kept and afresh coloring valid, every step of the windows
  timed;
- times the solve handed each coloring, through a copy of `solver.rs` with
  one line patched by a genrule so that it can take colors.

It uses no unsafe code but `tests/pool.rs`'s, the benchmarks' existing
pool. Phase 1 below would need none either.

**Coloring, µs a step** (the median of 5 replays, the mean over each
window of 60 steps):

| stage | pile 10 000 falling (steps 2-61, 4674 contacts) | pile 10 000 settled (401-460, 22 012) | pyramid 5050 (601-660, 14 950) |
|---|---|---|---|
| afresh: coloring (the solver's 43) | 8.9 | 39.7 | 26.1 |
| afresh: placing in batches (the solver's counting and placing, 55) | 9.0 | 40.8 | 27.7 |
| kept: the changes (ended contacts free colors, begun ones take them) | **2.0** | **0.0** | **0.0** |
| kept: the walk that finds what began and ended (the merge's walk, which the mod makes anyway) | 13.8 | 39.1 | 28.7 |
| kept: placing, one thread | 8.9 | 40.9 | 27.5 |
| kept: placing across 8 threads (two runs of the pool) | 7.5 | 18.7 | 15.2 |
| contacts begun / ended a step | 304 / 63 | 0.1 / 0.1 | 0 / 0 |
| persisting contacts whose afresh color changed, a step | 761 | 12 | 0 |

**Colors** (at most in the window; sizes at its last step):

| | afresh | kept | kept, repacked every 60 steps |
|---|---|---|---|
| pile falling | 7: 3334 … 497, 8 | 9: 2956 … 273, 9, 1 | 9 (repacked at 60, then 61 adds one): 3295 … 527, 22 |
| pile settled | 7: 4906, 4968, 4566, 3991, 2713, 854, 15 | 9: 4244, 4316, 3983, 3671, 3095, 1966, 678, 56, 4 | 7: as afresh |
| pyramid 5050 | 6: 2401 to 2575 | 6: as afresh | 6: as afresh |

No coloring overflowed 64. The kept colors fragment only where contacts
began in another order than pair order: the pyramid's all began at once.

**The merge's walk over a table kept in color order** (9 runs merged by
pair) against pair order (one run): pile settled 212 against 17 µs, falling
121 against 11, pyramid 71 against 12.

**The solve on each coloring**, and **the pass over pages**: in (a) and (c)
above.

## Recommendation

**Keep solving on the copy, and move the three things the ECS doesn't see
into it one at a time, each only as far as a measurement pays for it.**
Colors go first, since they are the one thing that is state; then the
staged run, which any system could use; then colored iteration, when
something besides physics needs it.

### Phase 1: contacts' colors in the world (physics; get-emj.74)

- `SolveColor` on each contact, taken when it begins, packed again every
  `repack` steps (a `Tuning` field; 60 measured), masks derived each step
  in the solve's gather (see (a)).
- Placing by counting sort across the threads, the bodies' states and
  atoms built by the threads too.
- **Needs:** the quality suites, the matrix and baselines rerun, since
  every result moves once; the repack interval chosen by what it does to
  the colors' count and the solve's time; the determinism tests unchanged
  (any valid coloring is bit for bit across threads).
- **Gain:** measured 5% of the solve at 8 threads with the coloring alone
  (879 → 838, 585 → 556); projected 9% with placing split; the serial part
  from about 140 µs to a few. It matters more at 40 000 bodies, where the
  serial coloring and copy were as long as the solve on 16 threads
  ([Parallel solving](physics.md#parallel-solving)).
- **No unsafe code, no ECS change.**
- **Not worth it** if the rerun quality suites move a bound the wrong way:
  then colors stay a function of the contacts, and the serial part stays.

### Phase 2: the staged run in `engine_ecs` (rejected as written)

**Rejected by the user, 2026-10-02.** As written, phase 2 is a general API
that lets any system take the host's pool and run its own parallel passes
on a copy of world data. The scheduler sees none of that: `Workers`
declares nothing. That makes physics's workaround the pattern for every
system, when parallel work should go through what the ECS provides.

**What replaces it** (decided by the user, 2026-10-02, get-znt.28):
parallel work is declared and run by the scheduler, never by a system. The
three spikes since settled the copy's place: storage shouldn't own it
(working-sets.md), contiguous columns don't remove it
(contiguous-columns.md), and as declared flows it costs nothing at 8
threads (flows-spike.md). A system declares the shape of its work (a map,
a fixed-order reduction, a colored staged run over a flow), and the
scheduler turns it into tasks on its own threads (get-znt.29). The
protocol did move into `engine_ecs`, as the dispatch behind `Passes`
(threads.md, "Dispatch"), not as a call any system can make. Kept below
for the record.

The proposal: move `lanes::run_across`'s protocol into `engine_ecs::par`: stages run in
order within one run of the executor, each cut into blocks, any thread
taking any block (claimed by `fetch_max`), a count a stage, spinning with
yields, no main thread, and a panic ending every wait:

```rust
// Runs `stages` in order across the workers: `blocks(s)` blocks in stage
// `s`, each `run(s, block)` once, a stage's blocks all done before any of
// the next starts. The same result at any thread count if each stage's
// blocks are independent.
workers.stages(stages, |s| blocks(s), |s, block| run(s, block));
```

Physics's solve becomes its first user, with nothing moved but the code
(the solver's determinism tests are its tests), and 3D's solver across
threads (get-emj.75) its second. Safe code: atomics and a lock a block, as
now. It should be built once the host pool is (get-znt.20), since a
staged run on threads spawned per run (`Scoped`) pays 110 µs of spawning.

### Phase 3: colored iteration over a relation (deferred)

`Colored<R>` as in (b), with the kept colors of phase 1 generalized into
`engine_ecs`, when a game has a relation of thousands of rows whose
system needs threads: joints, springs or cloth. It starts with a decision
by the user on the sharing mechanism: atomic views of a column (a small
unsafe cast, with Miri), or only the safe reduction for accumulating
relations.

**Likely superseded (2026-10-02).** A relation of thousands of rows whose
system needs threads would gather into a flow and run a colored staged
run over it (`Colored::passes`, flows-spike.md), as the solve does: safe
code, since the flow owns its memory, and as fast as the hand-tuned solve
at 8 threads. In world columns the same walk measured 17 to 40% slower
(contiguous-columns.md). What would be left of this phase is relations
that make one cheap pass, where a copy costs more than the work.

### Not recommended

- solving in world storage (c): measured 10% slower to save 11 µs;
- keeping the contacts' table in color order: the merge 12 times slower;
- colors as an exact live relation: the exact coloring isn't stable under
  change;
- entity-level footprints in the scheduler: nothing needs them;
- phase 2 as written, a staged run any system can use on a copy: rejected
  (2026-10-02), replaced by declared shapes the scheduler runs
  (get-znt.28).

## Open questions

- **Open question:** how to repack. Every N steps is simple and
  deterministic. Repacking when the last color holds less than a block per
  thread would repack only when it pays. Either makes results depend on
  when it last ran, which the world then carries.
- **Open question:** whether per-body color masks should be stored (a
  component on bodies, changed at the merge's spawns and despawns, as
  Box2D's body sets) rather than derived in the gather each step. Deriving
  them is a parallel pass of about the coloring's size; storing them makes
  the merge write bodies.
- **Open question:** joints. Box2D colors joints and contacts in one graph.
  When joints exist (none do yet), they should take colors from the same masks,
  which argues for stored masks.
- **Open question:** a cap on colors with an overflow, as Box2D's 12 (v3.1.1)
  or 24 and Rapier's 120. None of the scenes here passed 9, and the
  repacked 7.
- **Open question:** whether a canonical coloring exists that is both a
  function of the contacts and stable under change (local, by pair, like a
  hash), without doubling the colors. It would make colors a live relation
  and keep results independent of history. Not tried.
- **Open question:** for phase 3, whether `AtomicU32::from_mut_slice` (or a
  stable equivalent) removes the need for an unsafe cast.

[^workers]: *(History, 2026-10-04.)* When this was written the entry
    point was `Workers`, a parameter that declared nothing and handed out
    the executor the host installed (`engine_ecs::par`), with
    `par_for_each` and the page walks split over it. get-znt.31 moved
    every split onto a shape and deleted it (2026-10-03).

[^spike-code]: *(History, 2026-10-02.)* The spike's code was removed once its findings were written here: spikes are built to answer a question and then thrown away. Every spike target and command named in this doc builds and runs at commit `c72e8b2` (`git checkout c72e8b2`), the last commit with every spike building.
