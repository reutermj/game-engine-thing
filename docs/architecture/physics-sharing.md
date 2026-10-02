# Physics in 2D and 3D: what to share

**Status: accepted; phase 1 built** (proposed 2026-09-29, its beads
get-emj.77 to get-emj.87; phase 1, `physics_common`, built 2026-10-02 with
two of its items left, see "Phase 1"). Before it, `//engine/std/physics2d`
(2D) and `//engine/std/physics3d` (3D) shared no physics code: each had its
own math, narrowphase, solver, settings and contact handling, over one ECS
whose spatial storage (`const D`) and live relations (`Live<R>`) are
generic over the dimension. The analysis below is as of `f5df2da`; the
parity table is kept current. Features have moved
between them by hand, and some have not moved at all. This doc asks what
could be shared, what should be, and how to stop the drift. Everything in
it was read from the code at `f5df2da`, not from the other docs, and the
prior art from the fetched sources where we have them.

**In short:**

- **The drift is not mostly duplicated code.** What both mods copy today
  is about 200 lines each (`Slots`, `Softness`, `Closing`, the settings
  plumbing, the contact merge). The larger gap is about 900 lines of
  dimension-independent code that 2D has and 3D lacks, written against
  2D's types: sleeping (about 380), sensors, overlaps and events (about
  200), the lanes and coloring machinery (about 300). And there are
  decisions made in one mod and never re-measured in the other: the warm
  start a turning point carries, how many restitution passes, how friction
  mixes, how rotation is locked.
- **Keep two mods, two interfaces, two solvers.** Share what is
  dimension-independent through an implementation-only crate,
  `physics_common`, linked into both mods (not into their interfaces), so
  a change to it reloads the two physics mods and no game. Keep a parity
  table (below) current as the record of every difference and why.
- **Don't make the solver generic yet, and don't build one source twice.**
  The kernels differ in their algorithms, not only their math (friction
  per point against a disc and a twist; two points against four;
  recycling), 2D's locked path must stay bit for bit for the games'
  replays, and one interface built twice would put every 3D experiment
  into the build and reload of every 2D game. Revisit a generic kernel
  once 3D solves in lanes too, when the two can be compared shape for
  shape.

## Parity, 2D against 3D

Read from the code. "Same" means the same algorithm, whatever the math.
A bead in the last column tracks the gap; beads marked new were filed
with this doc.

| area | 2D (`physics`) | 3D (`physics3d`) | same? | bead |
|---|---|---|---|---|
| **shapes** | box, circle | box, sphere | same set, per dimension | – |
| rotation | `Rotation` (cos, sin) and `Spin`; first order, normalized every substep; no cap on a step's turn | `Rotation` (quaternion) and `AngularVelocity`; first order, normalized every substep (variants: once, exact); at most a quarter turn a step | same rule, 3D caps the turn | – |
| inertia | a box's `m (w² + h²) / 12`, a disc's `m r² / 2`, a scalar | a diagonal about the body's axes, turned to a world `Mat3` once a step | per dimension | – |
| statics | a collider without `Body` and `Velocity` (or `STATIC`) | a `Static` marker | no | – |
| **broadphase** | `Live<Contacts>`, `FAT` 0.02, sides by `AnyOf` (awake with or without body or velocity, against statics and sleepers), pairs across workers | `Live<Contacts>`, `FAT` 0.02, moving against `Static`, one thread | same relation, 3D's sides simpler; `FAT` shared (`physics_common`) | – |
| bounds | `(Collider, Rotation)` extents | `(Collider, Rotation)` extents, the box as turned | same | – |
| **narrowphase** | unturned pairs: the old tests, no points (the tile-seam rule is box against box by relative velocity); turned: SAT over both boxes' faces and clipping, up to 2 points, `u16` feature ids | SAT over 15 axes with the last step's axis tried first, clipping, reduced to 4 (Box3D's area rule), `u32` ids; GJK/EPA as a variant | same family, per dimension; 3D caches its axis | – |
| manifold storage | `Manifold` (normal, depth, pressed, was pressed) and `ContactPoints` apart, written only where there are points | points inline in `Manifold` (40 words), with the rotations and move for recycling | no, each measured | – |
| contact recycling | none: every pair found every step | box pairs carried while they move under 0.03 (Box3D's), halving the narrowphase on piles | 3D only | get-emj.44 |
| speculative contacts | within `MARGIN` 0.05, a gap may close in a substep and no more | the same | same, `MARGIN` shared (`physics_common`) | – |
| **solver: soft step** | Box2D v3's: 5 substeps (`Tuning`), 2 relaxing passes, stiffness 0.25 and static 0.5 of the substep rate, damping 10, push at most 3 | the same structure, 5 and 2, stiffness 0.2 and static 0.25 (a turning cube rocks on four points: physics.md, "Still at rest") | same structure, stiffness measured apart; `Softness` (`b2MakeSoft`), damping, push and bounce threshold shared (`physics_common`) | – |
| restitution | once after the substeps, from the closing speed before the step's gravity (`Closing::Before`), above 1.0, **four passes** over a contact's points | the same closing speed, **one pass** over up to four points | closing shared (`physics_common::Closing`; 3D's `Tuning` names four of its six options, its own enum in the interface mapped onto it), passes not | get-emj.82 |
| friction | per point, in the relaxing passes only, clamped by the point's normal impulse | per contact at the points' centroid, clamped to a disc, and twist about the normal, relaxing passes only | no | – |
| friction mixing | the smaller of the two | `sqrt(a b)` (Box2D's and Box3D's; Rapier averages) | **no, and not decided**; restitution the larger in both; neither in `physics_common` until get-emj.81 decides | get-emj.81 |
| warm-start carry | turning points: normal from the last substep, tangent averaged (`Carry::Normal`, decided get-emj.61); a contact whose ends don't turn: the mean | the mean of both (`Carry::Mean`); `Last` a variant; B never tried | **no** | get-emj.82 (new) |
| warm-start matching | by feature id (`ContactPoints::last`, in the interface) | by feature id (`lib.rs`, `warm`); nearest point within 1 cm a variant | same, not shared: 2D's is a method of an interface component, which can't use `physics_common` | get-emj.91 |
| order, lanes, threads | turning: Box2D's colors (`engine_ecs::shape`'s `Coloring::greedy` and placing since flows, `Passes` running the program), four lanes, across the host's threads bit for bit (`solve_across`); by level a variant; locked: one contact at a time in pair order | one contact at a time in pair order | 2D only | get-emj.52, get-emj.75, get-emj.74 |
| the solve's systems (since 2026-10-02) | a pipeline of nine systems over five [flows](flows.md) (`pipeline.rs`), the colored passes on `Passes` | a pipeline of six systems over three flows (`pipeline.rs`), the solve whole in one system, no shape | same shape: settings, gathers, solve, scatters; 3D's solve isn't split, its pair order being its result | get-znt.33, get-znt.35 |
| block solver | a variant (2x2 over a contact's two points) | none | 2D only, a variant | – |
| **sleeping, islands** | islands by union-find over pressed contacts; `Asleep`, `Slept`, `Still`, `Resting` in the world; every wake a game can cause | none | 2D only | get-emj.77 (new) |
| **sensors, layers, overlaps** | `layer`, `mask`, `senses`, `sensor`; `Overlap` entities | none: every pair collides | 2D only | get-emj.78 (new) |
| **events** | `Contact` (began pressing, with speed), `Trigger` | none | 2D only | get-emj.78 (new) |
| **pre-solve hooks** | `Response` per contact (friction, restitution, disabled), changed between `find_contacts` and `solve` | none: mixing is in `Manifold` | 2D only | get-emj.79 (new) |
| **settings** | `Gravity`, `Sleep`, `Tuning { substeps }` in the world; every other choice (`Params`: carry, closing, wide, block, separation) only in the arrays' variants | `Gravity` and a `Tuning` holding every measured choice, set by writing it, parsed from `sub=4,warm=cold` | same mechanism, 2D exposes one field | get-emj.85 (new) |
| **locking rotation** | by absence: no `Spin`, and no `Rotation`, takes the no-points path (a `Rotation` on every locked body cost 11-32%) | by zero inverse inertia; every body has a `Rotation` and pays for points (a locked 10 000 box pile 29 ms, where translation only took 4.9) | no | get-emj.80 (new) |
| kinematic bodies | `KINEMATIC`: moved by velocity, never pushed, wakes what it pushes | none | 2D only | get-emj.79 (new) |
| game surface | `Touching`, `gravity_scale`, `Spatial` queries (overlapping, any_at, cast) | none | 2D only | get-emj.79 (new) |
| **reload coverage** | the games' replays (nothing turns), one v1 to v2 swap under a locked pile, the sleeping reload tests; nothing reloads the turning, colored, threaded path | a turning pile replayed bit for bit while physics3d, the scene and the scheduler reload every frame | 3D's is stronger | get-emj.83 (new) |
| **tests** | 39 unit, 54 in the engine and on spatial queries, 36 quality and 51 behaviour against Box2D and Rapier, a baseline, equivalence of lanes, colors and threads bit for bit | 27 unit, 14 in the engine (reload included), 17 quality and 24 behaviour against Rapier, Jolt and Box3D, a baseline | same kinds; the harnesses are copies (being merged into a shared test kit) | – |

**Where the two differ on purpose, with a measurement behind it:** the
stiffness, manifold storage, recycling (3D first; 2D open), the tile-seam
rule, the friction model (a disc and a twist have no 2D counterpart). **Where
they differ because one was built after the other forked, and nobody
measured the other:** the carry, the restitution passes, friction mixing,
the rotation lock, the settings a world can set. Those are the rows the
table exists to catch.

## What each piece is

Every piece of both mods in one of three classes:

- **(a) dimension-independent:** the same logic in 2D and 3D, shareable
  as it is or with a small generic parameter (a component type, a vector
  type used only to carry values);
- **(b) the same algorithm over different math:** shareable only through
  a dimension-generic math layer (a `Dim` trait, or `const D`);
- **(c) different:** algorithms that exist in one dimension only.

Lines are code lines: neither blank nor a `//` comment, outside
`#[cfg(test)]` modules.[^count] 2D is 3890 of them in seven files, 3D 1926
in six.

### 2D (`engine/std/physics2d`, 3890)

| piece | where | lines | class |
|---|---|---|---|
| sleeping: islands by union-find | `sleep.rs` | 76 | (a), already free of 2D types |
| sleeping: the tables, what wakes an island, falling asleep, `load` | `lib.rs` (`wake_by_games`, `move_woken`, `wake_on_statics`, `fall_asleep`, `resolve`, the queries) | ~300 | (a), written against 2D's components |
| layers, sensors, overlaps and `Trigger`; the contacts' write-back (impulses, pressing, sleep links, `Contact`, `Touching`) | `lib.rs` (`collides`, `senses`, `awake`, the overlap merge, `wrote`) | ~260 | (a), but for `Touching`'s four sides |
| the contact merge with the world, serial and across workers | `lib.rs` | 86 | (a) |
| `Slots`, mod state, timings, messages, systems, the `Contacts` relation and its pairs, the `Gang` for `Workers` | `lib.rs` | ~200 | (a), a pattern both repeat |
| gathering bodies and colliders, writing bodies back, `meet`, `item` | `lib.rs` | ~275 | (b) |
| settings: `Params`, `Closing` (with its formula), `Carry`, `Wide`, the constants | `solver.rs` | 113 | (a) |
| entry points (`solve_with`, `solve_across`) | `solver.rs` | 59 | (a) |
| lanes: `F<N>`, coloring and ordering (`group`, `order`), `place`, blocks and the staged run without a main thread | `solver.rs` (`lanes`) | ~300 | (a) |
| `Softness` (Box2D's `b2MakeSoft`) | `solver.rs` | 10 | (a) |
| bodies, constraints, points, rows | `solver.rs` | ~125 | (b) |
| the solve one contact at a time, with the block solver and separation variants | `solver.rs` (`solve_all`, `prepare`, `pass`, `block`, `bounce`) | 377 | (b) |
| lanes: states, shared atoms, batches, gathers, setup, run, finish, the kernels | `solver.rs` (`lanes`) | ~810 | (b) |
| `Vec2`, `Rot` | `shapes.rs` | 112 | (b), the math layer |
| `Shape`, `Placed`, `Aabb`; `Spatial` | `shapes.rs`, `spatial.rs` | 124 | (b) |
| overlap, separation and ray tests | `shapes.rs` | 145 | (c) |
| narrowphase: axis-aligned tests with the seam rule, SAT and clipping of turned boxes | `narrow.rs` | 195 | (c) |
| components: `ContactPair`, `Overlap`, `Response`, `Sleep`, `Tuning`, `Asleep`, `Slept`, `Still`, `Resting`, `Trigger` | `components.rs` | 108 | (a), but component names are the mod's |
| components: bodies, colliders, `Manifold`, `ContactPoints`, `Impulse`, `Gravity`, `Contact` | `components.rs` | 199 | (b) |

**(a) 1525 lines (39%), (b) 2025 (52%), (c) 340 (9%).**

### 3D (`engine/std/physics3d`, 1926)

| piece | where | lines | class |
|---|---|---|---|
| `Tuning` and its enums, codes and parser; the solver's `Tuning` | `components.rs`, `solver.rs` | ~180 | (a) |
| mod state, timings, messages, systems, `Contacts`, `Slots` | `lib.rs` | ~125 | (a) |
| the contact merge; warm matching by id | `lib.rs` | ~60 | (a) |
| `Softness`, `closing` (2D's formula, copied) | `solver.rs` | 29 | (a) |
| `ContactPair` | `components.rs` | 9 | (a) |
| gathering and writing back, `stored`, `recycle`, `integrate_velocities` | `lib.rs` | ~220 | (b) |
| bodies, constraints, rows; the solve; rows, passes, friction on a disc and twist | `solver.rs` | ~350 | (b) |
| `Vec3`, `Quat`, `Mat3` | `math.rs` | 148 | (b), the math layer |
| components: bodies, colliders, `Manifold`, `Impulse` | `components.rs` | ~170 | (b) |
| manifold and point types | `narrow.rs` | ~30 | (b) |
| sphere tests, SAT over 15 axes with a cached axis, clipping, reduction to four | `narrow.rs` | ~350 | (c) |
| GJK and EPA, Jolt's way, kept to measure against | `gjk.rs` | 230 | (c) |
| solid inertia of a box and a sphere | `components.rs` | ~20 | (c) |

**(a) 402 lines (21%), (b) 921 (48%), (c) 603 (31%).**

### What the split says

- **Of 2D's 1525 lines of (a), 3D has about 200** (the plumbing both
  repeat, `Slots`, `Softness`, `Closing`, the merge). Most of 3D's own
  (a) is its settings, which are richer than 2D's. The rest of 2D's (a),
  about 900 lines, is features 3D doesn't have: sleeping, sensors and
  events, and the lanes and threads. That is what 3D would copy to reach
  parity, and what a copy would then drift from.
- **(b) is half of each mod, and its halves are not twins.** The soft step
  is one structure (substeps, a pushing pass, positions, relaxing passes,
  restitution once), but inside it 2D solves friction per point and 3D on
  a disc with a twist; 2D has two points and a block-solver variant, 3D
  four points and recycling; 2D's hot path is batches of four lanes, 3D's
  one row at a time. A generic kernel would be a trait with most of each
  mod's choices as associated items.
- **(c) is small in 2D and a third of 3D.** GJK in 3D is a measurement
  variant; the rest is what each dimension needs.
- **Commits say the same.** Since physics3d became a mod (2026-09-25), 15
  commits changed 2D's sources only, 10 3D's only, and 4 both: the
  broadphase through `Kept` and then `Live`, the quality tests, and the
  restitution closing speed (get-emj.56 in 2D, get-emj.60 in 3D, the same
  enum and formula written twice). Everything else that should have
  crossed, didn't.

[^count]: Counted at `f5df2da` by a script that splits each file at its
    last top-level `#[cfg(test)] mod` and counts lines that are neither
    blank nor start with `//`; the pieces by ranges of each file, so the
    "~" figures are to about ten lines.

## Prior art

Rapier, parry, Box2D, Box3D and Jolt were read in the copies the
comparisons fetch (runbook 005); Avian on its repository (`main`,
2026-09-29), since nothing fetches it.

- **Rapier 0.36: one source, two crates.** `rapier2d` and `rapier3d` ship
  the same `src` tree, file for file (`diff -r` finds nothing), built with
  the `dim2` or `dim3` feature. The math is type aliases chosen by
  feature: `Vector` is 2 or 3 wide, `AngVector` a scalar or a vector,
  `Rotation` a unit complex or a unit quaternion, angular inertia a scalar
  or a symmetric 3x3, all nalgebra types generic over the scalar (and over
  SIMD lanes through simba). Of its 239 files (80 000 lines), 114 have a
  `#[cfg(feature = "dim2")]` or `"dim3"`, 909 of them in all, most in the
  bodies, the constraint builders and the joints. parry does the same:
  344 files, 169 with a dimension `cfg`, 1516 of them. The builds differ
  beyond the math: `block-solver` is on by default in 2D and not in 3D.
  *Gains:* a fix or a feature lands in both, and neither can fall behind.
  *Pays:* half the files branch on the dimension, every change is
  compiled and tested twice, and a choice right for one dimension (the
  block solver) becomes a feature flag rather than a difference in code.
- **Box2D v3.1.1 and Box3D 0.1: two codebases, one author.** Erin Catto
  wrote Box3D (2025) from Box2D; nothing is shared at build time. Measured
  by matching lines (comments dropped, `b2`/`b3` made the same) between
  files of the same name: of Box3D's 43 400 lines, about 10 000 (23%) are
  lines of Box2D's 21 800. The copy is the bookkeeping: the bit set 89-94%,
  tables 78%, the dynamic tree 72%, id pools 67-71%, the broadphase, solver
  sets and sensors 60%, the constraint graph (the coloring) 58%, joints and
  bodies 52-54%, the solver's stages 45%, the world 38%. The math was
  rewritten: the contact solver 28%, distance 22%, the math functions 19%,
  hulls 2%, and the manifolds are new files (`convex_manifold.c`,
  `triangle_manifold.c`). And the drift runs one way, as ours does: Box3D
  recycles contacts (`physics_world.c`, "contact recycling optimization")
  and has SIMD for NEON and SSE2 in its contact solver, a scheduler and
  compounds; Box2D v3.1.1 has none of the recycling.
  *Gains:* each dimension is free to take its own shape. *Pays:* every
  fix to the shared quarter is made twice, by hand, or not.
- **Jolt 5.6: 3D only.** A 2D game is a 3D one constrained: a body's
  `EAllowedDOFs::Plane2D` allows x, y and a turn about z
  (`AllowedDOFs.h`). *Gains:* one engine. *Pays:* 2D pays for 3D in
  every body and pair. Measured here for storage: 2D in a 3D store at
  z = 0 costs `near_pairs` 13-16% and the lanes a fifth more memory
  (spatial-storage.md, "In 3D"), before any quaternion.
- **Avian: one source, two crates, as Rapier.** `crates/avian2d` and
  `crates/avian3d` each set `[lib] path = "../../src/lib.rs"` with a `2d`
  or `3d` feature, and `src/math` aliases `Vector` (`Vec2` or `Vec3`),
  `Rot` (`Rot2` or `Quat`), `AngularVector` (`f32` or a vector) and a
  `DIM` constant by feature, over parry2d or parry3d. It is not separate
  code over a shared crate. It is the closest to us, being ECS-first
  (Bevy): its components are the same names in two crates, `avian2d`'s
  `Position` and `avian3d`'s being different types, and a game links one.

What this says for us: every engine that shares 2D and 3D shares them
whole, by compiling one source twice, and the one that doesn't copied its
bookkeeping and rewrote its math, which is (a) against (b) and (c) above.
None shares at the level of a generic kernel; the nearest is Rapier's
SIMD code generic over the scalar and lane count, not the dimension.

## Options

What each costs us, in the terms this engine cares about:

- **The ECS-first design.** Components are dimension-specific types in
  each mod's interface (`component!` has no generics, and a component's
  name is its mod's: `physics2d::Asleep`, `physics3d::Asleep`). Interfaces
  are digested per mod, and a mod depending on physics reloads when its
  interface changes (mod-deps.md). A 2D interface change rebuilds 72
  actions and reloads 23 game mods; a 3D one, 17 and none of them
  (physics.md, "A mod").
- **Hot reload.** Whatever is shared is linked into each mod's library and
  swapped with it; it may hold no statics (a new image's statics start
  over: physics.md, "A mod").
- **Bit for bit.** 2D's arrays twin (`tests/arrays.rs`, `:tax`, the
  comparison) compiles `solver.rs` and `narrow.rs` by path; the colors
  spike patches `solver.rs` with `sed` and fails its build if the line
  moves; the lanes are held to the one-contact loop, the colors to their
  order, the threads to one thread, and pong's and the platformer's replays
  to the scalar locked path, all bit for bit. Rust neither reassociates
  nor contracts `f32` arithmetic, so moving code between crates keeps its
  values; it can move its speed (the lanes lean on LLVM's SLP vectorizer,
  and keeping `Live`'s update out of line was worth 4%), so `:tax` and
  `:solver_bench` are rerun with any move.
- **Drift prevented**, against what happened: the closing speed ported by
  hand; `Slots` and `Softness` copied; sleeping, sensors, events and the
  lanes not ported; the carry, the bounce passes, friction mixing and the
  rotation lock decided in one mod only.

### (i) Separate, with a parity table kept current

What there is, plus the table above in the tree and a rule that a change
to a row updates it (get-emj.84).

- ECS, reload, bit for bit, build: unchanged.
- Drift: *seen*, not prevented. The table would have shown the
  carry, bounce passes, mixing and lock rows as they happened; every
  feature still crosses as a copy (about 900 lines for 3D to reach 2D),
  which then drifts on its own.
- Cost: a habit, which is what failed so far.

### (ii) A `physics_common` crate for class (a)

A plain `rust_library` in `//engine/std/physics_common`, a `deps` entry of
both mods' implementations and of the arrays twin and the 3D bench, never
of an interface.

- ECS: unchanged. Components stay in each interface; the common code
  takes plain values (entities, indices, scalars) or is generic over a
  small trait. `engine_mod`'s digests cover the interface sources alone
  (`_mod_links`), so a change to the crate is an implementation change of
  both mods: they reload, no game does. A component both mods declare the
  same way (the sleep tables, `Overlap`) could come from a macro in the
  crate taking the mod's name, but that is an interface change of both,
  so it stays optional.
- Hot reload: unchanged (statically linked into each library, no statics).
- Bit for bit: the twin gains a dep; the moves change no arithmetic, and
  the equivalence tests and replays are how that's checked. The spike's
  `sed` moves with `group`.
- Build: one crate of a few hundred lines; a change to it rebuilds both
  mods, their twins and benches, about the 17 actions of a 3D change plus
  2D's implementation, and no game crate.
- Drift: prevents it for what both have (the closing speed would have
  been one change; mixing couldn't have split). For what only 2D has, it
  turns 3D's copy into glue, if the bookkeeping can be written over a
  trait naming each mod's components: sleeping and sensors are systems
  with queries of 2D's types, so they need systems generic over the
  mod's component types, which is unproved here (get-emj.87 is the spike).

### (iii) Dimension-generic code for (a) and (b)

A `Dim` trait (vector, rotation, angular vector, inertia, max points) or
`const D`, as spatial storage has, with the soft step, the rows and the
manifolds written once over it.

- ECS: as (ii). Components can't be generic, so each mod keeps its glue
  and its components; the generic code sits between them.
- Hot reload: unchanged.
- Bit for bit: the risk. 2D's locked scalar path must stay the games'
  replays to the bit, and 2D's lanes kernels are shaped for the
  vectorizer; a generic rewrite has to reproduce each operation's order.
  Storage went generic at no measured cost because its only dimensional
  code is a loop over axes; the solver's dimensional code is its
  algorithms.
- Build: more monomorphization, small; but every change to the kernel is
  a change to both mods, and must pass both mods' suites.
- Drift: prevents it in the kernels too (the carry and the bounce passes
  would be one parameter each). But the kernels differ in friction
  (per point against a disc with a twist), point count (2 against 4, with
  a block-solver variant in one and recycling in the other) and hot path
  (lanes against rows), so the trait would carry most of each mod's
  choices, which is Rapier's 909 `cfg`s in another spelling.

### (iv) One source, two builds (Rapier's and Avian's)

One set of sources, `engine_mod(name = "physics")` and
`engine_mod(name = "physics3d", crate_features = ["dim3"])`, interfaces
included.

- ECS: two interfaces from one source. A change to a shared interface
  file is an interface change of both: every 3D experiment rebuilds the
  72 actions and reloads the 23 mods of both games, which is what "A mod"
  measured and rejected for a common interface.
- Hot reload: unchanged.
- Bit for bit: the twins build with the feature; 2D's special paths (the
  tile-seam rule, the unturned no-points narrowphase, the block solver,
  `solve_all::<false>`) and 3D's (recycling, the cached axis, the variants
  in `Tuning`) become `cfg` branches, each needing its own replays.
- Build: every change compiles and tests twice.
- Drift: none possible. But it couples an experimental mod that changes
  daily to the one both games stand on.

### The options side by side

| | games rebuilt and reloaded by a shared change | bit-for-bit risk | drift it would have prevented | cost |
|---|---|---|---|---|
| (i) separate, table | none | none | none; it shows the decisions that split | a habit |
| **(ii) `physics_common`** | **none** | **none: code moves, arithmetic doesn't** | **the copies (closing speed, `Slots`, `Softness`, mixing); with the spike, the features 3D lacks** | **one crate; a spike for generic systems** |
| (iii) generic kernels | none | high: the locked path and the lanes | the kernel decisions too (carry, bounce passes) | a trait carrying most choices |
| (iv) one source | all of both games' | high | all | an experiment coupled to both games |

## Recommendation

**(i) and (ii) now, in phases; (iii) for the kernels only once both solve
in lanes; not (iv).** Each phase must move no value: `:tax` bit for bit,
pong's and the platformer's replays, physics3d's reload replay, both
baselines and every equivalence test unchanged.

### Phase 0: the parity table (get-emj.84)

This doc's table is the record. The rule, for the user to add to
CLAUDE.md: a change to either physics mod that adds, removes or re-decides
a row updates the table in the same commit, and a feature or decision
bead for one dimension names the other's bead or says in the table why it
doesn't apply. The decisions that split without anyone deciding go to
beads now: friction mixing (get-emj.81), the carry and the bounce passes in
3D (get-emj.82), 3D's rotation lock (get-emj.80), 2D's reload replay of the
turning path (get-emj.83).

### Phase 1: `physics_common`, for what both have (get-emj.85)

`Slots`, `Softness`, `Closing` and its formula, the mixing rules once
get-emj.81 picks one, the constants both use (`DAMPING_RATIO`, `MAX_PUSH`,
`BOUNCE_THRESHOLD`, the speculative `MARGIN`, `FAT`), matching points by
feature id. About 60 to 90 lines out of each mod, one crate in. This is small
on purpose: it proves the crate, the twin and the benches depending on it,
and that nothing moves, before anything larger rides on it.

**Built 2026-10-02, but for two items.** `//engine/std/physics_common` is
a `rust_library` in the `deps` of both mods (their twins and 2D's `v2`
included), and of everything that compiles their sources by path: 2D's
`core_test`, `:tax`, `narrow_bench`, `parallel_solver` and the
comparison's targets, 3D's `core_test`. In it: `Slots` (with its
`Recycle`), `Softness`, `DAMPING_RATIO`, `MAX_PUSH`, `BOUNCE_THRESHOLD`,
`MARGIN`, `FAT` and `Closing` with `speed`, each moved in a commit of
its own. The mods keep their old paths (`solver::BOUNCE_THRESHOLD`,
`narrow::MARGIN`) by `pub use`, so the comparisons didn't change. 3D's
`Closing` stays in its interface (its `Tuning` names it) and is mapped
onto the shared one. Every move moved no value: `:tax` bit for bit, both
baselines' `--all` output and `--long --all` output identical to the
commit before, pong's and the platformer's replays, physics3d's
`reload_test`. A change to the crate rebuilds physics2d's and physics3d's
libraries and no game crate (measured: a constant added to it rebuilt the
one cdylib `//game` has from it), and `bazel run //engine/std/physics2d`
reloads the mod under a running `//game`, its dependents untouched.

Left, and why:

- **Mixing:** friction still mixes by the smaller in 2D and by `sqrt(a b)`
  in 3D (get-emj.81, undecided); moving either would decide it. Restitution
  is the larger in both, but moving it alone splits the rule from its pair.
- **Matching by feature id:** 2D's is `ContactPoints::last`, a method of a
  component in 2D's interface, which can't depend on this crate; moving it
  out is an interface change of physics2d (every game rebuilt and
  reloaded), and its callers include the comparison's spikes. The two
  copies are one `position` over the ids each, and agree (get-emj.91).

### Phase 2: coloring and lanes (get-emj.86, with get-znt.21)

`F<N>`, Box2D's greedy coloring over each contact's two body indices
(`group`, `order`), placing, and the block sizes into `physics_common`;
the staged run into `engine_ecs::par` as parallel-relations.md's phase 2
already proposes (get-znt.21). About 300 lines of 2D's solver become
common. Then 3D's lanes (get-emj.52) and threads (get-emj.75) are a port of
its kernels only, and kept colors (get-emj.74) are built once for both.

**Rescoped by flows (2026-10-02, get-emj.86's note):** the coloring and
placing now live in `engine_ecs::shape` (`Coloring::greedy`, asserted
equal to 2D's `lanes::group` contact for contact; `Passes` packs the
colors and runs the program), and the staged run is the scheduler's
(get-znt.28), not `engine_ecs::par`, so get-znt.21's plan is superseded.
What is left for `physics_common` is `F<N>`, the lane array, and whatever
lanes helpers 3D's kernel shares with 2D's; 2D's `lanes::group` should
then give way to `shape::Coloring` if that is bit for bit.

### Phase 3: the bookkeeping 3D lacks (spike get-emj.87)

A spike of systems generic over a trait naming a mod's components
(position, body, the sleep tables), with each mod's own `component!`
declarations. If `Systems` takes such a system and the footprints and
2D's values come out the same, sleeping (get-emj.77) and sensors, overlaps
and events (get-emj.78) are built for 3D on 2D's code, about 580 lines
shared instead of copied. If it doesn't, record why here, and 3D copies
with the table as its check.

### Later: the kernels

Once 3D solves in lanes, compare the two kernels shape for shape: if what
differs is the math (a vector, a rotation, a point count) and not the
algorithm, a `Dim` trait for the lanes kernels is option (iii) at its
cheapest. Until then, the kernels stay each mod's own, with their
decisions in the table.

**Not recommended:** one source built twice (iv), which couples 3D's
experiments to both games' builds and reloads; a shared interface for
`Position` or `Velocity`, measured in physics.md ("A mod") and rejected for
the same reason; 2D as constrained 3D (Jolt's way), which costs 2D 13-16%
in the broadphase before any of the solver.

The test harnesses (the comparisons' scenes, measures, families and
baselines, once copies in `engine/std/physics/compare` and
`bench/physics3d`) are the testing side of the same problem, now merged
into a shared test kit (`//engine/std/physics_testkit`) apart from this doc.

## Open questions

- **Open question:** does `engine_api`'s `Systems` accept a system generic
  over a trait of component types, and do its queries declare the same
  footprints as the concrete ones? Phase 3 hangs on it (get-emj.87).
- **Open question:** should components both mods declare alike (the sleep
  tables, `Overlap`, `Resting`) come from one macro in `physics_common`?
  It saves little and makes a change to them an interface change of both.
- **Open question:** 2D's recycling (get-emj.44) would be the first feature
  to cross from 3D to 2D. Its bound on a pair's move is (b); whether its
  bookkeeping (carrying a manifold, the move since found) is common is for
  whoever builds it.
- **Open question:** when both kernels are in lanes, is the difference
  math or algorithm? That decides (iii).
