# Scheduling

What runs when within a frame, and how mods tell each other that something
happened. The unit of scheduling is a **system**: a function a mod
registers, with a phase, ordering constraints, and the components it reads
and writes. The code is in `engine/api/system.rs` (the declarations mods
make), `engine/ecs` (the parameters, and the footprints that decide what
may run together), `engine/loader/schedule.rs` (turning declarations into
an order) and `engine/loader/engine.rs` (running a frame). How a frame's
structural changes and events reach the world is
[storage.md](storage.md).

Decided 2026-09-23, as the first of four steps toward multithreading (see
[Toward parallelism](#toward-parallelism)). Everything here runs on one
thread; it is shaped so the later steps don't change what mods write.

## Systems

A mod declares its systems in `Mod::systems`, which every build runs when
it's opened, before it's swapped in:

```rust
impl Mod for Physics {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("integrate", Self::integrate).phase(phase::SIMULATE);
    }
}

impl Physics {
    fn integrate(&mut self, _: &mut (), _: &mut Cx, mut q: Query<(&mut Position, &Velocity)>) {
        q.for_each(|_, (position, velocity)| position.x += velocity.x * DT);
    }
}
```

A system is a function of the mod's state, its transient part, its `Cx`,
and up to eight **parameters**. Its name is qualified with the mod's
(`physics::integrate`). It must not capture anything: a fn item or a
closure without captures, because each build's systems are that build's
code, referenced by the loader until the next reload replaces them.

A mod's per-frame work is its systems and nothing else. One that declares
none (it only declares components, or only takes messages) runs nothing
each frame. A bootstrap's session isn't a system either: it's
[`Bootstrap::run`](overview.md#who-runs-the-loop), called once.[^step]

### Parameters declare access

A system's parameters are both what it reads and what it declares, so the
declaration can't drift from the code:

- **`Query<Data, Filter, Changes>`**: every entity with `Data`'s
  components, `&T` or `&mut T` or a tuple of up to eight, that passes
  `Filter` (`With<T>`, `Without<T>`). `Changes` declares the structural
  changes the system may make to the rows it yields: `Adds<T>`,
  `Removes<T>`, `Despawns`. Iterated with `q.for_each(|row, items| ..)`,
  looked up with `q.with(entity, ..)`, or `q.single(..)` for a component
  there's one of.
- **`Spawner<B>`**: spawns entities with the bundle `B`'s components.
- **`EventReader<E>`**, **`EventWriter<E>`**: the events of type `E` this
  system hasn't seen yet, and sending them.

A system has no other way into the world. **`cx.world()` panics in a
frame**, failing the system's mod: the whole world is for hooks and message
handlers, between frames. So a system's footprint is exactly its
parameters, which is what lets the scheduler decide what may run together,
and it's why a service called from a system can't touch the world either
(storage.md, "Services don't touch the world"). Two queries of one system
that would take conflicting guards (both on one component, one writing) are
refused when it's added, unless their filters keep them apart: one requires
a table-stored component the other excludes, as in `Query<&mut Velocity,
With<Walker>>` beside `Query<&Velocity, (With<Player>, Without<Walker>)>`.
A system that needs to reach further (exclusive systems) is deferred until
something needs one.

### Structural changes go through rows

`row.insert(value)`, `row.remove::<T>()` and `row.despawn()` on the rows a
query yields, as its `Changes` declare, and `spawner.spawn(bundle)`, go
into the system's log, which its apply node applies after it returns. The
system never sees its own changes; every system after it in the plan does,
the same frame. See [storage.md](storage.md#structural-changes-go-through-query-rows),
with a walkthrough. Hooks and message handlers change the world directly
through `cx.world()`, since nothing else runs then.

## Phases and order

A frame runs the phases in order, and each phase's systems in order. The
engine's phases are, in order:

| Phase      | For |
|------------|-----|
| `input`    | turning input (events, messages) into intent |
| `update`   | game logic, and the default |
| `simulate` | physics and other integration over the frame's intent |
| `late`     | reacting to the frame's outcome: cameras, cleanup |
| `render`   | producing output from the settled world |

A mod adds a phase relative to others: `s.phase("pong::serve")
.after(phase::SIMULATE).before(phase::LATE)`.

Within a phase, systems are ordered by their `.after(..)` and `.before(..)`
constraints, which name systems (`"input::apply"`). The rest ties on load
order, then declaration order, so the plan is deterministic and a lockstep
game replays exactly. Phases and constraints only order systems: nothing
waits at a phase's end, and a system waits only for the earlier nodes it
overlaps with. A constraint naming a system that isn't loaded is
ignored: mods come and go at runtime. A constraint across phases must agree
with the phase order.

The plan is rebuilt whenever the set of builds changes, and checked before
a load commits: a load that would make the constraints cyclic is refused
and leaves the running builds untouched, as a broken interface is.

`modctl schedule` prints the plan.

## Events

An event is a value one mod sends and others react to, without either
depending on the other's state or calling into it. It is the answer to a
provider that needs to reach back to its callers, and to a resident layer
that needs something from gameplay.

```rust
engine_api::event! {
    pub struct Scored: "pong::Scored" { pub by_player: bool }
}

fn score(&mut self, _: &mut (), _: &mut Cx, scored: EventWriter<Scored>) {
    scored.send(Scored { by_player: true });
}

fn cheer(&mut self, _: &mut (), _: &mut Cx, mut scored: EventReader<Scored>) {
    for s in scored.read() { ... }
}

cx.send_event(Scored { by_player: true });        // outside a frame
```

Events follow the component rules (they're declared the same way, and live
in the loader's world), with these semantics:

- **A system's events are published by its apply node**, after it
  returns: readers after it in the plan see them the same frame, readers
  before it the next, as with a structural change.
- **Each reader sees each event once.** The world keeps a cursor per
  system and event type, by name, so it survives the system's reload.
- **An event lives until the end of the frame after it was published
  for**, so a reader before its sender still sees it next frame. Unread
  events are then dropped.
- **Outside a frame, `cx.send_event`** publishes for the next frame, so
  every reader sees it then. Pong's text interface turns `up` into an event
  that the `input` phase applies, so input lands at a defined point in the
  frame.

Sending is declared (`EventWriter`) because a reader later in the plan
has to know, before the sender runs, whether to wait for it.

## Who schedules

The loader owns what makes a frame safe: the declarations, the plan, the
apply nodes, and refusing to run a system whose mod is already running. When each system runs is policy, and belongs to a mod:
the one providing `engine_api::scheduler::Scheduler`, a service the engine
declares rather than any mod, so a bootstrap reaches whichever scheduler the
game loads without depending on it.

```rust
impl Scheduler for Sequential {
    fn run_frame(&mut self, _: &mut (), cx: &mut Cx) {
        let Some(frame) = scheduler::begin(cx) else { return };
        for node in &frame.plan().nodes {
            frame.run(node.id);
        }
    }
}
```

`scheduler::begin` opens a frame and returns its plan: every system and,
after each that may change the world, its apply node, in plan order.
Dropping the `Frame` closes it, including when the scheduler panics. The loader refuses a frame inside a frame. A bootstrap
calls `cx.run_frame()`, which calls the loaded scheduler, or, if none is
loaded (or it has failed), runs the loader's own sequential frame: what tests
driving an `Engine` directly get.

The engine ships defaults in `//engine/std`: `realtime` and `lockstep`
bootstraps, the `clock` they publish, and the `sequential` scheduler.
`engine_game` uses `realtime` and `sequential` unless told otherwise. A
game can replace either, or fold scheduling into its own bootstrap.

A scheduler that owns no threads isn't running between frames, so it
needn't be resident: `sequential` hot-reloads like gameplay. The parallel one
(step 2) will own worker threads, and so will be resident.

## Toward parallelism

The four steps, of which this document is the first:

1. **Systems, phases and access**, on one thread. The ABI change, and the
   one that gets more expensive with every mod. Built, then reshaped by the
   storage redesign in [storage.md](storage.md): phase boundaries became
   dependency edges, and commands became changes through query rows.
2. **System parallelism.** Systems whose footprints don't overlap run at
   the same time. `engine_ecs::harness` already runs frames that way, in
   tests and the benchmark; the loader's side is a resident scheduler mod
   owning the workers (get-znt.5). Each node takes its guards with
   `try_lock`, so a scheduler bug is a failed frame, never a data race.
3. **Data parallelism.** `par_for_each` over a query's chunks, with a
   restricted task context.
4. **Pipeline parallelism.** The next frame's simulation during this
   frame's render, through an extract step or double-buffering, decided with
   the renderer spike. A reload drains the pipeline first.

Before step 3: measure the mod boundary (get-8in), which decides whether
data parallelism must work on column slices only.

**Open question:** worker threads and `thread_local!`. glibc keeps a library
mapped while a thread has TLS destructors registered in it, so a reloadable
mod using TLS on a long-lived worker leaks its old builds. Recycling workers
at a reload's drain is the likely answer.

[^step]: *(History, 2026-09-23.)* Mods used to have one per-frame hook,
    `Mod::step`, which also served as the bootstrap's session. For the first
    step of scheduling, a mod without systems got an implicit exclusive
    `step` system, so older mods kept working. It was removed once every mod
    had moved to systems: an undeclared, exclusive hook is exactly what the
    parallel steps can't schedule, and sharing a name with the session made
    a mod that ran nothing per frame opt out explicitly.
