# Scheduling

What runs when within a frame, and how mods tell each other that something
happened. The unit of scheduling is a **system**: a function a mod
registers, with a phase, ordering constraints, and the components it reads
and writes. The code is in `engine/api/system.rs` (the declarations and the
typed API mods use), `engine/loader/schedule.rs` (turning declarations into
an order) and `engine/loader/engine.rs` (running a frame).

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
    fn integrate(&mut self, _: &mut (), cx: &mut Cx, q: Query<(&mut Position, &Velocity)>) {
        for (_, (position, velocity)) in q.iter(cx) {
            position.x += velocity.x * DT;
        }
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

- **`Query<Q>`** where `Q` is `&T`, `&mut T`, or a tuple of up to four of
  them: every entity with all of those components. Reads `&T`, writes
  `&mut T`. `q.iter(cx)` borrows `cx` for the loop, so a query can't be
  alive across a service call or a structural change, the rule the world
  already had.
- **`EventReader<E>`**: the events of type `E` this system hasn't seen yet.

Access the parameters can't express is added on the builder:
`.reads::<T>()` and `.writes::<T>()` for components the system reaches
through `cx.world()` directly, and `.exclusive()` for a system that may do
anything, including structural changes on the spot.

**Access is checked on every world access.** While a non-exclusive system
runs, its own world calls are checked against what it declared: reading an
undeclared component, writing one declared read-only, or changing the
world's structure directly (`insert`, `remove`, `despawn`) is a violation.
The call does nothing, and the mod is marked failed with the reason, as for
a panic. A wrong declaration is a programming error, and in a multithreaded
schedule it would be a data race: the loudest failure is the kindest.

Not yet checked: what a service called from a system touches. The provider
runs as itself, with its own access unchecked. Step 2 has to settle this,
since a call makes the caller's footprint include the provider's.

### Structural changes go through commands

`cx.commands()` queues inserts, removes and despawns. They are applied at
the end of the phase, in the order the systems ran, so every system in a
phase sees the same structure. `spawn` happens at once: allocating an
entity touches no component storage. Outside a frame (a message handler, a
`load` hook), commands are applied before the next request or frame,
whichever comes first, so they never outlive the build that queued them.

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
game replays exactly. A constraint naming a system that isn't loaded is
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

cx.send_event(Scored { by_player: true });        // anywhere

fn cheer(&mut self, _: &mut (), cx: &mut Cx, scored: EventReader<Scored>) {
    for s in scored.read(cx) { ... }
}
```

Events follow the component rules (they're declared the same way, and live
in the loader's world), with these semantics:

- **Sent events become visible at the next phase boundary.** Every system
  in a phase sees the same events, whatever order the phase runs in.
- **Each reader sees each event once.** The loader keeps a cursor per
  system and event type, by name, so it survives the system's reload.
- **An event lives until the end of the frame after it was sent**, so a
  reader in an earlier phase still sees it next frame. Unread events are
  then dropped.
- **Sending works from anywhere**, including message handlers between
  frames. Pong's text interface turns `up` into an event that the `input`
  phase applies, so input lands at a defined point in the frame.

Sends are deferred, so senders never conflict with anything, which makes
them free to run in parallel later.

## Who schedules

The loader owns what makes a frame safe: the declarations, the plan, the
access checks, the phase boundaries, and refusing to run a system whose mod
is already running. When each system runs is policy, and belongs to a mod:
the one providing `engine_api::scheduler::Scheduler`, a service the engine
declares rather than any mod, so a bootstrap reaches whichever scheduler the
game loads without depending on it.

```rust
impl Scheduler for Sequential {
    fn run_frame(&mut self, _: &mut (), cx: &mut Cx) {
        let Some(frame) = scheduler::begin(cx) else { return };
        for phase in &frame.plan().phases {
            for system in &phase.systems {
                frame.run(system.id);
            }
            frame.end_phase();
        }
    }
}
```

`scheduler::begin` opens a frame (publishing what was queued between
frames) and returns the plan; dropping the `Frame` closes it, including when
the scheduler panics. The loader refuses a frame inside a frame. A bootstrap
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

1. **Systems, phases, access and commands**, on one thread. The ABI change,
   and the one that gets more expensive with every mod.
2. **System parallelism.** On hold for the storage redesign in
   [storage.md](storage.md), which replaces phase-boundary application of
   commands and events with dependency edges. Systems whose access doesn't
   conflict run at the same time. The loader claims a system's access atomically when it's run
   (`run_system`) and refuses a conflicting one, so a scheduler can't cause
   a data race, only a refusal.
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
