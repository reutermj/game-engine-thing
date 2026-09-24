# Mod dependencies

How one mod uses another mod's components and calls its functions, and how
reloads stay consistent when those change. The code is in `engine/defs.bzl` (the
`interface`/`mod_deps` attributes and the game reload target),
`engine/tools/mod_links.rs` (the interface digest), and `load_batch` in
`engine/loader/engine.rs`.

## Interface and implementation

A mod is two crates:

- **Its interface**: the components it declares, which other mods may use.
  `engine_mod(interface = ["components.rs"])` builds it as a library named
  after the mod, so a dependent writes `use physics::Velocity`.
- **Its implementation**: its systems, the shared library the engine loads.

A dependent names the mods it uses in `mod_deps` and compiles against their
interfaces, never their implementations. That split is what makes Bazel do
the propagation work. Editing `physics/lib.rs` rebuilds only `physics`; every
dependent's inputs are unchanged, so its library is byte-for-byte the same.
Editing `physics/components.rs` rebuilds `physics` and every mod that
compiled against its interface.

A mod that only declares components, such as `mods/transform`, has an
implementation of one line: `engine_api::export_mod!(engine_api::Inert);`.

Component names are namespaced by the declaring mod
(`"physics::Velocity"`) by convention. The engine doesn't enforce it.

## How the engine knows a mod's dependencies

Bazel knows the dependency graph; the running engine has only libraries. So
each library carries its part of the graph:

1. For every mod, a build action (`engine/tools/mod_links.rs`) computes an
   **interface digest**: a hash of the interface's source files and of the
   digests of the interfaces it depends on. A mod without an interface has
   an empty digest.
2. The same action writes the mod's digest and its dependencies'
   (`name:digest` pairs) into an environment file, which `engine_mod` passes
   to rustc with `rustc_env_files`.
3. `export_mod!` reads them with `option_env!` and returns them in `ModInfo`.

The engine therefore knows, for every loaded build, which interface it
provides and exactly which interfaces it was compiled against, with nothing
declared twice. The digest is computed from sources, so a comment-only edit
to an interface counts as an interface change. That errs toward a game
reload that wasn't strictly needed rather than a dependent reading a layout
it wasn't built for.

## What the engine enforces

After any load, every mod must run against the interfaces it was built
against. Concretely:

- **A mod's dependencies must be loaded.** Loading `spawner` without
  `physics` is refused.
- **Each dependency must have the interface digest the dependent recorded.**
  Loading a `spawner` built against a different `physics` interface than the
  running one is refused.
- **A reload may not strand a running dependent.** Reloading `physics` alone
  with a changed interface, while `spawner` still runs against the old one,
  is refused, and the error names the game's reload target:

  ```
  physics's interface changed, and spawner was built against the old one;
  reload them together with `./bazel run //game:reload`
  ```

- **A mod with dependents can't be unloaded.**

These checks run before anything changes, so a refused load leaves the
running builds untouched.

The component-level layout check from [ecs.md](ecs.md) (newest build wins;
older builds cut off) remains as a fallback for libraries built without
`engine_mod`, which record no dependencies.

## Two ways to reload

- **`./bazel run //mods/<name>`** reloads one mod. It is the fast path for an
  implementation change, and the only way to live-load a mod that isn't in
  the game.
- **`./bazel run //game:reload`** sends every mod in the game as one batch.
  Bazel rebuilds only what the edit affected, and the engine skips every
  library whose contents match the running one, so only the affected mods
  reload: for an interface change to `physics`, that is `physics` and
  whatever compiled against it, and nothing else.

A batch is atomic. Every changed build is opened and checked first, and if
any fails, nothing is swapped. Otherwise the old builds are retired
dependents-first and the new ones loaded dependencies-first, so no mod runs
between two layouts. Component migration then happens once, on the first
access by the new builds.

`engine_game` also loads a game's mods in dependency order, and includes
dependencies the game didn't list.

The per-mod command can't do the game's job because Bazel's graph only
points one way: `//mods/physics` knows what `physics` depends on, but not
what depends on `physics`. Only a target above every mod, the game, sees
which mods an interface change reaches.

### Alternatives that were considered

Decided 2026-09-23. The per-mod command could instead:

- **escalate on its own**, with `modctl` invoking Bazel to build the stranded
  dependents and sending a batch. One command in every case, at the cost of
  a Bazel-run binary running Bazel, and `modctl` depending on the workspace.
  Still possible later on top of the batch machinery.
- **accept the reload and strand the dependents**, cutting them off from the
  changed components until each is reloaded. The running game is partially
  broken between commands, and nothing says when it's whole again.
- **not exist for mods in a game**: always reload through the game. Always
  correct, but loses the direct per-mod command.

## Calls between mods

A mod can offer functions to the mods that depend on it: a **service**,
declared in its interface crate with `service!` and implemented by the mod.
Callers use the plain functions the macro generates there.

```rust
// loot's interface
engine_api::service! {
    pub trait Loot {
        /// The next drop from `table`, from the provider's seeded generator.
        fn roll(table: &str) -> Option<String>;
    }
}

// loot's implementation: a service method runs as the mod, with its state,
// transient part and Cx, like its systems
impl loot::Loot for Tables {
    fn roll(&mut self, _: &mut (), cx: &mut Cx, table: &str) -> Option<String> { ... }
}
export_mod!(Tables, provides = [loot::Loot]);

// a chest system, with loot in its mod_deps
match loot::roll(cx, "chest") {
    Ok(drop) => { ... }
    Err(e) => cx.log(format!("couldn't reach loot: {e}")),
}
```

Methods are declared as callers see them; the provider's side adds `&mut
self`, the transient part and its `Cx`. Signatures use ordinary Rust types
(the Rust ABI, which the one-compiler rule already makes safe; see
[ecs.md](ecs.md#one-compiler-per-session)). A service works the same whether
its provider is reloadable or resident.

**Every call asks the loader for the provider's current build.** The loader
keeps no table a caller can hold on to, so a provider reloaded between two
calls is simply called in its new build, and no caller can keep a pointer into
an old one. (Linking a caller to its provider directly wouldn't allow that:
the dynamic linker resolves a library's calls once, when it loads, and can't
repoint them at a provider's new build.) The lookup isn't cached yet; it's
cheap next to the call, and worth measuring before optimizing.

**What may cross.** Arguments passed by reference may be anything, `&dyn Fn`
included: a borrow can't outlive the call, and both builds are mapped while it
runs. Arguments and return values passed by value must be `Crossing` (every
`FieldType`, every component), because the other side may keep them. A
`Box<dyn Fn()>` argument is a compile error.

**A call doesn't reach the world.** Called from a system, the provider's
`cx.world()` panics, as the system's own would: a service method works on
its provider's state and its arguments, so the scheduler can order a call
like a second system of the provider's. A provider that must change the
world does it through an [event](scheduling.md#events) that one of its own
systems reads, as the platformer's rules take `Hurt` and `Bounce` from the
walkers. Called between frames (from a hook or a message handler), a
provider may use `cx.world()` like its caller. See storage.md, "Services
don't touch the world".

**Calls fail as values.** A call returns `Result<T, CallError>`: the provider
isn't loaded, has failed, panicked during the call (the panic stops in the
provider, which is marked failed), or is already running.

**No calls back into a running mod.** A call into a mod whose code is already
on the stack (the caller itself, or a mod that called the caller) would give
it a second `&mut` to its own state, so the loader refuses it
(`CallErrorKind::Reentrant`). `mod_deps` has no cycles, so this mostly stops a
mod calling itself; a provider that needs to reach back to its callers should
leave data or an [event](scheduling.md#events) for them.

**One provider per service.** A load that would give a service a second
provider is refused.

A mod's `load` may call the mods it depends on, which have already loaded.
Its `unload` and `close` can't make calls (`Unavailable`): they run while the
loader is swapping builds.

## Open questions

**Open question:** several providers of one interface, such as every mod
that adds tools to an editor. Enumerating them (`cx.providers::<dyn Tool>()`)
fits the call mechanism; it waits for a use.

**Open question:** partial use. A dependent that reads one field of
`Velocity` is still rebuilt when another field changes. The world could
project each mod's own view of a component instead, at the cost of copying
on access.

**Open question:** a mod in several games. The error names the reload target
of the game the engine was started with, which is the only one it knows.

**Open question:** removing a mod from a game. A game reload loads and
reloads, but never unloads a running mod the game no longer lists (or one
loaded live, like `hello`).
