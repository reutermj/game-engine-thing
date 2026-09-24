//! The ABI between the loader and mods.
//!
//! Everything that crosses the boundary is `#[repr(C)]` and exchanged through two
//! `extern "C"` symbols every mod exports:
//!
//! - `engine_mod_info() -> ModInfo`: what the mod's state looks like, queried
//!   before the loader commits to a new build.
//! - `engine_mod_main(ctx, op) -> Status`: the single entry point, cr.h style.
//!
//! A mod's data has two lifetimes. Its state (the [`Mod`] type itself, declared
//! with [`mod_state!`]) is owned by the loader and carried across reloads, so
//! it may only hold data that can't point into a build's code, the same rule
//! components follow. Its transient part ([`Mod::Transient`]) is made by every
//! build on load and dropped by the same build before it's swapped out, so it
//! may hold anything: closures, trait objects, crate objects. What's too
//! expensive to rebuild on every load belongs to a longer-lived owner (see
//! get-y5t). Mod code, including its `static`s, never survives a reload: every
//! reload is a fresh `dlopen`.
//!
//! Mods normally don't touch any of this directly; they implement [`Mod`] and
//! call [`export_mod!`].

use std::ffi::c_void;
use std::panic::{AssertUnwindSafe, catch_unwind};

pub mod scheduler;
mod service;
mod system;
pub use system::{
    Declarations, IntoSystem, PhaseBuilder, PhaseDesc, SystemBuilder, SystemDesc, SystemFn, Systems, phase,
};
#[doc(hidden)]
pub use system::__declare;

/// The ECS, shared with the loader as Rust types: see `engine_ecs`.
pub use engine_ecs;
pub use engine_ecs::{
    Adds, Bounds, Bundle, Component, ComponentDesc, Dt, Crossing, DefaultFn, Despawns, DropFn, Entity, Event, EventReader,
    EventWriter, FieldDesc, FieldKind, FieldType, Mut, OrderKey, Query, Removes, Row, SpatialKey, Spawner, Storage, With,
    Without, World, ChildOf, children_of, entity_key, pair_key, pairs_from,
    WorldMut, component, event, field_struct,
};
#[doc(hidden)]
pub use engine_ecs::{__component_fingerprint, __drop, __drop_fn, __fingerprint, __fingerprint_struct, __fnv, __storage, __write_default};
pub use service::{
    CallError, CallErrorKind, CallStatus, CallTarget, ErasedFn, MethodDesc, ServiceDesc,
};
#[doc(hidden)]
pub use service::{__begin_call, __end_call, __serve};
/// Bumped whenever any type crossing between the loader and a mod changes
/// shape: this crate's and `engine_ecs`'s.
pub const API_VERSION: u32 = 19;

pub const INFO_SYMBOL: &[u8] = b"engine_mod_info\0";
pub const MAIN_SYMBOL: &[u8] = b"engine_mod_main\0";

pub type InfoFn = unsafe extern "C" fn() -> ModInfo;
pub type MainFn = unsafe extern "C" fn(ctx: *mut ModContext, op: Op) -> Status;

/// Lifecycle operation passed to `engine_mod_main`. A plain integer rather than a
/// Rust enum so an unknown value from a newer loader isn't undefined behavior.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Op(pub u32);

impl Op {
    /// Code was just mapped in, on first load or after a reload. The state is
    /// live (the loader made it from `Default` if it's new); the build makes its
    /// transient part.
    pub const LOAD: Op = Op(0);
    /// Run the session: a bootstrap's [`Bootstrap::run`]. Refused
    /// (`Status::ERROR`) by a build that isn't a bootstrap.
    pub const RUN: Op = Op(1);
    /// Code is about to be swapped for a new build. The build drops its
    /// transient part; the state is kept (or migrated) for the next build.
    pub const UNLOAD: Op = Op(2);
    /// State is about to be freed: the mod is being removed, or the new build's
    /// state can't take it over. The build drops its transient part and state.
    pub const CLOSE: Op = Op(3);
    /// Handle the text in `ModContext::message`, sent over the control
    /// socket, and reply through `Host::reply`.
    pub const MESSAGE: Op = Op(4);
}

impl std::fmt::Debug for Op {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            Op::LOAD => f.write_str("LOAD"),
            Op::RUN => f.write_str("RUN"),
            Op::UNLOAD => f.write_str("UNLOAD"),
            Op::CLOSE => f.write_str("CLOSE"),
            Op::MESSAGE => f.write_str("MESSAGE"),
            Op(n) => write!(f, "Op({n})"),
        }
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Status(pub i32);

impl Status {
    pub const OK: Status = Status(0);
    /// Returned by a bootstrap's `run` when the engine was asked to quit.
    pub const QUIT: Status = Status(1);
    /// The mod failed (e.g. panicked). The loader stops stepping it until it's reloaded.
    pub const ERROR: Status = Status(-1);
    /// The mod declined a message (unknown command, bad argument); the reply
    /// says why. Unlike `ERROR`, the mod carries on.
    pub const REFUSED: Status = Status(2);
}

/// What a build says about itself. Pointers point into the library; the
/// loader copies what it keeps.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ModInfo {
    pub api_version: u32,
    /// The state's layout and schema, as for a component: the loader carries
    /// the state to the next build unchanged, migrates it field by field, or,
    /// if the version changed or there's no schema, has the old build drop it.
    pub state_version: u32,
    pub state_size: usize,
    pub state_align: usize,
    pub state_fields: *const FieldDesc,
    pub state_field_count: usize,
    pub state_drop: Option<DropFn>,
    pub state_default: DefaultFn,
    /// Digest of this build's interface (hex; empty if it has none), computed
    /// by Bazel.
    pub interface: *const u8,
    pub interface_len: usize,
    /// The interfaces this build compiled against, as `name:digest` pairs
    /// separated by commas.
    pub deps: *const u8,
    pub deps_len: usize,
    /// The services this build provides (`export_mod!(.., provides = [..])`).
    pub services: *const ServiceDesc,
    pub service_count: usize,
    /// Built with `engine_mod(resident = True)`: loaded once, never swapped.
    pub resident: bool,
    /// Fills a `Declarations` with the build's systems and phases
    /// (`Mod::systems`), interning what they name in the world. `false` if
    /// that panicked.
    pub declare: unsafe extern "C" fn(out: *mut c_void, world: *const World) -> bool,
    /// Exported with `export_mod!(.., bootstrap)`: it implements
    /// [`Bootstrap`] and takes `Op::RUN`.
    pub bootstrap: bool,
}

/// Services the loader provides to mods.
#[repr(C)]
pub struct Host {
    pub userdata: *mut c_void,
    pub log: unsafe extern "C" fn(ctx: *const ModContext, msg: *const u8, len: usize),
    /// Runs one frame with the loader's own sequential scheduler: every
    /// system in plan order, but those of mods already running. What
    /// `Cx::run_frame` falls back to when no `Scheduler` is loaded.
    pub step_mods: unsafe extern "C" fn(ctx: *const ModContext) -> Status,
    /// Appends to the reply to the message being handled.
    pub reply: unsafe extern "C" fn(ctx: *const ModContext, msg: *const u8, len: usize),
    /// Resolves a call to `service`'s `method` in the provider's current
    /// build, and marks the provider running until `end_call`.
    pub begin_call: unsafe extern "C" fn(
        ctx: *const ModContext,
        service: *const u8,
        service_len: usize,
        method: *const u8,
        method_len: usize,
        target: *mut CallTarget,
    ) -> CallStatus,
    /// Ends a call `begin_call` resolved; `panicked` marks the provider failed.
    pub end_call: unsafe extern "C" fn(ctx: *const ModContext, target: *const CallTarget, panicked: bool),
    /// Lets the loader do its work: serve requests (loads, messages, quit)
    /// and swap builds. Waits up to `timeout` for the first request. Messages
    /// addressed to the caller go to `handler`, since the caller is running.
    /// See [`Cx::pump_loader`].
    pub pump: unsafe fn(
        ctx: *const ModContext,
        timeout: std::time::Duration,
        handler: &mut dyn FnMut(&str) -> Result<String, String>,
    ) -> Pumped,
    /// A scheduler's frame primitives; see [`scheduler`]. `begin_frame`
    /// returns the plan, or `None` if a frame is already open.
    pub begin_frame: unsafe fn(ctx: *const ModContext) -> Option<scheduler::FramePlan>,
    pub run_node: unsafe fn(ctx: *const ModContext, node: usize) -> scheduler::Ran,
    pub end_frame: unsafe fn(ctx: *const ModContext),
    /// Sets how much time the next frame covers, in seconds: what fixed-rate
    /// phases accumulate steps from.
    pub set_frame_time: unsafe fn(ctx: *const ModContext, seconds: f32),
    /// The world every mod shares, which the loader owns. Mods reach it
    /// through their systems' parameters, and `Cx::world` between frames.
    pub world: *const World,
    /// What keeps the build behind `ctx` mapped, for the world to hold while
    /// values need that build's code.
    pub keepalive: unsafe fn(ctx: *const ModContext) -> Option<engine_ecs::Keepalive>,
}

/// What [`Cx::pump_loader`] did.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pumped {
    /// The loader did its work (possibly nothing); carry on.
    Continue,
    /// The engine was asked to quit: return `Status::QUIT` from the loop.
    Quit,
    /// Not a safe point: a mod that isn't resident is on the stack (only a
    /// resident bootstrap may pump, and not from inside `step_mods`).
    Refused,
}

/// Per-mod context. Lives in the loader and is stable across reloads.
#[repr(C)]
pub struct ModContext {
    pub host: *const Host,
    pub name: *const u8,
    pub name_len: usize,
    /// 0 on first load, incremented on every reload.
    pub generation: u32,
    /// Set by the loader on every load from one counter shared by all mods, so
    /// the world can tell a newer build's component layout from a stale one.
    pub loaded_at: u64,
    /// Loader-owned memory sized and aligned per the mod's `ModInfo`, holding a
    /// live value.
    pub state: *mut c_void,
    /// The running build's transient part, owned by that build: set in its
    /// `LOAD`, taken in its `UNLOAD` or `CLOSE`. Null between builds; the
    /// loader never looks behind it.
    pub transient: *mut c_void,
    /// The message being handled; only set during `Op::MESSAGE`.
    pub message: *const u8,
    pub message_len: usize,
}

/// Safe view of a [`ModContext`] handed to [`Mod`] callbacks.
pub struct Cx<'a> {
    pub(crate) raw: &'a mut ModContext,
}

impl Cx<'_> {
    pub fn name(&self) -> &str {
        // SAFETY: the loader keeps the name alive and valid UTF-8 for the context's lifetime.
        unsafe {
            std::str::from_utf8_unchecked(std::slice::from_raw_parts(
                self.raw.name,
                self.raw.name_len,
            ))
        }
    }

    pub fn generation(&self) -> u32 {
        self.raw.generation
    }

    pub fn log(&self, msg: impl AsRef<str>) {
        let msg = msg.as_ref();
        unsafe { ((*self.raw.host).log)(self.raw, msg.as_ptr(), msg.len()) }
    }

    /// One frame on the loader's own sequential scheduler. Bootstraps call
    /// [`Cx::run_frame`], which prefers a loaded `Scheduler`.
    pub fn step_mods(&self) -> Status {
        unsafe { ((*self.raw.host).step_mods)(self.raw) }
    }

    /// Hands the loader control: it serves pending requests (loads and
    /// reloads, messages, quit) and swaps builds, then returns. Waits up to
    /// `timeout` for a request if none is pending (`Duration::ZERO` doesn't
    /// wait). The loader has no loop of its own: the bootstrap owns the frame
    /// loop and calls this once per frame, or blocks in it while idle.
    ///
    /// Only a safe point if every mod on the stack is resident, since those
    /// builds are never swapped; otherwise it's refused. Messages sent to the
    /// caller while it pumps go to `handler`, called with a fresh `Cx`,
    /// rather than to its `Mod::message`, which would re-enter it.
    pub fn pump_loader(
        &mut self,
        timeout: std::time::Duration,
        mut handler: impl FnMut(&mut Cx, &str) -> Result<String, String>,
    ) -> Pumped {
        let raw: *mut ModContext = self.raw;
        let mut handler = |message: &str| {
            // The caller's `Cx` is borrowed by this call; the handler gets
            // its own, derived from it.
            let mut cx = Cx { raw: unsafe { &mut *raw } };
            handler(&mut cx, message)
        };
        unsafe { ((*self.raw.host).pump)(raw, timeout, &mut handler) }
    }

    fn reply(&self, text: &str) {
        unsafe { ((*self.raw.host).reply)(self.raw, text.as_ptr(), text.len()) }
    }

    /// The whole world, between frames: for hooks (`load`, `close`) and
    /// message handlers. In a frame the world is reached only through a
    /// system's parameters, so this panics there, which is also what keeps
    /// a service called from a system out of the world.
    pub fn world(&mut self) -> WorldMut<'_> {
        let host = unsafe { &*self.raw.host };
        let world = unsafe { &*host.world };
        let build = engine_ecs::Build {
            name: self.name().to_string(),
            loaded_at: self.raw.loaded_at,
            keepalive: unsafe { (host.keepalive)(self.raw) },
        };
        let name = self.name().to_string();
        world.between_frames(build).unwrap_or_else(|e| panic!("{name}: {e}"))
    }

    /// Sends an event, readable from the next frame: for code outside a
    /// frame. A system sends through an `EventWriter` parameter.
    pub fn send_event<E: Event>(&mut self, event: E) {
        self.world().send_event(event);
    }
}

/// Declares a mod's state: the data the loader carries from one build to the
/// next, so it follows the same rule as a component's fields (every field a
/// [`FieldType`]) and migrates the same way when its layout changes. Anything
/// that doesn't fit (a closure, a trait object, a crate's handle) goes in
/// [`Mod::Transient`], which each build makes for itself.
///
/// ```ignore
/// engine_api::mod_state! {
///     #[derive(Default)]
///     struct Counter {
///         frames: u64,
///         count: u64,
///     }
/// }
/// ```
///
/// Bump the version (`struct Counter, version = 1 { ... }`) when a field's
/// meaning changes, to reset the state instead of migrating it.
#[macro_export]
macro_rules! mod_state {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident $(, version = $version:literal)? {
            $($(#[$fmeta:meta])* $fvis:vis $field:ident : $ty:ty),* $(,)?
        }
    ) => {
        $(#[$meta])*
        $vis struct $name {
            $($(#[$fmeta])* $fvis $field: $ty),*
        }

        // SAFETY: every field is a `FieldType`, which rules out pointers into
        // the mod, and `FIELDS` is generated from the struct itself.
        unsafe impl $crate::ModState for $name {
            $(const VERSION: u32 = $version;)?
            const FIELDS: &'static [$crate::FieldDesc] = &[
                $($crate::FieldDesc::new::<$ty>(stringify!($field), ::std::mem::offset_of!($name, $field))),*
            ];
        }
    };
}

/// A mod's state: the data the loader carries from one build to the next.
/// Normally implemented by [`mod_state!`].
///
/// # Safety
///
/// The same rule as [`Component`]: the value outlives the build that made it,
/// so it must hold nothing that points into a mod's image (no references,
/// function pointers, trait objects, closures or `&'static str`). `FIELDS`, if
/// not empty, must describe every field exactly. Anything that doesn't fit
/// belongs in [`Mod::Transient`].
pub unsafe trait ModState: Default + 'static {
    /// Bump when a field's meaning changes, whether or not the layout does.
    /// A version change resets the state rather than migrating it.
    const VERSION: u32 = 0;
    const FIELDS: &'static [FieldDesc] = &[];
}

/// A mod. The type is its state (see [`ModState`]), carried across reloads;
/// `Transient` is everything else, rebuilt by every build.
pub trait Mod: ModState {
    /// This build's own data: made (from `Default`) before every `load`, and
    /// dropped by this build after its `unload` or `close`, while its code is
    /// still mapped. It may hold anything, including closures and trait
    /// objects, and it never reaches another build. `()` if there's nothing.
    type Transient: Default + 'static;

    /// Declares the mod's systems and phases; see [`Systems`]. Run by every
    /// build when it's opened. A mod's per-frame work is its systems: one
    /// that declares none (it only declares components, or only takes
    /// messages) runs nothing each frame.
    fn systems(_systems: &mut Systems<Self>)
    where
        Self: Sized,
    {
    }
    /// Called after every load, including reloads (`cx.generation() > 0`).
    fn load(&mut self, _transient: &mut Self::Transient, _cx: &mut Cx) {}
    /// Called before this build is swapped out.
    fn unload(&mut self, _transient: &mut Self::Transient, _cx: &mut Cx) {}
    /// Called before the state is dropped.
    fn close(&mut self, _transient: &mut Self::Transient, _cx: &mut Cx) {}
    /// Handles text sent with `modctl send <mod> <text>`. `Ok` is the reply;
    /// `Err` declines the message, with the reason as the reply.
    fn message(&mut self, _transient: &mut Self::Transient, cx: &mut Cx, _message: &str) -> Result<String, String> {
        Err(format!("{} doesn't take messages", cx.name()))
    }
}

#[doc(hidden)]
pub fn __info<T: Mod>(
    interface: &'static str,
    deps: &'static str,
    services: &'static [ServiceDesc],
    resident: bool,
    bootstrap: bool,
) -> ModInfo {
    ModInfo {
        api_version: API_VERSION,
        state_version: T::VERSION,
        state_size: size_of::<T>(),
        state_align: align_of::<T>(),
        state_fields: T::FIELDS.as_ptr(),
        state_field_count: T::FIELDS.len(),
        state_drop: __drop_fn::<T>(),
        state_default: __write_default::<T>,
        interface: interface.as_ptr(),
        interface_len: interface.len(),
        deps: deps.as_ptr(),
        deps_len: deps.len(),
        services: services.as_ptr(),
        service_count: services.len(),
        resident,
        declare: __declare::<T>,
        bootstrap,
    }
}

mod_state! {
    /// A mod with nothing to run, for one that only declares components:
    /// `export_mod!(engine_api::Inert);`.
    #[derive(Default)]
    pub struct Inert {}
}

impl Mod for Inert {
    type Transient = ();
}

/// A mod that runs the session: the frame loop. Exported with
/// `export_mod!(T, bootstrap)`, and built resident, since it's on the stack
/// whenever the loader swaps builds. See docs/architecture/overview.md, "Who
/// runs the loop".
pub trait Bootstrap: Mod {
    /// The session. Runs frames (`cx.step_mods()`) and hands the loader
    /// control between them (`cx.pump_loader`) until the pump says to quit,
    /// then returns `Status::QUIT`.
    fn run(&mut self, transient: &mut Self::Transient, cx: &mut Cx) -> Status;
}

/// `Op::RUN` for a bootstrap.
#[doc(hidden)]
pub unsafe fn __run<T: Bootstrap>(ctx: *mut ModContext) -> Status {
    let result = catch_unwind(AssertUnwindSafe(|| unsafe {
        let state = &mut *((*ctx).state as *mut T);
        let transient = &mut *transient::<T>(&raw mut (*ctx).transient);
        let mut cx = Cx { raw: &mut *ctx };
        state.run(transient, &mut cx)
    }));
    result.unwrap_or(Status::ERROR)
}

/// The running build's transient part, made if there's none yet (a `LOAD`
/// that panicked before making it).
pub(crate) unsafe fn transient<T: Mod>(slot: *mut *mut c_void) -> *mut T::Transient {
    unsafe {
        if (*slot).is_null() {
            *slot = Box::into_raw(Box::<T::Transient>::default()) as *mut c_void;
        }
        *slot as *mut T::Transient
    }
}

/// Takes the running build's transient part, to drop it.
unsafe fn take_transient<T: Mod>(slot: *mut *mut c_void) -> Box<T::Transient> {
    unsafe {
        let ptr = std::mem::replace(&mut *slot, std::ptr::null_mut());
        if ptr.is_null() { Box::default() } else { Box::from_raw(ptr as *mut T::Transient) }
    }
}

#[doc(hidden)]
pub unsafe fn __dispatch<T: Mod>(ctx: *mut ModContext, op: Op) -> Status {
    // Unwinding across `extern "C"` aborts the process; turn panics into errors instead.
    let result = catch_unwind(AssertUnwindSafe(|| unsafe {
        let state = &mut *((*ctx).state as *mut T);
        // Reached through a raw pointer, and before `cx` borrows the context.
        let slot: *mut *mut c_void = &raw mut (*ctx).transient;
        match op {
            Op::LOAD | Op::MESSAGE => {
                let transient = &mut *transient::<T>(slot);
                let mut cx = Cx { raw: &mut *ctx };
                match op {
                    Op::LOAD => state.load(transient, &mut cx),
                    _ => {
                        let message = std::slice::from_raw_parts(cx.raw.message, cx.raw.message_len);
                        let message = std::str::from_utf8(message).unwrap_or("");
                        return match state.message(transient, &mut cx, message) {
                            Ok(reply) => {
                                cx.reply(&reply);
                                Status::OK
                            }
                            Err(reason) => {
                                cx.reply(&reason);
                                Status::REFUSED
                            }
                        };
                    }
                }
            }
            Op::UNLOAD | Op::CLOSE => {
                // Dropped here, by this build, whatever happens.
                let mut transient = take_transient::<T>(slot);
                let mut cx = Cx { raw: &mut *ctx };
                if op == Op::UNLOAD {
                    state.unload(&mut transient, &mut cx);
                } else {
                    state.close(&mut transient, &mut cx);
                    drop(transient);
                    std::ptr::drop_in_place(state);
                }
            }
            _ => return Status::ERROR,
        }
        Status::OK
    }));
    result.unwrap_or(Status::ERROR)
}

/// Exports `ty` (a [`Mod`]) as this dynamic library's mod, and the services
/// it provides: `export_mod!(PhysicsMod, provides = [physics::Physics])`. A
/// bootstrap (a [`Bootstrap`]) says so: `export_mod!(Lockstep, bootstrap)`.
///
/// Also records the interface digests `engine_mod` computed for this build
/// (`ENGINE_INTERFACE_DIGEST`, `ENGINE_MOD_DEPS`, set at compile time), so the
/// loader knows what the build depends on. A library built without
/// `engine_mod` records none.
#[macro_export]
macro_rules! export_mod {
    ($ty:ty, bootstrap $(, provides = [$($service:path),* $(,)?])? $(,)?) => {
        $crate::export_mod!(@export $ty, true, [$($($service),*)?]);
    };
    ($ty:ty $(, provides = [$($service:path),* $(,)?])? $(,)?) => {
        $crate::export_mod!(@export $ty, false, [$($($service),*)?]);
    };
    (@export $ty:ty, $bootstrap:tt, [$($service:path),*]) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn engine_mod_info() -> $crate::ModInfo {
            // A const, so the list is in the library's static memory.
            const SERVICES: &[$crate::ServiceDesc] = &[$(<$ty as $service>::__SERVICE),*];
            $crate::__info::<$ty>(
                match option_env!("ENGINE_INTERFACE_DIGEST") {
                    Some(digest) => digest,
                    None => "",
                },
                match option_env!("ENGINE_MOD_DEPS") {
                    Some(deps) => deps,
                    None => "",
                },
                SERVICES,
                // Read here, where the mod compiles, like the digests above.
                option_env!("ENGINE_MOD_RESIDENT").is_some(),
                $bootstrap,
            )
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn engine_mod_main(
            ctx: *mut $crate::ModContext,
            op: $crate::Op,
        ) -> $crate::Status {
            $crate::export_mod!(@main $bootstrap, $ty, ctx, op)
        }
    };
    (@main true, $ty:ty, $ctx:ident, $op:ident) => {
        if $op == $crate::Op::RUN {
            unsafe { $crate::__run::<$ty>($ctx) }
        } else {
            unsafe { $crate::__dispatch::<$ty>($ctx, $op) }
        }
    };
    (@main false, $ty:ty, $ctx:ident, $op:ident) => {
        unsafe { $crate::__dispatch::<$ty>($ctx, $op) }
    };
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    static STATE_DROPS: AtomicUsize = AtomicUsize::new(0);
    static TRANSIENTS_MADE: AtomicUsize = AtomicUsize::new(0);
    static TRANSIENT_DROPS: AtomicUsize = AtomicUsize::new(0);

    mod_state! {
        struct Tracked {
            steps: u32,
        }
    }

    impl Default for Tracked {
        fn default() -> Self {
            Tracked { steps: 40 }
        }
    }

    impl Drop for Tracked {
        fn drop(&mut self) {
            STATE_DROPS.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// Stands in for what state can't hold: a closure.
    struct Scratch(Box<dyn Fn() -> u32>);

    impl Default for Scratch {
        fn default() -> Self {
            TRANSIENTS_MADE.fetch_add(1, Ordering::SeqCst);
            Scratch(Box::new(|| 7))
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            TRANSIENT_DROPS.fetch_add(1, Ordering::SeqCst);
        }
    }

    impl Tracked {
        fn tick(&mut self, scratch: &mut Scratch, _cx: &mut Cx) {
            self.steps += (scratch.0)() - 6;
            if self.steps > 41 {
                panic!("second tick panics");
            }
        }
    }

    impl Mod for Tracked {
        type Transient = Scratch;

        fn systems(s: &mut Systems<Self>) {
            s.add("tick", Self::tick);
        }
    }

    /// Tracked's one system, as the loader runs it.
    fn tick(ctx: &mut ModContext) -> Status {
        let world = World::new();
        let mut decls = Declarations::default();
        assert!(unsafe { __declare::<Tracked>(&mut decls as *mut Declarations as *mut c_void, &world as *const World) });
        assert_eq!(decls.systems.len(), 1);
        let log = engine_ecs::Log::default();
        let frame = engine_ecs::FrameCx { world: &world, log: &log, system: "t::tick", dt: 1.0 / 60.0 };
        unsafe { (decls.systems[0].run)(ctx, &frame, &decls.systems[0].params) }
    }

    /// A context with a live state, as the loader provides it. No host:
    /// nothing here calls back into one.
    fn context(state: &mut std::mem::MaybeUninit<Tracked>) -> ModContext {
        unsafe { (__info::<Tracked>("", "", &[], false, false).state_default)(state.as_mut_ptr() as *mut u8) };
        ModContext {
            host: std::ptr::null(),
            name: "t".as_ptr(),
            name_len: 1,
            generation: 0,
            loaded_at: 1,
            state: state.as_mut_ptr() as *mut c_void,
            transient: std::ptr::null_mut(),
            message: std::ptr::null(),
            message_len: 0,
        }
    }

    #[test]
    fn dispatch_rebuilds_the_transient_part_per_build_and_drops_the_state_on_close() {
        let mut state = std::mem::MaybeUninit::<Tracked>::uninit();
        let mut ctx = context(&mut state);
        assert_eq!(unsafe { state.assume_init_ref() }.steps, 40, "made from Default by the loader");
        let dispatch = |ctx: &mut ModContext, op| unsafe { __dispatch::<Tracked>(ctx, op) };
        let (made, dropped) = (TRANSIENTS_MADE.load(Ordering::SeqCst), TRANSIENT_DROPS.load(Ordering::SeqCst));

        assert_eq!(dispatch(&mut ctx, Op::LOAD), Status::OK);
        assert!(!ctx.transient.is_null(), "LOAD makes the transient part");
        assert_eq!(tick(&mut ctx), Status::OK);
        assert_eq!(unsafe { state.assume_init_ref() }.steps, 41, "the system used the transient closure");
        assert_eq!(tick(&mut ctx), Status::ERROR, "a panic must become ERROR, not unwind");
        assert_eq!(dispatch(&mut ctx, Op::RUN), Status::ERROR, "only a bootstrap runs");
        assert_eq!(dispatch(&mut ctx, Op(77)), Status::ERROR, "unknown ops are errors");
        assert_eq!(TRANSIENTS_MADE.load(Ordering::SeqCst), made + 1, "one per build, not per frame");

        // A reload: this build drops its transient part and leaves the state.
        assert_eq!(dispatch(&mut ctx, Op::UNLOAD), Status::OK);
        assert!(ctx.transient.is_null());
        assert_eq!(TRANSIENT_DROPS.load(Ordering::SeqCst), dropped + 1);
        assert_eq!(unsafe { state.assume_init_ref() }.steps, 42, "the state stays");
        // The next build makes its own.
        assert_eq!(dispatch(&mut ctx, Op::LOAD), Status::OK);
        assert_eq!(TRANSIENTS_MADE.load(Ordering::SeqCst), made + 2);

        let state_drops = STATE_DROPS.load(Ordering::SeqCst);
        assert_eq!(dispatch(&mut ctx, Op::CLOSE), Status::OK);
        assert_eq!(STATE_DROPS.load(Ordering::SeqCst), state_drops + 1, "CLOSE must drop the state");
        assert_eq!(TRANSIENT_DROPS.load(Ordering::SeqCst), dropped + 2, "and the transient part");
        assert!(ctx.transient.is_null());
    }

    #[test]
    fn mod_state_records_its_schema() {
        let info = __info::<Tracked>("", "", &[], false, false);
        assert_eq!(info.state_field_count, 1);
        assert!(info.state_drop.is_some(), "Tracked has a Drop impl");
        let field = unsafe { &*info.state_fields };
        assert_eq!((field.kind, field.offset), (FieldKind::U32, std::mem::offset_of!(Tracked, steps)));
    }

    component! {
        #[derive(Default)]
        struct Mixed: "test::Mixed" {
            a: u8,
            b: f64,
            c: Entity,
            d: bool,
        }
    }

    #[test]
    fn component_macro_records_every_field_where_the_compiler_put_it() {
        let fields: Vec<(String, FieldKind, usize)> = Mixed::FIELDS
            .iter()
            .map(|f| {
                let name = unsafe { std::slice::from_raw_parts(f.name, f.name_len) };
                (String::from_utf8(name.to_vec()).unwrap(), f.kind, f.offset)
            })
            .collect();
        assert_eq!(
            fields,
            [
                ("a".into(), FieldKind::U8, std::mem::offset_of!(Mixed, a)),
                ("b".into(), FieldKind::F64, std::mem::offset_of!(Mixed, b)),
                ("c".into(), FieldKind::ENTITY, std::mem::offset_of!(Mixed, c)),
                ("d".into(), FieldKind::BOOL, std::mem::offset_of!(Mixed, d)),
            ]
        );
        // Rust reorders fields; the schema must follow the real layout, which
        // is only true if these aren't simply declaration order.
        let offsets: Vec<usize> = fields.iter().map(|f| f.2).collect();
        assert_ne!(offsets, [0, 1, 9, 17], "{offsets:?}");
        assert_eq!(Mixed::NAME, "test::Mixed");
        assert_eq!(Mixed::VERSION, 0);
    }

    component! {
        #[derive(Default)]
        struct Versioned: "test::Versioned", version = 3 {
            a: u32,
        }
    }

    #[test]
    fn component_macro_passes_the_version_through() {
        assert_eq!(Versioned::VERSION, 3);
    }
}
