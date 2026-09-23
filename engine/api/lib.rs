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

mod ecs;
mod service;
pub use service::{
    CallError, CallErrorKind, CallStatus, CallTarget, ErasedFn, MethodDesc, ServiceDesc,
};
#[doc(hidden)]
pub use service::{__begin_call, __end_call, __serve};
pub use ecs::{
    Column, Component, ComponentDesc, ComponentId, Crossing, DefaultFn, DropFn, Entity, FieldDesc,
    FieldKind, FieldType, World, WorldApi,
};
#[doc(hidden)]
pub use ecs::{__drop, __drop_fn, __fingerprint, __fingerprint_struct, __fnv, __write_default};

/// Bumped whenever any `#[repr(C)]` type in this crate changes shape.
pub const API_VERSION: u32 = 8;

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
    /// Run one tick.
    pub const STEP: Op = Op(1);
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
            Op::STEP => f.write_str("STEP"),
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
    /// Returned by the bootstrap mod's step to shut the engine down.
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
}

/// Services the loader provides to mods.
#[repr(C)]
pub struct Host {
    pub userdata: *mut c_void,
    pub log: unsafe extern "C" fn(ctx: *const ModContext, msg: *const u8, len: usize),
    /// Steps every loaded mod except the caller, in load order. Meant for the
    /// bootstrap mod, which owns the frame loop.
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
    /// The entity/component store every mod shares. See [`World`].
    pub world: WorldApi,
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

    pub fn step_mods(&self) -> Status {
        unsafe { ((*self.raw.host).step_mods)(self.raw) }
    }

    fn reply(&self, text: &str) {
        unsafe { ((*self.raw.host).reply)(self.raw, text.as_ptr(), text.len()) }
    }

    /// The shared world. Borrows the context mutably, so log after a query
    /// rather than inside it.
    pub fn world(&mut self) -> World<'_> {
        World::new(self.raw)
    }
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

    /// Called after every load, including reloads (`cx.generation() > 0`).
    fn load(&mut self, _transient: &mut Self::Transient, _cx: &mut Cx) {}
    fn step(&mut self, transient: &mut Self::Transient, cx: &mut Cx) -> Status;
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
pub fn __info<T: Mod>(interface: &'static str, deps: &'static str, services: &'static [ServiceDesc]) -> ModInfo {
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

    fn step(&mut self, _: &mut (), _cx: &mut Cx) -> Status {
        Status::OK
    }
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
            Op::LOAD | Op::STEP | Op::MESSAGE => {
                let transient = &mut *transient::<T>(slot);
                let mut cx = Cx { raw: &mut *ctx };
                match op {
                    Op::LOAD => state.load(transient, &mut cx),
                    Op::STEP => return state.step(transient, &mut cx),
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
/// it provides: `export_mod!(PhysicsMod, provides = [physics::Physics])`.
///
/// Also records the interface digests `engine_mod` computed for this build
/// (`ENGINE_INTERFACE_DIGEST`, `ENGINE_MOD_DEPS`, set at compile time), so the
/// loader knows what the build depends on. A library built without
/// `engine_mod` records none.
#[macro_export]
macro_rules! export_mod {
    ($ty:ty $(, provides = [$($service:path),* $(,)?])? $(,)?) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn engine_mod_info() -> $crate::ModInfo {
            // A const, so the list is in the library's static memory.
            const SERVICES: &[$crate::ServiceDesc] = &[$($(<$ty as $service>::__SERVICE),*)?];
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
            )
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn engine_mod_main(
            ctx: *mut $crate::ModContext,
            op: $crate::Op,
        ) -> $crate::Status {
            unsafe { $crate::__dispatch::<$ty>(ctx, op) }
        }
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

    impl Mod for Tracked {
        type Transient = Scratch;

        fn step(&mut self, scratch: &mut Scratch, _cx: &mut Cx) -> Status {
            self.steps += (scratch.0)() - 6;
            if self.steps > 41 {
                panic!("second step panics");
            }
            Status::OK
        }
    }

    /// A context with a live state, as the loader provides it. No host:
    /// nothing here calls back into one.
    fn context(state: &mut std::mem::MaybeUninit<Tracked>) -> ModContext {
        unsafe { (__info::<Tracked>("", "", &[]).state_default)(state.as_mut_ptr() as *mut u8) };
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
        assert_eq!(dispatch(&mut ctx, Op::STEP), Status::OK);
        assert_eq!(unsafe { state.assume_init_ref() }.steps, 41, "the step used the transient closure");
        assert_eq!(dispatch(&mut ctx, Op::STEP), Status::ERROR, "a panic must become ERROR, not unwind");
        assert_eq!(dispatch(&mut ctx, Op(77)), Status::ERROR, "unknown ops are errors");
        assert_eq!(TRANSIENTS_MADE.load(Ordering::SeqCst), made + 1, "one per build, not per step");

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
        let info = __info::<Tracked>("", "", &[]);
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
