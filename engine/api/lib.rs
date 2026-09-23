//! The ABI between the loader and mods.
//!
//! Everything that crosses the boundary is `#[repr(C)]` and exchanged through two
//! `extern "C"` symbols every mod exports:
//!
//! - `engine_mod_info() -> ModInfo`: what the mod's state looks like, queried
//!   before the loader commits to a new build.
//! - `engine_mod_main(ctx, op) -> Status`: the single entry point, cr.h style.
//!
//! The loader owns each mod's state memory, so it survives a reload. Mod code,
//! including its `static`s, does not: every reload is a fresh `dlopen`.
//!
//! Mods normally don't touch any of this directly; they implement [`Mod`] and
//! call [`export_mod!`].

use std::ffi::c_void;
use std::panic::{AssertUnwindSafe, catch_unwind};

/// Bumped whenever any type in this file changes shape.
pub const API_VERSION: u32 = 1;

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
    /// Code was just mapped in, on first load or after a reload.
    pub const LOAD: Op = Op(0);
    /// Run one tick.
    pub const STEP: Op = Op(1);
    /// Code is about to be swapped for a new build. State is kept.
    pub const UNLOAD: Op = Op(2);
    /// State is about to be freed: the mod is being removed, or the new build's
    /// state layout doesn't match. Drop anything the state owns.
    pub const CLOSE: Op = Op(3);
}

impl std::fmt::Debug for Op {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            Op::LOAD => f.write_str("LOAD"),
            Op::STEP => f.write_str("STEP"),
            Op::UNLOAD => f.write_str("UNLOAD"),
            Op::CLOSE => f.write_str("CLOSE"),
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
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ModInfo {
    pub api_version: u32,
    /// Bump when the state's meaning changes without its size or alignment changing.
    pub state_version: u32,
    pub state_size: usize,
    pub state_align: usize,
}

impl ModInfo {
    /// Whether state written by a build with `self` can be handed to a build with `other`.
    pub fn state_compatible(&self, other: &ModInfo) -> bool {
        (self.state_version, self.state_size, self.state_align)
            == (other.state_version, other.state_size, other.state_align)
    }
}

/// Services the loader provides to mods.
#[repr(C)]
pub struct Host {
    pub userdata: *mut c_void,
    pub log: unsafe extern "C" fn(ctx: *const ModContext, msg: *const u8, len: usize),
    /// Steps every loaded mod except the caller, in load order. Meant for the
    /// bootstrap mod, which owns the frame loop.
    pub step_mods: unsafe extern "C" fn(ctx: *const ModContext) -> Status,
}

/// Per-mod context. Lives in the loader and is stable across reloads.
#[repr(C)]
pub struct ModContext {
    pub host: *const Host,
    pub name: *const u8,
    pub name_len: usize,
    /// 0 on first load, incremented on every reload.
    pub generation: u32,
    /// Loader-owned memory sized and aligned per the mod's `ModInfo`.
    pub state: *mut c_void,
    /// Set by the loader when `state` is zeroed memory rather than a live value.
    /// The mod initializes it during `Op::LOAD` and clears the flag.
    pub state_fresh: bool,
}

/// Safe view of a [`ModContext`] handed to [`Mod`] callbacks.
pub struct Cx<'a> {
    raw: &'a mut ModContext,
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
}

/// A mod is its own state: the loader keeps the value alive across reloads, and
/// each new build keeps running against it.
///
/// Changing the type's size or alignment, or [`Mod::STATE_VERSION`], makes the
/// loader drop the old value (via the old build's `close`) and start from
/// `Default` instead of reinterpreting incompatible memory.
pub trait Mod: Default + 'static {
    const STATE_VERSION: u32 = 0;

    /// Called after every load, including reloads (`cx.generation() > 0`).
    fn load(&mut self, _cx: &mut Cx) {}
    fn step(&mut self, cx: &mut Cx) -> Status;
    /// Called before this build is swapped out.
    fn unload(&mut self, _cx: &mut Cx) {}
    /// Called before the state is dropped.
    fn close(&mut self, _cx: &mut Cx) {}
}

#[doc(hidden)]
pub fn __info<T: Mod>() -> ModInfo {
    ModInfo {
        api_version: API_VERSION,
        state_version: T::STATE_VERSION,
        state_size: size_of::<T>(),
        state_align: align_of::<T>(),
    }
}

#[doc(hidden)]
pub unsafe fn __dispatch<T: Mod>(ctx: *mut ModContext, op: Op) -> Status {
    // Unwinding across `extern "C"` aborts the process; turn panics into errors instead.
    let result = catch_unwind(AssertUnwindSafe(|| unsafe {
        let ctx = &mut *ctx;
        let state = ctx.state as *mut T;
        if op == Op::LOAD && ctx.state_fresh {
            state.write(T::default());
            ctx.state_fresh = false;
        }
        let state = &mut *state;
        let mut cx = Cx { raw: ctx };
        match op {
            Op::LOAD => state.load(&mut cx),
            Op::STEP => return state.step(&mut cx),
            Op::UNLOAD => state.unload(&mut cx),
            Op::CLOSE => {
                state.close(&mut cx);
                std::ptr::drop_in_place(state);
            }
            _ => return Status::ERROR,
        }
        Status::OK
    }));
    result.unwrap_or(Status::ERROR)
}

/// Exports `ty` (a [`Mod`]) as this dynamic library's mod.
#[macro_export]
macro_rules! export_mod {
    ($ty:ty) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn engine_mod_info() -> $crate::ModInfo {
            $crate::__info::<$ty>()
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
