//! Calls between mods. A mod declares a service in its interface crate with
//! [`service!`]; mods that depend on it (`mod_deps`) call it through the
//! plain functions the macro generates there:
//!
//! ```ignore
//! // physics's interface
//! engine_api::service! {
//!     pub trait Physics {
//!         fn impulse(entity: Entity, dx: f32, dy: f32);
//!         fn raycast(from: (f32, f32), to: (f32, f32)) -> Option<Hit>;
//!     }
//! }
//!
//! // physics's implementation
//! impl physics::Physics for PhysicsMod {
//!     fn impulse(&mut self, transient: &mut (), cx: &mut Cx, entity: Entity, dx: f32, dy: f32) { ... }
//!     ...
//! }
//! engine_api::export_mod!(PhysicsMod, provides = [physics::Physics]);
//!
//! // any mod with physics in its mod_deps
//! let hit = physics::raycast(cx, (0.0, 0.0), (5.0, 0.0))?;
//! ```
//!
//! A call runs as the provider, with its state, transient part and `Cx`, like
//! its systems. Every call asks the loader for the provider's current build, so
//! a provider reloaded between two calls is simply called in its new build,
//! and nothing a consumer holds can point into an old one. See
//! docs/architecture/mod-deps.md, "Calls between mods".

use crate::{Cx, Mod, ModContext};

/// A service method with its real signature erased. The generated caller
/// knows the signature and casts it back; `engine_mod`'s interface checks
/// make sure both sides were built from the same declaration.
pub type ErasedFn = unsafe fn();

#[repr(C)]
#[derive(Clone, Copy)]
pub struct MethodDesc {
    pub name: *const u8,
    pub name_len: usize,
    pub call: ErasedFn,
}

impl MethodDesc {
    pub const fn new(name: &'static str, call: ErasedFn) -> MethodDesc {
        MethodDesc { name: name.as_ptr(), name_len: name.len(), call }
    }
}

/// One service a build provides, in its `ModInfo`. Points into the library;
/// the loader copies what it keeps.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ServiceDesc {
    pub name: *const u8,
    pub name_len: usize,
    pub methods: *const MethodDesc,
    pub method_count: usize,
}

impl ServiceDesc {
    pub const fn new(name: &'static str, methods: &'static [MethodDesc]) -> ServiceDesc {
        ServiceDesc { name: name.as_ptr(), name_len: name.len(), methods: methods.as_ptr(), method_count: methods.len() }
    }
}

/// `Host::begin_call`'s answer. A plain integer for the same reason as `Op`.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CallStatus(pub u32);

impl CallStatus {
    pub const OK: CallStatus = CallStatus(0);
    pub const NOT_PROVIDED: CallStatus = CallStatus(1);
    pub const NO_SUCH_METHOD: CallStatus = CallStatus(2);
    pub const PROVIDER_FAILED: CallStatus = CallStatus(3);
    pub const REENTRANT: CallStatus = CallStatus(4);
    pub const UNAVAILABLE: CallStatus = CallStatus(5);
}

/// What `Host::begin_call` resolved: the method in the provider's current
/// build, and the provider's context to run it with.
#[repr(C)]
pub struct CallTarget {
    pub call: Option<ErasedFn>,
    pub provider: *mut ModContext,
}

/// Why a call between mods didn't run, or didn't finish.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CallError {
    pub service: &'static str,
    pub method: &'static str,
    pub kind: CallErrorKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CallErrorKind {
    /// No loaded mod provides the service.
    NotProvided,
    /// The provider's build has no such method. `engine_mod`'s interface
    /// checks rule this out for mods it builds.
    NoSuchMethod,
    /// The provider failed earlier (it panicked) and won't run until reloaded.
    ProviderFailed,
    /// The provider is already running further up the stack: the caller
    /// itself, or a mod that (indirectly) called the caller. Running it again
    /// would give it a second `&mut` to its own state. Use an event instead.
    Reentrant,
    /// The provider panicked during this call. It's now marked failed.
    Panicked,
    /// The loader couldn't take the call right now.
    Unavailable,
}

impl std::fmt::Display for CallError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let what = match self.kind {
            CallErrorKind::NotProvided => "no loaded mod provides it",
            CallErrorKind::NoSuchMethod => "the provider has no such method",
            CallErrorKind::ProviderFailed => "its provider has failed; reload it",
            CallErrorKind::Reentrant => "its provider is already running (a call back into a running mod)",
            CallErrorKind::Panicked => "its provider panicked",
            CallErrorKind::Unavailable => "the loader can't take calls right now",
        };
        write!(f, "{}::{}: {what}", self.service, self.method)
    }
}

impl std::error::Error for CallError {}

/// Resolves a call through the loader. Called by the functions `service!`
/// generates, never directly.
#[doc(hidden)]
pub fn __begin_call(cx: &mut Cx, service: &'static str, method: &'static str) -> Result<CallTarget, CallError> {
    let mut target = CallTarget { call: None, provider: std::ptr::null_mut() };
    let status =
        unsafe { ((*cx.raw.host).begin_call)(cx.raw, service.as_ptr(), service.len(), method.as_ptr(), method.len(), &mut target) };
    let kind = match status {
        CallStatus::OK if target.call.is_some() => return Ok(target),
        CallStatus::NOT_PROVIDED => CallErrorKind::NotProvided,
        CallStatus::NO_SUCH_METHOD => CallErrorKind::NoSuchMethod,
        CallStatus::PROVIDER_FAILED => CallErrorKind::ProviderFailed,
        CallStatus::REENTRANT => CallErrorKind::Reentrant,
        _ => CallErrorKind::Unavailable,
    };
    Err(CallError { service, method, kind })
}

#[doc(hidden)]
pub fn __end_call(cx: &mut Cx, target: &CallTarget, panicked: bool) {
    unsafe { ((*cx.raw.host).end_call)(cx.raw, target, panicked) }
}

/// Runs a service method in its provider: the provider's side of every call.
/// `Err` means the method panicked; the panic stops here instead of unwinding
/// into the caller's build.
///
/// # Safety
/// `ctx` must be the provider's context, from `begin_call`, whose state is a `T`.
#[doc(hidden)]
// Err carries nothing: the Result crosses into the caller's build, and a
// panic payload, a Box<dyn Any> with this build's vtable, could not outlive it.
#[allow(clippy::result_unit_err)]
pub unsafe fn __serve<T: Mod, R>(ctx: *mut ModContext, method: impl FnOnce(&mut T, &mut T::Transient, &mut Cx) -> R) -> Result<R, ()> {
    let run = std::panic::AssertUnwindSafe(|| unsafe {
        let state = &mut *((*ctx).state as *mut T);
        let transient = &mut *crate::transient::<T>(&raw mut (*ctx).transient);
        let mut cx = Cx { raw: &mut *ctx };
        method(state, transient, &mut cx)
    });
    std::panic::catch_unwind(run).map_err(|_| ())
}

/// The return type of a method declared with or without `-> T`.
#[doc(hidden)]
#[macro_export]
macro_rules! __service_ret {
    () => {
        ()
    };
    ($ret:ty) => {
        $ret
    };
}

/// Declares a service: functions a mod provides for other mods to call. See
/// the module docs of `engine_api::service` for the whole picture.
///
/// Methods are declared as callers see them. The macro generates:
///
/// - for callers, a function per method in this crate, taking `&mut Cx` first
///   and returning `Result<T, CallError>`;
/// - for the provider, a trait to implement on its `Mod`, whose methods also
///   take `&mut self`, the transient part and the provider's `Cx` first.
///
/// Arguments and return values passed by value must be [`Crossing`](crate::Crossing)
/// (a `FieldType`, a component), because the other side may keep them;
/// references may be anything, since they can't outlive the call.
#[macro_export]
macro_rules! service {
    (
        $(#[$meta:meta])*
        $vis:vis trait $name:ident {
            $(
                $(#[$fmeta:meta])*
                fn $fn:ident ( $($arg:ident : $ty:ty),* $(,)? ) $(-> $ret:ty)? ;
            )*
        }
    ) => {
        $(#[$meta])*
        $vis trait $name: $crate::Mod {
            $(
                $(#[$fmeta])*
                fn $fn(
                    &mut self,
                    transient: &mut <Self as $crate::Mod>::Transient,
                    cx: &mut $crate::Cx,
                    $($arg: $ty),*
                ) -> $crate::__service_ret!($($ret)?);
            )*

            #[doc(hidden)]
            const __SERVICE: $crate::ServiceDesc = $crate::ServiceDesc::new(
                concat!(module_path!(), "::", stringify!($name)),
                &[$(
                    $crate::MethodDesc::new(stringify!($fn), unsafe {
                        ::std::mem::transmute::<
                            unsafe fn(*mut $crate::ModContext, $($ty),*) -> Result<$crate::__service_ret!($($ret)?), ()>,
                            $crate::ErasedFn,
                        >(
                            // Inside the `unsafe` block above, so the body may
                            // call `__serve`.
                            (|ctx: *mut $crate::ModContext, $($arg: $ty),*| {
                                $crate::__serve::<Self, _>(ctx, |state, transient, cx| {
                                    <Self as $name>::$fn(state, transient, cx, $($arg),*)
                                })
                            }) as unsafe fn(*mut $crate::ModContext, $($ty),*) -> Result<$crate::__service_ret!($($ret)?), ()>,
                        )
                    })
                ),*],
            );
        }

        // What crosses by value must be safe to outlive the build it came from.
        const _: () = {
            fn crossing<T: $crate::Crossing + ?Sized>() {}
            #[allow(dead_code)]
            fn check() {
                $($(crossing::<$ty>();)* crossing::<$crate::__service_ret!($($ret)?)>();)*
            }
        };

        $(
            $(#[$fmeta])*
            #[allow(clippy::too_many_arguments)]
            $vis fn $fn(
                cx: &mut $crate::Cx,
                $($arg: $ty),*
            ) -> Result<$crate::__service_ret!($($ret)?), $crate::CallError> {
                const SERVICE: &str = concat!(module_path!(), "::", stringify!($name));
                let target = $crate::__begin_call(cx, SERVICE, stringify!($fn))?;
                let call = unsafe {
                    ::std::mem::transmute::<
                        $crate::ErasedFn,
                        unsafe fn(*mut $crate::ModContext, $($ty),*) -> Result<$crate::__service_ret!($($ret)?), ()>,
                    >(target.call.expect("begin_call returned a method"))
                };
                let result = unsafe { call(target.provider, $($arg),*) };
                $crate::__end_call(cx, &target, result.is_err());
                result.map_err(|()| $crate::CallError {
                    service: SERVICE,
                    method: stringify!($fn),
                    kind: $crate::CallErrorKind::Panicked,
                })
            }
        )*
    };
}
