//! Systems: the functions a mod registers for the loader to run each frame,
//! with the phase they run in and their ordering. A system's parameters are
//! its declaration of what it touches; see `engine_ecs` and
//! docs/architecture/storage.md.
//!
//! ```ignore
//! impl Mod for Physics {
//!     type Transient = ();
//!
//!     fn systems(s: &mut Systems<Self>) {
//!         s.add("integrate", Self::integrate).phase(phase::SIMULATE);
//!     }
//! }
//!
//! impl Physics {
//!     fn integrate(&mut self, _: &mut (), cx: &mut Cx, mut q: Query<(&mut Position, &Velocity)>) {
//!         q.for_each(|_, (mut position, velocity)| position.x += velocity.x * DT);
//!     }
//! }
//! ```

use std::ffi::c_void;
use std::marker::PhantomData;
use std::panic::{AssertUnwindSafe, catch_unwind};

use engine_ecs::query::check_conflicts;
use engine_ecs::{ComponentDesc, Declare, FrameCx, Param, ParamDecl, World};

use crate::{Cx, Mod, ModContext, Status};

/// The engine's phases, in the order a frame runs them. A mod can add its
/// own between them with [`Systems::phase`]. Phases order systems; they
/// aren't barriers (docs/architecture/storage.md).
pub mod phase {
    /// Turning input (events, messages) into intent.
    pub const INPUT: &str = "input";
    /// Game logic. The default.
    pub const UPDATE: &str = "update";
    /// Physics and other integration over the frame's intent.
    pub const SIMULATE: &str = "simulate";
    /// Reacting to the frame's outcome.
    pub const LATE: &str = "late";
    /// Producing output from the settled world.
    pub const RENDER: &str = "render";

    pub const BUILTIN: [&str; 5] = [INPUT, UPDATE, SIMULATE, LATE, RENDER];
}

/// Runs one system of a build: fetches its parameters and calls it.
/// Catches panics: `Status::ERROR` means the system panicked.
pub type SystemFn = unsafe fn(ctx: *mut ModContext, frame: &FrameCx<'_>, params: &[ParamDecl]) -> Status;

/// What a build declares when it's opened: its systems and phases, and the
/// components and events they name, which the loader installs if the load
/// commits. Plain Rust types, read by a loader built by the same compiler.
#[derive(Default)]
pub struct Declarations {
    pub systems: Vec<SystemDesc>,
    pub phases: Vec<PhaseDesc>,
    pub components: Vec<ComponentDesc>,
    pub events: Vec<ComponentDesc>,
    /// Why the declarations can't be used: a component whose storage changed,
    /// or a system whose queries conflict.
    pub error: Option<String>,
}

pub struct SystemDesc {
    /// Unqualified; the loader prefixes the mod's name.
    pub name: String,
    pub phase: String,
    /// Qualified names (`"physics::integrate"`) of systems this one runs
    /// after, or before.
    pub after: Vec<String>,
    pub before: Vec<String>,
    pub params: Vec<ParamDecl>,
    /// This build's code.
    pub run: SystemFn,
}

pub struct PhaseDesc {
    pub name: String,
    pub after: Vec<String>,
    pub before: Vec<String>,
}

/// Where a mod declares its systems and phases: the argument to
/// [`Mod::systems`].
pub struct Systems<'w, T> {
    decls: Declarations,
    declare: Declare<'w>,
    _mod: PhantomData<fn() -> T>,
}

impl<T: Mod> Systems<'_, T> {
    /// Adds a system named `name` running `f`, in the `update` phase unless
    /// the builder says otherwise. `f` is a function of the mod's state, its
    /// transient part, its `Cx` and up to eight parameters (`Query`,
    /// `Spawner`, `EventReader`, `EventWriter`), and must not capture
    /// anything: a fn item, or a closure without captures.
    pub fn add<P, F: IntoSystem<T, P>>(&mut self, name: &str, f: F) -> SystemBuilder<'_> {
        let _ = f;
        let params = F::declare(&mut self.declare);
        if let Err(e) = check_conflicts(self.declare.world, name, &params) {
            self.decls.error.get_or_insert(e);
        }
        self.decls.systems.push(SystemDesc {
            name: name.into(),
            phase: phase::UPDATE.into(),
            after: Vec::new(),
            before: Vec::new(),
            params,
            run: run::<T, P, F>,
        });
        SystemBuilder { desc: self.decls.systems.last_mut().unwrap() }
    }

    /// Declares a phase, placed with the builder's `after` and `before`.
    /// Name it after the mod (`"pong::serve"`); several mods may declare the
    /// same phase, and its constraints add up.
    pub fn phase(&mut self, name: &str) -> PhaseBuilder<'_> {
        self.decls.phases.push(PhaseDesc { name: name.into(), after: Vec::new(), before: Vec::new() });
        PhaseBuilder { desc: self.decls.phases.last_mut().unwrap() }
    }
}

pub struct SystemBuilder<'a> {
    desc: &'a mut SystemDesc,
}

impl SystemBuilder<'_> {
    pub fn phase(self, phase: &str) -> Self {
        self.desc.phase = phase.into();
        self
    }

    /// Runs after the system named `system` (`"mod::system"`), if it's loaded
    /// and in the same phase or an earlier one.
    pub fn after(self, system: &str) -> Self {
        self.desc.after.push(system.into());
        self
    }

    pub fn before(self, system: &str) -> Self {
        self.desc.before.push(system.into());
        self
    }
}

pub struct PhaseBuilder<'a> {
    desc: &'a mut PhaseDesc,
}

impl PhaseBuilder<'_> {
    pub fn after(self, phase: &str) -> Self {
        self.desc.after.push(phase.into());
        self
    }

    pub fn before(self, phase: &str) -> Self {
        self.desc.before.push(phase.into());
        self
    }
}

/// A function that can be a system. `P` is its parameters, as a tuple.
/// Implemented for functions of the mod's state, transient part, `Cx` and up
/// to eight parameters.
pub trait IntoSystem<T: Mod, P>: Copy + 'static {
    fn declare(d: &mut Declare<'_>) -> Vec<ParamDecl>;
    #[doc(hidden)]
    fn call<'w>(self, state: &mut T, transient: &mut T::Transient, cx: &mut Cx, frame: &FrameCx<'w>, params: &'w [ParamDecl]);
}

macro_rules! into_system {
    ($($p:ident),*) => {
        impl<T: Mod, F, $($p: Param),*> IntoSystem<T, ($($p,)*)> for F
        where
            F: Copy + 'static,
            for<'a> &'a F: Fn(&mut T, &mut T::Transient, &mut Cx, $($p),*)
                + Fn(&mut T, &mut T::Transient, &mut Cx, $($p::Item<'_>),*),
        {
            #[allow(unused_variables)]
            fn declare(d: &mut Declare<'_>) -> Vec<ParamDecl> {
                vec![$($p::declare(d)),*]
            }

            #[allow(non_snake_case, unused_mut, unused_variables)]
            fn call<'w>(self, state: &mut T, transient: &mut T::Transient, cx: &mut Cx, frame: &FrameCx<'w>, params: &'w [ParamDecl]) {
                let mut params = params.iter();
                $(let $p = $p::fetch(frame, params.next().expect("one declaration per parameter"));)*
                // Picks the `Fn` over the parameters' items, whatever
                // lifetimes they were fetched with.
                fn call<T: Mod, $($p),*>(
                    f: impl Fn(&mut T, &mut T::Transient, &mut Cx, $($p),*),
                    state: &mut T,
                    transient: &mut T::Transient,
                    cx: &mut Cx,
                    $($p: $p),*
                ) {
                    f(state, transient, cx, $($p),*)
                }
                call(&self, state, transient, cx, $($p),*);
            }
        }
    };
}

into_system!();
into_system!(P1);
into_system!(P1, P2);
into_system!(P1, P2, P3);
into_system!(P1, P2, P3, P4);
into_system!(P1, P2, P3, P4, P5);
into_system!(P1, P2, P3, P4, P5, P6);
into_system!(P1, P2, P3, P4, P5, P6, P7);
into_system!(P1, P2, P3, P4, P5, P6, P7, P8);

/// The loader's entry point for one system: `SystemDesc::run`.
unsafe fn run<T: Mod, P, F: IntoSystem<T, P>>(ctx: *mut ModContext, frame: &FrameCx<'_>, params: &[ParamDecl]) -> Status {
    // `run` is a plain fn pointer, so the function it calls can't be stored
    // anywhere; it's rebuilt from nothing, which only a zero-sized one can be.
    const { assert!(size_of::<F>() == 0, "a system must not capture anything") };
    let result = catch_unwind(AssertUnwindSafe(|| unsafe {
        let f: F = std::mem::zeroed();
        let state = &mut *((*ctx).state as *mut T);
        let transient = &mut *crate::transient::<T>(&raw mut (*ctx).transient);
        let mut cx = Cx { raw: &mut *ctx };
        f.call(state, transient, &mut cx, frame, params);
    }));
    if result.is_ok() { Status::OK } else { Status::ERROR }
}

/// Fills `out` with `T`'s declarations, interning what they name in
/// `world`: `ModInfo::declare`. `false` if `Mod::systems` panicked.
#[doc(hidden)]
pub unsafe extern "C" fn __declare<T: Mod>(out: *mut c_void, world: *const World) -> bool {
    // SAFETY: the loader passes its world, which outlives the call.
    let world = unsafe { &*world };
    catch_unwind(AssertUnwindSafe(|| {
        let mut systems = Systems::<T> { decls: Declarations::default(), declare: Declare::new(world), _mod: PhantomData };
        T::systems(&mut systems);
        let Systems { mut decls, declare, .. } = systems;
        decls.components = declare.components;
        decls.events = declare.events;
        if let Some(e) = declare.error {
            decls.error.get_or_insert(e);
        }
        unsafe { *(out as *mut Declarations) = decls };
    }))
    .is_ok()
}
