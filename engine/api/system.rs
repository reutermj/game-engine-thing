//! Systems: the functions a mod registers for the loader to run each frame,
//! with the phase they run in, their ordering, and what they touch.
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
//!     fn integrate(&mut self, _: &mut (), cx: &mut Cx, q: Query<(&mut Position, &Velocity)>) {
//!         for (_, (position, velocity)) in q.iter(cx) { ... }
//!     }
//! }
//! ```
//!
//! A system's parameters are what it declares it touches, so the loader can
//! check every world access against them. See docs/architecture/scheduling.md.

use std::ffi::c_void;
use std::marker::PhantomData;
use std::panic::{AssertUnwindSafe, catch_unwind};

use crate::{Component, ComponentDesc, ComponentId, Cx, Entity, Mod, ModContext, Status};

/// The engine's phases, in the order a frame runs them. A mod can add its
/// own between them with [`Systems::phase`].
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

/// Runs one system of a build, with the build's context. Catches panics:
/// `Status::ERROR` means the system panicked.
pub type SystemFn = unsafe fn(ctx: *mut ModContext) -> Status;

/// What a build declares when it's opened. Plain Rust types, read by a
/// loader built by the same compiler (see ecs.md, "One compiler per session").
#[derive(Default)]
pub struct Declarations {
    pub systems: Vec<SystemDesc>,
    pub phases: Vec<PhaseDesc>,
}

pub struct SystemDesc {
    /// Unqualified; the loader prefixes the mod's name.
    pub name: String,
    pub phase: String,
    /// Qualified names (`"physics::integrate"`) of systems this one runs
    /// after, or before.
    pub after: Vec<String>,
    pub before: Vec<String>,
    pub access: Vec<Access>,
    /// May touch anything, and change the world's structure on the spot.
    pub exclusive: bool,
    /// This build's code.
    pub run: SystemFn,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Access {
    /// A component's or event's `NAME`.
    pub name: String,
    pub kind: AccessKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessKind {
    Read,
    Write,
    /// Reads events of this type.
    Events,
}

pub struct PhaseDesc {
    pub name: String,
    pub after: Vec<String>,
    pub before: Vec<String>,
}

/// Where a mod declares its systems and phases: the argument to
/// [`Mod::systems`].
pub struct Systems<T> {
    decls: Declarations,
    _mod: PhantomData<fn() -> T>,
}

impl<T: Mod> Systems<T> {
    /// Adds a system named `name` running `f`, in the `update` phase unless
    /// the builder says otherwise. `f` is a function of the mod's state, its
    /// transient part, its `Cx` and up to four parameters ([`Query`],
    /// [`EventReader`]), and must not capture anything: a fn item, or a
    /// closure without captures.
    pub fn add<P, F: IntoSystem<T, P>>(&mut self, name: &str, f: F) -> SystemBuilder<'_> {
        let _ = f;
        let mut access = Vec::new();
        F::declare(&mut access);
        self.decls.systems.push(SystemDesc {
            name: name.into(),
            phase: phase::UPDATE.into(),
            after: Vec::new(),
            before: Vec::new(),
            access,
            exclusive: false,
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

    /// Declares reading `C` through `cx.world()`, beyond the parameters.
    pub fn reads<C: Component>(self) -> Self {
        self.desc.access.push(Access { name: C::NAME.into(), kind: AccessKind::Read });
        self
    }

    /// Declares writing `C` through `cx.world()`, beyond the parameters.
    pub fn writes<C: Component>(self) -> Self {
        self.desc.access.push(Access { name: C::NAME.into(), kind: AccessKind::Write });
        self
    }

    /// Lets the system touch anything, and change the world's structure on
    /// the spot rather than through commands. It runs alone.
    pub fn exclusive(self) -> Self {
        self.desc.exclusive = true;
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

/// A system parameter: something a system receives, and declares by
/// receiving it.
pub trait SystemParam: 'static {
    fn declare(access: &mut Vec<Access>);
    #[doc(hidden)]
    fn make() -> Self;
}

/// A function that can be a system. `P` is its parameters, as a tuple.
/// Implemented for functions of the mod's state, transient part, `Cx` and up
/// to four parameters.
pub trait IntoSystem<T: Mod, P>: Copy + 'static {
    fn declare(access: &mut Vec<Access>);
    #[doc(hidden)]
    fn call(self, state: &mut T, transient: &mut T::Transient, cx: &mut Cx);
}

macro_rules! into_system {
    ($($p:ident),*) => {
        impl<T: Mod, F, $($p: SystemParam),*> IntoSystem<T, ($($p,)*)> for F
        where
            F: Fn(&mut T, &mut T::Transient, &mut Cx, $($p),*) + Copy + 'static,
        {
            fn declare(_access: &mut Vec<Access>) {
                $($p::declare(_access);)*
            }

            fn call(self, state: &mut T, transient: &mut T::Transient, cx: &mut Cx) {
                self(state, transient, cx, $($p::make()),*)
            }
        }
    };
}

into_system!();
into_system!(P1);
into_system!(P1, P2);
into_system!(P1, P2, P3);
into_system!(P1, P2, P3, P4);

/// The loader's entry point for one system: `SystemDesc::run`.
unsafe fn run<T: Mod, P, F: IntoSystem<T, P>>(ctx: *mut ModContext) -> Status {
    // `run` is a plain fn pointer, so the function it calls can't be stored
    // anywhere; it's rebuilt from nothing, which only a zero-sized one can be.
    const { assert!(size_of::<F>() == 0, "a system must not capture anything") };
    let result = catch_unwind(AssertUnwindSafe(|| unsafe {
        let f: F = std::mem::zeroed();
        let state = &mut *((*ctx).state as *mut T);
        let transient = &mut *crate::transient::<T>(&raw mut (*ctx).transient);
        let mut cx = Cx { raw: &mut *ctx };
        f.call(state, transient, &mut cx);
    }));
    if result.is_ok() { Status::OK } else { Status::ERROR }
}

/// Fills `out` (a `Declarations`) with `T`'s: `ModInfo::declare`. `false` if
/// `Mod::systems` panicked.
#[doc(hidden)]
pub unsafe extern "C" fn __declare<T: Mod>(out: *mut c_void) -> bool {
    catch_unwind(|| {
        let mut systems = Systems::<T> { decls: Declarations::default(), _mod: PhantomData };
        T::systems(&mut systems);
        unsafe { *(out as *mut Declarations) = systems.decls };
    })
    .is_ok()
}

/// A query parameter: every entity with all the components `Q` names. `Q` is
/// `&T`, `&mut T`, or a tuple of up to four of them.
pub struct Query<Q>(PhantomData<fn() -> Q>);

impl<Q: QueryData + 'static> SystemParam for Query<Q> {
    fn declare(access: &mut Vec<Access>) {
        let mut terms = Vec::new();
        Q::terms(&mut terms);
        for (i, (name, _)) in terms.iter().enumerate() {
            // One component twice would hand out two `&mut` to one value.
            assert!(!terms[..i].iter().any(|(n, _)| n == name), "a query names {name} twice");
        }
        access.extend(terms.into_iter().map(|(name, write)| Access {
            name: name.into(),
            kind: if write { AccessKind::Write } else { AccessKind::Read },
        }));
    }

    fn make() -> Self {
        Query(PhantomData)
    }
}

impl<Q: QueryData> Query<Q> {
    /// Every entity with all of the query's components, walking the first
    /// one's storage (put the rarest first). Borrows `cx` for the loop, so
    /// calls and commands wait until it's done.
    pub fn iter<'a>(&self, cx: &'a mut Cx) -> impl Iterator<Item = (Entity, Q::Item<'a>)> + 'a {
        let ctx: *const ModContext = cx.raw;
        let ids = Q::ids(ctx);
        let entities: &'a [Entity] = match ids.first() {
            Some(&(id, write)) => unsafe {
                let column = ((*(*ctx).host).world.column)(ctx, id, write);
                if column.len == 0 { &[] } else { std::slice::from_raw_parts(column.entities, column.len) }
            },
            None => &[],
        };
        entities.iter().filter_map(move |&e| Some((e, unsafe { Q::fetch(ctx, &ids, e)? })))
    }

    /// The query's components of one entity, if it has them all.
    pub fn get<'a>(&self, cx: &'a mut Cx, entity: Entity) -> Option<Q::Item<'a>> {
        let ctx: *const ModContext = cx.raw;
        unsafe { Q::fetch(ctx, &Q::ids(ctx), entity) }
    }
}

/// One term of a query: `&T` or `&mut T`.
///
/// # Safety
/// `item` must turn a pointer to a `Component` value into the right kind of
/// reference.
pub unsafe trait Term {
    type Component: Component;
    const WRITE: bool;
    type Item<'a>;
    #[doc(hidden)]
    unsafe fn item<'a>(value: *mut u8) -> Self::Item<'a>;
}

unsafe impl<T: Component> Term for &T {
    type Component = T;
    const WRITE: bool = false;
    type Item<'a> = &'a T;
    unsafe fn item<'a>(value: *mut u8) -> &'a T {
        unsafe { &*(value as *const T) }
    }
}

unsafe impl<T: Component> Term for &mut T {
    type Component = T;
    const WRITE: bool = true;
    type Item<'a> = &'a mut T;
    unsafe fn item<'a>(value: *mut u8) -> &'a mut T {
        unsafe { &mut *(value as *mut T) }
    }
}

/// What a [`Query`] fetches per entity.
///
/// # Safety
/// `fetch` must use the ids `ids` returned, in order.
pub unsafe trait QueryData {
    type Item<'a>;
    #[doc(hidden)]
    fn terms(out: &mut Vec<(&'static str, bool)>);
    #[doc(hidden)]
    fn ids(ctx: *const ModContext) -> Vec<(ComponentId, bool)>;
    #[doc(hidden)]
    unsafe fn fetch<'a>(ctx: *const ModContext, ids: &[(ComponentId, bool)], e: Entity) -> Option<Self::Item<'a>>;
}

fn register<T: Component>(ctx: *const ModContext) -> ComponentId {
    unsafe { ((*(*ctx).host).world.register)(ctx, &ComponentDesc::of::<T>()) }
}

unsafe fn get(ctx: *const ModContext, (id, write): (ComponentId, bool), e: Entity) -> Option<*mut u8> {
    let value = unsafe { ((*(*ctx).host).world.get)(ctx, e, id, write) };
    (!value.is_null()).then_some(value)
}

unsafe impl<A: Term> QueryData for A {
    type Item<'a> = A::Item<'a>;
    fn terms(out: &mut Vec<(&'static str, bool)>) {
        out.push((A::Component::NAME, A::WRITE));
    }
    fn ids(ctx: *const ModContext) -> Vec<(ComponentId, bool)> {
        vec![(register::<A::Component>(ctx), A::WRITE)]
    }
    unsafe fn fetch<'a>(ctx: *const ModContext, ids: &[(ComponentId, bool)], e: Entity) -> Option<Self::Item<'a>> {
        unsafe { Some(A::item(get(ctx, ids[0], e)?)) }
    }
}

macro_rules! query_tuple {
    ($($t:ident : $i:tt),+) => {
        unsafe impl<$($t: Term),+> QueryData for ($($t,)+) {
            type Item<'a> = ($($t::Item<'a>,)+);
            fn terms(out: &mut Vec<(&'static str, bool)>) {
                $(out.push(($t::Component::NAME, $t::WRITE));)+
            }
            fn ids(ctx: *const ModContext) -> Vec<(ComponentId, bool)> {
                vec![$((register::<$t::Component>(ctx), $t::WRITE)),+]
            }
            unsafe fn fetch<'a>(ctx: *const ModContext, ids: &[(ComponentId, bool)], e: Entity) -> Option<Self::Item<'a>> {
                // Every pointer first: a missing component ends the entity
                // before any reference is made.
                let values = [$(unsafe { get(ctx, ids[$i], e)? }),+];
                unsafe { Some(($($t::item(values[$i]),)+)) }
            }
        }
    };
}

query_tuple!(A: 0);
query_tuple!(A: 0, B: 1);
query_tuple!(A: 0, B: 1, C: 2);
query_tuple!(A: 0, B: 1, C: 2, D: 3);

/// An event type: a value one mod sends (`cx.send_event`) and others read
/// with an [`EventReader`]. Declared with [`event!`](crate::event), under the
/// same rules as a component.
///
/// # Safety
/// As for [`Component`].
pub unsafe trait Event: Component {}

/// Declares an event type, with the syntax of [`component!`](crate::component).
#[macro_export]
macro_rules! event {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident : $id:literal $($rest:tt)*
    ) => {
        $crate::component! { $(#[$meta])* $vis struct $name : $id $($rest)* }
        unsafe impl $crate::Event for $name {}
    };
}

/// A system parameter: the events of type `E` this system hasn't read yet.
pub struct EventReader<E>(PhantomData<fn() -> E>);

impl<E: Event> SystemParam for EventReader<E> {
    fn declare(access: &mut Vec<Access>) {
        access.push(Access { name: E::NAME.into(), kind: AccessKind::Events });
    }

    fn make() -> Self {
        EventReader(PhantomData)
    }
}

impl<E: Event> EventReader<E> {
    /// The events sent since this system last read them, oldest first; each
    /// is returned once. Events sent this phase aren't visible until the next.
    pub fn read<'a>(&self, cx: &'a mut Cx) -> &'a [E] {
        let ctx: *const ModContext = cx.raw;
        unsafe {
            let api = &(*(*ctx).host).world;
            let id = (api.register_event)(ctx, &ComponentDesc::of::<E>());
            let events = (api.read_events)(ctx, id);
            if events.len == 0 { &[] } else { std::slice::from_raw_parts(events.data as *const E, events.len) }
        }
    }
}

/// Structural changes, queued until the end of the phase: see [`Cx::commands`].
pub struct Commands<'a> {
    ctx: *const ModContext,
    _cx: PhantomData<&'a mut ModContext>,
}

impl<'a> Commands<'a> {
    pub(crate) fn new(ctx: &'a mut ModContext) -> Commands<'a> {
        Commands { ctx, _cx: PhantomData }
    }

    fn api(&self) -> &crate::WorldApi {
        unsafe { &(*(*self.ctx).host).world }
    }

    /// A new entity, at once: allocating one touches no component storage.
    pub fn spawn(&mut self) -> Entity {
        unsafe { (self.api().spawn)(self.ctx) }
    }

    /// Queues moving `value` onto `entity`, replacing any existing one.
    pub fn insert<T: Component>(&mut self, entity: Entity, value: T) {
        let id = register::<T>(self.ctx);
        let value = std::mem::ManuallyDrop::new(value);
        let moved = unsafe { (self.api().defer_insert)(self.ctx, entity, id, &*value as *const T as *const u8) };
        if !moved {
            drop(std::mem::ManuallyDrop::into_inner(value));
        }
    }

    pub fn remove<T: Component>(&mut self, entity: Entity) {
        let id = register::<T>(self.ctx);
        unsafe { (self.api().defer_remove)(self.ctx, entity, id) }
    }

    pub fn despawn(&mut self, entity: Entity) {
        unsafe { (self.api().defer_despawn)(self.ctx, entity) }
    }
}

impl Cx<'_> {
    /// Structural changes (insert, remove, despawn) queued until the end of
    /// the phase, so every system in a phase sees the same world. Outside a
    /// frame, they're applied at the start of the next.
    pub fn commands(&mut self) -> Commands<'_> {
        Commands::new(self.raw)
    }

    /// Sends an event, visible to readers from the next phase boundary. Works
    /// anywhere, including message handlers between frames.
    pub fn send_event<E: Event>(&mut self, event: E) {
        let ctx: *const ModContext = self.raw;
        let event = std::mem::ManuallyDrop::new(event);
        unsafe {
            let api = &(*(*ctx).host).world;
            let id = (api.register_event)(ctx, &ComponentDesc::of::<E>());
            if !(api.send_event)(ctx, id, &*event as *const E as *const u8) {
                drop(std::mem::ManuallyDrop::into_inner(event));
            }
        }
    }
}
