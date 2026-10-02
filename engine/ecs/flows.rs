//! Flows: typed values that live one frame (one step, in a fixed-rate
//! group) and pass from system to system, as values pass between the
//! stages of a map/reduce pipeline. A source makes one from the world,
//! stages see, edit or take it, and a sink writes what it carries back.
//! Each use is a parameter, so a declaration the frame's graph orders and
//! the plan check ([`check_plan`]) refuses out of turn:
//!
//! - [`Make<T>`]: this frame's `T`, starting from last frame's emptied;
//! - [`See<T>`]: borrows it, any number at once;
//! - [`Pass<T>`]: edits it in place and hands it on;
//! - [`Take<T>`]: owns it; dropped, it goes back to the bin.
//!
//! The world keeps each flow's value while a frame runs, and one emptied
//! value between frames (its bin), by name. See
//! docs/architecture/flows.md.
//!
//! The typed hand-off is Bevy's system piping (`pipe`, `In<T>`) made an
//! edge of its own; the kept allocations are Bevy's `Local<T>` and Timely
//! Dataflow's buffers handed back by swapping (docs/CREDITS.md).

use std::any::{Any, TypeId};
use std::collections::{BTreeMap, VecDeque};
use std::ops::{Deref, DerefMut};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex, PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};

use crate::query::{Declare, FrameCx, Param, ParamDecl};
use crate::world::{Build, Keepalive, TakeGuard, World};

// ---- Declaring ----

/// Emptied for the next frame's maker, its allocations kept.
pub trait Recycle {
    fn recycle(&mut self);
}

impl<T> Recycle for Vec<T> {
    fn recycle(&mut self) {
        self.clear();
    }
}

impl<T> Recycle for VecDeque<T> {
    fn recycle(&mut self) {
        self.clear();
    }
}

impl Recycle for String {
    fn recycle(&mut self) {
        self.clear();
    }
}

/// A value the maker sets whole each frame, such as its settings.
impl<T> Recycle for Option<T> {
    fn recycle(&mut self) {
        *self = None;
    }
}

macro_rules! scalar {
    ($($t:ty),*) => {
        $(impl Recycle for $t {
            fn recycle(&mut self) {
                *self = <$t>::default();
            }
        })*
    };
}
scalar!(bool, u8, u16, u32, u64, usize, i8, i16, i32, i64, isize, f32, f64);

/// A value that lives one frame: declared with [`flow!`](crate::flow).
pub trait Flow: Default + Send + Sync + 'static {
    /// What it's found by: two mods naming one flow share it.
    const NAME: &'static str;
    fn recycle(&mut self);
}

/// Declares a flow: a struct whose fields each [`Recycle`].
///
/// ```ignore
/// engine_api::flow! {
///     pub struct Bodies: "physics2d::flow::Bodies" {
///         pub entities: Vec<Entity>,
///         pub bodies: Vec<SolverBody>,
///     }
/// }
/// ```
#[macro_export]
macro_rules! flow {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident : $id:literal {
            $($(#[$fmeta:meta])* $fvis:vis $field:ident : $ty:ty),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Default)]
        $vis struct $name {
            $($(#[$fmeta])* $fvis $field: $ty),*
        }

        impl $crate::Flow for $name {
            const NAME: &'static str = $id;
            fn recycle(&mut self) {
                $($crate::Recycle::recycle(&mut self.$field);)*
            }
        }
    };
}

/// How a system uses a flow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlowAccess {
    Make,
    See,
    Pass,
    Take,
}

impl FlowAccess {
    /// Whether it excludes every other use while it runs: all but `See`.
    pub fn writes(self) -> bool {
        self != FlowAccess::See
    }
}

fn declare<T: Flow>(d: &mut Declare<'_>, access: FlowAccess) -> ParamDecl {
    let slot = d.flow::<T>();
    ParamDecl::Flow { slot, name: T::NAME, access, ty: TypeId::of::<T>(), ty_name: std::any::type_name::<T>() }
}

// ---- The store ----

/// A flow's value as the world holds it. An `Arc` so that each `See` can
/// hold its own typed handle, downcast once at fetch: a `Box<dyn Any>`
/// borrowed through a guard would be downcast, through its vtable, on every
/// deref, and a gather derefs its `Make` once a row. The exclusive uses move
/// the value out (`Arc::try_unwrap`) and back, an allocation each.
type Shared = Arc<dyn Any + Send + Sync>;

/// One flow: its value while a frame runs, and the emptied one kept for the
/// next `Make`.
pub(crate) struct FlowSlot {
    value: RwLock<Option<Shared>>,
    bin: Mutex<Option<Shared>>,
    /// The builds of every system that has used the flow, by build name:
    /// the value and the bin are boxed by one of them, and drop with its
    /// code. A `BTreeMap`, not a `HashMap`: a slot is made by whichever
    /// build first declares the flow, and an empty `HashMap` points into the
    /// image that made it (docs/lore/an-empty-hashmap-points-into-the-build-that-made-it.md).
    /// After `value` and `bin`, so they drop while it still maps their code.
    keepalives: Mutex<BTreeMap<String, Keepalive>>,
}

impl FlowSlot {
    pub(crate) fn new() -> FlowSlot {
        FlowSlot { value: RwLock::new(None), bin: Mutex::new(None), keepalives: Mutex::new(BTreeMap::new()) }
    }

    fn bin(&self) -> std::sync::MutexGuard<'_, Option<Shared>> {
        self.bin.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Moves the value into the bin, or drops it if the bin is full or
    /// recycling is off: what the end of a frame does with a value nothing
    /// took.
    fn expire(&self, recycling: bool) {
        let value = self.value.take_write().take();
        if let Some(value) = value {
            let mut bin = self.bin();
            if recycling && bin.is_none() {
                *bin = Some(value);
            }
        }
    }
}

impl World {
    /// The slot for flow `name`, made if it's new.
    pub fn intern_flow(&self, name: &str) -> usize {
        let mut by_name = self.flows_by_name.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(&f) = by_name.get(name) {
            return f;
        }
        let f = self.flows.push(FlowSlot::new());
        by_name.insert(name.into(), f);
        f
    }

    /// Records that `build` uses flow `name`, as its load commits: drops
    /// the flow's value and bin, whose type and drop code may be an older
    /// build's (a layout `TypeId` can't tell apart), and keeps `build`
    /// mapped while the flow may hold its values. The next `Make` starts
    /// from nothing. Between frames only, while every build that could
    /// have made the bin is still mapped.
    pub fn install_flow(&self, name: &str, build: &Build) {
        assert!(!self.frame_open(), "flows are installed between frames");
        let slot = self.flows.get(self.intern_flow(name));
        drop(slot.value.take_write().take());
        drop(slot.bin().take());
        if let Some(keepalive) = &build.keepalive {
            slot.keepalives.lock().unwrap_or_else(PoisonError::into_inner).insert(build.name.clone(), keepalive.clone());
        }
    }

    /// Whether `Make` starts from last frame's allocations (the default) or
    /// from nothing: for measuring what recycling is worth.
    pub fn set_flow_recycling(&self, on: bool) {
        self.flow_recycling.store(on, Ordering::Relaxed);
    }

    /// Empties every flow at the end of a frame, so none is seen in the
    /// next, and none between frames.
    pub(crate) fn expire_flows(&self) {
        let recycling = self.flow_recycling.load(Ordering::Relaxed);
        for slot in self.flows.iter() {
            slot.expire(recycling);
        }
    }

    /// Whether flow `name` holds a value or a bin: for tests of the store.
    pub fn flow_holds(&self, name: &str) -> (bool, bool) {
        let Some(&f) = self.flows_by_name.lock().unwrap_or_else(PoisonError::into_inner).get(name) else { return (false, false) };
        let slot = self.flows.get(f);
        let value = slot.value.read().unwrap_or_else(PoisonError::into_inner).is_some();
        (value, slot.bin().is_some())
    }

    /// The builds flow `name` keeps mapped, by name: for tests.
    pub fn flow_keepalives(&self, name: &str) -> Vec<String> {
        let Some(&f) = self.flows_by_name.lock().unwrap_or_else(PoisonError::into_inner).get(name) else { return Vec::new() };
        self.flows.get(f).keepalives.lock().unwrap_or_else(PoisonError::into_inner).keys().cloned().collect()
    }
}

fn slot<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> &'w FlowSlot {
    let ParamDecl::Flow { slot, .. } = decl else { panic!("a flow's declaration") };
    cx.world.flows.get(*slot)
}

/// `value` as a `T` of its own, moved out of its `Arc`.
fn unshare<T: Flow>(value: Shared, use_: &str) -> T {
    let typed = value.downcast::<T>().unwrap_or_else(|_| panic!("{}: declared as two types; {use_} found another", T::NAME));
    Arc::try_unwrap(typed).unwrap_or_else(|_| panic!("{}: {use_} while it's still seen: a scheduler bug", T::NAME))
}

fn missing(name: &str, use_: &str) -> ! {
    panic!("{name}: {use_} with none made this frame, or after it was taken; the plan check refuses such a plan")
}

// ---- The parameters ----

/// Makes this frame's `T`, from last frame's emptied one where there is
/// one. What it holds when the system returns is the frame's.
pub struct Make<'w, T: Flow> {
    guard: RwLockWriteGuard<'w, Option<Shared>>,
    value: Option<T>,
}

impl<T: Flow> Deref for Make<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.value.as_ref().expect("made at fetch")
    }
}

impl<T: Flow> DerefMut for Make<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.value.as_mut().expect("made at fetch")
    }
}

impl<T: Flow> Drop for Make<'_, T> {
    fn drop(&mut self) {
        if let Some(value) = self.value.take() {
            *self.guard = Some(Arc::new(value));
        }
    }
}

impl<T: Flow> Param for Make<'static, T> {
    type Item<'w> = Make<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, FlowAccess::Make)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Make<'w, T> {
        let slot = slot(cx, decl);
        let mut guard = slot.value.take_write();
        // A value already here is an earlier step's that nothing took: the
        // plan check allows one `Make` a step, so it's this step's start.
        let start = guard.take().or_else(|| slot.bin().take());
        let recycling = cx.world.flow_recycling.load(Ordering::Relaxed);
        let mut value = match start.filter(|_| recycling) {
            Some(start) => unshare::<T>(start, "made"),
            None => T::default(),
        };
        value.recycle();
        Make { guard, value: Some(value) }
    }
}

/// Borrows this frame's `T`.
pub struct See<'w, T: Flow> {
    // Held for its lock alone: a `Pass` or `Take` the graph failed to order
    // after this finds it taken, and panics, rather than racing.
    _guard: RwLockReadGuard<'w, Option<Shared>>,
    value: Arc<T>,
}

impl<T: Flow> Deref for See<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.value
    }
}

impl<T: Flow> Param for See<'static, T> {
    type Item<'w> = See<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, FlowAccess::See)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> See<'w, T> {
        let guard = slot(cx, decl).value.take_read();
        let shared = guard.as_ref().unwrap_or_else(|| missing(T::NAME, "seen")).clone();
        let value = shared.downcast::<T>().unwrap_or_else(|_| panic!("{}: declared as two types; seen as another", T::NAME));
        See { _guard: guard, value }
    }
}

/// Edits this frame's `T` in place, for whoever comes after.
pub struct Pass<'w, T: Flow> {
    guard: RwLockWriteGuard<'w, Option<Shared>>,
    value: Option<T>,
}

impl<T: Flow> Deref for Pass<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.value.as_ref().expect("passed at fetch")
    }
}

impl<T: Flow> DerefMut for Pass<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.value.as_mut().expect("passed at fetch")
    }
}

impl<T: Flow> Drop for Pass<'_, T> {
    fn drop(&mut self) {
        if let Some(value) = self.value.take() {
            *self.guard = Some(Arc::new(value));
        }
    }
}

impl<T: Flow> Param for Pass<'static, T> {
    type Item<'w> = Pass<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, FlowAccess::Pass)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Pass<'w, T> {
        let mut guard = slot(cx, decl).value.take_write();
        let shared = guard.take().unwrap_or_else(|| missing(T::NAME, "passed"));
        Pass { guard, value: Some(unshare::<T>(shared, "passed")) }
    }
}

/// Owns this frame's `T`: nothing after it sees it. Dropped, it goes back to
/// the bin for the next `Make`; `into_inner` keeps it instead.
pub struct Take<'w, T: Flow> {
    // Held for its lock alone, as `See`'s is.
    _guard: RwLockWriteGuard<'w, Option<Shared>>,
    slot: &'w FlowSlot,
    value: Option<T>,
    recycling: bool,
}

impl<T: Flow> Take<'_, T> {
    pub fn into_inner(mut self) -> T {
        self.value.take().expect("taken at fetch")
    }
}

impl<T: Flow> Deref for Take<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.value.as_ref().expect("taken at fetch")
    }
}

impl<T: Flow> DerefMut for Take<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.value.as_mut().expect("taken at fetch")
    }
}

impl<T: Flow> Drop for Take<'_, T> {
    fn drop(&mut self) {
        if let Some(value) = self.value.take()
            && self.recycling
        {
            let mut bin = self.slot.bin();
            if bin.is_none() {
                *bin = Some(Arc::new(value));
            }
        }
    }
}

impl<T: Flow> Param for Take<'static, T> {
    type Item<'w> = Take<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, FlowAccess::Take)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Take<'w, T> {
        let slot = slot(cx, decl);
        let mut guard = slot.value.take_write();
        let shared = guard.take().unwrap_or_else(|| missing(T::NAME, "taken"));
        let recycling = cx.world.flow_recycling.load(Ordering::Relaxed);
        Take { _guard: guard, slot, value: Some(unshare::<T>(shared, "taken")), recycling }
    }
}

// ---- The plan check ----

/// A system as the plan check sees it, in plan order.
pub struct PlanStep<'a> {
    /// `mod::system`.
    pub system: &'a str,
    pub phase: &'a str,
    /// The fixed-rate group it runs in, named for messages (`"simulate at
    /// 60 Hz"`), or `None` once a frame. A flow can't cross groups.
    pub group: Option<&'a str>,
    pub params: &'a [ParamDecl],
}

struct Use<'a> {
    step: &'a PlanStep<'a>,
    access: FlowAccess,
    ty: TypeId,
    ty_name: &'static str,
}

impl Use<'_> {
    fn what(&self, name: &str) -> String {
        format!("`{:?}<{name}>` in `{}`", self.access, self.step.system)
    }

    fn group(&self) -> String {
        self.step.group.unwrap_or("once a frame").to_string()
    }
}

/// Refuses a plan whose flows are used out of turn, with an error naming
/// the fix; see docs/architecture/flows.md, "The plan check". `steps` are
/// one run of the frame's node list, a fixed-rate group's phases once:
/// every step of a group repeats them.
pub fn check_plan(steps: &[PlanStep<'_>]) -> Result<(), String> {
    let mut flows: Vec<(&'static str, Vec<Use<'_>>)> = Vec::new();
    for step in steps {
        for p in ParamDecl::leaves(step.params) {
            let ParamDecl::Flow { name, access, ty, ty_name, .. } = p else { continue };
            let use_ = Use { step, access: *access, ty: *ty, ty_name };
            match flows.iter_mut().find(|(n, _)| n == name) {
                Some((_, uses)) => uses.push(use_),
                None => flows.push((name, vec![use_])),
            }
        }
    }
    for (name, uses) in &flows {
        check_flow(name, uses)?;
    }
    Ok(())
}

fn check_flow(name: &str, uses: &[Use<'_>]) -> Result<(), String> {
    let first = &uses[0];
    if let Some(other) = uses.iter().find(|u| u.step.group != first.step.group) {
        return Err(format!(
            "{} ({}) and {} ({}): a flow lives one step of its group, so its uses can't cross groups; use it in one group, or \
             carry the value across in a component or an event",
            other.what(name),
            other.group(),
            first.what(name),
            first.group(),
        ));
    }
    if let Some(other) = uses.iter().find(|u| u.ty != first.ty) {
        return Err(format!(
            "{name} is declared as two types: `{}` by `{}` and `{}` by `{}`; a flow shared between mods is declared once, in its \
             mod's interface",
            first.ty_name, first.step.system, other.ty_name, other.step.system,
        ));
    }
    // Where the flow is: made by `by`, or taken by `by`.
    enum State<'u, 'a> {
        Unmade,
        Made(&'u Use<'a>),
        Taken(&'u Use<'a>),
    }
    let mut state = State::Unmade;
    for (i, u) in uses.iter().enumerate() {
        state = match (u.access, state) {
            (FlowAccess::Make, State::Made(by)) => {
                return Err(format!(
                    "{} and `Make<{name}>` in `{}`: a flow has one maker; a stage that changes it takes `Pass<{name}>`",
                    u.what(name),
                    by.step.system
                ));
            }
            (FlowAccess::Make, _) => State::Made(u),
            (_, State::Unmade) => {
                let Some(maker) = uses[i..].iter().find(|m| m.access == FlowAccess::Make) else {
                    return Err(format!("{}: nothing loaded makes {name}", u.what(name)));
                };
                return Err(if maker.step.phase == u.step.phase {
                    format!(
                        "{} runs before {}; add `.after(\"{}\")` to `{}`",
                        u.what(name),
                        maker.what(name),
                        maker.step.system,
                        u.step.system
                    )
                } else {
                    format!(
                        "{} (phase {}) runs before {} (phase {}); move `{}` to phase {}, with `.after(\"{}\")`",
                        u.what(name),
                        u.step.phase,
                        maker.what(name),
                        maker.step.phase,
                        u.step.system,
                        maker.step.phase,
                        maker.step.system
                    )
                });
            }
            (_, State::Taken(taker)) => {
                return Err(if u.access == FlowAccess::Take {
                    format!(
                        "{} runs after {} took it: a flow has one taker; make one of them a `Pass` that runs before the other",
                        u.what(name),
                        taker.what(name)
                    )
                } else if taker.step.phase == u.step.phase {
                    format!(
                        "{} runs after {} took it; add `.before(\"{}\")` to `{}`",
                        u.what(name),
                        taker.what(name),
                        taker.step.system,
                        u.step.system
                    )
                } else {
                    format!(
                        "{} (phase {}) runs after {} (phase {}) took it; move `{}` to phase {}, with `.before(\"{}\")`",
                        u.what(name),
                        u.step.phase,
                        taker.what(name),
                        taker.step.phase,
                        u.step.system,
                        taker.step.phase,
                        taker.step.system
                    )
                });
            }
            (FlowAccess::Take, State::Made(_)) => State::Taken(u),
            (_, made) => made,
        };
    }
    Ok(())
}
