//! SPIKE, not engine code: the mechanism behind docs/architecture/flows-spike.md.
//! Kept so the numbers can be taken again; nothing in the engine depends on
//! it.
//!
//! A **flow** is a typed value that lives one frame and passes from system
//! to system, as a value passes between the stages of a map/reduce
//! pipeline: a source makes it from the world, stages see, edit or take it,
//! and a sink writes what it carries back to the world. Systems take it as
//! parameters, so each use is a declaration, and so an edge the frame's
//! graph orders on:
//!
//! - [`Make<T>`]: produces this frame's `T`, starting from last frame's
//!   emptied (its allocations kept: [`Recycle`]);
//! - [`See<T>`]: borrows it, any number at once;
//! - [`Pass<T>`]: edits it in place and hands it on (a hook between a
//!   source and its consumer);
//! - [`Take<T>`]: owns it, at most once a frame; dropped, its allocations
//!   go back for the next `Make`.
//!
//! Built on `engine_ecs`'s public API, without changing it:
//!
//! - **The edge is an event queue.** Each flow declares a marker event of
//!   its name, and its parameters declare the queue as `See` reads it and
//!   the others write it, so `graph.rs`'s event rule orders them exactly as
//!   a flow needs: `See`s together, anything else one at a time, in plan
//!   order. Writers get apply nodes that publish nothing.
//! - **The access kind rides in the declaration** as a count of `Dt`
//!   leaves (which touch nothing), for [`check`] to read: a stand-in for a
//!   `ParamDecl::Flow { access }` the real thing would add.
//! - **The values live beside the world**, in a store found by the world's
//!   address and leaked for the process: the real thing would keep them in
//!   the world, as it keeps event queues.
//!
//! The typed hand-off is Bevy's system piping (`pipe`, `In<T>`) made an
//! edge of its own; the kept allocations are Bevy's `Local<T>` and Timely
//! Dataflow's buffers handed back by swapping (docs/CREDITS.md).
//!
//! And the shapes parallel work takes, so that the scheduler sees all of
//! it rather than a system's own use of the pool (parallel-relations.md,
//! phase 2, rejected): [`par_map`], [`par_for_each_mut`], [`reduce`] (in an
//! order fixed by the input, not the threads) and [`Colored::passes`], a
//! graph's edges in colors, each color split across the host's threads.

use std::any::Any;
use std::collections::HashMap;
use std::ops::{Deref, DerefMut, Range};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Mutex, RwLock, RwLockReadGuard, RwLockWriteGuard};

use engine_ecs::harness::Schedule;
use engine_ecs::{Declare, Event, FrameCx, Param, ParamDecl, Workers, World};

// ---- Flows ----

/// Emptied for the next frame's producer, its allocations kept.
pub trait Recycle {
    fn recycle(&mut self);
}

impl<T> Recycle for Vec<T> {
    fn recycle(&mut self) {
        self.clear();
    }
}

/// A value the producer sets whole each frame, such as its settings.
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
scalar!(bool, u8, u32, u64, usize, i32, f32, f64);

/// A value that lives one frame: declared with [`flow!`](crate::flow).
pub trait Flow: Default + Send + Sync + 'static {
    /// Its name, which is also its edge's: two mods naming one flow share it.
    const NAME: &'static str;
    /// The marker event whose queue stands for the edge in the graph.
    type Edge: Event;
    fn recycle(&mut self);
}

/// Declares a flow: a struct whose fields each [`Recycle`], and its edge.
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

        const _: () = {
            ::engine_ecs::event! {
                #[derive(Default)]
                pub struct Edge: $id {}
            }
            impl $crate::Flow for $name {
                const NAME: &'static str = $id;
                type Edge = Edge;
                fn recycle(&mut self) {
                    $($crate::Recycle::recycle(&mut self.$field);)*
                }
            }
        };
    };
}

/// How a system uses a flow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    See = 1,
    Pass = 2,
    Take = 3,
    Make = 4,
}

impl Access {
    fn of(dts: usize) -> Option<Access> {
        [Access::See, Access::Pass, Access::Take, Access::Make].into_iter().find(|a| *a as usize == dts)
    }
}

/// One flow's value this frame, and the emptied one kept for the next.
struct Slot<T> {
    value: RwLock<Option<T>>,
    /// The frame `value` was made in.
    made: AtomicU64,
    bin: Mutex<Option<T>>,
}

/// A world's flows, by name, and its edges' names by queue.
#[derive(Default)]
struct Store {
    slots: Mutex<HashMap<&'static str, &'static (dyn Any + Send + Sync)>>,
    names: Mutex<HashMap<usize, &'static str>>,
    fresh: AtomicBool,
}

static STORES: Mutex<Vec<(usize, &'static Store)>> = Mutex::new(Vec::new());

fn store(world: &World) -> &'static Store {
    let key = world as *const World as usize;
    let mut stores = STORES.lock().unwrap();
    if let Some((_, s)) = stores.iter().find(|(k, _)| *k == key) {
        return s;
    }
    let s: &'static Store = Box::leak(Box::default());
    stores.push((key, s));
    s
}

/// Forgets `world`'s flows: for a new world at an address another had, as
/// tests make them.
pub fn reset(world: &World) {
    let key = world as *const World as usize;
    STORES.lock().unwrap().retain(|(k, _)| *k != key);
}

/// Whether `Make` starts from last frame's allocations (the default) or
/// from nothing: for measuring what recycling is worth.
pub fn set_recycling(world: &World, on: bool) {
    store(world).fresh.store(!on, Ordering::Relaxed);
}

fn slot<T: Flow>(world: &World) -> &'static Slot<T> {
    let s = store(world);
    let mut slots = s.slots.lock().unwrap();
    let any = *slots.entry(T::NAME).or_insert_with(|| {
        let slot: &'static Slot<T> = Box::leak(Box::new(Slot { value: RwLock::new(None), made: AtomicU64::new(0), bin: Mutex::new(None) }));
        slot
    });
    any.downcast_ref::<Slot<T>>().unwrap_or_else(|| panic!("{} declared as two types", T::NAME))
}

fn declare<T: Flow>(d: &mut Declare<'_>, access: Access) -> ParamDecl {
    let queue = d.event::<T::Edge>();
    store(d.world).names.lock().unwrap().insert(queue, T::NAME);
    ParamDecl::Group(vec![
        ParamDecl::Events { queue, write: access != Access::See },
        ParamDecl::Group(vec![ParamDecl::Dt; access as usize]),
    ])
}

/// A system's flows and how it uses each, from its declarations.
pub fn accesses(params: &[ParamDecl]) -> Vec<(usize, Access)> {
    let mut out = Vec::new();
    for p in params {
        if let ParamDecl::Group(members) = p {
            match members.as_slice() {
                [ParamDecl::Events { queue, .. }, ParamDecl::Group(tag)] if tag.iter().all(|t| matches!(t, ParamDecl::Dt)) => {
                    if let Some(a) = Access::of(tag.len()) {
                        out.push((*queue, a));
                        continue;
                    }
                }
                _ => {}
            }
            out.extend(accesses(members));
        }
    }
    out
}

/// What a frame's flows go through, checked against the plan before it
/// runs: each flow made at most once, and seen, edited or taken only after
/// it's made and before it's taken. A flow made and never taken is fine:
/// the next `Make` recycles it. Returns notes on flows nothing reads.
pub fn check(world: &World, schedule: &Schedule) -> Result<Vec<String>, String> {
    #[derive(Clone, Copy, PartialEq)]
    enum State {
        Unmade,
        Made { read: bool },
        Taken,
    }
    let names = store(world).names.lock().unwrap();
    let name = |q: usize| names.get(&q).copied().unwrap_or("?");
    let mut state: HashMap<usize, (State, &str)> = HashMap::new();
    for s in &schedule.systems {
        for (q, a) in accesses(&s.params) {
            let (st, by) = state.get(&q).copied().unwrap_or((State::Unmade, ""));
            let next = match (a, st) {
                (Access::Make, State::Made { .. }) => return Err(format!("{}: made by {by} and again by {}", name(q), s.name)),
                (Access::Make, _) => State::Made { read: false },
                (_, State::Unmade) => return Err(format!("{}: {a:?} by {} before anything makes it", name(q), s.name)),
                (_, State::Taken) => return Err(format!("{}: {a:?} by {} after {by} took it", name(q), s.name)),
                (Access::Take, State::Made { .. }) => State::Taken,
                (_, State::Made { .. }) => State::Made { read: true },
            };
            state.insert(q, (next, s.name.as_str()));
        }
    }
    let mut notes: Vec<String> = state
        .iter()
        .filter(|(_, (st, _))| *st == State::Made { read: false })
        .map(|(q, (_, by))| format!("{}: made by {by}, read by nothing", name(*q)))
        .collect();
    notes.sort();
    Ok(notes)
}

impl<T: Flow> Slot<T> {
    fn current(&self, world: &World, value: &Option<T>) -> bool {
        value.is_some() && self.made.load(Ordering::Acquire) == world.frame()
    }

    /// Last frame's value, if nothing took it, emptied into the bin.
    fn expire(&self, value: &mut Option<T>, fresh: bool) {
        // Taken whether or not it's kept: fresh, it's dropped here.
        if let Some(mut old) = value.take()
            && !fresh
        {
            old.recycle();
            *self.bin.lock().unwrap() = Some(old);
        }
    }
}

fn contended(name: &str) -> ! {
    panic!("{name}: two systems at once where the graph should have ordered them: a scheduler bug")
}

/// Borrows this frame's `T`.
pub struct See<'w, T: Flow>(RwLockReadGuard<'w, Option<T>>);

impl<T: Flow> Deref for See<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.0.as_ref().expect("checked at fetch")
    }
}

impl<T: Flow> Param for See<'static, T> {
    type Item<'w> = See<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, Access::See)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> See<'w, T> {
        let slot = slot::<T>(cx.world);
        let value = slot.value.try_read().unwrap_or_else(|_| contended(T::NAME));
        assert!(slot.current(cx.world, &value), "{}: seen with none made this frame, or taken", T::NAME);
        See(value)
    }
}

/// Edits this frame's `T` in place, for whoever comes after.
pub struct Pass<'w, T: Flow>(RwLockWriteGuard<'w, Option<T>>);

impl<T: Flow> Deref for Pass<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.0.as_ref().expect("checked at fetch")
    }
}

impl<T: Flow> DerefMut for Pass<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.0.as_mut().expect("checked at fetch")
    }
}

impl<T: Flow> Param for Pass<'static, T> {
    type Item<'w> = Pass<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, Access::Pass)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> Pass<'w, T> {
        let slot = slot::<T>(cx.world);
        let value = slot.value.try_write().unwrap_or_else(|_| contended(T::NAME));
        assert!(slot.current(cx.world, &value), "{}: passed with none made this frame, or taken", T::NAME);
        Pass(value)
    }
}

/// Makes this frame's `T`, from last frame's emptied one where there is
/// one. What it holds when the system returns is the frame's.
pub struct Make<'w, T: Flow>(RwLockWriteGuard<'w, Option<T>>);

impl<T: Flow> Deref for Make<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        self.0.as_ref().expect("made at fetch")
    }
}

impl<T: Flow> DerefMut for Make<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.0.as_mut().expect("made at fetch")
    }
}

impl<T: Flow> Param for Make<'static, T> {
    type Item<'w> = Make<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, Access::Make)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> Make<'w, T> {
        let slot = slot::<T>(cx.world);
        let fresh = store(cx.world).fresh.load(Ordering::Relaxed);
        let mut value = slot.value.try_write().unwrap_or_else(|_| contended(T::NAME));
        let now = cx.world.frame();
        assert!(!(value.is_some() && slot.made.load(Ordering::Acquire) == now), "{}: made twice in a frame", T::NAME);
        slot.expire(&mut value, fresh);
        let start = if fresh { None } else { slot.bin.lock().unwrap().take() };
        *value = Some(start.unwrap_or_default());
        slot.made.store(now, Ordering::Release);
        Make(value)
    }
}

/// Owns this frame's `T`: nothing after sees it. Dropped, it goes back for
/// the next `Make`, emptied; `into_inner` keeps it instead.
pub struct Take<'w, T: Flow> {
    value: Option<T>,
    slot: &'w Slot<T>,
    fresh: bool,
}

impl<T: Flow> Take<'_, T> {
    pub fn into_inner(mut self) -> T {
        self.value.take().expect("taken once")
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
        if let Some(mut v) = self.value.take()
            && !self.fresh
        {
            v.recycle();
            *self.slot.bin.lock().unwrap() = Some(v);
        }
    }
}

impl<T: Flow> Param for Take<'static, T> {
    type Item<'w> = Take<'w, T>;

    fn declare(d: &mut Declare<'_>) -> ParamDecl {
        declare::<T>(d, Access::Take)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> Take<'w, T> {
        let slot = slot::<T>(cx.world);
        let mut value = slot.value.try_write().unwrap_or_else(|_| contended(T::NAME));
        assert!(slot.current(cx.world, &value), "{}: taken with none made this frame, or taken already", T::NAME);
        let fresh = store(cx.world).fresh.load(Ordering::Relaxed);
        Take { value: value.take(), slot, fresh }
    }
}

// ---- Parallel shapes ----

/// `f` over each item, in parallel: the results in the items' order.
pub fn par_map<T: Sync, R: Send>(workers: &Workers, items: &[T], min: usize, f: impl Fn(usize, &T) -> R + Sync) -> Vec<R> {
    let parts = workers.map_ranges(items.len(), min, |r| r.map(|i| f(i, &items[i])).collect::<Vec<R>>());
    let mut out = Vec::with_capacity(items.len());
    parts.into_iter().for_each(|p| out.extend(p));
    out
}

/// `f` over each item, in parallel, each with its own `&mut`.
pub fn par_for_each_mut<T: Send>(workers: &Workers, items: &mut [T], min: usize, f: impl Fn(usize, &mut T) + Sync) {
    let n = items.len();
    let ranges = engine_ecs::par::even(n, workers.chunks(n, min));
    let parts: Vec<(usize, &mut [T])> =
        ranges.iter().map(|r| r.start).zip(engine_ecs::par::carve(items, ranges.iter().map(|r| r.len()))).collect();
    workers.map_each(parts, |_, (start, part)| {
        for (i, x) in part.iter_mut().enumerate() {
            f(start + i, x);
        }
    });
}

/// Items mapped a chunk of `chunk` at a time and folded in the items'
/// order: chunks fixed by the input, not the threads, so a float sum is the
/// same at any thread count. `None` for no items.
pub fn reduce<T: Sync, A: Send>(
    workers: &Workers,
    items: &[T],
    chunk: usize,
    map: impl Fn(&[T]) -> A + Sync,
    fold: impl Fn(A, A) -> A,
) -> Option<A> {
    let chunks: Vec<&[T]> = items.chunks(chunk.max(1)).collect();
    let parts = workers.map_each(chunks, |_, c| map(c));
    parts.into_iter().reduce(fold)
}

/// Not in any color: neither end moves, so solving it changes nothing.
pub const UNSOLVED: u32 = u32::MAX;
/// Past the colors: solved first, in order, on one thread (Box2D's overflow).
pub const OVERFLOW: u32 = u32::MAX - 1;
/// Colors a state's edges can take before they overflow.
pub const COLORS: usize = 64;

/// Each edge's color, so that no two edges of a color share a state that
/// moves; then how many of each, and how many overflowed.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Coloring {
    pub of: Vec<u32>,
    pub count: Vec<usize>,
    pub overflow: usize,
}

impl Coloring {
    /// Greedily, in edge order: the lowest color neither moving end has
    /// yet. With `fixed_apart`, not color 0 for an edge with an end that
    /// doesn't move, so the first color's edges all join moving states
    /// (Box2D v3's `b2AddContactToGraph`, with its reason: a static body's
    /// many edges would otherwise crowd color 0). `masks` is scratch, one
    /// word a state, kept by the caller to keep its allocation.
    pub fn greedy(&mut self, n: usize, ends: impl Fn(usize) -> (u32, u32), moves: &[bool], fixed_apart: bool, masks: &mut Vec<u64>) {
        masks.clear();
        masks.resize(moves.len(), 0);
        self.of.clear();
        self.count.clear();
        self.overflow = 0;
        for i in 0..n {
            let (a, b) = ends(i);
            let (a, b) = (a as usize, b as usize);
            if !moves[a] && !moves[b] {
                self.of.push(UNSOLVED);
                continue;
            }
            let at = |e: usize, masks: &[u64]| if moves[e] { masks[e] } else { 0 };
            let mut free = !(at(a, masks) | at(b, masks));
            if fixed_apart && (!moves[a] || !moves[b]) {
                free &= !1;
            }
            let k = free.trailing_zeros() as usize;
            if k >= COLORS {
                self.of.push(OVERFLOW);
                self.overflow += 1;
                continue;
            }
            for e in [a, b] {
                if moves[e] {
                    masks[e] |= 1 << k;
                }
            }
            if self.count.len() <= k {
                self.count.resize(k + 1, 0);
            }
            self.count[k] += 1;
            self.of.push(k as u32);
        }
    }

    /// Where each edge goes when a color's edges are packed `width` to an
    /// item, in edge order: `(item, lane)`, `None` if unsolved; and the
    /// items' layout: the overflow's first, one edge an item, then each
    /// color's.
    pub fn pack(&self, width: usize, place: &mut Vec<Option<(u32, u32)>>) -> Colored {
        let colors: Vec<usize> = self.count.iter().map(|c| c.div_ceil(width)).collect();
        let mut first = Vec::with_capacity(colors.len());
        let mut at = self.overflow;
        for c in &colors {
            first.push(at);
            at += c;
        }
        let mut filled = vec![0usize; colors.len()];
        let mut overflowed = 0;
        place.clear();
        place.extend(self.of.iter().map(|&k| match k {
            UNSOLVED => None,
            OVERFLOW => {
                overflowed += 1;
                Some((overflowed as u32 - 1, 0))
            }
            k => {
                let j = filled[k as usize];
                filled[k as usize] += 1;
                Some(((first[k as usize] + j / width) as u32, (j % width) as u32))
            }
        }));
        Colored { overflow: self.overflow, colors }
    }
}

/// Items in colors: the overflow's first, then each color's, consecutive.
/// No two items of a color touch one moving state, so a color's items can
/// run in any order on any thread with the same result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Colored {
    pub overflow: usize,
    /// Items in each color, in order.
    pub colors: Vec<usize>,
}

/// A stage of [`Colored::passes`]: every item, a color at a time (the
/// overflow first), or each of `n` states by range.
#[derive(Clone, Copy, Debug)]
pub enum Stage<K> {
    Items(K),
    Each(K, usize),
}

/// A unit of a stage that one thread takes at a time.
enum Part<'a, I> {
    Items(&'a mut [I]),
    Range(Range<usize>),
}

/// A block: one past the last stage that took it, and its part. A thread
/// takes it for stage `t` by raising the mark to `t + 1`, which only one
/// thread can do while it's lower (Box2D's `syncIndex`). The lock is only
/// how the taker has the part; nobody else holds it then. Padded to its own
/// lines, so taking one doesn't evict a neighbour another thread takes.
#[repr(align(128))]
struct Block<'a, I> {
    mark: AtomicUsize,
    part: Mutex<Part<'a, I>>,
}

#[repr(align(128))]
#[derive(Default)]
struct Count(AtomicUsize);

/// Set when a thread panics, so the others stop waiting for its blocks.
struct Failing<'a>(&'a AtomicBool);

impl Drop for Failing<'_> {
    fn drop(&mut self) {
        if std::thread::panicking() {
            self.0.store(true, Ordering::Relaxed);
        }
    }
}

/// Where thread `w` of `threads` starts in a stage of `n` blocks: its
/// share's first, so each thread takes the same blocks stage after stage
/// and finds them in its cache (Box2D's `GetWorkerStartIndex`).
fn first_block(w: usize, n: usize, threads: usize) -> usize {
    if n <= threads {
        return w % n;
    }
    let (per, rest) = (n / threads, n % threads);
    per * w + rest.min(w)
}

/// `0..n` in blocks of at least `least`, at most four a thread (Box2D's
/// `b2SolverStage` sizes).
fn blocks_of(n: usize, least: usize, threads: usize) -> impl Iterator<Item = Range<usize>> {
    let count = if n > least * 4 * threads { 4 * threads } else { n.div_ceil(least) };
    (0..count).map(move |k| n * k / count..n * (k + 1) / count)
}

/// Items a block holds at least, and states.
const ITEMS_A_BLOCK: usize = 4;
const STATES_A_BLOCK: usize = 32;

impl Colored {
    pub fn items(&self) -> usize {
        self.overflow + self.colors.iter().sum::<usize>()
    }

    /// Runs `program` over `items` (laid out as `self` says) and the states
    /// they share, across `workers`' threads, within one run of the
    /// executor: each stage's blocks all done before the next's start, each
    /// `Items` stage a color at a time. `item` runs once per item a stage,
    /// `each` once per range of states. With one thread, or a single block
    /// a stage, everything runs in order on the caller's: the same result,
    /// since a color's items don't share a moving state. States are shared
    /// as `states`, which kernels write only where their item's edges are
    /// (relaxed atomics, in physics: `lanes::Atom`).
    ///
    /// Box2D v3's staged solve (`b2SolverStage`, `solver.c`), generic: any
    /// thread takes any block of a stage, a thread that comes late skips
    /// stages already done, and there is no main thread. As
    /// physics2d's `lanes::run_across`, from which it was lifted.
    pub fn passes<I: Send, S: Sync + ?Sized, K: Copy + Sync>(
        &self,
        workers: &Workers,
        items: &mut [I],
        states: &S,
        program: &[Stage<K>],
        item: impl Fn(K, &mut I, &S) + Sync,
        each: impl Fn(K, Range<usize>, &S) + Sync,
    ) {
        self.passes_blocks(workers, items, states, program, |k, xs: &mut [I], s| xs.iter_mut().for_each(|x| item(k, x, s)), each);
    }

    /// `passes`, its kernel handed a block of a color's items at a time
    /// rather than one: the stage's work decided once a block, and the
    /// loop over the items the kernel's own, as `run_across`'s stages
    /// loop over their batches. A block's items are consecutive, of one
    /// color; with one thread, a stage's are all of them, in order.
    pub fn passes_blocks<I: Send, S: Sync + ?Sized, K: Copy + Sync>(
        &self,
        workers: &Workers,
        items: &mut [I],
        states: &S,
        program: &[Stage<K>],
        block: impl Fn(K, &mut [I], &S) + Sync,
        each: impl Fn(K, Range<usize>, &S) + Sync,
    ) {
        assert_eq!(items.len(), self.items(), "items as the coloring laid them out");
        let threads = workers.threads();
        if threads == 1 {
            for stage in program {
                match *stage {
                    Stage::Items(k) => block(k, items, states),
                    Stage::Each(k, n) => each(k, 0..n, states),
                }
            }
            return;
        }
        // Blocks: the overflow's (one, in order), each color's, then each
        // distinct `Each` size's ranges.
        let mut blocks: Vec<Block<'_, I>> = Vec::new();
        let new_block = |part| Block { mark: AtomicUsize::new(0), part: Mutex::new(part) };
        let mut rest = items;
        let mut groups: Vec<Range<usize>> = Vec::new();
        if self.overflow > 0 {
            let (head, tail) = rest.split_at_mut(self.overflow);
            rest = tail;
            blocks.push(new_block(Part::Items(head)));
            groups.push(0..1);
        }
        for &n in &self.colors {
            let (mut color, tail) = std::mem::take(&mut rest).split_at_mut(n);
            rest = tail;
            let from = blocks.len();
            for r in blocks_of(n, ITEMS_A_BLOCK, threads) {
                let (head, tail) = color.split_at_mut(r.len());
                color = tail;
                blocks.push(new_block(Part::Items(head)));
            }
            if blocks.len() > from {
                groups.push(from..blocks.len());
            }
        }
        let mut sized: Vec<(usize, Range<usize>)> = Vec::new();
        for stage in program {
            if let Stage::Each(_, n) = *stage
                && !sized.iter().any(|(m, _)| *m == n)
            {
                let from = blocks.len();
                blocks.extend(blocks_of(n, STATES_A_BLOCK, threads).map(|r| new_block(Part::Range(r))));
                sized.push((n, from..blocks.len()));
            }
        }
        let mut stages: Vec<(K, Range<usize>)> = Vec::new();
        for stage in program {
            match *stage {
                Stage::Items(k) => stages.extend(groups.iter().map(|g| (k, g.clone()))),
                Stage::Each(k, n) => stages.push((k, sized.iter().find(|(m, _)| *m == n).expect("sized above").1.clone())),
            }
        }
        let exec = |k: K, part: &mut Part<'_, I>| match part {
            Part::Items(xs) => block(k, xs, states),
            Part::Range(r) => each(k, r.clone(), states),
        };
        let done: Vec<Count> = stages.iter().map(|_| Count::default()).collect();
        let failed = AtomicBool::new(false);
        let run = |w: usize| {
            let _failing = Failing(&failed);
            for (t, (k, on)) in stages.iter().enumerate() {
                let (n, count) = (on.len(), &done[t].0);
                if n == 0 || count.load(Ordering::Acquire) == n {
                    continue;
                }
                let take = |b: usize| {
                    let b = &blocks[on.start + b];
                    if b.mark.fetch_max(t + 1, Ordering::AcqRel) > t {
                        return false;
                    }
                    exec(*k, &mut b.part.try_lock().expect("a block's taker alone has it"));
                    true
                };
                let (start, mut ran) = (first_block(w, n, threads), 0);
                let mut b = start;
                while take(b) {
                    ran += 1;
                    b = if b + 1 == n { 0 } else { b + 1 };
                }
                let mut b = start;
                loop {
                    b = if b == 0 { n - 1 } else { b - 1 };
                    if !take(b) {
                        break;
                    }
                    ran += 1;
                }
                if ran > 0 {
                    count.fetch_add(ran, Ordering::Release);
                }
                // Spinning: a stage is microseconds, a wake-up tens of them.
                let mut spins = 0u32;
                while count.load(Ordering::Acquire) < n {
                    if failed.load(Ordering::Relaxed) {
                        return;
                    }
                    std::hint::spin_loop();
                    spins = spins.wrapping_add(1);
                    if spins.is_multiple_of(1024) {
                        std::thread::yield_now();
                    }
                }
            }
        };
        workers.run(threads, run);
    }

    /// `passes`, a kernel per edge with its two states: `ends` says which.
    /// The shape the sketch began from (`|edge, a, b|`).
    pub fn edges<I: Send, A: Sync, K: Copy + Sync>(
        &self,
        workers: &Workers,
        items: &mut [I],
        states: &[A],
        program: &[Stage<K>],
        ends: impl Fn(&I) -> (u32, u32) + Sync,
        edge: impl Fn(K, &mut I, &A, &A) + Sync,
        each: impl Fn(K, Range<usize>, &[A]) + Sync,
    ) {
        self.passes(
            workers,
            items,
            states,
            program,
            |k, x, s: &[A]| {
                let (a, b) = ends(x);
                edge(k, x, &s[a as usize], &s[b as usize])
            },
            each,
        );
    }
}
