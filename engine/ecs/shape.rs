//! Parallel shapes: the only forms parallel work over a system's data
//! takes, each a parameter, so the plan knows which systems fan out and
//! how. A system hands its shape the frame's data and its kernels; what
//! runs them is the scheduler's (docs/architecture/flows.md, "Parallel
//! shapes"):
//!
//! - [`ParMap`]: a function of each item, in the items' order;
//! - [`Reduce`]: fixed chunks mapped and folded in the items' order;
//! - [`Passes`]: a program of stages over items in colors
//!   ([`Colored`]) and the states they share, Box2D v3's staged solve made
//!   generic.
//!
//! Each gives the same result on any number of threads, and on one. This
//! module fixes what that result is. Each runs across the world's executor
//! where it has more than one thread, as a task graph (`dispatch`;
//! docs/architecture/threads.md): `Passes` a stage a color or pass,
//! `ParMap` and `Reduce` one stage of blocks.

use std::marker::PhantomData;
use std::ops::Range;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use crate::dispatch::{Plan, block_range, blocks_of, dispatch, first_block};
use crate::par::{Executor, Split, one_stage};
use crate::query::{Declare, FrameCx, Param, ParamDecl};
use crate::world::World;

/// Which shape a system declared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeKind {
    Map,
    Reduce,
    Passes,
}

/// Panics unless `decl` is the declaration of a shape of `kind`, as every
/// other parameter's fetch checks its own: a shape is fetched only against
/// what its node declared, which is what the scheduler reads.
fn declared(decl: &ParamDecl, kind: ShapeKind) {
    assert!(matches!(decl, ParamDecl::Shape(k) if *k == kind), "a {kind:?} shape fetched against {decl:?}");
}

/// Maps a function over items. The lifetime is the frame's.
#[derive(Clone)]
pub struct ParMap<'w> {
    split: Split,
    _world: PhantomData<&'w World>,
}

impl Param for ParMap<'static> {
    type Item<'w> = ParMap<'w>;

    fn declare(_: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Shape(ShapeKind::Map)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> ParMap<'w> {
        declared(decl, ShapeKind::Map);
        ParMap { split: Split::of(cx.world), _world: PhantomData }
    }
}

impl ParMap<'_> {
    /// How many threads the map hands its items out across.
    pub fn threads(&self) -> usize {
        self.split.threads()
    }

    /// The world's threads, for a query's walk across them
    /// (`Query::par_for_each`).
    pub(crate) fn split(&self) -> &Split {
        &self.split
    }

    /// How many blocks of at least `min` items `n` items go in across the
    /// threads: one on one thread.
    fn blocks(&self, n: usize, min: usize) -> usize {
        self.split.executor().map_or(1, |e| blocks_of(n, min.max(1), e.threads()))
    }

    /// Sets `out` to `f(i, item)` of each item, in the items' order,
    /// keeping `out`'s allocation: in order on one thread; across the
    /// world's threads in blocks of at least `min` consecutive items, at
    /// most four a thread, each block's results into a list of its own made
    /// here with room for them (memory a worker allocates is its thread's:
    /// docs/lore/memory-a-task-allocates-is-its-threads.md), then moved
    /// into `out` in block order. Every call has returned when this does,
    /// and a call's panic is raised again here.
    pub fn map_into<T: Sync, R: Send>(&self, items: &[T], min: usize, out: &mut Vec<R>, f: impl Fn(usize, &T) -> R + Sync) {
        out.clear();
        let (n, count) = (items.len(), self.blocks(items.len(), min));
        let exec = match self.split.executor() {
            Some(exec) if count > 1 => exec,
            _ => return out.extend(items.iter().enumerate().map(|(i, x)| f(i, x))),
        };
        let blocks: Vec<Mutex<Vec<R>>> = (0..count).map(|k| Mutex::new(Vec::with_capacity(block_range(n, k, count).len()))).collect();
        one_stage(exec, count, &|b| {
            let mut made = blocks[b].try_lock().expect("a block's taker alone has it");
            made.extend(block_range(n, b, count).map(|i| f(i, &items[i])));
        });
        out.reserve(n);
        for b in blocks {
            out.append(&mut b.into_inner().expect("a finished block"));
        }
    }

    /// `f(i, &mut item)` for each item, each item's call independent of the
    /// others': in order on one thread; across the world's threads in
    /// blocks of at least `min` consecutive items, at most four a thread,
    /// each block run once, by whichever thread takes it (`dispatch`).
    /// Every call has returned when this does, and a call's panic is
    /// raised again here.
    ///
    /// An item may be a part of the caller's own making, slices of several
    /// arrays cut alike (physics's write-back: each part its run of
    /// contacts, their points and bodies), or a chunk of work with the list
    /// it fills, made by the caller (physics2d's broadphase and
    /// narrowphase), so that one run writes them all.
    pub fn for_each_mut<T: Send>(&self, items: &mut [T], min: usize, f: impl Fn(usize, &mut T) + Sync) {
        let count = self.blocks(items.len(), min);
        match self.split.executor() {
            Some(exec) if count > 1 => map_across(exec, items, count, &f),
            _ => items.iter_mut().enumerate().for_each(|(i, x)| f(i, x)),
        }
    }
}

/// `ParMap::for_each_mut` across `exec`'s threads: `items` cut into `count`
/// blocks, one stage of a plan, each block behind a lock only its taker
/// takes, as `Passes`' blocks are.
fn map_across<T: Send>(exec: &dyn Executor, items: &mut [T], count: usize, f: &(impl Fn(usize, &mut T) + Sync)) {
    let n = items.len();
    let mut blocks = Vec::with_capacity(count);
    let mut rest = items;
    for k in 0..count {
        let r = block_range(n, k, count);
        let (head, tail) = std::mem::take(&mut rest).split_at_mut(r.len());
        rest = tail;
        blocks.push((r.start, Mutex::new(head)));
    }
    one_stage(exec, count, &|b| {
        let (at, items) = &blocks[b];
        let mut items = items.try_lock().expect("a block's taker alone has it");
        items.iter_mut().enumerate().for_each(|(i, x)| f(at + i, x));
    });
}

/// Folds items in an order fixed by the input.
#[derive(Clone)]
pub struct Reduce<'w> {
    split: Split,
    _world: PhantomData<&'w World>,
}

impl Param for Reduce<'static> {
    type Item<'w> = Reduce<'w>;

    fn declare(_: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Shape(ShapeKind::Reduce)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Reduce<'w> {
        declared(decl, ShapeKind::Reduce);
        Reduce { split: Split::of(cx.world), _world: PhantomData }
    }
}

impl Reduce<'_> {
    /// How many threads the chunks are mapped across.
    pub fn threads(&self) -> usize {
        self.split.threads()
    }

    /// `map` of each chunk of `chunk` items (the last may be shorter),
    /// folded left to right in the items' order: chunks fixed by the input,
    /// not the threads, so a float sum is the same on any number. `None`
    /// for no items. Across the world's threads the chunks are mapped in
    /// blocks of consecutive chunks, at most four a thread, and folded on
    /// this thread after, in order. Every `map` has returned when this
    /// does, and a panic in one is raised again here.
    pub fn reduce<T: Sync, A: Send>(
        &self,
        items: &[T],
        chunk: usize,
        map: impl Fn(&[T]) -> A + Sync,
        fold: impl FnMut(A, A) -> A,
    ) -> Option<A> {
        let chunk = chunk.max(1);
        let m = items.len().div_ceil(chunk);
        let count = self.split.executor().map_or(1, |e| blocks_of(m, 1, e.threads()));
        let exec = match self.split.executor() {
            Some(exec) if count > 1 => exec,
            _ => return items.chunks(chunk).map(map).reduce(fold),
        };
        let blocks: Vec<Mutex<Vec<A>>> = (0..count).map(|k| Mutex::new(Vec::with_capacity(block_range(m, k, count).len()))).collect();
        one_stage(exec, count, &|b| {
            let mut mapped = blocks[b].try_lock().expect("a block's taker alone has it");
            mapped.extend(block_range(m, b, count).map(|c| map(&items[c * chunk..((c + 1) * chunk).min(items.len())])));
        });
        blocks.into_iter().flat_map(|b| b.into_inner().expect("a finished block")).reduce(fold)
    }
}

/// Runs a program of stages over items in colors.
#[derive(Clone)]
pub struct Passes<'w> {
    /// Hand kernels the shared form of the states even on one thread
    /// (`World::set_shapes_shared`).
    shared: bool,
    /// The world's executor, where it has more than one thread.
    across: Option<Arc<dyn Executor>>,
    _world: PhantomData<&'w World>,
}

impl Param for Passes<'static> {
    type Item<'w> = Passes<'w>;

    fn declare(_: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Shape(ShapeKind::Passes)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, decl: &'w ParamDecl) -> Passes<'w> {
        declared(decl, ShapeKind::Passes);
        let across = cx.world.executor().filter(|e| e.threads() > 1);
        Passes { shared: cx.world.shapes_shared.load(Ordering::Relaxed), across, _world: PhantomData }
    }
}

impl World {
    /// Whether `Passes` hands kernels their states as threads share them
    /// (`States::Shared`) even on one thread, where it hands them plain
    /// (`States::Plain`) by default: so a test can hold a kernel's shared
    /// path to its plain one before the scheduler runs it across threads,
    /// and a benchmark can measure what sharing costs.
    pub fn set_shapes_shared(&self, on: bool) {
        self.shapes_shared.store(on, Ordering::Relaxed);
    }
}

/// A state type a primitive can hand kernels as plain memory, on one
/// thread, or in a form threads can share, on several: `Shared`, made from
/// a value, read and written through `&`. Relaxed atomics of its fields'
/// bits, normally, which on x86 are plain loads and stores.
pub trait Shareable: Copy + Send + Sync {
    type Shared: Sync;
    fn share(&self) -> Self::Shared;
    fn load(shared: &Self::Shared) -> Self;
    fn store(shared: &Self::Shared, value: Self);
}

impl Shareable for f32 {
    type Shared = AtomicU32;

    fn share(&self) -> AtomicU32 {
        AtomicU32::new(self.to_bits())
    }

    fn load(shared: &AtomicU32) -> f32 {
        f32::from_bits(shared.load(Ordering::Relaxed))
    }

    fn store(shared: &AtomicU32, value: f32) {
        shared.store(value.to_bits(), Ordering::Relaxed)
    }
}

impl Shareable for u32 {
    type Shared = AtomicU32;

    fn share(&self) -> AtomicU32 {
        AtomicU32::new(*self)
    }

    fn load(shared: &AtomicU32) -> u32 {
        shared.load(Ordering::Relaxed)
    }

    fn store(shared: &AtomicU32, value: u32) {
        shared.store(value, Ordering::Relaxed)
    }
}

/// The states as a kernel of [`Passes::run`] gets them: the slice itself on
/// one thread, or its shared form on several. A kernel written over the
/// view's `get` and `set` serves both, at a match per access; one generic
/// over a view of the mod's own matches once a call and runs a copy of
/// itself for each (docs/architecture/flows.md, "On one thread").
pub enum States<'a, T: Shareable> {
    Plain(&'a mut [T]),
    Shared(&'a [T::Shared]),
}

impl<T: Shareable> States<'_, T> {
    /// The same states for a shorter while: to hand them on and keep them.
    #[inline(always)]
    pub fn reborrow(&mut self) -> States<'_, T> {
        match self {
            States::Plain(s) => States::Plain(s),
            States::Shared(s) => States::Shared(s),
        }
    }

    #[inline(always)]
    pub fn get(&self, i: usize) -> T {
        match self {
            States::Plain(s) => s[i],
            States::Shared(s) => T::load(&s[i]),
        }
    }

    #[inline(always)]
    pub fn set(&mut self, i: usize, value: T) {
        match self {
            States::Plain(s) => s[i] = value,
            States::Shared(s) => T::store(&s[i], value),
        }
    }

    pub fn len(&self) -> usize {
        match self {
            States::Plain(s) => s.len(),
            States::Shared(s) => s.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// A stage of [`Passes::run`]: every item, a color at a time (the overflow
/// first); each of `n` states by range; or every item at once.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stage<K> {
    Items(K),
    Each(K, usize),
    /// Every item in one stage, colors and overflow alike: for a kernel
    /// that writes its items alone and only reads the states, such as one
    /// that fills them (physics's batches, from the contacts each item's
    /// lanes hold). Items of different colors share states, so a kernel
    /// here that wrote one would race.
    All(K),
}

impl Passes<'_> {
    /// The next `run` on the system's thread, plain, when `serial`: for a
    /// program whose kernels would give another result across threads this
    /// time, which only their owner can tell (get-znt.39: physics2d's still
    /// bodies carrying a negative zero, which batches of one color write
    /// back racing).
    pub fn serial(mut self, serial: bool) -> Self {
        if serial {
            self.across = None;
        }
        self
    }

    /// How many threads `run` hands its stages out across.
    pub fn threads(&self) -> usize {
        self.across.as_ref().map_or(1, |e| e.threads())
    }

    /// Runs `program` over `items`, laid out as `layout` says, and the
    /// states they share: each stage done before the next starts, an
    /// `Items` stage the overflow's items and then each color's, an `All`
    /// stage every item at once. `block` gets consecutive items, from the
    /// index it's given, of one color in an `Items` stage, of any in an
    /// `All` stage: all of them on one thread, some on several, so it must
    /// treat them independently. `each` gets a range of `0..n`. Kernels get
    /// the states as [`States`]: the slice itself on one thread (`Plain`),
    /// and on several the shared form, made from it before the first stage
    /// and written back after the last (`Shared`), which kernels write only
    /// where their items' edges are, and never in an `All` stage. A
    /// color's items share no moving state, so any order of its blocks
    /// gives the same states.
    ///
    /// Across threads a stage is the overflow's one block, a color's
    /// blocks, every one of those blocks (`All`), or an `Each`'s ranges,
    /// sized as Box2D sizes them, each block run once, after the stage
    /// before (`dispatch`). Every kernel call has returned when `run` does,
    /// so no mod code is on another thread once the system has; a kernel's
    /// panic is raised again here.
    ///
    /// Box2D v3's staged solve (`b2SolverStage`, `solver.c`), generic.
    pub fn run<I: Send, T: Shareable, K: Copy + Sync>(
        &self,
        layout: &Colored,
        items: &mut [I],
        states: &mut [T],
        program: &[Stage<K>],
        block: impl Fn(K, usize, &mut [I], States<'_, T>) + Sync,
        each: impl Fn(K, Range<usize>, States<'_, T>) + Sync,
    ) {
        assert_eq!(items.len(), layout.items(), "items as the coloring laid them out");
        if let Some(exec) = &self.across {
            return across(&**exec, layout, items, states, program, &block, &each);
        }
        if self.shared {
            let shared: Vec<T::Shared> = states.iter().map(T::share).collect();
            stages(layout, items, program, &block, &each, States::Shared(&shared));
            for (s, x) in states.iter_mut().zip(&shared) {
                *s = T::load(x);
            }
        } else {
            stages(layout, items, program, &block, &each, States::Plain(states));
        }
    }
}

/// `Passes::run`'s stages on this thread.
fn stages<I, T: Shareable, K: Copy>(
    layout: &Colored,
    items: &mut [I],
    program: &[Stage<K>],
    block: &impl Fn(K, usize, &mut [I], States<'_, T>),
    each: &impl Fn(K, Range<usize>, States<'_, T>),
    mut states: States<'_, T>,
) {
    for stage in program {
        match *stage {
            Stage::Items(k) => {
                let (mut rest, mut at) = (&mut items[..], 0);
                for n in std::iter::once(layout.overflow).chain(layout.colors.iter().copied()) {
                    let (color, tail) = std::mem::take(&mut rest).split_at_mut(n);
                    rest = tail;
                    // A color can be empty: the greedy coloring keeps color
                    // 0 from edges at a fixed state.
                    if !color.is_empty() {
                        block(k, at, color, states.reborrow());
                    }
                    at += n;
                }
            }
            Stage::All(k) if !items.is_empty() => block(k, 0, items, states.reborrow()),
            Stage::Each(k, n) if n > 0 => each(k, 0..n, states.reborrow()),
            Stage::All(_) | Stage::Each(..) => {}
        }
    }
}

/// What a stage of the plan runs a block of.
#[derive(Clone, Copy)]
enum Work<K> {
    /// The kernel's argument, and the stage's first block of items.
    Items(K, usize),
    /// The kernel's argument: every block of items, in `across`'s `all`
    /// order.
    All(K),
    /// The kernel's argument, the states' count, and the stage's ranges.
    Each(K, usize, usize),
}

/// The fewest items a block of a color takes, and states a range: Box2D's
/// minimum block sizes (`b2SolverStage`), as `run_across` had them.
const ITEMS_A_BLOCK: usize = 4;
const STATES_A_BLOCK: usize = 32;

/// `Passes::run` across `exec`'s threads: the overflow one block (its
/// items share states, so they go in order on one thread, as Box2D's
/// overflow does), each color's items in blocks, each `Each` in ranges;
/// a stage a color of an `Items` stage, and a stage an `Each`. Stages over
/// the same blocks share their marks, so a thread takes the same blocks
/// pass after pass and finds them in its cache.
fn across<I: Send, T: Shareable, K: Copy + Sync>(
    exec: &dyn Executor,
    layout: &Colored,
    items: &mut [I],
    states: &mut [T],
    program: &[Stage<K>],
    block: &(impl Fn(K, usize, &mut [I], States<'_, T>) + Sync),
    each: &(impl Fn(K, Range<usize>, States<'_, T>) + Sync),
) {
    let threads = exec.threads();
    let mut plan = Plan::default();
    // Each block's first item, and its items behind a lock only its taker
    // takes (`try_lock`): how a block hands its kernel `&mut` items in safe
    // Rust.
    let mut blocks: Vec<(usize, Mutex<&mut [I]>)> = Vec::new();
    // Each non-empty group's first block and its count: the overflow's,
    // then each color's.
    let mut groups = Vec::new();
    let (mut rest, mut at) = (items, 0);
    for (i, n) in std::iter::once(layout.overflow).chain(layout.colors.iter().copied()).enumerate() {
        let (mut group, tail) = std::mem::take(&mut rest).split_at_mut(n);
        rest = tail;
        if n == 0 {
            continue;
        }
        let (first, count) = (blocks.len(), if i == 0 { 1 } else { blocks_of(n, ITEMS_A_BLOCK, threads) });
        for k in 0..count {
            let len = block_range(n, k, count).len();
            let (head, tail) = std::mem::take(&mut group).split_at_mut(len);
            group = tail;
            blocks.push((at, Mutex::new(head)));
            at += len;
        }
        groups.push((first, count));
    }
    let item_marks = plan.marks(blocks.len());
    // An `All` stage's blocks, in the order its claims take them.
    let all = all_order(&groups, threads);
    // Each `Each`'s marks, by its count: stages over the same states are
    // over the same ranges.
    let mut each_marks: Vec<(usize, usize)> = Vec::new();
    let mut work = Vec::new();
    plan.chain();
    for stage in program {
        match *stage {
            Stage::Items(k) => {
                for &(first, count) in &groups {
                    plan.stage(count, item_marks + first);
                    work.push(Work::Items(k, first));
                }
            }
            Stage::Each(k, n) if n > 0 => {
                let count = blocks_of(n, STATES_A_BLOCK, threads);
                let mark = match each_marks.iter().find(|(m, _)| *m == n) {
                    Some(&(_, mark)) => mark,
                    None => {
                        let mark = plan.marks(count);
                        each_marks.push((n, mark));
                        mark
                    }
                };
                plan.stage(count, mark);
                work.push(Work::Each(k, n, count));
            }
            // The groups' blocks, one stage: claims over them are claims
            // over every item once, and they share the groups' marks, a
            // stage like any over the same blocks.
            Stage::All(k) if !blocks.is_empty() => {
                plan.stage(blocks.len(), item_marks);
                work.push(Work::All(k));
            }
            Stage::All(_) | Stage::Each(..) => {}
        }
    }
    let shared: Vec<T::Shared> = states.iter().map(T::share).collect();
    // Never contended: the protocol gives a block to one thread a stage, so
    // a block found taken is a dispatch bug, and fails.
    let run = |k: K, b: usize| {
        let (at, items) = &blocks[b];
        block(k, *at, &mut items.try_lock().expect("a block's taker alone has it"), States::Shared(&shared));
    };
    dispatch(exec, &plan, &|t, b| match work[t] {
        Work::Items(k, first) => run(k, first + b),
        Work::All(k) => run(k, all[b]),
        Work::Each(k, n, count) => each(k, block_range(n, b, count), States::Shared(&shared)),
    });
    for (s, x) in states.iter_mut().zip(&shared) {
        *s = T::load(x);
    }
}

/// The blocks of `groups` (each its first block and count) in the order an
/// `All` stage hands them out: worker after worker, each its share of
/// every group, the blocks its claims start from in that group's stages
/// (`first_block`). So a worker that fills its items finds them in its
/// cache when it solves them, if the claims come out as planned. Measured
/// on physics2d's step_bench at 8 threads, against item order: the passes
/// 826-833 µs against 852-853 on the settled pile of 10 000, 514-516
/// against 529-532 on the pyramid of 5050, and `finish` 154 against 165 on
/// the pile (2026-10-03).
fn all_order(groups: &[(usize, usize)], threads: usize) -> Vec<usize> {
    let total = groups.last().map_or(0, |&(first, count)| first + count);
    let mut order = Vec::with_capacity(total);
    for w in 0..threads {
        for &(first, count) in groups {
            let share = if count <= threads {
                if w < count { w..w + 1 } else { 0..0 }
            } else {
                first_block(w, count, threads)..if w + 1 == threads { count } else { first_block(w + 1, count, threads) }
            };
            order.extend(share.map(|k| first + k));
        }
    }
    debug_assert_eq!(order.len(), total);
    order
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

    /// `pack`'s places the other way round: each item's edges, `width` to
    /// an item, lane by lane (`seats[item * width + lane]`), `EMPTY` where a
    /// lane has none; and the same layout. What a stage that fills its
    /// items from their edges reads (`Stage::All`), where `pack`'s places
    /// would have it look every edge up.
    pub fn seat(&self, width: usize, seats: &mut Vec<u32>) -> Colored {
        let colors: Vec<usize> = self.count.iter().map(|c| c.div_ceil(width)).collect();
        seats.clear();
        seats.resize((self.overflow + colors.iter().sum::<usize>()) * width, EMPTY);
        for (i, to) in self.seats(width).enumerate() {
            if let Some(to) = to {
                seats[to] = i as u32;
            }
        }
        Colored { overflow: self.overflow, colors }
    }

    /// Each edge's seat, `item * width + lane`, in edge order, `None` if
    /// unsolved: `seat`'s, for a caller that goes through the edges in
    /// their order rather than the items in theirs.
    pub fn seats(&self, width: usize) -> impl Iterator<Item = Option<usize>> + '_ {
        let mut next = [0; COLORS];
        let mut at = self.overflow * width;
        for (next, c) in next.iter_mut().zip(&self.count) {
            *next = at;
            at += c.div_ceil(width) * width;
        }
        let mut overflowed = 0;
        self.of.iter().map(move |&k| match k {
            UNSOLVED => None,
            OVERFLOW => {
                overflowed += 1;
                Some((overflowed - 1) * width)
            }
            k => {
                next[k as usize] += 1;
                Some(next[k as usize] - 1)
            }
        })
    }
}

/// A lane `Coloring::seat` gives no edge.
pub const EMPTY: u32 = u32::MAX;

/// Items in colors: the overflow's first, then each color's, consecutive.
/// No two items of a color touch one moving state, so a color's items can
/// run in any order on any thread with the same result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Colored {
    pub overflow: usize,
    /// Items in each color, in order.
    pub colors: Vec<usize>,
}

impl Colored {
    pub fn items(&self) -> usize {
        self.overflow + self.colors.iter().sum::<usize>()
    }
}
