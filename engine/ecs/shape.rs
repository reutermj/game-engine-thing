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
//! Each gives the same result on any number of threads, and on one, which
//! is how they run until the scheduler runs them across threads
//! (get-znt.34). This module fixes what that result is.

use std::marker::PhantomData;
use std::ops::Range;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::query::{Declare, FrameCx, Param, ParamDecl};
use crate::world::World;

/// Which shape a system declared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeKind {
    Map,
    Reduce,
    Passes,
}

macro_rules! shape_param {
    ($name:ident, $kind:ident) => {
        impl Param for $name<'static> {
            type Item<'w> = $name<'w>;

            fn declare(_: &mut Declare<'_>) -> ParamDecl {
                ParamDecl::Shape(ShapeKind::$kind)
            }

            fn fetch<'w>(_: &FrameCx<'w>, _: &'w ParamDecl) -> $name<'w> {
                $name(PhantomData)
            }
        }
    };
}

/// Maps a function over items. The lifetime is the frame's: the scheduler
/// that runs it across threads will hand its tasks out through it.
pub struct ParMap<'w>(PhantomData<&'w World>);
shape_param!(ParMap, Map);

impl ParMap<'_> {
    /// Sets `out` to `f(i, item)` of each item, in the items' order,
    /// keeping `out`'s allocation. `min` is the fewest items a task is
    /// worth, for the scheduler's split.
    pub fn map_into<T: Sync, R: Send>(&self, items: &[T], min: usize, out: &mut Vec<R>, f: impl Fn(usize, &T) -> R + Sync) {
        let _ = min;
        out.clear();
        out.extend(items.iter().enumerate().map(|(i, x)| f(i, x)));
    }

    /// `f(i, &mut item)` for each item, each item's call independent of the
    /// others'.
    pub fn for_each_mut<T: Send>(&self, items: &mut [T], min: usize, f: impl Fn(usize, &mut T) + Sync) {
        let _ = min;
        items.iter_mut().enumerate().for_each(|(i, x)| f(i, x));
    }
}

/// Folds items in an order fixed by the input.
pub struct Reduce<'w>(PhantomData<&'w World>);
shape_param!(Reduce, Reduce);

impl Reduce<'_> {
    /// `map` of each chunk of `chunk` items (the last may be shorter),
    /// folded left to right in the items' order: chunks fixed by the input,
    /// not the threads, so a float sum is the same on any number. `None`
    /// for no items.
    pub fn reduce<T: Sync, A: Send>(
        &self,
        items: &[T],
        chunk: usize,
        map: impl Fn(&[T]) -> A + Sync,
        fold: impl FnMut(A, A) -> A,
    ) -> Option<A> {
        items.chunks(chunk.max(1)).map(map).reduce(fold)
    }
}

/// Runs a program of stages over items in colors.
pub struct Passes<'w> {
    /// Hand kernels the shared form of the states even on one thread
    /// (`World::set_shapes_shared`).
    shared: bool,
    _world: PhantomData<&'w World>,
}

impl Param for Passes<'static> {
    type Item<'w> = Passes<'w>;

    fn declare(_: &mut Declare<'_>) -> ParamDecl {
        ParamDecl::Shape(ShapeKind::Passes)
    }

    fn fetch<'w>(cx: &FrameCx<'w>, _: &'w ParamDecl) -> Passes<'w> {
        Passes { shared: cx.world.shapes_shared.load(Ordering::Relaxed), _world: PhantomData }
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
/// first), or each of `n` states by range.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stage<K> {
    Items(K),
    Each(K, usize),
}

impl Passes<'_> {
    /// Runs `program` over `items`, laid out as `layout` says, and the
    /// states they share: each stage done before the next starts, an
    /// `Items` stage the overflow's items and then each color's. `block`
    /// gets consecutive items of one color: all of them on one thread,
    /// some on several, so it must treat them independently. `each` gets a
    /// range of `0..n`. Kernels get the states as [`States`]: the slice
    /// itself on one thread (`Plain`), and on several the shared form,
    /// made from it before the first stage and written back after the last
    /// (`Shared`), which kernels write only where their items' edges are.
    /// A color's items share no moving state, so any order of its blocks
    /// gives the same states.
    ///
    /// Box2D v3's staged solve (`b2SolverStage`, `solver.c`), generic, as
    /// physics2d's `lanes::run_across` is that solve.
    pub fn run<I: Send, T: Shareable, K: Copy + Sync>(
        &self,
        layout: &Colored,
        items: &mut [I],
        states: &mut [T],
        program: &[Stage<K>],
        block: impl Fn(K, &mut [I], States<'_, T>) + Sync,
        each: impl Fn(K, Range<usize>, States<'_, T>) + Sync,
    ) {
        assert_eq!(items.len(), layout.items(), "items as the coloring laid them out");
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
    block: &impl Fn(K, &mut [I], States<'_, T>),
    each: &impl Fn(K, Range<usize>, States<'_, T>),
    mut states: States<'_, T>,
) {
    for stage in program {
        match *stage {
            Stage::Items(k) => {
                let mut rest = &mut items[..];
                for n in std::iter::once(layout.overflow).chain(layout.colors.iter().copied()) {
                    let (color, tail) = std::mem::take(&mut rest).split_at_mut(n);
                    rest = tail;
                    // A color can be empty: the greedy coloring keeps color
                    // 0 from edges at a fixed state.
                    if !color.is_empty() {
                        block(k, color, states.reborrow());
                    }
                }
            }
            Stage::Each(k, n) if n > 0 => each(k, 0..n, states.reborrow()),
            Stage::Each(..) => {}
        }
    }
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

impl Colored {
    pub fn items(&self) -> usize {
        self.overflow + self.colors.iter().sum::<usize>()
    }
}
