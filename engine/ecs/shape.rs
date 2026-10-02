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
pub struct Passes<'w>(PhantomData<&'w World>);
shape_param!(Passes, Passes);

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
    /// range of `0..n`. States are shared as `&S`, which kernels write only
    /// where their items' edges are (relaxed atomics, in physics). A color's
    /// items share no moving state, so any order of its blocks gives the
    /// same states.
    ///
    /// Box2D v3's staged solve (`b2SolverStage`, `solver.c`), generic, as
    /// physics2d's `lanes::run_across` is that solve.
    pub fn run<I: Send, S: Sync + ?Sized, K: Copy + Sync>(
        &self,
        layout: &Colored,
        items: &mut [I],
        states: &S,
        program: &[Stage<K>],
        block: impl Fn(K, &mut [I], &S) + Sync,
        each: impl Fn(K, Range<usize>, &S) + Sync,
    ) {
        assert_eq!(items.len(), layout.items(), "items as the coloring laid them out");
        for stage in program {
            match *stage {
                Stage::Items(k) => {
                    let mut rest = &mut items[..];
                    for n in std::iter::once(layout.overflow).chain(layout.colors.iter().copied()) {
                        let (color, tail) = std::mem::take(&mut rest).split_at_mut(n);
                        rest = tail;
                        // A color can be empty: the greedy coloring keeps
                        // color 0 from edges at a fixed state.
                        if !color.is_empty() {
                            block(k, color, states);
                        }
                    }
                }
                Stage::Each(k, n) if n > 0 => each(k, 0..n, states),
                Stage::Each(..) => {}
            }
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
