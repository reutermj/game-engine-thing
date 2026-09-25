//! Why `solve` copies bodies out of the world, measured one cause at a
//! time: `./bazel run -c opt //engine/std/physics:solver_layout`.
//!
//! A pile of 10 000 runs in the engine to 400 steps (settled), then this
//! takes the solver's input as the mod would gather it (bodies in the
//! world's order, contacts in pair order), and runs the same sequential
//! impulses over it stored in different ways: AoS or SoA, flat or in pages,
//! inverse mass stored or derived from `Body`, bodies in other orders, and
//! in place in the world's own `Velocity` pages. Each must end bit for bit
//! as `solver::solve` on the copy does, or it isn't the same computation.
//! Then the copy itself (gather and scatter against the world's pages) is
//! timed, since that's what solving in place would save.
//!
//! The solver here is `solver::solve` made generic over where bodies are
//! kept (`Store`), with the same operations in the same order; the first
//! rows check that the generic one over the same array costs what the real
//! one does, so the other rows measure the storage, not the rewrite.
//! `solve_at` is the same again with each contact's bodies looked up once
//! at its start, to tell the cost of a lookup per touch from that of the
//! lookup itself. What it found: docs/architecture/physics.md, "What the
//! ECS costs".

#[path = "../solver.rs"]
mod solver;

use std::collections::HashMap;
use std::hint::black_box;
use std::path::PathBuf;
use std::sync::PoisonError;
use std::time::Instant;

use engine_ecs::{Entity, World};
use engine_loader::engine::Engine;
use physics::{Body, ContactPair, DYNAMIC, Gravity, Impulse, Manifold, Response, Vec2, Velocity};
use solver::{BETA, BOUNCE_THRESHOLD, Constraint, ITERATIONS, SLOP, SolverBody};

const DT: f32 = 1.0 / 60.0;
const REPS: usize = 31;
/// Rows in a spatial table's page (`engine_ecs::spatial::SPATIAL_PAGE_ROWS`),
/// checked against the world's tables below.
const SPATIAL_ROWS: usize = 16;

// ---- The solver, generic over storage ----

/// Where the solver finds a body's velocity, inverse mass and pseudo
/// velocity, by whatever index the constraints carry.
trait Store {
    fn inv(&self, i: u32) -> f32;
    fn v(&self, i: u32) -> Vec2;
    fn set_v(&mut self, i: u32, v: Vec2);
    fn p(&self, i: u32) -> Vec2;
    fn set_p(&mut self, i: u32, p: Vec2);
}

/// `solver::solve`, line for line, over a `Store`.
fn solve<S: Store>(s: &mut S, contacts: &mut [Constraint], dt: f32) {
    let mut targets = vec![0.0; contacts.len()];
    for (c, target) in contacts.iter_mut().zip(&mut targets) {
        let vn = (s.v(c.b) - s.v(c.a)).dot(c.normal);
        c.speed = -vn;
        *target = if c.depth < 0.0 {
            c.depth / dt
        } else if -vn > BOUNCE_THRESHOLD {
            -c.restitution * vn
        } else {
            0.0
        };
    }
    for c in contacts.iter() {
        let impulse = c.normal * c.jn + c.normal.perp() * c.jt;
        apply_v(s, c.a, c.b, impulse);
    }
    for _ in 0..ITERATIONS {
        for (c, &target) in contacts.iter_mut().zip(&targets) {
            let k = s.inv(c.a) + s.inv(c.b);
            if k == 0.0 {
                continue;
            }
            let rel = s.v(c.b) - s.v(c.a);
            let jn = (c.jn + (target - rel.dot(c.normal)) / k).max(0.0);
            let dn = jn - c.jn;
            c.jn = jn;
            apply_v(s, c.a, c.b, c.normal * dn);

            let t = c.normal.perp();
            let rel = s.v(c.b) - s.v(c.a);
            let limit = c.friction * c.jn;
            let jt = (c.jt - rel.dot(t) / k).clamp(-limit, limit);
            let dt_ = jt - c.jt;
            c.jt = jt;
            apply_v(s, c.a, c.b, t * dt_);
        }
    }
    let mut pj = vec![0.0f32; contacts.len()];
    for _ in 0..ITERATIONS {
        for (c, pj) in contacts.iter().zip(&mut pj) {
            let k = s.inv(c.a) + s.inv(c.b);
            if k == 0.0 || c.depth <= SLOP {
                continue;
            }
            let bias = BETA * (c.depth - SLOP) / dt;
            let rel = s.p(c.b) - s.p(c.a);
            let new = (*pj + (bias - rel.dot(c.normal)) / k).max(0.0);
            let d = new - *pj;
            *pj = new;
            let (ia, ib) = (s.inv(c.a), s.inv(c.b));
            let pa = s.p(c.a) - c.normal * d * ia;
            s.set_p(c.a, pa);
            let pb = s.p(c.b) + c.normal * d * ib;
            s.set_p(c.b, pb);
        }
    }
}

#[inline(always)]
fn apply_v<S: Store>(s: &mut S, a: u32, b: u32, impulse: Vec2) {
    let (ia, ib) = (s.inv(a), s.inv(b));
    let va = s.v(a) - impulse * ia;
    s.set_v(a, va);
    let vb = s.v(b) + impulse * ib;
    s.set_v(b, vb);
}

/// A constraint that carries its bodies' inverse masses, as Box2D's
/// contact constraints do, so the solver reads a body only for velocities.
#[derive(Clone, Copy)]
struct Cached {
    a: u32,
    b: u32,
    ia: f32,
    ib: f32,
    normal: Vec2,
    depth: f32,
    friction: f32,
    restitution: f32,
    jn: f32,
    jt: f32,
    speed: f32,
}

/// The same as `solve`, reading inverse masses from the constraint.
fn solve_cached<S: Store>(s: &mut S, contacts: &mut [Cached], dt: f32) {
    let mut targets = vec![0.0; contacts.len()];
    for (c, target) in contacts.iter_mut().zip(&mut targets) {
        let vn = (s.v(c.b) - s.v(c.a)).dot(c.normal);
        c.speed = -vn;
        *target = if c.depth < 0.0 {
            c.depth / dt
        } else if -vn > BOUNCE_THRESHOLD {
            -c.restitution * vn
        } else {
            0.0
        };
    }
    for c in contacts.iter() {
        let impulse = c.normal * c.jn + c.normal.perp() * c.jt;
        apply_cached(s, c, impulse);
    }
    for _ in 0..ITERATIONS {
        for (c, &target) in contacts.iter_mut().zip(&targets) {
            let k = c.ia + c.ib;
            if k == 0.0 {
                continue;
            }
            let rel = s.v(c.b) - s.v(c.a);
            let jn = (c.jn + (target - rel.dot(c.normal)) / k).max(0.0);
            let dn = jn - c.jn;
            c.jn = jn;
            apply_cached(s, c, c.normal * dn);

            let t = c.normal.perp();
            let rel = s.v(c.b) - s.v(c.a);
            let limit = c.friction * c.jn;
            let jt = (c.jt - rel.dot(t) / k).clamp(-limit, limit);
            let dt_ = jt - c.jt;
            c.jt = jt;
            apply_cached(s, c, t * dt_);
        }
    }
    let mut pj = vec![0.0f32; contacts.len()];
    for _ in 0..ITERATIONS {
        for (c, pj) in contacts.iter().zip(&mut pj) {
            let k = c.ia + c.ib;
            if k == 0.0 || c.depth <= SLOP {
                continue;
            }
            let bias = BETA * (c.depth - SLOP) / dt;
            let rel = s.p(c.b) - s.p(c.a);
            let new = (*pj + (bias - rel.dot(c.normal)) / k).max(0.0);
            let d = new - *pj;
            *pj = new;
            let pa = s.p(c.a) - c.normal * d * c.ia;
            s.set_p(c.a, pa);
            let pb = s.p(c.b) + c.normal * d * c.ib;
            s.set_p(c.b, pb);
        }
    }
}

#[inline(always)]
fn apply_cached<S: Store>(s: &mut S, c: &Cached, impulse: Vec2) {
    let va = s.v(c.a) - impulse * c.ia;
    s.set_v(c.a, va);
    let vb = s.v(c.b) + impulse * c.ib;
    s.set_v(c.b, vb);
}

// ---- Stores ----

/// The mod's copy: one array of `SolverBody`.
struct Aos(Vec<SolverBody>);

impl Store for Aos {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        self.0[i as usize].inv_mass
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        self.0[i as usize].v
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        self.0[i as usize].v = v;
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.0[i as usize].pseudo
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.0[i as usize].pseudo = p;
    }
}

/// Columns: what the world keeps, but flat.
struct Soa {
    v: Vec<Vec2>,
    inv: Vec<f32>,
    p: Vec<Vec2>,
}

impl Store for Soa {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        self.inv[i as usize]
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        self.v[i as usize]
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        self.v[i as usize] = v;
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.p[i as usize]
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.p[i as usize] = p;
    }
}

/// The mod's rule for a body's inverse mass, as `solve` gathers it (no body
/// is asleep here).
#[inline(always)]
fn derived(b: &Body) -> f32 {
    if b.kind == DYNAMIC { b.inv_mass } else { 0.0 }
}

/// Columns with the inverse mass derived from `Body` on every touch.
struct SoaDerived {
    v: Vec<Vec2>,
    body: Vec<Body>,
    p: Vec<Vec2>,
}

impl Store for SoaDerived {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        derived(&self.body[i as usize])
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        self.v[i as usize]
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        self.v[i as usize] = v;
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.p[i as usize]
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.p[i as usize] = p;
    }
}

/// Velocity, and maybe more, in pages of `R` rows, each its own allocation
/// as the world's are; indexed by `page * R + row`, so finding a value is
/// one load of the page's pointer more than a flat array.
type Pages<T, const R: usize> = Vec<Box<[T; R]>>;

fn pages_of<T: Copy + Default, const R: usize>(flat: &[T]) -> Pages<T, R> {
    flat.chunks(R)
        .map(|c| {
            let mut page = Box::new([T::default(); R]);
            page[..c.len()].copy_from_slice(c);
            page
        })
        .collect()
}

/// Every field paged.
struct PagedAll<const R: usize> {
    v: Pages<Vec2, R>,
    inv: Pages<f32, R>,
    p: Pages<Vec2, R>,
}

impl<const R: usize> Store for PagedAll<R> {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        self.inv[i as usize / R][i as usize % R]
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        self.v[i as usize / R][i as usize % R]
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        self.v[i as usize / R][i as usize % R] = v;
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.p[i as usize / R][i as usize % R]
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.p[i as usize / R][i as usize % R] = p;
    }
}

/// Only velocity paged (a column); inverse mass and pseudo velocity in
/// flat scratch arrays by the same index, holes and all.
struct PagedV<const R: usize> {
    v: Pages<Vec2, R>,
    inv: Vec<f32>,
    p: Vec<Vec2>,
}

impl<const R: usize> Store for PagedV<R> {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        self.inv[i as usize]
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        self.v[i as usize / R][i as usize % R]
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        self.v[i as usize / R][i as usize % R] = v;
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.p[i as usize]
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.p[i as usize] = p;
    }
}

/// A pointer per body to its velocity wherever it lives (the tried
/// in-place solve's shape), by flat index.
struct PerBody {
    v: Vec<*mut Vec2>,
    inv: Vec<f32>,
    p: Vec<Vec2>,
}

impl Store for PerBody {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        self.inv[i as usize]
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        // SAFETY: every pointer is into pages kept alive, unmoved, and
        // otherwise unborrowed while the store is used.
        unsafe { *self.v[i as usize] }
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        // SAFETY: as in `v`.
        unsafe { *self.v[i as usize] = v }
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.p[i as usize]
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.p[i as usize] = p;
    }
}

/// In place in the world: velocity in its `Velocity` pages, by
/// `page * 16 + row`, through a pointer per page; the inverse mass derived
/// from its `Body` page, or from a flat scratch array; pseudo velocity in
/// scratch, since it has no column.
struct InWorld {
    v: Vec<*mut Velocity>,
    body: Vec<*const Body>,
    inv: Option<Vec<f32>>,
    p: Vec<Vec2>,
}

impl Store for InWorld {
    #[inline(always)]
    fn inv(&self, i: u32) -> f32 {
        match &self.inv {
            Some(inv) => inv[i as usize],
            // SAFETY: as in `v`, for the world's `Body` pages, read only.
            None => derived(unsafe { &*self.body[i as usize / SPATIAL_ROWS].add(i as usize % SPATIAL_ROWS) }),
        }
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        // SAFETY: the pointers are into pages whose column guards `main`
        // holds while this is used, and indices are to rows that exist.
        let v = unsafe { &*self.v[i as usize / SPATIAL_ROWS].add(i as usize % SPATIAL_ROWS) };
        Vec2::new(v.x, v.y)
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        // SAFETY: as in `v`, under the write guards.
        let to = unsafe { &mut *self.v[i as usize / SPATIAL_ROWS].add(i as usize % SPATIAL_ROWS) };
        (to.x, to.y) = (v.x, v.y);
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.p[i as usize]
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.p[i as usize] = p;
    }
}

/// Velocity and pseudo velocity only (inverse masses in the constraints),
/// together: 16 bytes a body.
#[derive(Clone, Copy, Default)]
struct VP {
    v: Vec2,
    p: Vec2,
}

struct AosVP(Vec<VP>);

impl Store for AosVP {
    fn inv(&self, _: u32) -> f32 {
        unreachable!("in the constraint")
    }
    #[inline(always)]
    fn v(&self, i: u32) -> Vec2 {
        self.0[i as usize].v
    }
    #[inline(always)]
    fn set_v(&mut self, i: u32, v: Vec2) {
        self.0[i as usize].v = v;
    }
    #[inline(always)]
    fn p(&self, i: u32) -> Vec2 {
        self.0[i as usize].p
    }
    #[inline(always)]
    fn set_p(&mut self, i: u32, p: Vec2) {
        self.0[i as usize].p = p;
    }
}

// ---- Resolving a body once per contact ----

/// A velocity as some store keeps it: `Vec2`, or the world's `Velocity`.
trait Xy {
    fn get(&self) -> Vec2;
    fn set(&mut self, v: Vec2);
}

impl Xy for Vec2 {
    #[inline(always)]
    fn get(&self) -> Vec2 {
        *self
    }
    #[inline(always)]
    fn set(&mut self, v: Vec2) {
        *self = v;
    }
}

impl Xy for Velocity {
    #[inline(always)]
    fn get(&self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }
    #[inline(always)]
    fn set(&mut self, v: Vec2) {
        (self.x, self.y) = (v.x, v.y);
    }
}

/// One body, found: where its velocity and pseudo velocity are, and its
/// inverse mass, read once.
#[derive(Clone, Copy)]
struct At<V> {
    v: *mut V,
    inv: f32,
    p: *mut Vec2,
}

/// A store the solver looks a body up in once per contact, instead of on
/// every touch. Implementations hold raw base pointers made once, so a
/// lookup reads only the store's own tables.
trait Locate {
    type V: Xy;
    fn at(&self, i: u32) -> At<Self::V>;
}

/// `solve` with each contact's two bodies found once, at its start. The
/// same operations in the same order, so the same bits.
fn solve_at<S: Locate>(s: &S, contacts: &mut [Constraint], dt: f32) {
    // SAFETY, for every deref below: an `At` points into storage its store
    // keeps alive and unmoved, which nothing else touches during the solve;
    // `a` and `b` may be the same body only for the stand-in, whose
    // velocity is never changed (inverse mass 0), and even then each access
    // is a separate read or write, never two live references.
    let (v, set_v) = (|x: *mut S::V| unsafe { (*x).get() }, |x: *mut S::V, v: Vec2| unsafe { (*x).set(v) });
    let (p, set_p) = (|x: *mut Vec2| unsafe { *x }, |x: *mut Vec2, v: Vec2| unsafe { *x = v });
    let apply = |a: &At<S::V>, b: &At<S::V>, impulse: Vec2| {
        set_v(a.v, v(a.v) - impulse * a.inv);
        set_v(b.v, v(b.v) + impulse * b.inv);
    };
    let mut targets = vec![0.0; contacts.len()];
    for (c, target) in contacts.iter_mut().zip(&mut targets) {
        let (a, b) = (s.at(c.a), s.at(c.b));
        let vn = (v(b.v) - v(a.v)).dot(c.normal);
        c.speed = -vn;
        *target = if c.depth < 0.0 {
            c.depth / dt
        } else if -vn > BOUNCE_THRESHOLD {
            -c.restitution * vn
        } else {
            0.0
        };
    }
    for c in contacts.iter() {
        let (a, b) = (s.at(c.a), s.at(c.b));
        apply(&a, &b, c.normal * c.jn + c.normal.perp() * c.jt);
    }
    for _ in 0..ITERATIONS {
        for (c, &target) in contacts.iter_mut().zip(&targets) {
            let (a, b) = (s.at(c.a), s.at(c.b));
            let k = a.inv + b.inv;
            if k == 0.0 {
                continue;
            }
            let rel = v(b.v) - v(a.v);
            let jn = (c.jn + (target - rel.dot(c.normal)) / k).max(0.0);
            let dn = jn - c.jn;
            c.jn = jn;
            apply(&a, &b, c.normal * dn);

            let t = c.normal.perp();
            let rel = v(b.v) - v(a.v);
            let limit = c.friction * c.jn;
            let jt = (c.jt - rel.dot(t) / k).clamp(-limit, limit);
            let dt_ = jt - c.jt;
            c.jt = jt;
            apply(&a, &b, t * dt_);
        }
    }
    let mut pj = vec![0.0f32; contacts.len()];
    for _ in 0..ITERATIONS {
        for (c, pj) in contacts.iter().zip(&mut pj) {
            let (a, b) = (s.at(c.a), s.at(c.b));
            let k = a.inv + b.inv;
            if k == 0.0 || c.depth <= SLOP {
                continue;
            }
            let bias = BETA * (c.depth - SLOP) / dt;
            let rel = p(b.p) - p(a.p);
            let new = (*pj + (bias - rel.dot(c.normal)) / k).max(0.0);
            let d = new - *pj;
            *pj = new;
            set_p(a.p, p(a.p) - c.normal * d * a.inv);
            set_p(b.p, p(b.p) + c.normal * d * b.inv);
        }
    }
}

/// `[SolverBody]`, by base pointer.
struct AosAt(*mut SolverBody);

impl Locate for AosAt {
    type V = Vec2;
    #[inline(always)]
    fn at(&self, i: u32) -> At<Vec2> {
        // SAFETY: `i` indexes the array this points at (the constraints
        // were made for it).
        unsafe {
            let b = self.0.add(i as usize);
            At { v: &raw mut (*b).v, inv: (*b).inv_mass, p: &raw mut (*b).pseudo }
        }
    }
}

/// Pages of `R` rows, by a table of page pointers per field.
struct PagedAt<const R: usize> {
    v: Vec<*mut Vec2>,
    inv: Vec<*const f32>,
    p: Vec<*mut Vec2>,
}

impl<const R: usize> PagedAt<R> {
    fn of(s: &mut PagedAll<R>) -> PagedAt<R> {
        PagedAt {
            v: s.v.iter_mut().map(|p| p.as_mut_ptr()).collect(),
            inv: s.inv.iter().map(|p| p.as_ptr()).collect(),
            p: s.p.iter_mut().map(|p| p.as_mut_ptr()).collect(),
        }
    }
}

impl<const R: usize> Locate for PagedAt<R> {
    type V = Vec2;
    #[inline(always)]
    fn at(&self, i: u32) -> At<Vec2> {
        let (page, row) = (i as usize / R, i as usize % R);
        // SAFETY: as in `AosAt`, by page and row.
        unsafe { At { v: self.v[page].add(row), inv: *self.inv[page].add(row), p: self.p[page].add(row) } }
    }
}

/// A pointer per body to its velocity, the rest flat.
struct PerBodyAt {
    v: Vec<*mut Vec2>,
    inv: Vec<f32>,
    p: *mut Vec2,
}

impl Locate for PerBodyAt {
    type V = Vec2;
    #[inline(always)]
    fn at(&self, i: u32) -> At<Vec2> {
        // SAFETY: as in `AosAt`.
        unsafe { At { v: self.v[i as usize], inv: self.inv[i as usize], p: self.p.add(i as usize) } }
    }
}

/// In place in the world's pages, by `page * 16 + row`; the inverse mass
/// derived from `Body` once per contact; pseudo velocity in flat scratch
/// by the same index.
struct WorldAt {
    v: Vec<*mut Velocity>,
    body: Vec<*const Body>,
    p: *mut Vec2,
}

impl Locate for WorldAt {
    type V = Velocity;
    #[inline(always)]
    fn at(&self, i: u32) -> At<Velocity> {
        let (page, row) = (i as usize / SPATIAL_ROWS, i as usize % SPATIAL_ROWS);
        // SAFETY: as in `InWorld`.
        unsafe { At { v: self.v[page].add(row), inv: derived(&*self.body[page].add(row)), p: self.p.add(i as usize) } }
    }
}

// ---- The scene ----

/// The solver's input as `solve` gathers it at 10 000 settled, plus where
/// each body lives in the world.
struct Scene {
    /// In the world's order (table, page, row); the immovable stand-in for
    /// statics last.
    bodies: Vec<SolverBody>,
    body: Vec<Body>,
    entity: Vec<Entity>,
    /// Each body's `page * 16 + row` in the world, pages numbered across
    /// tables; the stand-in gets a page of its own after them.
    packed: Vec<u32>,
    pages: usize,
    contacts: Vec<Constraint>,
}

impl Scene {
    fn of(w: &World) -> Scene {
        let (cb, cv, cp) = (w.id("physics::Body").unwrap(), w.id("physics::Velocity").unwrap(), w.id("physics::Position").unwrap());
        let gravity = w.values::<Gravity>().unwrap().first().map_or(Vec2::ZERO, |(_, g)| Vec2::new(g.x, g.y));
        let (mut bodies, mut body, mut entity, mut packed) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut pages = 0usize;
        for t in w.tables() {
            let (Some(ib), Some(iv), Some(_)) = (t.column_index(cb), t.column_index(cv), t.column_index(cp)) else { continue };
            assert_eq!(t.page_rows, SPATIAL_ROWS, "bodies are in a spatial table");
            let rows = t.rows.read().unwrap_or_else(PoisonError::into_inner);
            let bs = t.columns[ib].read().unwrap_or_else(PoisonError::into_inner);
            let vs = t.columns[iv].read().unwrap_or_else(PoisonError::into_inner);
            for (p, es) in rows.iter().enumerate() {
                for (r, &e) in es.iter().enumerate() {
                    let (b, v) = (bs[p].as_slice::<Body>()[r], vs[p].as_slice::<Velocity>()[r]);
                    // Gravity first, as the step does before the solve.
                    let g = if b.kind == DYNAMIC { gravity * (b.gravity_scale * DT) } else { Vec2::ZERO };
                    bodies.push(SolverBody { v: Vec2::new(v.x, v.y) + g, inv_mass: derived(&b), pseudo: Vec2::ZERO });
                    body.push(b);
                    entity.push(e);
                    packed.push(((pages + p) * SPATIAL_ROWS + r) as u32);
                }
            }
            pages += rows.len();
        }
        let still = bodies.len() as u32;
        bodies.push(SolverBody::default());
        body.push(Body::fixed());
        packed.push((pages * SPATIAL_ROWS) as u32);
        pages += 1;
        let index: HashMap<Entity, u32> = entity.iter().enumerate().map(|(i, e)| (*e, i as u32)).collect();

        let manifolds: HashMap<Entity, Manifold> = w.values::<Manifold>().unwrap().into_iter().collect();
        let responses: HashMap<Entity, Response> = w.values::<Response>().unwrap().into_iter().collect();
        let impulses: HashMap<Entity, Impulse> = w.values::<Impulse>().unwrap().into_iter().collect();
        let mut pairs = w.values::<ContactPair>().unwrap();
        pairs.sort_by_key(|(_, p)| (p.a, p.b));
        let contacts = pairs
            .iter()
            .filter(|(e, _)| !responses[e].disabled)
            .map(|(e, p)| {
                let (m, r, j) = (manifolds[e], responses[e], impulses[e]);
                Constraint {
                    a: index.get(&p.a).copied().unwrap_or(still),
                    b: index.get(&p.b).copied().unwrap_or(still),
                    normal: Vec2::new(m.nx, m.ny),
                    depth: m.depth,
                    friction: r.friction,
                    restitution: r.restitution,
                    jn: j.normal,
                    jt: j.tangent,
                    speed: 0.0,
                }
            })
            .collect();
        Scene { bodies, body, entity, packed, pages, contacts }
    }

    /// The contacts with body indices through `map`.
    fn contacts_by(&self, map: &[u32]) -> Vec<Constraint> {
        self.contacts.iter().map(|c| Constraint { a: map[c.a as usize], b: map[c.b as usize], ..*c }).collect()
    }

    fn cached_by(&self, map: &[u32]) -> Vec<Cached> {
        self.contacts
            .iter()
            .map(|c| Cached {
                a: map[c.a as usize],
                b: map[c.b as usize],
                ia: self.bodies[c.a as usize].inv_mass,
                ib: self.bodies[c.b as usize].inv_mass,
                normal: c.normal,
                depth: c.depth,
                friction: c.friction,
                restitution: c.restitution,
                jn: c.jn,
                jt: c.jt,
                speed: c.speed,
            })
            .collect()
    }

    /// A flat array by `map`'s indices, `fill` in the holes.
    fn scatter<T: Copy>(&self, map: &[u32], len: usize, fill: T, f: impl Fn(usize) -> T) -> Vec<T> {
        let mut out = vec![fill; len];
        for (i, &m) in map.iter().enumerate() {
            out[m as usize] = f(i);
        }
        out
    }
}

// ---- Checking and timing ----

/// The solver's outputs in the scene's order: velocity, pseudo velocity,
/// then each contact's impulses and speed, as bits.
type Outcome = (Vec<[u32; 4]>, Vec<[u32; 3]>);

fn outcome(n: usize, body: impl Fn(usize) -> (Vec2, Vec2), contacts: impl Iterator<Item = (f32, f32, f32)>) -> Outcome {
    let b = (0..n).map(&body).map(|(v, p)| [v.x.to_bits(), v.y.to_bits(), p.x.to_bits(), p.y.to_bits()]).collect();
    let c = contacts.map(|(jn, jt, s)| [jn.to_bits(), jt.to_bits(), s.to_bits()]).collect();
    (b, c)
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(f64::total_cmp);
    xs[xs.len() / 2]
}

/// One solve over fresh inputs from `setup`, timed; with the outcome when
/// asked, which must be the reference's.
type Variant<'a> = Box<dyn FnMut(bool) -> (f64, Option<Outcome>) + 'a>;

fn time<'a, I: 'a>(
    setup: impl Fn() -> I + 'a,
    mut run: impl FnMut(&mut I) + 'a,
    check: impl Fn(&I) -> Outcome + 'a,
) -> Variant<'a> {
    Box::new(move |checked| {
        let mut input = setup();
        let start = Instant::now();
        run(black_box(&mut input));
        let us = start.elapsed().as_secs_f64() * 1e6;
        (us, checked.then(|| check(&input)))
    })
}

/// The variants, run round robin so drift in the clock (boost, heat) lands
/// on all of them alike rather than on whichever ran last.
struct Report<'a> {
    reference: Outcome,
    rows: Vec<(String, Variant<'a>)>,
}

impl<'a> Report<'a> {
    fn row(&mut self, name: &str, v: Variant<'a>) {
        self.rows.push((name.to_string(), v));
    }

    fn finish(mut self) {
        let mut times = vec![Vec::new(); self.rows.len()];
        let mut same = vec![false; self.rows.len()];
        for round in 0..REPS {
            for (k, (_, v)) in self.rows.iter_mut().enumerate() {
                let (us, out) = v(round == REPS - 1);
                times[k].push(us);
                if let Some(out) = out {
                    same[k] = out == self.reference;
                }
            }
        }
        println!("| storage | µs | Δ | min | bit for bit |");
        println!("|---|---|---|---|---|");
        let base = median(times[0].clone());
        for (k, (name, _)) in self.rows.iter().enumerate() {
            let us = median(times[k].clone());
            let min = times[k].iter().copied().fold(f64::MAX, f64::min);
            println!("| {name} | {us:.0} | {:+.0} | {min:.0} | {} |", us - base, if same[k] { "yes" } else { "**no**" });
        }
    }
}

fn main() {
    let manifest = engine_control::read_manifest(&std::env::var("PILE").unwrap()).unwrap();
    let dir = std::env::temp_dir().join(format!("physics-solver-layout-{}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), PathBuf::from(&dir));
    e.load_batch(&manifest.mods).expect("loading the pile");
    e.send("pile", "widen 400").unwrap();
    e.send("pile", "drop 10000").unwrap();
    e.send("lockstep", "step 400").unwrap();
    let scene = Scene::of(e.world());
    let s = &scene;
    let n = s.bodies.len();
    let filled = s.packed.len() as f64 / s.pages as f64;
    println!(
        "10 000 settled: {} bodies (and the stand-in for statics), {} contacts, {} pages of {SPATIAL_ROWS} ({filled:.1} rows each)",
        n - 1,
        s.contacts.len(),
        s.pages,
    );
    let with_depth = s.contacts.iter().filter(|c| c.depth > SLOP).count();
    let movable = s.contacts.iter().filter(|c| s.bodies[c.a as usize].inv_mass + s.bodies[c.b as usize].inv_mass != 0.0).count();
    println!("  {movable} contacts solved per velocity iteration, {with_depth} sunk past the slop (the split impulse's)\n");

    let identity: Vec<u32> = (0..n as u32).collect();
    let aos_out = |b: &(Vec<SolverBody>, Vec<Constraint>), map: &[u32]| {
        outcome(n, |i| (b.0[map[i] as usize].v, b.0[map[i] as usize].pseudo), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
    };
    let reference = {
        let mut b = (s.bodies.clone(), s.contacts.clone());
        solver::solve(&mut b.0, &mut b.1, DT);
        aos_out(&b, &identity)
    };
    // In place in the world's pages, under its column guards.
    let w = e.world();
    let (cb, cv) = (w.id("physics::Body").unwrap(), w.id("physics::Velocity").unwrap());
    let tables: Vec<_> = w.tables().filter(|t| t.column_index(cb).is_some() && t.column_index(cv).is_some() && t.column_index(w.id("physics::Position").unwrap()).is_some()).collect();
    let mut vguards: Vec<_> = tables.iter().map(|t| t.columns[t.column_index(cv).unwrap()].write().unwrap()).collect();
    let bguards: Vec<_> = tables.iter().map(|t| t.columns[t.column_index(cb).unwrap()].read().unwrap()).collect();
    // The stand-in for statics needs a velocity somewhere: a page of its own.
    let mut still = Box::new([Velocity { x: 0.0, y: 0.0 }; SPATIAL_ROWS]);
    let still_body = Box::new([Body::fixed(); SPATIAL_ROWS]);
    let mut vptrs: Vec<*mut Velocity> = vguards.iter_mut().flat_map(|g| g.iter_mut().map(|c| c.as_mut_slice::<Velocity>().as_mut_ptr())).collect();
    vptrs.push(still.as_mut_ptr());
    let mut bptrs: Vec<*const Body> = bguards.iter().flat_map(|g| g.iter().map(|c| c.as_slice::<Body>().as_ptr())).collect();
    bptrs.push(still_body.as_ptr());
    assert_eq!(vptrs.len(), s.pages);
    let reset_world = |vptrs: &[*mut Velocity]| {
        for (i, &k) in s.packed.iter().enumerate() {
            let v = s.bodies[i].v;
            // SAFETY: as in `InWorld`.
            unsafe { *vptrs[k as usize / SPATIAL_ROWS].add(k as usize % SPATIAL_ROWS) = Velocity { x: v.x, y: v.y } };
        }
    };
    let world_out = |b: &(InWorld, Vec<Constraint>)| {
        outcome(n, |i| (b.0.v(s.packed[i]), b.0.p[s.packed[i] as usize]), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
    };

    let mut r = Report { reference, rows: Vec::new() };

    // (a) layout, flat.
    r.row(
        "`solver::solve` on `[SolverBody]` (the mod's copy)",
        time(|| (s.bodies.clone(), s.contacts.clone()), |b| solver::solve(&mut b.0, &mut b.1, DT), |b| aos_out(b, &identity)),
    );
    r.row(
        "generic solver, same AoS array",
        time(
            || (Aos(s.bodies.clone()), s.contacts.clone()),
            |b| solve(&mut b.0, &mut b.1, DT),
            |b| outcome(n, |i| (b.0.0[i].v, b.0.0[i].pseudo), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );
    let soa = || Soa {
        v: s.bodies.iter().map(|b| b.v).collect(),
        inv: s.bodies.iter().map(|b| b.inv_mass).collect(),
        p: vec![Vec2::ZERO; n],
    };
    let soa_out = |b: &(Soa, Vec<Constraint>), map: &[u32]| {
        outcome(n, |i| (b.0.v[map[i] as usize], b.0.p[map[i] as usize]), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
    };
    r.row("(a) SoA: v, inv_mass, pseudo in three flat arrays", time(|| (soa(), s.contacts.clone()), |b| solve(&mut b.0, &mut b.1, DT), |b| soa_out(b, &identity)));

    // (c) derived inverse mass.
    r.row(
        "(c) SoA, inv_mass derived from a flat `[Body]` each touch",
        time(
            || (SoaDerived { v: s.bodies.iter().map(|b| b.v).collect(), body: s.body.clone(), p: vec![Vec2::ZERO; n] }, s.contacts.clone()),
            |b| solve(&mut b.0, &mut b.1, DT),
            |b| outcome(n, |i| (b.0.v[i], b.0.p[i]), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );
    r.row(
        "(c) inv_mass in the constraint, AoS {v, pseudo}",
        time(
            || (AosVP(s.bodies.iter().map(|b| VP { v: b.v, p: Vec2::ZERO }).collect()), s.cached_by(&identity)),
            |b| solve_cached(&mut b.0, &mut b.1, DT),
            |b| outcome(n, |i| (b.0.0[i].v, b.0.0[i].p), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );
    r.row(
        "(c) inv_mass in the constraint, SoA v and pseudo",
        time(
            || (soa(), s.cached_by(&identity)),
            |b| solve_cached(&mut b.0, &mut b.1, DT),
            |b| outcome(n, |i| (b.0.v[i], b.0.p[i]), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );

    // (b) pages: the world's own occupancy for 16-row pages (spatial pages
    // aren't full), full pages for 256 (a plain table packs its rows).
    let slots16 = s.pages * SPATIAL_ROWS;
    let paged16 = || {
        let v = s.scatter(&s.packed, slots16, Vec2::ZERO, |i| s.bodies[i].v);
        let inv = s.scatter(&s.packed, slots16, 0.0, |i| s.bodies[i].inv_mass);
        (v, inv)
    };
    r.row(
        "(b) 16-row pages as the world fills them: v, inv, pseudo paged",
        time(
            || {
                let (v, inv) = paged16();
                (PagedAll::<16> { v: pages_of(&v), inv: pages_of(&inv), p: pages_of(&vec![Vec2::ZERO; slots16]) }, s.contacts_by(&s.packed))
            },
            |b| solve(&mut b.0, &mut b.1, DT),
            |b| {
                let at = |i: usize| (s.packed[i] as usize / 16, s.packed[i] as usize % 16);
                outcome(n, |i| (b.0.v[at(i).0][at(i).1], b.0.p[at(i).0][at(i).1]), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        ),
    );
    r.row(
        "(b) 16-row pages: v paged, inv and pseudo flat by the same index",
        time(
            || {
                let (v, inv) = paged16();
                (PagedV::<16> { v: pages_of(&v), inv, p: vec![Vec2::ZERO; slots16] }, s.contacts_by(&s.packed))
            },
            |b| solve(&mut b.0, &mut b.1, DT),
            |b| {
                let at = |i: usize| (s.packed[i] as usize / 16, s.packed[i] as usize % 16);
                outcome(n, |i| (b.0.v[at(i).0][at(i).1], b.0.p[s.packed[i] as usize]), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        ),
    );
    r.row(
        "(b) the same holes, flat: SoA by `page * 16 + row`",
        time(
            || {
                let (v, inv) = paged16();
                (Soa { v, inv, p: vec![Vec2::ZERO; slots16] }, s.contacts_by(&s.packed))
            },
            |b| solve(&mut b.0, &mut b.1, DT),
            |b| soa_out(b, &s.packed),
        ),
    );
    r.row(
        "(b) 256-row pages, full: v, inv, pseudo paged",
        time(
            || {
                let (v, inv): (Vec<Vec2>, Vec<f32>) = s.bodies.iter().map(|b| (b.v, b.inv_mass)).unzip();
                (PagedAll::<256> { v: pages_of(&v), inv: pages_of(&inv), p: pages_of(&vec![Vec2::ZERO; n]) }, s.contacts.clone())
            },
            |b| solve(&mut b.0, &mut b.1, DT),
            |b| outcome(n, |i| (b.0.v[i / 256][i % 256], b.0.p[i / 256][i % 256]), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );
    r.row(
        "(b) a pointer per body into 16-row pages (the tried in-place shape)",
        time(
            || {
                let (v, inv) = paged16();
                let mut pages: Pages<Vec2, 16> = pages_of(&v);
                let ptrs = s.packed.iter().map(|&k| &mut pages[k as usize / 16][k as usize % 16] as *mut Vec2).collect();
                (pages, PerBody { v: ptrs, inv: s.bodies.iter().map(|b| b.inv_mass).collect(), p: vec![Vec2::ZERO; n] }, s.contacts.clone(), inv)
            },
            |b| solve(&mut b.1, &mut b.2, DT),
            |b| {
                let at = |i: usize| (s.packed[i] as usize / 16, s.packed[i] as usize % 16);
                outcome(n, |i| (b.0[at(i).0][at(i).1], b.1.p[i]), b.2.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        ),
    );

    // Each body found once per contact, not on every touch.
    r.row(
        "once per contact: AoS",
        time(
            || (s.bodies.clone(), s.contacts.clone()),
            |b| solve_at(&AosAt(b.0.as_mut_ptr()), &mut b.1, DT),
            |b| aos_out(b, &identity),
        ),
    );
    r.row(
        "once per contact: 16-row pages as the world fills them, all fields paged",
        time(
            || {
                let (v, inv) = paged16();
                (PagedAll::<16> { v: pages_of(&v), inv: pages_of(&inv), p: pages_of(&vec![Vec2::ZERO; slots16]) }, s.contacts_by(&s.packed))
            },
            |b| solve_at(&PagedAt::of(&mut b.0), &mut b.1, DT),
            |b| {
                let at = |i: usize| (s.packed[i] as usize / 16, s.packed[i] as usize % 16);
                outcome(n, |i| (b.0.v[at(i).0][at(i).1], b.0.p[at(i).0][at(i).1]), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        ),
    );
    r.row(
        "once per contact: 256-row pages, all fields paged",
        time(
            || {
                let (v, inv): (Vec<Vec2>, Vec<f32>) = s.bodies.iter().map(|b| (b.v, b.inv_mass)).unzip();
                (PagedAll::<256> { v: pages_of(&v), inv: pages_of(&inv), p: pages_of(&vec![Vec2::ZERO; n]) }, s.contacts.clone())
            },
            |b| solve_at(&PagedAt::of(&mut b.0), &mut b.1, DT),
            |b| outcome(n, |i| (b.0.v[i / 256][i % 256], b.0.p[i / 256][i % 256]), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );
    r.row(
        "once per contact: a pointer per body into 16-row pages",
        time(
            || {
                let (v, _) = paged16();
                let mut pages: Pages<Vec2, 16> = pages_of(&v);
                let ptrs = s.packed.iter().map(|&k| &mut pages[k as usize / 16][k as usize % 16] as *mut Vec2).collect();
                let mut p = vec![Vec2::ZERO; n];
                let at = PerBodyAt { v: ptrs, inv: s.bodies.iter().map(|b| b.inv_mass).collect(), p: p.as_mut_ptr() };
                (pages, at, s.contacts.clone(), p)
            },
            |b| solve_at(&b.1, &mut b.2, DT),
            |b| {
                let at = |i: usize| (s.packed[i] as usize / 16, s.packed[i] as usize % 16);
                outcome(n, |i| (b.0[at(i).0][at(i).1], b.3[i]), b.2.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        ),
    );

    // (d) order: the same computation with the bodies renumbered. Contacts
    // stay in pair order, so the arithmetic is the same.
    fn ordered(s: &Scene, order: Vec<u32>) -> Variant<'_> {
        // `order[k]` is the body put at index k; `map[i]` where body i went.
        let n = s.bodies.len();
        let mut map = vec![0u32; n];
        for (k, &i) in order.iter().enumerate() {
            map[i as usize] = k as u32;
        }
        let setup_map = map.clone();
        time(
            move || (order.iter().map(|&i| s.bodies[i as usize]).collect::<Vec<_>>(), s.contacts_by(&setup_map)),
            |b| solver::solve(&mut b.0, &mut b.1, DT),
            move |b| {
                let body = |i: usize| (b.0[map[i] as usize].v, b.0[map[i] as usize].pseudo);
                outcome(n, body, b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        )
    }
    let mut by_entity: Vec<u32> = (0..n as u32 - 1).collect();
    by_entity.sort_by_key(|&i| s.entity[i as usize]);
    by_entity.push(n as u32 - 1);
    r.row("(d) AoS, bodies in entity order", ordered(s, by_entity));
    let mut seen = vec![false; n];
    let mut first_touch = Vec::with_capacity(n);
    for c in &s.contacts {
        for i in [c.a, c.b] {
            if !std::mem::replace(&mut seen[i as usize], true) {
                first_touch.push(i);
            }
        }
    }
    first_touch.extend((0..n as u32).filter(|&i| !seen[i as usize]));
    r.row("(d) AoS, bodies in the order contacts first touch them", ordered(s, first_touch));
    let mut shuffled: Vec<u32> = (0..n as u32).collect();
    let mut x = 0x2545_f491_4f6c_dd1du64;
    for i in (1..n).rev() {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        shuffled.swap(i, (x % (i as u64 + 1)) as usize);
    }
    r.row("(d) AoS, bodies shuffled", ordered(s, shuffled));
    // A different computation (Gauss-Seidel in another order), for how much
    // contact order is worth: timed only.
    r.row(
        "(d) AoS, contacts sorted by their bodies' world order (not the same sums)",
        time(
            || {
                let mut c = s.contacts.clone();
                c.sort_by_key(|c| (c.a.min(c.b), c.a.max(c.b)));
                (s.bodies.clone(), c)
            },
            |b| solver::solve(&mut b.0, &mut b.1, DT),
            |b| aos_out(b, &identity),
        ),
    );

    r.row(
        "in place: the world's `Velocity` pages, inv_mass from its `Body` pages",
        time(
            || {
                reset_world(&vptrs);
                (InWorld { v: vptrs.clone(), body: bptrs.clone(), inv: None, p: vec![Vec2::ZERO; slots16] }, s.contacts_by(&s.packed))
            },
            |b| solve(&mut b.0, &mut b.1, DT),
            world_out,
        ),
    );
    r.row(
        "in place: the world's `Velocity` pages, inv_mass in flat scratch",
        time(
            || {
                reset_world(&vptrs);
                let inv = s.scatter(&s.packed, slots16, 0.0, |i| s.bodies[i].inv_mass);
                (InWorld { v: vptrs.clone(), body: bptrs.clone(), inv: Some(inv), p: vec![Vec2::ZERO; slots16] }, s.contacts_by(&s.packed))
            },
            |b| solve(&mut b.0, &mut b.1, DT),
            world_out,
        ),
    );
    r.row(
        "in place: the world's `Velocity` pages, inv_mass in the constraint",
        time(
            || {
                reset_world(&vptrs);
                (InWorld { v: vptrs.clone(), body: bptrs.clone(), inv: None, p: vec![Vec2::ZERO; slots16] }, s.cached_by(&s.packed))
            },
            |b| solve_cached(&mut b.0, &mut b.1, DT),
            |b| outcome(n, |i| (b.0.v(s.packed[i]), b.0.p[s.packed[i] as usize]), b.1.iter().map(|c| (c.jn, c.jt, c.speed))),
        ),
    );

    r.row(
        "once per contact, in place: the world's `Velocity` and `Body` pages",
        time(
            || {
                reset_world(&vptrs);
                let mut p = vec![Vec2::ZERO; slots16];
                let at = WorldAt { v: vptrs.clone(), body: bptrs.clone(), p: p.as_mut_ptr() };
                (at, s.contacts_by(&s.packed), p)
            },
            |b| solve_at(&b.0, &mut b.1, DT),
            |b| {
                let v = |k: u32| unsafe { (*b.0.v[k as usize / SPATIAL_ROWS].add(k as usize % SPATIAL_ROWS)).get() };
                outcome(n, |i| (v(s.packed[i]), b.2[s.packed[i] as usize]), b.1.iter().map(|c| (c.jn, c.jt, c.speed)))
            },
        ),
    );
    println!("µs per solve, the median of {REPS} round robin, -c opt, one thread; Δ over the first row\n");
    r.finish();

    // (e) the copy: what solving in place saves. Gathering walks the pages
    // as `moving.for_each` does; scattering writes velocities back (the
    // mod writes positions too, which an in-place solve would still have
    // to).
    reset_world(&vptrs);
    let mut gathers = Vec::new();
    let mut scatters = Vec::new();
    for _ in 0..REPS {
        let start = Instant::now();
        let mut out: Vec<SolverBody> = Vec::with_capacity(n);
        let mut page = 0;
        for bg in &bguards {
            for bc in bg.iter() {
                let (bs, vs) = (bc.as_slice::<Body>(), vptrs[page]);
                for (r, b) in bs.iter().enumerate() {
                    // SAFETY: as in `InWorld`; `r` is within the page's rows.
                    let v = unsafe { *vs.add(r) };
                    out.push(SolverBody { v: Vec2::new(v.x, v.y), inv_mass: derived(b), pseudo: Vec2::ZERO });
                }
                page += 1;
            }
        }
        out.push(SolverBody::default());
        gathers.push(start.elapsed().as_secs_f64() * 1e6);
        let out = black_box(out);
        let start = Instant::now();
        let mut i = 0;
        for vg in vguards.iter_mut() {
            for vc in vg.iter_mut() {
                for v in vc.as_mut_slice::<Velocity>() {
                    (v.x, v.y) = (out[i].v.x, out[i].v.y);
                    i += 1;
                }
            }
        }
        scatters.push(start.elapsed().as_secs_f64() * 1e6);
    }
    println!("\n(e) the copy, against the world's pages: gathering bodies {:.1} µs, writing velocities back {:.1} µs", median(gathers), median(scatters));

    drop((vguards, bguards));
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
}
