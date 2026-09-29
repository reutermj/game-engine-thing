//! Other solvers for the arrays, beside `solver::solve`, picked by
//! `VARIANTS=arrays:<spec>`: what the settling work (get-emj.35) tried,
//! kept so its tables in physics.md, "Settling", can be run again.
//!
//! - `split`: the split-impulse solver `solver.rs` replaced
//!   (`tests/split_impulse.rs`); `split/pf=1` adds friction on its pseudo
//!   velocities, limited by the pseudo normal impulse, `pf=2` by that and
//!   the real one; `split/beta=<b>` corrects that share of penetration a
//!   step; `split/persist=<b>` corrects that share instead for a contact
//!   that pressed last step, and `split/decay=<k>` a share of `beta / (1 +
//!   k * age)`, age in steps. `nosplit`: its pseudo velocities thrown away.
//! - `ngs/it=<n>/beta=<b>/slop=<s>/max=<m>`: its velocity iterations, then
//!   position iterations that move bodies apart directly (non-linear
//!   Gauss-Seidel), as Jolt's `SolvePositionConstraint` does (and Box2D
//!   v2.4's `SolvePositionConstraints`); Jolt's defaults but the slop.
//! - `soft/<key>=<value>/...`: `solver::solve`'s soft step with its
//!   constants changed: `sub` substeps, `it` pushing passes and `relax`
//!   rigid ones a substep, `hz` and `static` the stiffness (in hertz; a
//!   static contact's `static` times as stiff), `zeta`, `push`, `slop` (a
//!   depth left unpushed), `bf=1` friction in the pushing pass too, `g=0`
//!   gravity all in the first substep instead of a share in each.
//! - `rot/<key>=<value>/...`: `solver::solve_points` with rotation's
//!   choices changed (physics.md, "Rotation"): `sep` how a point's
//!   separation follows its bodies turning (0 its arms turned, as Box2D; 1
//!   to first order; 2 not at all), `int` how rotation is carried through
//!   the substeps (0 as a rotation, as Box2D; 1 as an angle), `relax`,
//!   `sub`, `warm` how points are warm-started (0 not at all; 1 by feature
//!   id, as Box2D; 2 by the nearest last point, as parry can; 3 by feature
//!   id, and a new feature's by the nearest), `deepest=1`
//!   a contact's deepest point alone, `stiff` and `static` the contacts'
//!   stiffness as a share of the substep rate (`STIFFNESS`,
//!   `STATIC_STIFFNESS`), `block=0` a contact's two points one after the
//!   other in the relax passes; `scalar=1` one contact at a time
//!   (`Wide::Off`), `levels=<n>` by level `n` wide, `colored=<n>`
//!   graph-colored `n` wide (the default, 4; physics.md, "The solver's
//!   speed"); `carry` what a turning contact's points carry to the next step
//!   (0 the last substep's impulses; 1 the normal's last and the tangent's
//!   mean, the default; 2 both means, as before get-emj.48;
//!   `solver::Carry`); `order` the contacts in another order than the
//!   pairs': 1 reversed, 2 shuffled each step, 3 rows from the top down
//!   (physics.md, "Why colors let the pyramid fall"), 4 the colors' order
//!   (`solver::order`), which with `scalar=1` is the default's computation
//!   solved one contact at a time; `closing` what restitution takes a
//!   contact's closing speed from (0 with the step's gravity in it, 1 before
//!   it, 2 with half, 3 as the bodies meet, 4 the rebound less the step's
//!   gravity, 5 with it but gated before it; `solver::Closing`, physics.md,
//!   "Bounces"); `threads=<n>` the default solved across `n` threads
//!   (`solver::solve_across`), which is it bit for bit, and with `late=1`
//!   threads that come one at a time, the last first (`Backwards`).

use std::cell::RefCell;
use std::collections::HashMap;

use physics::Vec2;

use crate::arrays::Warm;
use crate::solver::{
    BOUNCE_THRESHOLD, Carry, Closing, Constraint, Integrate, PARAMS, Points, Separation, SolverBody, Spinning, Wide, order, solve_with,
};
use crate::split_impulse as old;

/// The host's executor as the solver takes it: `rot/threads=<n>` solves
/// across `n` threads spawned for each step (`engine_ecs::Scoped`), which
/// is the solve on one bit for bit (`solver::solve_across`).
impl crate::solver::Gang for engine_ecs::Workers {
    fn threads(&self) -> usize {
        engine_ecs::Workers::threads(self)
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        engine_ecs::Workers::run(self, tasks, f)
    }
}

/// `n` threads spawned for each run, as a solver's `Gang`.
pub fn scoped(n: usize) -> engine_ecs::Workers {
    engine_ecs::Workers::new(Some(std::sync::Arc::new(engine_ecs::Scoped(n))))
}

/// An executor that says it has `n` threads and runs every task on the
/// caller's, one after another, the last first (`rot/threads=<n>/late=1`):
/// a solve across threads whose threads come one at a time, each after the
/// last has left, must still finish, and the same.
struct Backwards(usize);

impl engine_ecs::Executor for Backwards {
    fn threads(&self) -> usize {
        self.0
    }

    fn run(&self, tasks: usize, f: &(dyn Fn(usize) + Sync)) {
        (0..tasks).rev().for_each(f);
    }
}

/// `rot/closing=<n>`: what restitution takes a contact's closing speed from
/// (`solver::Closing`), by its place here.
const CLOSINGS: [Closing; 6] = [Closing::Stepped, Closing::Before, Closing::Half, Closing::Met, Closing::Less, Closing::Gate];

/// A solver for the arrays: over bodies some of which may turn and
/// contacts some of which may have points, which only
/// `solver::solve_points` and its `rot` variants solve.
pub type Boxed = Box<dyn Fn(&mut [SolverBody], &mut [Spinning], &mut [Constraint], &mut [Points], f32)>;

/// A solver without rotation, refusing a scene where something turns.
fn without_points(f: impl Fn(&mut [SolverBody], &mut [Constraint], f32) + 'static) -> Boxed {
    Box::new(move |b, s, c, p, dt| {
        assert!(p.is_empty() && s.is_empty(), "this variant has no rotation");
        f(b, c, dt)
    })
}

/// A variant: its solver, and how the arrays feed it: how contact points
/// are warm-started, and whether a contact keeps only its deepest point.
pub struct Variant {
    pub solve: Boxed,
    pub warm: Warm,
    pub deepest: bool,
}

impl Variant {
    fn of(solve: Boxed) -> Variant {
        Variant { solve, warm: Warm::Ids, deepest: false }
    }
}

pub fn parse(spec: &str) -> Variant {
    let mut parts = spec.split("/");
    let name = parts.next().unwrap();
    let kvs: Vec<(String, f32)> = parts
        .map(|kv| {
            let (k, v) = kv.split_once("=").expect("key=value");
            (k.to_string(), v.parse().expect("a number"))
        })
        .collect();
    if name == "split" || name == "nosplit" {
        let mut s = Split { beta: old::BETA, friction: 0, keep: name == "split", persist: None, decay: 0.0 };
        for (k, v) in kvs {
            match k.as_str() {
                "beta" => s.beta = v,
                "pf" => s.friction = v as u32,
                "persist" => s.persist = Some(v),
                "decay" => s.decay = v,
                _ => panic!("split: {k}"),
            }
        }
        let ages = RefCell::new(HashMap::new());
        return Variant::of(without_points(move |b, c, dt| split(&s, &mut ages.borrow_mut(), b, c, dt)));
    }
    if name == "ngs" {
        let mut s = Ngs { iterations: 2, beta: 0.2, slop: old::SLOP, max: 0.2 };
        for (k, v) in kvs {
            match k.as_str() {
                "it" => s.iterations = v as u32,
                "beta" => s.beta = v,
                "slop" => s.slop = v,
                "max" => s.max = v,
                _ => panic!("ngs: {k}"),
            }
        }
        return Variant::of(without_points(move |b, c, dt| ngs(&s, b, c, dt)));
    }
    if name == "rot" {
        let mut params = PARAMS;
        let mut order = 0;
        let mut threads = 0;
        let mut late = false;
        let mut v = Variant::of(Box::new(|_, _, _, _, _| {}));
        for (k, x) in kvs {
            match k.as_str() {
                "sep" => params.separation = [Separation::Turned, Separation::Linear, Separation::Fixed][x as usize],
                "int" => params.integrate = [Integrate::Rotation, Integrate::Angle][x as usize],
                "relax" => params.relax = x as usize,
                "sub" => params.substeps = x as usize,
                "stiff" => params.stiffness = x,
                "static" => params.static_stiffness = x,
                "block" => params.block = x != 0.0,
                "scalar" if x != 0.0 => params.wide = Wide::Off,
                "levels" => params.wide = Wide::Levels(x as usize),
                "colored" => params.wide = Wide::Colored(x as usize),
                "warm" => v.warm = [Warm::None, Warm::Ids, Warm::Nearest, Warm::Either][x as usize],
                "deepest" => v.deepest = x != 0.0,
                "order" => order = x as u32,
                "carry" => params.carry = [Carry::Last, Carry::Normal, Carry::Mean][x as usize],
                "closing" => params.closing = CLOSINGS[x as usize],
                "threads" => threads = x as usize,
                "late" => late = x != 0.0,
                _ => panic!("rot: {k}"),
            }
        }
        if threads > 0 {
            assert_eq!(order, 0, "rot/threads solves in the colors' order");
            let gang = if late { engine_ecs::Workers::new(Some(std::sync::Arc::new(Backwards(threads)))) } else { scoped(threads) };
            v.solve = Box::new(move |b, s, c, p, dt| crate::solver::solve_across(&params, (b, s), c, p, dt, &gang));
        } else if order == 0 {
            v.solve = Box::new(move |b, s, c, p, dt| solve_with(&params, (b, s), c, p, dt));
        } else {
            let seed = RefCell::new(0x9e37_79b9_u32);
            v.solve = Box::new(move |b, s, c, p, dt| {
                let at = match order {
                    4 => solver_order(b, s, c),
                    _ => shuffled(order, &mut seed.borrow_mut(), c),
                };
                reordered(&at, c, |c| solve_with(&params, (b, s), c, p, dt))
            });
        }
        return v;
    }
    assert_eq!(name, "soft", "no variant {name}");
    let mut s = Soft::default();
    for (k, v) in kvs {
        match k.as_str() {
            "sub" => s.sub = v as u32,
            "it" => s.iters = v as u32,
            "relax" => s.relax = v as u32,
            "hz" => s.hz = Some(v),
            "zeta" => s.zeta = v,
            "static" => s.static_mul = v,
            "push" => s.push = v,
            "slop" => s.slop = v,
            "bf" => s.bias_friction = v != 0.0,
            "g" => s.spread_gravity = v != 0.0,
            _ => panic!("soft: {k}"),
        }
    }
    Variant::of(without_points(move |b, c, dt| soft(&s, b, c, dt)))
}

/// The colors' order (`rot/order=4`), as the default solve groups these
/// contacts.
fn solver_order(bodies: &[SolverBody], spinning: &[Spinning], contacts: &[Constraint]) -> Vec<usize> {
    order(Wide::Colored(4), bodies, spinning, contacts)
}

/// Another order than the pairs' (`rot/order`): 1 reversed, 2 shuffled
/// afresh each step, 3 by their lower body, highest first (a pyramid's rows
/// from the top down, each in pair order, and the ground last: by the
/// higher body, the arrays' immovable body, which is last, would put the
/// ground first).
fn shuffled(order: u32, seed: &mut u32, contacts: &[Constraint]) -> Vec<usize> {
    let mut at: Vec<usize> = (0..contacts.len()).collect();
    match order {
        1 => at.reverse(),
        2 => {
            for i in (1..at.len()).rev() {
                // xorshift32: any fixed sequence will do.
                *seed ^= *seed << 13;
                *seed ^= *seed >> 17;
                *seed ^= *seed << 5;
                at.swap(i, *seed as usize % (i + 1));
            }
        }
        3 => at.sort_by_key(|&i| std::cmp::Reverse(contacts[i].a.min(contacts[i].b))),
        _ => panic!("rot/order={order}"),
    }
    at
}

/// `solve` over the contacts in the order `at`, and back in pair order
/// after, so the arrays see no difference but the solve's. Solved one at a
/// time (`scalar=1`) this is that order's sweep; grouped, it is the groups
/// of that order (levels: the same sweep, bit for bit).
fn reordered(at: &[usize], contacts: &mut [Constraint], solve: impl FnOnce(&mut [Constraint])) {
    let mut mine: Vec<Constraint> = at.iter().map(|&i| contacts[i]).collect();
    solve(&mut mine);
    for (&i, c) in at.iter().zip(mine) {
        contacts[i] = c;
    }
}

// ---- The split impulse, as it was, and with friction on its pseudo velocities ----

#[derive(Clone, Copy)]
struct Split {
    beta: f32,
    /// 0: none; 1: limited by the pseudo normal impulse; 2: by it and the
    /// real one.
    friction: u32,
    /// Whether the pseudo velocities move bodies (`nosplit` throws them away).
    keep: bool,
    persist: Option<f32>,
    decay: f32,
}

fn split(s: &Split, ages: &mut HashMap<(u32, u32), u32>, bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    let mut ob: Vec<old::SolverBody> = bodies.iter().map(|b| old::SolverBody::new(b.v, b.inv_mass, b.gravity)).collect();
    let mut oc: Vec<old::Constraint> = contacts
        .iter()
        .map(|c| old::Constraint {
            a: c.a,
            b: c.b,
            normal: c.normal,
            depth: c.depth,
            friction: c.friction,
            restitution: c.restitution,
            jn: c.jn,
            jt: c.jt,
            speed: 0.0,
        })
        .collect();
    old::solve(&mut ob, &mut oc, dt);
    // Each contact's age: steps it has been found in a row.
    let mut aged = HashMap::with_capacity(oc.len());
    let betas: Vec<f32> = oc
        .iter()
        .map(|c| {
            let age = ages.get(&(c.a, c.b)).map_or(0, |a| a + 1);
            aged.insert((c.a, c.b), age);
            match s.persist {
                Some(b) if c.jn > 0.0 => b,
                _ => s.beta / (1.0 + s.decay * age as f32),
            }
        })
        .collect();
    *ages = aged;
    if s.friction != 0 || s.beta != old::BETA || s.persist.is_some() || s.decay != 0.0 {
        pseudo(s, &mut ob, &oc, &betas, dt);
    }
    for (b, o) in bodies.iter_mut().zip(&ob) {
        b.v = o.v;
        b.moved = if s.keep { o.displacement(dt) } else { o.v * dt };
    }
    for (c, o) in contacts.iter_mut().zip(&oc) {
        c.jn = o.jn;
        c.jt = o.jt;
        c.speed = o.speed;
    }
}

/// The split impulse's pseudo velocities again, with `s`'s settings.
fn pseudo(s: &Split, bodies: &mut [old::SolverBody], contacts: &[old::Constraint], betas: &[f32], dt: f32) {
    for b in bodies.iter_mut() {
        b.pseudo = Vec2::ZERO;
    }
    let mut pj = vec![0.0f32; contacts.len()];
    let mut pt = vec![0.0f32; contacts.len()];
    for _ in 0..old::ITERATIONS {
        for (((c, pj), pt), beta) in contacts.iter().zip(&mut pj).zip(&mut pt).zip(betas) {
            let (a, b) = (c.a as usize, c.b as usize);
            let k = bodies[a].inv_mass + bodies[b].inv_mass;
            if k == 0.0 {
                continue;
            }
            if c.depth > old::SLOP {
                let bias = beta * (c.depth - old::SLOP) / dt;
                let rel = bodies[b].pseudo - bodies[a].pseudo;
                let new = (*pj + (bias - rel.dot(c.normal)) / k).max(0.0);
                let d = new - *pj;
                *pj = new;
                push(bodies, a, b, c.normal * d);
            }
            if s.friction == 0 {
                continue;
            }
            let limit = c.friction * if s.friction == 1 { *pj } else { *pj + c.jn };
            if limit == 0.0 {
                continue;
            }
            let t = c.normal.perp();
            let rel = bodies[b].pseudo - bodies[a].pseudo;
            let new = (*pt - rel.dot(t) / k).clamp(-limit, limit);
            let d = new - *pt;
            *pt = new;
            push(bodies, a, b, t * d);
        }
    }
}

fn push(bodies: &mut [old::SolverBody], a: usize, b: usize, impulse: Vec2) {
    let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
    bodies[a].pseudo -= impulse * ia;
    bodies[b].pseudo += impulse * ib;
}

// ---- Position iterations (non-linear Gauss-Seidel) ----

#[derive(Clone, Copy)]
struct Ngs {
    iterations: u32,
    beta: f32,
    slop: f32,
    max: f32,
}

/// The split-impulse solver's velocity iterations, its pseudo velocities
/// thrown away, then position iterations over where the bodies got to:
/// Jolt's `sSolvePositionConstraint`, translation only.
fn ngs(s: &Ngs, bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    let velocity = Split { beta: old::BETA, friction: 0, keep: false, persist: None, decay: 0.0 };
    split(&velocity, &mut HashMap::new(), bodies, contacts, dt);
    for _ in 0..s.iterations {
        for c in contacts.iter() {
            let (a, b) = (c.a as usize, c.b as usize);
            let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
            let k = ia + ib;
            if k == 0.0 {
                continue;
            }
            let sep = (-c.depth + (bodies[b].moved - bodies[a].moved).dot(c.normal) + s.slop).max(-s.max);
            if sep >= 0.0 {
                continue;
            }
            let lambda = -s.beta * sep / k;
            bodies[a].moved -= c.normal * (lambda * ia);
            bodies[b].moved += c.normal * (lambda * ib);
        }
    }
}

// ---- The soft step, with its constants changed ----

#[derive(Clone, Copy, Debug)]
struct Soft {
    sub: u32,
    iters: u32,
    relax: u32,
    /// The stiffness between moving bodies: `solver::STIFFNESS` of the
    /// substep rate if not given.
    hz: Option<f32>,
    zeta: f32,
    static_mul: f32,
    push: f32,
    slop: f32,
    bias_friction: bool,
    spread_gravity: bool,
}

impl Default for Soft {
    fn default() -> Soft {
        use crate::solver::*;
        Soft {
            sub: SUBSTEPS as u32,
            iters: 1,
            relax: RELAX_ITERATIONS as u32,
            hz: None,
            zeta: DAMPING_RATIO,
            static_mul: STATIC_STIFFNESS / STIFFNESS,
            push: MAX_PUSH,
            slop: 0.0,
            bias_friction: false,
            spread_gravity: true,
        }
    }
}

#[derive(Clone, Copy)]
struct Softness {
    rate: f32,
    mass: f32,
    impulse: f32,
}

fn softness(hz: f32, zeta: f32, h: f32) -> Softness {
    let omega = 2.0 * std::f32::consts::PI * hz;
    let a1 = 2.0 * zeta + h * omega;
    let a2 = h * omega * a1;
    let a3 = 1.0 / (1.0 + a2);
    Softness { rate: omega / a1, mass: a2 * a3, impulse: a3 }
}

/// `solver::solve` written plainly, every constant a setting: with the
/// defaults the same computation, but not bit for bit.
fn soft(s: &Soft, bodies: &mut [SolverBody], contacts: &mut [Constraint], dt: f32) {
    let sub = s.sub.max(1);
    let h = dt / sub as f32;
    let inv_h = 1.0 / h;
    let hz = s.hz.unwrap_or(crate::solver::STIFFNESS * inv_h);
    let moving_soft = softness(hz, s.zeta, h);
    let static_soft = softness(s.static_mul * hz, s.zeta, h);
    let n = contacts.len();
    let mut total = vec![0.0f32; n];
    let mut total_t = vec![0.0f32; n];
    let share = if s.spread_gravity { 1.0 / sub as f32 } else { 1.0 };
    let mut lam: Vec<f32> = contacts.iter().map(|c| c.jn * share).collect();
    let mut lam_t: Vec<f32> = contacts.iter().map(|c| c.jt * share).collect();
    for c in contacts.iter_mut() {
        c.speed = -(bodies[c.b as usize].v - bodies[c.a as usize].v).dot(c.normal);
    }
    if s.spread_gravity {
        for b in bodies.iter_mut() {
            b.v -= b.gravity;
        }
    }
    let mut dp = vec![Vec2::ZERO; bodies.len()];
    for step in 0..sub {
        if s.spread_gravity {
            for b in bodies.iter_mut() {
                b.v += b.gravity * share;
            }
        } else if step > 0 {
            // Gravity came all at once, and last step's impulse with it.
            lam.iter_mut().chain(lam_t.iter_mut()).for_each(|l| *l = 0.0);
        }
        if s.spread_gravity || step == 0 {
            for (i, c) in contacts.iter().enumerate() {
                let j = c.normal * lam[i] + c.normal.perp() * lam_t[i];
                let (a, b) = (c.a as usize, c.b as usize);
                let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
                bodies[a].v -= j * ia;
                bodies[b].v += j * ib;
            }
        }
        let pass = |bodies: &mut [SolverBody], lam: &mut [f32], lam_t: &mut [f32], dp: &[Vec2], bias_on: bool| {
            for (i, c) in contacts.iter().enumerate() {
                let (a, b) = (c.a as usize, c.b as usize);
                let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
                let k = ia + ib;
                if k == 0.0 {
                    continue;
                }
                let m = 1.0 / k;
                let sep = -c.depth + (dp[b] - dp[a]).dot(c.normal);
                let (mut bias, mut ms, mut is) = (0.0, 1.0, 0.0);
                if sep > 0.0 {
                    bias = sep * inv_h;
                } else if bias_on {
                    let sf = if ia == 0.0 || ib == 0.0 { static_soft } else { moving_soft };
                    bias = (sf.rate * (sep + s.slop).min(0.0)).max(-s.push);
                    ms = sf.mass;
                    is = sf.impulse;
                }
                let vn = (bodies[b].v - bodies[a].v).dot(c.normal);
                let imp = -m * ms * (vn + bias) - is * lam[i];
                let new = (lam[i] + imp).max(0.0);
                let d = new - lam[i];
                lam[i] = new;
                let p = c.normal * d;
                bodies[a].v -= p * ia;
                bodies[b].v += p * ib;
                if bias_on && !s.bias_friction {
                    continue;
                }
                let t = c.normal.perp();
                let vt = (bodies[b].v - bodies[a].v).dot(t);
                let limit = c.friction * lam[i];
                let new = (lam_t[i] - m * vt).clamp(-limit, limit);
                let d = new - lam_t[i];
                lam_t[i] = new;
                let p = t * d;
                bodies[a].v -= p * ia;
                bodies[b].v += p * ib;
            }
        };
        for _ in 0..s.iters {
            pass(bodies, &mut lam, &mut lam_t, &dp, true);
        }
        for (d, b) in dp.iter_mut().zip(bodies.iter()) {
            *d += b.v * h;
        }
        for _ in 0..s.relax {
            pass(bodies, &mut lam, &mut lam_t, &dp, false);
        }
        for i in 0..n {
            total[i] += lam[i];
            total_t[i] += lam_t[i];
        }
    }
    for (i, c) in contacts.iter().enumerate() {
        if c.restitution == 0.0 || c.speed <= BOUNCE_THRESHOLD || total[i] == 0.0 {
            continue;
        }
        let (a, b) = (c.a as usize, c.b as usize);
        let (ia, ib) = (bodies[a].inv_mass, bodies[b].inv_mass);
        let k = ia + ib;
        if k == 0.0 {
            continue;
        }
        let vn = (bodies[b].v - bodies[a].v).dot(c.normal);
        let new = (lam[i] - (vn - c.restitution * c.speed) / k).max(0.0);
        let d = new - lam[i];
        total[i] += d;
        let p = c.normal * d;
        bodies[a].v -= p * ia;
        bodies[b].v += p * ib;
    }
    for (i, c) in contacts.iter_mut().enumerate() {
        c.jn = total[i];
        c.jt = total_t[i];
    }
    for (b, d) in bodies.iter_mut().zip(&dp) {
        b.moved = *d;
    }
}
