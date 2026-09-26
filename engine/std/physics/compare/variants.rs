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

use std::cell::RefCell;
use std::collections::HashMap;

use physics::Vec2;

use crate::solver::{BOUNCE_THRESHOLD, Constraint, SolverBody};
use crate::split_impulse as old;

pub type Boxed = Box<dyn Fn(&mut [SolverBody], &mut [Constraint], f32)>;

pub fn parse(spec: &str) -> Boxed {
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
        return Box::new(move |b, c, dt| split(&s, &mut ages.borrow_mut(), b, c, dt));
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
        return Box::new(move |b, c, dt| ngs(&s, b, c, dt));
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
    Box::new(move |b, c, dt| soft(&s, b, c, dt))
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
