//! SPIKE (docs/architecture/dispatch-spike.md, get-znt.30): physics3d's
//! solve as its pipeline runs it (`prepare`, `passes`, `finish`:
//! pipeline.rs), colored, with its passes handed to a dispatcher
//! (physics2d/compare/dispatch.rs) instead of `Passes::run`. The kernels
//! are the mod's (`solver::lanes`), unchanged. 3D has no arrays to capture
//! inputs from, so they're gathered from pile3d's world between steps, as
//! the pipeline's sources gather them: the contacts the last narrowphase
//! found, the bodies as the step left them. A step's real input but for
//! the next narrowphase's refresh, which a dispatcher doesn't see.

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use engine_ecs::shape::{Stage, States};
use engine_loader::engine::Engine;
use physics3d::{AngularVelocity, Body, ContactPair, Gravity, Impulse, Manifold, Rotation, Vec3, Velocity};

use crate::dispatch::{self, Plant, Prepared, Protocol, Trace};
use crate::solver::lanes::{Shared, Staged, State, Step};
use crate::solver::{self, Constraint, ContactPoint, SolverBody};

/// The step, as lockstep's.
pub const DT: f32 = 1.0 / 60.0;
const LANES: usize = 4;

#[derive(Clone)]
pub struct Input {
    pub bodies: Vec<SolverBody>,
    pub contacts: Vec<Constraint>,
}

/// Every value a solve leaves, as text that is equal where the bits are
/// (`Debug` prints an `f32` as the shortest text that reads back to it, a
/// negative zero as `-0.0`): exact.rs's way.
pub fn bits(i: &Input) -> String {
    format!("{:?}{:?}", i.bodies, i.contacts)
}

/// pile3d's `build` (`what`, as step_bench sends it) stepped to each of
/// `at`, the solver's input gathered at each.
pub fn capture(what: &str, at: &[u32]) -> Vec<Input> {
    static RUNS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let manifest = engine_control::read_manifest(&std::env::var("PILE3D").expect("PILE3D")).unwrap();
    let base = std::env::var("TEST_TMPDIR").map(PathBuf::from).unwrap_or_else(|_| std::env::temp_dir());
    let k = RUNS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let dir = base.join(format!("physics3d-dispatch-{}-{k}", std::process::id()));
    let e = Engine::new(manifest.bootstrap.clone(), dir.clone());
    e.load_batch(&manifest.mods).expect("loading pile3d");
    e.send("pile3d", &format!("build {what}")).unwrap();
    let mut now = 0;
    let mut out = Vec::new();
    for &step in at {
        e.send("lockstep", &format!("step {}", step - now)).unwrap();
        now = step;
        out.push(gather(&e));
    }
    drop(e);
    let _ = std::fs::remove_dir_all(dir);
    out
}

/// The pipeline's `gather_bodies` and `gather_contacts`, over the world's
/// values: moving bodies in entity order, then one standing for every
/// static; contacts in pair order.
fn gather(e: &Engine) -> Input {
    let w = e.world();
    fn map<T>(v: Vec<(engine_ecs::Entity, T)>) -> HashMap<engine_ecs::Entity, T> {
        v.into_iter().collect()
    }
    let body: HashMap<_, Body> = map(w.values::<Body>().unwrap_or_default());
    let spin: HashMap<_, AngularVelocity> = map(w.values::<AngularVelocity>().unwrap_or_default());
    let rot: HashMap<_, Rotation> = map(w.values::<Rotation>().unwrap_or_default());
    let g = w.values::<Gravity>().unwrap_or_default().first().map_or(Vec3::ZERO, |(_, g)| g.vec());
    let mut moving: Vec<(engine_ecs::Entity, Velocity)> =
        w.values::<Velocity>().unwrap_or_default().into_iter().filter(|(e, _)| body.contains_key(e)).collect();
    moving.sort_by_key(|(e, _)| *e);
    let index: HashMap<_, u32> = moving.iter().enumerate().map(|(k, (e, _))| (*e, k as u32)).collect();
    let mut bodies: Vec<SolverBody> = moving
        .iter()
        .map(|(e, v)| {
            let b = &body[e];
            let gravity = if b.inv_mass > 0.0 { g * DT } else { Vec3::ZERO };
            let (v, w) = (Vec3::new(v.x, v.y, v.z), spin.get(e).map_or(Vec3::ZERO, |w| Vec3::new(w.x, w.y, w.z)));
            SolverBody::new(v, w, b.inv_mass, b.inv_inertia(), rot[e].quat(), gravity)
        })
        .collect();
    bodies.push(SolverBody::default());
    let still = moving.len() as u32;
    let manifold: HashMap<_, Manifold> = map(w.values::<Manifold>().unwrap_or_default());
    let impulse: HashMap<_, Impulse> = map(w.values::<Impulse>().unwrap_or_default());
    let mut pairs = w.values::<ContactPair>().unwrap_or_default();
    pairs.sort_by_key(|(_, p)| (p.a, p.b));
    let contacts = pairs
        .iter()
        .map(|(c, pair)| {
            let (m, j) = (&manifold[c], &impulse[c]);
            let mut points = [ContactPoint::default(); crate::MAX_POINTS];
            for (k, p) in points[..m.count as usize].iter_mut().enumerate() {
                let (ra, depth) = m.point(k);
                *p = ContactPoint { ra, depth, jn: j.normal[k], speed: 0.0 };
            }
            Constraint {
                a: index.get(&pair.a).copied().unwrap_or(still),
                b: index.get(&pair.b).copied().unwrap_or(still),
                normal: m.normal(),
                offset: m.offset(),
                friction: m.friction,
                restitution: m.restitution,
                count: m.count as usize,
                points,
                jt: Vec3::new(j.tx, j.ty, j.tz),
                twist: j.twist,
            }
        })
        .collect();
    Input { bodies, contacts }
}

#[derive(Clone, Copy)]
pub enum How<'a> {
    One,
    Across(Protocol, &'a dyn engine_ecs::Executor, Plant),
}

/// What the pipeline keeps between steps: the graph flow's payload.
#[derive(Default)]
pub struct Solve {
    staged: Staged<LANES>,
    program: Vec<Stage<Step>>,
}

impl Solve {
    /// `prepare`: false where the lanes don't take the step (and then
    /// `finish` solves it whole, one contact at a time, on one thread),
    /// which includes a step that isn't shareable (`lanes::shareable`,
    /// get-znt.39's condition, checked by the mod's own `prepare`).
    pub fn prepare(&mut self, i: &mut Input) -> bool {
        let how = solver::Tuning::default();
        if !self.staged.prepare(&mut i.bodies, &mut i.contacts, DT, &how) {
            return false;
        }
        self.staged.program(&mut self.program);
        true
    }

    pub fn passes(&mut self, i: &Input, how: How, trace: Option<&Trace>) {
        let (layout, mut items, states, kernels) = self.staged.split(&i.bodies);
        let program = &self.program;
        match how {
            How::One => dispatch::one_thread(
                layout,
                &mut items,
                states,
                program,
                &|k, block, s| match s {
                    States::Plain(s) => kernels.block(k, block, s),
                    States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
                },
                &|k, r, s| match s {
                    States::Plain(s) => kernels.each(k, r, s),
                    States::Shared(s) => kernels.each(k, r, &mut Shared(s)),
                },
            ),
            How::Across(protocol, exec, plant) => dispatch::passes(
                (protocol, exec, true, plant),
                layout,
                &mut items,
                states,
                program,
                |k, block, s| match s {
                    States::Plain(s) => kernels.block(k, block, s),
                    States::Shared(s) => kernels.block(k, block, &mut Shared(s)),
                },
                |k, r, s| match s {
                    States::Plain(s) => kernels.each(k, r, s),
                    States::Shared(s) => kernels.each(k, r, &mut Shared(s)),
                },
                trace,
            ),
        }
    }

    /// The whole solve, the pipeline's way: µs of prepare, passes, finish.
    pub fn solve(&mut self, i: &mut Input, how: How, trace: Option<&Trace>) -> [f64; 3] {
        let t0 = Instant::now();
        if !self.prepare(i) {
            solver::in_order(&mut i.bodies, &mut i.contacts, DT, &solver::Tuning::default());
            return [0.0, 0.0, t0.elapsed().as_secs_f64() * 1e6];
        }
        let t1 = Instant::now();
        self.passes(i, how, trace);
        let t2 = Instant::now();
        self.staged.finish(&mut i.bodies, &mut i.contacts);
        let us = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1e6;
        [us(t0, t1), us(t1, t2), us(t2, Instant::now())]
    }

    /// Each stage's blocks as a dispatcher at `threads` lays them out.
    pub fn stage_blocks(&mut self, i: &Input, threads: usize) -> Vec<usize> {
        let (layout, mut items, states, _) = self.staged.split(&i.bodies);
        let p: Prepared<'_, _, State, Step> = Prepared::new(layout, &mut items, states, &self.program, threads);
        let run = |_: usize, _: usize| {};
        p.program(&run).stages.iter().map(|s| s.0).collect()
    }

    /// Groups, the overflow's batches, all batches, the widest and the
    /// narrowest group's.
    pub fn layout(&self) -> [usize; 5] {
        self.staged.layout()
    }
}
