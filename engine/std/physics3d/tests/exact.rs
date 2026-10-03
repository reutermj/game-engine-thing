//! physics3d's exact fingerprint (physics-testing.md, "The exact
//! fingerprint"): what `exact_test` checks against `exact.txt`, and what
//! `:exact` prints and writes. Two layers, so a change says which it was:
//!
//! - **The mod**: pile3d's scene stepped in the engine, every frame's
//!   bodies and contacts hashed by component, so the first frame and the
//!   first component that moved say where a change went in (a manifold
//!   before a velocity is the narrowphase; a position alone, the
//!   write-back).
//! - **The kernel**: `solver::solve` alone, on bodies and contacts made
//!   here from a seed (`kernel_inputs`), at each of its tunings. No
//!   narrowphase, gather or world in it, so it moves when the arithmetic
//!   does, and when the order the solver takes contacts in does: colored
//!   (`order=colored`, get-emj.90), every kernel line moves.
//!
//! 3D has no second implementation to hold the mod to bit for bit, as 2D
//! has its arrays: a pinned value is the other side. Both scenes avoid
//! libm (sines, powers), so what they pin is the code, the compiler and
//! its flags, not the host's C library: `sqrt` is exact by IEEE 754.

use std::fmt::Debug;
use std::path::PathBuf;

use engine_ecs::Component;
use engine_loader::engine::Engine;
use physics3d::{AngularVelocity, Impulse, Manifold, Position, Rotation, Velocity};

use crate::solver::{self, Constraint, ContactPoint, SolverBody};
use crate::{MAX_POINTS, Quat, Tuning, Vec3};

/// The step, as lockstep's.
pub const DT: f32 = 1.0 / 60.0;

/// The pinned fingerprint, as the tool writes it.
pub const PINNED: &str = include_str!("exact.txt");
/// Where it is in the workspace, for `--write`.
pub const PATH: &str = "engine/std/physics3d/tests/exact.txt";

/// FNV-1a, 64 bits: a hash nothing outside this file has to agree with,
/// and no crate to pin.
#[derive(Clone, Copy)]
struct Fnv(u64);

impl Fnv {
    fn new() -> Fnv {
        Fnv(0xcbf2_9ce4_8422_2325)
    }

    /// Hashed through `Debug`, which prints an `f32` as the shortest text
    /// that reads back to the same bits, so equal text is equal bits (a
    /// NaN's payload aside, which no body should carry).
    fn add(&mut self, x: &impl Debug) {
        for b in format!("{x:?}").bytes() {
            self.0 = (self.0 ^ b as u64).wrapping_mul(0x0100_0000_01b3);
        }
    }
}

/// The mod's scene: a mixed pile (spheres, and boxes whose three half
/// extents differ) dropped free to turn, then a plank and a cube thrown in
/// spinning, so they land on edges and corners (contacts of one to four
/// points), a sphere spun on to roll, and, locked, a box that can't turn.
/// Spins rather than `turn`, which takes a sine. No kinematic body: 3D has
/// none (lib.rs). 44 bodies for 120 frames: well under a second.
const SCENE: &[Step] = &[
    Step::Send("build mixed 40"),
    Step::Frames(40),
    Step::Send("body box 0.5 0.125 0.25 at 0.3 6 -0.2 v 0.5 -1 0 w 4 -2 3"),
    Step::Send("body box 0.5 0.5 0.5 at -0.6 7.5 0.5 v 0 -2 0 w 1 7 -5"),
    Step::Send("body sphere 0.4 at 0.2 9 0.6 v 1 0 0 w 0 0 -8"),
    Step::Send("lock"),
    Step::Send("body box 0.5 0.5 0.5 at 0.5 10.5 -0.5 v 0 -3 0"),
    Step::Frames(80),
];

pub enum Step {
    Send(&'static str),
    Frames(u32),
}

/// What the mod's run saw besides its hashes, for the test to check the
/// scene is what it says: how many contacts had each count of points over
/// the frames, the locked box's turn rate at the end, and how many bodies
/// had turned by then.
#[derive(Debug, Default)]
pub struct Seen {
    pub points: [u32; MAX_POINTS + 1],
    pub locked_w: [f32; 3],
    pub turned: usize,
}

/// One frame's hash of every value of `T`, by entity.
fn hash<T: Component + Clone + Debug>(e: &Engine) -> u64 {
    let mut all = e.world().values::<T>().unwrap_or_default();
    all.sort_by_key(|(e, _)| *e);
    let mut h = Fnv::new();
    for (e, v) in &all {
        h.add(e);
        h.add(v);
    }
    h.0
}

/// How the mod's scene is run besides the pinned way: a `Tuning` written
/// into its world first (`pile3d tune`), and the passes handed their
/// states shared (`World::set_shapes_shared`), as threads will.
#[derive(Clone, Copy, Debug, Default)]
pub struct Run {
    pub tune: &'static str,
    pub shared: bool,
}

/// The mod's lines, one a frame, and what it saw: as pinned at
/// `Run::default()`.
pub fn the_mod(run: Run) -> (Vec<String>, Seen) {
    static RUNS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let manifest = engine_control::read_manifest(&std::env::var("PILE3D").expect("PILE3D")).unwrap();
    let base = std::env::var("TEST_TMPDIR").map(PathBuf::from).unwrap_or_else(|_| std::env::temp_dir());
    let k = RUNS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let dir = base.join(format!("physics3d-exact-{}-{k}", std::process::id()));
    let engine = Engine::new(manifest.bootstrap.clone(), dir.clone());
    engine.load_batch(&manifest.mods).expect("loading pile3d");
    let send = |to: &str, m: &str| engine.send(to, m).unwrap_or_else(|err| panic!("{to} {m:?}: {err}"));
    if !run.tune.is_empty() {
        send("pile3d", &format!("tune {}", run.tune));
    }
    engine.world().set_shapes_shared(run.shared);
    let (mut lines, mut seen, mut frame) = (Vec::new(), Seen::default(), 0);
    let mut locked = None;
    for step in SCENE {
        match step {
            Step::Send(m) => {
                // The last body sent is the locked box.
                if let Some(e) = send("pile3d", m).strip_prefix("body ") {
                    locked = Some(e.to_string());
                }
            }
            Step::Frames(n) => {
                for _ in 0..*n {
                    send("lockstep", "step 1");
                    frame += 1;
                    lines.push(format!(
                        "mod {frame} position={:016x} rotation={:016x} velocity={:016x} angular={:016x} manifold={:016x} impulse={:016x}",
                        hash::<Position>(&engine),
                        hash::<Rotation>(&engine),
                        hash::<Velocity>(&engine),
                        hash::<AngularVelocity>(&engine),
                        hash::<Manifold>(&engine),
                        hash::<Impulse>(&engine),
                    ));
                    for (_, m) in engine.world().values::<Manifold>().unwrap_or_default() {
                        seen.points[m.count as usize] += 1;
                    }
                }
            }
        }
    }
    let locked = locked.expect("a body sent");
    for (e, w) in engine.world().values::<AngularVelocity>().unwrap_or_default() {
        if format!("{}:{}", e.index, e.generation) == locked {
            seen.locked_w = [w.x, w.y, w.z];
        }
    }
    seen.turned = engine.world().values::<Rotation>().unwrap_or_default().iter().filter(|(_, q)| q.quat() != Quat::IDENTITY).count();
    drop(engine);
    let _ = std::fs::remove_dir_all(&dir);
    (lines, seen)
}

/// splitmix64, as scenes.rs's `Rng`: enough randomness for inputs.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in [lo, hi).
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * ((self.next() >> 40) as f32 / (1u64 << 24) as f32)
    }

    fn vec(&mut self, r: f32) -> Vec3 {
        Vec3::new(self.range(-r, r), self.range(-r, r), self.range(-r, r))
    }

    /// A unit vector, from a cube's draw normalized (no sines).
    fn unit(&mut self) -> Vec3 {
        loop {
            let v = self.vec(1.0);
            if v.len() > 0.1 {
                return v.normalize();
            }
        }
    }
}

/// Moving bodies in the kernel's inputs; one more stands for the statics.
pub const KERNEL_BODIES: usize = 24;

/// The kernel's inputs: bodies and contacts as the gather makes them,
/// from a seed, in pair order. Everything a pass branches on happens in
/// them: moving pairs and pairs with the static, a body that can't turn, a
/// heavy one, contacts of one to four points, gaps (speculative) and
/// overlaps past the push's limit, warm impulses and none, friction
/// against fast sliding and twisting, and closing speeds over
/// `BOUNCE_THRESHOLD` on contacts that bounce. Not physically consistent
/// (no shape has these points), which no kernel needs to be: a kernel in
/// lanes (get-emj.52) must be `solver::solve` bit for bit on any input,
/// and these are the inputs to hold it to.
pub fn kernel_inputs() -> (Vec<SolverBody>, Vec<Constraint>) {
    let mut rng = Rng(0x3d_e4ac7);
    let mut bodies: Vec<SolverBody> = (0..KERNEL_BODIES)
        .map(|i| {
            let mass = if i == 5 { 40.0 } else { rng.range(0.5, 4.0) };
            let inertia = if i == 3 { Vec3::ZERO } else { Vec3::new(rng.range(1.0, 12.0), rng.range(1.0, 12.0), rng.range(1.0, 12.0)) };
            let q = loop {
                let (v, w) = (rng.vec(1.0), rng.range(-1.0, 1.0));
                let n = (v.dot(v) + w * w).sqrt();
                if n > 0.1 {
                    break Quat { v: v * (1.0 / n), w: w / n };
                }
            };
            let gravity = Vec3::new(0.0, -9.81 * DT, 0.0);
            SolverBody::new(rng.vec(3.0) + gravity, rng.vec(4.0), 1.0 / mass, inertia * (1.0 / mass), q, gravity)
        })
        .collect();
    bodies.push(SolverBody::default());
    let still = KERNEL_BODIES as u32;
    let mut pairs: Vec<(u32, u32)> = (0..60)
        .map(|_| {
            let a = (rng.next() % KERNEL_BODIES as u64) as u32;
            let b = (rng.next() % (KERNEL_BODIES as u64 + 1)) as u32;
            if b == a { (a, still) } else { (a.min(b), a.max(b)) }
        })
        .collect();
    pairs.sort();
    pairs.dedup();
    let contacts = pairs
        .into_iter()
        .enumerate()
        .map(|(k, (a, b))| {
            let n = rng.unit();
            let count = 1 + k % MAX_POINTS;
            let warm = k % 3 != 0;
            // A bouncy contact, touching and closing fast (b is driven into
            // a below): one held apart would only close its gap, and push
            // nothing to bounce.
            let restitution = if k % 4 == 1 { 0.6 } else { 0.0 };
            let mut points = [ContactPoint::default(); MAX_POINTS];
            for p in &mut points[..count] {
                let ra = n * rng.range(0.1, 0.5) + rng.vec(0.5);
                // Mostly within the speculative margin; now and then deeper
                // than a step pushes out.
                let depth = match rng.next() % 7 {
                    _ if restitution > 0.0 => rng.range(0.0, 0.01),
                    0 => rng.range(0.06, 0.2),
                    _ => rng.range(-0.04, 0.03),
                };
                let jn = if warm { rng.range(0.0, 2.0) } else { 0.0 };
                *p = ContactPoint { ra, depth, jn, speed: 0.0 };
            }
            let tangent = n.perp();
            let (jt, twist) = if warm { (tangent * rng.range(-1.0, 1.0), rng.range(-0.3, 0.3)) } else { (Vec3::ZERO, 0.0) };
            if restitution > 0.0 && b != still {
                bodies[b as usize].v -= n * 4.0;
            } else if restitution > 0.0 {
                bodies[a as usize].v += n * 4.0;
            }
            Constraint {
                a,
                b,
                normal: n,
                offset: -n * rng.range(0.5, 1.2) + rng.vec(0.2),
                friction: rng.range(0.1, 0.9),
                restitution,
                count,
                points,
                jt,
                twist,
            }
        })
        .collect();
    (bodies, contacts)
}

/// The tunings the kernel is pinned at: the default, then each other way
/// the solver can be told to solve, one at a time. `int=exact` is left out:
/// it takes a sine.
pub const KERNEL_TUNINGS: &[&str] = &[
    "",
    "sub=1",
    "sub=7,relax=2",
    "stiff=0.5,static=0.9",
    "fpush=1",
    "int=once",
    "inertia=substep",
    "anchors=linear",
    "carry=last",
    "closing=stepped",
    "closing=half",
    "closing=met",
];

/// Steps the kernel solves its inputs for at each tuning: each starts from
/// the last's velocities, rotations and impulses, so a change anywhere in
/// one step reaches the outputs of the next.
pub const KERNEL_STEPS: usize = 3;

/// `solver::solve` at `tuning` on `bodies` and `contacts`, `KERNEL_STEPS`
/// times, as a step hands the next its results.
pub fn kernel_run(tuning: &str, bodies: &mut [SolverBody], contacts: &mut [Constraint], each: impl FnMut(&[SolverBody], &[Constraint])) {
    run_at(&solver::Tuning::of(&Tuning::parse(tuning).expect("a tuning")), bodies, contacts, each)
}

/// `kernel_run`, `lanes` at a time whatever `tuning` says
/// (`solver::Tuning::lanes`; 0 is one at a time in pair order), and in
/// lanes however few contacts fill them (`sparse_alone` off).
pub fn kernel_run_in(
    tuning: &str,
    lanes: usize,
    bodies: &mut [SolverBody],
    contacts: &mut [Constraint],
    each: impl FnMut(&[SolverBody], &[Constraint]),
) {
    let how = solver::Tuning { lanes, sparse_alone: false, ..solver::Tuning::of(&Tuning::parse(tuning).expect("a tuning")) };
    run_at(&how, bodies, contacts, each)
}

fn run_at(
    how: &solver::Tuning,
    bodies: &mut [SolverBody],
    contacts: &mut [Constraint],
    mut each: impl FnMut(&[SolverBody], &[Constraint]),
) {
    for _ in 0..KERNEL_STEPS {
        solver::solve(bodies, contacts, DT, how);
        each(bodies, contacts);
        for b in bodies.iter_mut() {
            b.q = b.rotation();
        }
    }
}

/// The kernel's lines, one a tuning in each order: every output of every
/// step, hashed; each tuning with `variant` after it (pinned at ""). The
/// order is named, not the default's, since the kernel solves in the
/// solver's order: by level first (`kernel <tuning>`, its lines as pinned
/// before colouring), then colored (`kernel colored/<tuning>`). A change
/// to the arithmetic moves both; one to the coloring, the colored alone.
pub fn the_kernel(variant: &str) -> Vec<String> {
    let mut lines = Vec::new();
    for (order, prefix) in [("levels", ""), ("colored", "colored/")] {
        for tuning in KERNEL_TUNINGS {
            let (mut bodies, mut contacts) = kernel_inputs();
            let (mut hb, mut hc) = (Fnv::new(), Fnv::new());
            kernel_run(&format!("{tuning},order={order},{variant}"), &mut bodies, &mut contacts, |b, c| {
                b.iter().for_each(|b| hb.add(b));
                c.iter().for_each(|c| hc.add(c));
            });
            let name = if tuning.is_empty() { "default" } else { tuning };
            lines.push(format!("kernel {prefix}{name} bodies={:016x} contacts={:016x}", hb.0, hc.0));
        }
    }
    lines
}

/// The pinned lines whose first word is `layer`.
pub fn pinned(layer: &str) -> Vec<&'static str> {
    PINNED.lines().filter(|l| l.split_whitespace().next() == Some(layer)).collect()
}

/// What differs between the pinned lines and these, or None: how many
/// lines, and the first, both ways, with the fields that moved.
pub fn differ(pinned: &[&str], now: &[String]) -> Option<String> {
    let mut moved = pinned.iter().zip(now).filter(|(p, n)| **p != n.as_str());
    let first = moved.next();
    let count = first.is_some() as usize + moved.count();
    if count == 0 && pinned.len() == now.len() {
        return None;
    }
    let mut out = format!("{count} of {} lines differ ({} pinned)", now.len(), pinned.len());
    if let Some((p, n)) = first {
        let fields: Vec<&str> = p.split_whitespace().zip(n.split_whitespace()).filter(|(a, b)| a != b).map(|(a, _)| a).collect();
        let name = |f: &str| f.split('=').next().unwrap_or(f).to_string();
        let head: Vec<&str> = p.split_whitespace().take(2).collect();
        out += &format!(
            "; first at `{}`, in {}\n  pinned: {p}\n  now:    {n}",
            head.join(" "),
            fields.iter().map(|f| name(f)).collect::<Vec<_>>().join(", ")
        );
    }
    Some(out)
}

/// The file `--write` writes: a header, then the mod's and the kernel's
/// lines.
pub fn file(the_mod: &[String], kernel: &[String]) -> String {
    let mut out = String::from(
        "# physics3d's exact fingerprint (physics-testing.md, \"The exact fingerprint\").\n\
         # Written by `./bazel run //engine/std/physics3d:exact -- --write`, never by hand,\n\
         # and only in a commit that changes results on purpose, whose message says why.\n",
    );
    for l in the_mod.iter().chain(kernel) {
        out += l;
        out.push('\n');
    }
    out
}
