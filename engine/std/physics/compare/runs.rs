//! Every run a test binary makes, made once (`physics_testkit::runs`): the
//! quality tests, the behaviour tests and the baseline (`record.rs`) ask
//! for the same scenes, and whichever asks first runs one while the others
//! wait for it.

use physics_testkit::runs::Runs;

use crate::behave::{self, Behaviour};
use crate::scene::Scene;
use crate::settle::{self, Settling};
use crate::{Sim, ecs};

static SETTLED: Runs<Settling> = Runs::new();
static BEHAVED: Runs<Behaviour> = Runs::new();
static ASLEEP: Runs<Option<u32>> = Runs::new();

/// Each of `items` through `f`, in threads of their own, the results in
/// their order. At most 256 at once: a long bounce grid is 9900 runs, and
/// several tests spawning that many side by side ran out of threads.
pub fn par<A: Sync, T: Send>(items: &[A], f: impl Fn(&A) -> T + Sync) -> Vec<T> {
    physics_testkit::runs::par(items, 256, f)
}

/// The variant `SOLVER` names in the environment (`variants.rs`), which
/// runs in place of ours as built: how the bounds were checked to fail on
/// the solvers they should.
pub fn solver() -> Option<String> {
    std::env::var("SOLVER").ok().filter(|s| !s.is_empty())
}

/// `spec`, a `rot` variant, on the solve `SOLVER` names where that is one
/// too (`rot/carry=1` and `rot/order=3` make `rot/carry=1/order=3`): so the
/// tests that pick an order or a grouping of their own still run the
/// option being weighed.
pub fn with_solver(spec: &str) -> String {
    match solver() {
        Some(s) if s.starts_with("rot") => spec.replacen("rot", &s, 1),
        _ => spec.to_string(),
    }
}

/// Our step on arrays: the variant `spec`, or with "" ours as built (or
/// what `SOLVER` names).
pub fn ours(scene: &Scene, turning: bool, spec: &str) -> ecs::Flat {
    let spec = if spec.is_empty() { solver().unwrap_or_default() } else { spec.to_string() };
    let label = if spec.is_empty() { "ours".to_string() } else { format!("ours ({spec})") };
    ecs::Flat::ours(scene, turning, &spec, &label)
}

/// The physics mod in the engine on `scene`, with sleeping or not.
pub fn mod_in_engine(scene: &Scene, turning: bool, sleep: bool) -> ecs::Ecs {
    let manifest = engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap();
    ecs::Ecs::new(&manifest, scene, sleep, turning)
}

/// `scene` settled on arrays over `steps` by `ours` with `spec`.
pub fn settled(scene: Scene, turning: bool, steps: u32, spec: &str) -> Settling {
    let key = format!("{} {turning} {steps} {spec} {:?}", scene.text(), solver());
    (*SETTLED.get(key, || settle::settle(&mut ours(&scene, turning, spec), &scene, turning, steps))).clone()
}

/// `scene` settled on the mod in the engine at `substeps`, which a game
/// sets by writing physics's `Tuning`: what the arrays can't take, having
/// no world to read it from.
pub fn settled_on_mod(scene: Scene, turning: bool, substeps: u32, steps: u32) -> Settling {
    let key = format!("mod {} {turning} {substeps} {steps}", scene.text());
    let run = SETTLED.get(key, || {
        let mut m = mod_in_engine(&scene, turning, false);
        m.substeps(substeps);
        settle::settle(&mut m, &scene, turning, steps)
    });
    (*run).clone()
}

/// The look (every `settle::EVERY` steps) at which every body of `scene`
/// is asleep, on the mod with its default sleeping; none by `max`.
pub fn asleep_at(scene: Scene, turning: bool, max: u32) -> Option<u32> {
    let key = format!("{} {turning} {max}", scene.text());
    *ASLEEP.get(key, || {
        let mut m = mod_in_engine(&scene, turning, true);
        let bodies = scene.build().iter().filter(|s| s.dynamic).count();
        let mut step = 0;
        while m.asleep() < bodies {
            if step >= max {
                return None;
            }
            m.step(settle::EVERY);
            step += settle::EVERY;
        }
        Some(step)
    })
}

/// `scene`, a behaviour scene, run by `ours`, bodies turning.
pub fn behaved(scene: Scene) -> Behaviour {
    let key = format!("{} {:?}", scene.text(), solver());
    (*BEHAVED.get(key, || behave::behave(&mut ours(&scene, true, ""), &scene, true))).clone()
}
