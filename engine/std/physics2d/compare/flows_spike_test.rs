//! The flows spike's physics pipeline (`flows_physics.rs`) against the
//! solve as built, bit for bit, on small scenes stepped by the mod: at one
//! thread and across threads, with the generic primitive's kernel a batch
//! of four, the same through a pointer, and the solver alone a kernel per
//! edge. The spike's bench (`flows_spike`) checks the same at 10 000
//! bodies before it times anything.

#[allow(dead_code)]
#[path = "flows_spike_arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[allow(dead_code)]
mod flows_physics;
#[path = "flows_spike_narrow.rs"]
mod narrow;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
#[allow(dead_code)]
#[path = "flows_spike_solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "flows_spike_split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::sync::Arc;

use engine_ecs::{Scoped, Workers};
use flows_physics::Shape;
use scene::Scene;
pub use sim::{Dyn, Sim};
use solver::lanes::Prepared;
use solver::{Constraint, Points, SolverBody, Spinning, Wide};

fn manifest() -> engine_control::Manifest {
    engine_control::read_manifest(&std::env::var("SCENE_GAME").unwrap()).unwrap()
}

/// A pyramid falling into place and a small pile, turning, at steps where
/// they press, slide and bounce: the pipeline against the solve as built,
/// on the world, at 1 and 3 threads.
#[test]
fn the_pipeline_is_the_solve_as_built_bit_for_bit() {
    let manifest = manifest();
    for (scene, steps) in [(Scene::Pyramid { base: 12 }, [1, 20, 60]), (Scene::Pile { n: 300, width: 41.0, stagger: true }, [40, 90, 200])]
    {
        let mut ecs = ecs::Ecs::new(&manifest, &scene, false, true);
        let mut at = 0;
        for step in steps {
            ecs.step(step - at);
            at = step;
            let world = ecs.engine().world();
            flows::reset(world);
            let [bodies, turning, contacts, points] =
                flows_physics::check_against_reference(world, &[Shape::Batch, Shape::Dyn, Shape::Block]);
            assert!(
                bodies > 70 && turning > 70 && contacts > 50 && points > 50,
                "{scene:?} at step {step}: {bodies} {turning} {contacts} {points}"
            );
            world.set_executor(Some(Arc::new(Scoped(3))));
            flows_physics::check_against_reference(world, &[Shape::Batch, Shape::Dyn, Shape::Block]);
            world.set_executor(None);
        }
    }
}

/// The solver's input as a step hands it over, captured from the arrays'
/// step on the same scene.
fn inputs() -> Vec<(Vec<SolverBody>, Vec<Spinning>, Vec<Constraint>, Vec<Points>)> {
    let manifest = manifest();
    let mut ecs = ecs::Ecs::new(&manifest, &Scene::Pyramid { base: 12 }, false, true);
    let mut out = Vec::new();
    for _ in 0..4 {
        ecs.step(15);
        let world = ecs.engine().world();
        let (mut bodies, mut spinning, mut constraints, mut points) = flows_physics::gathered(world);
        // And a kinematic body in contact with the statics, at its normal
        // and at points: contacts neither end of which moves, which the
        // solve finishes before its passes (`unsolved`), and which the
        // scenes don't have.
        let still = bodies.len() as u32 - 1;
        bodies.push(SolverBody::new(physics2d::Vec2::new(1.0, 0.0), 0.0, physics2d::Vec2::ZERO));
        spinning.push(Spinning::new(still + 1, 0.5, 0.0));
        let normal = physics2d::Vec2::new(0.0, 1.0);
        let at_normal =
            Constraint { a: still, b: still + 1, normal, depth: -0.01, friction: 0.5, jn: 0.25, jt: 0.125, ..Constraint::default() };
        constraints.push(at_normal);
        let point = solver::ContactPoint { separation: -0.01, jn: 0.5, jt: 0.25, ..Default::default() };
        points.push(Points { count: 2, point: [point; 2], solved: false });
        constraints.push(Constraint { points: points.len() as u32, ..at_normal });
        out.push((bodies, spinning, constraints, points));
    }
    out
}

/// The solver alone, every shape and width of the generic primitive, at 1
/// and 4 threads: `solve_across` bit for bit, including a kernel per edge.
#[test]
fn every_shape_of_the_generic_passes_is_the_solve_as_built() {
    let params = solver::Params::of(&physics2d::Tuning::DEFAULT);
    for (bodies, spinning, constraints, points) in inputs() {
        let one = Workers::default();
        let (mut b, mut s, mut c, mut p) = (bodies.clone(), spinning.clone(), constraints.clone(), points.clone());
        solver::solve_across(&params, (&mut b, &mut s), &mut c, &mut p, 1.0 / 60.0, &one);
        let want = (b, s, c, p);
        for threads in [1, 4] {
            let w = Workers::new(Some(Arc::new(Scoped(threads))));
            for (wide, shape) in [
                (4, Shape::Batch),
                (4, Shape::Dyn),
                (4, Shape::Block),
                (8, Shape::Batch),
                (8, Shape::Block),
                (1, Shape::Batch),
                (1, Shape::Edge),
            ] {
                let params = solver::Params { wide: Wide::Colored(wide), ..params };
                let (mut b, mut s, mut c, mut p) = (bodies.clone(), spinning.clone(), constraints.clone(), points.clone());
                let dt = 1.0 / 60.0;
                match wide {
                    4 => solver::lanes::solve_flow(&mut Prepared::<4>::default(), &params, (&mut b, &mut s), &mut c, &mut p, dt, &w, shape),
                    8 => solver::lanes::solve_flow(&mut Prepared::<8>::default(), &params, (&mut b, &mut s), &mut c, &mut p, dt, &w, shape),
                    _ => solver::lanes::solve_flow(&mut Prepared::<1>::default(), &params, (&mut b, &mut s), &mut c, &mut p, dt, &w, shape),
                }
                let bits = |x: &(Vec<SolverBody>, Vec<Spinning>, Vec<Constraint>, Vec<Points>)| format!("{x:?}");
                assert!(bits(&(b, s, c, p)) == bits(&want), "{wide} wide, {shape:?}, {threads} threads");
            }
        }
    }
}
