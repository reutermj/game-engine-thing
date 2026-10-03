//! physics3d's solve held exactly to its pinned fingerprint (`exact.rs`,
//! `exact.txt`; physics-testing.md, "The exact fingerprint"): any change
//! to a bit of a body or a contact, from the gather, the solver, the
//! write-back or the narrowphase, fails here. A change meant to move
//! results rewrites the file with `:exact -- --write`, in its own commit.

// The solver's and narrowphase's modules, as `core_test` has them: only
// what the kernel's run uses of them is used.
#![allow(dead_code)]

#[path = "exact.rs"]
mod exact;
#[path = "../gjk.rs"]
mod gjk;
#[path = "../narrow.rs"]
mod narrow;
#[path = "../solver.rs"]
mod solver;

use solver::{Constraint, SolverBody};

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Order, Quat, Reduce, Shape, Tuning, Vec3};

const UPDATE: &str = "a change meant to move results rewrites it: ./bazel run //engine/std/physics3d:exact -- --write";

/// The mod in the engine, every frame of pile3d's scene, bit for bit as
/// pinned; and the scene is what `exact.rs` says it is, so the pin covers
/// what it claims to.
#[test]
fn the_mod_is_its_pinned_fingerprint_bit_for_bit() {
    let (lines, seen) = exact::the_mod(exact::Run::default());
    println!("{seen:?}");
    for (count, n) in seen.points.iter().enumerate().skip(1) {
        assert!(*n > 0, "no contact of {count} points in the scene: {seen:?}");
    }
    assert!(seen.turned > 20, "too few bodies turned: {seen:?}");
    assert_eq!(seen.locked_w, [0.0; 3], "the locked box turns");
    if let Some(d) = exact::differ(&exact::pinned("mod"), &lines) {
        panic!("the mod isn't its pinned fingerprint: {d}\n{UPDATE}");
    }
}

/// The mod's passes run on one thread, where the kernels get their states
/// plain; across threads they get them shared (`States::Shared`), a path
/// of their own. Shared on one thread, the mod is its fingerprint bit for
/// bit, by level and colored alike, so the scheduler's threads
/// (get-znt.34) start from a path already held to it (2D's
/// `the_mod_is_the_arrays_with_its_states_shared`).
#[test]
fn the_mod_is_its_fingerprint_with_its_states_shared() {
    for tune in ["", "order=colored"] {
        let (plain, _) = exact::the_mod(exact::Run { tune, shared: false });
        let (shared, _) = exact::the_mod(exact::Run { tune, shared: true });
        let plain: Vec<&str> = plain.iter().map(String::as_str).collect();
        if let Some(d) = exact::differ(&plain, &shared) {
            panic!("{tune:?}: the states shared aren't the states plain: {d}");
        }
    }
}

/// The mod solves in the order its world's `Tuning` names: by level it is
/// the default, colored it is not. Each against a run with a `Tuning` in
/// its world, an entity, which moves every body's index from the pinned
/// run's.
#[test]
fn the_mod_solves_in_the_order_its_world_sets() {
    let run = |tune| exact::the_mod(exact::Run { tune, shared: false }).0;
    let (default, levels, colored) = (run("lanes=4"), run("order=levels"), run("order=colored"));
    assert_eq!(default, levels, "by level, named, isn't the default");
    let moved = levels.iter().zip(&colored).filter(|(a, b)| a != b).count();
    assert!(moved > levels.len() / 2, "colored, only {moved} of {} frames differ from by level", levels.len());
}

/// `solver::solve` alone, on the kernel's inputs at each tuning, bit for
/// bit as pinned: the arithmetic, apart from the order it is run in.
#[test]
fn the_kernel_is_its_pinned_fingerprint_bit_for_bit() {
    if let Some(d) = exact::differ(&exact::pinned("kernel"), &exact::the_kernel("")) {
        panic!("the solver isn't its pinned fingerprint: {d}\n{UPDATE}");
    }
}

/// The lanes (`solver::lanes`) are the solve one contact at a time in
/// their order bit for bit (by level the sweep in pair order, colored the
/// sweep over the colors' order, `solver::in_order`), at every width and
/// under every tuning the kernel is pinned at, and `int=exact` and
/// `carry=normal` besides (`int=exact` the bodies' stage, which both
/// share, but a sine is no reason to leave a tuning out of a comparison
/// on one host): every body and contact after each step, compared as
/// `Debug` prints them, which tells `-0.0` from `0.0`. On the kernel's
/// inputs, and on them with what they lack (`odd_inputs`). The
/// fingerprint pins the default width and order; this holds the others,
/// and the scalar reference, to it. It is the test a colored solve
/// across threads has to pass too.
#[test]
fn the_lanes_are_the_solve_one_contact_at_a_time_bit_for_bit() {
    let tunings = exact::KERNEL_TUNINGS.iter().copied().chain(["int=exact", "carry=normal"]);
    for tuning in tunings {
        for order in ["levels", "colored"] {
            let tuning = format!("{tuning},order={order}");
            for (inputs, make) in [("kernel", exact::kernel_inputs as fn() -> _), ("odd", odd_inputs)] {
                let run = |lanes: usize| {
                    let (mut bodies, mut contacts) = make();
                    let mut steps = Vec::new();
                    exact::kernel_run_in(&tuning, lanes, &mut bodies, &mut contacts, |b, c| steps.push(format!("{b:?}\n{c:?}")));
                    steps
                };
                let one = run(0);
                for lanes in [1, 4, 8] {
                    for (step, (a, b)) in one.iter().zip(run(lanes)).enumerate() {
                        assert!(*a == b, "{tuning:?}, {inputs} inputs, {lanes} lanes: step {step} differs from one at a time");
                    }
                }
            }
        }
    }
}

/// The two orders are two computations, which the kernel's inputs tell
/// apart under every tuning: without this, the equivalence above could
/// compare the levels with themselves under both names.
#[test]
fn the_colored_order_is_another_computation() {
    for tuning in exact::KERNEL_TUNINGS {
        let run = |order: &str| {
            let (mut bodies, mut contacts) = exact::kernel_inputs();
            exact::kernel_run(&format!("{tuning},order={order}"), &mut bodies, &mut contacts, |_, _| {});
            (bodies, contacts)
        };
        let ((lb, lc), (cb, cc)) = (run("levels"), run("colored"));
        let moved = lb.iter().zip(&cb).filter(|(x, y)| x != y).count();
        assert!(moved > exact::KERNEL_BODIES / 2, "{tuning:?}: colored, only {moved} bodies differ from by level");
        assert_ne!(lc, cc, "{tuning:?}: colored, the contacts' impulses are by level's");
    }
}

/// The kernel's inputs with what they lack, for the lanes' equivalence
/// (the pinned inputs stay as they are): a body that moves but takes no
/// impulse, as a kinematic one would (3D's gather makes none, but the
/// solver takes it), so a body nothing pushes is read moving; a contact
/// between it and the statics, neither end of which moves; and one
/// between the body that can't turn and the statics, carrying warm
/// friction and twist, which it has a tangent mass for and no twist mass.
fn odd_inputs() -> (Vec<SolverBody>, Vec<Constraint>) {
    let (mut bodies, mut contacts) = exact::kernel_inputs();
    let (still, kinematic) = (exact::KERNEL_BODIES as u32, 7);
    let b = &mut bodies[kinematic as usize];
    (b.inv_mass, b.inv_inertia, b.gravity) = (0.0, Vec3::ZERO, Vec3::ZERO);
    let three = *contacts.iter().find(|c| c.count == 3).expect("a contact of three points");
    let warm = Constraint { jt: three.normal.perp() * 0.5, twist: 0.3, ..three };
    contacts.push(Constraint { a: 3, b: still, ..warm });
    contacts.push(Constraint { a: kinematic, b: still, ..warm });
    // In pair order, as the gather hands them over.
    contacts.sort_by_key(|c| (c.a, c.b));
    (bodies, contacts)
}

/// The kernel's inputs couple their contacts through shared bodies, so the
/// order they are solved in shows in every output: reversed, every tuning's
/// result differs. Without this, a kernel test could pass an order change
/// it can't see.
#[test]
fn the_kernels_inputs_see_the_order_they_are_solved_in() {
    for tuning in exact::KERNEL_TUNINGS {
        let (mut bodies, mut contacts) = exact::kernel_inputs();
        let (mut rb, mut rc) = (bodies.clone(), contacts.clone());
        rc.reverse();
        exact::kernel_run(tuning, &mut bodies, &mut contacts, |_, _| {});
        exact::kernel_run(tuning, &mut rb, &mut rc, |_, _| {});
        rc.reverse();
        let moved = bodies.iter().zip(&rb).filter(|(x, y)| x != y).count();
        assert!(moved > exact::KERNEL_BODIES / 2, "{tuning:?}: reversed, only {moved} bodies moved");
        assert_ne!(contacts, rc, "{tuning:?}: reversed, the contacts' impulses are the same");
    }
}

/// The kernel's inputs bounce, slide past their friction and push out
/// overlaps, which their outputs show: a contact solved differently in
/// each way would be pinned without being exercised otherwise.
#[test]
fn the_kernels_inputs_exercise_what_a_pass_branches_on() {
    let (bodies, contacts) = exact::kernel_inputs();
    let (mut b, mut c) = (bodies.clone(), contacts.clone());
    // The last substep's impulses, so a contact's friction is its last
    // relax pass's, clamped against that pass's normal impulses.
    let how = solver::Tuning::of(&Tuning::parse("carry=last").unwrap());
    solver::solve(&mut b, &mut c, exact::DT, &how);
    let bounced =
        c.iter().filter(|c| c.restitution > 0.0 && c.points[..c.count].iter().any(|p| p.speed > solver::BOUNCE_THRESHOLD)).count();
    assert!(bounced > 0, "nothing closes fast enough to bounce");
    let (mut dead, mut dc) = (bodies.clone(), contacts.clone());
    dc.iter_mut().for_each(|c| c.restitution = 0.0);
    solver::solve(&mut dead, &mut dc, exact::DT, &how);
    let bouncing = b.iter().zip(&dead).filter(|(x, y)| x.v != y.v).count();
    assert!(bouncing > 0, "restitution changes nothing");
    let at_limit = c
        .iter()
        .filter(|c| {
            let total: f32 = c.points[..c.count].iter().map(|p| p.jn).sum();
            c.restitution == 0.0 && total > 0.0 && (c.jt.len() - c.friction * total).abs() <= 1e-3 * c.friction * total
        })
        .count();
    assert!(at_limit > 0, "no contact slid at its friction's limit");
    let deep = contacts.iter().filter(|c| c.points[..c.count].iter().any(|p| p.depth > solver::MAX_PUSH * exact::DT)).count();
    assert!(deep > 0, "no overlap past what a step pushes out");
    let apart = contacts.iter().filter(|c| c.points[..c.count].iter().any(|p| p.depth < 0.0)).count();
    assert!(apart > 0, "no speculative gap");
    for count in 1..=MAX_POINTS {
        assert!(contacts.iter().any(|c| c.count == count), "no contact of {count} points");
    }
}
