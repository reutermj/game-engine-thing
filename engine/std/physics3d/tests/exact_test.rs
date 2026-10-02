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

pub use physics3d::{Anchors, BoxBox, Carry, Closing, Inertia, Integrate, MAX_POINTS, Mat3, Quat, Reduce, Shape, Tuning, Vec3};

const UPDATE: &str = "a change meant to move results rewrites it: ./bazel run //engine/std/physics3d:exact -- --write";

/// The mod in the engine, every frame of pile3d's scene, bit for bit as
/// pinned; and the scene is what `exact.rs` says it is, so the pin covers
/// what it claims to.
#[test]
fn the_mod_is_its_pinned_fingerprint_bit_for_bit() {
    let (lines, seen) = exact::the_mod();
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

/// `solver::solve` alone, on the kernel's inputs at each tuning, bit for
/// bit as pinned: the arithmetic, apart from the order it is run in.
#[test]
fn the_kernel_is_its_pinned_fingerprint_bit_for_bit() {
    if let Some(d) = exact::differ(&exact::pinned("kernel"), &exact::the_kernel()) {
        panic!("the solver isn't its pinned fingerprint: {d}\n{UPDATE}");
    }
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
