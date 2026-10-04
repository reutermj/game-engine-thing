//! What a paddle hit does to the ball, as plain numbers: `rebound` applies
//! it to the world, and `rules_test` checks it without an engine.

/// How much a hit off-center adds to vertical speed, per cell off center.
pub const SPIN: f32 = 3.0;
/// Speed gained on every return, up to `MAX_SPEED`.
pub const SPEEDUP: f32 = 1.05;
pub const MAX_SPEED: f32 = 40.0;

/// A `Contact`'s normal `n`, which points from its `a` to its `b`, as seen
/// from the ball. Pong spawns the ball first, so it is always `a` today,
/// and no game test would see this turned the wrong way round.
pub fn from_ball(n: [f32; 2], ball_is_a: bool) -> [f32; 2] {
    if ball_is_a { n } else { [-n[0], -n[1]] }
}

/// The ball's velocity after it hit a paddle: `n` is the contact's normal
/// from the ball, `side` -1 for the left paddle and 1 for the right, `v`
/// what physics bounced the ball to, and `offset` how far the ball's centre
/// was from the paddle's (positive below it).
///
/// Only a return gets pong's rule, the speed-up and the spin: a hit on the
/// paddle's face that physics turned back into the court. The ends and the
/// back are walls. The ball meets an end only once its centre is past the
/// face line, so it is going in whatever pong does; the rule there sped it
/// up and spun it without returning it (get-c3s). A corner's normal leans
/// between face and end, and physics may not turn a ball it meets
/// there, so the turn is checked too, not only the normal.
///
/// Every hit leaves at `MAX_SPEED` at most, an end's too: an end moving at
/// paddle speed adds to the ball's, and could add again each time a ball
/// pinched between it and a wall meets it.
pub fn after_hit(n: [f32; 2], side: f32, v: [f32; 2], offset: f32) -> [f32; 2] {
    if on_face(n, side) && v[0] * side < 0.0 { capped([v[0] * SPEEDUP, v[1] + offset * SPIN]) } else { capped(v) }
}

/// Whether the normal `n`, from the ball, points into the paddle from the
/// court more than along it: the face, or the face's half of a corner.
fn on_face(n: [f32; 2], side: f32) -> bool {
    n[0] * side > n[1].abs()
}

/// `v` scaled down to `MAX_SPEED` if it's faster, keeping its direction.
/// Clamping `vx` and `vy` apart let a ball leave at 40 * sqrt 2 (get-7la).
fn capped(v: [f32; 2]) -> [f32; 2] {
    let speed = v[0].hypot(v[1]);
    if speed <= MAX_SPEED {
        return v;
    }
    let k = MAX_SPEED / speed;
    [v[0] * k, v[1] * k]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn speed(v: [f32; 2]) -> f32 {
        v[0].hypot(v[1])
    }

    /// The scale is f32 arithmetic: its result may round a hair over.
    fn within_cap(v: [f32; 2]) -> bool {
        speed(v) <= MAX_SPEED * (1.0 + f32::EPSILON)
    }

    /// The hits `hit_no_bounce` flagged (presentation-spike.md section 7),
    /// with the normal physics reported, from the ball to the paddle: each
    /// met a paddle's end, 2.28 to 2.60 off its centre, past the 2.25 its
    /// face reaches, at 17 to 44 cells/s. Physics's bounce stands: `v`
    /// here is the ball's velocity before, with `vy` turned as an end would.
    #[test]
    fn the_flagged_end_hits_get_no_speed_up_or_spin() {
        // (side, normal, offset, velocity)
        let flagged = [
            (-1.0, [-0.0, -0.99999994], 2.4901342, [-16.0, 4.8]),
            (1.0, [-0.0, 1.0], -2.2825942, [40.0, -17.985834]),
            (1.0, [-0.0, 1.0], -2.6032903, [26.06229, -16.98239]),
            (1.0, [-0.0, 1.0], -2.338944, [40.0, -9.258637]),
        ];
        for (side, n, offset, v) in flagged {
            let out = after_hit(n, side, v, offset);
            assert!(out[0] * side > 0.0, "the end hit at {offset} turned the ball: {v:?} to {out:?}");
            if speed(v) <= MAX_SPEED {
                assert_eq!(out, v, "an end hit at {offset} off centre, {} cells/s, normal {n:?}", speed(v));
            } else {
                // Only scaled down: neither sped up nor spun.
                let cross = out[0] * v[1] - out[1] * v[0];
                assert!(within_cap(out) && out[0] < v[0] && cross.abs() < 1e-3, "{v:?} to {out:?}");
            }
        }
    }

    /// A hit square on either face, turned by physics, is a return.
    #[test]
    fn a_face_hit_is_a_return() {
        assert_eq!(after_hit([-1.0, 0.0], -1.0, [16.0, 5.6], 0.0), [16.0 * SPEEDUP, 5.6]);
        assert_eq!(after_hit([1.0, 0.0], 1.0, [-16.0, 5.6], 0.0), [-16.0 * SPEEDUP, 5.6]);
    }

    /// The right paddle's face seen from either end of the contact.
    #[test]
    fn a_face_hit_is_one_whichever_end_of_the_contact_the_ball_is() {
        let v = [-16.0, 0.0];
        assert_eq!(after_hit(from_ball([1.0, 0.0], true), 1.0, v, 0.0)[0], -16.0 * SPEEDUP);
        assert_eq!(after_hit(from_ball([-1.0, 0.0], false), 1.0, v, 0.0)[0], -16.0 * SPEEDUP);
        assert_eq!(after_hit(from_ball([1.0, 0.0], false), 1.0, v, 0.0), v);
    }

    /// At a corner the normal leans: the face's when it is more across than
    /// along, the end's otherwise.
    #[test]
    fn a_corner_hit_is_the_face_when_its_normal_is_mostly_across() {
        let v = [-16.0, 0.0];
        assert_ne!(after_hit([0.8, 0.6], 1.0, v, 2.3), v);
        assert_ne!(after_hit([-0.8, -0.6], -1.0, [16.0, 0.0], 2.3), [16.0, 0.0]);
        assert_eq!(after_hit([0.6, 0.8], 1.0, v, 2.3), v);
        assert_eq!(after_hit([-0.6, 0.8], -1.0, [16.0, 0.0], 2.3), [16.0, 0.0]);
    }

    /// A corner hit with the face's normal that physics didn't turn: the
    /// ball still heading for the goal. Replayed from a session once the
    /// ends were left to physics (frame 5082 of the scripted versus
    /// session), where it was spun to 24.5 down and then pinched between
    /// the paddle's end and the wall.
    #[test]
    fn a_corner_hit_physics_did_not_turn_is_not_a_return() {
        let v = [2.526058, 16.76238];
        assert_eq!(after_hit([0.7700691, -0.6379605], 1.0, v, 2.5626907), v);
    }

    /// A ball behind a paddle (the paddle caught up with it on the way into
    /// the goal) meets its back, which faces the goal, not the court.
    #[test]
    fn the_back_of_a_paddle_is_not_its_face() {
        assert_eq!(after_hit([1.0, 0.0], -1.0, [-16.0, 0.0], 0.0), [-16.0, 0.0]);
        assert_eq!(after_hit([-1.0, 0.0], 1.0, [16.0, 0.0], 0.0), [16.0, 0.0]);
    }

    /// get-7la: clamping `vx` and `vy` to 40 each let a ball leave at 56.6.
    #[test]
    fn no_ball_leaves_a_hit_faster_than_max_speed() {
        // Returns off the right face: a 40-cell/s ball (vx at the cap)
        // struck 2.2 off centre from above and from below, and a slower one
        // struck far enough off centre for the spin to pass the cap.
        for (v, offset) in [([-40.0, 17.98], -2.2), ([-40.0, -17.98], 2.2), ([-38.0, 30.0], 2.25), ([-20.0, 0.0], 2.25)] {
            let out = after_hit([1.0, 0.0], 1.0, v, offset);
            assert!(within_cap(out), "{v:?} at {offset} left at {out:?}, {}", speed(out));
        }
        // An end moving at 16 that turned a 40-cell/s ball's vy: 44.8
        // (frame 3524 of the scripted versus session, once ends were left
        // to physics).
        let out = after_hit([0.0, -1.0], 1.0, [16.0, 44.79997], 2.27);
        assert!(within_cap(out), "{out:?}, {}", speed(out));
    }

    /// The cap scales the speed, so the ball leaves in the direction the hit
    /// gave it.
    #[test]
    fn the_cap_keeps_the_direction() {
        let (v, offset) = ([40.0, 10.0], 2.0);
        let uncapped = [v[0] * SPEEDUP, v[1] + offset * SPIN];
        let out = after_hit([-1.0, 0.0], -1.0, v, offset);
        assert!(speed(out) > MAX_SPEED * 0.999, "{out:?}");
        let cross = out[0] * uncapped[1] - out[1] * uncapped[0];
        assert!(cross.abs() < 1e-3 && out[0] * uncapped[0] > 0.0, "{out:?} turned from {uncapped:?}");
    }

    /// Below the cap a return is the speed-up and the spin, as before.
    #[test]
    fn a_return_below_the_cap_speeds_up_and_spins() {
        let out = after_hit([-1.0, 0.0], -1.0, [16.0, 5.6], -1.0);
        assert!((out[0] - 16.8).abs() < 1e-4 && (out[1] - 2.6).abs() < 1e-4, "{out:?}");
    }
}
