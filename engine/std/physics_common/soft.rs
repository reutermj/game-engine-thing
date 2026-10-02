/// Heavily damped, as in Box2D: a contact pushes out without bouncing.
pub const DAMPING_RATIO: f32 = 10.0;
/// The fastest a contact pushes bodies apart, so a deep overlap comes
/// apart over a few steps, not in one throw.
pub const MAX_PUSH: f32 = 3.0;
/// Closing speeds below this don't bounce, so resting bodies settle.
pub const BOUNCE_THRESHOLD: f32 = 1.0;

/// A soft contact's constants for a substep of `h`: how fast it pushes
/// out per unit of penetration, and how much of a rigid impulse it takes
/// (`mass`) and of its accumulated one it lets go (`impulse`). Box2D's
/// `b2MakeSoft` (Erin Catto's soft step, docs/CREDITS.md).
#[derive(Clone, Copy, Debug)]
pub struct Softness {
    pub rate: f32,
    pub mass: f32,
    pub impulse: f32,
}

impl Softness {
    #[inline]
    pub fn new(hertz: f32, zeta: f32, h: f32) -> Softness {
        let omega = 2.0 * std::f32::consts::PI * hertz;
        let a1 = 2.0 * zeta + h * omega;
        let a2 = h * omega * a1;
        let a3 = 1.0 / (1.0 + a2);
        Softness { rate: omega / a1, mass: a2 * a3, impulse: a3 }
    }
}
