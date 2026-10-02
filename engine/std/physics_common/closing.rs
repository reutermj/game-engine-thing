use crate::BOUNCE_THRESHOLD;

/// What restitution bounces a contact back from, and judges against
/// `BOUNCE_THRESHOLD`: its closing speed as the step found it, before or
/// after the step's gravity, which each mod has already added to every
/// body's velocity when its solve begins. The options weighed for
/// get-emj.56 and get-emj.60 (physics.md, "Bounces"): `Before` is the
/// default in both mods, what every reference does and the one that can't
/// return energy a body didn't bring; the others are the comparisons'
/// variants (3D's `Tuning` names the first four).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Closing {
    /// With the step's gravity in it: until 2026-09-29, when it returned a
    /// step of gravity more than a bounce came in with (get-emj.56).
    Stepped,
    /// As the step began, before its gravity: Box2D's `relativeVelocity`,
    /// taken at `b2PrepareContactsTask` before its substeps' gravity, as
    /// Rapier's and Box3D's are. The default.
    Before,
    /// With half the step's gravity: the speed half a step on.
    Half,
    /// As the step began, grown by gravity over the gap left, or over as
    /// far as the step falls if that's less (√(v² + 2 a s)): the speed at
    /// the moment the bodies meet, or as a speculative contact catches them
    /// short of it.
    Met,
    /// With the step's gravity, the rebound less that gravity (e c − g h).
    Less,
    /// With the step's gravity, but none if it came in under the threshold
    /// before it.
    Gate,
}

impl Closing {
    /// The speed restitution bounces a contact back from, from its closing
    /// speed `c` with the step's gravity in it, the share of that the
    /// gravity gave (`g`, the ends' difference along the normal), its gap as
    /// found (`sep`), the step and its restitution.
    #[inline(always)]
    pub fn speed(self, c: f32, g: f32, sep: f32, dt: f32, e: f32) -> f32 {
        let before = c - g;
        match self {
            Closing::Stepped => c,
            Closing::Before => before,
            Closing::Half => c - 0.5 * g,
            Closing::Met if g > 0.0 && sep > 0.0 => {
                let v = before.max(0.0);
                let fall = (v * dt + 0.5 * g * dt).min(sep);
                (v * v + 2.0 * (g / dt) * fall).sqrt()
            }
            Closing::Met => before,
            Closing::Less if e > 0.0 => c - g / e,
            Closing::Less => c,
            Closing::Gate if before > BOUNCE_THRESHOLD => c,
            Closing::Gate => 0.0,
        }
    }
}
