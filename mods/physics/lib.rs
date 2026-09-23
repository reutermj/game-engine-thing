//! A system: no state of its own, just code over the world's components.
//! Edit the integration and `./bazel run //mods/physics`; the entities keep
//! their positions and move by the new rule.

use engine_api::{Cx, Mod, Query, Systems, export_mod, phase};
use physics::Velocity;
use transform::Position;

/// The bootstrap mod's frame rate.
const DT: f32 = 1.0 / 60.0;

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics {}
}

impl Physics {
    fn integrate(&mut self, _: &mut (), cx: &mut Cx, q: Query<(&Velocity, &mut Position)>) {
        for (_, (velocity, position)) in q.iter(cx) {
            position.x += velocity.x * DT;
            position.y += velocity.y * DT;
        }
    }
}

impl Mod for Physics {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("integrate", Self::integrate).phase(phase::SIMULATE);
    }
}

export_mod!(Physics);
