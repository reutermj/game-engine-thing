//! A system: no state of its own, just code over the world's components.
//! Edit the integration and `./bazel run //mods/physics`; the entities keep
//! their positions and move by the new rule.

use engine_api::{Cx, Mod, Status, export_mod};
use physics::Velocity;
use transform::Position;

/// The bootstrap mod's frame rate.
const DT: f32 = 1.0 / 60.0;

engine_api::mod_state! {
    #[derive(Default)]
    struct Physics {}
}

impl Mod for Physics {
    type Transient = ();

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        for (_, velocity, position) in cx.world().query2::<Velocity, Position>() {
            position.x += velocity.x * DT;
            position.y += velocity.y * DT;
        }
        Status::OK
    }
}

export_mod!(Physics);
