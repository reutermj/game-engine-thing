//! Game time, as data. Whichever bootstrap mod drives the frame loop writes
//! one `Clock` into the world before each frame, so systems read `dt` from it
//! instead of assuming a frame rate, and work unchanged under a real-time or
//! a lockstep bootstrap.

use engine_api::{WorldMut, component};

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Clock: "clock::Clock" {
        /// Frames run so far, including this one.
        pub frame: u64,
        /// Seconds this frame covers.
        pub dt: f32,
    }
}

/// The world's clock, if a bootstrap mod has published one: for code
/// between frames. A system reads it with a `Query<&Clock>`.
pub fn now(world: &mut WorldMut) -> Option<Clock> {
    world.single::<&Clock, _>(|_, clock| *clock)
}

/// Publishes `clock`, on the entity `slot` names, making one if there's
/// none (or it's gone): what a bootstrap does before each frame.
pub fn publish(world: &mut WorldMut, slot: &mut Option<engine_api::Entity>, clock: Clock) {
    match slot.filter(|&e| world.is_alive(e)) {
        Some(e) => world.insert(e, clock),
        None => *slot = Some(world.spawn((clock,))),
    }
}
