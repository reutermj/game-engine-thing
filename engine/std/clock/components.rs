//! Game time, as data. Whichever bootstrap mod drives the frame loop writes
//! one `Clock` into the world before each frame, so systems read `dt` from it
//! instead of assuming a frame rate, and work unchanged under a real-time or
//! a lockstep bootstrap.

use engine_api::{World, component};

component! {
    #[derive(Debug, Default, Copy)]
    pub struct Clock: "clock::Clock" {
        /// Frames run so far, including this one.
        pub frame: u64,
        /// Seconds this frame covers.
        pub dt: f32,
    }
}

/// The world's clock, if a bootstrap mod has published one.
pub fn now(world: &mut World) -> Option<Clock> {
    world.query::<Clock>().next().map(|(_, clock)| *clock)
}
