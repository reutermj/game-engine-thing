//! SPIKE (get-3hd.1): `spike_record`'s interface, what the session recorder
//! sees of a game. The windowed lockstep bootstrap writes the session log
//! (`lockstep_window.rs`), but it is resident, so it may only depend on
//! resident mods and can't name pong's types: `spike_pong_record`, a game
//! mod, fills a `Watch` at the end of every frame, and the bootstrap reads
//! it after the frame. The replay tool (`pong_replay.rs`) reads the same
//! `Watch` to check its frames against the log.
//!
//! Its own resident mod, like `spike_turns`, for the same reason.

use engine_api::{WorldMut, component};

component! {
    /// The game as of the end of frame `frame`.
    #[derive(Debug, Default, Copy)]
    pub struct Watch: "spike_record::Watch" {
        /// The frame this was written in (the `Clock`'s); 0 before any.
        pub frame: u64,
        /// `pong::Steer` events the frame applied (sent between it and the
        /// last), and the last one's intent, which is the one pong keeps.
        pub steers: u32,
        pub steer: f32,
        /// Left, right.
        pub score: [u32; 2],
        /// The ball's x, y, vx, vy.
        pub ball: [f32; 4],
        /// The left and right paddles' centre y.
        pub paddles: [f32; 2],
    }
}

pub fn watch(world: &mut WorldMut) -> Option<Watch> {
    world.single::<&Watch, _>(|_, w| *w)
}
