//! SPIKE (get-3hd.1): `spike_platform`'s interface, D6's platform service:
//! the window and its event loop, behind plain data. A window crosses as
//! its X11 handles (numbers), which a presenter makes its surface from.

engine_api::field_struct! {
    /// What one pump of the window's events found.
    #[derive(Debug, Default)]
    pub struct Pumped {
        pub close: bool,
        pub width: u32,
        pub height: u32,
        /// Window events handled by this pump.
        pub events: u32,
        /// Keys pressed since the last pump, as winit names them.
        pub keys: Vec<String>,
        /// How long the pump took.
        pub micros: u32,
    }
}

engine_api::field_struct! {
    /// The window as X11 knows it: an Xlib `Display *` (as a number), its
    /// screen, and the window's id. `display` is 0 when there's no window.
    #[derive(Debug, Default)]
    pub struct WindowHandles {
        pub display: u64,
        pub screen: i32,
        pub window: u64,
        pub width: u32,
        pub height: u32,
    }
}

engine_api::service! {
    pub trait Platform {
        /// Handles the window's pending events without waiting.
        fn pump() -> Pumped;
        fn window() -> WindowHandles;
        /// SPIKE ONLY, the experiment D9 rules out: the address of a
        /// `presentation_gfx::SharedGpu` (instance, adapter, device and
        /// queue) this resident mod made with its own copy of wgpu, on the
        /// adapter whose name contains `adapter`. A plain number, since no
        /// wgpu object may cross a call; a caller that dereferences it uses
        /// another library's wgpu objects from its own copy of wgpu.
        fn shared_gpu(adapter: String) -> u64;
    }
}

engine_api::service! {
    /// Draws the last frame again, between frames: what a bootstrap that
    /// idles (lockstep) calls when the window was exposed or resized, so a
    /// spectator window doesn't go stale while no frames run. Declared here,
    /// not in the presenter's interface, because the bootstrap is resident
    /// and so may only depend on resident mods; the call resolves by name
    /// to whichever presenter build is loaded, if any (`NotProvided`
    /// otherwise). Returns whether anything was drawn.
    pub trait Redraw {
        fn redraw() -> bool;
    }
}
