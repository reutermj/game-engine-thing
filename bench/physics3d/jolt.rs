//! Jolt Physics through jolt_shim.cpp: JobSystemSingleThreaded, translation
//! DOFs only, and Jolt's default solver unless [`Iters::Eight`].

use crate::ffi;
use crate::{Backend, Config, Iters, Spec};

pub struct Jolt {
    world: *mut ffi::JoltWorld,
    dynamic: Vec<u32>,
}

impl Jolt {
    pub fn new(config: &Config) -> Self {
        let iters = match config.iters {
            Iters::Default => 0,
            Iters::Eight => 8,
        };
        let c =
            ffi::P3Config { max_bodies: config.max_bodies, velocity_iters: iters, position_iters: iters, allow_sleep: config.sleep as i32 };
        // SAFETY: the shim copies what it needs from the config.
        let world = unsafe { ffi::p3_jolt_create(&c) };
        assert!(!world.is_null());
        Jolt { world, dynamic: Vec::new() }
    }
}

impl Drop for Jolt {
    fn drop(&mut self) {
        // SAFETY: the world came from p3_jolt_create and is dropped once.
        unsafe { ffi::p3_jolt_destroy(self.world) }
    }
}

impl Backend for Jolt {
    fn name(&self) -> String {
        "jolt".into()
    }

    fn solver(&self) -> String {
        let (mut v, mut p) = (0, 0);
        // SAFETY: a live world and two valid out pointers.
        unsafe { ffi::p3_jolt_iterations(self.world, &mut v, &mut p) };
        format!("velocity steps {v}, position steps {p}")
    }

    fn add(&mut self, bodies: &[Spec]) -> Vec<u32> {
        let specs = ffi::to_c(bodies);
        let mut handles = vec![0u32; bodies.len()];
        // SAFETY: both arrays hold bodies.len() elements.
        unsafe { ffi::p3_jolt_add(self.world, bodies.len() as u32, specs.as_ptr(), handles.as_mut_ptr()) };
        self.dynamic.extend(bodies.iter().zip(&handles).filter(|(s, _)| !s.fixed).map(|(_, h)| *h));
        handles
    }

    fn step(&mut self, dt: f32) {
        // SAFETY: a live world.
        unsafe { ffi::p3_jolt_step(self.world, dt) }
    }

    fn state(&self, out: &mut Vec<([f32; 3], [f32; 3])>) {
        let mut buf = vec![0f32; 6 * self.dynamic.len()];
        // SAFETY: every handle came from this world; buf holds 6 floats each.
        unsafe { ffi::p3_jolt_read(self.world, self.dynamic.len() as u32, self.dynamic.as_ptr(), buf.as_mut_ptr()) };
        ffi::unpack(&buf, out);
    }

    fn touching(&self) -> usize {
        // SAFETY: a live world.
        unsafe { ffi::p3_jolt_touching(self.world) as usize }
    }

    /// None: Jolt's per-stage timings exist only with JPH_PROFILE_ENABLED,
    /// which instruments every job and so would slow what is being timed.
    fn stages(&self) -> Vec<(&'static str, f64)> {
        Vec::new()
    }
}
