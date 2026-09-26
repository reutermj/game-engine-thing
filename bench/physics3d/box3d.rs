//! Box3D through box3d_shim.c: one worker and no task callbacks, rotation
//! locked by motion locks, and Box3D's usual 4 substeps unless [`Iters::Eight`].

use crate::ffi;
use crate::{Backend, Config, Iters, Spec};

pub struct Box3d {
    world: *mut ffi::Box3dWorld,
    dynamic: Vec<u32>,
}

impl Box3d {
    pub fn new(config: &Config) -> Self {
        let c = ffi::P3Config {
            max_bodies: config.max_bodies,
            velocity_iters: match config.iters {
                Iters::Default => 0,
                Iters::Eight => 8,
            },
            position_iters: 0,
            allow_sleep: config.sleep as i32,
        };
        // SAFETY: the shim copies what it needs from the config.
        let world = unsafe { ffi::p3_box3d_create(&c) };
        assert!(!world.is_null());
        Box3d { world, dynamic: Vec::new() }
    }
}

impl Drop for Box3d {
    fn drop(&mut self) {
        // SAFETY: the world came from p3_box3d_create and is dropped once.
        unsafe { ffi::p3_box3d_destroy(self.world) }
    }
}

impl Backend for Box3d {
    fn name(&self) -> String {
        "box3d".into()
    }

    fn solver(&self) -> String {
        // SAFETY: a live world.
        let substeps = unsafe { ffi::p3_box3d_substeps(self.world) };
        format!("soft step, {substeps} substeps (1 solve + 1 relax each)")
    }

    fn add(&mut self, bodies: &[Spec]) -> Vec<u32> {
        let specs = ffi::to_c(bodies);
        let mut handles = vec![0u32; bodies.len()];
        // SAFETY: both arrays hold bodies.len() elements.
        unsafe { ffi::p3_box3d_add(self.world, bodies.len() as u32, specs.as_ptr(), handles.as_mut_ptr()) };
        self.dynamic.extend(bodies.iter().zip(&handles).filter(|(s, _)| !s.fixed).map(|(_, h)| *h));
        handles
    }

    fn step(&mut self, dt: f32) {
        // SAFETY: a live world.
        unsafe { ffi::p3_box3d_step(self.world, dt) }
    }

    fn state(&self, out: &mut Vec<([f32; 3], [f32; 3])>) {
        let mut buf = vec![0f32; 6 * self.dynamic.len()];
        // SAFETY: every handle came from this world; buf holds 6 floats each.
        unsafe { ffi::p3_box3d_read(self.world, self.dynamic.len() as u32, self.dynamic.as_ptr(), buf.as_mut_ptr()) };
        ffi::unpack(&buf, out);
    }

    fn touching(&self) -> usize {
        // SAFETY: a live world.
        unsafe { ffi::p3_box3d_touching(self.world) as usize }
    }

    fn stages(&self) -> Vec<(&'static str, f64)> {
        let mut p = [0f32; 4];
        // SAFETY: a live world and room for the 4 floats the shim writes.
        unsafe { ffi::p3_box3d_profile(self.world, p.as_mut_ptr()) };
        let us = |ms: f32| ms as f64 * 1000.0;
        vec![("step", us(p[0])), ("pairs (broad)", us(p[1])), ("collide (narrow)", us(p[2])), ("solve", us(p[3]))]
    }
}
