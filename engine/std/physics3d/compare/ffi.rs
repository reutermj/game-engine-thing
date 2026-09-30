//! The C ABI of shim.h, mirrored by hand: change both together.

use crate::{Shape, Spec, State};

#[repr(C)]
pub struct P3Spec {
    shape: i32,
    dims: [f32; 3],
    pos: [f32; 3],
    vel: [f32; 3],
    fixed: i32,
    rot: [f32; 4],
    friction: f32,
    restitution: f32,
    mass: f32,
}

impl From<&Spec> for P3Spec {
    fn from(s: &Spec) -> Self {
        let (shape, dims) = match s.shape {
            Shape::Sphere(r) => (0, [r, 0.0, 0.0]),
            Shape::Box(h) => (1, h),
        };
        P3Spec {
            shape,
            dims,
            pos: s.pos,
            vel: s.vel,
            fixed: s.fixed as i32,
            rot: s.rot,
            friction: s.friction,
            restitution: s.restitution,
            mass: s.mass,
        }
    }
}

#[repr(C)]
pub struct P3Config {
    pub max_bodies: u32,
    pub velocity_iters: i32,
    pub position_iters: i32,
    pub allow_sleep: i32,
    pub rotate: i32,
    pub gravity: [f32; 3],
}

#[repr(C)]
pub struct JoltWorld {
    _opaque: [u8; 0],
}

#[repr(C)]
pub struct Box3dWorld {
    _opaque: [u8; 0],
}

unsafe extern "C" {
    pub fn p3_jolt_create(config: *const P3Config) -> *mut JoltWorld;
    pub fn p3_jolt_destroy(world: *mut JoltWorld);
    pub fn p3_jolt_add(world: *mut JoltWorld, n: u32, specs: *const P3Spec, out_handles: *mut u32);
    pub fn p3_jolt_step(world: *mut JoltWorld, dt: f32);
    pub fn p3_jolt_read(world: *const JoltWorld, n: u32, handles: *const u32, out: *mut f32);
    pub fn p3_jolt_touching(world: *const JoltWorld) -> u32;
    pub fn p3_jolt_iterations(world: *const JoltWorld, velocity: *mut i32, position: *mut i32);

    pub fn p3_box3d_create(config: *const P3Config) -> *mut Box3dWorld;
    pub fn p3_box3d_destroy(world: *mut Box3dWorld);
    pub fn p3_box3d_add(world: *mut Box3dWorld, n: u32, specs: *const P3Spec, out_handles: *mut u32);
    pub fn p3_box3d_step(world: *mut Box3dWorld, dt: f32);
    pub fn p3_box3d_read(world: *const Box3dWorld, n: u32, handles: *const u32, out: *mut f32);
    pub fn p3_box3d_touching(world: *const Box3dWorld) -> u32;
    pub fn p3_box3d_substeps(world: *const Box3dWorld) -> i32;
    pub fn p3_box3d_profile(world: *const Box3dWorld, out4: *mut f32);
}

/// Marshals specs for a shim's add call.
pub fn to_c(bodies: &[Spec]) -> Vec<P3Spec> {
    bodies.iter().map(P3Spec::from).collect()
}

/// Floats a shim's read writes per body.
pub const STATE_FLOATS: usize = 13;

/// Unpacks a shim's read buffer (position, velocity, rotation, angular
/// velocity).
pub fn unpack(buf: &[f32], out: &mut Vec<State>) {
    out.clear();
    out.extend(buf.as_chunks::<STATE_FLOATS>().0.iter().map(|c| State {
        pos: [c[0], c[1], c[2]],
        vel: [c[3], c[4], c[5]],
        rot: [c[6], c[7], c[8], c[9]],
        ang: [c[10], c[11], c[12]],
    }));
}
