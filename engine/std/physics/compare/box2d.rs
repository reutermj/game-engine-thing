//! Box2D v3 through `box2d_shim.c`: the only FFI in the comparison, and
//! the only unsafe code, kept here. Every call passes plain numbers or a
//! pointer to a buffer the Rust side owns for the call.

use std::collections::VecDeque;

use crate::scene::{DT, GRAVITY, Scene, Spec};
use crate::{Dyn, Sim};

#[repr(C)]
struct World {
    _opaque: [u8; 0],
}

unsafe extern "C" {
    fn bx_new(gx: f32, gy: f32, sleep: i32, continuous: i32) -> *mut World;
    fn bx_free(w: *mut World);
    fn bx_add(w: *mut World, dynamic: i32, circle: i32, x: f32, y: f32, hx: f32, hy: f32, friction: f32, restitution: f32) -> i32;
    fn bx_set_velocity(w: *mut World, handle: i32, vx: f32, vy: f32);
    fn bx_remove(w: *mut World, handle: i32);
    fn bx_step(w: *mut World, dt: f32, substeps: i32);
    fn bx_state(w: *const World, handle: i32, out: *mut f32);
    fn bx_profile(w: *const World, out: *mut f32, len: i32) -> i32;
    fn bx_counters(w: *const World, out: *mut i32, len: i32) -> i32;
}

/// `b2Profile`'s fields, in its declaration order (types.h, v3.1.1): all
/// milliseconds for the last step.
const PROFILE: [&str; 22] = [
    "step",
    "pairs",
    "collide",
    "solve",
    "mergeIslands",
    "prepareStages",
    "solveConstraints",
    "prepareConstraints",
    "integrateVelocities",
    "warmStart",
    "solveImpulses",
    "integratePositions",
    "relaxImpulses",
    "applyRestitution",
    "storeImpulses",
    "splitIslands",
    "transforms",
    "hitEvents",
    "refit",
    "bullets",
    "sleepIslands",
    "sensors",
];
/// `b2Counters`: body, shape, contact, joint and island counts, then stack
/// and tree figures, the task count, and constraints per graph color (12).
const COUNTERS: usize = 10 + 12;

pub struct Box2d {
    world: *mut World,
    label: String,
    substeps: i32,
    /// Handles of the dynamic bodies, in the order they were added.
    dynamic: VecDeque<(i32, Spec)>,
    alive: usize,
    /// Per `PROFILE` field, milliseconds summed since `reset`.
    profile: [f64; 22],
}

impl Box2d {
    /// `substeps` is Box2D's own: 4 by default, as its samples and
    /// benchmarks step.
    pub fn new(scene: &Scene, substeps: i32, sleep: bool) -> Box2d {
        // Continuous collision is left on, Box2D's default: it only acts for
        // bodies moving fast against statics, which nothing here does after
        // the first frames of rain.
        // SAFETY: plain values in, a world the shim allocated out, freed in Drop.
        let world = unsafe { bx_new(0.0, GRAVITY, sleep as i32, 1) };
        let label = if substeps == 4 { "Box2D".to_string() } else { format!("Box2D, {substeps} substeps") };
        let mut b = Box2d { world, label, substeps, dynamic: VecDeque::new(), alive: 0, profile: [0.0; 22] };
        let mut n = [0f32; 22];
        // SAFETY: `n` is 22 floats long and the shim writes at most `len`.
        let have = unsafe { bx_profile(world, n.as_mut_ptr(), 22) };
        assert_eq!(have, 22, "b2Profile changed shape: update PROFILE");
        let mut c = [0i32; COUNTERS];
        // SAFETY: as above, `c` is COUNTERS ints.
        let have = unsafe { bx_counters(world, c.as_mut_ptr(), COUNTERS as i32) };
        assert_eq!(have as usize, COUNTERS, "b2Counters changed shape");
        for s in scene.build() {
            b.add(&s);
        }
        if sleep {
            b.label += ", sleeping";
        }
        b
    }

    fn add(&mut self, s: &Spec) {
        // SAFETY: the world is live; the rest are plain values.
        let h = unsafe { bx_add(self.world, s.dynamic as i32, s.circle as i32, s.x, s.y, s.hx, s.hy, s.friction, s.restitution) };
        if s.dynamic {
            // SAFETY: `h` is the handle just returned.
            unsafe { bx_set_velocity(self.world, h, s.vx, s.vy) };
            self.dynamic.push_back((h, *s));
            self.alive += 1;
        }
    }

    fn counters(&self) -> [i32; COUNTERS] {
        let mut c = [0i32; COUNTERS];
        // SAFETY: `c` is COUNTERS ints.
        unsafe { bx_counters(self.world, c.as_mut_ptr(), COUNTERS as i32) };
        c
    }

    fn ms(&self, field: &str) -> f64 {
        self.profile[PROFILE.iter().position(|f| *f == field).unwrap()]
    }
}

impl Drop for Box2d {
    fn drop(&mut self) {
        // SAFETY: made by bx_new, freed once.
        unsafe { bx_free(self.world) };
    }
}

impl Sim for Box2d {
    fn label(&self) -> String {
        self.label.clone()
    }

    fn step(&mut self, n: u32) {
        let mut p = [0f32; 22];
        for _ in 0..n {
            // SAFETY: the world is live, `p` is 22 floats.
            unsafe {
                bx_step(self.world, DT, self.substeps);
                bx_profile(self.world, p.as_mut_ptr(), 22);
            }
            for (sum, ms) in self.profile.iter_mut().zip(p) {
                *sum += ms as f64;
            }
        }
    }

    fn rain(&mut self, arrivals: &[Spec], alive: usize) {
        for s in arrivals {
            self.add(s);
            if self.alive > alive {
                let (h, _) = self.dynamic.pop_front().unwrap();
                // SAFETY: a live handle, removed once.
                unsafe { bx_remove(self.world, h) };
                self.alive -= 1;
            }
        }
    }

    fn bodies(&self) -> Vec<Dyn> {
        let mut out = [0f32; 5];
        self.dynamic
            .iter()
            .map(|(h, s)| {
                // SAFETY: a live handle; `out` is 5 floats.
                unsafe { bx_state(self.world, *h, out.as_mut_ptr()) };
                Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: out[0], y: out[1], vx: out[2], vy: out[3], angle: out[4] }
            })
            .collect()
    }

    fn reset(&mut self) {
        self.profile = [0.0; 22];
    }

    /// Box2D's pairs (new pairs from moved proxies) and refit (growing the
    /// moved ones in the tree) as the broadphase; collide (every contact's
    /// manifold) as the narrowphase; the constraint stages as the solver.
    fn stages(&self) -> Vec<(String, f64)> {
        let us = |f: &str| self.ms(f) * 1e3;
        let mut v = vec![
            ("broadphase".to_string(), us("pairs") + us("refit")),
            ("narrowphase".to_string(), us("collide")),
            ("solver".to_string(), us("prepareStages") + us("solveConstraints")),
        ];
        for f in PROFILE.iter().skip(1) {
            v.push((format!("b2:{f}"), us(f)));
        }
        v.push(("engine step".to_string(), us("step")));
        v
    }

    /// Contacts in the constraint graph: touching, with a manifold.
    fn contacts(&self) -> usize {
        self.counters()[10..].iter().map(|&c| c as usize).sum()
    }

    fn native(&self) -> String {
        let c = self.counters();
        format!(
            "contacts {} (touching {}), islands {}, colors used {}",
            c[2],
            self.contacts(),
            c[4],
            c[10..].iter().filter(|&&n| n > 0).count()
        )
    }
}
