//! How soon a scene comes to rest in an engine, and how it stands then:
//! what the comparison's `SETTLE` prints and the quality tests
//! (`quality_test.rs`) bound, measured by the same code for every engine.

use std::time::Instant;

use crate::quality::{self, Quality};
use crate::scene::Scene;
use crate::{Dyn, Sim};

/// Below this every body counts as at rest: the sleep threshold of ours
/// and of Box2D.
pub const REST: f32 = 0.05;
/// Steps between looks: rest is known to within this.
pub const EVERY: u32 = 10;
/// The steps at the end `Settling::energy_tail` looks over.
pub const TAIL: u32 = 200;

#[derive(Clone, Debug, Default)]
pub struct Settling {
    pub label: String,
    /// The first look with every body under `REST`, and the look it stayed
    /// so from to the end; `None`, never.
    pub first_rest: Option<u32>,
    pub rest_from: Option<u32>,
    /// The fastest body at steps 100, 200 and 400.
    pub fastest: Vec<f32>,
    /// The deepest overlap at any look, and the greatest mean overlap:
    /// sinking while it settles, landing included.
    pub deepest_during: f32,
    pub mean_during: f64,
    /// At step 400, and at the end.
    pub at400: Quality,
    pub end: Quality,
    /// The most energy a body at any look in the last `TAIL` steps: where
    /// a scene sways, the energy at the end is where in the swing it ended.
    pub energy_tail: f64,
    /// Stacks and pyramids: how far the top box (the last body) is from
    /// where it began, at the end.
    pub top_moved: Option<f32>,
    /// Wall time a step, µs.
    pub us: f64,
}

fn start(scene: &Scene) -> Vec<Dyn> {
    let dynamic = scene.build().into_iter().filter(|s| s.dynamic);
    dynamic.map(|s| Dyn { circle: s.circle, hx: s.hx, hy: s.hy, x: s.x, y: s.y, vx: 0.0, vy: 0.0, angle: 0.0, w: 0.0 }).collect()
}

/// `sim` stepped to `max`, looked at every `EVERY` steps.
pub fn settle(sim: &mut dyn Sim, scene: &Scene, turning: bool, max: u32) -> Settling {
    let mut s = Settling { label: sim.label(), ..Settling::default() };
    let (mut wall, mut step) = (0.0, 0);
    while step < max {
        let t = Instant::now();
        sim.step(EVERY);
        wall += t.elapsed().as_secs_f64();
        step += EVERY;
        let bodies = sim.bodies();
        let top = bodies.iter().map(Dyn::speed).fold(0.0, f32::max);
        if top < REST {
            s.rest_from.get_or_insert(step);
            s.first_rest.get_or_insert(step);
        } else {
            if s.rest_from.is_some() && std::env::var_os("TRACE").is_some() {
                let (i, b) = bodies.iter().enumerate().max_by(|a, b| a.1.speed().total_cmp(&b.1.speed())).unwrap();
                eprintln!("{}: moving again at {step}: body {i} at {:.2}, {:.2} at {top:.3}", s.label, b.x, b.y);
            }
            s.rest_from = None;
        }
        if [100, 200, 400].contains(&step) {
            s.fastest.push(top);
        }
        let q = quality::measure(scene, &bodies, turning);
        s.deepest_during = s.deepest_during.max(q.max_depth);
        s.mean_during = s.mean_during.max(q.mean_depth);
        if step == 400 {
            s.at400 = q;
        }
        if step + TAIL > max {
            s.energy_tail = s.energy_tail.max(q.energy);
        }
        if step >= max {
            s.end = q;
        }
    }
    if matches!(scene, Scene::Pyramid { .. } | Scene::Stack { .. }) {
        let (a, b) = (start(scene), sim.bodies());
        let (a, b) = (a.last().unwrap(), b.last().unwrap());
        s.top_moved = Some((b.x - a.x).hypot(b.y - a.y));
    }
    s.us = wall * 1e6 / step.max(1) as f64;
    s
}
