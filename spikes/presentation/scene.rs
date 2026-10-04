//! SPIKE (get-3hd.1): `spike_scene`, the game: drawables (`Place`,
//! `Look`) and a system that moves a share of them each frame, a
//! different share each frame, so change detection has something to find.
//!
//! - `spawn <n>`: replaces the scene with `n` drawables, grouped by
//!   material (so the extract's order is already batched);
//! - `churn <percent>`: how many move a frame (default 1);
//! - `pattern spread|block`: the movers spread evenly over the rows,
//!   found by a walk of them all (default), or one block of rows,
//!   reached by entity;
//! - `count`: how many there are.
//!
//! Two builds (`scene_a`, `scene_b`) differ in how things move, so a
//! reload shows on screen.

use engine_api::{Cx, Entity, Mod, Query, Systems, export_mod, phase};
use spike_draw::{Look, Place};

engine_api::mod_state! {
    #[derive(Default)]
    struct Scene {
        frame: u64,
        churn: u32,
        /// Whether the rows that move are one block, not spread.
        block: bool,
        spawned: Vec<Entity>,
        width: f32,
        height: f32,
    }
}

#[cfg(not(feature = "b"))]
const BUILD: &str = "a: drifting right";
#[cfg(feature = "b")]
const BUILD: &str = "b: swirling";

impl Scene {
    fn animate(&mut self, _: &mut (), _: &mut Cx, mut q: Query<&mut Place>) {
        self.frame += 1;
        let churn = if self.churn == 0 { 1 } else { self.churn.min(100) } as u64;
        let (f, w) = (self.frame, self.width.max(1.0));
        #[cfg(feature = "b")]
        let h = self.height.max(1.0);
        let n = self.spawned.len().max(1) as u64;
        let k = (n * churn / 100).max(1);
        #[cfg(not(feature = "b"))]
        let step = move |p: &mut Place| p.x = (p.x + 3.0) % w;
        #[cfg(feature = "b")]
        let step = move |p: &mut Place| {
            let (cx, cy) = (w / 2.0, h / 2.0);
            let (dx, dy) = (p.x - cx, p.y - cy);
            let (s, c) = (0.05f32).sin_cos();
            p.x = cx + dx * c - dy * s;
            p.y = cy + dx * s + dy * c;
        };
        if self.block {
            // One block of rows, reached by entity: only their pages are
            // written, where a walk with `&mut Place` writes every page.
            let start = ((f * k) % n) as usize;
            for j in 0..k as usize {
                let e = self.spawned[(start + j) % n as usize];
                q.with(e, |_, mut p| step(&mut p));
            }
            return;
        }
        // Spread: every hundredth row moves (more, for more churn), others
        // each frame, so every page has some.
        let mut i = 0u64;
        q.for_each(|_, mut p| {
            if (i + f) % 100 < churn {
                step(&mut p);
            }
            i += 1;
        });
    }
}

impl Mod for Scene {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("animate", Self::animate).phase(phase::UPDATE);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        cx.log(format!("scene build {BUILD}"));
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let words: Vec<&str> = message.split_whitespace().collect();
        let number = |w: &str| w.parse::<u32>().map_err(|_| format!("not a number: {w:?}"));
        match words[..] {
            ["spawn", n] => self.spawn(cx, number(n)? as usize, 1280.0, 720.0),
            ["churn", p] => self.churn = number(p)?,
            ["pattern", "block"] => self.block = true,
            ["pattern", "spread"] => self.block = false,
            ["count"] => {}
            ["build"] => return Ok(BUILD.into()),
            _ => return Err("usage: spawn <n> | churn <percent> | pattern spread|block | count | build".into()),
        }
        Ok(format!("{} drawables, churn {}%", self.spawned.len(), self.churn.max(1)))
    }

    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        for e in self.spawned.drain(..) {
            world.despawn(e);
        }
    }
}

impl Scene {
    fn spawn(&mut self, cx: &mut Cx, n: usize, w: f32, h: f32) {
        let mut world = cx.world();
        for e in self.spawned.drain(..) {
            world.despawn(e);
        }
        (self.width, self.height) = (w, h);
        let mut s: u64 = 0x9e37_79b9_7f4a_7c15;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        };
        for i in 0..n {
            let size = 4.0 + (next() % 13) as f32;
            let place = Place { x: (next() % w as u64) as f32, y: (next() % h as u64) as f32 };
            let look = Look {
                w: size,
                h: size,
                colour: (next() as u32) | 0x8000_0000,
                shape: (i % 2) as u32,
                material: (i * 4 / n.max(1)) as u32,
            };
            self.spawned.push(world.spawn((place, look)));
        }
    }
}

export_mod!(Scene);
