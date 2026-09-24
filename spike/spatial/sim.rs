//! Scenes and the step, over any layout. The step is the physics mod's:
//! gravity, the layout's broadphase, the real narrowphase and solver
//! (compiled from engine/std/physics), then positions written row by row,
//! which is what dirties them, then upkeep, then region queries. Contacts
//! are solved in pair order and bodies in id order, so every layout must
//! produce the same simulation, bit for bit.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use physics::{Aabb, Placed, Shape, Vec2};

use crate::layout::{Layout, PAYLOAD, Row, Visit};
use crate::narrow::{MARGIN, collide};
use crate::solver::{Constraint, SolverBody, solve};

pub const DT: f32 = 1.0 / 60.0;
const QUERIES: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    /// Bodies dropped into a walled box: the solver, and bodies crossing
    /// cells as they fall and settle.
    Pile,
    /// The platformer's level: 80 static tiles, two tall walls, a running
    /// player and a pacing walker.
    Platformer,
    /// Many bodies moving freely without contacts: upkeep and queries at
    /// scale.
    Drift,
}

pub struct Scene {
    pub kind: Kind,
    pub rows: Vec<Row>,
    pub gravity: Vec2,
    /// The region queries sample.
    pub area: Aabb,
    pub respawn: Option<(u32, Vec2)>,
}

fn row(id: u32, x: f32, y: f32, hx: f32, hy: f32, circle: bool, inv_mass: f32) -> Row {
    Row {
        id,
        pos: Vec2::new(x, y),
        half: Vec2::new(hx, hy),
        circle,
        vel: Vec2::ZERO,
        inv_mass,
        payload: [id as u64; PAYLOAD],
    }
}

pub fn pile(n: u32) -> Scene {
    let (w, h) = (40.0, 30.0);
    let mut rows = vec![
        row(0, w / 2.0, h + 0.5, w / 2.0 + 1.0, 0.5, false, 0.0),
        row(1, -0.5, h / 2.0, 0.5, h, false, 0.0),
        row(2, w + 0.5, h / 2.0, 0.5, h, false, 0.0),
    ];
    let per_row = ((w - 2.0) / 1.2) as u32;
    for k in 0..n {
        let (col, r) = (k % per_row, k / per_row);
        let jitter = ((k * 7919) % 100) as f32 / 100.0 * 0.2 - 0.1;
        rows.push(row(3 + k, 1.5 + col as f32 * 1.2 + jitter, h - 1.0 - r as f32 * 1.2, 0.45, 0.45, k % 2 == 0, 1.0));
    }
    Scene {
        kind: Kind::Pile,
        rows,
        gravity: Vec2::new(0.0, 20.0),
        area: Aabb { min: Vec2::new(0.0, -15.0), max: Vec2::new(w, h) },
        respawn: None,
    }
}

pub fn platformer() -> Scene {
    let map: Vec<&str> = include_str!("../../platformer/level/map.txt").lines().collect();
    let (w, h) = (map[0].len() as f32, map.len() as f32);
    let mut rows = vec![
        row(0, -0.5, h / 2.0, 0.5, h * 4.0, false, 0.0),
        row(1, w + 0.5, h / 2.0, 0.5, h * 4.0, false, 0.0),
    ];
    let (mut player, mut spawn) = (None, Vec2::ZERO);
    for (y, line) in map.iter().enumerate() {
        for (x, c) in line.chars().enumerate() {
            let (cx, cy) = (x as f32 + 0.5, y as f32 + 0.5);
            let id = rows.len() as u32;
            match c {
                '#' => rows.push(row(id, cx, cy, 0.5, 0.5, false, 0.0)),
                'E' => rows.push(row(id, cx, cy, 0.5, 0.5, false, 1.0)),
                'P' => {
                    rows.push(row(id, cx, cy, 0.4, 0.475, false, 1.0));
                    (player, spawn) = (Some(id), Vec2::new(cx, cy));
                }
                _ => {}
            }
        }
    }
    Scene {
        kind: Kind::Platformer,
        rows,
        gravity: Vec2::new(0.0, 40.0),
        area: Aabb { min: Vec2::ZERO, max: Vec2::new(w, h) },
        respawn: player.map(|p| (p, spawn)),
    }
}

/// A deterministic stream in `[0, 1)`.
fn lcg(state: &mut u64) -> f32 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*state >> 40) as f32 / (1u64 << 24) as f32
}

pub fn drift(n: u32) -> Scene {
    let size = (n as f32).sqrt() * 2.0;
    let mut s = 7;
    let rows = (0..n)
        .map(|id| {
            let mut r = row(id, lcg(&mut s) * size, lcg(&mut s) * size, 0.45, 0.45, true, 1.0);
            r.vel = Vec2::new(lcg(&mut s) * 10.0 - 5.0, lcg(&mut s) * 10.0 - 5.0);
            r
        })
        .collect();
    Scene {
        kind: Kind::Drift,
        rows,
        gravity: Vec2::ZERO,
        area: Aabb { min: Vec2::ZERO, max: Vec2::new(size, size) },
        respawn: None,
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Timing {
    pub frames: u32,
    pub pairs: Duration,
    pub upkeep: Duration,
    pub queries: Duration,
    /// Upkeep after a one-row write (a respawn): what automatic re-sorting
    /// after every writer costs a small writer.
    pub small_upkeep: Duration,
    pub small_writes: u32,
    pub moved: usize,
    pub candidates: usize,
    pub pair_visit: Visit,
    pub query_visit: Visit,
    pub hits: usize,
}

pub struct Sim {
    pub kind: Kind,
    gravity: Vec2,
    area: Aabb,
    respawn: Option<(u32, Vec2)>,
    movers: Vec<u32>,
    /// Solver slot per id; statics share the last.
    slot: Vec<u32>,
    cache: BTreeMap<(u32, u32), (f32, f32)>,
    frame: u32,
    query_seed: u64,
}

impl Sim {
    pub fn new(scene: &Scene) -> Sim {
        let mut movers: Vec<u32> = scene.rows.iter().filter(|r| r.moves()).map(|r| r.id).collect();
        movers.sort_unstable();
        let mut slot = vec![movers.len() as u32; scene.rows.len()];
        for (i, &id) in movers.iter().enumerate() {
            slot[id as usize] = i as u32;
        }
        Sim {
            kind: scene.kind,
            gravity: scene.gravity,
            area: scene.area,
            respawn: scene.respawn,
            movers,
            slot,
            cache: BTreeMap::new(),
            frame: 0,
            query_seed: 11,
        }
    }

    /// The regions this frame's queries probe: player-sized boxes spread
    /// over the scene, the same for every layout.
    pub fn regions(&mut self) -> Vec<Aabb> {
        let a = self.area;
        (0..QUERIES)
            .map(|_| {
                let c = Vec2::new(
                    a.min.x + lcg(&mut self.query_seed) * (a.max.x - a.min.x),
                    a.min.y + lcg(&mut self.query_seed) * (a.max.y - a.min.y),
                );
                Aabb { min: c - Vec2::new(2.0, 2.0), max: c + Vec2::new(2.0, 2.0) }
            })
            .collect()
    }

    pub fn step(&mut self, l: &mut dyn Layout, t: &mut Timing) {
        self.frame += 1;
        t.frames += 1;
        if self.kind == Kind::Drift {
            // No contacts to solve, but the broadphase's cost at scale is
            // the question.
            let mut pairs = Vec::new();
            let start = Instant::now();
            t.pair_visit += l.pairs(MARGIN, &mut pairs);
            t.pairs += start.elapsed();
            t.candidates += pairs.len();
            self.drift(l);
        } else {
            self.physics(l, t);
        }
        let start = Instant::now();
        t.moved += l.upkeep().moved;
        t.upkeep += start.elapsed();

        if let Some((player, spawn)) = self.respawn.filter(|_| self.frame % 60 == 0) {
            let r = l.get_mut(player);
            (r.pos, r.vel) = (spawn, Vec2::ZERO);
            let start = Instant::now();
            t.moved += l.upkeep().moved;
            t.small_upkeep += start.elapsed();
            t.small_writes += 1;
        }

        let regions = self.regions();
        let mut out = Vec::new();
        let start = Instant::now();
        for r in &regions {
            t.query_visit += l.query(r, &mut out);
        }
        t.queries += start.elapsed();
        t.hits += out.len();
    }

    fn drift(&mut self, l: &mut dyn Layout) {
        let a = self.area;
        for &id in &self.movers {
            let r = l.get_mut(id);
            r.pos += r.vel * DT;
            if r.pos.x < a.min.x || r.pos.x > a.max.x {
                r.vel.x = -r.vel.x;
            }
            if r.pos.y < a.min.y || r.pos.y > a.max.y {
                r.vel.y = -r.vel.y;
            }
        }
    }

    fn physics(&mut self, l: &mut dyn Layout, t: &mut Timing) {
        let (g, frame, kind) = (self.gravity, self.frame, self.kind);
        l.each_velocity(&mut |r| {
            if r.moves() {
                r.vel += g * DT;
                // The platformer's two movers pace back and forth.
                if kind == Kind::Platformer {
                    let speed = if r.half.x < 0.45 { 7.0 } else { 3.0 };
                    let period = if r.half.x < 0.45 { 90 } else { 150 };
                    r.vel.x = if (frame / period) % 2 == 0 { speed } else { -speed };
                }
            }
        });

        let mut pairs = Vec::new();
        let start = Instant::now();
        t.pair_visit += l.pairs(MARGIN, &mut pairs);
        t.pairs += start.elapsed();
        t.candidates += pairs.len();

        let placed = |r: &Row| Placed {
            shape: if r.circle { Shape::Circle(r.half.x) } else { Shape::Box(r.half) },
            at: r.pos,
        };
        let mut contacts = Vec::new();
        let mut keys = Vec::new();
        for (a, b) in pairs {
            let (ra, rb) = (*l.get(a), *l.get(b));
            let Some(m) = collide(&placed(&ra), &placed(&rb), rb.vel - ra.vel) else { continue };
            let (jn, jt) = self.cache.get(&(a, b)).copied().unwrap_or_default();
            contacts.push(Constraint {
                a: self.slot[a as usize],
                b: self.slot[b as usize],
                normal: m.normal,
                depth: m.depth,
                friction: 0.3,
                restitution: 0.1,
                jn,
                jt,
                speed: 0.0,
            });
            keys.push((a, b));
        }
        let mut bodies: Vec<SolverBody> = self
            .movers
            .iter()
            .map(|&id| {
                let r = l.get(id);
                SolverBody { v: r.vel, inv_mass: r.inv_mass, pseudo: Vec2::ZERO }
            })
            .collect();
        bodies.push(SolverBody::default());
        solve(&mut bodies, &mut contacts, DT);
        self.cache = keys.into_iter().zip(&contacts).map(|(k, c)| (k, (c.jn, c.jt))).collect();
        for (&id, b) in self.movers.iter().zip(&bodies) {
            let r = l.get_mut(id);
            r.vel = b.v;
            r.pos += (b.v + b.pseudo) * DT;
        }
    }
}
