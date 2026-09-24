//! Sleeping (opt in, with a `Sleep` entity): which bodies are asleep, and
//! the islands that fall asleep and wake together. See
//! docs/architecture/physics.md, "Sleeping".
//!
//! Kept in the mod's transient state, by entity index, not in the world: a
//! prototype. So a reload wakes everything, which is safe (sleeping bodies
//! are at rest) but not free, and a body despawned asleep stays counted in
//! `asleep` until its index is reused.

use engine_api::Entity;

#[derive(Clone, Copy, Default)]
struct Slot {
    generation: u32,
    live: bool,
    /// Seconds slower than `Sleep::speed`, while awake.
    still: f32,
    asleep: bool,
    /// The island it fell asleep in, woken together.
    island: u32,
}

#[derive(Default)]
pub struct Sleepers {
    slots: Vec<Slot>,
    islands: u32,
    /// Bodies asleep: when none are, the step skips every check.
    pub asleep: usize,
}

impl Sleepers {
    pub fn is_asleep(&self, e: Entity) -> bool {
        self.asleep > 0 && self.slots.get(e.index as usize).is_some_and(|s| s.live && s.generation == e.generation && s.asleep)
    }

    /// A body's slot, fresh if the index was another entity's.
    fn slot(&mut self, e: Entity) -> &mut Slot {
        let i = e.index as usize;
        if self.slots.len() <= i {
            self.slots.resize(i + 1, Slot::default());
        }
        let s = &mut self.slots[i];
        if !s.live || s.generation != e.generation {
            if s.live && s.asleep {
                self.asleep -= 1;
            }
            *s = Slot { generation: e.generation, live: true, ..Slot::default() };
        }
        s
    }

    pub fn wake_all(&mut self) {
        for s in &mut self.slots {
            (s.asleep, s.still) = (false, 0.0);
        }
        self.asleep = 0;
    }

    /// Wakes `e`'s island, if it's asleep: a kinematic body moving into it.
    pub fn wake(&mut self, e: Entity) {
        if self.is_asleep(e) {
            let island = self.slots[e.index as usize].island;
            self.wake_islands(&[island]);
        }
    }

    fn wake_islands(&mut self, islands: &[u32]) {
        for s in self.slots.iter_mut().filter(|s| s.live && s.asleep && islands.contains(&s.island)) {
            (s.asleep, s.still) = (false, 0.0);
            self.asleep -= 1;
        }
    }

    /// After a step: `awake` is every awake dynamic body and its speed, and
    /// `links` the pressed contacts between dynamic bodies. Updates how long
    /// each has been still, wakes the islands a moving body touched, and
    /// puts to sleep each island still for `time`. Returns the bodies that
    /// fell asleep, whose velocities the caller zeroes.
    pub fn update(&mut self, dt: f32, speed: f32, time: f32, awake: &[(Entity, f32)], links: &[(Entity, Entity)]) -> Vec<Entity> {
        for &(e, v) in awake {
            let s = self.slot(e);
            s.still = if v < speed { s.still + dt } else { 0.0 };
        }
        // Islands of awake bodies, by union-find over their indices here.
        let mut local = vec![u32::MAX; self.slots.len()];
        for (k, &(e, _)) in awake.iter().enumerate() {
            local[e.index as usize] = k as u32;
        }
        let at = |e: Entity| local.get(e.index as usize).copied().filter(|&k| k != u32::MAX);
        let mut parent: Vec<u32> = (0..awake.len() as u32).collect();
        fn root(parent: &mut [u32], mut k: u32) -> u32 {
            while parent[k as usize] != k {
                parent[k as usize] = parent[parent[k as usize] as usize];
                k = parent[k as usize];
            }
            k
        }
        let mut wake = Vec::new();
        for &(a, b) in links {
            match (at(a), at(b)) {
                (Some(i), Some(j)) => {
                    let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
                    parent[ri.max(rj) as usize] = ri.min(rj);
                }
                // An awake body against a sleeping one: if it's moving, the
                // sleeper's island wakes; if it's still, it waits to fall
                // asleep in its own island.
                (Some(i), None) | (None, Some(i)) => {
                    let other = if at(a).is_some() { b } else { a };
                    if self.slots[awake[i as usize].0.index as usize].still < time && self.is_asleep(other) {
                        wake.push(self.slots[other.index as usize].island);
                    }
                }
                (None, None) => {}
            }
        }
        wake.sort_unstable();
        wake.dedup();
        self.wake_islands(&wake);
        // An island sleeps when its stillest-for-least body has been still
        // for `time`.
        let mut least = vec![f32::INFINITY; awake.len()];
        for k in 0..awake.len() as u32 {
            let r = root(&mut parent, k) as usize;
            least[r] = least[r].min(self.slots[awake[k as usize].0.index as usize].still);
        }
        let mut island = vec![u32::MAX; awake.len()];
        let mut fell = Vec::new();
        for k in 0..awake.len() {
            let r = root(&mut parent, k as u32) as usize;
            if least[r] < time {
                continue;
            }
            if island[r] == u32::MAX {
                self.islands += 1;
                island[r] = self.islands;
            }
            let e = awake[k].0;
            let s = self.slot(e);
            (s.asleep, s.island) = (true, island[r]);
            self.asleep += 1;
            fell.push(e);
        }
        fell
    }
}
