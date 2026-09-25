//! Sleeping (on unless a `Sleep` entity turns it off): the islands that
//! fall asleep and wake together. See docs/architecture/physics.md, "Sleeping".
//!
//! The world says who's asleep: a sleeping body has `Asleep`, which puts
//! it in tables of its own. This is the mod's copy of that by entity index,
//! with each awake body's time spent still, so the step can ask about any
//! entity in O(1). It lives in the mod's transient part, so the step can
//! borrow it beside the state, and is handed to the next build through the
//! state at a reload (`Physics::unload`): so a reload forgets nothing,
//! not even how long awake bodies have been still. A build that starts
//! without one (the first, or one whose state was reset) rebuilds it from
//! the world, which knows who's asleep but not for how long.

use engine_api::{Entity, field_struct};

field_struct! {
    #[derive(Copy, Default)]
    struct Slot {
        generation: u32,
        live: bool,
        /// Seconds slower than `Sleep::speed`, while awake.
        still: f32,
        asleep: bool,
        /// The island it fell asleep in, woken together.
        island: u32,
    }
}

field_struct! {
    #[derive(Default)]
    pub struct Sleepers {
        slots: Vec<Slot>,
        /// The last island made: ids must stay unique across reloads, since a
        /// sleeping body's is in the world.
        islands: u32,
        /// Bodies asleep, as this copy has them.
        pub asleep: u64,
        /// Bodies woken since the step last moved them out of their sleeping
        /// tables (`Physics::move_woken`).
        pub woken: Vec<Entity>,
    }
}

impl Sleepers {
    pub fn is_asleep(&self, e: Entity) -> bool {
        self.asleep > 0 && self.slots.get(e.index as usize).is_some_and(|s| s.live && s.generation == e.generation && s.asleep)
    }

    /// A body's slot, fresh if the index was another entity's. One asleep
    /// there was despawned, and wakes its island as `wake_missing` would
    /// have: a spawn can reuse its index in the step it goes, before
    /// anything looked for it missing.
    fn slot(&mut self, e: Entity) -> &mut Slot {
        let i = e.index as usize;
        if self.slots.len() <= i {
            self.slots.resize(i + 1, Slot::default());
        }
        let was = self.slots[i];
        if was.live && was.generation != e.generation && was.asleep {
            self.slots[i].asleep = false;
            self.asleep -= 1;
            self.woken.push(Entity { index: e.index, generation: was.generation });
            self.wake_islands(&[was.island]);
        }
        let s = &mut self.slots[i];
        if !s.live || s.generation != e.generation {
            *s = Slot { generation: e.generation, live: true, ..Slot::default() };
        }
        s
    }

    /// `e` is asleep in `island`, as the world says: at load, or when a
    /// game put it to sleep.
    pub fn adopt(&mut self, e: Entity, island: u32) {
        let s = self.slot(e);
        if !s.asleep {
            (s.asleep, s.island) = (true, island);
            self.asleep += 1;
        }
        self.islands = self.islands.max(island);
    }

    /// Every sleeping body, woken.
    pub fn wake_all(&mut self) {
        for s in self.slots.iter_mut().filter(|s| s.live && s.asleep) {
            (s.asleep, s.still) = (false, 0.0);
        }
        self.asleep = 0;
    }

    /// Wakes `e`'s island, if it's asleep: a kinematic body moving into it,
    /// a game setting its velocity, or its support gone.
    pub fn wake(&mut self, e: Entity) {
        if self.is_asleep(e) {
            let island = self.slots[e.index as usize].island;
            self.wake_islands(&[island]);
        }
    }

    /// Wakes the islands of every body this copy has asleep that `alive`
    /// (by entity) doesn't: despawned, or woken by a game removing its
    /// `Asleep`. The bodies they rest on or under may no longer be where
    /// they were.
    pub fn wake_missing(&mut self, alive: impl Fn(Entity) -> bool) {
        let mut islands: Vec<u32> = Vec::new();
        for (i, s) in self.slots.iter_mut().enumerate() {
            if s.live && s.asleep && !alive(Entity { index: i as u32, generation: s.generation }) {
                islands.push(s.island);
                (s.asleep, s.live) = (false, false);
                self.asleep -= 1;
                self.woken.push(Entity { index: i as u32, generation: s.generation });
            }
        }
        islands.sort_unstable();
        islands.dedup();
        self.wake_islands(&islands);
    }

    /// `islands` sorted.
    fn wake_islands(&mut self, islands: &[u32]) {
        if islands.is_empty() {
            return;
        }
        for (i, s) in self.slots.iter_mut().enumerate() {
            if s.live && s.asleep && islands.binary_search(&s.island).is_ok() {
                (s.asleep, s.still) = (false, 0.0);
                self.asleep -= 1;
                self.woken.push(Entity { index: i as u32, generation: s.generation });
            }
        }
    }

    /// After a step: `awake` is every awake dynamic body and its speed, and
    /// `links` the pressed contacts between dynamic bodies. Updates how long
    /// each has been still, wakes the islands a moving body touched, and
    /// puts to sleep each island still for `time`. Returns the bodies that
    /// fell asleep and their islands, which the caller stops and moves to
    /// their sleeping tables.
    pub fn update(&mut self, dt: f32, speed: f32, time: f32, awake: &[(Entity, f32)], links: &[(Entity, Entity)]) -> Vec<(Entity, u32)> {
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
            fell.push((e, island[r]));
        }
        fell
    }
}
