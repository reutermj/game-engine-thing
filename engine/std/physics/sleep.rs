//! Sleeping (on unless a `Sleep` entity turns it off): the islands that
//! fall asleep and wake together. See docs/architecture/physics.md, "Sleeping".
//!
//! Everything sleeping keeps is in the world: `Asleep` on a sleeping body
//! (tables of its own), `Slept` beside it (physics's record that it took
//! the body as asleep, and in which island), `Still` on an awake body
//! slower than the threshold, and `Resting` on a contact at rest. What's
//! here is a step's arithmetic over them: how many steps still is enough,
//! the islands awake bodies make, and the list of bodies woken in a system,
//! which its apply node moves. So a reload hands nothing over.

use engine_api::Entity;

/// What a system has woken, for its apply node: islands, as the waking is
/// seen, then the bodies in them (`Physics::resolve`).
#[derive(Default)]
pub struct Sleepers {
    /// Islands woken since the last `resolve`.
    pub waking: Vec<u32>,
    /// Bodies woken since the step last moved them out of their sleeping
    /// tables (`Physics::move_woken`).
    pub woken: Vec<Entity>,
    /// `enough`'s last answer, and the `dt` and `time` it was for.
    enough_for: u64,
    enough: u64,
}

impl Sleepers {
    /// The steps of `dt` a body must be still for to have been still for
    /// `time` seconds, as they were counted when a body kept its seconds:
    /// the first sum of that many `dt`s, added one at a time in `f32`, that
    /// reaches `time`. So a body falls asleep in the step it did then, where
    /// `time / dt` can be a step early (0.5 s at 120 a second is 61 steps).
    /// Never, if the sum stops growing first. Kept, being a loop.
    pub fn enough(&mut self, dt: f32, time: f32) -> u64 {
        let key = ((dt.to_bits() as u64) << 32) | time.to_bits() as u64;
        if self.enough_for != key || self.enough == 0 {
            let (mut sum, mut n) = (0.0f32, 0u64);
            while sum < time {
                let next = sum + dt;
                if next == sum {
                    n = u64::MAX;
                    break;
                }
                (sum, n) = (next, n + 1);
            }
            (self.enough_for, self.enough) = (key, n.max(1));
        }
        self.enough
    }
}

/// After a step: `awake` is every awake dynamic body and the steps it has
/// been still for, this one included (0 if it's moving: see
/// `physics::Still`); `between` the pressed contacts between two of them,
/// and `against` those between one of them and a sleeping body, with the
/// sleeper's island. Returns the islands a moving body pressed on, to
/// wake, and the bodies that fell asleep, each island still for `enough`
/// steps, numbered after `last`.
pub fn islands(
    enough: u64,
    awake: &[(Entity, u64)],
    between: &[(Entity, Entity)],
    against: &[(Entity, u32)],
    last: &mut u32,
) -> (Vec<u32>, Vec<(Entity, u32)>) {
    // Islands of awake bodies, by union-find over their indices here.
    let len = awake.iter().map(|(e, _)| e.index as usize + 1).max().unwrap_or(0);
    let mut local = vec![u32::MAX; len];
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
    for &(a, b) in between {
        let (Some(i), Some(j)) = (at(a), at(b)) else { continue };
        let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
        parent[ri.max(rj) as usize] = ri.min(rj);
    }
    // An awake body against a sleeping one: if it's moving, the sleeper's
    // island wakes; if it's still, it waits to fall asleep in its own.
    let mut wake: Vec<u32> =
        against.iter().filter(|&&(e, _)| at(e).is_some_and(|k| awake[k as usize].1 < enough)).map(|&(_, i)| i).collect();
    wake.sort_unstable();
    wake.dedup();
    // An island sleeps when its stillest-for-least body has been still for
    // `enough` steps.
    let mut least = vec![u64::MAX; awake.len()];
    for k in 0..awake.len() as u32 {
        let r = root(&mut parent, k) as usize;
        least[r] = least[r].min(awake[k as usize].1);
    }
    let mut island = vec![u32::MAX; awake.len()];
    let mut fell = Vec::new();
    for k in 0..awake.len() {
        let r = root(&mut parent, k as u32) as usize;
        if least[r] < enough {
            continue;
        }
        if island[r] == u32::MAX {
            *last += 1;
            island[r] = *last;
        }
        fell.push((awake[k].0, island[r]));
    }
    (wake, fell)
}
