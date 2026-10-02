use engine_ecs::{Entity, Recycle};

/// Entities to positions in a list, by entity index: ids are small dense
/// integers, so a vector by index finds one in O(1), where sorting the list
/// and binary-searching it was most of gathering. The generation is kept,
/// so a stale id (despawned since, its index reused) finds nothing.
#[derive(Default)]
pub struct Slots(Vec<(u32, u32)>);

impl Slots {
    pub fn of(entities: impl Iterator<Item = Entity> + Clone) -> Slots {
        let mut slots = Slots::default();
        slots.fill(entities);
        slots
    }

    /// Refilled in place, keeping its allocation: a flow's.
    pub fn fill(&mut self, entities: impl IntoIterator<Item = Entity, IntoIter: Clone>) {
        let entities = entities.into_iter();
        let len = entities.clone().map(|e| e.index as usize + 1).max().unwrap_or(0);
        self.0.clear();
        self.0.resize(len, (u32::MAX, u32::MAX));
        for (k, e) in entities.enumerate() {
            self.0[e.index as usize] = (e.generation, k as u32);
        }
    }

    pub fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }

    pub fn insert(&mut self, e: Entity, k: u32) {
        let i = e.index as usize;
        if self.0.len() <= i {
            self.0.resize(i + 1, (u32::MAX, u32::MAX));
        }
        self.0[i] = (e.generation, k);
    }
}

impl Recycle for Slots {
    fn recycle(&mut self) {
        self.0.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn e(index: u32, generation: u32) -> Entity {
        Entity { index, generation }
    }

    #[test]
    fn finds_by_index_and_generation() {
        let slots = Slots::of([e(3, 0), e(0, 1), e(7, 2)].into_iter());
        assert_eq!(slots.get(e(3, 0)), Some(0));
        assert_eq!(slots.get(e(0, 1)), Some(1));
        assert_eq!(slots.get(e(7, 2)), Some(2));
        // A stale id: its index reused since.
        assert_eq!(slots.get(e(7, 1)), None);
        // Inside the vector but never filled, and past its end.
        assert_eq!(slots.get(e(5, 0)), None);
        assert_eq!(slots.get(e(8, 0)), None);
    }

    #[test]
    fn inserting_grows_and_refilling_forgets() {
        let mut slots = Slots::of([e(1, 0)].into_iter());
        slots.insert(e(4, 3), 9);
        assert_eq!(slots.get(e(4, 3)), Some(9));
        slots.fill([e(2, 0)]);
        assert_eq!(slots.get(e(1, 0)), None);
        assert_eq!(slots.get(e(4, 3)), None);
        assert_eq!(slots.get(e(2, 0)), Some(0));
        slots.recycle();
        assert_eq!(slots.get(e(2, 0)), None);
    }
}
