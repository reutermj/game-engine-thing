//! Builds of `ranker`, which keeps `fz::Rank`s, an ordered key, and each
//! frame records the order its `report` system walks them in. a keys them
//! ascending and b descending, with the same layout, so a reload between
//! them changes only the key glue; c widens the key and adds a field, keying
//! ascending. Ranks are unique (`add` and `set` refuse a taken one), so the
//! order never falls to the entity tie-break. Mirrored by the reload
//! fuzzer's model (engine/tests/fuzz/reload_ops.rs).

use engine_api::{Cx, Entity, Mod, OrderKey, Query, Systems, export_mod, phase};

#[cfg(feature = "a")]
const BUILD: &str = "a";
#[cfg(feature = "b")]
const BUILD: &str = "b";
#[cfg(feature = "c")]
const BUILD: &str = "c";

#[cfg(not(feature = "c"))]
engine_api::component! {
    #[derive(Default)]
    pub struct Rank: "fz::Rank", order = key {
        pub n: u32,
    }
}

#[cfg(feature = "c")]
engine_api::component! {
    pub struct Rank: "fz::Rank", order = key {
        pub bonus: u8,
        pub n: u64,
    }
}

#[cfg(feature = "c")]
impl Default for Rank {
    fn default() -> Rank {
        Rank { bonus: 9, n: 0 }
    }
}

impl Rank {
    fn of(n: u64) -> Rank {
        Rank { n: n as _, ..Rank::default() }
    }
}

impl OrderKey for Rank {
    #[inline]
    fn key(&self) -> u128 {
        if cfg!(feature = "b") { (u32::MAX as u64 - self.n as u64) as u128 } else { self.n as u128 }
    }
}

engine_api::mod_state! {
    #[derive(Default)]
    struct Ranker {
        seen: Vec<u64>,
        loads: u32,
    }
}

impl Ranker {
    fn report(&mut self, _: &mut (), _: &mut Cx, mut ranks: Query<&Rank>) {
        self.seen.clear();
        ranks.for_each_ordered(|_, rank| self.seen.push(rank.n as u64));
    }

    fn find(cx: &mut Cx, n: u64) -> Vec<Entity> {
        let mut found = Vec::new();
        cx.world().for_each::<&Rank>(|e, rank| {
            if rank.n as u64 == n {
                found.push(e);
            }
        });
        found
    }
}

impl Mod for Ranker {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("report", Self::report).phase(phase::LATE);
    }

    fn load(&mut self, _: &mut (), _: &mut Cx) {
        self.loads += 1;
    }

    /// `add <n>`, `set <from> <to>`, `del <n>`, `dump`, `boom`. Ranks are
    /// kept below `u32::MAX`, so every build can hold every rank.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let words: Vec<&str> = message.split(' ').collect();
        let num = |i: usize| -> Result<u64, String> {
            words.get(i).and_then(|w| w.parse().ok()).ok_or_else(|| format!("ranker: bad {message:?}"))
        };
        match words[0] {
            "add" => {
                let n = num(1)?;
                if !Self::find(cx, n).is_empty() {
                    return Err(format!("ranker:{BUILD} has {n}"));
                }
                cx.world().spawn((Rank::of(n),));
                Ok(format!("ranker:{BUILD} added {n}"))
            }
            "set" => {
                let (from, to) = (num(1)?, num(2)?);
                if Self::find(cx, from).is_empty() || !Self::find(cx, to).is_empty() {
                    return Err(format!("ranker:{BUILD} can't set {from} to {to}"));
                }
                // A write in place: the table re-sorts as the walk ends.
                cx.world().for_each::<&mut Rank>(|_, mut rank| {
                    if rank.n as u64 == from {
                        rank.n = to as _;
                    }
                });
                Ok(format!("ranker:{BUILD} set {from} to {to}"))
            }
            "del" => {
                let found = Self::find(cx, num(1)?);
                let mut world = cx.world();
                for &e in &found {
                    world.despawn(e);
                }
                Ok(format!("ranker:{BUILD} deleted {}", found.len()))
            }
            "dump" => Ok(format!("ranker:{BUILD} loads={} seen={:?}", self.loads, self.seen)),
            "boom" => fz_planted::panic(format!("ranker:{BUILD} panics on boom")),
            _ => Err(format!("ranker doesn't understand {message:?}")),
        }
    }
}

export_mod!(Ranker);
