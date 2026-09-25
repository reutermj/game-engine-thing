//! Builds of `keeper`, which keeps items in the world: `fz::Item`, a table
//! component with heap fields, and `fz::Flag`, a sparse one, both changed
//! from build to build, as is its own state, which holds heap data too. What
//! each build does, and what its layouts are, is mirrored by the reload
//! fuzzer's model (engine/tests/fuzz/reload_ops.rs); change the two together.
//!
//! | build | `Item` | `Flag` | state | step | also |
//! |---|---|---|---|---|---|
//! | a | A | A, sparse | A | +1 | |
//! | b | A | A, sparse | A | +2 | |
//! | c | C (reordered, widened, a field added and one removed) | C, sparse | C (migrates) | +3 | |
//! | d | C at version 1 (resets) | C, sparse | C at version 1 (resets) | +5 | |
//! | e | A | A, sparse | A | +1 | panics in `load` |
//! | f | A | A, sparse | A | +1 | panics in `unload` and `close` |
//! | s | A | A, table storage | A | +1 | refused once `Flag` is known as sparse |

use engine_api::{Cx, Entity, Mod, Query, Systems, export_mod};

#[cfg(feature = "a")]
const BUILD: (&str, u64) = ("a", 1);
#[cfg(feature = "b")]
const BUILD: (&str, u64) = ("b", 2);
#[cfg(feature = "c")]
const BUILD: (&str, u64) = ("c", 3);
#[cfg(feature = "d")]
const BUILD: (&str, u64) = ("d", 5);
#[cfg(feature = "e")]
const BUILD: (&str, u64) = ("e", 1);
#[cfg(feature = "f")]
const BUILD: (&str, u64) = ("f", 1);
#[cfg(feature = "s")]
const BUILD: (&str, u64) = ("s", 1);

/// Layout A of everything: every build but c and d.
#[cfg(not(any(feature = "c", feature = "d")))]
mod layout {
    engine_api::component! {
        pub struct Item: "fz::Item" {
            pub id: u32,
            pub count: u32,
            pub name: String,
            pub tags: Vec<String>,
        }
    }

    /// Owns heap memory, so a migration that forgets to drop the default's
    /// field before overwriting it leaks, and one that shares it between
    /// values frees it twice.
    impl Default for Item {
        fn default() -> Item {
            Item { id: 0, count: 0, name: "anon".into(), tags: vec!["new".into()] }
        }
    }

    #[cfg(not(feature = "s"))]
    engine_api::component! {
        #[derive(Default)]
        pub struct Flag: "fz::Flag", storage = sparse {
            pub level: u8,
            pub note: String,
        }
    }

    // A component's storage is fixed for the session, so this build is
    // refused once `Flag` is known as sparse (and, loaded first, has every
    // other build refused).
    #[cfg(feature = "s")]
    engine_api::component! {
        #[derive(Default)]
        pub struct Flag: "fz::Flag" {
            pub level: u8,
            pub note: String,
        }
    }

    impl Flag {
        pub fn new(level: u8, note: String) -> Flag {
            Flag { level, note }
        }
    }

    engine_api::mod_state! {
        #[derive(Default)]
        pub struct Keeper {
            pub loads: u32,
            pub spawned: u32,
            pub journal: Vec<String>,
        }
    }

    pub type Count = u32;
    pub type Level = u8;
}

/// Layout C: `Item` reordered, `count` widened, `weight` added and `tags`
/// removed; `Flag`'s level widened to signed and `seen` added; the state's
/// `spawned` widened and `extra` added. d is the same at version 1.
#[cfg(any(feature = "c", feature = "d"))]
mod layout {
    #[cfg(feature = "c")]
    engine_api::component! {
        pub struct Item: "fz::Item" {
            pub weight: f64,
            pub name: String,
            pub count: u64,
            pub id: u32,
        }
    }

    #[cfg(feature = "d")]
    engine_api::component! {
        pub struct Item: "fz::Item", version = 1 {
            pub weight: f64,
            pub name: String,
            pub count: u64,
            pub id: u32,
        }
    }

    impl Default for Item {
        fn default() -> Item {
            let (weight, name) = if cfg!(feature = "c") { (1.5, "anon-c") } else { (2.5, "anon-d") };
            Item { weight, name: name.into(), count: 0, id: 0 }
        }
    }

    engine_api::component! {
        pub struct Flag: "fz::Flag", storage = sparse {
            pub note: String,
            pub level: i32,
            pub seen: bool,
        }
    }

    impl Default for Flag {
        fn default() -> Flag {
            Flag { note: String::new(), level: 0, seen: true }
        }
    }

    impl Flag {
        pub fn new(level: i32, note: String) -> Flag {
            Flag { level, note, seen: false }
        }
    }

    #[cfg(feature = "c")]
    engine_api::mod_state! {
        pub struct Keeper {
            pub journal: Vec<String>,
            pub spawned: u64,
            pub loads: u32,
            pub extra: u64,
        }
    }

    #[cfg(feature = "d")]
    engine_api::mod_state! {
        pub struct Keeper, version = 1 {
            pub journal: Vec<String>,
            pub spawned: u64,
            pub loads: u32,
            pub extra: u64,
        }
    }

    impl Default for Keeper {
        fn default() -> Keeper {
            Keeper { journal: Vec::new(), spawned: 0, loads: 0, extra: 7 }
        }
    }

    pub type Count = u64;
    pub type Level = i32;
}

use layout::{Count, Flag, Item, Keeper, Level};

fn planted_panic(what: &str) -> ! {
    fz_planted::panic(format!("keeper:{} panics {what}", BUILD.0))
}

impl Keeper {
    fn tick(&mut self, _: &mut (), _: &mut Cx, mut items: Query<&mut Item>) {
        items.for_each(|_, mut item| item.count = item.count.wrapping_add(BUILD.1 as Count));
    }

    /// Names `Flag`, so it is installed with this build when the load
    /// commits rather than on first use.
    fn flags(&mut self, _: &mut (), _: &mut Cx, mut flags: Query<&Flag>) {
        flags.for_each(|_, _| {});
    }

    /// The entities whose item has `id`.
    fn with_id(cx: &mut Cx, id: u32) -> Vec<Entity> {
        let mut found = Vec::new();
        cx.world().for_each::<&Item>(|e, item| {
            if item.id == id {
                found.push(e)
            }
        });
        found
    }

    fn describe(item: &Item, flag: Option<Flag>) -> String {
        #[cfg(not(any(feature = "c", feature = "d")))]
        let (item, flag) = (
            format!("{}/{}/{}/{}", item.id, item.count, item.name, item.tags.join(",")),
            flag.map_or("-".into(), |f| format!("{}:{}", f.level, f.note)),
        );
        #[cfg(any(feature = "c", feature = "d"))]
        let (item, flag) = (
            format!("{}/{}/{}/w{}", item.id, item.count, item.name, item.weight),
            flag.map_or("-".into(), |f| format!("{}:{}:{}", f.level, f.note, f.seen)),
        );
        format!("{item}/{flag}")
    }

    fn dump(&self, cx: &mut Cx) -> String {
        let mut items = Vec::new();
        cx.world().for_each::<&Item>(|e, item| items.push((e, item.clone())));
        // Flags after the walk: `get` locks the table the walk holds.
        let world = cx.world();
        let mut items: Vec<String> = items.into_iter().map(|(e, item)| Self::describe(&item, world.get::<Flag>(e))).collect();
        items.sort();
        #[cfg(not(any(feature = "c", feature = "d")))]
        let extra = "-".to_string();
        #[cfg(any(feature = "c", feature = "d"))]
        let extra = self.extra.to_string();
        format!(
            "keeper:{} loads={} spawned={} extra={extra} journal=[{}] items=[{}]",
            BUILD.0,
            self.loads,
            self.spawned,
            self.journal.join(","),
            items.join(" ")
        )
    }

    /// Every item with `id`, changed by `f`: how many.
    fn each_with(cx: &mut Cx, id: u32, mut f: impl FnMut(&mut Item)) -> usize {
        let mut n = 0;
        cx.world().for_each::<&mut Item>(|_, mut item| {
            if item.id == id {
                f(&mut item);
                n += 1;
            }
        });
        n
    }
}

impl Mod for Keeper {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("tick", Self::tick);
        s.add("flags", Self::flags);
    }

    fn load(&mut self, _: &mut (), _: &mut Cx) {
        if cfg!(feature = "e") {
            planted_panic("in load");
        }
        self.loads += 1;
    }

    fn unload(&mut self, _: &mut (), _: &mut Cx) {
        if cfg!(feature = "f") {
            planted_panic("in unload");
        }
    }

    fn close(&mut self, _: &mut (), _: &mut Cx) {
        if cfg!(feature = "f") {
            planted_panic("in close");
        }
    }

    /// `spawn <id> <name>`, `despawn <id>`, `rename <id> <name>`, `bulk <id>
    /// <n>` (adds to the count), `tag <id> <word>`, `flag <id> <level>`,
    /// `unflag <id>`, each on every item with that id; `dump`; `boom`.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let words: Vec<&str> = message.split(' ').collect();
        let num = |i: usize| -> Result<u64, String> {
            words.get(i).and_then(|w| w.parse().ok()).ok_or_else(|| format!("keeper: bad {message:?}"))
        };
        let word = |i: usize| words.get(i).map(|w| w.to_string()).unwrap_or_default();
        let build = BUILD.0;
        match words[0] {
            "dump" => Ok(self.dump(cx)),
            "spawn" => {
                let id = num(1)? as u32;
                cx.world().spawn((Item { id, name: word(2), ..Item::default() },));
                self.spawned += 1;
                self.journal.push(format!("{build}+{id}"));
                Ok(format!("keeper:{build} spawned {id}"))
            }
            "despawn" => {
                let found = Self::with_id(cx, num(1)? as u32);
                let mut world = cx.world();
                for &e in &found {
                    world.despawn(e);
                }
                Ok(format!("keeper:{build} despawned {}", found.len()))
            }
            "rename" => {
                let name = word(2);
                let n = Self::each_with(cx, num(1)? as u32, |item| item.name = name.clone());
                Ok(format!("keeper:{build} renamed {n}"))
            }
            "bulk" => {
                let by = num(2)?;
                let n = Self::each_with(cx, num(1)? as u32, |item| item.count = item.count.wrapping_add(by as Count));
                Ok(format!("keeper:{build} bulked {n}"))
            }
            #[cfg(not(any(feature = "c", feature = "d")))]
            "tag" => {
                let tag = word(2);
                let n = Self::each_with(cx, num(1)? as u32, |item| item.tags.push(tag.clone()));
                Ok(format!("keeper:{build} tagged {n}"))
            }
            #[cfg(any(feature = "c", feature = "d"))]
            "tag" => Err(format!("keeper:{build} has no tags")),
            "flag" => {
                let found = Self::with_id(cx, num(1)? as u32);
                let level = num(2)? as Level;
                let mut world = cx.world();
                for &e in &found {
                    world.insert(e, Flag::new(level, format!("{build}{level}")));
                }
                Ok(format!("keeper:{build} flagged {}", found.len()))
            }
            "unflag" => {
                let found = Self::with_id(cx, num(1)? as u32);
                let mut world = cx.world();
                for &e in &found {
                    world.remove::<Flag>(e);
                }
                Ok(format!("keeper:{build} unflagged {}", found.len()))
            }
            "boom" => planted_panic("on boom"),
            _ => Err(format!("keeper doesn't understand {message:?}")),
        }
    }
}

export_mod!(Keeper);
