//! Builds of the mod `bag`, which keeps a `Vec<String>` in the world and adds
//! a word to it every step. v1 and v2 share the component's layout; v3 adds a
//! field. Tests heap data crossing builds: allocated by one library's code,
//! grown and freed by another's.

use engine_api::{Cx, Mod, Query, Systems, export_mod};

#[cfg(not(feature = "v3"))]
engine_api::component! {
    #[derive(Default)]
    pub struct Bag: "test::Bag" {
        pub words: Vec<String>,
    }
}

#[cfg(feature = "v3")]
engine_api::component! {
    #[derive(Default)]
    pub struct Bag: "test::Bag" {
        pub note: String,
        pub words: Vec<String>,
    }
}

#[cfg(feature = "v1")]
const BUILD: &str = "v1";
#[cfg(feature = "v2")]
const BUILD: &str = "v2";
#[cfg(feature = "v3")]
const BUILD: &str = "v3";

engine_api::mod_state! {
    #[derive(Default)]
    struct BagMod {
        steps: u32,
    }
}

impl BagMod {
    fn fill(&mut self, _: &mut (), cx: &mut Cx, bags: Query<&mut Bag>) {
        self.steps += 1;
        for (_, bag) in bags.iter(cx) {
            bag.words.push(format!("{BUILD}-{}", self.steps));
            #[cfg(feature = "v3")]
            {
                bag.note = format!("{} words", bag.words.len());
            }
        }
    }
}

impl Mod for BagMod {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("fill", Self::fill);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        if world.query::<Bag>().next().is_none() {
            let e = world.spawn();
            world.insert(e, Bag::default());
        }
    }

}

export_mod!(BagMod);
