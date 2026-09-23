//! Builds of the mod `bag`, which keeps a `Vec<String>` in the world and adds
//! a word to it every step. v1 and v2 share the component's layout; v3 adds a
//! field. Tests heap data crossing builds: allocated by one library's code,
//! grown and freed by another's.

use engine_api::{Cx, Mod, Status, export_mod};

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

#[derive(Default)]
struct BagMod {
    steps: u32,
}

impl Mod for BagMod {
    fn load(&mut self, cx: &mut Cx) {
        let mut world = cx.world();
        if world.query::<Bag>().next().is_none() {
            let e = world.spawn();
            world.insert(e, Bag::default());
        }
    }

    fn step(&mut self, cx: &mut Cx) -> Status {
        self.steps += 1;
        for (_, bag) in cx.world().query::<Bag>() {
            bag.words.push(format!("{BUILD}-{}", self.steps));
            #[cfg(feature = "v3")]
            {
                bag.note = format!("{} words", bag.words.len());
            }
        }
        Status::OK
    }
}

export_mod!(BagMod);
