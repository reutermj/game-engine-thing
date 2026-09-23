//! Two builds of the mod `mover`, each with its own definition of the
//! component `test::Pos`, to test migration between them.

use engine_api::{Cx, Mod, Status, component, export_mod};

#[cfg(feature = "v1")]
component! {
    #[derive(Default, Copy)]
    pub struct Pos: "test::Pos" {
        pub x: f32,
        pub y: f32,
    }
}

// Reordered, `y` widened, `z` added with a non-zero default.
#[cfg(feature = "v2")]
component! {
    #[derive(Copy)]
    pub struct Pos: "test::Pos" {
        pub y: f64,
        pub x: f32,
        pub z: f32,
    }
}

#[cfg(feature = "v2")]
impl Default for Pos {
    fn default() -> Self {
        Pos { y: 0.0, x: 0.0, z: 7.0 }
    }
}

#[derive(Default)]
struct Mover {
    spawned: bool,
}

impl Mod for Mover {
    fn load(&mut self, cx: &mut Cx) {
        let mut world = cx.world();
        if !self.spawned {
            let e = world.spawn();
            #[cfg(feature = "v1")]
            world.insert(e, Pos { x: 1.5, y: 2.5 });
            #[cfg(feature = "v2")]
            world.insert(e, Pos { y: 2.5, x: 1.5, z: 0.0 });
            self.spawned = true;
        }
        // Touching the component is what registers this build's layout.
        world.query::<Pos>().count();
    }

    fn step(&mut self, _cx: &mut Cx) -> Status {
        Status::OK
    }
}

export_mod!(Mover);
