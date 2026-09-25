//! Two builds of the mod `mover`, each with its own definition of the
//! component `test::Pos`, to test migration between them.

use engine_api::{Cx, Mod, component, export_mod};

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

engine_api::mod_state! {
    #[derive(Default)]
    struct Mover {
        spawned: bool,
    }
}

impl Mod for Mover {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        let mut world = cx.world();
        if !self.spawned {
            #[cfg(feature = "v1")]
            world.spawn((Pos { x: 1.5, y: 2.5 },));
            #[cfg(feature = "v2")]
            world.spawn((Pos { y: 2.5, x: 1.5, z: 0.0 },));
            self.spawned = true;
        }
        // A mod without systems declares nothing, so naming the component is
        // what installs this build's layout.
        world.id::<Pos>();
    }
}

export_mod!(Mover);
