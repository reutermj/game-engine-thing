//! A mod that depends on `base`'s interface. Built once against each
//! interface of `base`, as user_v1 and user_v2.

use engine_api::{Cx, Mod, Query, Systems, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct User {}
}

impl User {
    fn touch(&mut self, _: &mut (), _: &mut Cx, mut shared: Query<&base::Shared>) {
        shared.for_each(|_, _| {});
    }
}

impl Mod for User {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("touch", Self::touch);
    }
}

export_mod!(User);
