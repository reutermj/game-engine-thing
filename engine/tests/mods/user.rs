//! A mod that depends on `base`'s interface. Built once against each
//! interface of `base`, as user_v1 and user_v2.

use engine_api::{Cx, Mod, Status, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct User {}
}

impl Mod for User {
    type Transient = ();

    fn step(&mut self, _: &mut (), cx: &mut Cx) -> Status {
        cx.world().query::<base::Shared>().count();
        Status::OK
    }
}

export_mod!(User);
