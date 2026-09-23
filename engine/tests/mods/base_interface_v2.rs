//! `base`'s interface as built by base_v2: `Shared` gained a field.

engine_api::component! {
    #[derive(Debug, Default, Copy)]
    pub struct Shared: "test::Shared" {
        pub a: u32,
        pub b: u32,
    }
}
