//! `base`'s interface as built by base_v1 and base_v1b.

engine_api::component! {
    #[derive(Debug, Default)]
    pub struct Shared: "test::Shared" {
        pub a: u32,
    }
}
