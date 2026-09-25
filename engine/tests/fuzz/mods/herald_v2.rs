//! `herald`'s interface as herald_c builds it: the event has gained a field,
//! so its layout changed and so did the interface's digest. A mod built
//! against v1 can't run beside it.

engine_api::event! {
    #[derive(Debug, Default)]
    pub struct Note: "fz::Note" {
        pub n: u64,
        pub from: String,
        pub weight: u32,
    }
}

engine_api::service! {
    pub trait Herald {
        /// Adds `by` to the calls herald's state counts: the new count.
        fn bump(by: u64) -> u64;
        /// Which build answers.
        fn build() -> String;
    }
}
