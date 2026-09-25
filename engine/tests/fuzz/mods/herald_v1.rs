//! `herald`'s interface as herald_a, _b and _p build it: the event it sends
//! and the service it provides.

engine_api::event! {
    #[derive(Debug, Default)]
    pub struct Note: "fz::Note" {
        pub n: u64,
        pub from: String,
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
