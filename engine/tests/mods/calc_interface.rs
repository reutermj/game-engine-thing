//! `calc`'s interface: a service for testing calls between mods.

engine_api::service! {
    pub trait Calc {
        fn apply(x: i64) -> i64;
        /// The build and how many calls it has served.
        fn describe() -> String;
        /// Takes a borrow, which may be anything.
        fn greet(name: &str) -> String;
        fn boom();
        /// Calls `apply` from inside `calc`: a call back into a running mod.
        fn recurse() -> String;
    }
}
