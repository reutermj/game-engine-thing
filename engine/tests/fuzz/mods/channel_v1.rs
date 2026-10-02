//! `channel`'s interface as channel_v1 builds it: the flow `source` makes
//! and `peek` and `relay` use. Declared apart from its maker, so a reader
//! can be loaded without one and the plan check, not a dependency, is what
//! refuses it.

engine_api::flow! {
    pub struct Stream: "fz::Stream" {
        pub values: Vec<u64>,
    }
}
