//! `streamer`'s interface as streamer_v1 builds it: the flow it makes.

engine_api::flow! {
    pub struct Samples: "test::Samples" {
        pub values: Vec<u64>,
    }
}
