//! `streamer`'s interface as streamer_v2 builds it: the flow it makes, with
//! a field before the values, so v1's layout and v2's differ.

engine_api::flow! {
    pub struct Samples: "test::Samples" {
        pub label: String,
        pub values: Vec<u64>,
    }
}
