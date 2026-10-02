//! `channel`'s interface as channel_v2 builds it: `fz::Stream` with a field
//! before the values, so a bin of v1's layout read as v2's would be wrong.

engine_api::flow! {
    pub struct Stream: "fz::Stream" {
        pub tag: String,
        pub values: Vec<u64>,
    }
}
