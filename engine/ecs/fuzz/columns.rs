//! libFuzzer target: `ErasedColumn`'s operations, against vectors
//! (`ecs_ops::columns`), with the input as the driver's choices. A failed
//! check panics, which aborts, which libFuzzer reports with the input that
//! did it.

#![no_main]

use std::sync::Once;

/// libFuzzer's entry point, called once per input.
///
/// # Safety
/// libFuzzer passes `size` readable bytes at `data`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn LLVMFuzzerTestOneInput(data: *const u8, size: usize) -> i32 {
    static QUIET: Once = Once::new();
    QUIET.call_once(ecs_ops::quiet_expected_panics);
    // SAFETY: as libFuzzer promises; a null pointer comes only with 0.
    let data = if size == 0 { &[][..] } else { unsafe { std::slice::from_raw_parts(data, size) } };
    ecs_ops::columns(&mut ecs_ops::Bytes::new(data), usize::MAX);
    0
}
