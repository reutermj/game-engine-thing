//! libFuzzer target: sessions of loads, reloads, unloads, frames and
//! messages on a real engine, against the model (`reload_ops::run`), with
//! the input as the driver's choices. A failed check panics, which aborts,
//! which libFuzzer reports with the input that did it.

#![no_main]

/// Operations per input at most: a session costs a few milliseconds a load,
/// and libFuzzer's own limit on input length would allow thousands.
const STEPS: usize = 200;

/// libFuzzer's entry point, called once per input.
///
/// # Safety
/// libFuzzer passes `size` readable bytes at `data`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn LLVMFuzzerTestOneInput(data: *const u8, size: usize) -> i32 {
    // SAFETY: as libFuzzer promises; a null pointer comes only with 0.
    let data = if size == 0 { &[][..] } else { unsafe { std::slice::from_raw_parts(data, size) } };
    reload_ops::run(&mut reload_ops::Bytes::new(data), STEPS);
    0
}
