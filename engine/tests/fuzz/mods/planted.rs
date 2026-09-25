//! What the fuzzer's test mods share: the panics planted in them.

/// A silent panic: the fuzzer plants millions, and the default hook would
/// print each. It is this build's own hook (every mod links its own std),
/// set as a zero-sized closure, so setting it allocates nothing the image
/// could leak when it's unmapped.
pub fn panic(what: String) -> ! {
    std::panic::set_hook(Box::new(|_| {}));
    panic!("{what}");
}
