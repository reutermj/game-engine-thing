//! Builds of `greeter`, which greets with a closure: the case the rule on mod
//! state exists for. v1 and v2 differ only in the word.
//!
//! By default it keeps the closure (a vtable in this library) and a string
//! literal (bytes in this library) in its state, which only a hand-written
//! `unsafe impl ModState` lets through: the wrong way, for the loader's net to
//! catch. With `right`, the state holds just the count and the closure is in
//! the transient part, which each build makes for itself. With `address`,
//! the state (from `mod_state!`) keeps an address in the first build as a
//! plain number, which the net must not take for a pointer.

use engine_api::{Cx, Mod, Systems, export_mod};

#[cfg(feature = "v1")]
const WORD: &str = "hello";
#[cfg(feature = "v2")]
const WORD: &str = "bonjour";

#[cfg(not(any(feature = "right", feature = "address")))]
mod greeter {
    use super::*;

    #[derive(Default)]
    pub struct Greeter {
        word: &'static str,
        greet: Option<Box<dyn Fn() -> String>>,
        greetings: u32,
    }

    // SAFETY: none; this is the mistake under test.
    unsafe impl engine_api::ModState for Greeter {}

    impl Mod for Greeter {
        type Transient = ();

        fn load(&mut self, _: &mut (), _cx: &mut Cx) {
            self.word = WORD;
            self.greet = Some(Box::new(|| format!("{WORD}!")));
        }

        fn systems(s: &mut Systems<Self>) {
            s.add("count", |greeter: &mut Self, _: &mut (), _: &mut Cx| greeter.greetings += 1);
        }

        fn message(&mut self, _: &mut (), _cx: &mut Cx, _message: &str) -> Result<String, String> {
            let greet = self.greet.as_ref().ok_or("no greeting")?;
            Ok(format!("{} ({} greetings, word {})", greet(), self.greetings, self.word))
        }
    }
}

#[cfg(feature = "right")]
mod greeter {
    use super::*;

    engine_api::mod_state! {
        #[derive(Default)]
        pub struct Greeter {
            greetings: u32,
        }
    }

    #[derive(Default)]
    pub struct Hooks {
        greet: Option<Box<dyn Fn() -> String>>,
    }

    impl Mod for Greeter {
        type Transient = Hooks;

        fn load(&mut self, hooks: &mut Hooks, _cx: &mut Cx) {
            hooks.greet = Some(Box::new(|| format!("{WORD}!")));
        }

        fn systems(s: &mut Systems<Self>) {
            s.add("count", |greeter: &mut Self, _: &mut Hooks, _: &mut Cx| greeter.greetings += 1);
        }

        fn message(&mut self, hooks: &mut Hooks, _cx: &mut Cx, _message: &str) -> Result<String, String> {
            let greet = hooks.greet.as_ref().ok_or("no greeting")?;
            Ok(format!("{} ({} greetings, word {WORD})", greet(), self.greetings))
        }
    }
}

#[cfg(feature = "address")]
mod greeter {
    use super::*;

    fn greet() -> String {
        format!("{WORD}!")
    }

    engine_api::mod_state! {
        #[derive(Default)]
        pub struct Greeter {
            greetings: u32,
            /// The first build's `greet`, as a number: data to the state rule,
            /// but what the loader's pointer scan would take for a pointer
            /// into that build.
            first: u64,
        }
    }

    impl Mod for Greeter {
        type Transient = ();

        fn load(&mut self, _: &mut (), _cx: &mut Cx) {
            if self.first == 0 {
                // The point: an address kept as a number.
                self.first = greet as fn() -> String as usize as u64;
            }
        }

        fn systems(s: &mut Systems<Self>) {
            s.add("count", |greeter: &mut Self, _: &mut (), _: &mut Cx| greeter.greetings += 1);
        }

        fn message(&mut self, _: &mut (), _cx: &mut Cx, _message: &str) -> Result<String, String> {
            Ok(format!("{} ({} greetings, word {WORD})", greet(), self.greetings))
        }
    }
}

export_mod!(greeter::Greeter);
