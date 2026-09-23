//! Builds of `greeter`, which greets with a closure: the case the rule on mod
//! state exists for. v1 and v2 differ only in the word.
//!
//! By default it keeps the closure (a vtable in this library) and a string
//! literal (bytes in this library) in its state, which only a hand-written
//! `unsafe impl ModState` lets through: the wrong way, for the loader's net to
//! catch. With `right`, the state holds just the count and the closure is in
//! the transient part, which each build makes for itself.

use engine_api::{Cx, Mod, Status, export_mod};

#[cfg(feature = "v1")]
const WORD: &str = "hello";
#[cfg(feature = "v2")]
const WORD: &str = "bonjour";

#[cfg(not(feature = "right"))]
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

        fn step(&mut self, _: &mut (), _cx: &mut Cx) -> Status {
            self.greetings += 1;
            Status::OK
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

        fn step(&mut self, _: &mut Hooks, _cx: &mut Cx) -> Status {
            self.greetings += 1;
            Status::OK
        }

        fn message(&mut self, hooks: &mut Hooks, _cx: &mut Cx, _message: &str) -> Result<String, String> {
            let greet = hooks.greet.as_ref().ok_or("no greeting")?;
            Ok(format!("{} ({} greetings, word {WORD})", greet(), self.greetings))
        }
    }
}

export_mod!(greeter::Greeter);
