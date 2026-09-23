//! Calls `calc::Calc` and reports what happened, through messages, so tests
//! can drive calls between mods: `apply <n>`, `describe`, `greet <name>`,
//! `boom`, `recurse`, and `at_load` (the result of a call made in `load`).

use engine_api::{Cx, Mod, Status, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Caller {
        at_load: i64,
    }
}

impl Mod for Caller {
    type Transient = ();

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        self.at_load = calc::apply(cx, 100).unwrap_or(-1);
    }

    fn step(&mut self, _: &mut (), _cx: &mut Cx) -> Status {
        Status::OK
    }

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        let (command, arg) = message.split_once(' ').unwrap_or((message, ""));
        let result = match command {
            "apply" => calc::apply(cx, arg.parse().map_err(|e| format!("{e}"))?).map(|v| v.to_string()),
            "describe" => calc::describe(cx),
            "greet" => calc::greet(cx, arg),
            "boom" => calc::boom(cx).map(|()| "no panic".into()),
            "recurse" => calc::recurse(cx),
            "at_load" => Ok(self.at_load.to_string()),
            _ => return Err(format!("unknown command {command:?}")),
        };
        result.map_err(|e| e.to_string())
    }
}

export_mod!(Caller);
