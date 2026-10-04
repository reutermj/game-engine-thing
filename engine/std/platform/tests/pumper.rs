//! Calls the platform as a bootstrap would, on a message: `pump` replies
//! what the pump found, or why the call didn't run.

use engine_api::{Cx, Mod, export_mod};

engine_api::mod_state! {
    #[derive(Default)]
    struct Pumper {}
}

impl Mod for Pumper {
    type Transient = ();

    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "pump" => match platform::pump(cx) {
                Ok(p) => {
                    let inputs: Vec<String> = p.inputs.iter().map(|i| format!("{}:{}={}", i.device, i.control, i.value)).collect();
                    Ok(format!(
                        "pump {} inputs [{}] window {}",
                        p.micros,
                        inputs.join(" "),
                        platform::window(cx).map_err(|e| e.to_string())?.system
                    ))
                }
                Err(e) => Ok(format!("{:?}", e.kind)),
            },
            _ => Err("usage: pump".into()),
        }
    }
}

export_mod!(Pumper);
