//! A platform provider with no window, for the tests: each pump reports
//! one key going down, numbered by the pump, so a test sees which provider
//! answered and that it kept its state between calls. The shape of the
//! headless provider phase 2 builds, minus everything real.

use engine_api::{Cx, Mod, export_mod};
use platform::{DeviceInput, KEYBOARD, Pumped, WindowHandles};

engine_api::mod_state! {
    #[derive(Default)]
    struct FakePlatform {
        pumps: u32,
    }
}

impl Mod for FakePlatform {
    type Transient = ();
}

impl platform::Platform for FakePlatform {
    fn pump(&mut self, _: &mut (), _: &mut Cx) -> Pumped {
        self.pumps += 1;
        let key = DeviceInput { device: KEYBOARD, pad: 0, control: "KeyW".into(), value: 1.0 };
        Pumped { width: 0, height: 0, inputs: vec![key], micros: self.pumps, ..Pumped::default() }
    }

    fn window(&mut self, _: &mut (), _: &mut Cx) -> WindowHandles {
        WindowHandles::default()
    }
}

export_mod!(FakePlatform, provides = [platform::Platform]);
