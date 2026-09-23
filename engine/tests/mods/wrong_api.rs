//! A mod that claims a different mod API version, as if built against a
//! newer engine.

use engine_api::{API_VERSION, ModContext, ModInfo, Op, Status};

#[unsafe(no_mangle)]
pub extern "C" fn engine_mod_info() -> ModInfo {
    ModInfo { api_version: API_VERSION + 1, state_version: 0, state_size: 0, state_align: 1 }
}

#[unsafe(no_mangle)]
pub extern "C" fn engine_mod_main(_ctx: *mut ModContext, _op: Op) -> Status {
    Status::OK
}
