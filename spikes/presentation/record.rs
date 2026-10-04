//! SPIKE (get-3hd.1): `spike_record`, which declares the recorder's data
//! (`record_interface.rs`) and runs nothing. Resident, as `spike_turns` is,
//! so the resident bootstrap may depend on it.

engine_api::export_mod!(engine_api::Inert);
