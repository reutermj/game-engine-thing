//! The loader as a library: everything the engine binary does except reading
//! the manifest and running the loop. Split out so tests can drive an
//! `Engine` directly, stepping exact frame counts with no process or socket.

pub mod control_server;
pub mod engine;
mod schedule;
