//! A spike of how entities refer to each other (get-emj.18): contacts as a
//! physics-owned table (`table`) or as entities (`entities`), and
//! structural links (a collider to its body, a child to its parent) as
//! fields with back-references, as compound data, or split across tables as
//! archetype pairs would be (`links`). The cases a game writes are in
//! `cases`; the numbers come from `bench`.

#[path = "../../engine/std/physics/narrow.rs"]
pub mod narrow;
#[path = "../../engine/std/physics/solver.rs"]
pub mod solver;

pub mod common;
pub mod entities;
pub mod table;

pub mod cases;
pub mod links;
