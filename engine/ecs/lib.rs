//! The ECS every mod and the loader share: components and their schemas,
//! archetype tables in pages, sparse sets, events, the parameters systems
//! declare themselves with, and the frame's dependency graph. See
//! docs/architecture/storage.md and docs/architecture/ecs.md.
//!
//! Shared as Rust types: the loader owns the `World`, and mods' code works
//! on it directly, which the one-compiler rule makes sound (ecs.md, "One
//! compiler per session"). The unsafe code is type erasure: `erased`
//! (columns) and `schema` (migrating values as bytes), with the drop and
//! default glue `component!` generates in `component`, and `world` calling
//! them to migrate. None of it depends on concurrency.

pub mod between;
pub mod component;
pub mod erased;
pub mod events;
pub mod graph;
pub mod harness;
pub mod ordered;
pub mod par;
pub mod query;
pub mod schema;
pub mod spatial;
pub mod world;

pub use between::WorldMut;
#[doc(hidden)]
pub use component::{__component_fingerprint, __drop, __drop_fn, __fingerprint, __fingerprint_struct, __fnv, __write_default};
pub use component::{Component, ComponentDesc, Crossing, DefaultFn, DropFn, Entity, FieldDesc, FieldKind, FieldType, Storage};
pub use events::{Event, EventReader, EventWriter};
pub use ordered::{ChildOf, OrderKey, children_of, entity_key, pair_key, pairs_from};
pub use par::{Executor, Scoped, Workers};
pub use query::{
    Adds, Bundle, Change, ChangeDecl, Changes, ColumnMut, Data, Declare, Despawns, Dt, Filter, FilterDecl, FrameCx, Log, Mut, NearSide,
    Page, Param, ParamDecl, Query, QueryDecl, Removes, Row, Spawner, With, Without, near_pairs, near_pairs_with,
};
pub use spatial::{Bounds, SpatialKey};
pub use world::{Build, ComponentId, Keepalive, Structural, TableId, World};
