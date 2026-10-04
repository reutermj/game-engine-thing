//! The ECS every mod and the loader share: components and their schemas,
//! archetype tables in pages, sparse sets, events, flows, the parameters
//! systems declare themselves with (parallel shapes among them), and the
//! frame's dependency graph. See
//! docs/architecture/storage.md and docs/architecture/ecs.md.
//!
//! Shared as Rust types: the loader owns the `World`, and mods' code works
//! on it directly, which the one-compiler rule makes sound (ecs.md, "One
//! compiler per session"). The unsafe code is type erasure: `erased`
//! (columns) and `schema` (migrating values as bytes), with the drop and
//! default glue `component!` generates in `component`, and `world` calling
//! them to migrate. None of it depends on concurrency.
//!
//! Mods don't depend on this crate: they see what `engine_api` re-exports,
//! and `engine_mod` refuses it in a mod's deps. So `pub` here means public
//! to the loader, the scheduler's thread host and tests; what a mod may
//! reach is `engine_api`'s list. That is how the frame machinery
//! (`FrameCx`, `frame_cx`, `Param::fetch`, `harness`) and the raw
//! broadphase (`near_pairs`) stay out of mods' hands while the loader and
//! tests share them: a system gets a shape, or anything else, only by
//! declaring it (get-znt.51, get-znt.48).

pub mod between;
pub mod component;
mod dispatch;
pub mod erased;
pub mod events;
pub mod flows;
pub mod graph;
pub mod harness;
pub mod live;
pub mod ordered;
mod par;
pub mod query;
pub mod schema;
pub mod shape;
pub mod spatial;
pub mod world;

pub use between::WorldMut;
#[doc(hidden)]
pub use component::{__component_fingerprint, __drop, __drop_fn, __fingerprint, __fingerprint_struct, __fnv, __write_default};
pub use component::{Component, ComponentDesc, Crossing, DefaultFn, DropFn, Entity, FieldDesc, FieldKind, FieldType, Storage};
pub use events::{Event, EventReader, EventWriter};
pub use flows::{Flow, FlowAccess, Make, Pass, Recycle, See, Take};
pub use live::{AnyOf, Live, Proximity, Tables};
pub use ordered::{ChildOf, OrderKey, children_of, entity_key, pair_key, pairs_from};
pub use par::{Executor, Scoped};
pub use query::{
    Adds, Bundle, Change, ChangeDecl, Changes, ColumnMut, Compose, Data, Declare, Despawns, Dt, Fetched, Filter, FilterDecl, FrameCx, Log,
    Mut, NearSide, Page, Param, ParamDecl, Query, QueryDecl, Removes, Row, Spawner, With, Without, frame_cx, near_pairs,
};
pub use shape::{Colored, Coloring, ParMap, Passes, Reduce, ShapeKind, Shareable, Stage, States};
pub use spatial::{Bounds, Extents, SpatialKey};
pub use world::{Build, ComponentId, Keepalive, Structural, TableId, World};
